# -*- coding: utf-8 -*-
"""NodePalletServo 单元测试。

本节点（含 `TfCamBaseResolver`）不访问硬件：托盘位姿与箱子观测都在黑板上、
TF 查询器是注入的、伺服误差是纯计算。所以这个测试也不需要 ROS、不需要相机。

黑板权限是刻意分开的：节点只注册 `latest_pallet` / `latest_box_obs` 的
**READ**，写这两个键的是另一个 client（`make_writer`），模拟真实运行里持有
WRITE 的上游 `NodePalletPose` 与 YOLO 适配器。第一版的 `NodePalletPose` 测试
用自己的 client 去写，被 py_trees 挡了下来 —— 那个报错是对的。

运行：
    pytest orchestration/nodes/tests/test_node_pallet_servo.py -m unit -v

CI 的 verify:opensource 跑的正是 `pytest orchestration/nodes/tests/ -m unit`，
所以这个文件会被自动带上。
"""
import dataclasses
from types import SimpleNamespace
import json
import logging
import math
import shutil
import sys
import time
from pathlib import Path

import numpy as np
import py_trees
import pytest
from py_trees.common import Access, Status

from core.common.transform import matrix_to_pose6d, pose6d_to_matrix
from orchestration.nodes.node_inject_servo_input import NodeInjectServoInput
from orchestration.nodes.node_pallet_servo import (
    DumpWriter, NodePalletServo, _normalize_slots)
from orchestration.nodes.node_pallet_servo import TfCamBaseResolver
from skills.atomic.perception.pallet_servo.algorithm import (
    BoxObservation,
    Reject,
    ServoError,
    servo_error,
)

pytestmark = pytest.mark.unit


# --------------------------------------------------------------------------- #
# TfCamBaseResolver：逐级回退
# --------------------------------------------------------------------------- #
class FakeLookup:
    """一个可以随时掐断的假 TF 查询器。"""

    def __init__(self, T=None):
        self.T = T
        self.calls = 0

    def __call__(self, target, source):
        self.calls += 1
        return None if self.T is None else np.asarray(self.T, np.float64)


def _resolver(lookup, **over):
    kwargs = dict(camera_frame="camera_color_optical_frame",
                  base_frames=["base_link", "base_link_lb", "base_footprint"])
    kwargs.update(over)
    return TfCamBaseResolver(lookup, **kwargs)


def test_tf_success_returns_tf():
    T = np.eye(4)
    T[0, 3] = 0.25
    out, src = _resolver(FakeLookup(T)).resolve()
    assert src == "tf", src
    assert np.allclose(out, T)


def test_base_frame_candidates_are_tried_in_order():
    """第一个基座帧名查不到时，要接着试候选里的下一个。"""
    seen = []

    def lookup(target, source):
        seen.append(source)
        return np.eye(4) if source == "base_link_lb" else None

    out, src = _resolver(lookup).resolve()
    assert src == "tf", src
    assert seen == ["base_link", "base_link_lb"], seen


def test_short_outage_uses_cache_without_degrading():
    """短期抖动沿用最近一次成功值，**不降级**。"""
    T = np.eye(4)
    T[1, 3] = 0.5
    lookup = FakeLookup(T)
    r = _resolver(lookup)

    out, src = r.resolve()
    assert src == "tf"
    lookup.T = None
    for _ in range(4):                       # 连续 4 帧失败，还没到 5
        out, src = r.resolve()
        assert src == "cache", src
        assert np.allclose(out, T), out


def test_consecutive_failures_degrade_after_the_limit():
    """连续失败满 `fail_limit` 帧才降级到下一级。"""
    lookup = FakeLookup(np.eye(4))
    param = np.eye(4)
    param[2, 3] = 9.0
    r = _resolver(lookup, param=param, fail_limit=5)

    assert r.resolve()[1] == "tf"
    lookup.T = None
    for _ in range(4):
        assert r.resolve()[1] == "cache"
    out, src = r.resolve()                   # 第 5 次失败
    assert src == "param", src
    assert np.allclose(out, param), out


def test_no_cache_degrades_immediately():
    """一帧都没成功过时没有缓存可用，第一帧就降级。"""
    r = _resolver(FakeLookup(None))
    out, src = r.resolve()
    assert src == "identity", src
    assert np.allclose(out, np.eye(4)), out


def test_no_param_falls_back_to_identity():
    r = _resolver(FakeLookup(None), param=None)
    _, src = r.resolve()
    assert src == "identity"


def test_recovery_snaps_back_to_tf():
    """TF 恢复后要回到 tf 并清零计数，不是永久降级。"""
    T = np.eye(4)
    lookup = FakeLookup(T)
    r = _resolver(lookup, param=np.eye(4) * 7.0, fail_limit=2)
    assert r.resolve()[1] == "tf"
    lookup.T = None
    assert r.resolve()[1] == "cache"          # 第 1 帧失败：缓存还在，还没到上限
    assert r.resolve()[1] == "param"          # 第 2 帧失败：降级到显式参数
    assert r.fail_count == 2
    lookup.T = T
    out, src = r.resolve()
    assert src == "tf", src
    assert np.allclose(out, T), out
    assert r.fail_count == 0                  # 恢复后计数清零


# --------------------------------------------------------------------------- #
# helpers：合成场景（与 test_pallet_servo.py 同一套真值构造）
# --------------------------------------------------------------------------- #
FX = FY = 1000.0
CX, CY = 640.0, 400.0
PALLET_W_MM, PALLET_H_MM = 1200.0, 800.0
K_PARAM = [[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]]
IMAGE_SIZE = [3840, 2400]


def straight_down_camera() -> np.ndarray:
    """托盘正上方 1 m、垂直向下看。此时 1 mm = 1 px。

    平移那一步不能省：设成零的话托盘就落在相机光心上，`behind_camera` 会把
    每一帧都拒掉（那个判据是对的——真正在光心上的点投影不出东西）。
    """
    T = np.eye(4)
    T[2, 3] = 1.0
    return T


def tilt_cam_base(deg=5.0) -> np.ndarray:
    """一个**非单位阵**的 `T_cam_base`：旋转块不是 I、平移也不是零。

    为什么非要非单位阵：`T_cam_base @ pose6d_to_matrix(pose)` 这笔几何在两处各算
    一次（技能层算给算法用、节点算给 dump 用），而**单位阵下两处恒等** ——
    把组合顺序调过来，两条路径照样给出同一组数、回归网照样全绿。要钉住"顺序"
    就必须让乘积对顺序敏感。
    """
    a = math.radians(deg)
    T = np.eye(4)
    T[:3, :3] = [[1.0, 0.0, 0.0],
                 [0.0, math.cos(a), -math.sin(a)],
                 [0.0, math.sin(a), math.cos(a)]]
    T[:3, 3] = [0.05, -0.02, 0.10]
    return T


def synth_box(bottom_mid, half_len, theta_rad, height):
    d = np.array([math.cos(theta_rad), math.sin(theta_rad)], np.float64)
    up = np.array([d[1], -d[0]], np.float64)
    m = np.asarray(bottom_mid, np.float64)
    p0 = m + half_len * d
    p1 = m - half_len * d
    return np.array([p0, p1, p1 + height * up, p0 + height * up])


def a_box(e_bottom=137.0, e_right=88.0, theta=math.radians(7.0), stamp=0.0):
    """造一个"箱子在托盘内侧、离边 `e_bottom` / `e_right` px"的观测。

    ⚠️ **入参是"到边的距离"（正值），不是带符号的 `e`**（2026-09-30 写明）。
    台面的 v 范围是 `[CY, CY+800]`、u 范围是 `[CX, CX+1200]`，所以"内侧"就是从
    参考边往 `+v` / `−u` 走 —— 这个函数一直这么摆，**符号改了之后一行不用动**。
    （`e` 本身的符号 2026-09-30 翻过：箱子在内侧现在算**负**。`a_box()` 造的场景
    没变，变的是 `servo_error` 算出来的那个数，见各用例里的期望值。）
    """
    half, height = 150.0, 300.0
    right_mid_u = CX + PALLET_W_MM - e_right
    bottom_mid_u = (right_mid_u - half * math.cos(theta)
                    - (height / 2.0) * math.sin(theta))
    quad = synth_box((bottom_mid_u, CY + e_bottom), half, theta, height)
    return BoxObservation(quad=quad.tolist(), label="box", confidence=0.9,
                          stamp=stamp)


def make_node(tmp_path, **over):
    params = {
        "ref_edges": ["y=0", "x=W"],
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM,
        "image_size": IMAGE_SIZE,
        "dump_dir": str(tmp_path / "dump"),
        # ★ 本文件的既有用例**全是 base_link 老路径的回归网**（版本门禁直算、
        #   `T_cam_base` 来自 TF、dump 里 `t_cam_base_src` 期望 "tf"/"identity"）。
        #   新默认是 "camera"（相机系 + 按图像时刻配对），那条路走的是另外一套
        #   机制，由本文件末尾的新用例与 `test_pallet_box_pairing_e2e.py` 覆盖。
        #   默认在这里显式钉成 base_link，是为了让 40 多条既有用例**一条都不用改
        #   语义**（brief Step 17 的原话就是"在那条用例里显式加 pallet_frame:
        #   base_link，不要改默认值"—— 这里是同一个动作，只是这些用例共用了一个
        #   构造函数）。要测相机系就在用例里显式写 `pallet_frame="camera"`。
        "pallet_frame": "base_link",
    }
    params.update(over)
    node = NodePalletServo("servo", "servo", "ns", params)
    node.initialise()
    # ★ 节点启动时是**未激活**的（要 call `/pallet_servo/slot` 才出数）。本文件
    #   既有用例全都在测"激活之后"的行为，所以这里直接激活第 1 组。
    #   "未激活"那条路由 test_inactive_publishes_an_invalid_frame 单独覆盖，
    #   服务本身由 test_the_slot_service_* 那几条覆盖（单测里没有 ROS，服务
    #   建不起来，调用的是同一个 `_activate_slot`）。
    node._activate_slot(1)
    return node


def make_writer(*keys):
    """模拟上游：独立 client，持有这些键的写权限。"""
    writer = py_trees.blackboard.Client(name="test_upstream")
    for key in keys:
        writer.register_key(key=key, access=Access.WRITE)
    return writer


def write_inputs(writer, pallet_pose=None, box=None, pallet_version=1,
                 box_version=1):
    setattr(writer, "latest_pallet", pallet_pose)
    setattr(writer, "latest_pallet_version", pallet_version)
    setattr(writer, "latest_box_obs", box)
    setattr(writer, "latest_box_obs_version", box_version)


def replay_dump(payload):
    """把**节点自己写出的** dump 原样喂回 `servo_error`（design §6.1 硬要求）。

    与 `test_dump_writer_writes_something_rerunnable` 的区别是那个用的是手搓
    的 dict：把 payload 里的字段改名、把 `use_distortion` 删掉、把某个
    `BoxObservation` 字段漏掉，手搓的那条路**照样绿**，而真 dump 已经重跑不了
    了。这里只认落盘的那份 JSON。

    `use_distortion` 必须按**技能层的口径**还原：技能层在 `use_distortion=False`
    时把 `D` 置空（`skill.py` 的 `on_initialize`），所以"把 dump 里的 `D` 原样
    传回去"在那一侧会算出另一组数 —— 这就是"少一项就重跑不了"的样板。
    """
    # box 的字段一个都不能少：漏掉的会被 BoxObservation 的默认值悄悄补齐，
    # 重跑不报错、结果也不对 —— 必须显式钉住
    box_fields = {f.name for f in dataclasses.fields(BoxObservation)}
    assert set(payload["box"]) == box_fields, (
        f"dump 里的 box 字段与 BoxObservation 对不上："
        f"缺 {sorted(box_fields - set(payload['box']))}，"
        f"多 {sorted(set(payload['box']) - box_fields)}")

    D = None if payload["D"] is None else np.array(payload["D"], np.float64)
    if not payload["use_distortion"]:
        D = None
    # dump 里存的是**整张槽位表** + 当前槽位（板上写 N 组、轮到时切），
    # 所以"重跑这一帧"要按 `active_slot` 取**当前那一组** —— 拿整张表喂回去
    # 只会得到 `ref_edges_arity`（那是形状错，不是这一帧的几何）。
    edges = payload["ref_edges"]
    if edges and isinstance(edges[0], (list, tuple)):
        edges = edges[int(payload["active_slot"]) - 1]
    return servo_error(np.array(payload["T_cam_pallet"], np.float64),
                       payload["pallet_size_mm"], edges,
                       BoxObservation(**payload["box"]),
                       np.array(payload["K"], np.float64), D,
                       payload["image_size"])


def only_dump(tmp_path):
    """落盘的那一份 dump（每帧应当只写一份）。"""
    dumps = sorted((tmp_path / "dump").glob("*.json"))
    assert len(dumps) == 1, dumps
    return json.loads(dumps[0].read_text(encoding="utf-8"))


@pytest.fixture(autouse=True)
def _no_dry_run(monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)


# --------------------------------------------------------------------------- #
# DumpWriter
# --------------------------------------------------------------------------- #
def test_dump_writer_gating(tmp_path):
    """`dump_on` 决定什么时候写；`dump_dir` 为空则整体关闭。"""
    w = DumpWriter(tmp_path, ["reject", "warn"])
    assert w.enabled
    assert w.should_write(rejected=True, warn=[])
    assert w.should_write(rejected=False, warn=["x"])
    assert not w.should_write(rejected=False, warn=[])

    w = DumpWriter(tmp_path, "all")
    assert w.should_write(rejected=False, warn=[])

    w = DumpWriter(tmp_path, [])
    assert not w.enabled
    assert not w.should_write(rejected=True, warn=["x"])

    assert not DumpWriter(None, "all").enabled


def test_dump_on_unsupported_value_is_said_out_loud(tmp_path, caplog):
    """`dump_on` 写错了必须**点名说出来**，不能静默变成"永不落盘"（§7）。

    两条老路都是**静默**关闭：`"reject"`（少一层方括号，只认字符串 `"all"`）
    与 `["all"]`（`"all"` 是字符串形式专用的）。真出问题要 dump 的那一刻，
    现场只会看到"怎么一个 dump 都没有"。
    """
    with caplog.at_level(logging.WARNING):
        w = DumpWriter(tmp_path, "reject")           # 少写了一层方括号
    assert not w.enabled
    assert "dump_on 不认识" in caplog.text, caplog.text
    assert "'reject'" in caplog.text, caplog.text    # 实际值要点出来
    assert "all" in caplog.text and "reject" in caplog.text   # 可取值也要点出来

    caplog.clear()
    with caplog.at_level(logging.WARNING):
        w = DumpWriter(tmp_path, ["all"])
    assert not w.enabled
    assert "dump_on 里有不认识的值" in caplog.text, caplog.text
    assert "'all'" in caplog.text, caplog.text

    # 不认识的那一项忽略，**认识的那一项照常生效**（不是整体失效）
    caplog.clear()
    with caplog.at_level(logging.WARNING):
        w = DumpWriter(tmp_path, ["reject", "all"])
    assert w.enabled
    assert w.should_write(rejected=True, warn=[])
    assert not w.should_write(rejected=False, warn=["x"])
    assert "不认识" in caplog.text, caplog.text

    # 正确写法（含"关闭"）一个字都不许吵
    caplog.clear()
    with caplog.at_level(logging.WARNING):
        assert DumpWriter(tmp_path, "all").enabled
        assert DumpWriter(tmp_path, ["reject", "warn"]).enabled
        assert DumpWriter(tmp_path, []).enabled is False
        assert DumpWriter(None, "all").enabled is False
        assert DumpWriter(tmp_path, "").enabled is False      # 空串 = 关闭，有意写法
        assert DumpWriter(tmp_path, None).enabled is False
    assert "dump_on" not in caplog.text, caplog.text


def test_dump_writer_dedupes_and_rate_limits(tmp_path, caplog):
    """逐字相同的不重复写；离上一份不足 `min_interval_sec` 的不写 —— 但**不是无声丢弃**。

    伺服按 10 Hz 跑，而"整帧被拒"可以是**持续**状态：托盘太远或太侧时
    `edge_too_short` 一直成立，箱子观测却每帧都在更新 → 每帧都是新的版本对。
    不拦的话是一帧一份文件（≈1.5 KB/份 → 15 KB/s、50 MB/h）加每份一条
    WARNING，正好把要调试的上下文冲掉。
    """
    # ① 逐字相同：不写；少写的份数要**说在下一份的日志里**
    w = DumpWriter(tmp_path, "all", min_interval_sec=0.0)
    assert w.write({"reason": "same", "box_v": 1}) is not None
    assert w.write({"reason": "same", "box_v": 1}) is None
    assert len(list(tmp_path.glob("*.json"))) == 1
    with caplog.at_level(logging.WARNING):
        assert w.write({"reason": "moved", "box_v": 2}) is not None
    assert "此前跳过 1 份" in caplog.text, caplog.text
    assert "逐字相同 1" in caplog.text, caplog.text
    assert len(list(tmp_path.glob("*.json"))) == 2

    # ② 默认节流：内容变了、但离上一份不足 1 s → 也不写
    slow_dir = tmp_path / "slow"
    slow = DumpWriter(slow_dir, "all")
    assert slow.write({"n": 1}) is not None
    assert slow.write({"n": 2}) is None
    assert slow.write({"n": 3}) is None
    assert len(list(slow_dir.glob("*.json"))) == 1

    # ③ 节流可以关掉：`dump_min_interval_sec: 0` 就是"每帧一份"（调试跑法）
    fast_dir = tmp_path / "fast"
    fast = DumpWriter(fast_dir, "all", min_interval_sec=0.0)
    for i in range(3):
        assert fast.write({"n": i}) is not None
    assert len(list(fast_dir.glob("*.json"))) == 3


def test_dump_writer_rotates_keeping_the_newest(tmp_path):
    """★ 轮转：目录里**只留最近 `keep` 份**，从最旧的开始删（I4）。

    节流管的是"写得多快"，轮转管的是"写多久" —— `payload["stamp"] = time.time()`
    让"逐字相同"永不命中，只剩 1 秒限流，于是默认配置下是**稳态**的 ≈1 份/秒：
    7×24 就是 ≈8.6 万个 inode / ≈130 MB 一天。写爆磁盘的是"没有轮转"，不是"写"。
    """
    keep = 3
    w = DumpWriter(tmp_path, "all", min_interval_sec=0.0, keep=keep)
    first = w.write({"n": 0})
    for i in range(1, keep + 4):
        assert w.write({"n": i}) is not None
    names = sorted(p.name for p in tmp_path.glob("*.json"))
    assert len(names) == keep, f"没轮转：留下 {len(names)} 份 {names}"
    # 留下的是**最新的**：最后一份在、第一份没了
    assert json.loads((tmp_path / names[-1]).read_text(encoding="utf-8"))["n"] \
        == keep + 3
    assert first is not None and not first.exists(), \
        f"最旧的一份该被删掉：{first}"

    # 轮转**只删本类写出的文件名**：dump_dir 是操作员给的，别的东西一个不动
    (tmp_path / "notes.txt").write_text("别删我", encoding="utf-8")
    (tmp_path / "other.json").write_text("{}", encoding="utf-8")
    for i in range(keep + 2):
        assert w.write({"n": 100 + i}) is not None
    assert (tmp_path / "notes.txt").read_text(encoding="utf-8") == "别删我"
    assert (tmp_path / "other.json").exists()
    assert len(list(tmp_path.glob("*-*.json"))) == keep


def test_dump_write_failure_returns_none_without_raising(tmp_path, caplog):
    """盘满/目录被删/权限不对：`write()` 记 WARNING 后返回 None，**绝不抛**。

    **持续**失败（盘满）只喊第一次，之后按 `DumpWriter.FAIL_REPORT_EVERY` 计数 —— 与
    `_make_dir` 的"一次后关掉"同一个口径，逐帧 WARNING 同样会冲掉上下文。
    节流在这里关掉（`min_interval_sec=0`）：本条测的是**写失败**这条路。
    """
    w = DumpWriter(tmp_path / "gone", "all", min_interval_sec=0.0)
    assert w.enabled and w.write({"k": 1}) is not None

    shutil.rmtree(tmp_path / "gone")                 # 运行中目录没了
    with caplog.at_level(logging.WARNING):
        assert w.write({"k": 2}) is None             # 不抛
        for i in range(3, 6):                        # 一直失败也不刷屏
            assert w.write({"k": i}) is None
    assert "dump 写失败" in caplog.text, caplog.text
    assert str(tmp_path / "gone") in caplog.text, caplog.text
    assert caplog.text.count("dump 写失败") == 1, caplog.text


def test_dump_writer_writes_something_rerunnable(tmp_path):
    """dump 必须**完整到能重跑**——这是 design §6.1 的第二条硬要求。"""
    from skills.atomic.perception.pallet_servo.algorithm import (
        Reject, ServoError, servo_error)

    w = DumpWriter(tmp_path, "all")
    T = straight_down_camera()
    payload = {
        "reason": "reject:edge_too_short: 只有 2.4 px（阈值 20 px）",
        "T_cam_pallet": T.tolist(),
        "T_cam_base": np.eye(4).tolist(),
        "t_cam_base_src": "identity",
        # AABB 的底边（v2）落在参考底边内 137 px、右边（u2）落在参考右边内
        # 88 px —— 与 `a_box()` 那条路径给出的 (137, 88) 是同一对数，
        # 重跑出来的必须正是它们（brief 里这两格原为 900/800，会得到 400/940）
        "box": {"u1": 600.0, "v1": 500.0, "u2": 1752.0, "v2": 537.0,
                "quad": None, "label": "box", "confidence": 0.9, "stamp": 0.0},
        "ref_edges": ["y=0", "x=W"],
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM,
        "D": None,
        "use_distortion": True,
        "image_size": IMAGE_SIZE,
        "pallet_version": 3,
        "box_version": 7,
        "stamp": 0.0,
    }
    path = w.write(payload)
    assert path is not None and path.exists(), path

    back = json.loads(path.read_text(encoding="utf-8"))
    # 凭 dump 里的东西重跑，必须得到同一个结果
    obs = BoxObservation(**back["box"])
    again = servo_error(np.array(back["T_cam_pallet"], np.float64),
                        back["pallet_size_mm"], back["ref_edges"], obs,
                        np.array(back["K"], np.float64), back["D"],
                        back["image_size"])
    assert isinstance(again, ServoError), again
    assert abs(again.e_bottom_px + 137.0) < 1e-9, again.e_bottom_px
    assert back["t_cam_base_src"] == "identity"
    assert back["pallet_version"] == 3 and back["box_version"] == 7
    print(f"    dump 可重跑：{path.name}")


# --------------------------------------------------------------------------- #
# NodePalletServo
# --------------------------------------------------------------------------- #
def test_node_dry_run_skips_everything(tmp_path, monkeypatch):
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    node = make_node(tmp_path)
    assert node.update() == Status.SUCCESS


def test_node_startup_rejections_are_failures(tmp_path):
    """§7 的启动时拒绝：配置错就早失败，且 feedback 要说清下一步。"""
    assert make_node(tmp_path, ref_edges=["y=0", "y=H"]).update() == Status.FAILURE
    node = make_node(tmp_path, ref_edges=["y=0", "y=H"])
    assert "ref_edges" in node.feedback_message, node.feedback_message

    node = make_node(tmp_path, pallet_size_mm=None,
                     config_path=str(tmp_path / "nope.yaml"))
    assert node.update() == Status.FAILURE
    # M5：文案里**参数那条排第一**（新链路走 `pallet_size_mm` rosparam）。
    # 从前第一位是"先跑 pallet_calibrate.py 生成 config/pallet_tag.yaml"，
    # 而那个产物在新链路里已废弃、仓库里根本没有它 —— 照它做会卡在死路上。
    assert "pallet_size_mm" in node.feedback_message, node.feedback_message
    assert "apriltag" in node.feedback_message, node.feedback_message
    assert "pallet_calibrate" not in node.feedback_message, node.feedback_message

    node = make_node(tmp_path, K=[[1.0, 0.0], [0.0, 1.0]])
    assert node.update() == Status.FAILURE
    assert "K" in node.feedback_message, node.feedback_message

    node = make_node(tmp_path, T_cam_base_param=[[1.0, 0.0], [0.0, 1.0]])
    assert node.update() == Status.FAILURE
    assert "T_cam_base" in node.feedback_message, node.feedback_message


def test_reentering_initialise_does_not_respam_the_config_error(tmp_path, caplog):
    """★ 配置错误那条 ERROR 要**去重**（py_trees 每 tick 重进 `initialise()`）。

    `Behaviour.tick()` 对"状态不是 RUNNING"的行为每 tick 重进 `initialise()`，
    而配置错误这条路**恒返回 FAILURE** —— 不去重就是**每 tick 一条 ERROR**
    （10 Hz ≈ **600 条/分钟**），几百条一模一样的行把现场日志冲掉，而它要传达
    的只有第一行。

    ⚠️ 去重键取**整条文案**：换了配置（另一条错误）文案就变，新错误照样要落盘
    —— 只压"同一句话"，不压"新事实"。
    """
    import logging
    with caplog.at_level(logging.ERROR):
        node = make_node(tmp_path, ref_edges=["y=0", "y=H"])
        for _ in range(5):                       # py_trees 每 tick 就是这么干的
            node.initialise()
        errors = [r for r in caplog.records if r.levelno == logging.ERROR]
        assert len(errors) == 1, (
            f"同一句配置错误落了 {len(errors)} 条 —— 10 Hz 下就是 600 条/分钟："
            f"{[r.getMessage() for r in errors][:3]}")

        # 换了个错误（真换了配置）→ 文案变了，要落新的一条
        node._config_err = "另一个配置错误"
        node.initialise()
        errors = [r for r in caplog.records if r.levelno == logging.ERROR]
        assert len(errors) == 2, [r.getMessage() for r in errors]
    print("    同一句配置错误只落 1 条，换了一句才落新的")


@pytest.mark.parametrize("bad,needle", [
    ([640], "[640]"),
    ([640, 480, 2], "[640, 480, 2]"),
    ("640x480", "'640x480'"),
    ([0, 480], "480.0"),
    ([640, -480], "-480.0"),
    (["a", "b"], "['a', 'b']"),
    (640, "640"),
])
def test_node_image_size_bad_shape_is_a_startup_error(tmp_path, bad, needle):
    """`image_size` 的形状在**启动时**就查掉：坏值不许活到第一帧。

    不查的话它会在第一帧的 `on_execute` 里抛（`ref_edge_px` 取
    `image_size[1]` 抛 IndexError），被技能层的宽接接住 → **判成"这一帧被拒"**：
    每帧一条看着像几何退化的 WARNING（还带实际值、阈值，像模像样），外带每帧
    一份 dump（默认 `dump_on` 里就有 `"reject"`）。一个纯配置错误被伪装成几何
    问题，正是设计文档 §7 要堵的事。
    """
    node = make_node(tmp_path, image_size=bad)
    assert node.update() == Status.FAILURE, bad
    assert "image_size" in node.feedback_message, node.feedback_message
    assert needle in node.feedback_message, node.feedback_message   # 实际值要点出来
    assert not list((tmp_path / "dump").glob("*.json")), "配置错误不该留下 dump"


def test_node_image_size_may_be_omitted(tmp_path):
    """不给 `image_size` 是合法配置：只是不查"整条边跑到图外"。"""
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()), a_box())
    node = make_node(tmp_path, image_size=None)
    assert node.update() == Status.RUNNING, node.feedback_message
    assert getattr(node.global_blackboard, "latest_servo_error", None) is not None


def test_node_missing_K_is_a_startup_error_not_a_crash(tmp_path):
    """漏给 `K` 必须是**启动时的配置错误**，不许活到 `initialise()` 里抛异常。

    `_matrix_or_none(None, ...)` 返回 `(None, None)` —— 于是"压根没给"与"给了且
    合法"在那里长得一模一样，漏给曾经被判成"没有配置错误"。随后初始化 INFO 里那句
    `np.round(self._K, 3)` 抛 `TypeError`，而它在 `initialise()` 里 —— py_trees
    **不接**这个钩子抛的异常，于是**整棵树连本节点每帧的日志一起没**。

    与上一条 `image_size` 同源（设计文档 §7：配置错误要早失败），但后果更重：
    那条只是把一个配置错误伪装成几何退化（每帧一条像模像样的 WARNING），
    这条是**直接把树打死**。

    这条路径是真会走到的：`NOTES.md` 与本 README 都教人把 `K` 写进场景 JSON。
    """
    # `make_node` 里就会跑 `initialise()`，所以"没抛异常"本身就是修复的证明
    node = make_node(tmp_path, K=None)
    assert node.update() == Status.FAILURE, "漏给 K 要 FAILURE，不是把树打死"
    assert "K" in node.feedback_message, node.feedback_message
    assert not list((tmp_path / "dump").glob("*.json")), "配置错误不该留下 dump"


@pytest.mark.parametrize("bad_K", [
    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],        # 占位符最常见的样子
    [[0.0, 0.0, 640.0], [0.0, 1000.0, 400.0], [0.0, 0.0, 1.0]],  # 只漏了 fx
    [[1000.0, 0.0, 640.0], [0.0, 0.0, 400.0], [0.0, 0.0, 1.0]],  # 只漏了 fy
    [[-1000.0, 0.0, 640.0], [0.0, 1000.0, 400.0], [0.0, 0.0, 1.0]],  # 负焦距
])
def test_node_degenerate_K_is_a_startup_error(tmp_path, bad_K):
    """★ 形状对但**值退化**的 `K` 必须当场报配置错误，不许静默算出垃圾。

    真机场景里 `K` 是**手抄** `/camera/color/camera_info` 的，所以"先写个占位符、
    回头再填"是必然会发生的动作。全 0 的 `K` 形状是合法的 3×3（`_matrix_or_none`
    只查形状与 NaN/inf），于是它一路通过、每个投影点都塌到主点上 ——
    `theta` 与两个垂距照样是"像模像样"的数，日志、dump、叠图上都看不出这是配置
    错误。**比整棵树炸掉难查得多**，所以在启动时用一句话把它变成显式的失败。

    `cx/cy` 为 0 是**合法**的（主点落在图像角上虽然怪，但不是退化），别一起禁掉。
    """
    node = make_node(tmp_path, K=bad_K)
    assert node.update() == Status.FAILURE, f"退化 K 该 FAILURE，实际 {node.status}"
    assert "K" in node.feedback_message, node.feedback_message
    assert not list((tmp_path / "dump").glob("*.json")), "配置错误不该留下 dump"


def test_node_cx_cy_zero_is_still_accepted(tmp_path):
    """反面：只有 `fx/fy` 必须为正 —— `cx/cy` 为 0 不该被判成退化。"""
    node = make_node(tmp_path, K=[[1000.0, 0.0, 0.0],
                                  [0.0, 1000.0, 0.0],
                                  [0.0, 0.0, 1.0]])
    assert node.update() == Status.RUNNING, node.feedback_message
    assert "退化" not in node.feedback_message


def test_node_missing_size_does_not_build_a_tf_listener(tmp_path, monkeypatch):
    """**没有台面尺寸时不许起 TF 监听**，配置对了也只起一次。

    py_trees 的 `Behaviour.tick()` 对"状态不是 RUNNING"的行为**每 tick 重进
    `initialise()`**（`test_inject_plays_the_list_of_boxes_across_ticks` 那条
    用例证明了这条机制是真的），而本节点在"没有台面尺寸"这条路上返回 FAILURE。
    监听器要是构造在尺寸检查**之前**：装了 ROS 的机器上就是每秒 50 次
    构造/析构 `/tf` + `/tf_static` 的订阅（没装 ROS 时是 100 行/秒 WARNING），
    每帧还重读一遍 YAML。这条路的现实性在于 `pallet_size_mm` 是新增的键，
    **现存的 `config/pallet_tag.yaml` 一份都没有它** → 第一次接真机走的就是它。
    """
    calls = []
    monkeypatch.setattr("orchestration.nodes.node_pallet_servo._make_tf_lookup",
                        lambda *a, **k: calls.append(1))

    broken = make_node(tmp_path / "a", pallet_size_mm=None,
                       config_path=str(tmp_path / "nope.yaml"))
    assert broken.update() == Status.FAILURE
    for _ in range(5):                     # 模拟 py_trees 每 tick 重进 initialise()
        broken.initialise()
    assert calls == [], "配置错误这条路上不该起 TF 监听"

    good = make_node(tmp_path / "b")
    assert calls == [1], "配置对了要起监听"
    good.initialise()
    good.initialise()
    assert calls == [1], "重进 initialise() 不许重建监听（缓存与降级计数会被丢掉）"


def test_node_survives_upstream_objects_of_the_wrong_type(tmp_path, caplog):
    """上游给的是**别的类型**（dict / list）时，节点不许被 dump 路径弄死。

    `latest_box_obs` 写成普通 dict（离线工具那份 JSON 正是这个形状）、
    `latest_pallet` 写成 6 个数的 list，都会让 `on_execute` 抛异常 —— 技能层
    接住了（`skill_base.py:40-44` 只包 `on_execute`），判成"这一帧被拒"，而
    默认的 `dump_on` 里就有 `"reject"`：`_maybe_dump` 于是拿同一批坏对象**再解
    一遍**（`pose.to_list()` / `pose6d_to_matrix(pose)` / `getattr(box, f)`），
    这一次是在技能层那个 try **之外** —— 异常一路抛穿 `update()`，而 py_trees
    **不接** `update()` 抛出的异常 → **整棵树连每帧的日志一起没**。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    pose_list = list(matrix_to_pose6d(straight_down_camera()).to_list())
    setattr(writer, "latest_pallet", pose_list)          # 不是 Pose6D
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_box_obs", {"u1": 100.0, "v1": 100.0,        # 不是 BoxObservation
                                       "u2": 200.0, "v2": 200.0})
    setattr(writer, "latest_box_obs_version", 1)

    node = make_node(tmp_path)
    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING          # 不是异常，更不是整棵树没
        write_inputs(writer, pose_list, {"u1": 100.0, "v1": 100.0,
                                         "u2": 200.0, "v2": 200.0},
                     pallet_version=1, box_version=2)
        assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None
    assert "取不出来" in caplog.text, caplog.text       # 说明白 dump 为什么没落
    assert not list((tmp_path / "dump").glob("*.json"))
    assert caplog.text.count("取不出来") == 1, (
        "同一份坏输入反复来，这条 WARNING 要去重（否则又是每帧一条）")


@pytest.mark.parametrize("raw,expected", [
    ("false", False), ("False", False), (" 0 ", False), ("no", False),
    (False, False), (0, False),
    ("true", True), ("1", True), ("Yes", True), (True, True), (1, True),
])
def test_node_use_distortion_understands_string_booleans(tmp_path, raw, expected):
    """`use_distortion` 也会以**字符串**递进来，而裸 `bool()` 在这里是错的。

    工厂的 `READ_BOARD` 分支不做类型转换，所以 `use_distortion: "false"` 真会到；
    `bool("false") is True` → 畸变照开 → 与算像素的那一侧**静默分叉**（实测
    135.918 / 137.000 px，两个数看着都像模像样）。判据是**行为**：字符串写法
    必须给出与显式布尔写法**逐位相同**的三个量。

    三个量都要比，**不能只看 `e_bottom`**：这套合成场景里托盘底边正好压在
    `v = cy` 那一行上，畸变对它的 v 没有影响（u 上的位移不改变"底边到箱子底边"
    这个垂距），只看 `e_bottom` 会让这条用例在修复前也通过 —— 变量是 `e_right`
    与 `theta`（托盘右边在 u 上被畸变推走 ~8 px）。
    """
    D = [0.35, -0.12, 0.0, 0.0, 0.0]
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()),
                 a_box(theta=math.radians(60.0)))

    as_string = make_node(tmp_path / "str", D=D, use_distortion=raw,
                          key="servo_from_string")
    assert as_string.update() == Status.RUNNING
    got = getattr(as_string.global_blackboard, "servo_from_string", None)

    as_bool = make_node(tmp_path / "bool", D=D, use_distortion=expected,
                        key="servo_from_bool")
    assert as_bool.update() == Status.RUNNING
    want = getattr(as_bool.global_blackboard, "servo_from_bool", None)

    assert got is not None and want is not None
    assert abs(got.e_bottom_px - want.e_bottom_px) < 1e-9, (
        f"{raw!r} 被解释成了另一个值（e_bottom {got.e_bottom_px} vs "
        f"{want.e_bottom_px}）")
    assert abs(got.e_right_px - want.e_right_px) < 1e-9, (
        f"{raw!r} 被解释成了另一个值（e_right {got.e_right_px} vs "
        f"{want.e_right_px}）")
    assert abs(got.theta_rad - want.theta_rad) < 1e-12, (
        f"{raw!r} 被解释成了另一个值（theta {got.theta_rad} vs {want.theta_rad}）")


def test_node_use_distortion_unreadable_falls_back_to_the_default(tmp_path, caplog):
    """解释不了的值：按**默认值**（true）+ WARNING 点名实际值，**绝不抛**。

    与注入类节点"解释不了就关掉"的处置不同（那边关掉才是安全侧，见
    `node_inject_servo_input`），这里没有更安全的一侧 —— 所以退回本参数的默认值
    并说清实际值；`initialise()` 抛出才是绝不能干的（那会掀翻整棵树）。
    """
    with caplog.at_level(logging.WARNING):
        node = make_node(tmp_path, use_distortion="maybe")
    assert node._use_distortion is True
    assert "use_distortion" in caplog.text and "maybe" in caplog.text, caplog.text


def test_both_nodes_share_one_boolean_parser():
    """两个节点用的是**同一个** `parse_bool_param`，不是一个抄一个。

    `use_distortion` 与 `enabled` 是同一个坑（`bool("false") is True`）的两处
    实例，各写一份就是分叉的入口 —— 这条把"共用一份实现"钉死。
    """
    from orchestration.nodes import node_inject_servo_input as inject_mod
    from orchestration.nodes import node_pallet_servo as servo_mod
    from skills.atomic.perception.pallet_servo import algorithm as algo

    assert inject_mod.parse_bool_param is algo.parse_bool_param
    assert servo_mod.parse_bool_param is algo.parse_bool_param


def test_node_reads_size_from_calibration_yaml(tmp_path):
    """没给 `pallet_size_mm` 参数时，从 `config/pallet_tag.yaml` 读。"""
    import yaml
    cfg = tmp_path / "pallet_tag.yaml"
    cfg.write_text(yaml.safe_dump({"pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
                                   "T_pallet_tag_by_marker": {}}),
                   encoding="utf-8")

    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()), a_box())

    node = make_node(tmp_path, pallet_size_mm=None, config_path=str(cfg))
    assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None, node.feedback_message
    assert abs(out.e_bottom_px + 137.0) < 1e-9, out.e_bottom_px


def test_node_waits_while_inputs_are_missing(tmp_path):
    """黑板上还没有托盘/箱子时保持 RUNNING —— 这不是异常，是正常状态。"""
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    node = make_node(tmp_path)

    write_inputs(writer, None, None)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None

    write_inputs(writer, matrix_to_pose6d(straight_down_camera()), None)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None


def test_node_waits_when_keys_were_never_written(tmp_path):
    """键**压根没写过**（不是写了 `None`）→ 照旧 RUNNING，**不是异常**。

    上一版这里读的是 `getattr(self.global_blackboard, key, None)`，而 py_trees
    的 `Client.__getattr__` 在键不存在时抛的是 **`KeyError`**（不是
    `AttributeError`），那个 `None` 默认值根本接不住 —— `KeyError` 会一路抛穿
    `update()`。上一条测试写的是 `None`，那是个**存在**的键，所以没盖住这一格。

    这条路径不是假想的：`latest_box_obs` 至今全文没有生产者（Task 10 的模拟源
    是第一个），在那之前真实运行里的第一次 tick 就是"一个键都没写过"。
    """
    # 只注册 WRITE 权限，**一个键都不写**（register_key 只注册权限、不建值）
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    node = make_node(tmp_path)

    assert node.update() == Status.RUNNING, node.feedback_message
    assert "等输入" in node.feedback_message, node.feedback_message
    assert "箱子 缺" in node.feedback_message, node.feedback_message

    # 只写托盘的键，箱子的两个键仍然**从未**被写过
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 1)
    assert node.update() == Status.RUNNING, node.feedback_message
    assert "托盘 有" in node.feedback_message, node.feedback_message
    assert "箱子 缺" in node.feedback_message, node.feedback_message
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 0


def test_node_publishes_and_bumps_version(tmp_path):
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()), a_box())

    node = make_node(tmp_path)
    assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None, node.feedback_message
    assert abs(out.e_bottom_px + 137.0) < 1e-9, out.e_bottom_px
    assert abs(out.e_right_px + 88.0) < 1e-9, out.e_right_px
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 1
    # 出处字段由节点盖上去
    assert out.pallet_version == 1 and out.box_version == 1
    assert out.t_cam_base_src == "identity"
    assert out.stamp > 0.0, "节点必须盖时间戳"
    print(f"    节点输出：{out.to_log_line()}")


def test_node_output_equals_a_direct_algorithm_call(tmp_path):
    """节点不许自己加工几何：写进黑板的数必须与直接调 `servo_error` 一致。"""
    from skills.atomic.perception.pallet_servo.algorithm import servo_error

    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    box = a_box()
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()), box)

    node = make_node(tmp_path)
    assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_servo_error", None)

    direct = servo_error(straight_down_camera(),
                         (PALLET_W_MM, PALLET_H_MM), ("y=0", "x=W"), box,
                         np.array(K_PARAM, np.float64), None, IMAGE_SIZE)
    assert abs(out.e_bottom_px - direct.e_bottom_px) < 1e-9
    assert abs(out.e_right_px - direct.e_right_px) < 1e-9
    assert abs(out.theta_rad - direct.theta_rad) < 1e-12


def test_node_version_gate_skips_recompute(tmp_path):
    """两个版本都没变 → 不重算（否则 10 Hz 每 tick 都白算一遍）。"""
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()), a_box())
    node = make_node(tmp_path)

    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 1
    for _ in range(3):
        assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 1

    # 箱子动了（版本 +1）→ 必须重算
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()),
                 a_box(e_bottom=100.0), pallet_version=1, box_version=2)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 2
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert abs(out.e_bottom_px + 100.0) < 1e-9, out.e_bottom_px


def test_node_version_gate_reacts_to_the_pallet_version_alone(tmp_path):
    """**只有托盘**版本变（箱子版本没变）时也要重算。

    版本门禁是等值比较 `(pose_version, box_version)`，两个分量都得参与 ——
    门禁用例原来只动过箱子那一个分量，这条补上托盘那一个（M62）。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()), a_box(),
                 pallet_version=1, box_version=1)
    node = make_node(tmp_path)
    assert node.update() == Status.RUNNING
    first = getattr(node.global_blackboard, "latest_servo_error", None)
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 1

    # 托盘位姿动了（相机退到 1.2 m），箱子观测**一个字没改**
    back = straight_down_camera()
    back[2, 3] = 1.2
    write_inputs(writer, matrix_to_pose6d(back), a_box(),
                 pallet_version=2, box_version=1)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 2
    second = getattr(node.global_blackboard, "latest_servo_error", None)
    assert (second.pallet_version, second.box_version) == (2, 1)
    # 看 `e_right`：相机退远后托盘右边（x=W）在图上左移，e_right 从 −88 变到 +112
    # （符号 2026-09-30 翻过：内侧为负）。
    # （`e_bottom` 在这套合成场景里恰好不动 —— 托盘底边是 y=0，投影在 v=CY 上，
    # 与相机高度无关；三个量里只有它不敏感。这个细节正是这条用例的价值。）
    assert abs(second.e_right_px - first.e_right_px) > 1.0, (
        "托盘版本变了却没重算", first.e_right_px, second.e_right_px)
    assert abs(second.e_right_px - 112.0) < 1e-6, second.e_right_px


def test_node_warns_once_when_the_version_key_was_never_written(tmp_path, caplog):
    """生产者**写了值、却从没写过 `_version`**：必须一次性点名，不许静默停摆。

    这一格原来是最坏的一种：版本默认成 0，门禁第一次就停在 `(0, 0)`，节点算一次
    之后**永远不再算**，而 feedback 一直停在那一帧健康的读数上、日志一声不响 ——
    伺服于是持续追一个过期目标，现场看不出任何异常。根因是
    `_read_blackboard(..., version, 0)` 把"从未写过"和"写了个 0"混成了一个 0。

    黑板是**进程级**的，同文件里前面的用例早写过版本键了 —— 这条要的恰恰是"从
    未写过"，所以先清空（与 `_make_inject` 同一句先例）。
    """
    py_trees.blackboard.Blackboard.clear()
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_box_obs", a_box())
    # 两个 `_version` 键**一个都不写**（register_key 只注册权限、不建值）

    node = make_node(tmp_path)
    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING
        out = getattr(node.global_blackboard, "latest_servo_error", None)
        assert out is not None, node.feedback_message
        assert abs(out.e_bottom_px + 137.0) < 1e-9, out.e_bottom_px
        for _ in range(4):
            assert node.update() == Status.RUNNING
    assert "latest_pallet_version **从未被写过**" in caplog.text, caplog.text
    assert "latest_box_obs_version **从未被写过**" in caplog.text, caplog.text
    assert "latest_pallet" in caplog.text, caplog.text       # 点名到具体键
    assert caplog.text.count("从未被写过") == 2, (
        "一次性警告：每个键只喊一次，不是每帧一遍")
    # 停摆本身也钉住：这正是要提醒的那件事
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 1


def test_node_reject_frame_writes_nothing_but_dumps(tmp_path):
    """退化帧：不写黑板、版本不自增、feedback 带原因、dump 落盘。"""
    far = straight_down_camera()
    far[2, 3] = 500.0                    # 相机退到 500 m 高 → 边太短
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(far), a_box())

    node = make_node(tmp_path, log_every_n=1)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 0
    assert "edge_too_short" in node.feedback_message, node.feedback_message
    assert "阈值" in node.feedback_message, node.feedback_message

    dumps = sorted((tmp_path / "dump").glob("*.json"))
    assert len(dumps) == 1, dumps
    payload = json.loads(dumps[0].read_text(encoding="utf-8"))
    assert "edge_too_short" in payload["reason"], payload["reason"]
    assert payload["ref_edges"] == [["y=0", "x=W"]]   # 与 make_node 的参数一致
    assert payload["active_slot"] == 1                # 重跑时要按它取当前那组
    assert np.allclose(payload["T_cam_pallet"], far)
    print(f"    退化帧：黑板未写、dump 已落 {dumps[0].name}")


def test_node_and_skill_report_the_same_frame_number(tmp_path, caplog):
    """同一个事件只许有**一个**帧号：技能那条与节点那条必须是同一个数。

    节点每帧都调 `initialize()`（技能的超时靠它刷新 `_start_time`），而技能原来
    在 `on_initialize` 里把 `_frame_no` 归零 → 每一条技能 WARNING 都写"第 1 帧"，
    节点那条写的是"第 N 帧"。两条都是 WARNING，默认级别下**都会落盘** ——
    对不上就没法用帧号把两条日志对起来。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    far = straight_down_camera()
    far[2, 3] = 500.0                    # 相机退到 500 m 高 → 边太短

    node = make_node(tmp_path)
    with caplog.at_level(logging.WARNING):
        # 第 1 帧：健康的
        write_inputs(writer, matrix_to_pose6d(straight_down_camera()), a_box(),
                     pallet_version=1, box_version=1)
        assert node.update() == Status.RUNNING
        # 第 2 帧：被拒
        write_inputs(writer, matrix_to_pose6d(far), a_box(),
                     pallet_version=2, box_version=2)
        assert node.update() == Status.RUNNING
    assert "托盘伺服第 2 帧被拒" in caplog.text, caplog.text       # 节点那条
    assert "托盘伺服本帧被拒（第 2 帧）" in caplog.text, caplog.text  # 技能那条


def test_node_warn_frame_publishes_and_dumps(tmp_path):
    """带 warn 的帧要**照常输出**，同时留下 dump。"""
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()),
                 a_box(theta=math.radians(60.0)))

    node = make_node(tmp_path)
    assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None, "带 warn 的帧不该被丢掉"
    assert out.warn, out.warn
    assert len(list((tmp_path / "dump").glob("*.json"))) == 1


def test_node_dump_replays_its_own_reject(tmp_path):
    """节点自己落的 dump，重跑必须得到**同一个 Reject** —— 与内存里那一帧一致。

    这是 design §6.1 第二条硬要求的回归网：字段改名、少写一项，都会在这条上炸。
    """
    far = straight_down_camera()
    far[2, 3] = 500.0                    # 相机退到 500 m 高 → 边太短
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(far), a_box(),
                 pallet_version=3, box_version=7)

    node = make_node(tmp_path)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None

    payload = only_dump(tmp_path)
    again = replay_dump(payload)
    direct = servo_error(far, (PALLET_W_MM, PALLET_H_MM), ["y=0", "x=W"], a_box(),
                         np.array(K_PARAM, np.float64), None, IMAGE_SIZE)
    assert isinstance(again, Reject), again
    assert isinstance(direct, Reject), direct
    assert again == direct, (again, direct)          # 值对象：code + detail 全等
    # 与**内存里**那一帧的结果一致（拒绝帧不进黑板，feedback 就是它的原文）
    assert str(again) == node.feedback_message, (str(again), node.feedback_message)
    # 出处也在 dump 里，且对得上这一帧
    assert payload["t_cam_base_src"] == "identity", payload["t_cam_base_src"]
    assert (payload["pallet_version"], payload["box_version"]) == (3, 7)
    print(f"    dump 重跑出同一个 Reject：{again}")


@pytest.mark.parametrize("use_distortion", [True, False])
def test_node_dump_replays_its_own_warn_frame(tmp_path, use_distortion):
    """带 warn 的帧：dump 重跑出的**三个量 + warn 列表**必须与内存里那一帧一致。

    `use_distortion` 两侧都测，且 `D` 给的是明显不为零的数：技能层在
    `use_distortion=False` 时把 `D` 置空，所以"把 dump 里的 `D` 原样传回去"
    在那一侧会算出另一组数 —— dump 少这一项就重跑不了，这里钉住它。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()),
                 a_box(theta=math.radians(60.0)))

    node = make_node(tmp_path, D=[0.35, -0.12, 0.0, 0.0, 0.0],
                     use_distortion=use_distortion)
    assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None, "带 warn 的帧不该被丢掉"
    assert out.warn, out.warn

    payload = only_dump(tmp_path)
    assert payload["use_distortion"] is use_distortion, payload["use_distortion"]
    again = replay_dump(payload)
    assert isinstance(again, ServoError), again
    assert abs(again.e_bottom_px - out.e_bottom_px) < 1e-9, (again, out)
    assert abs(again.e_right_px - out.e_right_px) < 1e-9, (again, out)
    assert abs(again.theta_rad - out.theta_rad) < 1e-12, (again, out)
    assert again.warn == out.warn, (again.warn, out.warn)
    print(f"    dump 重跑一致（use_distortion={use_distortion}）："
          f"{again.to_log_line()}")


def test_node_dump_pins_the_composition_order_with_a_non_identity_t_cam_base(
        tmp_path, monkeypatch):
    """dump 回归网把两处 `T_cam_base @ pose6d_to_matrix(pose)` **钉在一起** —— 前提是
    `T_cam_base` **不是单位阵**。

    两处指的是技能层（算给算法用，`PalletServoSkill.on_execute`）与节点（算给
    dump 用，`_maybe_dump`）。原来的 dump 用例 `t_cam_base` **全是单位阵**：单位阵
    下乘法可交换，"两处逐字一致"这句话退化成空话 —— 谁把技能层的组合顺序调过来，
    套件照样全绿，而 dump 已经静默偏向（M62 的另一半）。
    """
    T = tilt_cam_base()
    monkeypatch.setattr("orchestration.nodes.node_pallet_servo._make_tf_lookup",
                        lambda *a, **k: (lambda target, source: T))

    pose = matrix_to_pose6d(straight_down_camera())
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, pose,
                 a_box(theta=math.radians(60.0)))     # 带 warn → 默认就会落 dump

    node = make_node(tmp_path)
    assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None and out.warn, out
    assert out.t_cam_base_src == "tf", out.t_cam_base_src

    payload = only_dump(tmp_path)
    assert np.allclose(payload["T_cam_base"], T), payload["T_cam_base"]
    # 节点自己算的那一笔 = 技能层那一笔（顺序一致）
    assert np.allclose(payload["T_cam_pallet"], T @ pose6d_to_matrix(pose))
    # 而且**交换顺序会得到明显不同的矩阵** —— 上面那条断言因此有牙
    swapped = pose6d_to_matrix(pose) @ T
    assert not np.allclose(payload["T_cam_pallet"], swapped)
    assert np.abs(np.array(payload["T_cam_pallet"]) - swapped).max() > 0.05

    again = replay_dump(payload)
    assert isinstance(again, ServoError), again
    assert abs(again.e_bottom_px - out.e_bottom_px) < 1e-9, (again, out)
    assert abs(again.e_right_px - out.e_right_px) < 1e-9, (again, out)
    assert abs(again.theta_rad - out.theta_rad) < 1e-12, (again, out)
    print(f"    非单位阵 T_cam_base 下 dump 与技能层一致（src={out.t_cam_base_src}）："
          f"e_bottom={out.e_bottom_px:+.3f}px")


def test_node_dump_dir_that_cannot_be_made_is_only_a_warning(tmp_path, caplog):
    """`dump_dir` 建不出来（此处让它落在一个**文件**底下）→ 帧照常发布。"""
    blocker = tmp_path / "not_a_dir"
    blocker.write_text("x", encoding="utf-8")        # 拿文件当目录的父级

    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()),
                 a_box(theta=math.radians(60.0)))

    with caplog.at_level(logging.WARNING):
        node = make_node(tmp_path, dump_dir=str(blocker / "dump"))
    assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None, node.feedback_message          # 帧照常发布
    assert out.warn, out.warn
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 1
    assert "dump 目录建不出来" in caplog.text, caplog.text
    assert str(blocker / "dump") in caplog.text, caplog.text


def test_node_dump_write_failure_does_not_break_the_loop(tmp_path, caplog):
    """运行中 dump 写失败（目录被删/盘满）→ 照常发布、版本照增、拒绝帧照旧拒。

    一个**诊断**设施把伺服环路搞崩，正好和它的用途相反。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()),
                 a_box(theta=math.radians(60.0)))
    node = make_node(tmp_path)
    assert (tmp_path / "dump").is_dir()
    shutil.rmtree(tmp_path / "dump")                 # 运行中目录没了

    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None, node.feedback_message          # 帧照常发布
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 1
    assert "dump 写失败" in caplog.text, caplog.text
    assert str(tmp_path / "dump") in caplog.text, caplog.text

    # 拒绝帧那一侧：dump 照样写不成，但"不发布、版本不自增"的契约不变
    far = straight_down_camera()
    far[2, 3] = 500.0
    write_inputs(writer, matrix_to_pose6d(far), a_box(),
                 pallet_version=1, box_version=2)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error") is not None
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 1
    assert "edge_too_short" in node.feedback_message, node.feedback_message


def test_node_dump_can_be_disabled(tmp_path):
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()),
                 a_box(theta=math.radians(60.0)))
    node = make_node(tmp_path, dump_on=[])
    assert node.update() == Status.RUNNING
    assert not (tmp_path / "dump").exists()


# --------------------------------------------------------------------------- #
# NodeInjectServoInput：离线模拟源（写真实链路的那两个键）
# --------------------------------------------------------------------------- #
def _sim_payload(boxes):
    """一份 pick_servo_inputs.py 会写出来的模拟输入（字段与工具一一对应）。"""
    return {
        "source": "test",
        "ref_edges": ["y=0", "x=W"],
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM, "D": None, "image_size": IMAGE_SIZE,
        "use_distortion": True,
        "pallet_pose": matrix_to_pose6d(straight_down_camera()).to_list(),
        "box": boxes,
    }


def _write_json(path, payload):
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    return path


def write_sim_json(path, boxes):
    """造一份 pick_servo_inputs.py 会写出来的模拟输入。"""
    return str(_write_json(path, _sim_payload(boxes)))


def _make_inject(sim_path, **over):
    params = {"sim_path": str(sim_path)}
    params.update(over)
    # 全局黑板是**进程级**的：同文件里前面的用例已经写过 latest_pallet 了，
    # 而这一块用例断言的恰恰是"写了没有"，必须从干净的黑板起步。
    # 仓库先例：test_for_each.py / test_repeat_until.py 里的 Blackboard.clear()。
    py_trees.blackboard.Blackboard.clear()
    node = NodeInjectServoInput("inject", "inject", "ns", params)
    node.initialise()
    return node


def make_inject(tmp_path, boxes, **over):
    return _make_inject(write_sim_json(tmp_path / "sim.json", boxes), **over)


def make_inject_json(tmp_path, payload, **over):
    """像 `make_inject`，但 payload 自己给 —— 用来造坏输入。"""
    return _make_inject(_write_json(tmp_path / "sim.json", payload), **over)


def _box_dict(e_bottom=137.0, e_right=88.0, theta=math.radians(7.0)):
    half, height = 150.0, 300.0
    right_mid_u = CX + PALLET_W_MM - e_right
    bottom_mid_u = (right_mid_u - half * math.cos(theta)
                    - (height / 2.0) * math.sin(theta))
    quad = synth_box((bottom_mid_u, CY + e_bottom), half, theta, height)
    return {"u1": float(quad[:, 0].min()), "v1": float(quad[:, 1].min()),
            "u2": float(quad[:, 0].max()), "v2": float(quad[:, 1].max()),
            "quad": quad.tolist(), "label": "box", "confidence": 1.0,
            "stamp": 0.0}


def test_inject_writes_the_two_real_keys(tmp_path):
    """模拟源写的是**真实链路里的那两个键**，不是自造的键。"""
    node = make_inject(tmp_path, _box_dict())
    bb = node.global_blackboard
    # 源节点口径：有东西要写就是 RUNNING，由父节点决定何时收 —— 与
    # `NodePercep` / `NodePalletPose` 一样。返回 SUCCESS 会让 py_trees 每 tick
    # 重进 `initialise()`，逐条播放的语义随即失效（见下面 tick 驱动的用例）。
    assert node.update() == Status.RUNNING
    assert getattr(bb, "latest_pallet", None) is not None
    assert getattr(bb, "latest_pallet_version", 0) == 1
    obs = getattr(bb, "latest_box_obs", None)
    assert isinstance(obs, BoxObservation), obs
    assert obs.quad is not None and len(obs.quad) == 4
    print("    模拟源写入 latest_pallet / latest_box_obs + 版本号")


def test_inject_plays_a_sequence_one_per_tick(tmp_path):
    """`box` 是列表时播放一条，版本号递增 —— 这样才看得到伺服"实时"变化。

    播完之后必须**停手**：拿最后一条反复写的话版本号一直涨，下游会跟着一遍遍
    重算同一个数。

    这条直接调 `update()`，所以**碰不到**行为树的那条路径；"逐 tick、且
    `initialise()` 不许重进"的树语义由下面那条 tick 驱动的用例钉住。
    """
    boxes = [_box_dict(e_bottom=137.0), _box_dict(e_bottom=100.0),
             _box_dict(e_bottom=40.0)]
    node = make_inject(tmp_path, boxes)
    bb = node.global_blackboard
    for i in range(3):
        assert node.update() == Status.RUNNING
        assert getattr(bb, "latest_box_obs_version") == i + 1
    for _ in range(3):
        assert node.update() == Status.RUNNING
    assert getattr(bb, "latest_box_obs_version") == 3, "播完之后版本号不该再涨"
    print("    序列播放：三条各写一次，播完停手")


def test_inject_plays_the_list_across_ticks_without_replaying(tmp_path):
    """**走 tick 机制**：三条必须逐 tick 各播一条，并且 `initialise()` 不许重进。

    上面那条直接调 `update()`，永远碰不到 py_trees 的这条路径：
    `Behaviour.tick()` 在 `status != RUNNING` 时**每 tick 重进 `initialise()`**，
    而 `initialise()` 会把游标归零 —— 于是"列表"其实只剩第一条：每 tick 重写
    同一个框、版本号却一直涨，下游的版本门禁每帧重算同一个数，正是 `update()`
    里那句"播完停手"要避免的事。

    `tick_once()` 是 py_trees 2.2 里的直接形式（`tick()` 是生成器；仓库里
    `test_repeat_until.py` 用的也是 `tick_once()`）。
    """
    boxes = [_box_dict(e_bottom=137.0), _box_dict(e_bottom=100.0),
             _box_dict(e_bottom=40.0)]
    node = make_inject(tmp_path, boxes)
    bb = node.global_blackboard

    entries = []                                  # initialise() 被重进了几次
    original = node.initialise

    def counting_initialise():
        entries.append(1)
        return original()

    node.initialise = counting_initialise

    for i, entry in enumerate(boxes):
        node.tick_once()                          # 父节点怎么 tick，这里就怎么 tick
        assert node.status == Status.RUNNING, node.feedback_message
        assert getattr(bb, "latest_box_obs").quad == entry["quad"], (
            f"第 {i + 1} tick 播的不是第 {i + 1} 条")
        assert getattr(bb, "latest_box_obs_version") == i + 1
        assert getattr(bb, "latest_pallet_version") == i + 1, "托盘也该跟着写"
    assert len(entries) == 1, (
        f"initialise() 被重进了 {len(entries)} 次：游标被归零，只会反复播第一条")

    last = getattr(bb, "latest_box_obs")
    for _ in range(3):
        node.tick_once()
    assert len(entries) == 1, "播完之后又重进了 initialise()（列表会从头再播）"
    assert getattr(bb, "latest_box_obs") is last, "播完之后又写了一次"
    assert getattr(bb, "latest_box_obs_version") == 3, "播完之后版本号还在涨"
    assert "播放完毕" in node.feedback_message, node.feedback_message
    print("    tick 驱动：三条按序播完，initialise() 只进一次，播完停手")


def test_inject_leaves_untouched_when_disabled(tmp_path):
    """`enabled: false` 时什么都不写 —— 真机上靠它把这个节点关掉。

    断言的是整个 storage **一个字节都没变**（比 `getattr(bb, key, None) is None`
    严得多：py_trees 在键**从未被写过**时 `getattr` 抛的是 `KeyError`，那个
    `None` 默认值接不住，仓库里 `node_pallet_servo._read_blackboard` 的注释写着
    这个坑；而且它分不清"写了个 `None`"与"压根没写"）。

    起点不是空黑板：构造时按真生产者的口径预置了 `None` / `0` 四个键（见
    `test_inject_presets_the_keys_like_the_real_producer`）。那不是"写数据"，
    只要 update 期间这几个值**原样不动**，就还是"什么都不写"。
    """
    node = make_inject(tmp_path, _box_dict(), enabled=False)
    before = dict(py_trees.blackboard.Blackboard.storage)
    # 前提：起点只有那四个预置键（make_inject 里的 clear + 构造时的预置），
    # 否则下面那句"没动过"可能本来就是真的
    assert before == {"/latest_pallet": None, "/latest_pallet_version": 0,
                      "/latest_box_obs": None, "/latest_box_obs_version": 0}, before

    # 关掉 = 不是这条链路上的参与者，**不占着树**：SUCCESS 让父节点照常往下走。
    # 若返回 RUNNING，真机上"关掉模拟源"这件事本身就会把整条分支永远卡住。
    assert node.update() == Status.SUCCESS
    assert dict(py_trees.blackboard.Blackboard.storage) == before, (
        "enabled=false 却写了黑板")
    print("    enabled=false：黑板一个键都没动")


def test_inject_presets_the_keys_like_the_real_producer(tmp_path):
    """构造时就把四个键置成 `None` / `0` —— 与 `NodePalletPose.__init__` 同一个写法。

    `register_key` 在 py_trees 2.x 里**只注册权限、不创建值**：不预置的话，别的
    节点在本节点第一次写之前 `getattr` 就是 `KeyError`（真生产者踩过、并把坑写在
    注释里）。本节点的用途就是"和真生产者分不出两样"，这一条也得一样。
    """
    py_trees.blackboard.Blackboard.clear()
    node = NodeInjectServoInput("inject", "inject", "ns",
                                {"sim_path": str(tmp_path / "nope.json")})
    bb = node.global_blackboard
    # 故意**不带默认值**：py_trees 的 `__getattr__` 在键从未被写过时抛 KeyError，
    # 这一串就是"预置过了"的证明
    assert getattr(bb, "latest_pallet") is None
    assert getattr(bb, "latest_pallet_version") == 0
    assert getattr(bb, "latest_box_obs") is None
    assert getattr(bb, "latest_box_obs_version") == 0

    # 换键名（键是参数）也要预置
    other = NodeInjectServoInput("inject2", "inject2", "ns",
                                 {"sim_path": str(tmp_path / "nope.json"),
                                  "pallet_key": "sim_pallet",
                                  "box_key": "sim_box"})
    assert getattr(other.global_blackboard, "sim_pallet") is None
    assert getattr(other.global_blackboard, "sim_box") is None
    assert getattr(other.global_blackboard, "sim_box_version") == 0
    print("    构造即预置 None/0（与 NodePalletPose.__init__ 一致）")


# --- write_pallet: false —— 真机过渡期：托盘走 apriltag，箱子还在手点 ---------- #
def _make_real_producer():
    """冒充 `NodePalletPose`：一个持有 `latest_pallet` 写权限的 client。"""
    client = py_trees.blackboard.Client(name="fake_pallet_pose")
    for key in ("latest_pallet", "latest_pallet_version"):
        client.register_key(key=key, access=Access.WRITE)
    return client


def test_inject_write_pallet_false_never_touches_the_pallet_key(tmp_path):
    """`write_pallet: false` 时**连键都不创建** —— 托盘整个让给真生产者。

    真机过渡期的第三种状态：托盘已经是真数据（`NodePalletPose` 从二维码反算），
    只有箱子还得手点。不加这个开关的话两个节点会**同时往 `latest_pallet` 上写**，
    伺服读到哪一份取决于 tick 顺序，**没有任何报错**。

    判据是**黑板 storage 里没有这两个键**，而不是 `getattr(bb, key, None)`：
    · `getattr(bb, "latest_pallet")` 会抛 **`AttributeError`**（"client 'inject'
      does not have read/write access"）—— 本节点压根没为它注册权限。但那句话
      说的是**权限**，如果换成一个注册过 READ 的 client 去读，就会变成 `KeyError`
      （py_trees 先查权限、再查存在）。两种异常都只说明"这个 client 读不到"，
      分不清"键不存在"和"键存在但没权限"。
    · `getattr(bb, key, None)` 更是接不住 `KeyError`（仓库里 `_read_blackboard`
      的注释写着这个坑）。
    直查 storage 才是在问"这个键到底建了没有"。
    """
    node = make_inject(tmp_path, _box_dict(), write_pallet="false")
    assert node.update() == Status.RUNNING, node.feedback_message

    storage = py_trees.blackboard.Blackboard.storage
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 1, "箱子该照写"
    assert "/latest_pallet" not in storage, "托盘键被本节点创建了"
    assert "/latest_pallet_version" not in storage, "托盘版本键被本节点创建了"
    print("    write_pallet=false：只写箱子，托盘键从未被创建")


def test_inject_write_pallet_false_lets_the_real_producer_own_the_key(tmp_path):
    """★ 真生产者写进去的托盘位姿，被本节点 tick 多少次都要**原样不动**。

    这是这条开关存在的全部理由。断言的是值和版本号**都没变**：版本号是下游的
    门禁（`NodePalletServo` 靠 `latest_pallet_version` 判断要不要重算），
    本节点只要碰一下它，伺服就会拿同一个托盘白算一遍。
    """
    boxes = [_box_dict(e_bottom=137.0), _box_dict(e_bottom=100.0),
             _box_dict(e_bottom=40.0)]
    node = make_inject(tmp_path, boxes, write_pallet="false")
    producer = _make_real_producer()
    pose = matrix_to_pose6d(straight_down_camera())
    setattr(producer, "latest_pallet", pose)
    setattr(producer, "latest_pallet_version", 7)

    for _ in range(5):
        assert node.update() == Status.RUNNING
    assert getattr(producer, "latest_pallet") is pose, "托盘位姿被本节点覆盖了"
    assert getattr(producer, "latest_pallet_version") == 7, "托盘版本号被本节点动了"
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 3, (
        "箱子该照常逐条播（3 条播完停手）")
    print("    write_pallet=false：真生产者的位姿与版本号都不受影响")


def test_inject_write_pallet_false_does_not_need_pallet_pose_in_the_json(tmp_path):
    """`write_pallet: false` 时 JSON 里没有 `pallet_pose` 也**不该失败**。

    与下一条**互为反面**：同一份缺 `pallet_pose` 的输入，默认开关下必须是 FAILURE
    （那是坏输入），关了托盘开关后必须是 RUNNING（那份数据不归本节点管，
    为它失败只会凭空多一个失败模式）。
    """
    payload = _sim_payload(_box_dict())
    payload.pop("pallet_pose")

    node = make_inject_json(tmp_path, payload, write_pallet="false")
    assert node.update() == Status.RUNNING, node.feedback_message
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 1
    print("    write_pallet=false：JSON 缺 pallet_pose 也照跑")


def test_inject_write_pallet_true_still_requires_the_pallet_pose(tmp_path):
    """反面：默认（写托盘）时缺 `pallet_pose` 仍是 FAILURE，未被上一条放松。"""
    payload = _sim_payload(_box_dict())
    payload.pop("pallet_pose")

    node = make_inject_json(tmp_path, payload)          # write_pallet 默认 true
    assert node.update() == Status.FAILURE, "写托盘时缺 pallet_pose 必须还是坏输入"
    assert "pallet_pose" in node.feedback_message, node.feedback_message
    print("    write_pallet=true：缺 pallet_pose 仍然是 FAILURE")


@pytest.mark.parametrize("raw,expected", [
    ("false", False), ("False", False), (" 0 ", False), ("no", False), (False, False),
    ("true", True), ("1", True), ("Yes", True), (True, True),
])
def test_inject_write_pallet_understands_string_booleans(tmp_path, raw, expected):
    """与 `enabled` 同一个坑：场景 JSON 会递字符串进来，而 `bool("false") is True`。

    判据是"托盘键建了没有" —— 那正是这个开关唯一的效果。
    """
    node = make_inject(tmp_path, _box_dict(), write_pallet=raw)
    assert node.update() == Status.RUNNING
    storage = py_trees.blackboard.Blackboard.storage
    if expected:
        assert "/latest_pallet" in storage, f"write_pallet={raw!r} 该写托盘却没有"
        assert getattr(node.global_blackboard, "latest_pallet_version") == 1
    else:
        assert "/latest_pallet" not in storage, f"write_pallet={raw!r} 没关掉托盘"
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 1, "箱子总是写的"
    print(f"    write_pallet={raw!r} → {'写托盘' if expected else '不写托盘'}")


def test_inject_write_pallet_with_a_value_it_cannot_read_falls_back_to_true(tmp_path, caplog):
    """解释不了的值**退回默认 true**（写托盘）并报错 —— 方向与 `enabled` 相反。

    `enabled` 那边解释不了按"关掉"走，因为关掉是安全的（真机必须能停下）。
    这里反过来：这里的"严"是**什么都不写**，那会让下游永远等输入、比写多了更难查，
    所以退回默认行为才是安全的那一侧。两处方向不同是刻意的，各自写了理由。
    """
    with caplog.at_level(logging.ERROR):
        node = make_inject(tmp_path, _box_dict(), write_pallet="也许吧")
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1, (
        "解释不了的值该退回默认 true（写托盘）")
    assert "write_pallet" in caplog.text and "也许吧" in caplog.text, caplog.text
    print("    write_pallet='也许吧' → 退回 true 并报错")


@pytest.mark.parametrize("raw,expected", [
    ("false", False), ("False", False), (" 0 ", False), ("no", False), ("NO", False),
    (False, False), (0, False),
    ("true", True), ("1", True), ("Yes", True), (True, True), (1, True),
])
def test_inject_enabled_understands_string_booleans(tmp_path, raw, expected):
    """`enabled` 会以**字符串**递进来（场景 JSON / 黑板），而 `bool("false") is True`。

    工厂的 READ_BOARD 分支不像 RESOLVED 那样转类型，所以字符串是真会到的；仓库里
    已有的 `_as_bool` 就是为这条路写的。真机上这是硬约束：必须能真关掉。
    """
    node = make_inject(tmp_path, _box_dict(), enabled=raw)
    bb = node.global_blackboard
    before = dict(py_trees.blackboard.Blackboard.storage)
    # 开着的在干活（RUNNING，源节点口径）；关掉的什么都不做、也不占着树（SUCCESS）
    assert node.update() == (Status.RUNNING if expected else Status.SUCCESS)
    if expected:
        assert getattr(bb, "latest_pallet_version") == 1
        assert getattr(bb, "latest_box_obs_version") == 1
    else:
        assert dict(py_trees.blackboard.Blackboard.storage) == before, (
            f"enabled={raw!r} 没关掉")
    print(f"    enabled={raw!r} → {'写入' if expected else '不写'}（按 {expected} 解释）")


def test_inject_enabled_with_a_value_it_cannot_read_stays_off_and_shouts(tmp_path, caplog):
    """既不认识又解释不了的 `enabled`：**按关掉处理**（真机上必须能关），但要在
    feedback 与日志里喊出来 —— 静默当 true 跑正是这条的病根。"""
    with caplog.at_level(logging.ERROR):
        node = make_inject(tmp_path, _box_dict(), enabled="maybe")
        before = dict(py_trees.blackboard.Blackboard.storage)
        assert node.update() == Status.FAILURE, "配置错误要 FAILURE，不是默默成功"
    assert "enabled" in node.feedback_message, node.feedback_message
    assert repr("maybe") in node.feedback_message, node.feedback_message  # 值要点出来
    assert "enabled" in caplog.text and repr("maybe") in caplog.text, caplog.text
    assert dict(py_trees.blackboard.Blackboard.storage) == before, "坏 enabled 写了黑板"
    print("    enabled='maybe'：按关掉处理 + FAILURE + 日志点名实际值")


# --- 坏输入：能解析、但字段不对（工具用 write_text 非原子写盘，这几种都会真发生） ---
def _bad_truncated(path):
    """写盘写到一半：`json.JSONDecodeError`（`ValueError` 的子类）。"""
    text = json.dumps(_sim_payload(_box_dict()), ensure_ascii=False)
    path.write_text(text[: len(text) // 2], encoding="utf-8")
    return path


def _bad_not_an_object(path):
    return _write_json(path, [1, 2, 3])


def _bad_no_pallet_pose(path):
    payload = _sim_payload(_box_dict())
    payload.pop("pallet_pose")
    return _write_json(path, payload)


def _bad_short_pallet_pose(path):
    payload = _sim_payload(_box_dict())
    payload["pallet_pose"] = [0.0, 0.0, 1.0]
    return _write_json(path, payload)


def _bad_no_box(path):
    """`box` 整条漏掉 —— 列表形状只能手写，而工具永远只写单个对象。"""
    payload = _sim_payload(_box_dict())
    payload.pop("box")
    return _write_json(path, payload)


def _bad_null_box(path):
    """`box: null` —— 第一版会变成 `[None]` 再在 `raw["u1"]` 上 TypeError。"""
    payload = _sim_payload(_box_dict())
    payload["box"] = None
    return _write_json(path, payload)


def _bad_box_entry_not_an_object(path):
    return _write_json(path, _sim_payload([None]))


def _bad_box_entry_missing_u1(path):
    entry = _box_dict()
    entry.pop("u1")
    return _write_json(path, _sim_payload([entry]))


def _bad_box_entry_not_a_number(path):
    entry = _box_dict()
    entry["u1"] = "一百三十七"
    return _write_json(path, _sim_payload([entry]))


_BAD_INPUTS = [
    ("截断的 JSON（非原子写盘写到一半）", _bad_truncated, "JSON"),
    ("顶层不是对象", _bad_not_an_object, "对象"),
    ("没有 pallet_pose", _bad_no_pallet_pose, "pallet_pose"),
    ("pallet_pose 只有 3 个数", _bad_short_pallet_pose, "pallet_pose"),
    ("没有 box 字段", _bad_no_box, "box"),
    ("box 是 null", _bad_null_box, "box"),
    ("box[0] 不是对象", _bad_box_entry_not_an_object, "box[0]"),
    ("box[0] 缺 u1", _bad_box_entry_missing_u1, "u1"),
    ("box[0] 的 u1 不是数字", _bad_box_entry_not_a_number, "u1"),
]


@pytest.mark.parametrize("writer,needle",
                         [(w, n) for _, w, n in _BAD_INPUTS],
                         ids=[case for case, _, _ in _BAD_INPUTS])
def test_inject_bad_payload_is_a_failure_not_an_exception(tmp_path, caplog, writer, needle):
    """字段不对的输入**只许 FAILURE**，绝不许把异常抛穿行为树。

    py_trees 的 `Behaviour.tick()` **不接** `initialise()` / `update()` 抛出的
    异常：抛出去就是整棵树（连同伺服节点每帧的日志）一起没，比 FAILURE 糟得多。
    消息里必须**点名是哪个字段**，否则现场只看到"失败了"。
    """
    with caplog.at_level(logging.ERROR):
        node = _make_inject(writer(tmp_path / "sim.json"))   # initialise 里就该拦下
    # 起点：只有构造时预置的那四个键 —— 坏输入连"写了一半"都不许有
    preset = {"/latest_pallet": None, "/latest_pallet_version": 0,
              "/latest_box_obs": None, "/latest_box_obs_version": 0}
    assert dict(py_trees.blackboard.Blackboard.storage) == preset

    assert node.update() == Status.FAILURE
    assert needle in node.feedback_message, node.feedback_message
    # feedback 与 logger.error 各说一遍：树上只看得到 feedback、现场只看得到日志
    assert needle in caplog.text, caplog.text
    assert dict(py_trees.blackboard.Blackboard.storage) == preset, "坏输入写了黑板"
    print(f"    {needle!r} 点名在 feedback 与日志里，黑板一个键都没写")


def test_inject_bad_input_does_not_shout_every_tick(tmp_path, caplog):
    """坏输入不许按 tick 刷屏：**原文没变就只喊一次**，变了才再喊。

    坏输入 → FAILURE → py_trees 每 tick 重进 `initialise()`，同一份坏文件于是按
    tick 频率重读、重读一次喊一次（跑到 50 Hz 时现场只剩这一条）。伺服节点用
    `_log_throttled` 解决同一个问题；这里按原文比对（消息里带着路径、字段名和
    异常原文，所以"真变了"自然会再喊）。读文件本身**不省**：文件修好之后必须
    能自己恢复，那正是重进 `initialise()` 的唯一好处。
    """
    path = tmp_path / "sim.json"
    _bad_no_box(path)
    with caplog.at_level(logging.ERROR):
        node = _make_inject(str(path))
        for _ in range(5):
            node.tick_once()
        assert node.status == Status.FAILURE, node.feedback_message
        assert caplog.text.count("没有 `box` 字段") == 1, caplog.text

        _bad_null_box(path)                      # 输入真变了：另一种坏法
        for _ in range(3):
            node.tick_once()
        assert caplog.text.count("没有 `box` 字段") == 1, caplog.text
        assert caplog.text.count("`box` 是 null") == 1, caplog.text
    print("    坏输入：ERROR 只在原文变了时喊，不再按 tick 刷屏")


def test_inject_version_keeps_climbing_across_reentry(tmp_path):
    """重进 `initialise()`（行为树重跑这一段）不许把版本号打回 0。

    真生产者 `NodePalletPose.update` 是"读黑板上的当前值 + 1"，版本号单调。这里
    若在 `initialise()` 里归零，重播就会写出 3 → 1 的**回退** —— 今天的消费侧是
    等式门禁所以看不出，换成单调门禁就会把回退当成"陈旧数据"永远跳过。
    """
    node = make_inject(tmp_path, _box_dict())
    bb = node.global_blackboard
    for i in range(3):
        node.initialise()
        assert node.update() == Status.RUNNING
        assert getattr(bb, "latest_pallet_version") == i + 1
        assert getattr(bb, "latest_box_obs_version") == i + 1
    print("    重进 initialise()：版本 1 → 2 → 3，不掉回 1")


def test_inject_version_continues_from_the_blackboard(tmp_path):
    """黑板上已有别的生产者写的版本号时，从它接着涨 —— 与真生产者同一个口径。

    真跑时 `NodePalletPose` / YOLO 适配器可能先写过（本节点是后接上来的那个），
    发出去的版本号不许比黑板上的小。
    """
    node = make_inject(tmp_path, _box_dict())
    bb = node.global_blackboard
    setattr(bb, "latest_pallet_version", 7)        # 就当 NodePalletPose 写过 7 次
    node.initialise()
    assert node.update() == Status.RUNNING
    assert getattr(bb, "latest_pallet_version") == 8
    assert getattr(bb, "latest_box_obs_version") == 1, "没被写过的键从 0 起步"
    print("    黑板上 v7 → 本节点接着写 v8")


def test_inject_version_reads_a_missing_key_as_zero(tmp_path):
    """版本键被清掉（谁 `unset` 了 / 场景刚清过黑板）也不许抛 `KeyError` ——
    与伺服节点 `node_pallet_servo._read_blackboard` 同一个兜底。"""
    node = make_inject(tmp_path, _box_dict())
    bb = node.global_blackboard
    py_trees.blackboard.Blackboard.unset("/latest_pallet_version")
    py_trees.blackboard.Blackboard.unset("/latest_box_obs_version")
    node.initialise()
    assert node.update() == Status.RUNNING
    assert getattr(bb, "latest_pallet_version") == 1
    assert getattr(bb, "latest_box_obs_version") == 1
    print("    版本键不在黑板上：当 0，写回 1")


def test_inject_init_log_shows_the_optics_to_compare_against(tmp_path, caplog):
    """init 日志要把 JSON 里的 `K` / `D` / `image_size` / `use_distortion` 打出来。

    本节点是那份 JSON 的**唯一读者**，而工具写 `use_distortion` 就是为了让下游拿
    它对参数（工具里那句"别两边默默分叉"）。伺服节点报的是自己 params 里的 K/D：
    两边的数不一致必须**在日志里看得见**，不能悄悄算错。
    """
    with caplog.at_level(logging.INFO):
        make_inject(tmp_path, _box_dict())
    text = caplog.text
    assert str(K_PARAM) in text, text            # 内参要原值，不是"已配置"
    assert "D=None" in text, text
    assert str(IMAGE_SIZE) in text, text         # 图尺寸也要原值
    assert "use_distortion=True" in text, text
    assert "NodePalletServo" in text, text       # 说清是跟谁对
    print("    init 日志带 K/D/image_size/use_distortion，并点名和谁对")


# --------------------------------------------------------------------------- #
# 内参来源：**实时的相机话题优先，场景里写死的只是兜底**
# --------------------------------------------------------------------------- #
# 这段替换掉旧的 `_FakeCameraInfo` / `_patch_hardware` / `_FakeHw` —— 它们补的是
# `get_shared_hardware()`，而那条路**已经删掉了**（它会 `rospy.init_node()`，在没有
# master 的 CI runner 上无限重试，把 job 拖到 1 小时超时）。
# 现在只读一条话题，所以只需要换掉 `_read_camera_info()`。
def _no_camera(monkeypatch, info=None):
    """把读相机话题换掉。`info` 是 `(K, D, size, 原因)`，默认"读不到"。"""
    monkeypatch.setattr(NodePalletServo, "_read_camera_info",
                        lambda self: info or (None, None, None, "测试里没给相机"))


def _camera_info_msg(fx, fy, cx, cy, w, h, dist=None):
    """一份 `sensor_msgs/CameraInfo` 的样子（K 是 9 个数的平铺）。"""
    return SimpleNamespace(K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
                           D=list(dist) if dist else [],
                           width=w, height=h)


def _patch_camera_msg(monkeypatch, msg):
    """换掉 `rospy.wait_for_message`，让 `_read_camera_info` 走**真解析**。"""
    import sys
    import types
    fake_rospy = types.ModuleType("rospy")
    fake_rospy.wait_for_message = lambda topic, typ, timeout=None: msg
    fake_sensor = types.ModuleType("sensor_msgs.msg")
    fake_sensor.CameraInfo = object
    fake_msgs = types.ModuleType("sensor_msgs")
    fake_msgs.msg = fake_sensor
    monkeypatch.setitem(sys.modules, "rospy", fake_rospy)
    monkeypatch.setitem(sys.modules, "sensor_msgs", fake_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", fake_sensor)


def test_camera_intrinsics_win_over_the_scene_params(tmp_path, monkeypatch):
    """★ **实时的相机话题优先，场景里写死的 K 只是兜底。**

    写死的值**会过期**：换分辨率、换机器人、换相机型号，场景 JSON 里的 K 不会
    跟着变，而那是**静默**的错（三个量照算，只是全偏）。反过来"相机没起来"是
    看得见的，所以那时候才轮到写死值。

    抓帧那条路一直是读话题的（`pick_servo_inputs.py:482` → `meta.json`），本节点
    以前要人手抄 —— 这两条路没有理由不一样。
    """
    _patch_camera_msg(monkeypatch, _camera_info_msg(111.0, 222.0, 333.0, 444.0,
                                                    640, 480))
    # **场景里不写 K**，否则按"显式钉死"处理、根本不去读话题
    node = make_node(tmp_path, K=None)
    assert node._K[0, 0] == 111.0 and node._K[1, 1] == 222.0, node._K
    assert node._K[0, 2] == 333.0 and node._K[1, 2] == 444.0, node._K
    assert node._image_size == [640, 480], node._image_size
    assert node.update() == Status.RUNNING, node.feedback_message
    print(f"    话题优先：K 用的是相机报的 {node._K[0,0]:g}/{node._K[1,1]:g}，"
          f"图尺寸 {node._image_size}")


def test_a_scene_K_pins_the_intrinsics_without_touching_the_topic(tmp_path,
                                                                  monkeypatch):
    """★ **场景里写了 `K` 就一个话题都不碰** —— 这条同时钉住 CI 不会挂。

    两个理由：① 确定性 —— CI 上 rosmaster 是活的、相机在跑，若"给了 K 还去读
    话题"，同一个断言在本地（没相机）和 CI（有相机）结果就不同；② 显式即钉死，
    写 K 的人是在说"就用这个"。

    **测试替身会直接炸**：这里把 `_read_camera_info` 换成"一被调用就断言失败"，
    所以只要实现里漏了这道判断，这条立刻红 —— 而 CI 上它会红成"挂住 1 小时"。
    """
    def _must_not_be_called(self):
        raise AssertionError("场景里写了 K，却还是去读了相机话题")
    monkeypatch.setattr(NodePalletServo, "_read_camera_info", _must_not_be_called)

    node = make_node(tmp_path)        # make_node 默认给 K
    assert node._K[0, 0] == FX, node._K
    assert "场景参数" in node._intrinsics_src, node._intrinsics_src
    assert node.update() == Status.RUNNING, node.feedback_message
    print(f"    写了 K → 没碰话题，来源={node._intrinsics_src}")


def test_the_source_of_the_intrinsics_is_always_reported(tmp_path, monkeypatch):
    """来源必须写进日志 —— 现场第一件要核对的就是"这一版跑的到底是哪个 K"。"""
    _patch_camera_msg(monkeypatch, _camera_info_msg(111.0, 222.0, 333.0, 444.0,
                                                    640, 480))
    node = make_node(tmp_path, K=None)
    assert "camera_info" in node._intrinsics_src, node._intrinsics_src

    # 反面：读不到话题 → 退回参数，来源也要如实说
    _no_camera(monkeypatch)
    fallback = make_node(tmp_path)     # 这次给 K
    assert fallback._K[0, 0] == FX, fallback._K
    assert "场景参数" in fallback._intrinsics_src, fallback._intrinsics_src
    print(f"    来源如实记录：{node._intrinsics_src} / {fallback._intrinsics_src}")


def test_camera_distortion_and_size_are_taken_too(tmp_path, monkeypatch):
    """相机给畸变与图尺寸时一并采信 —— 三样都在同一份 camera_info 里。"""
    _patch_camera_msg(monkeypatch, _camera_info_msg(
        1000.0, 1000.0, 640.0, 400.0, 1280, 800,
        dist=[0.1, -0.2, 0.001, 0.002, 0.0]))
    node = make_node(tmp_path, K=None)
    assert node._D is not None, node._D
    assert list(np.round(node._D, 3)) == [0.1, -0.2, 0.001, 0.002, 0.0], node._D
    assert node._image_size == [1280, 800], node._image_size
    print("    畸变系数与图尺寸也来自话题")


def test_params_are_used_when_the_camera_is_not_there(tmp_path, monkeypatch):
    """读不到话题时**必须**还能用场景参数跑 —— 离线回放与单测都靠这条。"""
    _no_camera(monkeypatch)
    node = make_node(tmp_path, K=None, image_size=None)   # 只给 K 也够
    node2 = make_node(tmp_path)                           # 全给
    assert node2.update() == Status.RUNNING, node2.feedback_message
    assert node2._K[0, 0] == FX, node2._K
    assert list(node2._image_size) == list(IMAGE_SIZE), node2._image_size
    print("    相机不在 → 退回场景参数，照跑")


def test_missing_K_without_a_camera_names_both_ways_out(tmp_path, monkeypatch):
    """两边都没有时的报错必须**同时给出两条出路**，否则部署的人不知道下一步。"""
    _no_camera(monkeypatch)
    node = make_node(tmp_path, K=None)
    assert node.update() == Status.FAILURE
    msg = node.feedback_message
    assert "camera_info" in msg and "K" in msg, msg
    assert "①" in msg and "②" in msg, f"两条出路要点名：{msg}"
    print("    两边都没有 → 报错同时给出起相机 / 给 K 两条出路")


def test_a_degenerate_K_from_the_camera_is_not_silently_replaced(tmp_path,
                                                                 monkeypatch):
    """★ 相机报了个退化的 K（fx=0）时，**不许静默退回场景参数**。

    退回会把一个真问题（相机没标定）盖掉，而三个量照算不误 —— 正是本项目最忌
    的那类静默错。"没数据"和"数据是坏的"是两件事：前者可以退回，后者必须报。
    """
    _patch_camera_msg(monkeypatch, _camera_info_msg(0.0, 0.0, 0.0, 0.0, 640, 480))
    node = make_node(tmp_path, K=None)
    assert node.update() == Status.FAILURE, "相机报了退化 K 却没有报错"
    assert "退化" in node.feedback_message, node.feedback_message
    print("    相机报退化 K → 报错，不静默退回写死值")


def test_the_camera_topic_is_read_at_most_once_per_node(tmp_path, monkeypatch):
    """★ **成功读一次就够**：给了 `K` 时一次都不问；没给时最多问一次。

    py_trees 的 `Behaviour.tick()` 在状态不是 RUNNING 时**每 tick 重进
    `initialise()`**。没有缓存时，"读不到相机"这条路上每个 tick 都要等满
    `CAMERA_INFO_TIMEOUT_SEC` —— 50 Hz 的 tick 配 0.5 秒的等待，现场彻底卡死。

    ⚠️ **失败那条路的策略后来改过**（见
    `test_a_camera_that_is_not_up_yet_is_retried_not_stuck_forever`）：失败不再
    "永久缓存"，而是**按 `CAMERA_INFO_RETRY_SEC` 节流地重试**（否则相机晚起
    就是永久 FAILURE）。本用例走的是"给了 K"这一支，一个话题都不碰。
    """
    calls = []

    def _counting(self):
        calls.append(1)
        return None, None, None, "测试里没给相机"
    monkeypatch.setattr(NodePalletServo, "_read_camera_info", _counting)

    node = make_node(tmp_path)                    # 给了 K，但构造里也会问一次吗？
    node.initialise()
    node.initialise()
    node.initialise()
    # 给了 K 时按"显式钉死"处理，**一次都不该问**（这条比从前的 `<= 1` 更紧）
    assert len(calls) == 0, f"写了 K 还是去读了 {len(calls)} 次相机话题"
    print(f"    写了 K：一次都不问（实际 {len(calls)} 次）")


def test_a_camera_that_is_not_up_yet_is_retried_not_stuck_forever(tmp_path,
                                                                   monkeypatch):
    """★ `_intrinsics_err`（相机启动时还没起来）**必须能自愈**，不是永久 FAILURE。

    两处一起才堵死的那条死路（这一轮修的）：

      * `_camera_info_once()` 从前连**失败**都永久缓存（"每实例只读一次"）；
      * `initialise()` 还把 `_intrinsics_err` **折进 `_config_err`** —— 而
        `_config_err` 一旦写上就再也不会被清掉。

    于是"相机在树启动那一刻还没起来"（真机上很常见）⇒ 节点**永久 FAILURE**、
    每 tick 一条 ERROR（10 Hz ≈ 600 条/分钟）、**不可能自愈**，只能重启整棵树。
    `_resolve_size()` 那条路早就保留着"重进就重读"以便"配置修好即自愈"，两处
    不该不一致。

    ⚠️ 重试**必须节流**：读一次要等满 `CAMERA_INFO_TIMEOUT_SEC`（0.5s），
    每个 tick 都试就把 tick 拖成 0.5s 的卡死 —— 那正是当初加缓存要解决的事。
    """
    state = {"ok": False, "calls": 0}

    def _read(self):
        state["calls"] += 1
        if not state["ok"]:
            return None, None, None, "测试里没给相机"
        return (np.array([[1000.0, 0.0, 640.0],
                          [0.0, 1000.0, 400.0],
                          [0.0, 0.0, 1.0]]), None, [1280, 800], "")
    monkeypatch.setattr(NodePalletServo, "_read_camera_info", _read)

    node = make_node(tmp_path, K=None, image_size=None)   # `make_node` 里跑过一次
    assert state["calls"] == 1, state
    assert node.update() == Status.FAILURE
    assert "camera_info" in node.feedback_message, node.feedback_message

    # 立刻重进（py_trees 每 tick 就是这么干的）：节流之内，不该再问
    node.initialise()
    assert state["calls"] == 1, (
        f"节流没生效：一个 tick 就问一次，每次 0.5s 的等待会把现场卡死"
        f"（实际问了 {state['calls']} 次）")

    # 相机起来了（等价于"`CAMERA_INFO_RETRY_SEC` 过去了"）
    state["ok"] = True
    node._camera_info_retry_after = 0.0
    node.initialise()
    assert state["calls"] == 2, state
    assert node._intrinsics_err == "", node._intrinsics_err
    assert node._K[0, 0] == 1000.0, node._K
    assert node.update() == Status.RUNNING, node.feedback_message
    print("    相机晚起 → 按节流重试并自愈（不再是永久 FAILURE）")


# --------------------------------------------------------------------------- #
# pallet_frame：坐标系参数（新默认 = camera）
# --------------------------------------------------------------------------- #
def test_pallet_frame_defaults_to_camera_and_skips_tf():
    """★ 新默认：`pallet_frame="camera"` → 不构造 TF 解析器、`T_cam_base` 是单位阵。

    这条是"少一个静默失败源"的落地：`TfCamBaseResolver` 在 TF 断掉时会**静默**
    退回单位阵（只有一条 WARNING），而 base 系的位姿被当成相机系直接投影，
    三个数看着像模像样。声明成 `"camera"` 之后这条回退路径根本不存在。
    """
    node = NodePalletServo("servo", "servo", "ns", {
        "ref_edges": ["y=0", "x=W"],
        "pallet_size_mm": [1200.0, 1000.0],
        "K": [[1000.0, 0.0, 640.0], [0.0, 1000.0, 400.0], [0.0, 0.0, 1.0]],
        "image_size": [1280, 800],
    })
    assert node._pallet_frame == "camera", node._pallet_frame
    node.initialise()
    assert node._resolver is None, \
        "pallet_frame=camera 时不该构造 TF 解析器（少一个静默失败源）"


def test_pallet_frame_base_link_keeps_the_old_behaviour():
    """`pallet_frame="base_link"` 时行为与改动前**逐位一致**。

    这条是老路径（`NodePalletPose` 写 base_link 系位姿）的回归网。
    """
    node = NodePalletServo("servo", "servo", "ns", {
        "pallet_frame": "base_link",
        "ref_edges": ["y=0", "x=W"],
        "pallet_size_mm": [1200.0, 1000.0],
        "K": [[1000.0, 0.0, 640.0], [0.0, 1000.0, 400.0], [0.0, 0.0, 1.0]],
        "image_size": [1280, 800],
    })
    assert node._pallet_frame == "base_link"
    node.initialise()
    assert node._resolver is not None, \
        "pallet_frame=base_link 时必须构造 TF 解析器"


def test_an_unknown_pallet_frame_falls_back_to_camera_with_a_warning(caplog):
    """认不出来的值退回**默认值 camera** 并点名报出实际值。

    退回的方向是安全的：`camera` 不查 TF、没有静默回退路径。反过来退回
    `base_link` 会凭空多一条"TF 断了就静默用单位阵"的路。
    """
    with caplog.at_level(logging.WARNING):
        node = NodePalletServo("servo", "servo", "ns", {
            "pallet_frame": "camera_link",          # 拼错了
            "ref_edges": ["y=0", "x=W"],
            "pallet_size_mm": [1200.0, 1000.0],
            "K": [[1000.0, 0.0, 640.0], [0.0, 1000.0, 400.0], [0.0, 0.0, 1.0]],
            "image_size": [1280, 800],
        })
    assert node._pallet_frame == "camera"
    # **点名报出实际值**：认错的坐标系会让参考边整体偏掉而三个数照样算得出来，
    # 一条不说话的 WARNING 等于没有
    assert "camera_link" in caplog.text, caplog.text
    assert "pallet_frame" in caplog.text, caplog.text


def test_camera_pallet_frame_reports_a_declared_identity_not_a_silent_one():
    """`t_cam_base_src` 要写成"声明过的单位阵"，不是"静默回退的单位阵"。

    两者在图上和日志里必须区分得开 —— 后者是已知的坑。
    """
    node = NodePalletServo("servo", "servo", "ns", {
        "ref_edges": ["y=0", "x=W"],
        "pallet_size_mm": [1200.0, 1000.0],
        "K": [[1000.0, 0.0, 640.0], [0.0, 1000.0, 400.0], [0.0, 0.0, 1.0]],
        "image_size": [1280, 800],
    })
    node.initialise()
    T, src = node._resolve_t_cam_base()
    import numpy as np
    assert np.allclose(T, np.eye(4))
    assert src == "identity(相机系位姿)", src
    assert "identity" in src and "相机系" in src


def test_base_link_mode_outputs_without_any_stamp_key(tmp_path):
    """★ base_link 老路径 **绕开配对**：没有 `latest_pallet_stamp` 这个键也照样出数。

    addendum A10 的回归网。`NodePalletPose`（老路径的生产者）全文**没有任何
    `stamp`**，而 spec §7.1 要求这条路径与改动前**逐位一致** —— 配对在老路径上
    只会产出空（托盘侧没有可用的图像时刻），两者不可兼得，以回归要求为准。

    这条挡的是"两种模式都走配对"那个写法：那时本用例会**一个数都拿不到**
    （`pair_none`），而没有任何报错。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_box_obs", a_box())
    setattr(writer, "latest_box_obs_version", 1)
    # ⚠️ `latest_pallet_stamp` **一个键都不注册、不写**（老生产者的行为）

    node = make_node(tmp_path)                     # make_node 默认 base_link
    assert node._pallet_frame == "base_link", node._pallet_frame
    assert node.update() == Status.RUNNING, node.feedback_message
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None, (
        f"base_link 老路径一个数都没出（A10 的回归就断在这里）："
        f"{node.feedback_message}")
    assert out.t_cam_base_src == "identity", out.t_cam_base_src

    # 三个量必须与"直接调算法"逐位一致（老路径没有配对，输入就是黑板上的原样）
    direct = servo_error(straight_down_camera(),
                         (PALLET_W_MM, PALLET_H_MM), ("y=0", "x=W"), a_box(),
                         np.array(K_PARAM, np.float64), None, IMAGE_SIZE)
    assert abs(out.e_bottom_px - direct.e_bottom_px) < 1e-9, (out, direct)
    assert abs(out.e_right_px - direct.e_right_px) < 1e-9, (out, direct)
    assert abs(out.theta_rad - direct.theta_rad) < 1e-12, (out, direct)
    print(f"    base_link 无 stamp 键照样出数：{out.to_log_line()}")


def test_camera_mode_pairs_by_image_stamp_and_declares_the_identity(tmp_path):
    """★ 生产默认（不给 `pallet_frame` = camera）走的是**配对**那条路。

    托盘与箱子**同 stamp** → 配得上、出数；`t_cam_base_src` 写的是**声明过的**
    单位阵。两个检测器耗时差一个量级也没关系 —— 配对的依据是**输入图像的采集
    时刻**，不是处理完成时刻（见 `pallet_frame/algorithm.py` 的模块 docstring）。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_pallet_stamp", 1.0)
    setattr(writer, "latest_box_obs", a_box(stamp=1.0))
    setattr(writer, "latest_box_obs_version", 1)

    node = make_node(tmp_path, pallet_frame="camera")
    assert node._resolver is None, "camera 模式不该有 TF 解析器"
    assert node.update() == Status.RUNNING, node.feedback_message
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None, node.feedback_message
    assert abs(out.e_bottom_px + 137.0) < 1e-9, out.e_bottom_px
    assert out.t_cam_base_src == "identity(相机系位姿)", out.t_cam_base_src
    print(f"    camera 默认路径出数：{out.to_log_line()}")


def test_reinitialise_with_the_same_versions_still_outputs(tmp_path):
    """★ 重进 `initialise()` 后**同一个版本号**再喂一遍，仍要出数（F3）。

    `initialise()` 里那三行（`_pairing.reset()` + 两路版本号归零）的注释主张
    "**必须一起清**"，但以前只有"清窗"那半有网：真生产者写的版本号是 `read+1`
    **单调递增**的，重进之后新版本必然不等于旧版本，"版本没变 → 不 push"这条路
    在既有用例里**根本走不到**（去掉那两行版本号归零，全套仍然全绿）。

    **版本号会重复**的那类场景才走得到：bag 循环播放、生产者进程重启后重新从 1
    开始（而伺服上次只见过第 1 帧）。那时只清窗、不清版本号 → 两路观测**一个都
    推不进**配对窗，`resolve()` 在空窗上返回 `None` → **永久静默**，而且一个错
    都不报（与水位线不清是同一个后果，只是换了个地方）。

    ⚠️ 这条**同时**钉住"清窗"（stamp 从 5.0 退回 1.0 是整体回跳）与"清版本号"
    （版本号原样不动）两半：少了任一半，第二次都配不出数。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp",
                         "latest_box_obs", "latest_box_obs_version")

    def feed(stamp, version):
        setattr(writer, "latest_pallet",
                matrix_to_pose6d(straight_down_camera()))
        setattr(writer, "latest_pallet_version", version)
        setattr(writer, "latest_pallet_stamp", stamp)
        setattr(writer, "latest_box_obs", a_box(stamp=stamp))
        setattr(writer, "latest_box_obs_version", version)

    node = make_node(tmp_path, pallet_frame="camera")
    feed(5.0, 1)
    assert node.update() == Status.RUNNING, node.feedback_message
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 1
    assert getattr(node.global_blackboard, "latest_servo_error", None) is not None

    # 生产者重启 / bag 从头再放：**版本号原样重来**（还是 1），时间戳整体回跳
    node.initialise()
    feed(1.0, 1)
    assert node.update() == Status.RUNNING, node.feedback_message
    got = getattr(node.global_blackboard, "latest_servo_error_version", 0)
    assert got == 2, (
        f"重进 initialise() 之后同一个版本号推不进配对窗了（版本停在 {got}）—— "
        f"`_pairing.reset()` 清了水位线，但 `_last_pallet_version` / "
        f"`_last_box_version` 没跟着归零：版本没变就一个观测都不 push，"
        f"`resolve()` 在空窗上永久返回 None：{node.feedback_message}")


def test_a_pallet_stamp_of_zero_does_not_get_pushed(tmp_path, caplog):
    """★ `latest_pallet_stamp` 读到 `0.0` → **不 push** + 告警一次（addendum A7）。

    `0.0` 是**我们自己的** `NodePalletObs` 按契约写的（上游没填 `header.stamp`
    时它写原始值，不编造处理时刻）。这一帧必须**配不上**，而不是拿 tick 时刻去
    凑 —— 凑出来的时刻会让两个检测器的耗时差变成配对的时间偏差，配上一对不同帧
    的图像，而三个数照样算得出来。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_pallet_stamp", 0.0)      # 上游没填 header.stamp
    setattr(writer, "latest_box_obs", a_box(stamp=1.0))
    setattr(writer, "latest_box_obs_version", 1)

    node = make_node(tmp_path, pallet_frame="camera")
    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING
        for _ in range(3):
            assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None, \
        "stamp=0 的那一帧不该配出数（0.0 是「没填 header」的信号，不是时刻）"
    assert "检测器没填" in caplog.text, caplog.text
    assert caplog.text.count("检测器没填") == 1, \
        "一次性告警：不是每 tick 一遍"


def test_a_stamp_of_zero_is_not_pushed_even_when_the_clock_would_pair(
        tmp_path, caplog):
    """★ A7 第二支的**独立网**：`stamp=0` 的那一帧是**不 push**，不是"推了配不上"。

    ⚠️ 与上一条的分工（评审证伪了上一条的判别力）：上一条里箱子 stamp 是 `1.0`、
    tick 时刻是 `time.time()`，两路差着整个 epoch —— 就算把 A7 两支合并成"都退回
    tick 时刻、照常 push"，`pair_dt` 也会把这一对拒掉，**断言"不出数"被过度决定了**。
    评审的变异 M3b 正是这样：两支合并后**全套 266 条仍然全绿**，而实测那个变异版
    在"pallet stamp=0.0 + box stamp=time.time()"时**真的出了数**（`e_bottom=+137.0px`）
    —— 配上一对不同帧的图像、三个数照样算得出来，**正是 A7 明文要防的那件事**。

    所以这里让**阈值不再是借口**：`max_dt_s` 放宽到 1.0s，箱子用 `time.time()`
    （与"退回 tick 时刻"的那个时刻同源）—— 只要那一帧被 push 了，它就**必然配上**。
    断言"不出数"于是只可能来自**没 push**。

    ⚠️ 这条**不拿 `pair_dt` 解释**：它测的是"不 push"，与阈值无关（`pair_none`
    才是"托盘侧压根没有观测"的名字）。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_pallet_stamp", 0.0)      # 上游没填 header.stamp
    import time as _time
    # 箱子这一路是**健康的**：stamp 就是此刻（与"退回 tick 时刻"同源）
    setattr(writer, "latest_box_obs", a_box(stamp=_time.time()))
    setattr(writer, "latest_box_obs_version", 1)

    node = make_node(tmp_path, pallet_frame="camera", max_dt_s=1.0)
    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING
        assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None, \
        ("stamp=0 的那一帧被 push 了 —— 它拿 tick 时刻凑出一个时刻，配上了"
         "一对不同帧的图像，三个数照样算得出来（A7 要防的正是这个）")
    assert "pair_none" in node.feedback_message, (
        "配不上的原因该是'托盘侧没有观测'，不是 pair_dt："
        f"{node.feedback_message}")


def test_a_missing_pallet_stamp_key_still_pushes_with_the_tick_clock(tmp_path,
                                                                    caplog):
    """★ 反过来：**键压根没有**（老生产者）→ 退回 tick 时刻**照常 push**。

    两种"没有时间戳"的处置是**有意不同**的（addendum A7 改后）：
      * 键读不到 = 老生产者（`NodePalletPose` 那类契约里没有它）→ 退回 tick 时刻，
        否则整条老路径彻底哑掉，而且**没有任何报错**；
      * 键在、值是 `0.0` = 我们自己的检测器没填 `header.stamp` → 不 push。
    """
    # 黑板是**进程级**的，前面的用例早写过 `latest_pallet_stamp` 了 —— 这条要的
    # 恰恰是"这个键从来没有过"，所以先清空（与
    # `test_node_warns_once_when_the_version_key_was_never_written` 同一句先例）。
    py_trees.blackboard.Blackboard.clear()
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 1)
    # `latest_pallet_stamp` **不注册、不写**
    import time as _time
    setattr(writer, "latest_box_obs", a_box(stamp=_time.time()))
    setattr(writer, "latest_box_obs_version", 1)

    node = make_node(tmp_path, pallet_frame="camera",
                     max_dt_s=1.0)         # 退回的是 tick 时刻，差值在毫秒级
    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_servo_error", None)
    assert out is not None, f"老生产者这条路上一个数都没出：{node.feedback_message}"
    assert "latest_pallet_stamp" in caplog.text, caplog.text
    print(f"    没有 stamp 键 → 退回 tick 时刻，照样出数：{out.to_log_line()}")


def test_the_missing_stamp_warning_fires_again_after_reinitialise(tmp_path, caplog):
    """★ M7：`_stamp_warned` 必须在 `initialise()` 里**复位**。

    它从前是懒建属性（`getattr(self, "_stamp_warned", False)` 然后置 True），
    于是**只在本实例上响一次** —— 换场景 / 重树 / 重进 `initialise()` 之后，
    "上游没填 `header.stamp`"这条**再也不会响**，而它正是"两个检测器的耗时差
    变成配对的时间偏差、配上一对不同帧的图像"那类静默错的第一线索。

    与 `_last_live_warn` / `_last_*_seen` 同一处、同一口径（那几个早就复位了）。
    """
    # 黑板是**进程级**的，别的用例早写过 `latest_pallet_stamp` 了 —— 这条要的
    # 恰恰是"这个键从来没有过"，所以先清空（与上一条同款先例）。
    py_trees.blackboard.Blackboard.clear()
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_box_obs", a_box(stamp=1.0))
    setattr(writer, "latest_box_obs_version", 1)
    # `latest_pallet_stamp` **不注册、不写** → `_push_pallet` 走"老生产者"那一支

    import logging
    node = make_node(tmp_path, pallet_frame="camera", max_dt_s=1.0)
    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING
    assert "老生产者" in caplog.text, caplog.text

    # 换场景 / 重树：py_trees 会重新 `initialise()`
    caplog.clear()
    node.initialise()
    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING
    assert "老生产者" in caplog.text, (
        f"重进 initialise() 之后这条告警不再响了 —— 它又变成了一次性的静默："
        f"{caplog.text}")
    print("    重进 initialise() → 缺 stamp 的告警重新响")


def test_agg_misspelling_is_said_out_loud(tmp_path, caplog):
    """★ `agg` 的白名单规范化在**本层**（addendum A3）。

    `average_pairs` 对拼错的 `agg` **静默**退回 `mean`（纯函数、零状态、不 log，
    那是有意的）；但参数是从 ROS rosparam / 场景 JSON 进来的，可见性归节点层 ——
    不喊这一声，操作员会以为自己在用中位数。
    """
    with caplog.at_level(logging.WARNING):
        node = make_node(tmp_path, agg="meean")
    assert node._agg == "mean", node._agg
    assert "agg" in caplog.text and "meean" in caplog.text, caplog.text

    # 合法值一个字都不许吵；大小写不同的 "Mean" 也是**不认**的（白名单是精确匹配）
    caplog.clear()
    with caplog.at_level(logging.WARNING):
        assert make_node(tmp_path / "a", agg="median")._agg == "median"
        assert make_node(tmp_path / "b", agg="mean")._agg == "mean"
    assert "agg" not in caplog.text, caplog.text


def test_bad_pairing_numbers_fall_back_to_meaningful_defaults(tmp_path, caplog):
    """★ `max_dt_s` / `window` / `pair_cache` 的坏值**在参数层就说得出口**（A3）。

    不能靠"0 到了下层会被拒"来暴露配置缺失 —— 那要等到第一帧才发现，而
    `max_dt_s=NaN` 更糟：`dt > nan` 恒为 False，配对门禁**静默失效**且一个
    reject 都不报。
    """
    with caplog.at_level(logging.WARNING):
        node = make_node(tmp_path, max_dt_s=float("nan"), window="abc",
                         pair_cache=0)
    assert node._max_dt_s == 0.05, node._max_dt_s
    assert node._pairing._window == 5, node._pairing._window
    assert node._pairing._cache == 1, node._pairing._cache
    assert "max_dt_s" in caplog.text, caplog.text
    assert "window" in caplog.text, caplog.text
    assert "pair_cache" in caplog.text, caplog.text

    # 合法的取值一个字都不许吵（含合法的 0：`window: 0` 被钳成 1 = 不平滑）
    caplog.clear()
    with caplog.at_level(logging.WARNING):
        node = make_node(tmp_path / "ok", max_dt_s=0.033, window=0, pair_cache=3)
    assert node._max_dt_s == 0.033
    assert node._pairing._window == 1, node._pairing._window
    assert node._pairing._cache == 3
    assert "max_dt_s" not in caplog.text and "pair_cache" not in caplog.text


def test_bad_overlay_and_liveness_numbers_do_not_throw(tmp_path, caplog):
    """★ `overlay_period_s` / `live_timeout_s` 的坏值走**同一条纪律**（M-2 / M-3）。

    * `overlay_period_s` 早先用裸 `float()`：`"abc"` **在 `__init__` 里抛**，掀翻
      建树 —— 与本提交为 `agg`/`window`/`pair_cache`/`max_dt_s` 立的那条纪律
      正好相反（坏值一律不抛 + 点名 + 退默认）。
    * `live_timeout_s` 早先 `max(0.0, ...)`：`-1` 静默变成 0 = **关掉唯一的检测器
      存活告警**，而且不记 WARNING。0 本身是文档里写明的"不查"，照收不吵。
    """
    with caplog.at_level(logging.WARNING):
        node = make_node(tmp_path, overlay_period_s="abc", live_timeout_s=-1)
    assert node._overlay_period_s == 0.2, node._overlay_period_s
    assert node._live_timeout_s == 5.0, node._live_timeout_s
    assert "overlay_period_s" in caplog.text, caplog.text
    assert "live_timeout_s" in caplog.text, caplog.text

    # 合法值一个字都不许吵：`live_timeout_s: 0` = 不查（写明的关法），
    # `overlay_period_s: 0` = 不降频（每帧都发）
    caplog.clear()
    with caplog.at_level(logging.WARNING):
        node = make_node(tmp_path / "z", live_timeout_s=0, overlay_period_s=0.0)
    assert node._live_timeout_s == 0.0, node._live_timeout_s
    assert node._overlay_period_s == 0.0, node._overlay_period_s
    assert "live_timeout_s" not in caplog.text, caplog.text
    assert "overlay_period_s" not in caplog.text, caplog.text


def test_omitting_live_timeout_s_does_not_shout(tmp_path, caplog):
    """★ **没给** `live_timeout_s` ≠ 坏参数：一个字都不许吵（F1）。

    全仓**没有任何**配置写过这个键（`pallet_servo_real_v1/py_tree.json` 的
    `NodePalletServo` 配了 7 个键 —— `pallet_frame` / `ref_edges` / `pallet_size_mm` /
    `use_distortion` / `max_dt_s` / `window` / `overlay_out`，就是没有
    `live_timeout_s`），所以真机上每次建树走的都是这条路。上一轮把读数从
    `_float_or` 换成 `_nonnegative_float_param`
    时**丢了内联默认值**：`get()` 返回 `None`，助手走"不是数字"那一支 ——
    于是每次建树刷一条**假 WARNING**（数值仍对，但文案是假的），把 M-3 想立起来
    的"坏值点名"淹进同一类噪声：真写了个 `-1` 时，操作员看到的是同一种东西。

    ⚠️ 与上面的 `test_bad_overlay_and_liveness_numbers_do_not_throw` 分工：
    那条钉"**坏了**要点名"（`-1` / `"abc"`），这条钉"**没给**不许吵"。两条合起来
    才是完整口径 —— "不抛 + 点名 + 退默认"里的"点名"只对**坏值**。

    ⚠️ 用 `pallet_frame="camera"`：`base_link` 那条会去起 TF 监听，CI 上没有
    roscore，于是**环境**先喊一条 "起不了 TF 监听"（与本条无关），断言就失去
    判别力了（V1 变异也会"红"）。
    """
    with caplog.at_level(logging.WARNING):
        node = make_node(tmp_path, pallet_frame="camera")   # 参数表里**没有**这个键
    assert node._live_timeout_s == 5.0, node._live_timeout_s
    shouted = [r.getMessage() for r in caplog.records
               if r.levelno >= logging.WARNING]
    assert not shouted, (
        f"没传 live_timeout_s 却吵了 {len(shouted)} 条：{shouted} —— "
        f"「没给」不是坏参数；缺省被当成坏值，真坏值就淹没在噪声里了")


def test_the_pairing_reject_dump_says_which_versions_missed(tmp_path, caplog):
    """★ 配对失败的 dump 里要写**是哪两个版本配不上**（Minor M-4）。

    早先传的是占位的 `(0, 0)`：复现"配不上"时最有用的两个数被丢掉了 —— 现场只
    看得到"一个 reject"，看不到"是第 7 帧的托盘对第 9 帧的箱子"。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 7)          # 都不是 0：与占位值分得开
    setattr(writer, "latest_pallet_stamp", 1.0)
    setattr(writer, "latest_box_obs", a_box(stamp=1.5))   # 差 0.5s > max_dt_s=0.05
    setattr(writer, "latest_box_obs_version", 9)

    node = make_node(tmp_path, pallet_frame="camera")
    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING
    assert "pair_dt" in node.feedback_message, node.feedback_message
    dumps = sorted((tmp_path / "dump").glob("*.json"))
    assert len(dumps) == 1, dumps
    payload = json.loads(dumps[0].read_text(encoding="utf-8"))
    assert payload["pallet_version"] == 7, payload["pallet_version"]
    assert payload["box_version"] == 9, payload["box_version"]


def test_camera_mode_survives_upstream_objects_of_the_wrong_type(tmp_path,
                                                                 caplog):
    """★ camera 模式下上游给了**别的类型** → 不出数，但**绝不抛穿 update()**。

    配对那一段是新加的，而它解的是 `pose6d_to_matrix(pose)` 与 `box.stamp` ——
    这两个调用都在 `update()` 里、**不在技能层的 `try` 里**（技能层只包
    `on_execute`）。坏类型抛出去就是**整棵树连每帧的日志一起没**。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", [0.0] * 6)          # 不是 Pose6D
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_pallet_stamp", 1.0)
    setattr(writer, "latest_box_obs", {"u1": 1.0, "v1": 2.0})   # 不是 BoxObservation
    setattr(writer, "latest_box_obs_version", 1)

    node = make_node(tmp_path, pallet_frame="camera")
    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING
        assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None
    assert "推不进配对窗" in caplog.text, caplog.text
    assert caplog.text.count("推不进配对窗") == 2, (
        "两条 WARNING 各说各的（托盘一次、箱子一次），且都按原文去重")


def test_a_stalled_detector_is_shouted_about(tmp_path, caplog):
    """★ 存活闸（addendum A4）：某一侧不再有新观测 → 一次性 WARNING。

    `resolve()` 返回 `None` 有**两种含义且不可区分**（正常去重 / 某路停摆），
    所以节点必须自己判：它知道这一 tick 有没有往某一侧推过东西。没有这条，
    "托盘检测挂了"就只是**静默**的 `RUNNING`，黑板停在旧值上，操作员一无所知。
    """
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp",
                         "latest_box_obs", "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_pallet_stamp", 1.0)
    setattr(writer, "latest_box_obs", a_box(stamp=1.0))
    setattr(writer, "latest_box_obs_version", 1)

    node = make_node(tmp_path, pallet_frame="camera", live_timeout_s=0.05)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_servo_error", None) is not None

    # 两路都不再出新观测（版本号一个都不动），时钟往前走。
    # ⚠️ **每 tick 都要推进 `mono`**（这里用"把 seen 往回拨"等效地做到）：真机上
    # 10 Hz、`worst` 每 tick 长 0.1s，文案里的 `{worst:.1f}s` 于是**每 tick 都变**
    # —— 如果去重按整条文案比，10 个 tick 就是 10 条 WARNING。同一微秒内的两次
    # tick 会让 `{:.1f}` 输出同一个数，**测不出**这个问题（评审判的就是它）。
    node._last_pallet_seen -= 1.0
    node._last_box_seen -= 1.0
    with caplog.at_level(logging.WARNING):
        for _ in range(10):                      # 1 秒 × 10 Hz
            assert node.update() == Status.RUNNING
            node._last_pallet_seen -= 0.1
            node._last_box_seen -= 0.1
    assert "停摆" in caplog.text, caplog.text
    assert caplog.text.count("检测器停摆？") == 1, (
        f"停摆告警被刷了 {caplog.text.count('检测器停摆？')} 遍 —— 去重键里带了"
        f"每 tick 都在变的时长（10 Hz 下每 tick 一条长文案）")
    assert "停摆" in node.feedback_message, node.feedback_message
    # feedback 里**照旧带当前时长**：去重只压日志，不压操作员看到的那一行
    assert "已经" in node.feedback_message and "s 没有" in node.feedback_message, \
        node.feedback_message
    # ⚠️ `stale_s` 顶不上这个用：它是**延迟闸**，在这里是 0（不查）
    assert node._stale_s == 0.0

    # 恢复：有新观测进来 → 黑板重新出数（告警槽位也要复位）
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 2)
    setattr(writer, "latest_pallet_stamp", 2.0)
    setattr(writer, "latest_box_obs", a_box(stamp=2.0))
    setattr(writer, "latest_box_obs_version", 2)
    assert node.update() == Status.RUNNING
    assert node._last_live_warn is None, "恢复之后告警槽位该复位"


# --------------------------------------------------------------------------- #
# 可视化：回调只存帧、update() 里画、发图前先自检
# --------------------------------------------------------------------------- #
def _fake_color_msg(width, height, step=None, fill=7):
    """一份 `sensor_msgs/Image` 的样子（`step` 给大就能造出行末填充）。"""
    step = width * 3 if step is None else step
    data = np.full((height, step), fill, np.uint8).tobytes()
    return SimpleNamespace(height=height, width=width, step=step, data=data)


class _FakePublisher:
    def __init__(self):
        self.n = 0
        self.msg = None

    def publish(self, msg):
        self.n += 1
        self.msg = msg


def _fake_ros_modules(monkeypatch):
    """假的 `rospy` / `sensor_msgs.msg`：只为让**发图那一步真的走完**。

    不碰真 ROS（`rospy.Time.now()` 在没 `init_node()` 时是抛的，那会把这条用例
    变成"永远发不出去"，而不是"包装对不对"）。
    """
    import sys
    import types

    class _Image:
        def __init__(self):
            self.header = types.SimpleNamespace()

    fake_rospy = types.ModuleType("rospy")
    # ⚠️ `now` 一个字都不许动：发图那几条既有用例依赖它返回 1.0。
    # `from_sec` 是伺服误差话题那条路要用的（`_publish_error` 用它把配对的图像
    # 采集时刻装进 `header.stamp`）。
    fake_rospy.Time = SimpleNamespace(
        now=lambda: SimpleNamespace(to_sec=lambda: 1.0),
        from_sec=lambda s: SimpleNamespace(to_sec=lambda: float(s)))
    # `core.is_initialized()` 是"这个进程有没有 ROS"的唯一判据 ——
    # **真模块里怎么写的就照抄什么**（`rospy/core.py` 里就是个模块级函数）。
    # 假成"没起过"是**有意**的：本文件里除了那两条专门测守卫的用例，走的都是
    # 离线 / 单测这条现实路径（`NodePalletObs.initialise()` 用的是同一句判据）。
    # 不设它的话 `_ros_node_initialized()` 会把"判不出来"当成"没有 ROS"——
    # 结论一样，但这条用例就少了判别力。
    fake_rospy.core = SimpleNamespace(is_initialized=lambda: False)
    fake_sensor = types.ModuleType("sensor_msgs.msg")
    fake_sensor.Image = _Image
    fake_msgs = types.ModuleType("sensor_msgs")
    fake_msgs.msg = fake_sensor
    monkeypatch.setitem(sys.modules, "rospy", fake_rospy)
    monkeypatch.setitem(sys.modules, "sensor_msgs", fake_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", fake_sensor)


def test_the_color_callback_only_stores_the_frame(tmp_path, caplog):
    """★ 回调**只存最新帧，绝不画图**（画图几十毫秒，会拖垮回调队列）。

    顺带钉住行末填充的处理（`step` ≥ `width*3`）与"装不下就丢掉这一帧"。
    """
    node = make_node(tmp_path, overlay_out="")
    node._on_color(_fake_color_msg(8, 4, step=8 * 3 + 4))      # 行末 4 字节填充
    with node._overlay_lock:
        got = node._last_color
    assert got is not None and got.shape == (4, 8, 3), got.shape
    assert np.all(got == 7), got

    # 一行装不下 width*3 → **不抛**，只记一次警告，上一帧原样留着
    with caplog.at_level(logging.WARNING):
        node._on_color(_fake_color_msg(8, 4, step=8))
    with node._overlay_lock:
        assert node._last_color is got
    assert "彩色帧解不开" in caplog.text, caplog.text

    # 没有底图时**不出图**，但要计数（不是静默）
    node._overlay_pub = _FakePublisher()
    with node._overlay_lock:
        node._last_color = None
    node._maybe_publish_overlay(SimpleNamespace(), 1)
    assert node._overlay_pub.n == 0
    assert node._n_overlay_skip == 1, node._n_overlay_skip


def test_the_overlay_publishes_a_well_formed_image(tmp_path, monkeypatch, caplog):
    """★ 叠加图那条路走得通：**真 render + 真数值自检 + 消息包装**（Step 11-14）。

    用的是**到位态**夹具（箱子四角取成台面四角本身）：三个量都 ≈ 0、五样东西
    都在图内 —— 那一帧 `numeric_self_check` 必须返回空（Task 3 花了三轮才把
    "伺服到位那一帧必然失败"修掉），所以这条用例同时是那条修复的回归网。

    ⚠️ 这一整段**必须包在 try 里**：可视化是诊断设施，它不该有能力把节点弄死。
    """
    import types
    from skills.atomic.perception.pallet_servo.algorithm import (
        project_pallet_points)

    K = np.array([[1000.0, 0.0, 640.0], [0.0, 1000.0, 400.0], [0.0, 0.0, 1.0]])
    size = (1200.0, 800.0)
    canvas_w, canvas_h = 1280, 1200
    # 托盘**正对相机**（中心落在光轴上）：1200×800mm 在 1m 深、fx=1000 下投影成
    # u∈[40,1240]、v∈[50,850]，五样东西都在图内。与 render 的自检夹具同源。
    T = np.eye(4)
    T[:3, 3] = (-0.6, -0.35, 1.0)
    quad = np.asarray(project_pallet_points(
        T, [(0.0, 0.0, 0.0), (size[0], 0.0, 0.0),
            (size[0], size[1], 0.0), (0.0, size[1], 0.0)], K), np.float64)
    # 台面四角顺序是 [左下,右下,右上,左上]，箱子契约顺序是 [右下,左下,左上,右上]
    box_quad = [quad[1], quad[0], quad[3], quad[2]]
    err = servo_error(T, size, ("y=0", "x=W"),
                      BoxObservation(quad=[list(p) for p in box_quad]), K,
                      image_size=(canvas_w, canvas_h))
    assert isinstance(err, ServoError), err
    assert abs(err.e_bottom_px) < 1e-6 and abs(err.e_right_px) < 1e-6, err

    _fake_ros_modules(monkeypatch)
    node = make_node(tmp_path, overlay_out="/pallet_servo/overlay",
                     K=K.tolist(), image_size=[canvas_w, canvas_h],
                     pallet_size_mm=list(size))
    pub = _FakePublisher()
    node._overlay_pub = pub
    node._last_T_cam_pallet = T
    node._on_color(_fake_color_msg(canvas_w, canvas_h))

    with caplog.at_level(logging.WARNING):
        node._maybe_publish_overlay(err, 1)
    assert pub.n == 1, f"没发图：{node._last_overlay_warn}"
    msg = pub.msg
    assert (msg.height, msg.width) == (canvas_h, canvas_w), (msg.height, msg.width)
    assert msg.encoding == "bgr8" and msg.step == canvas_w * 3
    assert len(msg.data) == canvas_h * canvas_w * 3, len(msg.data)
    assert msg.header.frame_id == node._camera_frame, msg.header.frame_id
    assert "没通过数值自检" not in caplog.text, caplog.text

    # 降频：紧接着再来一次不发第二张（伺服 10Hz、图 5Hz 就够看）
    node._maybe_publish_overlay(err, 1)
    assert pub.n == 1, "没按 overlay_period_s 降频"
    print(f"    叠加图发得出去：{msg.width}x{msg.height} {msg.encoding} "
          f"{len(msg.data)} 字节")


def test_the_overlay_can_be_turned_off_entirely(tmp_path):
    """`overlay_out` 给空串 = **完全不发图**（省 CPU、省带宽）。"""
    node = make_node(tmp_path, overlay_out="")
    assert node._overlay_pub is None
    assert node._color_sub is None and node._box_uv_sub is None
    node._maybe_publish_overlay(SimpleNamespace(), 1)      # 空转，不抛
    assert node._n_overlay == 0


def test_the_overlay_self_check_warning_is_deduped_by_probe_not_by_pixels(
        tmp_path, monkeypatch, caplog):
    """★ 自检失败的去重键**不带探针坐标**（F2，与存活告警是同一类病）。

    `numeric_self_check` 报出来的每条问题里都嵌着**探针像素坐标**，而坏帧是按
    发图频率（5 Hz）发生的事：托盘一动 `(ui, vi)` 每帧都变 —— 按整条文案去重
    **恒不相等**，等于没去重。**数据相关**的自检失败（比如托盘临到画面边缘）
    正会让坐标每帧漂，那时就是 5 Hz 刷屏，而且刷在最需要看日志的时刻。
    操作员要行动的信息是"**哪条探针、怎么坏的**"（探针名 + 失败种类），坐标留在
    正文里 —— 那是现场读数。

    ⚠️ 只把 `render` 的两个函数换成桩（渲染细节不是这条的被测对象）：被测的是
    **节点这一侧的去重策略**，而问题串用的是 `render.py` 的真实格式。
    """
    import skills.atomic.perception.pallet_servo.render as render

    monkeypatch.setattr(render, "render_servo_overlay",
                        lambda *a, **k: (np.zeros((40, 40, 3), np.uint8), []))
    # 同一批"探针坏了"的事实，**坐标每帧漂**（托盘在动）
    problems = iter([
        f"托盘参考边 底边: ({100 + d}, {200 + d}) 附近没有目标颜色 (0, 255, 0)"
        for d in range(10)])
    monkeypatch.setattr(render, "numeric_self_check",
                        lambda canvas, probes, tolerance=8: [next(problems)])

    node = make_node(tmp_path, overlay_period_s=0.0)     # 不降频：每 tick 都试发
    node._overlay_pub = _FakePublisher()
    node._on_color(_fake_color_msg(40, 40))
    err = SimpleNamespace(e_bottom_px=1.0, e_right_px=2.0, theta_rad=0.01,
                          pallet_version=1, box_version=1,
                          t_cam_base_src="identity", box_source="window",
                          warn=[])

    with caplog.at_level(logging.WARNING):
        for _ in range(5):
            node._maybe_publish_overlay(err, 1)
    n = caplog.text.count("没通过数值自检")
    assert n == 1, (
        f"坐标每帧漂就让去重失效了：5 次发图喊了 {n} 条（键里带着逐帧在变的"
        f"读数）—— {caplog.text}")
    # 键里不许有坐标（探针名 + 失败种类就够了）
    assert "100" not in node._last_overlay_warn, node._last_overlay_warn
    assert node._last_overlay_warn == "托盘参考边 底边: ", node._last_overlay_warn

    # 换了**另一条**探针坏 → 是新事实，要重新喊（键不能粗到把不同的探针并掉）
    problems = iter(["箱子 底边: (7, 8) 附近没有目标颜色 (0, 255, 0)"])
    with caplog.at_level(logging.WARNING):
        node._maybe_publish_overlay(err, 1)
    assert caplog.text.count("没通过数值自检") == 2, caplog.text


def test_a_second_broken_probe_is_shouted_about_while_the_first_stays_broken(
        tmp_path, monkeypatch, caplog):
    """★ Task 5 遗留的口径缺口：**原来那条仍坏、又新增一条** → 也要重新喊。

    既有用例证明的是"**换**一条探针坏 → 重新喊"（键从 `A` 变成 `B`）。这里补的
    是 `A` → `A; B`：**事实变多了**（两条探针同时坏），键也必须跟着变。
    若把键实现成"只看第一条"或"按条数幂等之外的任何粗粒度"，这一条会红。

    现场对应的是**逐步恶化**：先是托盘参考边贴到画面边缘，接着箱子那条探针也被
    别的图层文字盖住 —— 后者必须能在日志里看见，而不是被"同一个键"静默吞掉。
    """
    import skills.atomic.perception.pallet_servo.render as render

    monkeypatch.setattr(render, "render_servo_overlay",
                        lambda *a, **k: (np.zeros((40, 40, 3), np.uint8), []))
    a = "托盘参考边 底边: (100, 200) 附近没有目标颜色 (0, 255, 0)"
    b = "箱子 底边: (7, 8) 附近没有目标颜色 (0, 255, 0)"
    problems = iter([[a], [a, b]])          # 第一帧只有 A；第二帧 A 仍坏、又多了 B
    monkeypatch.setattr(render, "numeric_self_check",
                        lambda canvas, probes, tolerance=8: next(problems))

    node = make_node(tmp_path, overlay_period_s=0.0)
    node._overlay_pub = _FakePublisher()
    node._on_color(_fake_color_msg(40, 40))
    err = SimpleNamespace(e_bottom_px=1.0, e_right_px=2.0, theta_rad=0.01,
                          pallet_version=1, box_version=1,
                          t_cam_base_src="identity", box_source="window",
                          warn=[])

    with caplog.at_level(logging.WARNING):
        node._maybe_publish_overlay(err, 1)                 # A
        assert caplog.text.count("没通过数值自检") == 1, caplog.text
        node._maybe_publish_overlay(err, 1)                 # A 仍坏 + 新增 B
    assert caplog.text.count("没通过数值自检") == 2, (
        f"原来那条仍坏时新增了一条探针，日志里却看不到 —— 键粗到把'多坏一条'"
        f"并掉了：{caplog.text}")
    assert "箱子 底边" in caplog.text, caplog.text


def test_the_overlay_self_check_warning_is_reminded_periodically(
        tmp_path, monkeypatch, caplog):
    """★ I2：自检**长期**不过时不能只有开头一条 WARNING —— 每 N tick 提醒一次。

    托盘长期贴着画面边缘 / 探针被别的图层文字盖住时，自检**一直**不过、图**一直**
    不发 —— 而 `/pallet_servo/overlay` 一条消息都没有。**"话题一条消息都没有"与
    "rqt 里话题名写错了"长得一模一样**，操作员的第一反应恰恰是后者。检测器停摆
    专门加了 `live_timeout_s` 防静默，可视化这条路同样不能没有存活口径。

    ⚠️ 提醒**按 tick 推进**（`_last_overlay_warn_tick`），且不该把去重打掉：
    同一 tick 内连着调多少次都还是那一条（去重仍然有效）。
    """
    import skills.atomic.perception.pallet_servo.render as render

    monkeypatch.setattr(render, "render_servo_overlay",
                        lambda *a, **k: (np.zeros((40, 40, 3), np.uint8), []))
    monkeypatch.setattr(render, "numeric_self_check",
                        lambda canvas, probes, tolerance=8:
                        ["托盘参考边 底边: (100, 200) 附近没有目标颜色 (0, 255, 0)"])

    node = make_node(tmp_path, overlay_period_s=0.0)
    node._overlay_pub = _FakePublisher()
    node._on_color(_fake_color_msg(40, 40))
    err = SimpleNamespace(e_bottom_px=1.0, e_right_px=2.0, theta_rad=0.01,
                          pallet_version=1, box_version=1,
                          t_cam_base_src="identity", box_source="window",
                          warn=[])

    with caplog.at_level(logging.WARNING):
        for tick in range(1, 62):                # 60 个 tick = 两个提醒周期
            node._ticks = tick
            node._maybe_publish_overlay(err, 1)
    assert caplog.text.count("报同一个问题") == 2, (
        f"周期提醒没响（或响的次数不对）：{caplog.text.count('报同一个问题')} 条 —— "
        f"长期不过的自检会彻底安静")
    assert caplog.text.count("没通过数值自检") == 3, caplog.text   # 首次 + 两次提醒


def test_the_overlay_counters_are_visible_in_the_log_when_nothing_goes_out(
        tmp_path, caplog):
    """★ I2：`_n_overlay` / `_n_overlay_skip` 从前**只写不读** —— 现在要进日志。

    一个长期发不出图的链路（底图话题没发 / 自检一直不过），操作员**无从判断**
    "到底发出去过没有"，日志里只有开头那一条 WARNING。计数器进日志之后，
    "图发不出去"就是**可观察**的，而不是只能靠猜。

    ⚠️ 级别必须是 **INFO**：随仓库提供的 `config/log_config.yaml` 全局级别就是
    INFO，打进 DEBUG（`_log_throttled`）等于没打 —— 而这条日志的全部目的就是
    "在默认配置下看得见"。
    """
    node = make_node(tmp_path, overlay_period_s=0.0)
    node._overlay_pub = _FakePublisher()
    with caplog.at_level(logging.INFO):
        for tick in range(1, 62):
            node._ticks = tick
            node._maybe_publish_overlay(SimpleNamespace(), 1)     # 没有底图
    assert node._n_overlay_skip == 61, node._n_overlay_skip
    assert caplog.text.count("因无底图跳过") >= 2, (
        f"计数器没进日志 —— 发不出图时现场一片安静：{caplog.text}")
    assert "发图 0 张" in caplog.text, caplog.text
    # 三个数**一起**报：只看"发图 0 张"分不出是底图没来还是自检不过
    assert "自检不过 0 次" in caplog.text, caplog.text



# --------------------------------------------------------------------------- #
# 伺服误差话题（design 2026-09-24）
# --------------------------------------------------------------------------- #
class _FakeServoPub:
    """假发布器：只记下发过什么。**不需要 roscore。**"""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    @property
    def n(self):
        return len(self.msgs)

    @property
    def last(self):
        return self.msgs[-1] if self.msgs else None


def _fake_servo_msgs(monkeypatch):
    """假的 `pallet_servo_msgs.msg.PalletServoError`：只为让**发话题那一步真的走完**。

    真消息类要编译过 catkin 包才有，而 CI 里那个包**不一定编过**（与
    `pallet_detection_msgs` 同一个处境 —— 见 `NodePalletObs` 为什么"按字段名取"
    而不是 import 那个消息类）。
    """
    import sys
    import types

    # 只把**这一个名字**换掉，其余属性原样透传 —— 见 `test_the_publisher_is_...`
    # 那条用例的注释（`rospy.core.is_initialized()` 是"有没有 ROS"的唯一判据，
    # 连它一起换掉就等于把被测的那句话换成常量）。
    _rospy = sys.modules.get("rospy")
    if _rospy is not None:
        monkeypatch.setattr(_rospy, "Publisher",
                            lambda *a, **k: types.SimpleNamespace(
                                publish=lambda msg: None),
                            raising=False)

    class _Err:
        __slots__ = ("header", "valid", "reject", "e_bottom_px",
                     "e_right_px", "theta_rad")

        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.valid = False
            self.reject = ""
            self.e_bottom_px = 0.0
            self.e_right_px = 0.0
            self.theta_rad = 0.0

    fake = types.ModuleType("pallet_servo_msgs.msg")
    fake.PalletServoError = _Err
    pkg = types.ModuleType("pallet_servo_msgs")
    pkg.msg = fake
    monkeypatch.setitem(sys.modules, "pallet_servo_msgs", pkg)
    monkeypatch.setitem(sys.modules, "pallet_servo_msgs.msg", fake)


def test_the_servo_error_topic_publishes_a_valid_frame(tmp_path, monkeypatch):
    """★ 有效帧：`valid=true`、三个量正确、stamp 是**配对的图像采集时刻**。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera",
                     servo_error_out="/pallet_servo/dis")
    node._servo_error_pub = _FakeServoPub()
    node._publish_error(valid=True, reject="", e_bottom=-23.5, e_right=114.1,
                        theta=-0.0229, stamp=1234.5)
    msg = node._servo_error_pub.last
    assert msg.valid is True
    assert msg.reject == ""
    assert msg.e_bottom_px == -23.5 and msg.e_right_px == 114.1
    assert abs(msg.theta_rad - (-0.0229)) < 1e-12
    assert msg.header.stamp.to_sec() == 1234.5
    assert msg.header.frame_id == "camera_color_optical_frame"


def test_the_topic_covers_every_exit_of_update(tmp_path, monkeypatch):
    """★ `update()` 的每个出口都要在话题上留痕 —— 尤其是**停摆**。

    没有"停摆"那一条的话，「检测器挂了」= 「没有新消息」，而这跟
    「roscore 挂了 / 话题名写错了」长得一模一样。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    # 黑板是**进程级**的，前面的用例早写过这四个键了 —— 而本条的第一个断言要的
    # 恰恰是"一个输入都没有"，所以先清空（与本文件既有用例同一句先例）。
    py_trees.blackboard.Blackboard.clear()
    node = make_node(tmp_path, pallet_frame="camera")
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp", "latest_box_obs",
                         "latest_box_obs_version")

    # 出口 6：等输入
    node.update()
    assert pub.n == 1 and pub.last.valid is False
    assert "等输入" in pub.last.reject, pub.last.reject

    # 出口 9：同一状态重复 tick —— 心跳周期内**不重发**
    node.update()
    assert pub.n == 1, f"心跳周期内重发了：{pub.n}"

    # 出口 5/2：给一个配不成对的输入
    # ⚠️ 本文件**没有** `_pose6d` 这个辅助 —— 位姿一律走
    #    `matrix_to_pose6d(straight_down_camera())`（见 573 / 716 行那几处既有用法）。
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()), a_box(),
                 pallet_version=1, box_version=1)
    setattr(writer, "latest_pallet_stamp", 100.0)
    node.update()
    assert pub.last.valid is False
    assert pub.last.reject, "无效帧必须带原因"


def test_the_invalid_frame_fills_nan_not_zero(tmp_path, monkeypatch):
    """★ `valid=false` 时三个量是 NaN —— **不是 0.0**。

    `0.0` 恰好是「误差为零 = 完全对准」的意思，漏判 `valid` 的消费者会以为
    箱子已经到位然后一动不动。这是本设计里最危险的那条静默错。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera")
    node._servo_error_pub = _FakeServoPub()
    node._publish_invalid("停摆？ 托盘 已经 8.2s 没有新的观测")
    msg = node._servo_error_pub.last
    assert msg.valid is False
    assert math.isnan(msg.e_bottom_px)
    assert math.isnan(msg.e_right_px)
    assert math.isnan(msg.theta_rad)
    assert "停摆" in msg.reject


def test_the_heartbeat_resends_invalid_frames(tmp_path, monkeypatch):
    """★ 无效帧内容不变时按心跳重发 —— 消费者**永远能在一秒内**查到状态。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera",
                     servo_error_heartbeat_s=0.05)
    node._servo_error_pub = _FakeServoPub()
    node._publish_invalid("等输入（托盘 缺，箱子 缺）")
    assert node._servo_error_pub.n == 1
    node._publish_invalid("等输入（托盘 缺，箱子 缺）")     # 内容没变、时间没到
    assert node._servo_error_pub.n == 1, "心跳周期内不该重发"
    time.sleep(0.06)
    node._publish_invalid("等输入（托盘 缺，箱子 缺）")     # 过了心跳周期
    assert node._servo_error_pub.n == 2, "过了心跳周期该重发"


def test_the_heartbeat_zero_means_only_on_change(tmp_path, monkeypatch):
    """`servo_error_heartbeat_s=0` ⇒ 只在内容变化时发（要"一条都不重复"就用它）。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera",
                     servo_error_heartbeat_s=0.0)
    node._servo_error_pub = _FakeServoPub()
    node._publish_invalid("A")
    node._publish_invalid("A")
    assert node._servo_error_pub.n == 1
    node._publish_invalid("B")
    assert node._servo_error_pub.n == 2


def test_a_changed_reject_publishes_immediately(tmp_path, monkeypatch):
    """拒绝原因变了要**立刻**发，不等心跳。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera",
                     servo_error_heartbeat_s=100.0)
    node._servo_error_pub = _FakeServoPub()
    node._publish_invalid("A")
    node._publish_invalid("B")
    assert node._servo_error_pub.n == 2


def test_empty_out_builds_no_publisher_and_sends_nothing(tmp_path, monkeypatch):
    """`servo_error_out=""` ⇒ 连发布器都不建，一次都不发。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera", servo_error_out="")
    node.initialise()
    assert node._servo_error_pub is None
    node._publish_invalid("随便")           # 不该抛
    assert node._servo_error_pub is None


def test_publish_failure_is_a_warning_not_an_exception(tmp_path, monkeypatch,
                                                       caplog):
    """★ 发布抛异常时只记 WARNING，**绝不掀翻伺服环路**。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera")

    class _Boom:
        def publish(self, msg):
            raise RuntimeError("roscore 挂了")

    node._servo_error_pub = _Boom()
    with caplog.at_level(logging.WARNING):
        node._publish_error(valid=True, reject="", e_bottom=1.0,
                            e_right=2.0, theta=0.1, stamp=1.0)   # 不抛
    assert "发伺服误差失败" in caplog.text, caplog.text


def test_dry_run_sends_nothing(tmp_path, monkeypatch):
    """干跑不建发布器（与可视化同一条纪律）。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    node = NodePalletServo("servo", "servo", "ns", {
        "ref_edges": ["y=0", "x=W"], "pallet_size_mm": [1200.0, 1000.0],
        "K": K_PARAM, "image_size": IMAGE_SIZE, "pallet_frame": "camera",
        "servo_error_out": "/pallet_servo/dis"})
    node.initialise()
    assert node._servo_error_pub is None


def test_the_valid_frame_stamp_is_the_paired_image_time(tmp_path, monkeypatch):
    """★ 有效帧的 `header.stamp` 是**配对的图像采集时刻**，不是出数时刻。

    这一条决定了录下来的 bag 能不能跟图像对齐。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera")
    node._servo_error_pub = _FakeServoPub()
    node._last_pair_stamp = 987.25
    node._publish_error(valid=True, reject="", e_bottom=0.0, e_right=0.0,
                        theta=0.0, stamp=node._last_pair_stamp)
    assert node._servo_error_pub.last.header.stamp.to_sec() == 987.25


def test_a_real_ros_node_does_build_the_publisher(tmp_path, monkeypatch):
    """★ `init_node()` 起过时**必须真的建发布器** —— 否则上面那条守卫就把它关死了。

    这条是 `_ros_node_initialized()` 的**另一半**：没有它的话，守卫写成恒 `False`
    也照样全绿，而真机上一个话题都发不出去（现场表现恰好就是"控制器收不到误差"）。
    做法是把 `is_initialized` 换成 `True`（**只换这一个名字**，其余原样透传），
    再看 `rospy.Publisher` 有没有被调到、话题名对不对。
    """
    import types
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    import rospy

    monkeypatch.setattr(rospy.core, "is_initialized", lambda: True)
    built = []

    def _publisher(topic, msg_type, **kwargs):
        built.append((topic, msg_type))
        return types.SimpleNamespace(publish=lambda msg: None)

    monkeypatch.setattr(rospy, "Publisher", _publisher)

    node = make_node(tmp_path, pallet_frame="camera",
                     servo_error_out="/pallet_servo/dis")
    # ⚠️ 只看**伺服误差**这一条：同一次 `initialise()` 里可视化那条也会建一个
    # `rospy.Publisher`（`Image`），断言整张表会被它污染。
    servo_built = [(topic, msg_type) for topic, msg_type in built
                   if topic == "/pallet_servo/dis"]
    assert servo_built == [
        ("/pallet_servo/dis",
         sys.modules["pallet_servo_msgs.msg"].PalletServoError)], built
    assert node._servo_error_pub is not None


def test_no_ros_node_means_no_publisher_and_no_shouting(tmp_path, monkeypatch,
                                                       caplog):
    """★ **没有 `init_node()`**（离线跑 / 单测）⇒ 不建发布器，**而且一个字都不许吵**。

    不守这一句的话，每个没 `init_node()` 的进程每次建树都多一条
    「伺服误差话题起不来」的 WARNING，而**真正的**"`pallet_servo_msgs` 没编译"
    就淹进同一类噪声里（与 `live_timeout_s` 那条 F1 是同一个病）。

    ⚠️ `pallet_servo_msgs` 在这个进程里**是 import 得到的**（devel 在 sys.path 上），
    所以这条钉的确实是"没有 ROS"这一支，不是"消息包没编译"那一支。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    import rospy

    monkeypatch.setattr(rospy.core, "is_initialized", lambda: False)

    with caplog.at_level(logging.WARNING):
        node = make_node(tmp_path, pallet_frame="camera")
    assert node._servo_error_pub is None
    # ⚠️ 只看**伺服误差**这条路的告警：同一次 `initialise()` 里可视化那条也起不来
    # （假 `rospy` 没有 `Subscriber`），那是另一个话题、另一条纪律。
    shouted = [r.getMessage() for r in caplog.records
               if r.levelno >= logging.WARNING and "伺服误差" in r.getMessage()]
    assert not shouted, (
        f"没有 ROS 时却吵了 {len(shouted)} 条：{shouted} —— 那不是「话题起不来」，"
        f"是「现在还没有 ROS」")


def test_a_stalled_detector_publishes_an_invalid_frame(tmp_path, monkeypatch):
    """★ **停摆必须出现在话题上** —— 这一条是本设计存在的理由之一。

    没有它的话，「检测器挂了」= 「没有新消息」，而这跟「roscore 挂了 / 话题名
    写错了」在消息层面**长得一模一样** —— 消费者唯一能做的就是等，而永远等不到。

    只测 `update()` 的返回值与日志是不够的（那两条早就有了）：这里钉的是
    `_check_liveness` 的描述**真的被送上了话题**。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp", "latest_box_obs",
                         "latest_box_obs_version")
    setattr(writer, "latest_pallet", matrix_to_pose6d(straight_down_camera()))
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_pallet_stamp", 1.0)
    setattr(writer, "latest_box_obs", a_box(stamp=1.0))
    setattr(writer, "latest_box_obs_version", 1)

    node = make_node(tmp_path, pallet_frame="camera", live_timeout_s=0.05)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    assert node.update() == Status.RUNNING
    assert pub.last.valid is True, "第一帧是有效帧"

    # 两路都不再出新观测：把两个"上次见过"的时钟往回拨（等效于时间往前走）
    node._last_pallet_seen -= 1.0
    node._last_box_seen -= 1.0
    assert node.update() == Status.RUNNING
    assert pub.last.valid is False, "停摆却发了一条有效帧"
    assert "停摆" in pub.last.reject, pub.last.reject
    assert "托盘" in pub.last.reject, pub.last.reject


def test_the_valid_frame_published_by_update_carries_the_paired_stamp(
        tmp_path, monkeypatch):
    """★ `update()` 发出去的有效帧，`header.stamp` 是**配对的图像采集时刻**。

    上一条同类用例直接调 `_publish_error(stamp=...)`，钉的是**包装**；这一条走
    完整的 `update()`，钉的是**调用点** —— 把 `stamp=self._last_pair_stamp`
    写成 `stamp=None`（发出去的变成出数时刻）时只有这一条会红。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp", "latest_box_obs",
                         "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()),
                 a_box(stamp=4321.5), pallet_version=1, box_version=1)
    setattr(writer, "latest_pallet_stamp", 4321.5)

    node = make_node(tmp_path, pallet_frame="camera")
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    assert node.update() == Status.RUNNING
    assert pub.last.valid is True, pub.last.reject
    assert pub.last.header.stamp.to_sec() == 4321.5, (
        f"有效帧的 stamp 不是配对的图像采集时刻："
        f"{pub.last.header.stamp.to_sec()}（这一条决定 bag 能不能跟图像对齐）")


def test_the_overlay_labels_the_ref_edges_of_the_slot_in_this_frame(
        tmp_path, monkeypatch, caplog):
    """★ `update()` 走完后，图上参考边的标签是**这一帧那一组**的原文（Task 5）。

    钉的是 `update()` 里的**实参**：写死 `self._slots[0]`、或者用
    `self._active_slot` 而不是本 tick 的快照 `slot`，都只有这一条会红 ——
    `_maybe_publish_overlay` 自己的用例全是**直接调用**、显式传 slot，绕过了
    `update()` 里这次传参（把实参换成 `self._slots[0]`，全套仍然全绿，实测）。

    ⚠️ **必须走 `update()`**：切片基准是"这一帧"的快照，而"切了槽位但本 tick
    还没消费"这一格正好是差一位 bug 的藏身处 —— 直接调就永远构造不出来。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    from skills.atomic.perception.pallet_servo.algorithm import (
        project_pallet_points)
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp", "latest_box_obs",
                         "latest_box_obs_version")

    # 台面 [1200, 800]、相机在光轴上正对，K 也是本文件的夹具 → 五样东西都在图内，
    # 到那一帧（箱子四角取成台面四角本身）数值自检必然通过、图必然发得出去。
    size = (PALLET_W_MM, PALLET_H_MM)
    k = np.array(K_PARAM, np.float64)
    T = np.eye(4)
    T[:3, 3] = (-0.6, -0.35, 1.0)
    quad = np.asarray(project_pallet_points(
        T, [(0.0, 0.0, 0.0), (size[0], 0.0, 0.0),
            (size[0], size[1], 0.0), (0.0, size[1], 0.0)], k), np.float64)
    # 台面四角顺序 [左下,右下,右上,左上]，箱子契约顺序 [右下,左下,左上,右上]

    def _box(stamp):
        return BoxObservation(quad=[list(p) for p in
                                    (quad[1], quad[0], quad[3], quad[2])],
                              label="box", confidence=0.9, stamp=stamp)

    def _feed(stamp, version):
        setattr(writer, "latest_pallet", matrix_to_pose6d(T))
        setattr(writer, "latest_pallet_version", version)
        setattr(writer, "latest_pallet_stamp", stamp)
        setattr(writer, "latest_box_obs", _box(stamp))
        setattr(writer, "latest_box_obs_version", version)

    _feed(1.0, 1)

    # 第 2 组故意与第 1 组不同：写死 `_slots[0]` 时标签立刻露馅。
    canvas_w, canvas_h = 1280, 1200
    node = make_node(tmp_path, pallet_frame="camera", overlay_period_s=0.0,
                     image_size=[canvas_w, canvas_h], K=k.tolist(),
                     pallet_size_mm=list(size),
                     ref_edges=[["y=0", "x=W"], ["y=H", "x=W"]])
    seen = {}

    def _capture(color, *, error, T_cam_pallet, size_mm, K, D=None,
                 yolo_uv=None, hud_lines=(), ref_edge_labels=None):
        seen["labels"] = ref_edge_labels
        return np.zeros((40, 40, 3), np.uint8), []

    import skills.atomic.perception.pallet_servo.render as render
    monkeypatch.setattr(render, "render_servo_overlay", _capture)
    monkeypatch.setattr(render, "numeric_self_check",
                        lambda canvas, probes, tolerance=8: [])

    pub = _FakePublisher()
    node._overlay_pub = pub
    node._on_color(_fake_color_msg(canvas_w, canvas_h))

    # 第 1 组：喂一帧 → 出数、发图、标签是第 1 组
    assert node.update() == Status.RUNNING, node.feedback_message
    assert pub.n == 1, f"没发图：{node._last_overlay_warn}"
    assert seen.get("labels") == ["y=0", "x=W"], seen

    # ★ **服务回调晚到**：本 tick 开头拍下的快照是第 2 组，而回调在发图之前才把
    #   `_active_slot` 改成 1（服务跑在 ROS 线程，这正是快照存在的理由）。
    #   标签必须跟**快照**走 —— 用 `self._active_slot` 时这一条会红。
    assert node._activate_slot(2)[0] is True
    _feed(2.0, 2)
    publish_error = node._publish_error

    def _late_slot_switch(*args, **kwargs):
        out = publish_error(*args, **kwargs)
        node._active_slot = 1                  # 回调在这之后才落地
        return out

    monkeypatch.setattr(node, "_publish_error", _late_slot_switch)
    assert node.update() == Status.RUNNING, node.feedback_message
    assert node._active_slot == 1, "这条用例的前提是回调把槽位改回了 1"
    assert seen.get("labels") == ["y=H", "x=W"], (
        f"标签跟着 `_active_slot` 走了，而它是这一帧之后才改的："
        f"{seen.get('labels')} —— 图上指的边与算出来的数分了家，还一个错都不报")

    # 下一帧（快照也已经是 1 组）标签跟着回第 1 组
    monkeypatch.setattr(node, "_publish_error", publish_error)
    _feed(3.0, 3)
    assert node.update() == Status.RUNNING, node.feedback_message
    assert seen.get("labels") == ["y=0", "x=W"], (
        f"图上参考边的标签用的不是这一帧那一组：{seen} —— 指错边比不标更糟")


def test_reentering_initialise_does_not_respam_the_topic(tmp_path, monkeypatch):
    """★ 心跳状态槽位**不许在 `initialise()` 里重置**（配置错误那条路恒 FAILURE）。

    py_trees 对"状态不是 RUNNING"的行为**每 tick 重进 `initialise()`** —— 一重置
    就是每秒几十条**逐字相同**的话题消息，正好把消费者与现场日志一起淹掉。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera",
                     servo_error_heartbeat_s=100.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._publish_invalid("配置错误 —— 某某")
    assert pub.n == 1
    node.initialise()                    # py_trees 每 tick 都会这么干
    node._publish_invalid("配置错误 —— 某某")
    assert pub.n == 1, (
        f"重进 `initialise()` 之后重发了 {pub.n - 1} 条一样的心跳 —— "
        f"配置错误那条路恒 FAILURE，一重置就是每秒几十条")


def test_the_normal_dedup_path_publishes_nothing(tmp_path, monkeypatch):
    """★ 出口 9（`resolve()` 返回 `None` 且**没停摆**）**一条都不许发**。

    `resolve()` 返回 `None` 有两种含义：① 同一对已经报过（正常去重）② 某一侧
    没有新观测。① 的情况下**那一对的有效帧刚刚已经发过了**，再补一条无效帧
    是错的 —— 消费者会把一条好帧看成"不能用"，然后停下来。

    重复 tick 同一个输入即可命中：版本号没变、stamp 没变、存活闸也没超时。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    writer = make_writer("latest_pallet", "latest_pallet_version",
                         "latest_pallet_stamp", "latest_box_obs",
                         "latest_box_obs_version")
    write_inputs(writer, matrix_to_pose6d(straight_down_camera()),
                 a_box(stamp=7.0), pallet_version=1, box_version=1)
    setattr(writer, "latest_pallet_stamp", 7.0)

    node = make_node(tmp_path, pallet_frame="camera")
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    assert node.update() == Status.RUNNING
    assert pub.n == 1 and pub.last.valid is True, pub.last.reject

    assert node.update() == Status.RUNNING        # 同一对，重复 tick
    assert pub.n == 1, (
        f"正常去重那一路发了 {pub.n - 1} 条无效帧（{pub.last.reject!r}）—— "
        f"那一对的有效帧刚刚已经发过了")
    assert pub.last.valid is True, pub.last.reject


def test_no_ros_still_finishes_initialise(tmp_path, monkeypatch):
    """★ 没有 ROS 时**只跳过建发布器**，`initialise()` 的其余部分照常跑完。

    建发布器那段一旦用 `return` 提前跳出（而不是只跳过构造），后面那句
    「初始化 INFO」日志与 `feedback_message` 就**永远不会打** —— 而那正是现场
    第一件要核对的日志行（K / ref_edges / 内参来源）。现场表现是"日志里少了
    那一段"，没有任何报错。

    ⚠️ 这一条**必须让 `rospy` 是个"真的没 init_node"的模块**：用假 rospy 时
    `Publisher` 属性根本不存在，提前 `return` 与正确写法都会走到 `except`，
    两种实现都绿 —— 那样这条用例就白写了。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    import rospy

    monkeypatch.setattr(rospy.core, "is_initialized", lambda: False)

    node = make_node(tmp_path, pallet_frame="camera")
    assert node._servo_error_pub is None
    # 走到 `initialise()` 的**最后一行**才会写这句（正是"没提前 return"的证据）
    assert node.feedback_message.startswith("就绪："), node.feedback_message


def test_a_stalled_detector_does_not_respam_the_topic(tmp_path, monkeypatch):
    """★ 停摆文案里的秒数**不许让去重失效** —— 否则就是每 tick 一条（10 Hz）。

    停摆那条 `reject` 是 `检测器停摆？ 托盘 已经 8.2s 没有**新的观测**…`，而
    `8.2s` **每 tick 都在长** —— 拿原文当去重键恒不相等，去重形同虚设。

    ⚠️ 这一条是**真机跑出来的**：第一次端到端验收时 `rostopic echo` 上停摆之后
    每秒 ~10 条（心跳本该是 1 条），而节点层单测全绿 —— 因为那些用例调的是
    `_publish_invalid("A")` 这种**不含读数的短文案**，正好绕开了这一格。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera",
                     servo_error_heartbeat_s=100.0)   # 心跳放到很大 = 只靠去重
    node._servo_error_pub = _FakeServoPub()

    # 同一路停摆，秒数逐帧在长（真机文案的形状）
    node._publish_invalid("检测器停摆？ 托盘 已经 5.0s 没有**新的观测**（阈值 5s）")
    node._publish_invalid("检测器停摆？ 托盘 已经 5.1s 没有**新的观测**（阈值 5s）")
    node._publish_invalid("检测器停摆？ 托盘 已经 5.2s 没有**新的观测**（阈值 5s）")
    assert node._servo_error_pub.n == 1, (
        f"秒数一变就重发了 {node._servo_error_pub.n - 1} 条 —— 去重键没砍掉读数")

    # 但**换了停摆侧**要立刻发（稳定部分不同）
    node._publish_invalid("检测器停摆？ 箱子 已经 5.3s 没有**新的观测**（阈值 5s）")
    assert node._servo_error_pub.n == 2, "换了停摆侧却不当回事"
    # 发出去的仍是**原文**（秒数是现场读数，要留给人看）
    assert "5.3s" in node._servo_error_pub.last.reject


def test_the_invalid_frame_stamp_is_the_publish_time(tmp_path, monkeypatch):
    """★ 无效帧的 `header.stamp` 是**发布时刻**，不是 0。

    契约在 `PalletServoError.msg`：`valid=false` 的帧是「**当前状态**的声明」
    （"托盘停摆了"），不是"某一帧图像的检测结果" —— 它的时间就该是**声明时刻**。

    ⚠️ 填 0 会让 `rostopic echo` 上停摆帧的时间戳全变成 1970，**录 bag 回放时
    时间轴错乱**（消费者判"最后一条消息多久没来了"会直接失效）。这条是 2026-09-24
    真机验收时推翻原设计（原设计写的正是 0）之后补的。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, pallet_frame="camera")
    node._servo_error_pub = _FakeServoPub()
    node._publish_invalid("等输入（托盘 缺，箱子 缺）")
    # 假 rospy 的 `Time.now()` 返回 1.0（见 `_fake_ros_modules`）
    assert node._servo_error_pub.last.header.stamp.to_sec() == 1.0, (
        f"无效帧的 stamp 不是发布时刻：{node._servo_error_pub.last.header.stamp}")


# --------------------------------------------------------------------------- #
# 参考边槽位 + 开关服务
# --------------------------------------------------------------------------- #
# ★ **必须用 f-string 引常量**，不能把 1200 / 1000 写死：本文件的
#   `PALLET_W_MM, PALLET_H_MM = 1200.0, 800.0` —— 若把 `y=1000` 写死，第 2 组的
#   底边就**超出台面 H=800**，两条用例会挂在 `ref_edge_out_of_range` 上。
SLOTS = [["y=0", f"x={PALLET_W_MM}"], [f"y={PALLET_H_MM}", f"x={PALLET_W_MM}"]]


def test_slots_accept_both_shapes():
    """`ref_edges` 的两种形状：两条边（一组）或数组的数组（多组）。"""
    assert _normalize_slots(["y=0", f"x={PALLET_W_MM}"]) == [["y=0", f"x={PALLET_W_MM}"]]
    assert _normalize_slots(SLOTS) == SLOTS
    assert _normalize_slots([SLOTS[0]]) == [SLOTS[0]]


def test_inactive_publishes_an_invalid_frame(tmp_path, monkeypatch):
    """★ 未激活时**不算**、发一条 valid=false —— 而不是干脆不发。

    不发的话下游分不清「伺服结束了」与「行为树挂了」，而这两种的处置不同。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    py_trees.blackboard.Blackboard.clear()
    node = NodePalletServo("servo", "servo", "ns", {
        "ref_edges": SLOTS,
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM, "image_size": IMAGE_SIZE,
        "dump_dir": str(tmp_path / "dump"), "pallet_frame": "base_link",
    })
    node.initialise()
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    assert node._active_slot == 0

    status = node.update()

    assert status == Status.RUNNING
    assert pub.n == 1, "未激活时要照发一条"
    assert pub.last.valid is False
    assert "伺服未启动" in pub.last.reject, pub.last.reject
    assert math.isnan(pub.last.e_bottom_px), "无效帧的三个量必须是 NaN"


def test_the_slot_service_switches_and_stops(tmp_path, monkeypatch):
    """服务：1..N 切换、0 停止、越界拒绝且**不动当前状态**。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path, ref_edges=SLOTS)
    assert node._active_slot == 1          # make_node 激活了第 1 组

    ok, message = node._activate_slot(2)
    assert ok is True, message
    assert node._active_slot == 2
    assert f"y={PALLET_H_MM}" in message, message   # 人话里要写清切到了哪组

    ok, message = node._activate_slot(0)
    assert ok is True, message
    assert node._active_slot == 0

    # ★ 越界：**拒绝且不改状态** —— 手滑发个 3 不该把正在跑的伺服停掉
    node._activate_slot(1)
    ok, message = node._activate_slot(3)
    assert ok is False, message
    assert node._active_slot == 1, "越界调用不许改状态"
    assert "3" in message and "2" in message, message

    ok, _ = node._activate_slot(-1)
    assert ok is False
    assert node._active_slot == 1


def test_the_active_slot_decides_which_edges_are_used(tmp_path, monkeypatch):
    """★ 数值自检：切槽位之后**参考边的像素坐标真的变了** —— 不是"数看着对"。

    判据是诊断量 `ref_bottom_px` / `ref_right_px`，它们是参考边端点的图像投影。
    两组的**底边**不同（`y=0` vs `y=H`），投影就必须不同；两组的**右边**是同一条
    `x=W`，投影就必须**逐位相同** —— 后者是反向判据：拿的确实是"当前这一组"，
    而不是别的东西凑出来的。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    writer = make_writer("latest_pallet", "latest_box_obs",
                         "latest_pallet_stamp", "latest_box_obs_stamp")
    T = straight_down_camera()
    writer.set("latest_pallet", matrix_to_pose6d(T))
    writer.set("latest_pallet_stamp", 100.0)
    # ⚠️ **必须带 `quad`**：camera 模式走配对，而配对窗对箱子四角逐点平均
    # （`average_pairs` 读 `b.quad`）—— 只给 AABB 的那一帧在窗里等于"没有四角"，
    # 节点会合理地报「配对结果用不了」、一个数都不出。真实生产者
    # （`NodeBoxObs` / 点点工具 / 本文件的 `a_box()`）都填 `quad`，这里照它们的
    # 形状补上四角，AABB 那两个数仍按 brief 原样保留。
    writer.set("latest_box_obs", BoxObservation(
        u1=CX - 150.0, v1=CY - 120.0, u2=CX + 150.0, v2=CY + 120.0,
        quad=[[CX + 150.0, CY + 120.0], [CX - 150.0, CY + 120.0],
              [CX - 150.0, CY - 120.0], [CX + 150.0, CY - 120.0]],
        stamp=100.0))
    writer.set("latest_box_obs_stamp", 100.0)

    node = make_node(tmp_path, ref_edges=SLOTS, pallet_frame="camera",
                     max_dt_s=1.0)
    node._activate_slot(1)
    node.update()
    first = node.global_blackboard.get("latest_servo_error")
    assert first is not None, "槽位 1 应该出数（黑板没写 → 上一帧被拒了）"

    node._activate_slot(2)
    node.update()
    second = node.global_blackboard.get("latest_servo_error")
    assert second is not None, "槽位 2 应该出数"

    assert not np.array_equal(first.ref_bottom_px, second.ref_bottom_px), \
        "换槽位后参考边像素没变 —— 说明槽位根本没生效"

    # ⚠️ **右边这两组是**一样**的（`SLOTS` 两组的第 2 条边都是 `x=W`）——
    #    它**必须逐位相同**：换槽位只该换掉表里真正不同的那条边，右边要是也跟着
    #    变了，说明实现拿的不是"当前这一组"（而是别的东西凑出来的）。
    assert np.array_equal(first.ref_right_px, second.ref_right_px), \
        "两组的右边是同一条 x=W，投影不该变"

    # 而且两个结果**不是随便什么不同**：槽位 2 的底边是 y=H。这台相机在托盘
    # 正上方垂直向下看，托盘 +y 与图像 +v 同向（1 mm = 1 px）—— 所以 y=H 的
    # 底边在图上**更靠下**（v 更大），不是 brief 里写的"更靠上"。
    assert second.ref_bottom_px[:, 1].mean() > first.ref_bottom_px[:, 1].mean()
    assert abs(second.ref_bottom_px[:, 1].mean()
               - first.ref_bottom_px[:, 1].mean() - PALLET_H_MM) < 1e-6


def test_the_whole_slot_table_is_checked_at_startup(tmp_path, monkeypatch):
    """★ **整表**校验：第 2 组写错要在启动时报，不留到轮到它才发现。

    那时箱子已经在托盘上了 —— 而"轮到了才发现"意味着前一个箱子白搬。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = NodePalletServo("servo", "servo", "ns", {
        # 第 2 组两条都沿 x 展开 → 平行，非法。**两个值都在台面内**（H=800），
        # 免得越界先一步抢了话头、让这条用例测到的是别的拒绝码。
        "ref_edges": [["y=0", f"x={PALLET_W_MM}"], ["y=0", f"y={PALLET_H_MM}"]],
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM, "image_size": IMAGE_SIZE,
        "dump_dir": str(tmp_path / "dump"), "pallet_frame": "base_link",
    })
    node.initialise()

    assert node._config_err is not None, "整表校验应该在 initialise 就报错"
    assert "第 2 组" in node._config_err, node._config_err
    assert "parallel" in node._config_err, node._config_err


def test_the_out_of_range_slot_is_caught_at_startup(tmp_path, monkeypatch):
    """绝对写法越界也是启动时的配置错误（尺寸在 `initialise()` 里才定）。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = NodePalletServo("servo", "servo", "ns", {
        "ref_edges": [["y=0", f"x={PALLET_W_MM}"], ["y=0", f"x={PALLET_W_MM + 50}"]],
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM, "image_size": IMAGE_SIZE,
        "dump_dir": str(tmp_path / "dump"), "pallet_frame": "base_link",
    })
    node.initialise()

    assert node._config_err is not None
    assert "out_of_range" in node._config_err, node._config_err


def test_no_slot_service_means_stuck_inactive_not_a_crash(tmp_path, monkeypatch):
    """服务起不来（`pallet_servo_msgs` 没编译）时：WARNING + 节点照常构造。

    行为是"永远不出有效数"，**不是崩溃** —— 与可视化/发布器同一条纪律。
    """
    _fake_ros_modules(monkeypatch)          # 假 rospy **没有** Service 属性
    py_trees.blackboard.Blackboard.clear()
    node = NodePalletServo("servo", "servo", "ns", {
        "ref_edges": SLOTS,
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM, "image_size": IMAGE_SIZE,
        "dump_dir": str(tmp_path / "dump"), "pallet_frame": "base_link",
    })
    node.initialise()                       # 不许抛

    assert node._slot_srv is None
    assert node._active_slot == 0
    assert node._size is not None, "服务起不来不该影响其它初始化"


def test_reentering_initialise_does_not_rebuild_the_slot_service(tmp_path, monkeypatch):
    """`initialise()` 每个非 RUNNING tick 都会重进 —— 服务只建一次。"""
    _fake_ros_modules(monkeypatch)
    import types

    import rospy

    built = []
    handlers = []

    class _FakeSrv:
        def __init__(self, name, srv_type, handler):
            built.append(name)
            handlers.append(handler)

    class _Response:
        def __init__(self):
            self.ok = False
            self.message = ""

    # ⚠️ 这条用例要的是"**真的建过**服务、且只建一次"，所以不能只假 `rospy.Service`：
    #    假 `rospy` 的 `core.is_initialized()` 是 `False`，服务那段会被守卫直接
    #    跳过 —— 那时 `built` 恒为空，"只建一次"这条断言就**永远没有牙**
    #    （它连"一次"都没等到）。所以这里把 `is_initialized` 换成 `True`，
    #    并补一个最小可用的 `pallet_servo_msgs.srv`（只换这两个名字，其余原样透传
    #    —— 与 `test_a_real_ros_node_does_build_the_publisher` 同一手法）。
    monkeypatch.setattr(rospy.core, "is_initialized", lambda: True)
    monkeypatch.setattr(sys.modules["rospy"], "Service", _FakeSrv, raising=False)
    srv = types.ModuleType("pallet_servo_msgs.srv")
    srv.SetServoSlot = object
    srv.SetServoSlotResponse = _Response
    pkg = types.ModuleType("pallet_servo_msgs")
    pkg.srv = srv
    monkeypatch.setitem(sys.modules, "pallet_servo_msgs", pkg)
    monkeypatch.setitem(sys.modules, "pallet_servo_msgs.srv", srv)

    node = make_node(tmp_path, ref_edges=SLOTS)
    node.initialise()
    node.initialise()
    assert built == ["/pallet_servo/slot"], built

    # 顺带把**处理器那一层**也走一遍：服务建起来却从不调用的话，处理器里的接线
    # （`_activate_slot` 的返回值怎么进 response）一行都没被执行过 —— 而那正是
    # `rosservice call` 真正打到的那一层。
    response = handlers[0](SimpleNamespace(slot=2))
    assert response.ok is True and node._active_slot == 2, response.message
    assert f"y={PALLET_H_MM}" in response.message, response.message
    response = handlers[0](SimpleNamespace(slot=99))     # 越界：拒绝且不改状态
    assert response.ok is False and node._active_slot == 2, response.message


def test_config_error_beats_inactive(tmp_path, monkeypatch):
    """★ **两个条件同时成立**时按配置错误报 —— 配置错误优先于未激活。

    上面那些用例各只覆盖一半：`test_inactive_publishes_an_invalid_frame` 是
    "未激活 + 合法配置"，配置错误的几条又都经 `make_node` 激活过。少了这条交叉，
    把 `update()` 里那两段换回"未激活在前"的**整套测试一条都不会变红** ——
    判据处于无保护状态。

    ⚠️ **不能走 `make_node`**：它内部会 `_activate_slot(1)`，把"未激活"这个前提
    破坏掉（见它的注释）。这里照 `test_inactive_publishes_an_invalid_frame` 的
    样子直接构造。

    ⚠️ 参考边用常量写（`PALLET_H_MM = 800.0`）：写死一个超出台面的数会让
    `ref_edge_out_of_range` 抢了话头，而这条用例要的是**平行**那个拒绝码。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = NodePalletServo("servo", "servo", "ns", {
        # 第 1 组两条都沿 x 展开 → 平行 → `ref_edges_parallel`。两个值都在台面
        # 内（0 与 H），所以不会先撞上越界。
        "ref_edges": ["y=0", f"y={PALLET_H_MM}"],
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM, "image_size": IMAGE_SIZE,
        "dump_dir": str(tmp_path / "dump"), "pallet_frame": "base_link",
    })
    node.initialise()
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    assert node._config_err is not None, "配置错误该在 initialise 就报出来"
    assert node._active_slot == 0, "本用例的前提是**未激活**"

    status = node.update()

    # 1) 报的是配置错误，不是"未激活"。RUNNING 意味着现场只会看到"伺服未启动"、
    #    反复 call 服务却什么都没变。
    assert status == Status.FAILURE
    # 2) feedback 是配置错误原文，且**不含**"伺服未启动"
    assert "parallel" in node.feedback_message, node.feedback_message
    assert "ref_edges" in node.feedback_message, node.feedback_message
    assert "伺服未启动" not in node.feedback_message, node.feedback_message
    # 3) 心跳承诺不能断：配置错误这一支也要发一条 `valid=False`
    assert pub.n == 1, "配置错误时也要照发一条"
    assert pub.last.valid is False
    assert "parallel" in pub.last.reject, pub.last.reject
    assert "伺服未启动" not in pub.last.reject, pub.last.reject
