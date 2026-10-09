# -*- coding: utf-8 -*-
"""配对端到端：**两个消息 → 两个节点 → 黑板 → 配对 → 伺服 → 三个量**。

零数据、零硬件、零 ROS、零消息包 —— 全在 CI 里跑。

钉住的是**配对这条接缝**：
  * 同一帧图像的两个观测配得上，且平滑用的是**同一批配对**
  * 差一帧配不上，且拒绝原因写清了实际值与阈值
  * 只有一边时是 `pair_none`，不是崩

运行：
    pytest orchestration/nodes/tests/test_pallet_box_pairing_e2e.py -m unit -v
"""
import math
from types import SimpleNamespace

import numpy as np
import py_trees
import pytest
from py_trees.common import Access, Status

from orchestration.nodes.node_box_obs import NodeBoxObs
from orchestration.nodes.node_pallet_obs import NodePalletObs
from orchestration.nodes.node_pallet_servo import NodePalletServo

pytestmark = pytest.mark.unit

FX = FY = 1000.0
CX, CY = 640.0, 400.0
PALLET_W_MM, PALLET_H_MM = 1200.0, 800.0

QUAD = [[980.0, 1080.0], [700.0, 1120.0], [680.0, 800.0], [960.0, 760.0]]


def _pt(u, v):
    return SimpleNamespace(x=u, y=v)


def _header(stamp):
    return SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp))


def straight_down(x_mm=0.0):
    T = np.eye(4)
    T[2, 3] = 1.0
    T[0, 3] = x_mm / 1000.0
    return T


def pallet_msg(stamp, x_mm=0.0, valid=True, source="color"):
    T = straight_down(x_mm)
    return SimpleNamespace(
        header=_header(stamp),
        T_cam_pallet=[float(v) for v in T.reshape(-1)],
        det=float(np.linalg.det(T[:3, :3])),
        valid=valid, source=source,
        size_mm=[PALLET_W_MM, PALLET_H_MM],
        corners_uv=[_pt(u, v) for u, v in
                    [(680.0, 800.0), (960.0, 760.0), (980.0, 1080.0),
                     (700.0, 1120.0)]],
        n_used=1, n_slots=1, n_failed=0,
        spread_mm=0.0, spread_deg=0.0, latency_ms=12.0,
        diag="", rejects="")


def box_msg(stamp, du=0.0, valid=True, source="window"):
    return SimpleNamespace(
        header=_header(stamp),
        corners_uv=[_pt(u + du, v) for u, v in QUAD],
        valid=valid, source=source, spread_px=1.5, rejects="",
        n_used=5, n_slots=5, n_failed=0, angle_deg=1.05,
        box_uv=[_pt(680.0, 760.0), _pt(980.0, 1120.0)], latency_ms=66.0)


def _writer(*keys):
    client = py_trees.blackboard.Client(name="pairing_upstream")
    for key in keys:
        client.register_key(key=key, access=Access.WRITE)
    return client


def _make_nodes(**servo_over):
    pallet_node = NodePalletObs("pallet", "pallet", "ns",
                                {"pallet_key": "latest_pallet"})
    box_node = NodeBoxObs("box", "box", "ns", {"box_key": "latest_box_obs"})
    for node in (pallet_node, box_node):
        node._subscriber = object()
        node._setup_error = ""
    params = {"ref_edges": ["y=0", "x=W"],
              "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
              "K": [[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]],
              # ⚠️ 这副合成场景里**托盘跨到图外**：托盘系 x∈[0,1200]mm、相机高
              # 1m、FX=1000 → u∈[CX, CX+1200]=[640,1840]。与兄弟文件
              # `test_box_obs_to_servo_e2e.py` 用同一个图尺寸（3840×2400）。
              # （2026-09-30 之前图宽 1280 会让参考边两个端点都出图、被
              #  `edge_off_image` 拦下；那条判据已经删了，所以图宽现在只是
              #  "让整条边都看得见"的方便取值，不再是正确性的前提。）
              "image_size": [3840, 2400]}
    params.update(servo_over)
    servo = NodePalletServo("servo", "servo", "ns", params)
    # ★ 节点启动时是**未激活**的（要 call `/pallet_servo/slot` 才出数）。本文件
    #   这几条钉的是配对这条接缝，都在"激活之后"的语境里，所以这里先激活第 1 组
    #   —— 与 `test_node_pallet_servo.py` 的 `make_node` 同一处、同一个理由。
    #   各用例后面自己再调一次 `initialise()`（模拟 py_trees 重进）也不影响：
    #   激活状态在 `__init__` 里定，重进不会把它清掉。
    servo.initialise()
    servo._activate_slot(1)
    return pallet_node, box_node, servo


def _run(pairs, servo_over=None):
    """喂一串 (pallet_msg, box_msg)，返回最后一次的 ServoError（或 None）。"""
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp",
                     "latest_box_obs", "latest_box_obs_version")
    pallet_node, box_node, servo = _make_nodes(**(servo_over or {}))
    servo.initialise()
    out = None
    for pm, bm in pairs:
        if pm is not None:
            pallet_node._on_message(pm)
            assert pallet_node.update() == Status.RUNNING
        if bm is not None:
            box_node._on_message(bm)
            assert box_node.update() == Status.RUNNING
        assert servo.update() == Status.RUNNING, servo.feedback_message
        got = getattr(servo.global_blackboard, "latest_servo_error", None)
        if got is not None:
            out = got
    return servo, out


def test_same_stamp_pairs_and_produces_the_three_numbers():
    servo, err = _run([(pallet_msg(1.00), box_msg(1.00))])
    assert err is not None, f"没出数：{servo.feedback_message}"
    assert not isinstance(err, str), err
    assert math.isfinite(err.e_bottom_px) and math.isfinite(err.e_right_px)
    assert abs(err.theta_rad) > math.radians(1.0), \
        f"theta={math.degrees(err.theta_rad):.3f}° 太小，链路可能退化了"
    print(f"    三个量：e_bottom={err.e_bottom_px:+.1f}px "
          f"e_right={err.e_right_px:+.1f}px theta={math.degrees(err.theta_rad):+.2f}°")


def test_the_detection_cost_does_not_matter():
    """★ 耗时差不是问题：两个检测器看的是**同一帧图像**（stamp 相同）。

    托盘耗时 500ms、箱子 60ms —— 只要 stamp 一样，照样配上。
    （这里用同一对 stamp 模拟；真实场景里耗时差体现在**到达时刻**上，
    而 `_run` 是按到达顺序喂的。）
    """
    servo, err = _run([(pallet_msg(3.300), box_msg(3.300))])
    assert err is not None, servo.feedback_message
    assert not isinstance(err, str), err
    assert err.box_source == "quad"


def test_a_frame_apart_does_not_pair(caplog):
    """★ 差一帧（33ms）之外配不上 → 不出数，feedback 写实际差值。

    顺带钉住：**配对失败也是一帧"异常帧"**，`dump_on` 默认含 `"reject"`，
    所以它要留下一份能重跑的 dump（brief Step 10 的 4d）。
    """
    import logging
    with caplog.at_level(logging.WARNING):
        servo, err = _run([(pallet_msg(3.300), box_msg(3.400))])
    assert err is None, f"配不上却出了数：{err}"
    assert "pair_dt" in servo.feedback_message, servo.feedback_message
    assert "0.1" in servo.feedback_message, servo.feedback_message
    assert "托盘伺服写下 dump" in caplog.text, \
        f"配对失败没留下 dump：{caplog.text}"


def test_only_one_side_is_pair_none_not_a_crash():
    """只有一边有观测 → **不出数、不崩**。

    ⚠️ **偏离 brief Step 3 的一处断言**（有意）：brief 写的是
    `assert "pair_none" in servo.feedback_message`，但节点在配对**之前**就有一道
    更早的"等输入"闸（`pose is None or box is None`，改动前就有、Step 17/20 要求
    它继续工作），所以只有一边时**根本走不到配对**，feedback 报的是**更具体的**
    "哪个输入还缺"。`pair_none` 是**窗层**对这件事的判据 —— 两条都在这里钉住。
    """
    servo, err = _run([(pallet_msg(1.0), None)])
    assert err is None
    assert "等输入" in servo.feedback_message, servo.feedback_message
    assert "箱子 缺" in servo.feedback_message, servo.feedback_message

    # 窗层对"只有一边"的判据是 pair_none（不是崩、也不是静默配上）
    from skills.atomic.perception.pallet_frame import (
        PalletFrameWindow, PalletObservation, PairReject)
    window = PalletFrameWindow()
    window.push_pallet(PalletObservation(T_cam_pallet=straight_down(), stamp=1.0))
    only_pallet = window.resolve(now=0.0)
    assert isinstance(only_pallet, PairReject) and only_pallet.code == "pair_none", \
        repr(only_pallet)
    assert "托盘 1 个观测、箱子 0 个观测" in only_pallet.detail, only_pallet.detail


def test_the_window_averages_the_paired_frames():
    """★ 平滑用的是**同一批配对**：托盘与箱子各推 3 对，都进同一个窗。

    托盘 x = 0/10/20mm → 均值 10mm；箱子 du = 0/2/4 → 均值 2。
    如果两边各自开窗（而不是同一批配对），这两个均值不会同时成立。

    ⚠️ **两边都要断言**（评审证伪了只有箱子那一条的判别力）：只断言箱子四角的话，
    把 `pose = matrix_to_pose6d(paired.T_cam_pallet)` 去掉（托盘直接用黑板最新一帧、
    不平滑）**全套仍然全绿** —— 而"配对之后再开窗、两边同滞后"正是 Task 2 把窗放在
    配对之后的**全部理由**，这个设计主张此前没有任何测试钉住。
    托盘侧的数：台面中心在 `u = x_mm + 1240`（x=0/10/20 → 1240/1250/1260），
    窗平均 → **1250**；不平滑（用最后一帧）→ 1260。`e_right` 同向：878 vs 886。
    """
    pairs = [(pallet_msg(1.00, x_mm=0.0), box_msg(1.00, du=0.0)),
             (pallet_msg(1.10, x_mm=10.0), box_msg(1.10, du=2.0)),
             (pallet_msg(1.20, x_mm=20.0), box_msg(1.20, du=4.0))]
    servo, err = _run(pairs, servo_over={"window": 3})
    assert err is not None and not isinstance(err, str), servo.feedback_message
    # 箱子四角是逐点平均：第一个角 u 从 980 → 982
    assert abs(err.box_bottom_px[0][0] - 982.0) < 1e-6, err.box_bottom_px
    # 托盘侧同样是窗上的平均（不平滑的话是 1260.0 / 886.0）
    assert abs(err.pallet_center_px[0] - 1250.0) < 1e-6, err.pallet_center_px
    # ⚠️ 符号 2026-09-30 翻过：箱子在托盘内侧算**负**，所以这里是 −878。
    assert abs(err.e_right_px + 878.0) < 1e-6, err.e_right_px


def test_window_1_is_no_smoothing():
    pairs = [(pallet_msg(1.00, x_mm=0.0), box_msg(1.00, du=0.0)),
             (pallet_msg(1.10, x_mm=10.0), box_msg(1.10, du=2.0))]
    servo, err = _run(pairs, servo_over={"window": 1})
    assert err is not None and not isinstance(err, str), servo.feedback_message
    assert abs(err.box_bottom_px[0][0] - 982.0) < 1e-6, \
        f"window=1 该用最后一帧，实际 {err.box_bottom_px}"


def test_the_same_pair_is_not_recomputed_every_tick():
    """同一对不重复出数 —— 节点每 tick 都调 `update()`。"""
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp",
                     "latest_box_obs", "latest_box_obs_version")
    pallet_node, box_node, servo = _make_nodes()
    servo.initialise()
    pallet_node._on_message(pallet_msg(1.0))
    pallet_node.update()
    box_node._on_message(box_msg(1.0))
    box_node.update()
    assert servo.update() == Status.RUNNING
    v1 = getattr(servo.global_blackboard, "latest_servo_error_version")
    for _ in range(5):
        assert servo.update() == Status.RUNNING
    v2 = getattr(servo.global_blackboard, "latest_servo_error_version")
    assert v1 == v2, f"同一对被重算了 {v2 - v1} 次"


def test_a_stale_pallet_is_rejected_once_it_ages_out(monkeypatch):
    """★ 整条链路都停住时（两个观测都旧）→ `stale`，**不靠外推救**。

    喂一对（正常出数），再喂**新的一对**但把"现在"推到很后面 —— 新的一对
    stamp 仍然互相配得上，但已经比"现在"旧太多了。

    ⚠️ 注意 `resolve()` 的**去重在前、stale 在后**：拿同一对反复调只会得到
    `None`（"没有新的配对"），不会得到 `stale`。所以这里必须喂**新的一对**。

    ⚠️ **偏离 brief Step 3 的地方（有意）**：brief 那份把"现在"交给 `time.time()`，
    而场景里的 stamp 是 `1.0` / `2.0` 这种**假时刻** —— `stale_s` 比的是
    `now - pallet.stamp`，两个数**必须来自同一个时钟**才有意义（addendum A4 明确
    要求先把 `now` 的口径定死）。所以这里把节点的时钟换成**可控的接缝**
    `_now_sec()`：场景与断言（age = 10.0 − 2.0 = 8.0）与 brief 完全一致。
    """
    clock = {"t": 1.0}
    monkeypatch.setattr(NodePalletServo, "_now_sec", lambda self: clock["t"])
    pallet_node, box_node, servo = _make_nodes(stale_s=0.05)
    servo.initialise()

    pallet_node._on_message(pallet_msg(1.0))
    pallet_node.update()
    box_node._on_message(box_msg(1.0))
    box_node.update()
    assert servo.update() == Status.RUNNING
    assert getattr(servo.global_blackboard, "latest_servo_error", None) is not None, \
        "第一对在 stale 之前就该出数"

    # 新的一对（stamp 互相配得上），但"现在"已经跑到很后面了
    clock["t"] = 10.0
    pallet_node._on_message(pallet_msg(2.0))
    pallet_node.update()
    box_node._on_message(box_msg(2.0))
    box_node.update()
    assert servo.update() == Status.RUNNING
    assert "stale" in servo.feedback_message, servo.feedback_message
    assert "8.0" in servo.feedback_message, servo.feedback_message

    # 陈旧帧**不写黑板**：版本停在上一帧
    assert getattr(servo.global_blackboard, "latest_servo_error_version") == 1, \
        "陈旧帧不该覆盖黑板上的三个量"


def test_stale_is_a_delay_gate_not_a_liveness_gate(monkeypatch):
    """★ `stale_s` 是**延迟闸**不是**存活闸**（addendum A4）—— 两者别混。

    一侧停摆（不再有新观测）时 `resolve()` 返回的是 `None`，**永远轮不到**
    `stale`：`stale` 判在**去重之后**，已消费的那一对根本走不到它。所以"检测器
    挂了"不能靠 `stale_s` 报出来 —— 那是 `live_timeout_s` 的活。

    这条把两个量的**分工**钉住：同一个停摆场景，`stale_s` 调到很小也一声不响，
    而存活闸喊了。
    """
    clock = {"t": 1.0}
    monkeypatch.setattr(NodePalletServo, "_now_sec", lambda self: clock["t"])
    pallet_node, box_node, servo = _make_nodes(stale_s=1e-6, live_timeout_s=0.5)
    servo.initialise()

    pallet_node._on_message(pallet_msg(1.0))
    pallet_node.update()
    box_node._on_message(box_msg(1.0))
    box_node.update()
    assert servo.update() == Status.RUNNING
    assert "配不成对" not in servo.feedback_message, servo.feedback_message

    # 时钟往前跑很远，但**两路都不再有新观测** → stale 一声不响，存活闸喊
    clock["t"] = 100.0
    servo._last_pallet_seen -= 10.0
    servo._last_box_seen -= 10.0
    assert servo.update() == Status.RUNNING
    assert "配不成对" not in servo.feedback_message, (
        f"停摆判成了 stale —— 那是延迟闸，判不到'检测器挂了'："
        f"{servo.feedback_message}")
    assert "停摆" in servo.feedback_message, servo.feedback_message


def test_reinitialise_resets_the_pairing_window():
    """★ 重进 `initialise()` 必须 `reset()` 配对窗：**时钟整体回跳之后不许静默**。

    `_last_key` 是"已经消费掉的观测"的水位线（bag 循环播放 / 换时间源之后新观测的
    stamp 全都**小于**它），`resolve()` 于是永久返回 `None` —— 伺服黑板停在回跳前
    的值上，**一个错都不报**。这条不变量此前没有网；漏掉 `reset()` 时本用例红。
    """
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp",
                     "latest_box_obs", "latest_box_obs_version")
    pallet_node, box_node, servo = _make_nodes()
    servo.initialise()

    def feed(pallet_stamp, box_stamp):
        pallet_node._on_message(pallet_msg(pallet_stamp))
        assert pallet_node.update() == Status.RUNNING
        box_node._on_message(box_msg(box_stamp))
        assert box_node.update() == Status.RUNNING
        return servo.update()

    assert feed(5.0, 5.0) == Status.RUNNING
    v1 = getattr(servo.global_blackboard, "latest_servo_error_version", 0)

    # bag 从头再放一遍：同一批 stamp **更小**了，水位线不清就永远配不出数
    servo.initialise()
    assert feed(1.0, 1.0) == Status.RUNNING
    v2 = getattr(servo.global_blackboard, "latest_servo_error_version", 0)
    assert v2 == v1 + 1, (
        f"重进 initialise() 之后配不出数了（版本停在 {v1}→{v2}）—— "
        f"`_last_key` 的水位线没清，回跳后的观测全被当成'旧的'："
        f"{servo.feedback_message}")


def test_valid_false_pallet_never_reaches_the_servo():
    """★ `valid=false` 的托盘帧**到不了伺服**：黑板没写，伺服停在"等输入"。

    ⚠️ 与上一条同一处偏离：brief 期望 `pair_none`，实际停在更早的"等输入"
    （托盘侧压根没有值）。断言的是**同一件事**：这一帧没有被当成有效观测、
    一个数都出不来。
    """
    servo, err = _run([(pallet_msg(1.0, valid=False), box_msg(1.0))])
    assert err is None
    assert "等输入" in servo.feedback_message, servo.feedback_message
    assert "托盘 缺" in servo.feedback_message, servo.feedback_message
