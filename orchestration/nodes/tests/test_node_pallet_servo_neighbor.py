# -*- coding: utf-8 -*-
"""`NodePalletServo` 的**按档位分流**（`servo_slots`）单元测试。

1、2 箱走托盘路线、3~5 箱走邻居路线这件事，分两个层次钉住：

* **`servo_slots` 的解析**（缺省 = 全 pallet、非法值 = 启动时配置错误）；
* **`update()` 的两条分流路径** —— 尤其是那个卡点：`latest_pallet` 为 `None`
  时，`pallet` 档必须照旧早退「等输入」，而 `neighbor` 档**必须仍然出数**
  （托盘被手里那个箱子挡死正是邻居路线的场景，见 `neighbor_merge_spec.md` §8.1）。

邻居场景的几何（箱子摆在哪、谁是谁的邻居）与
`skills/atomic/perception/pallet_servo/tests/test_neighbor.py` 同一套合成布局：
真实尺寸 530×350mm、左列长边横放、右列长边竖放。这里**只借用布局**，判据是
节点层的（出不出数、走哪条路、reason 是什么），算法层的判据归那个文件。

跑法::

    pytest orchestration/nodes/tests/test_node_pallet_servo_neighbor.py -m unit -v
"""
import math
from pathlib import Path

import numpy as np
import py_trees
import pytest
from py_trees.common import Status

from orchestration.nodes.node_pallet_servo import (
    NodePalletServo,
    _NEIGHBOR_SIDES,
    _normalize_servo_slots,
    _slot_bias_of,
    _slot_mode_of,
)
# 夹具与假件**直接复用**既有那个测试文件里的（`py_trees` 黑板是进程级单例，
# 两处各造一份"假上游"只会分叉）。它是同目录下的兄弟模块，按模块名导入即可。
from orchestration.nodes.tests.test_node_pallet_servo import (
    IMAGE_SIZE,
    K_PARAM,
    PALLET_H_MM,
    PALLET_W_MM,
    _FakePublisher,
    _FakeServoPub,
    _fake_ros_modules,
    _fake_servo_msgs,
    make_node,
)
from skills.atomic.perception.pallet_servo import neighbor as neighbor_algo
from skills.atomic.perception.pallet_servo.algorithm import BoxObservation

pytestmark = pytest.mark.unit


# --------------------------------------------------------------------------- #
# 邻居场景：与 test_neighbor.py 的合成布局逐数相同（真实尺寸 530x350mm）
# --------------------------------------------------------------------------- #
BOX_LONG_MM, BOX_SHORT_MM = 530.0, 350.0
PX_PER_MM = 0.63
L_LONG = BOX_LONG_MM * PX_PER_MM
L_SHORT = BOX_SHORT_MM * PX_PER_MM
_X_L = (335.0, 335.0 + L_LONG)
_X_R = (_X_L[1], _X_L[1] + L_SHORT)
_V_BOT = 680.0
_ROWS = {'5': (_V_BOT - 3 * L_SHORT, _V_BOT - 2 * L_SHORT),
         '2': (_V_BOT - 2 * L_SHORT, _V_BOT - 1 * L_SHORT),
         '3': (_V_BOT - 1 * L_SHORT, _V_BOT),
         '1': (_V_BOT - 1 * L_LONG, _V_BOT),
         '4': (_V_BOT - 2 * L_LONG, _V_BOT - 1 * L_LONG)}
_COLS = {'L': _X_L, 'R': _X_R}
_PLACE = {3: ('L', '3'), 2: ('L', '2'), 5: ('L', '5'),
          1: ('R', '1'), 4: ('R', '4')}


def _rect(x0, y0, x1, y1) -> np.ndarray:
    """契约序四角（右下 左下 左上 右上），与 `algorithm.box_corners` 的 AABB 同序。"""
    return np.array([[x1, y1], [x0, y1], [x0, y0], [x1, y0]], float)


def _layout():
    return {k: _rect(_COLS[c][0], _ROWS[r][0], _COLS[c][1], _ROWS[r][1])
            for k, (c, r) in _PLACE.items()}


def _candidate_scene(held_key=3):
    """`(cands, box)`：候选列表 + 手里箱（**两个对象，内容相同**）。

    ⚠️ 与真实链路一样**故意不给同一个对象**：黑板上的 `latest_box_obs` 与
    `latest_box_candidates` 里对应的那个是 `node_box_obs` 分别造的（见接入契约
    §2/§5），所以 `held is cands[i]` 恒为假 —— 节点必须自己按四角找回来。
    这条正是 `_nearest_candidate` 存在的理由，用同一个对象测就把它绕过去了。
    """
    lay = _layout()
    cands = [BoxObservation(quad=q.tolist(), confidence=0.9)
             for _k, q in sorted(lay.items())]
    box = BoxObservation(quad=lay[held_key].tolist(), confidence=0.9)
    return cands, box


def _writer(*keys):
    writer = py_trees.blackboard.Client(name="test_neighbor_upstream")
    for key in keys:
        writer.register_key(key=key, access=py_trees.common.Access.WRITE)
    return writer


def _servo_slots(sides=("right", "none", "left")):
    """1、2 走托盘、3/4/5 走邻居 —— 与 `board.json` 里那份同形。"""
    return ([{"slot": 1, "mode": "pallet"}, {"slot": 2, "mode": "pallet"}]
            + [{"slot": i + 3, "mode": "neighbor", "side": s}
               for i, s in enumerate(sides)])


def _bare_node(tmp_path, **over):
    """不走 `make_node`（它会 `_activate_slot(1)`，而配置错误那几条要的是原始状态）。"""
    params = {
        "ref_edges": [["y=0", "x=W"]] * 5,
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM, "image_size": IMAGE_SIZE,
        "dump_dir": str(tmp_path / "dump"), "pallet_frame": "camera",
    }
    params.update(over)
    return NodePalletServo("servo", "servo", "ns", params)


# --------------------------------------------------------------------------- #
# 1. `servo_slots` 的解析
# --------------------------------------------------------------------------- #
def test_servo_slots_defaults_to_all_pallet():
    """★ 缺省 / 空数组 → 全部 `("pallet", "none", 0, 0)`（与加这个键之前一致）。"""
    for raw in (None, [], (), ""):
        modes, err = _normalize_servo_slots(raw)
        assert err is None, err
        assert modes == [("pallet", "none", 0.0, 0.0)], modes


def test_servo_slots_parses_the_board_shape():
    modes, err = _normalize_servo_slots(_servo_slots())
    assert err is None, err
    assert modes == [("pallet", "none", 0.0, 0.0),
                     ("pallet", "none", 0.0, 0.0),
                     ("neighbor", "right", 0.0, 0.0),
                     ("neighbor", "none", 0.0, 0.0),
                     ("neighbor", "left", 0.0, 0.0)]


def test_servo_slots_orders_by_slot_number():
    """表项的顺序就是档位 —— 乱序写也要按 `slot` 排回来（错位不许靠运气）。"""
    modes, err = _normalize_servo_slots([
        {"slot": 3, "mode": "neighbor", "side": "left"},
        {"slot": 1, "mode": "pallet"},
        {"slot": 2, "mode": "neighbor", "side": "right"}])
    assert err is None, err
    assert modes == [("pallet", "none", 0.0, 0.0),
                     ("neighbor", "right", 0.0, 0.0),
                     ("neighbor", "left", 0.0, 0.0)]


def test_servo_slots_parses_output_biases():
    """★ 每档可选的 `e_right_bias_px` / `e_bottom_bias_px` —— 默认都 0，越界也 0。"""
    modes, err = _normalize_servo_slots([
        {"slot": 1, "mode": "pallet"},
        {"slot": 2, "mode": "neighbor", "side": "right",
         "e_right_bias_px": -8.0, "e_bottom_bias_px": 3.0},
        {"slot": 3, "mode": "neighbor", "side": "left",
         "e_bottom_bias_px": -2.5}])
    assert err is None, err
    assert modes == [("pallet", "none", 0.0, 0.0),
                     ("neighbor", "right", -8.0, 3.0),
                     ("neighbor", "left", 0.0, -2.5)]
    assert _slot_bias_of(modes, 0) == 0.0
    assert _slot_bias_of(modes, 1) == -8.0
    assert _slot_bias_of(modes, 1, "bottom") == 3.0
    assert _slot_bias_of(modes, 2, "bottom") == -2.5
    assert _slot_bias_of(modes, 99) == 0.0          # 越界兜底
    assert _slot_bias_of(modes, 99, "bottom") == 0.0
    assert _slot_bias_of([("neighbor", "right")], 0) == 0.0   # 二元组兼容
    with pytest.raises(ValueError):
        _slot_bias_of(modes, 1, "up")


@pytest.mark.parametrize("key", ["e_right_bias_px", "e_bottom_bias_px"])
@pytest.mark.parametrize("bad", ["-8px", "", None, float("nan"), float("inf")])
def test_servo_slots_bad_bias_is_an_error(key, bad):
    """静默按 0 跑 = "配了没生效"，现场只能从机械臂停的位置倒推。"""
    modes, err = _normalize_servo_slots([
        {"slot": 1, "mode": "neighbor", "side": "right", key: bad}])
    assert modes == [] and err is not None and key in err, err


@pytest.mark.parametrize("bad", [
    [{"slot": 1, "mode": "pallete"}],          # 拼错
    [{"slot": 1, "mode": "NEIGHBOR-x"}],
    [{"slot": 1, "mode": ""}],
    [{"slot": 1, "mode": "托盘"}],
])
def test_servo_slots_bad_mode_is_an_error(bad):
    modes, err = _normalize_servo_slots(bad)
    assert err is not None and "mode" in err, (modes, err)
    assert "pallet/neighbor" in err, err


@pytest.mark.parametrize("bad", [
    [{"slot": 1, "mode": "neighbor", "side": "up"}],
    [{"slot": 1, "mode": "neighbor", "side": "Right"}],   # 大写**不认**（与
    [{"slot": 1, "mode": "neighbor", "side": "RIGHT"}],   #  NeighborSlot 同一口径）
    [{"slot": 1, "mode": "neighbor", "side": ""}],
])
def test_servo_slots_bad_side_is_an_error(bad):
    modes, err = _normalize_servo_slots(bad)
    assert err is not None and "side" in err, (modes, err)
    assert "left/right/none" in err, err


@pytest.mark.parametrize("bad", ["PALLET", "Neighbor", "PALLET "])
def test_servo_slots_mode_is_case_sensitive(bad):
    """`mode` 的大小写也**不放宽**：与 `side` 同一条纪律（静默的宽容 = 少一层保护）。"""
    modes, err = _normalize_servo_slots([{"slot": 1, "mode": bad}])
    assert modes == [] and err is not None, (modes, err)


def test_servo_slots_non_dict_item_is_an_error():
    modes, err = _normalize_servo_slots([["pallet"]])
    assert modes == [] and err is not None and "不是对象" in err, err


def test_servo_slots_side_whitelist_matches_neighbor():
    """节点侧那份白名单与 `neighbor.NeighborSlot` 的**必须一致**。

    两边各有一份是有意的（节点那份要在启动时说出"第几档"，`NeighborSlot` 抛的
    `ValueError` 里没有档位），代价就是可能分叉 —— 这条用例就是那道闸。
    """
    assert tuple(_NEIGHBOR_SIDES) == tuple(neighbor_algo._SIDES), (
        _NEIGHBOR_SIDES, neighbor_algo._SIDES)


def test_slot_mode_of_falls_back_to_pallet_past_the_end():
    """表比 `ref_edges` 短 = "后面几档都是老路线"，按 pallet 兜底（不是报错）。"""
    assert _slot_mode_of([("neighbor", "right")], 0) == ("neighbor", "right")
    assert _slot_mode_of([("neighbor", "right")], 1) == ("pallet", "none")
    assert _slot_mode_of([], 3) == ("pallet", "none")


# --------------------------------------------------------------------------- #
# 2. 非法 `servo_slots` → **启动时**的配置错误
# --------------------------------------------------------------------------- #
def test_a_bad_mode_is_a_startup_config_error(tmp_path, monkeypatch):
    """★ 静默按 pallet 跑是最难查的一类错：本该走邻居的档会拿"托盘看不见"的
    位姿去算，而**照样算得出三个数**。所以必须在启动时就说。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = _bare_node(tmp_path, servo_slots=[
        {"slot": 1, "mode": "pallet"},
        {"slot": 2, "mode": "neighbor", "side": "middle"}])
    node.initialise()

    assert node._config_err is not None, "非法 side 该在 initialise 就报错"
    assert "side" in node._config_err and "middle" in node._config_err, \
        node._config_err
    assert node.feedback_message.startswith("配置错误 ——"), node.feedback_message
    # 配置错误那条路恒 FAILURE，且**不会**出数
    assert node.update() == Status.FAILURE


def test_a_short_servo_slots_table_is_legal(tmp_path, monkeypatch):
    """表比 `ref_edges` 短是**合法**的（"后面几档都是老路线"），不报错。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = _bare_node(tmp_path, servo_slots=[{"slot": 1, "mode": "pallet"}])
    node.initialise()
    assert node._config_err is None, node._config_err
    assert node.feedback_message.startswith("就绪："), node.feedback_message


def test_pallet_slot_without_ref_edges_is_a_config_error(tmp_path, monkeypatch):
    """★ **pallet 档必须有 `ref_edges`**（2026-10-17 修）——那才是配置错误。

    上一版对"`servo_slots` 更长"是**截掉 + 一条 WARNING**，那是个功能死角：
    `_activate_slot` 按 `len(ref_edges)` 算总数，于是 `ref_edges` 4 组 +
    `servo_slots` 5 档时，**第 5 档永远切不过去**（真机实测报"表里只有 4 组"），
    而它本该走邻居。判据不该是"两边一样长"，而是**逐档**看：
    `pallet` 档没有参考边 → 错；`neighbor` 档不要参考边 → 合法。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    # 2 组 ref_edges、3 档表，且第 3 档是 pallet -> 错
    node = _bare_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 2,
                      servo_slots=[{"slot": 1, "mode": "pallet"},
                                   {"slot": 2, "mode": "pallet"},
                                   {"slot": 3, "mode": "pallet"}])
    node.initialise()
    assert node._config_err, "pallet 档缺 ref_edges 必须报配置错误"
    assert "pallet" in node._config_err and "第 3 档" in node._config_err, \
        node._config_err


def test_neighbor_slots_beyond_ref_edges_are_legal_and_reachable(tmp_path,
                                                                monkeypatch):
    """★ **`servo_slots` 比 `ref_edges` 长、多出来的是 neighbor → 合法**，
    且**每一档都切得过去**（含最后一档）。

    这正是真机那份板子的形状（4 组 ref_edges + 5 档表，第 5 档是 neighbor）——
    上一版会把第 5 档截掉，于是它永远切不过去。两条一起钉：
    ① 总档位按两者取大；② 超出 `ref_edges` 的那几档**不许**再去索引 `self._slots`。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = _bare_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 4,
                      servo_slots=_servo_slots()[:3] + [
                          {"slot": 4, "mode": "neighbor", "side": "none"},
                          {"slot": 5, "mode": "neighbor", "side": "left"}])
    node.initialise()
    assert node._config_err is None, node._config_err
    for n in (1, 2, 3, 4, 5):
        ok, msg = node._activate_slot(n)
        assert ok, f"第 {n} 档切不过去：{msg}"
    ok, msg = node._activate_slot(6)
    # ⚠️ 文案说**档**不说"组"：档位数与 `ref_edges` 组数不再相等，说"组"会把
    #    人引去数错那张表 —— 真机那次的误判正是被这句话引出来的。
    assert not ok and "只有 5 档" in msg, msg
    # 邻居档的回复要带 mode/side，且**不撒谎**说它有什么参考边
    ok, msg = node._activate_slot(5)
    assert "neighbor" in msg and "left" in msg, msg
    assert "不用参考边" in msg, msg


# --------------------------------------------------------------------------- #
# 3. 邻居档：`latest_pallet` 为 None 也要出数（§8.1 那个卡点）
# --------------------------------------------------------------------------- #
def test_neighbor_route_does_not_recompute_the_same_input(tmp_path, monkeypatch):
    """★ **同一份输入只算一次**（2026-10-17，真机看到 50Hz）。

    行为树主循环固定 **50Hz**；托盘路线靠配对窗天然去重，而邻居路线从前是
    **每个 tick 都重算同一个框** —— 相机只有 30Hz、检测器更慢，多出来全是空转。
    现在用 `(箱子版本, 候选版本)` 当门禁：版本没变就**不算、不发布**。

    ⚠️ 版本变了必须**重新算**（不能把"没变"判成永远重复）。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", cands)
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(3)

    assert node.update() == Status.RUNNING
    n_after_first = pub.n
    assert n_after_first == 1, "第一帧该出一条有效数"
    err1 = getattr(node.global_blackboard, "latest_servo_error", None)
    v1 = getattr(node.global_blackboard, "latest_servo_error_version", 0)

    # 同版本再 tick 两次：一次都不能重算、不能重发
    for _ in range(2):
        assert node.update() == Status.RUNNING
    assert pub.n == n_after_first, (
        f"同一份输入被重算了：{n_after_first} -> {pub.n}（50Hz 空转的来源）")
    assert getattr(node.global_blackboard, "latest_servo_error", None) is err1
    assert getattr(node.global_blackboard, "latest_servo_error_version", 0) == v1

    # 版本 +1 => 必须重新算
    writer.set("latest_box_obs_version", 2)
    writer.set("latest_box_candidates_version", 2)
    assert node.update() == Status.RUNNING
    assert pub.n == n_after_first + 1, "新输入到了必须重算，不能被去重卡死"


def test_neighbor_slot_outputs_even_without_a_pallet_pose(tmp_path, monkeypatch):
    """★★ **本组最要紧的一条**：托盘被挡死（`latest_pallet is None`）时，
    邻居档必须照旧出一条有效数 —— 而不是卡在「等输入」上。

    参考实现（改动前）那道闸是 `if pose is None or box is None:`，邻居档会**永远**
    停在那儿：现场表现是"call 了服务、话题上一直是等输入"，看起来完全像个正常的
    "上游还没出数"。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    # 手里那个箱子的观测仍在（这一路检测没坏），托盘位姿**没有**
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", cands)
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(3)

    status = node.update()

    assert status == Status.RUNNING
    assert "等输入" not in node.feedback_message, node.feedback_message
    err = getattr(node.global_blackboard, "latest_servo_error", None)
    assert err is not None, (
        f"邻居档在 latest_pallet=None 时没出数（feedback={node.feedback_message!r}）")
    # 三个被控量是**邻居路线**给的：这一帧是理想摆放，三个量都该 ≈ 0。
    assert abs(err.e_bottom_px) < 4.0, err.e_bottom_px
    assert abs(err.e_right_px) < 3.0, err.e_right_px
    assert abs(math.degrees(err.theta_rad)) < 0.5, math.degrees(err.theta_rad)
    # 出处要能分辨：图上/日志上得看得出这一帧是哪套算的
    assert err.box_source == "neighbor", err.box_source
    assert any(w.startswith("source=neighbor") for w in err.warn), err.warn


def test_neighbor_slot_works_without_any_configured_size(tmp_path, monkeypatch):
    """★ 主流程删掉 `place_servo_pallet_size_mm` 后，neighbor 档必须照常出数。

    尺寸现在只从检测器上报的 `latest_pallet_size_mm` 来；托盘被挡死时这个键
    可能一直是空的，而 neighbor 路线**本来就不需要尺寸** —— 不能因为
    `_size is None` 把整档卡住。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", cands)
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0, pallet_size_mm=None)
    assert node._config_err is None, node._config_err
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(3)

    assert node.update() == Status.RUNNING
    err = getattr(node.global_blackboard, "latest_servo_error", None)
    assert err is not None, node.feedback_message
    assert err.box_source == "neighbor", err.box_source
    assert pub.n >= 1 and pub.last is not None


def test_output_biases_are_applied_before_publishing(tmp_path, monkeypatch):
    """★ 档位偏置（`e_right_bias_px` / `e_bottom_bias_px`）：**new = old + bias**，
    而且加在写黑板 / 发话题**之前** —— 下游与叠加图看到的是同一个量。

    理想摆放的原始 `e_right` / `e_bottom` ≈ 0（既有用例钉死在 ±3/±4px 内）；
    配 `e_right -8` 后应当报 ≈ -8，配 `e_bottom -30` 后应当报 ≈ -30。它不是
    "误差修正"，而是**目标偏置**：控制器把该量驱到 0，于是实际差停在偏置处。
    `theta` 不受影响。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", cands)
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    slots = _servo_slots()
    slots[2]["e_right_bias_px"] = -8.0             # 第 3 档（邻居 right）
    slots[2]["e_bottom_bias_px"] = -30.0
    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=slots, pallet_frame="camera",
                     max_dt_s=1.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(3)

    assert node.update() == Status.RUNNING
    err = getattr(node.global_blackboard, "latest_servo_error", None)
    assert err is not None, node.feedback_message
    assert -11.0 < err.e_right_px < -5.0, err.e_right_px
    assert -34.0 < err.e_bottom_px < -26.0, err.e_bottom_px
    assert abs(math.degrees(err.theta_rad)) < 0.5, math.degrees(err.theta_rad)
    # 话题上发的必须是同一个（加过偏置的）量
    assert pub.last is not None
    assert pub.last.e_right_px == pytest.approx(err.e_right_px)
    assert pub.last.e_bottom_px == pytest.approx(err.e_bottom_px)
    # 同一张表里没配偏置的档位不受影响（slot 1 = pallet，bias 缺省 0）
    assert _slot_bias_of(node._slot_modes, 0) == 0.0


def test_neighbor_frame_fills_the_diagnostics_and_leaves_empty_arrays(
        tmp_path, monkeypatch):
    """诊断量：手里箱的两条边、上邻居底边、侧邻居近边；**没有侧邻居时留空数组**。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    # 第 4 档 `side="none"` —— 这一档本来就没有侧邻居
    cands, box = _candidate_scene(held_key=4)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", cands)
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(4)
    assert node.update() == Status.RUNNING

    err = getattr(node.global_blackboard, "latest_servo_error", None)
    assert err is not None, node.feedback_message
    # 手里箱自己那两条边（`box_edges` 的定义）与**上邻居的底边**都填上了
    assert np.asarray(err.box_bottom_px).shape == (2, 2)
    assert np.asarray(err.box_right_px).shape == (2, 2)
    assert np.asarray(err.ref_bottom_px).shape == (2, 2)
    assert np.asarray(err.bottom_inward_px).shape == (2,)
    assert np.asarray(err.bottom_foot_px).shape == (2,)
    # 侧邻居不存在 → **空数组**（不是编一个数、也不是 None）
    assert np.asarray(err.ref_right_px).size == 0, err.ref_right_px
    assert np.asarray(err.right_inward_px).size == 0, err.right_inward_px
    # 上邻居的中心给了（给 render 判内法向用）
    assert np.asarray(err.pallet_center_px).shape == (2,)


def test_neighbor_slot_uses_the_candidates_not_the_pallet_route(
        tmp_path, monkeypatch):
    """邻居档**不许**走托盘路线：托盘位姿与箱子观测都齐、`ref_edges` 也合法时，
    出的仍是 `box_source="neighbor"` 的那一套。

    少了这条，"邻居档退化成托盘档"会**全绿** —— 两条路都能出数、三个量同名，
    而基准完全不同。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    from core.common.transform import matrix_to_pose6d
    from orchestration.nodes.tests.test_node_pallet_servo import (
        straight_down_camera)

    cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", cands)
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", matrix_to_pose6d(straight_down_camera()))
    writer.set("latest_pallet_version", 1)
    writer.set("latest_pallet_stamp", 100.0)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(3)

    assert node.update() == Status.RUNNING
    err = getattr(node.global_blackboard, "latest_servo_error", None)
    assert err is not None, node.feedback_message
    assert err.box_source == "neighbor", err.box_source
    # 配对那一套的版本号在邻居路线上**没有意义** → 留 -1，不假装配对过
    assert err.pallet_version == -1 and err.box_version == -1, (
        err.pallet_version, err.box_version)


# --------------------------------------------------------------------------- #
# 4. pallet 档：`latest_pallet` 为 None 时**照旧**早退「等输入」（行为不变）
# --------------------------------------------------------------------------- #
def test_pallet_slot_still_waits_when_the_pallet_pose_is_missing(
        tmp_path, monkeypatch):
    """★ 与上面对称的那一条：`pallet` 档的行为**一个字节都不许变**。

    托盘路线非有托盘位姿不可（三个量全部由台面参考边的投影给出），所以这一档
    仍然要早退，文案也仍然是原来那句。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    # 候选列表**给足**（邻居档本来能出数）—— 这一条要证明的就是 pallet 档不看它
    writer.set("latest_box_candidates", cands)
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(1)                 # ★ 第 1 档 = pallet

    assert node.update() == Status.RUNNING
    assert "等输入" in node.feedback_message, node.feedback_message
    assert "托盘 缺" in node.feedback_message, node.feedback_message
    assert "箱子 有" in node.feedback_message, node.feedback_message
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 0
    # 无效帧照发（心跳承诺），原文与改动前逐字相同
    assert pub.last is not None and pub.last.valid is False
    assert pub.last.reject == "等输入（托盘 缺，箱子 有）", pub.last.reject


def test_pallet_slot_with_no_servo_slots_key_behaves_as_before(tmp_path,
                                                              monkeypatch):
    """★ 没配 `servo_slots` 时**行为与改动前完全一致**：同样的输入、同样的出口。"""
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    # 黑板是**进程级**的，前面的用例早写过这些键了 —— 这条要的恰恰是"一个输入
    # 都没有"（与本文件既有用例同一句先例）。
    py_trees.blackboard.Blackboard.clear()
    node = make_node(tmp_path, pallet_frame="camera")   # 不传 servo_slots
    pub = _FakeServoPub()
    node._servo_error_pub = pub

    assert node.update() == Status.RUNNING
    assert node.feedback_message == "等输入（托盘 缺，箱子 缺）", \
        node.feedback_message


# --------------------------------------------------------------------------- #
# 5. 邻居档拿不到候选 → `_publish_invalid`，reason 是 `neighbor_*`
# --------------------------------------------------------------------------- #
def test_neighbor_slot_without_candidates_publishes_an_invalid_frame(
        tmp_path, monkeypatch):
    """候选列表空着（`node_box_obs` 还没出东西）→ 无效帧 + `neighbor_*` 的原因。

    **不许**退回托盘路线（操作员 2026-10-17 裁定）：拒了就拒了 —— 悄悄换成另一套
    基准会让三个数看着一样像模像样。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    _cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", [])          # ★ 空列表，不是 None
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(3)

    assert node.update() == Status.RUNNING
    assert pub.last is not None, "无效帧要照发"
    assert pub.last.valid is False
    assert pub.last.reject.startswith("neighbor_"), pub.last.reject
    assert math.isnan(pub.last.e_bottom_px), "无效帧的三个量必须是 NaN"
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None
    assert getattr(node.global_blackboard, "latest_servo_error_version") == 0


def test_neighbor_slot_no_up_neighbor_publishes_the_reject_reason(
        tmp_path, monkeypatch):
    """候选**在**、但挑不出上邻居（只剩手里箱自己）→ 同样是 `neighbor_*`。

    这一条走的是 `neighbor_servo` 自己那条 `neighbor_no_up`（不是节点层那句
    "候选空着"），所以 reason 的原文来自算法层 —— 整条链路的失败模式要一致。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    _cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    # 候选里只有手里箱自己（真实链路里"先放头一个箱子"那一帧就是这个形状）
    writer.set("latest_box_candidates", [BoxObservation(
        quad=box.quad, confidence=0.9)])
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(3)

    assert node.update() == Status.RUNNING
    assert pub.last is not None and pub.last.valid is False
    assert pub.last.reject.startswith("neighbor_no_up"), pub.last.reject


def test_neighbor_slot_without_any_box_still_waits(tmp_path, monkeypatch):
    """箱子观测本身没有 → 「等输入」，但文案要说清是**邻居路线**缺箱子。

    照抄托盘那句会写"托盘 缺" —— 现场会去查托盘检测器，而那条路本来就没坏。
    """
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    _cands, _box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", None)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", [])
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0)
    pub = _FakeServoPub()
    node._servo_error_pub = pub
    node._activate_slot(3)

    assert node.update() == Status.RUNNING
    assert "等输入" in node.feedback_message, node.feedback_message
    assert "邻居路线" in node.feedback_message, node.feedback_message
    assert "托盘" not in node.feedback_message, node.feedback_message
    assert getattr(node.global_blackboard, "latest_servo_error", None) is None


# --------------------------------------------------------------------------- #
# 6. 两条路线之外：邻居档的帧不能被托盘那一套的残留污染
# --------------------------------------------------------------------------- #
def test_a_neighbor_frame_does_not_reuse_the_last_pallet_pose(tmp_path,
                                                              monkeypatch):
    """★ 邻居帧出图时**不许**拿上一帧托盘路线留下的 `T_cam_pallet`。

    那会让图上凭空多出一个与这一帧毫无关系、却画得规规矩矩的台面矩形 ——
    现场据此判断"托盘还在视野里"，而其实这一帧根本没有托盘位姿。
    """
    import skills.atomic.perception.pallet_servo.render as render_mod

    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", cands)
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0, overlay_period_s=0.0)
    node._activate_slot(3)

    seen = {}
    # ⚠️ 邻居路线走的是 `render_neighbor_overlay`（2026-10-17 起与托盘那套分家，
    #    理由见那个函数的 docstring）—— 这条用例要钉的是"邻居帧不用上一帧的托盘
    #    位姿去画台面"，所以**盯的就是邻居那个入口**。盯错函数会让这条用例
    #    静默地什么都没测（`seen` 一直是空 dict）。
    real_render = render_mod.render_neighbor_overlay

    def _capture(*args, **kwargs):
        seen.update(kwargs)
        return real_render(*args, **kwargs)

    monkeypatch.setattr(render_mod, "render_neighbor_overlay", _capture)
    node._overlay_pub = _FakePublisher()
    node._last_color = np.full((400, 600, 3), 30, np.uint8)
    # 假装上一帧托盘路线留下过一个"看着很正经"的位姿
    node._last_T_cam_pallet = np.eye(4)

    assert node.update() == Status.RUNNING
    assert node._overlay_pub.n == 1, "这一帧该出图（自检用的是真的渲染）"
    assert seen, "render 没被调到 —— 这条用例就没测到东西"
    # 邻居那套画法**连 `T_cam_pallet` 这个参数都没有**（台面矩形那一层不存在），
    # 所以"拿上一帧的位姿去画台面"这件事在接口上就不可能发生。
    assert "T_cam_pallet" not in seen, (
        f"邻居画法收到了托盘位姿：{sorted(seen)}")
    # 真正要传对的是这一帧自己的邻居框 —— 传空了图就没意义。
    assert seen.get("neighbors"), "邻居框没传给渲染"


def test_neighbor_hud_drops_pallet_only_fields(tmp_path, monkeypatch):
    """★ **邻居档的 HUD 只留三个数**（2026-10-17 操作员裁定）。

    托盘的尺寸 / 上游尺寸 / 配对版本号 / `t_cam_base` 对判读邻居帧**没有作用**
    （那条路线根本不看它们），留着只会把有用的数挤下去。同时：
    * **不许再出现"符号未现场核实"** —— 符号已经按操作员现场裁定定死；
    * 三个被控量必须在。
    """
    import skills.atomic.perception.pallet_servo.render as render_mod

    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    cands, box = _candidate_scene(held_key=3)
    writer = _writer("latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp", "latest_box_obs",
                     "latest_box_obs_version", "latest_box_candidates",
                     "latest_box_candidates_version")
    writer.set("latest_box_obs", box)
    writer.set("latest_box_obs_version", 1)
    writer.set("latest_box_candidates", cands)
    writer.set("latest_box_candidates_version", 1)
    writer.set("latest_pallet", None)
    writer.set("latest_pallet_version", 1)

    node = make_node(tmp_path, ref_edges=[["y=0", "x=W"]] * 5,
                     servo_slots=_servo_slots(), pallet_frame="camera",
                     max_dt_s=1.0, overlay_period_s=0.0)
    node._activate_slot(3)

    seen = {}
    real_render = render_mod.render_neighbor_overlay

    def _capture(*args, **kwargs):
        seen.update(kwargs)
        return real_render(*args, **kwargs)

    monkeypatch.setattr(render_mod, "render_neighbor_overlay", _capture)
    node._overlay_pub = _FakePublisher()
    node._last_color = np.full((400, 600, 3), 30, np.uint8)

    assert node.update() == Status.RUNNING
    assert seen, "render 没被调到 —— 这条用例就没测到东西"
    hud = list(seen.get("hud_lines") or ())
    joined = " | ".join(hud)
    assert any("e_bottom=" in line for line in hud), joined
    assert any("e_right=" in line for line in hud), joined
    assert any("theta=" in line for line in hud), joined
    assert "size=" not in joined, f"邻居档不该有托盘尺寸那一行：{joined}"
    assert "from_upstream" not in joined, joined
    assert "pallet_v=" not in joined and "box_v=" not in joined, joined
    assert "t_cam_base=" not in joined, joined
    assert "符号未" not in joined, joined

