# -*- coding: utf-8 -*-
"""NodeServoErrorFromTopic 单元测试。

不需要 ROS、不需要相机、不需要 `pallet_servo_msgs`：解析那层是**纯函数**，只按字段名
取，所以这里喂的就是普通的假对象。CI 只跑 `pytest orchestration/nodes/tests/ -m unit`。

最要紧的两条互为反面：

  * `test_an_invalid_frame_is_never_written`
    —— `valid=false` 时三个量是 **NaN**（契约），而且上游在停摆期间**每秒重发一次**
       这种帧。原样搬到黑板上，下游 calc / judge 会判到 `valid=false` 直接 FAILURE
       —— 而"箱子还没推进视野"这种**正常状态**就把整棵树判死，这个场景根本用不了。
  * `test_a_valid_frame_is_written_with_both_keys`
    —— 有效帧必须写、且 `_version` 必须一起涨；只写值不写版本号，下游的版本门禁停在
       (0, 0)，只算一次然后永远不再重算。

运行：
    pytest orchestration/nodes/tests/test_node_servo_error_from_topic.py -m unit -v
"""
from types import SimpleNamespace

import pytest
from py_trees.common import Status

from core.domain.servo_types import AlignObservation, is_observation_like, observation_field
from orchestration.nodes.node_servo_error_from_topic import (
    NodeServoErrorFromTopic,
    parse_servo_error_message,
)

pytestmark = pytest.mark.unit

NAN = float("nan")
KEY = "latest_servo_error"


# --------------------------------------------------------------------------- #
# 假消息
# --------------------------------------------------------------------------- #
def servo_error(e_bottom=12.5, e_right=-30.0, theta=0.05, valid=True, reject="",
                stamp=100.0, frame_id="camera_color_optical_frame"):
    """`pallet_servo_msgs/PalletServoError` 的形状（字段名逐个对 *.msg）。"""
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp),
                               frame_id=frame_id),
        valid=valid, reject=reject,
        e_bottom_px=e_bottom, e_right_px=e_right, theta_rad=theta)


def invalid_heartbeat(reject="等输入（托盘 缺，箱子 有）", stamp=100.0):
    """停摆心跳：`valid=false` + 三个 NaN —— 上游每秒发的就是这一条。"""
    return servo_error(e_bottom=NAN, e_right=NAN, theta=NAN, valid=False,
                       reject=reject, stamp=stamp)


# --------------------------------------------------------------------------- #
# 解析层（纯函数）
# --------------------------------------------------------------------------- #
def test_parses_a_valid_frame():
    parsed = parse_servo_error_message(servo_error())
    assert parsed is not None
    assert (parsed["e_bottom_px"], parsed["e_right_px"], parsed["theta_rad"]) \
        == (12.5, -30.0, 0.05)
    assert parsed["valid"] is True and parsed["has_valid_field"] is True
    assert parsed["numbers_ok"] is True and parsed["why"] == ""
    assert parsed["reject"] == ""
    assert parsed["stamp"] == 100.0
    print("    PalletServoError 五个字段直通")


def test_a_missing_stamp_becomes_the_sentinel_zero():
    """`header.stamp` 取不到 → **哨兵 `0.0`**（见 `_stamp_sec`）。"""
    msg = servo_error()
    msg.header = SimpleNamespace(frame_id="camera_color_optical_frame")
    assert parse_servo_error_message(msg)["stamp"] == 0.0
    no_header = servo_error()
    del no_header.header
    assert parse_servo_error_message(no_header)["stamp"] == 0.0


def test_unrecognised_messages_return_none_and_never_raise():
    """认不出来只许返回 `None`，**绝不许抛** —— 话题配错不该把整棵树弄死。"""
    for bad in (None, 42, "字符串", SimpleNamespace(), SimpleNamespace(valid=True)):
        assert parse_servo_error_message(bad) is None, f"{bad!r} 不该解析出观测"


def test_an_invalid_frame_still_parses_but_is_flagged_unusable():
    """★ 无效帧是**正常输入**：要解析得出来，交由调用方判 `valid` 并告警。

    解析层在这里 `return None`，会把"上游在发无效帧"误报成"消息形状不对" ——
    同一件事分两层说，日志才对得上现场。
    """
    parsed = parse_servo_error_message(invalid_heartbeat())
    assert parsed is not None, "无效帧不是「认不出的消息」"
    assert parsed["valid"] is False and parsed["has_valid_field"] is True
    assert parsed["numbers_ok"] is False
    assert "e_bottom_px" in parsed["why"], parsed["why"]
    assert parsed["reject"] == "等输入（托盘 缺，箱子 有）"
    print("    无效帧：解析得出来，但 numbers_ok=False 且带 reject 原文")


def test_a_message_without_a_valid_field_defaults_to_true():
    """没有 `valid` 字段 ≠ `valid=false`（与 `NodeBoxObs` 同口径）。

    把"没有这个说法"当成 false，会把本来能用的帧全拒掉，节点变成永远不写、
    下游永远"等观测"。
    """
    msg = servo_error()
    del msg.valid
    parsed = parse_servo_error_message(msg)
    assert parsed is not None
    assert parsed["has_valid_field"] is False
    assert parsed["valid"] is True, "没有 valid 字段时默认必须是 True"


def test_degrees_sent_as_radians_is_flagged():
    """`45.0` 是有限数 —— 只有跟 2π 比才看得出来它是**角度**。"""
    parsed = parse_servo_error_message(servo_error(theta=45.0))
    assert parsed["numbers_ok"] is False
    assert "弧度" in parsed["why"], parsed["why"]
    assert parse_servo_error_message(servo_error(theta=0.7854))["numbers_ok"] is True


# --------------------------------------------------------------------------- #
# 节点层
# --------------------------------------------------------------------------- #
def make_node(**over):
    params = {"topic": "/pallet_servo/dis"}
    params.update(over)
    node = NodeServoErrorFromTopic("servo_error_from_topic", "servo_error_from_topic",
                                   "ns", params)
    # 测试环境没有 ROS master，`initialise()` 会给出 setup_error（那是**对的**行为，
    # 下面单独有一条用例钉它）。这里把订阅那一层假装好，专心测黑板与闸门。
    node._subscriber = object()
    node._setup_error = ""
    return node


def feed(node, msg):
    """走真实的回调路径，不直接塞 `_pending`。"""
    node._on_message(msg)


def obs_of(node, key=KEY):
    return getattr(node.global_blackboard, key, "缺")


def version_of(node, key=KEY):
    return getattr(node.global_blackboard, f"{key}_version", "缺")


def test_a_valid_frame_is_written_with_both_keys():
    node = make_node()
    assert node.update() == Status.RUNNING, "还没收到消息时是 RUNNING，不是 FAILURE"
    assert obs_of(node) is None
    assert version_of(node) == 0

    feed(node, servo_error(stamp=1.0))
    assert node.update() == Status.RUNNING
    obs = obs_of(node)
    assert obs is not None and is_observation_like(obs), obs
    assert (obs.e_bottom_px, obs.e_right_px, obs.theta_rad) == (12.5, -30.0, 0.05)
    # ★ 写下去的观测必须**自称有效**：下游 `observation_field(obs, "valid", True)`
    #   为 False 时会直接 FAILURE —— 那正是本条闸门存在的理由（见下一条用例）。
    assert observation_field(obs, "valid", True) is True
    assert observation_field(obs, "reject", "") == ""
    # **两个键必须一起写**：只写值不写 version，下游版本门禁停在 (0,0)。
    assert version_of(node) == 1
    print("    写入 latest_servo_error + latest_servo_error_version")


def test_the_written_object_is_the_shared_observation_type():
    """写的必须是 `servo_types.AlignObservation` —— 下游一行都不用改就靠这个。"""
    node = make_node()
    feed(node, servo_error(stamp=1.0))
    assert node.update() == Status.RUNNING
    assert isinstance(obs_of(node), AlignObservation), type(obs_of(node))


def test_an_invalid_frame_is_never_written():
    """★ `valid=false`（停摆心跳）一个字都不写 —— 否则下游立刻 FAILURE。

    签名是"三个量都是 NaN"，而对面的 calc / judge 一见 `valid=false` 就 FAILURE。
    而 `等输入（托盘 缺，箱子 有）` 是**正常状态**：一条正常状态把整棵树判死，
    这个场景就根本用不了。
    """
    node = make_node()
    feed(node, servo_error(stamp=1.0))
    assert node.update() == Status.RUNNING
    assert version_of(node) == 1

    feed(node, invalid_heartbeat(stamp=2.0))
    for _ in range(3):
        assert node.update() == Status.RUNNING, "无效帧绝不能把树判死"
    assert version_of(node) == 1, "版本号不该动"
    assert obs_of(node).e_bottom_px == 12.5, "黑板该留上一帧有效观测"
    assert "valid=false" in node.feedback_message, node.feedback_message
    assert "等输入" in node.feedback_message, node.feedback_message
    print("    valid=false → 不写黑板、版本号不动、feedback 带上上游的原因")


def test_nan_with_valid_true_is_never_written():
    """★ 最阴的一条：`valid=true` 但三个量是 NaN。

    契约说无效帧才是 NaN，所以这一帧是**上游的 bug**。放它过去，下游拿到的三个量
    全是 NaN：比较全 False、`abs(NaN) > 阈值` 也全 False —— 判决与计算都静默走歪，
    而消息看上去"有效"。
    """
    node = make_node()
    feed(node, servo_error(e_bottom=NAN, e_right=NAN, theta=NAN, valid=True, stamp=1.0))
    assert node.update() == Status.RUNNING
    assert obs_of(node) is None and version_of(node) == 0
    assert "有限数" in node.feedback_message, node.feedback_message


def test_degrees_sent_as_radians_is_never_written():
    """角度当弧度（45.0 而不是 0.785）：数值像模像样，乘上 gain 就一次性转过头。"""
    node = make_node()
    feed(node, servo_error(theta=45.0, stamp=1.0))
    assert node.update() == Status.RUNNING
    assert obs_of(node) is None and version_of(node) == 0
    assert "弧度" in node.feedback_message, node.feedback_message

    feed(node, servo_error(theta=0.7854, stamp=2.0))       # 合法的 45°
    assert node.update() == Status.RUNNING
    assert version_of(node) == 1


def test_the_same_frame_is_not_rewritten_every_tick():
    """同一帧只写一次；**新的一帧要写**（哪怕三个数一样）。

    键里带 `stamp` 的理由：新帧代表"底盘动过之后又量了一次、还是没对上"，该让下游
    重算一步；没有 `stamp` 就只能按三个数去重，那种"动过了但误差没变"会被静默吞掉，
    迭代计数（`servo_max_iters`）也就永远到不了。
    """
    node = make_node()
    feed(node, servo_error(stamp=1.0))
    for _ in range(5):
        assert node.update() == Status.RUNNING
    assert version_of(node) == 1, "同一帧被重复写了"

    feed(node, servo_error(stamp=2.0))                     # 三个数一样，但换了帧
    assert node.update() == Status.RUNNING
    assert version_of(node) == 2, "新的一帧该写"

    feed(node, servo_error(e_bottom=3.0, stamp=3.0))       # 三个数也变了
    assert node.update() == Status.RUNNING
    assert version_of(node) == 3
    print("    同一帧只写 1 次；新帧（数相同）与新数都写")


def test_an_unparseable_message_keeps_the_last_good_value():
    """认不出来的消息**不覆盖**黑板上的好值 —— 下游继续用上一帧。"""
    node = make_node()
    feed(node, servo_error(stamp=1.0))
    assert node.update() == Status.RUNNING
    before = obs_of(node)

    feed(node, SimpleNamespace(完全不认识="嗯"))
    assert node.update() == Status.RUNNING, "坏消息不该把节点变成 FAILURE"
    assert obs_of(node) is before
    assert version_of(node) == 1
    print("    认不出的消息：RUNNING + 不覆盖黑板")


def test_the_stale_warning_fires_even_while_invalid_heartbeats_keep_coming():
    """★ 停摆时上游**仍在发**无效帧 —— 用"最后一次收到消息"当口径永远不响。

    这里的做法是把"最后一帧有效观测"的时刻往回拨 6 s（不 sleep，避免慢测试与抖动）：
    等价于"上游已经 6 秒没给有效观测"，而话题是活的。
    """
    node = make_node(stale_after_sec=5.0)
    feed(node, servo_error(stamp=1.0))
    assert node.update() == Status.RUNNING
    assert node.feedback_message.startswith("e_bottom"), node.feedback_message

    node._last_valid_monotonic -= 6.0                      # 假装 6 秒前才出过数
    feed(node, invalid_heartbeat(stamp=2.0))
    assert node.update() == Status.RUNNING
    assert "没有新的**有效**观测" in node.feedback_message, node.feedback_message

    # 又出一帧有效观测 → 告警停、feedback 回到三个数
    feed(node, servo_error(e_bottom=1.0, stamp=3.0))
    assert node.update() == Status.RUNNING
    assert "没有新的" not in node.feedback_message, node.feedback_message
    print("    无效帧持续时按「多久没有新的有效观测」告警，恢复后立刻停")


def test_no_stale_warning_by_default():
    """`stale_after_sec` 默认 0 = 不查（与 `NodeBoxObs` 同口径）。"""
    node = make_node()
    feed(node, servo_error(stamp=1.0))
    assert node.update() == Status.RUNNING
    node._last_valid_monotonic -= 3600.0
    feed(node, invalid_heartbeat(stamp=2.0))
    assert node.update() == Status.RUNNING
    assert "没有新的" not in node.feedback_message, node.feedback_message


def test_string_booleans_are_parsed():
    """场景 JSON 的 READ_BOARD 分支会传字符串布尔，`bool("false")` 是 True。"""
    node = NodeServoErrorFromTopic("n", "n", "ns", {"enabled": "false"})
    assert node._enabled is False, 'enabled="false" 被当成了开'
    assert node.update() == Status.SUCCESS, "关掉时不占着树"


def test_an_empty_topic_or_a_bad_enabled_is_a_loud_failure():
    """配置错误走 FAILURE，不静默跑一个什么都不干的节点。"""
    assert make_node(topic="").update() == Status.FAILURE
    bad = NodeServoErrorFromTopic("n", "n", "ns", {"enabled": "maybe"})
    assert bad._enabled is False and bad._enabled_error
    assert bad.update() == Status.FAILURE


def test_initialise_without_ros_fails_loudly_instead_of_crashing(monkeypatch):
    """没有 ROS master 时：`initialise()` 不许抛，`update()` 给 FAILURE。

    py_trees 的 `tick()` **不接** `initialise()` 抛出的异常 —— 抛出去就是整棵树连
    每帧日志一起没。所以这一层必须把异常转成状态。
    """
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    node = NodeServoErrorFromTopic("n", "n", "ns", {"topic": "/pallet_servo/dis"})
    node.initialise()                      # 不许抛
    if node._setup_error:                  # 本机确实没 ROS → 走这条
        assert node.update() == Status.FAILURE
        print(f"    无 ROS：不抛，update() → FAILURE（{node._setup_error[:40]}…）")
    else:                                  # 有 ROS 的环境
        assert node._subscriber is not None
        print("    有 ROS：订阅已建立")


def test_dry_run_subscribes_to_nothing(monkeypatch):
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    node = NodeServoErrorFromTopic("n", "n", "ns", {"topic": "/pallet_servo/dis"})
    node.initialise()
    assert node._subscriber is None, "dry-run 不该订阅话题"
    assert node.update() == Status.SUCCESS, "dry-run 一律 SUCCESS（不占着树）"


def test_repeated_initialise_does_not_rebuild_the_subscriber():
    """幂等闸：父节点重新初始化这一支时不许反复建订阅。"""
    node = make_node()
    first = node._subscriber
    node.initialise()
    node.initialise()
    assert node._subscriber is first, "initialise() 重进时重建了订阅"
