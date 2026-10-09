# -*- coding: utf-8 -*-
"""NodeBoxObs 单元测试。

不需要 ROS、不需要相机、不需要 `box_detection_msgs`：解析那层是**纯函数**，
只按字段名取，所以这里喂的就是普通的假对象。这也是当初把它写成纯函数的理由 ——
CI 的 `verify:opensource` 只跑 `pytest orchestration/nodes/tests/ -m unit`。

最要紧的两条用例是**互为反面**的一对：

  * `test_box_detection_with_valid_false_is_not_written`
    —— 上游说 `valid=false` 时四角就是**原始 YOLO 轴对齐框**（旋转没恢复），
       拿去做伺服会算出一个**错的 theta**，而四角看着规规矩矩是个矩形。
  * `test_a_message_without_a_valid_field_is_still_written`
    —— `vision_msgs/Detection2DArray` 之类**没有** `valid` 这个说法。
       "没有这个字段"**不等于**"valid=false"；当成 false 会把本来能用的 AABB
       全拒掉，节点变成永远不写。

运行：
    pytest orchestration/nodes/tests/test_node_box_obs.py -m unit -v
"""
from types import SimpleNamespace

import pytest
import py_trees
from py_trees.common import Status

from orchestration.nodes.node_box_obs import (
    NodeBoxObs,
    parse_box_message,
    to_box_observation,
)

pytestmark = pytest.mark.unit

# 一帧规规矩矩的箱子四角（契约顺序：右下 → 左下 → 左上 → 右上）
CORNERS = [(457.0, 258.0), (196.0, 253.0), (199.0, 424.0), (454.0, 428.0)]


# --------------------------------------------------------------------------- #
# 假消息
# --------------------------------------------------------------------------- #
class _Pt:
    def __init__(self, x, y):
        self.x, self.y = x, y


def _header(stamp=1.0):
    return SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp))


def box_detection(corners=CORNERS, valid=True, source="window", spread=1.5,
                  rejects="", stamp=1.0):
    """`box_detection_msgs/BoxDetection` 的形状（字段名逐个对 `WORKLOG` §17.2）。"""
    return SimpleNamespace(
        header=_header(stamp),
        corners_uv=[_Pt(u, v) for u, v in corners],
        valid=valid, source=source, spread_px=spread, rejects=rejects,
        n_used=5, n_slots=5, n_failed=0, angle_deg=1.05,
        box_uv=[_Pt(196.0, 253.0), _Pt(457.0, 428.0)], latency_ms=66.0)


def detection2d_array(u0, v0, u1, v1, stamp=1.0):
    """`vision_msgs/Detection2DArray` 的形状：bbox 是**中心 + 尺寸**，没有 valid。"""
    bbox = SimpleNamespace(center=SimpleNamespace(x=(u0 + u1) / 2.0,
                                                  y=(v0 + v1) / 2.0),
                           size_x=abs(u1 - u0), size_y=abs(v1 - v0))
    return SimpleNamespace(header=_header(stamp), detections=[SimpleNamespace(bbox=bbox)])


def polygon_stamped(corners=CORNERS, stamp=1.0):
    return SimpleNamespace(header=_header(stamp),
                           polygon=SimpleNamespace(points=[_Pt(u, v) for u, v in corners]))


# --------------------------------------------------------------------------- #
# 解析层（纯函数）
# --------------------------------------------------------------------------- #
def test_parses_box_detection_corners_in_contract_order():
    """`corners_uv` 的顺序**就是**契约顺序 —— 直接填，不重排。"""
    parsed = parse_box_message(box_detection())
    assert parsed is not None
    assert parsed["corners"] == [[float(u), float(v)] for u, v in CORNERS], parsed["corners"]
    assert parsed["valid"] is True and parsed["has_valid_field"] is True
    assert parsed["source"] == "window"
    assert parsed["spread_px"] == 1.5
    obs = to_box_observation(parsed)
    assert obs.quad == parsed["corners"]
    assert (obs.u1, obs.v1, obs.u2, obs.v2) == (196.0, 253.0, 457.0, 428.0), \
        (obs.u1, obs.v1, obs.u2, obs.v2)
    print("    BoxDetection 四角直通契约顺序，AABB 由四角算出")


def test_parses_detection2darray_center_size():
    """`vision_msgs` 的 bbox 是**中心 + 尺寸**；换算在这里做唯一一次。"""
    parsed = parse_box_message(detection2d_array(100.0, 200.0, 300.0, 500.0))
    assert parsed is not None
    assert parsed["has_valid_field"] is False, "Detection2DArray 没有 valid 字段"
    # **解析层自己的默认必须是 True**，不能靠调用方记得先看 `has_valid_field`。
    # 这条断言是变异测试逼出来的：节点的闸门写作
    #   `has_valid_field and require_valid and not valid`
    # 于是"没有字段"这一路上 `valid` 取什么值**都**走不到闸门里 —— 光有节点级
    # 用例根本区分不出 `True if valid is None else bool(valid)` 与 `bool(valid)`。
    # 不钉住它，改动这一行就没有任何测试会红。
    assert parsed["valid"] is True, "没有 valid 字段 ≠ valid=false，默认必须是 True"
    # 轴对齐框 → 契约顺序四角：右下、左下、左上、右上
    assert parsed["corners"] == [[300.0, 500.0], [100.0, 500.0],
                                 [100.0, 200.0], [300.0, 200.0]], parsed["corners"]
    print("    center+size → 契约顺序四角（右下/左下/左上/右上）")


def test_parses_polygon_stamped():
    parsed = parse_box_message(polygon_stamped())
    assert parsed is not None
    assert parsed["corners"] == [[float(u), float(v)] for u, v in CORNERS]


def test_parses_flat_corner_fields():
    """`u1/v1/u2/v2` 或 `x1/y1/x2/y2` 这种平铺写法也认。"""
    flat = SimpleNamespace(header=_header(), u1=10.0, v1=20.0, u2=30.0, v2=40.0)
    parsed = parse_box_message(flat)
    assert parsed is not None
    assert parsed["corners"] == [[30.0, 40.0], [10.0, 40.0],
                                 [10.0, 20.0], [30.0, 20.0]]
    xyxy = SimpleNamespace(header=_header(), x1=10.0, y1=20.0, x2=30.0, y2=40.0)
    assert parse_box_message(xyxy)["corners"] == parsed["corners"]


def test_unrecognised_and_bad_messages_return_none_and_never_raise():
    """认不出来只许返回 `None`，**绝不许抛** —— 上游换个写法不该把整棵树弄死。"""
    for bad in (None, 42, "字符串", SimpleNamespace(),
                SimpleNamespace(corners_uv=[]),
                SimpleNamespace(corners_uv=[_Pt(1.0, 2.0)]),           # 只有 1 个点
                SimpleNamespace(corners_uv=[_Pt(float("nan"), 0.0)] * 4),
                SimpleNamespace(corners_uv=[_Pt(1e9, 0.0)] * 4),       # 明显不是像素
                SimpleNamespace(detections=[]),
                SimpleNamespace(detections="不是列表")):
        assert parse_box_message(bad) is None, f"{bad!r} 不该解析出四角"


def test_aabb_synthesis_matches_the_algorithm_layer():
    """AABB → 四角的合成方式必须与 `algorithm.box_corners()` **逐字一致**。

    两处各写一遍是有风险的（顺序错了不报错、只是边取错），所以这条把两边钉在一起。
    """
    from skills.atomic.perception.pallet_servo.algorithm import (
        BoxObservation as AlgBox, box_corners)
    parsed = parse_box_message(detection2d_array(100.0, 200.0, 300.0, 500.0))
    ours = to_box_observation(parsed)
    theirs = box_corners(AlgBox(u1=100.0, v1=200.0, u2=300.0, v2=500.0))
    assert ours.quad == [list(p) for p in theirs], (ours.quad, theirs.tolist())
    print("    与 algorithm.box_corners() 的合成顺序逐点一致")


# --------------------------------------------------------------------------- #
# 节点层
# --------------------------------------------------------------------------- #
def make_node(tmp_path, **over):
    params = {"topic": "/box/detection", "box_key": "latest_box_obs"}
    params.update(over)
    node = NodeBoxObs("box", "box", "ns", params)
    # 测试环境没有 ROS master，`initialise()` 会给出 setup_error（那是**对的**行为，
    # 下面单独有一条用例钉它）。这里把订阅那一层假装好，专心测黑板与闸门。
    node._subscriber = object()
    node._setup_error = ""
    return node


def feed(node, msg):
    """走真实的回调路径，不直接塞 `_pending`。"""
    node._on_message(msg)


def written(node):
    return node.global_blackboard.get("latest_box_obs") \
        if hasattr(node.global_blackboard, "get") else \
        getattr(node.global_blackboard, "latest_box_obs", None)


def test_box_detection_is_written_with_both_keys(tmp_path):
    node = make_node(tmp_path)
    assert node.update() == Status.RUNNING, "还没收到消息时是 RUNNING，不是 FAILURE"
    assert getattr(node.global_blackboard, "latest_box_obs", "缺") is None
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 0

    feed(node, box_detection(stamp=1.0))
    assert node.update() == Status.RUNNING
    obs = getattr(node.global_blackboard, "latest_box_obs")
    assert obs is not None and obs.quad == [[float(u), float(v)] for u, v in CORNERS]
    # **两个键必须一起写**：只写值不写 version，下游的版本门禁会停在 (0,0)、
    # 只算一次然后永远不再重算。
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 1
    print("    写入 latest_box_obs + latest_box_obs_version")


def test_box_detection_with_valid_false_is_not_written(tmp_path):
    """★ `valid=false` = 四角是**原始 YOLO 框**，旋转没恢复 —— 绝不能写。

    上游 `.msg` 的注释原话：「`false` = 四角就是原始 YOLO 框，别拿去做伺服」。
    拿它去算 `theta` 会得到一个**错的角**，而四角看着规规矩矩是个矩形、
    `e_bottom_px` 也像模像样 —— 又是一条静默错。
    """
    node = make_node(tmp_path)
    feed(node, box_detection(valid=False, source="yolo_fallback", spread=-1.0,
                             rejects="no_plane×5", stamp=2.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_box_obs", "缺") is None, \
        "valid=false 的帧一个字都不该写"
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 0
    assert "valid=false" in node.feedback_message, node.feedback_message
    print("    valid=false → 不写黑板，版本号不动，feedback 说明原因")


def test_a_message_without_a_valid_field_is_still_written(tmp_path):
    """★ 与上一条**互为反面**：没有 `valid` 字段 ≠ `valid=false`。

    把"没有这个字段"当成 false，会把 `Detection2DArray` 之类本来能用的输入
    全拒掉，节点变成永远不写、下游永远"等输入"。
    """
    node = make_node(tmp_path)
    feed(node, detection2d_array(100.0, 200.0, 300.0, 500.0, stamp=3.0))
    assert node.update() == Status.RUNNING
    obs = getattr(node.global_blackboard, "latest_box_obs")
    assert obs is not None, "没有 valid 字段的消息该照常写"
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 1
    print("    没有 valid 字段 → 照写（不能当成 false）")


def test_the_same_observation_is_not_rewritten_every_tick(tmp_path):
    """同一条观测只写一次；来了新的才写。

    回调来一条就存一条，而 `update()` 每 tick 都跑 —— 不加这道闸就是
    "40 Hz 相机、10 Hz tick，每 tick 把同一帧重写一遍"：版本号一直涨，
    下游版本门禁每次放行，伺服拿同一个框白算。`NodeInjectServoInput` 的注释
    把这条写死了（「不要拿最后一条反复写……10 Hz 白算」）。
    """
    node = make_node(tmp_path)
    feed(node, box_detection(stamp=1.0))
    for _ in range(5):
        assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 1, \
        "同一条观测被重复写了"

    feed(node, box_detection(stamp=2.0, corners=[(u, v - 10.0) for u, v in CORNERS]))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 2, \
        "来了新的一条该写、版本号该涨"
    print("    同一观测重复 tick 只写 1 次；新观测才涨版本号")


def test_max_spread_rejects_a_shaky_window(tmp_path):
    """窗内四角散布太大 = 这一帧不稳，跳过（默认 0 = 不查）。"""
    node = make_node(tmp_path, max_spread_px=5.0)
    feed(node, box_detection(spread=12.0, stamp=1.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_box_obs", "缺") is None
    assert "散布" in node.feedback_message, node.feedback_message

    feed(node, box_detection(spread=1.5, stamp=2.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_box_obs") is not None
    print("    max_spread_px 生效：12px 跳过、1.5px 放行")


def test_unparseable_message_keeps_the_last_good_value(tmp_path):
    """认不出来的消息**不覆盖**黑板上的好值 —— 下游继续用上一帧。"""
    node = make_node(tmp_path)
    feed(node, box_detection(stamp=1.0))
    assert node.update() == Status.RUNNING
    before = getattr(node.global_blackboard, "latest_box_obs")
    feed(node, SimpleNamespace(完全不认识="嗯"))
    assert node.update() == Status.RUNNING, "坏消息不该把节点变成 FAILURE"
    assert getattr(node.global_blackboard, "latest_box_obs") is before
    assert getattr(node.global_blackboard, "latest_box_obs_version") == 1
    print("    认不出的消息：RUNNING + 不覆盖黑板")


def test_a_missing_stamp_is_written_as_a_sentinel_not_as_now(tmp_path, caplog):
    """★ 缺 `header.stamp` 时写**哨兵 `0.0`**，绝不退 `time.time()`（M4）。

    从前这里写的是 `stamp or time.time()`，理由是"箱子的 stamp 来自**本仓库
    自己的** `box_detection`（逐字转发图像 header），有总比没有强"。**那个前提
    一破**（相机驱动发 `stamp=0`，或检测器自报 `rospy.Time.now()` —— 设计点名
    的典型错误），编出来的那个数会**落在 `max_dt_s` 内**，配对侧照单全收：
    配上一对**不同帧的图像**，误差里混着相机运动，而三个数照样算得出来。
    写 `0.0` 则配对侧显式 reject（`pair_stamp_zero` / `pair_dt`），一眼可见。
    """
    import logging
    import time as _time

    node = make_node(tmp_path)
    msg = box_detection(stamp=1.0)
    msg.header = SimpleNamespace()                 # 上游**没填** header.stamp
    before = _time.time()
    with caplog.at_level(logging.WARNING):
        feed(node, msg)
        for _ in range(3):
            assert node.update() == Status.RUNNING
    obs = getattr(node.global_blackboard, "latest_box_obs")
    assert obs is not None, "没时间戳只是「没时间」，不是「不能用」—— 照写"
    assert obs.stamp == 0.0, (
        f"缺 stamp 时写了 {obs.stamp!r} —— 那是**编出来的时刻**，配对侧会当真的收下")
    assert obs.stamp < before - 1.0, "写成 time.time() 了：它看起来像时间，配对上就是灾难"
    # 一次性告警，不是每 tick 一遍（4 tick 只该有 1 条）。
    # ⚠️ 数**记录条数**，不数 `"header.stamp"` 出现的次数 —— 那句话自己就提了两次。
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1, [r.getMessage() for r in warnings]
    assert "不编造处理时刻" in caplog.text, caplog.text
    print("    缺 stamp → 写哨兵 0.0 + 告警一次（不编造时刻）")


def test_a_negative_stamp_is_written_as_is(tmp_path):
    """非正的时间戳**原样写**（负数不折成 `0.0`）—— 与托盘侧同一口径。

    "不编造"的语义是**保留上游给的实际值**：负数也是上游给的。折成 0.0 会让
    日志里"写的是 0.0"与黑板上"其实是 -5.0"对不上，而两者都是"上游坏了"的证据，
    差别在于能不能一眼看出坏成什么样。
    """
    node = make_node(tmp_path)
    feed(node, box_detection(stamp=-5.0))
    assert node.update() == Status.RUNNING
    obs = getattr(node.global_blackboard, "latest_box_obs")
    assert obs is not None and obs.stamp == -5.0, obs.stamp


def test_string_booleans_are_parsed(monkeypatch, tmp_path):
    """场景 JSON 的 READ_BOARD 分支会传字符串布尔，`bool("false")` 是 True。"""
    node = NodeBoxObs("box", "box", "ns",
                      {"enabled": "false", "require_valid": "false"})
    assert node._enabled is False, 'enabled="false" 被当成了开'
    assert node._require_valid is False
    # 关掉时不占着树
    assert node.update() == Status.SUCCESS
    print('    enabled="false" / require_valid="false" 正确解析')


def test_initialise_without_ros_fails_loudly_instead_of_crashing(tmp_path, monkeypatch):
    """没有 ROS master 时：`initialise()` 不许抛，`update()` 给 FAILURE。

    py_trees 的 `tick()` **不接** `initialise()` 抛出的异常 —— 抛出去就是整棵树
    连每帧日志一起没。所以这一层必须把异常转成状态。
    """
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    node = NodeBoxObs("box", "box", "ns", {"topic": "/box/detection"})
    node.initialise()                      # 不许抛
    if node._setup_error:                  # 本机确实没 ROS → 走这条
        assert node.update() == Status.FAILURE
        assert node._setup_error, "setup_error 该说明原因"
        print(f"    无 ROS：initialise() 不抛，update() → FAILURE（{node._setup_error[:40]}…）")
    else:                                  # 有 ROS 的环境
        assert node._subscriber is not None
        print("    有 ROS：订阅已建立")


def test_dry_run_subscribes_to_nothing(monkeypatch, tmp_path):
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    node = NodeBoxObs("box", "box", "ns", {"topic": "/box/detection"})
    node.initialise()
    assert node._subscriber is None, "dry-run 不该订阅话题"
    assert node.update() == Status.SUCCESS, "dry-run 一律 SUCCESS（不占着树）"
    print("    dry-run：不订阅 + SUCCESS")


def test_repeated_initialise_does_not_rebuild_the_subscriber(tmp_path):
    """幂等闸：父节点重新初始化这一支时不许反复建订阅。"""
    node = make_node(tmp_path)
    first = node._subscriber
    node.initialise()
    node.initialise()
    assert node._subscriber is first, "initialise() 重进时重建了订阅"
    print("    initialise() 重进不重建订阅")
