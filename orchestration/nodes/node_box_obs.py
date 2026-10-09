# -*- coding: utf-8 -*-
"""NodeBoxObs：订阅箱子检测话题 → 转 `BoxObservation` → 写黑板。

这是 `latest_box_obs` 的**第一个真生产者**。在这之前那个键全文没有生产者
（只有离线模拟源 `NodeInjectServoInput` 与测试在写），伺服节点上真机只会
永远停在"等输入"。

黑板契约（照抄 NodePercep / NodePalletPose / NodeInjectServoInput 的写法）
    写  latest_box_obs / latest_box_obs_version      BoxObservation

## 上游话题：`box_detection_msgs/BoxDetection`

默认话题 `/box/detection`，发布者是本仓库的 `box_detection` 包
（算法在 `skills/atomic/perception/box_frame/`；交接文档
`WORKLOG_box_frame_fit.md` §17）。它有**四角**，正好是伺服要的东西：

    corners_uv   geometry_msgs/Point32[4]  顺序 **右下 → 左下 → 左上 → 右上**
                                           —— 与 `BoxObservation.quad` 的契约顺序**一致**，直接填
    valid        bool                      **消费端只需看这一个**
    source       string                    window / single / yolo_fallback
    spread_px    float32                   窗内各帧四角到平均值的最远距离
    rejects      string                    失败槽的原因，便于排查

⚠️ **`valid == false` 的帧绝对不能用**：那时四角就是**原始 YOLO 轴对齐框**，
箱子旋转没恢复（msg 注释原话：「`false` = 四角就是原始 YOLO 框，别拿去做伺服」）。
拿它去算伺服，`theta` 是错的，而**四角看着规规矩矩是个矩形**、`e_bottom_px`
也像模像样 —— 又是一条静默错。所以这种帧**一个字都不写黑板**（默认
`require_valid: true`），下游因此继续用上一帧的好值。

## `stamp` 写哨兵，不编造（**与 `NodePalletObs` 同一口径**）

`header.stamp` 取不到 / 非正时，`BoxObservation.stamp` 写的是**上游给的那个数**
（取不到就 `0.0`），**绝不退回本节点的处理时刻**，并在 `update()` 里告警一次。

这里从前写的是 `stamp or time.time()`，理由是箱子的 stamp 来自**本仓库自己的**
`box_detection`（`box_detection_node.py:327` 逐字转发图像 header），"有总比没有强"。
**那个前提一破**（相机驱动发 `stamp=0`，或检测器自报 `rospy.Time.now()` ——
设计点名的典型错误），编出来的数会**落在 `max_dt_s` 内**静默配上一对不同帧的
图像，误差里混着相机运动，而三个数照样算得出来。写 `0.0` 则配对侧**显式 reject**
（`pair_stamp_zero` / `pair_dt`），失败模式一眼可见。

## 为什么是"按字段名取"而不是 import 那个消息类

`box_detection_msgs` 是本仓库
`infrastructure/ros_packages/src/ros_vision/box_detection_msgs/` 下的 **catkin 包**，
要单独放进 ROS 工作区编译。
这里按**字段名**取（鸭子类型），于是：① LeTools 不多一条构建依赖；
② 换一个"形状一样、包名不一样"的消息照样能用；③ 测试里可以用假的
消息对象，不必装 ROS —— 解析函数因此能进 CI（见本文件末尾的测试文件）。

订阅本身仍然需要一个消息类：`msg_type` 参数给**全名**，运行期用
`roslib.message.get_message_class()` 解析，所以**换话题/换类型都只改参数**。

节点持续返回 RUNNING（源节点口径，与 `NodePercep` / `NodePalletPose` 一致）。
"""
from __future__ import annotations

import threading
import time
from typing import Any, List, Optional, Sequence

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.utils.blackboard import (
    bump_version,
    is_dry_run,
    num_or_none,
    read_version,
)
from skills.atomic.perception.pallet_servo.algorithm import (
    BoxObservation,
    parse_bool_param,
)

logger = get_logger(__name__)

# 默认话题与消息类型。**换成别的话题只改 `topic`**；消息形状不一样时改 `msg_type`
# （全名，如 `vision_msgs/Detection2DArray`），解析那层认不出会只记一次 WARNING。
TOPIC_DEFAULT = "/box/detection"
MSG_TYPE_DEFAULT = "box_detection_msgs/BoxDetection"

# 四角必须落在图里 —— 这个范围只用来挡"明显不是像素"的输入（比如有人把
# 归一化坐标发过来了）。真图尺寸不在这里判，那是伺服节点的事（它有 `image_size`）。
_MAX_PLAUSIBLE_PX = 20000.0


# --------------------------------------------------------------------------- #
# 解析：**纯函数，不碰 rospy** —— 测试因此能进 CI
# --------------------------------------------------------------------------- #
def _point_uv(pt) -> Optional[List[float]]:
    """一个点 → `[u, v]`。认 `Point32`/`Point`（`.x/.y`）、`(u, v)`、`[u, v]`。"""
    if pt is None:
        return None
    u, v = num_or_none(getattr(pt, "x", None)), num_or_none(getattr(pt, "y", None))
    if u is None or v is None:
        try:
            u, v = num_or_none(pt[0]), num_or_none(pt[1])
        except (TypeError, IndexError, KeyError):
            return None
    if u is None or v is None:
        return None
    return [u, v]


def _points_uv(seq) -> Optional[List[List[float]]]:
    """一串点 → `[[u, v], ...]`；任何一个取不出来就整体 None。"""
    if seq is None:
        return None
    try:
        pts = [_point_uv(p) for p in seq]
    except TypeError:
        return None
    if not pts or any(p is None for p in pts):
        return None
    return pts                      # type: ignore[return-value]


def _bbox_center_size(det) -> Optional[List[List[float]]]:
    """`vision_msgs` 的 `bbox`（**中心 + 尺寸**）→ 轴对齐框的两个角。

    仓库里两个 YOLO 脚本都是这么填的（`yolo_box_segment_ros.py` 的
    `bbox.center.x = (x1+x2)/2`、`bbox.size_x = x2-x1`），所以这里的换算是
    **唯一一次**：`u1 = cx - sx/2`、`u2 = cx + sx/2`，`v` 同理。
    """
    bbox = getattr(det, "bbox", None)
    if bbox is None:
        return None
    cx, cy = num_or_none(getattr(getattr(bbox, "center", None), "x", None)), \
        num_or_none(getattr(getattr(bbox, "center", None), "y", None))
    sx, sy = num_or_none(getattr(bbox, "size_x", None)), num_or_none(getattr(bbox, "size_y", None))
    if None in (cx, cy, sx, sy) or sx <= 0.0 or sy <= 0.0:
        return None
    return [[cx - sx / 2.0, cy - sy / 2.0], [cx + sx / 2.0, cy + sy / 2.0]]


def _corner_quad(corners: Sequence) -> Optional[List[List[float]]]:
    """四角 → 契约顺序 `[右下, 左下, 左上, 右上]`。

    **本节点不重排**：四角不是"一份边的清单"就是别人选好的环序，乱动只会把
    `bottom`/`right` 换到别的角上去。`BoxDetection.corners_uv` 的顺序恰好就是
    契约顺序，所以这里是**直通**。唯一的例外是 AABB 合成（见 `_aabb_quad`），
    那是我们从两个角**造**出四个角，造的按契约造。
    """
    pts = _points_uv(corners)
    if pts is None or len(pts) != 4:
        return None
    return pts


def _aabb_quad(u1: float, v1: float, u2: float, v2: float) -> List[List[float]]:
    """轴对齐框的两个角 → 契约顺序的四个角（`[右下, 左下, 左上, 右上]`）。

    与 `pallet_servo/algorithm.py` 的 `box_corners()` 合成方式**逐字一致**
    （图像 u 向右、v 向下）：`p0=(u_max,v_max)`、`p1=(u_min,v_max)`、
    `p2=(u_min,v_min)`、`p3=(u_max,v_min)`。
    """
    u_lo, u_hi = sorted((u1, u2))
    v_lo, v_hi = sorted((v1, v2))
    return [[u_hi, v_hi], [u_lo, v_hi], [u_lo, v_lo], [u_hi, v_lo]]


def _sane(corners: Sequence[Sequence[float]]) -> bool:
    """四角是不是**像像素**的有限数。挡的是"归一化坐标/全零/NaN"这类明显不对的输入。"""
    for u, v in corners:
        for value in (u, v):
            if value != value or abs(value) > _MAX_PLAUSIBLE_PX:
                return False
    return True


def parse_box_message(msg) -> Optional[dict]:
    """ROS 消息 → `{"corners", "valid", "source", "spread_px", "rejects", "stamp"}`。

    认不出来返回 `None`（调用方只记 WARNING，不抛、不 FAILURE）。**纯函数**：
    只按字段名取，不 import 任何消息类型，所以测试里喂假对象就能跑。

    依次尝试这些布局：

    1. `corners_uv`（`BoxDetection`）—— **四角**，契约顺序
    2. `quad` / `corners` / `polygon.points` —— 别的四角说法
    3. `bbox`（**中心 + 尺寸**，`vision_msgs` 的写法）—— 合成轴对齐四角
    4. `u1/v1/u2/v2` 或 `x1/y1/x2/y2` —— 直接的角点字段
    5. `detections[0]`（`Detection2DArray`）—— 递归一层
    """
    if msg is None:
        return None

    # 5) 数组：取第一条（上游只发一条；真有多条时取最"像箱子"的那条由 `pick` 参数管，
    #    但那需要 confidence，而 BoxDetection 没有 —— 保持简单，取第一条）
    detections = getattr(msg, "detections", None)
    if detections is not None:
        try:
            n = len(detections)
        except TypeError:
            n = 0
        if n == 0:
            return None
        inner = parse_box_message(detections[0])
        if inner is not None:
            inner["n_detections"] = n
        return inner

    valid = getattr(msg, "valid", None)
    source = str(getattr(msg, "source", "") or "")
    spread = num_or_none(getattr(msg, "spread_px", None))
    rejects = str(getattr(msg, "rejects", "") or "")

    corners = None
    mode = ""

    # 1) BoxDetection 的四角
    corners = _corner_quad(getattr(msg, "corners_uv", None))
    if corners is not None:
        mode = "corners"

    # 2) 别的四角说法
    if corners is None:
        for field in ("quad", "corners"):
            corners = _corner_quad(getattr(msg, field, None))
            if corners is not None:
                mode = field
                break
    if corners is None:
        polygon = getattr(msg, "polygon", None)
        corners = _corner_quad(getattr(polygon, "points", None))
        if corners is not None:
            mode = "polygon"

    # 3) 中心 + 尺寸（vision_msgs 的 bbox）
    if corners is None:
        pair = _bbox_center_size(msg)
        if pair is not None:
            corners = _aabb_quad(pair[0][0], pair[0][1], pair[1][0], pair[1][1])
            mode = "bbox_center_size"

    # 4) 直接的角点字段
    if corners is None:
        u1, v1 = num_or_none(getattr(msg, "u1", None)), num_or_none(getattr(msg, "v1", None))
        u2, v2 = num_or_none(getattr(msg, "u2", None)), num_or_none(getattr(msg, "v2", None))
        if None in (u1, v1, u2, v2):
            u1, v1 = num_or_none(getattr(msg, "x1", None)), num_or_none(getattr(msg, "y1", None))
            u2, v2 = num_or_none(getattr(msg, "x2", None)), num_or_none(getattr(msg, "y2", None))
        if None not in (u1, v1, u2, v2):
            corners = _aabb_quad(u1, v1, u2, v2)
            mode = "xyxy"

    if corners is None or not _sane(corners):
        return None

    stamp = 0.0
    header = getattr(msg, "header", None)
    stamp_obj = getattr(header, "stamp", None)
    to_sec = getattr(stamp_obj, "to_sec", None)
    if callable(to_sec):
        stamp = num_or_none(to_sec()) or 0.0
    else:
        stamp = num_or_none(stamp_obj) or 0.0

    return {
        "corners": corners,
        # 没有 `valid` 字段的消息（`Detection2DArray` 之类）**不能当 False 处理** ——
        # 那会把本来能用的 AABB 全拒掉。它只是"没有这个说法"，按 True 放行，
        # 由下游从 `box_source` 看出差在哪。
        "valid": True if valid is None else bool(valid),
        "has_valid_field": valid is not None,
        "source": source,
        "spread_px": spread,
        "rejects": rejects,
        "mode": mode,
        # ⚠️ **缺 `header.stamp` 时写哨兵 `0.0`，绝不退 `time.time()`**（与
        #    `NodePalletObs` 同一口径）。这里从前写的是 `stamp or time.time()`，
        #    理由是"箱子的 stamp 来自**本仓库自己的** `box_detection`，那个生产者
        #    可靠"；`box_detection_node.py:327` 确实是 `Header(stamp=
        #    color_msg.header.stamp)` 逐字转发。**但那个前提一破**（相机驱动发
        #    `stamp=0`，或检测器自报 `rospy.Time.now()` —— 后者正是设计点名的
        #    典型错误），编出来的那个数会**落在 `max_dt_s` 内**，配对侧照单全收：
        #    配上一对**不同帧的图像**，误差里混着相机运动，而三个数照样算得出来。
        #    写 `0.0` 则配对侧**显式 reject**（`pair_stamp_zero` / `pair_dt`），
        #    失败模式一眼可见。判别"是编的还是在报实际值"的代价，比丢掉一帧大。
        #    （本节点在 `update()` 里对 `<= 0` 的帧**告警一次**，见 `_warn_stamp`。）
        "stamp": stamp,
    }


def to_box_observation(parsed: dict) -> BoxObservation:
    """解析结果 → 契约对象。四角填 `quad`，同时给出它们的 AABB。"""
    corners = parsed["corners"]
    us = [p[0] for p in corners]
    vs = [p[1] for p in corners]
    return BoxObservation(
        u1=min(us), v1=min(vs), u2=max(us), v2=max(vs),
        quad=[[float(p[0]), float(p[1])] for p in corners],
        label=parsed.get("source", ""),
        confidence=1.0,
        stamp=float(parsed.get("stamp", 0.0)),
    )


# --------------------------------------------------------------------------- #
# 节点
# --------------------------------------------------------------------------- #
class NodeBoxObs(BaseAction):
    """订阅箱子检测话题 → 写黑板 `latest_box_obs` + `_version`。

    params:
        topic           订阅的话题，默认 ``/box/detection``。**换话题只改这一个**
        msg_type        消息类型全名，默认 ``box_detection_msgs/BoxDetection``；
                        运行期用 ``roslib.message.get_message_class()`` 解析
        box_key         写黑板的键，默认 ``latest_box_obs``（要与伺服节点的
                        ``box_key`` 对得上）
        require_valid   默认 true：``valid == false`` 的帧**一个字都不写**
        max_spread_px   默认 0（不查）。>0 时窗内四角散布超过它就跳过这一帧
        stale_after_sec 默认 0（不查）。>0 时超时报"陈旧"，**仍然 RUNNING**
        queue_size      订阅队列，默认 1（只要最新帧，别堆积）
        enabled         默认 true；false 时一个字都不写并返回 SUCCESS（不占着树）
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._topic = str(self.params.get("topic", TOPIC_DEFAULT)).strip()
        self._msg_type = str(self.params.get("msg_type", MSG_TYPE_DEFAULT)).strip()
        self._box_key = str(self.params.get("box_key", "latest_box_obs")).strip()
        # 布尔参数**不许用 `bool()` 顶替**：`bool("false") is True`，而场景 JSON 的
        # READ_BOARD 分支真会传字符串。同 `node_pallet_servo.py` 的 `use_distortion`。
        self._require_valid = parse_bool_param(
            self.params.get("require_valid", True), "require_valid")
        try:
            self._enabled = parse_bool_param(self.params.get("enabled", True), "enabled")
            self._enabled_error = ""
        except ValueError as exc:
            self._enabled = False
            self._enabled_error = str(exc)
            logger.error("NodeBoxObs 参数错误：%s —— 按 enabled=false 处理", exc)
        self._max_spread = float(self.params.get("max_spread_px", 0.0) or 0.0)
        self._stale_after = float(self.params.get("stale_after_sec", 0.0) or 0.0)
        self._queue_size = max(1, int(self.params.get("queue_size", 1)))

        # 配置错误（话题名空）也走 FAILURE：别静默订阅一个空话题。
        self._config_err = "" if self._topic else "topic 不能为空"
        if not self._msg_type:
            self._config_err = self._config_err or "msg_type 不能为空"

        self._lock = threading.Lock()
        self._subscriber = None
        self._pending: Optional[dict] = None        # 回调只往这儿放，绝不碰黑板
        self._last_written_key = None               # 同一条观测不重写（见 update()）
        self._received_monotonic = 0.0
        self._ticks = 0
        self._written = 0
        self._setup_error = ""
        self._last_warn: Optional[str] = None
        self._last_type_warn: Optional[str] = None
        # ★ `header.stamp` 非正那条告警**自己的槽位**（不与 `_warn_once` 共用）：
        #   共用槽位只有一格，"stamp 没填"与"valid=false" / "散布超限"会每 tick
        #   互相覆盖、双双刷屏。同 `NodePalletObs._warn_stamp_once` 的做法。
        self._last_stamp_warn: Optional[str] = None

        # ---- 黑板：注册 + **同时**置初值 -------------------------------------
        # `register_key` 在 py_trees 2.x 里**只注册权限、不创建值**，漏了 set 就是
        # 下游的 `KeyError`（`NodePercep` 踩过并写在注释里的坑）。
        # **两个键必须一起写**：只写值不写 `_version`，下游 `NodePalletServo` 的
        # 版本门禁会停在 (0, 0)、只算一次然后**永远不再重算**（它会为此喊一条
        # WARNING）。同 `node_inject_servo_input.py` 的写法。
        for key in (self._box_key, f"{self._box_key}_version"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE)
        self.global_blackboard.set(self._box_key, None)
        setattr(self.global_blackboard, f"{self._box_key}_version", 0)

    # ------------------------------------------------------------------ 生命周期
    def initialise(self):
        self._setup_error = ""
        with self._lock:
            self._pending = None
            self._received_monotonic = 0.0

        if self._config_err or self._enabled_error:
            self.feedback_message = self._config_err or self._enabled_error
            return
        if is_dry_run() or not self._enabled:
            self.feedback_message = "dry-run / enabled=false：不订阅话题"
            return

        # ★ 幂等闸。py_trees 的 `Behaviour.tick()` 对"状态不是 RUNNING"的行为
        #   **每 tick 重进 `initialise()`**（见 `node_pallet_servo.py` 里那段完整
        #   来龙去脉）；本节点在正常路径上永远 RUNNING，所以正常情况下进不来第二次
        #   —— 但父节点重新初始化这一支时会。没有这一句就是"反复建/析构订阅"。
        if self._subscriber is not None:
            return

        try:
            import rospy
            from roslib.message import get_message_class

            if not rospy.core.is_initialized():
                raise RuntimeError("ROS 节点尚未初始化（没有 master？）")
            msg_class = get_message_class(self._msg_type)
            if msg_class is None:
                raise RuntimeError(
                    f"找不到消息类型 {self._msg_type!r} —— 它所在的包要能被 import "
                    f"（catkin 包先 catkin_make 再 source devel/setup.bash，并把 "
                    f"devel/lib/python*/dist-packages 放进 PYTHONPATH）")
            self._subscriber = rospy.Subscriber(
                self._topic, msg_class, self._on_message,
                queue_size=self._queue_size, tcp_nodelay=True)
            logger.info("NodeBoxObs 订阅 %s（%s）→ 写 %s(+_version)"
                        "；require_valid=%s max_spread_px=%g",
                        self._topic, self._msg_type, self._box_key,
                        self._require_valid, self._max_spread)
        except Exception as exc:                     # noqa: BLE001
            # **绝不把异常抛出 `initialise()`**：py_trees 的 `tick()` 不接这个钩子
            # 抛出的异常，抛出去就是整棵树连每帧日志一起没。同 inject 节点的纪律。
            self._setup_error = str(exc)
            self._cleanup_subscriber()
            self.feedback_message = f"订阅 {self._topic} 失败：{exc}"

    def update(self):
        if self._config_err or self._enabled_error:
            return Status.FAILURE
        if is_dry_run() or not self._enabled:
            # 不占着树：与 `NodeInjectServoInput` 同口径（`enabled: false` 时
            # 返回 SUCCESS，否则"真机上关掉它"会把整条分支永远卡住）。
            return Status.SUCCESS
        if self._setup_error:
            self.feedback_message = f"NodeBoxObs 初始化失败：{self._setup_error}"
            return Status.FAILURE

        self._ticks += 1
        with self._lock:
            parsed = self._pending
            received = self._received_monotonic

        if parsed is None:
            # **"还没收到消息"是正常状态**，不是异常：RUNNING + feedback，不崩。
            self.feedback_message = f"等 {self._topic} 的消息"
            if self._ticks % 50 == 1:
                logger.debug("NodeBoxObs 等首条消息：%s", self._topic)
            return Status.RUNNING

        if self._stale_after > 0.0 and time.monotonic() - received >= self._stale_after:
            self._warn_once(f"{self._topic} 已经 {time.monotonic() - received:.1f}s "
                            f"没有新消息了（黑板上还是上一帧）")

        # `header.stamp` 非正 → **告警一次**（`parse_box_message` 现在写哨兵而不是
        # 退 `time.time()`，见那里 `"stamp"` 那段）。排在两道闸之前：这一帧就算
        # 因为别的原因被跳过，"上游没填 stamp"这件事仍然成立，仍然该被说出来。
        if parsed["stamp"] <= 0.0:
            self._warn_stamp(parsed["stamp"])

        # ---- 逐条闸门：过不去就**一个字都不写**，让下游继续用上一帧 ----------
        if parsed["has_valid_field"] and self._require_valid and not parsed["valid"]:
            self.feedback_message = (
                f"跳过：这一帧 valid=false（source={parsed['source'] or '?'}），"
                f"四角就是原始 YOLO 框，拿去做伺服会算错 theta")
            self._warn_once(
                f"{self._topic} 这一帧 valid=false（source={parsed['source'] or '?'}，"
                f"rejects={parsed['rejects'] or '无'}）—— 四角是**原始 YOLO 框**"
                f"（旋转没恢复），**不写黑板**，下游继续用上一帧")
            return Status.RUNNING
        if (self._max_spread > 0.0 and parsed["spread_px"] is not None
                and parsed["spread_px"] > self._max_spread):
            self.feedback_message = (
                f"跳过：窗内四角散布 {parsed['spread_px']:.1f}px > "
                f"{self._max_spread:.1f}px，这一帧不稳")
            self._warn_once(f"{self._topic} 四角散布 {parsed['spread_px']:.1f}px 超过"
                            f"阈值 {self._max_spread:.1f}px —— 跳过这一帧")
            return Status.RUNNING

        # **同一条观测只写一次。** 回调来一条就存一条，而 `update()` 每 tick 都会被
        # 调用 —— 不加这一句就是"40 Hz 相机、10 Hz tick，每 tick 把同一帧重写一遍"：
        # 版本号一直涨，下游的版本门禁每次都放行，伺服于是拿同一个框 10 Hz 白算。
        # `NodeInjectServoInput` 的注释把这条写死了（「播完就停手，不要拿最后一条
        # 反复写……10 Hz 白算」），这里是同一个道理。
        key = (parsed["stamp"], parsed["source"],
               tuple(tuple(p) for p in parsed["corners"]))
        if key == self._last_written_key:
            self.feedback_message = (f"箱子观测没变（{parsed['mode']}，"
                                     f"source={parsed['source'] or '-'}），不重写")
            return Status.RUNNING
        self._last_written_key = key

        obs = to_box_observation(parsed)
        setattr(self.global_blackboard, self._box_key, obs)
        bump_version(self.global_blackboard, self._box_key)
        self._written += 1
        self.feedback_message = (
            f"箱子四角 {[(round(p[0]), round(p[1])) for p in parsed['corners']]}"
            f"（{parsed['mode']}，source={parsed['source'] or '-'}）")
        if self._ticks % 30 == 1:
            logger.debug("NodeBoxObs 第 %d 次写入 %s v%d：%s", self._written,
                         self._box_key,
                         read_version(self.global_blackboard,
                                       f"{self._box_key}_version"),
                         self.feedback_message)
        # ★ **永远 RUNNING**（源节点口径）。返回 SUCCESS/FAILURE 会让 py_trees
        #   每 tick 重进 `initialise()`。
        return Status.RUNNING

    def terminate(self, new_status):
        self._cleanup_subscriber()

    # ------------------------------------------------------------------ 回调（ROS 线程）
    def _on_message(self, msg) -> None:
        """只做「消息 → 解析结果」存内存。**黑板一个字都不碰**（只能在 tick 线程写）。"""
        parsed = parse_box_message(msg)
        if parsed is None:
            # 认不出来的消息类型：**只喊一次**（按实际类型去重），点出真实类型方便
            # 加分支；绝不抛、绝不 FAILURE —— 上游换了个写法不该把整棵树弄死。
            actual = ""
            header = getattr(msg, "_connection_header", None)
            if isinstance(header, dict):
                actual = str(header.get("type", ""))
            if actual != self._last_type_warn:
                self._last_type_warn = actual
                logger.warning(
                    "NodeBoxObs 认不出 %s 上的消息（实际类型 %r，配置的 msg_type 是 "
                    "%r）—— 这一帧不写黑板。把消息的字段名贴回来，在 "
                    "parse_box_message() 里加一条分支即可（它是纯函数，进 CI 很容易）",
                    self._topic, actual or "未知", self._msg_type)
            return
        with self._lock:
            self._pending = parsed
            self._received_monotonic = time.monotonic()

    # ------------------------------------------------------------------ 工具
    def _cleanup_subscriber(self) -> None:
        """换出句柄再注销：`unregister()` 抛异常也不留野句柄。"""
        subscriber, self._subscriber = self._subscriber, None
        if subscriber is not None:
            try:
                subscriber.unregister()
            except Exception:                        # noqa: BLE001
                pass

    def _warn_once(self, message: str) -> None:
        """同一句原文只喊一次，变了再喊 —— 断流/坏帧是按 tick 频率发生的事。"""
        if message != self._last_warn:
            self._last_warn = message
            logger.warning("NodeBoxObs %s", message)

    def _warn_stamp(self, stamp) -> None:
        """`header.stamp` 取不到 / 非正 → 喊一次（**自己的槽位**，见 `__init__`）。

        **去重键是整条文案**（不是 `_warn_once` 的共用槽位）：`stamp` 是这一帧
        *实际写进黑板*的那个数，持续没填时它**恒等于同一个值**（`0.0`），
        文案因此稳定、不会随帧变化；而 `source` 那类易变量**不在文案里** ——
        它正是会让去重失效的东西（同 `NodePalletObs._warn_stamp_once`）。

        ⚠️ 与托盘侧的口径**核过**：写的是**实际值**（`stamp!r`），不是写死的
        `0.0` —— 非正这一支里 `0.0` 与**负数**都会走到这儿，写死就是在让日志说假话。
        """
        msg = (f"{self._topic} 这一帧的 `header.stamp` 取不到 / 非正（取到 "
               f"{stamp!r}）—— 黑板上的 `BoxObservation.stamp` 写的是**原始值 "
               f"{stamp!r}**，不编造处理时刻。**配对会失败**（下游按 "
               f"`pair_stamp_zero` / `pair_dt` 显式 reject，这比配上一对不同帧的"
               f"图像好）—— 去修上游：本仓库的 `box_detection` 那条链路或它上游的"
               f"相机驱动没填 `header.stamp`")
        if msg != self._last_stamp_warn:
            self._last_stamp_warn = msg
            logger.warning("NodeBoxObs %s", msg)
