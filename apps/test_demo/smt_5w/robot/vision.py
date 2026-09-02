"""商超任务的 YOLO 三维目标跟踪器。

ROS 消息类型和订阅由 RobotIO 统一管理；普通 Python 环境仍可通过
``robot_io=None, auto_subscribe=False`` 测试消息解析、缓存和新鲜度逻辑。
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


DEFAULT_TOPIC = "/robot_yolov8_info"
DEFAULT_MAX_AGE = 0.5


class VisionError(RuntimeError):
    """视觉能力层的基础异常。"""


class VisionRuntimeError(VisionError):
    """RobotIO 或订阅初始化失败。"""


class VisionTimeoutError(VisionError):
    """等待新鲜目标位置超时。"""


@dataclass(frozen=True)
class ObjectObservation:
    """一条带接收时间的三维目标观测。

    ``received_monotonic`` 专用于新鲜度判断，不受 ROS 仿真时间或系统时钟回拨
    影响；``source_stamp`` 保留消息原始时间戳，供日志和数据追踪使用。
    """

    object_id: int
    position: Tuple[float, float, float]
    received_monotonic: float
    source_stamp: Any = None
    frame_id: str = ""
    confidence: Optional[float] = None

    def age(self, now: Optional[float] = None) -> float:
        """返回观测年龄，单位为秒。"""

        current = time.monotonic() if now is None else float(now)
        return max(0.0, current - self.received_monotonic)

    def is_fresh(self, max_age: Optional[float], now: Optional[float] = None) -> bool:
        """判断观测是否仍在新鲜度窗口内。``None`` 表示不限制。"""

        if max_age is None:
            return True
        return self.age(now) <= max(0.0, float(max_age))

    def as_dict(self) -> Dict[str, Any]:
        """转换成便于日志和跨层传递的字典。"""

        return {
            "object_id": self.object_id,
            "position": self.position,
            "x": self.position[0],
            "y": self.position[1],
            "z": self.position[2],
            "received_monotonic": self.received_monotonic,
            "source_stamp": self.source_stamp,
            "frame_id": self.frame_id,
            "confidence": self.confidence,
        }


class ObjectPositionTracker:
    """订阅 YOLO 检测结果，并只向调用方暴露新鲜三维坐标。

    Args:
        robot_io: RobotIO。启用订阅时必须显式注入；离线解析测试可以省略。
        topic: 三维检测结果 topic。
        max_age: 默认新鲜度窗口，单位秒；设为 ``None`` 可关闭过期过滤。
        auto_subscribe: 是否在构造时立即调用 :meth:`start`。
    """

    def __init__(
        self,
        robot_io: Any = None,
        topic: str = DEFAULT_TOPIC,
        max_age: Optional[float] = DEFAULT_MAX_AGE,
        auto_subscribe: bool = True,
    ):
        self.robot_io = robot_io
        self.topic = str(topic or DEFAULT_TOPIC)
        self.max_age = _validate_max_age(max_age)
        self.subscriber = None
        self._lock = threading.RLock()
        self._latest: Dict[int, ObjectObservation] = {}
        self._current: Dict[int, List[ObjectObservation]] = {}
        self._started = False

        if auto_subscribe:
            self.start()

    def start(self) -> "ObjectPositionTracker":
        """开始订阅；重复调用是幂等的，初始化失败会明确抛出异常。"""

        if self._started:
            return self
        try:
            if self.robot_io is None:
                raise VisionRuntimeError("启动视觉订阅前必须显式注入 robot_io")
            self.subscriber = self.robot_io.subscribe_detections(
                callback=self.detection_callback,
                topic=self.topic,
            )
        except Exception as exc:
            raise VisionRuntimeError(
                "无法订阅视觉 topic %s: %s" % (self.topic, exc)
            ) from exc
        if self.subscriber is None:
            raise VisionRuntimeError("订阅视觉 topic %s 未返回 subscriber" % self.topic)
        self._started = True
        self.robot_io.loginfo("视觉跟踪已订阅 %s", self.topic)
        return self

    def close(self) -> None:
        """注销订阅；多次调用安全。"""

        subscriber = self.subscriber
        self.subscriber = None
        self._started = False
        unregister = getattr(subscriber, "unregister", None)
        if callable(unregister):
            try:
                unregister()
            except Exception as exc:
                raise VisionRuntimeError("注销视觉订阅失败: %s" % exc) from exc

    def detection_callback(self, msg: Any) -> None:
        """解析 ``Detection2DArray`` 并更新带时间戳缓存。

        单条格式错误的 detection 会被跳过并记录警告，不会清空同帧其他有效
        目标。最新缓存按新鲜度自然过期，不再因为某一帧漏检而立即全部丢失。
        """

        received = time.monotonic()
        source_stamp, frame_id = _message_metadata(msg)
        current: Dict[int, List[ObjectObservation]] = {}

        for detection in list(getattr(msg, "detections", []) or []):
            try:
                observation = _parse_detection(
                    detection,
                    received_monotonic=received,
                    source_stamp=source_stamp,
                    frame_id=frame_id,
                )
            except (TypeError, ValueError, AttributeError) as exc:
                if self.robot_io is not None:
                    self.robot_io.logwarn("忽略无效 YOLO detection: %s", exc)
                continue
            if observation is None:
                continue
            current.setdefault(observation.object_id, []).append(observation)

        with self._lock:
            self._current = current
            for object_id, observations in current.items():
                # 与旧行为兼容：同一 ID 多目标时 latest 取消息中的最后一项；
                # 所有位置仍可通过 get_positions_by_id() 获取。
                self._latest[object_id] = observations[-1]
            self._prune_locked(received)

    def clear(self, object_id: Any = None) -> None:
        """清空全部缓存，或只清空指定目标 ID。"""

        with self._lock:
            if object_id is None:
                self._latest.clear()
                self._current.clear()
                return
            normalized = _object_id(object_id)
            self._latest.pop(normalized, None)
            self._current.pop(normalized, None)

    def get_observation_by_id(
        self,
        object_id: Any,
        max_age: Optional[float] = None,
    ) -> Optional[ObjectObservation]:
        """返回指定 ID 的最新新鲜观测，过期或不存在时返回 ``None``。"""

        age_limit = self.max_age if max_age is None else _validate_max_age(max_age)
        normalized = _object_id(object_id)
        now = time.monotonic()
        with self._lock:
            observation = self._latest.get(normalized)
            if observation is None or not observation.is_fresh(age_limit, now):
                return None
            return observation

    def get_latest_position_by_id(
        self,
        object_id: Any,
        max_age: Optional[float] = None,
    ) -> Optional[Tuple[float, float, float]]:
        """兼容旧 API：返回指定 ID 最新坐标，过期时返回 ``None``。"""

        observation = self.get_observation_by_id(object_id, max_age=max_age)
        return None if observation is None else observation.position

    def get_positions_by_id(
        self,
        object_id: Any,
        max_age: Optional[float] = None,
    ) -> List[Tuple[float, float, float]]:
        """返回最近一帧中指定 ID 的全部新鲜坐标。"""

        age_limit = self.max_age if max_age is None else _validate_max_age(max_age)
        normalized = _object_id(object_id)
        now = time.monotonic()
        with self._lock:
            observations = list(self._current.get(normalized, []))
        return [
            observation.position
            for observation in observations
            if observation.is_fresh(age_limit, now)
        ]

    def get_all_latest_observations(
        self,
        max_age: Optional[float] = None,
    ) -> Dict[int, ObjectObservation]:
        """返回全部未过期的最新观测。"""

        age_limit = self.max_age if max_age is None else _validate_max_age(max_age)
        now = time.monotonic()
        with self._lock:
            return {
                object_id: observation
                for object_id, observation in self._latest.items()
                if observation.is_fresh(age_limit, now)
            }

    def get_all_latest_positions(
        self,
        max_age: Optional[float] = None,
    ) -> Dict[int, Tuple[float, float, float]]:
        """兼容旧 API：返回全部未过期目标的最新坐标。"""

        return {
            object_id: observation.position
            for object_id, observation in self.get_all_latest_observations(max_age).items()
        }

    def get_all_positions(
        self,
        max_age: Optional[float] = None,
    ) -> Dict[int, List[Tuple[float, float, float]]]:
        """兼容旧 API：返回最近一帧中全部未过期坐标。"""

        age_limit = self.max_age if max_age is None else _validate_max_age(max_age)
        now = time.monotonic()
        with self._lock:
            snapshot = {key: list(value) for key, value in self._current.items()}
        return {
            object_id: [
                observation.position
                for observation in observations
                if observation.is_fresh(age_limit, now)
            ]
            for object_id, observations in snapshot.items()
            if any(observation.is_fresh(age_limit, now) for observation in observations)
        }

    def wait_for_position(
        self,
        object_id: Any,
        timeout: float,
        max_age: Optional[float] = None,
        poll_interval: float = 0.05,
    ) -> Tuple[float, float, float]:
        """阻塞等待新鲜坐标；超时或 RobotIO 关闭时明确抛出异常。"""

        timeout_value = _non_negative_float(timeout, "timeout")
        interval = max(0.01, _non_negative_float(poll_interval, "poll_interval"))
        deadline = time.monotonic() + timeout_value
        normalized = _object_id(object_id)

        while self.robot_io is None or not self.robot_io.is_shutdown():
            position = self.get_latest_position_by_id(normalized, max_age=max_age)
            if position is not None:
                return position
            if time.monotonic() >= deadline:
                raise VisionTimeoutError(
                    "等待 YOLO ID=%s 的新鲜位置超时（%.3f 秒）"
                    % (normalized, timeout_value)
                )
            duration = min(interval, max(0.0, deadline - time.monotonic()))
            if self.robot_io is None:
                time.sleep(duration)
            else:
                self.robot_io.sleep(duration)
        raise VisionRuntimeError("RobotIO 已关闭，等待 YOLO 位置中断")

    def _prune_locked(self, now: float) -> None:
        """在持锁状态下清理过期 latest，避免缓存无限增长。"""

        if self.max_age is None:
            return
        stale_ids = [
            object_id
            for object_id, observation in self._latest.items()
            if not observation.is_fresh(self.max_age, now)
        ]
        for object_id in stale_ids:
            self._latest.pop(object_id, None)


def _parse_detection(
    detection: Any,
    received_monotonic: float,
    source_stamp: Any,
    frame_id: str,
) -> Optional[ObjectObservation]:
    results = list(getattr(detection, "results", []) or [])
    if not results:
        return None
    result = results[0]
    object_id, confidence = _classification(result)
    position = _position_from_result(result)
    if not all(math.isfinite(value) for value in position):
        raise ValueError("目标坐标包含非有限值: %r" % (position,))
    return ObjectObservation(
        object_id=object_id,
        position=position,
        received_monotonic=received_monotonic,
        source_stamp=source_stamp,
        frame_id=frame_id,
        confidence=confidence,
    )


def _classification(result: Any) -> Tuple[int, Optional[float]]:
    hypothesis = getattr(result, "hypothesis", None)
    raw_id = getattr(result, "id", None)
    if raw_id in (None, "") and hypothesis is not None:
        raw_id = getattr(hypothesis, "class_id", None)
    if raw_id in (None, ""):
        raise ValueError("检测结果缺少类别 ID")
    confidence = getattr(result, "score", None)
    if confidence is None and hypothesis is not None:
        confidence = getattr(hypothesis, "score", None)
    return _object_id(raw_id), None if confidence is None else float(confidence)


def _position_from_result(result: Any) -> Tuple[float, float, float]:
    pose = getattr(result, "pose", None)
    # 兼容 vision_msgs 不同版本中的 PoseWithCovariance/PoseWithCovarianceStamped。
    for _ in range(3):
        if pose is None or hasattr(pose, "position"):
            break
        pose = getattr(pose, "pose", None)
    position = getattr(pose, "position", None)
    if position is None:
        raise ValueError("检测结果缺少三维 position")
    return (float(position.x), float(position.y), float(position.z))


def _message_metadata(msg: Any) -> Tuple[Any, str]:
    header = getattr(msg, "header", None)
    return getattr(header, "stamp", None), str(getattr(header, "frame_id", "") or "")


def _validate_max_age(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    return _non_negative_float(value, "max_age")


def _non_negative_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("%s 必须是数字" % name) from exc
    if not math.isfinite(result) or result < 0.0:
        raise ValueError("%s 必须是有限的非负数" % name)
    return result


def _object_id(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("目标 ID 无效: %r" % (value,)) from exc


__all__ = [
    "DEFAULT_MAX_AGE",
    "DEFAULT_TOPIC",
    "ObjectObservation",
    "ObjectPositionTracker",
    "VisionError",
    "VisionRuntimeError",
    "VisionTimeoutError",
]
