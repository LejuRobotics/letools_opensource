"""二维码识别与复扫对齐能力。

模块导入阶段不加载 ROS。订阅、发布、休眠和日志均通过正式 RobotIO 接口完成。
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Tuple

from .geometry import scan_head_points, yaw_from_quat


DEFAULT_DETECTION_TOPIC = "/robot_tag_info"
DEFAULT_HEAD_TOPIC = "/robot_head_motion_data"
DEFAULT_FRESHNESS = 1.0


class QRError(RuntimeError):
    """二维码能力层基础异常。"""


class QRRuntimeError(QRError):
    """RobotIO、订阅或发布接口不可用。"""


class QRScanTimeout(QRError):
    """指定二维码在时限内未出现。"""


class QRAlignmentError(QRError):
    """二维码横向对齐失败。"""


@dataclass(frozen=True)
class QRObservation:
    """一条二维码位姿观测。"""

    tag_id: int
    x: float
    y: float
    z: float
    yaw: float
    quat: Tuple[float, float, float, float]
    received_monotonic: float
    stamp: Any = None
    frame_id: str = ""

    def age(self, now: Optional[float] = None) -> float:
        current = time.monotonic() if now is None else float(now)
        return max(0.0, current - self.received_monotonic)

    def is_fresh(self, max_age: Optional[float], now: Optional[float] = None) -> bool:
        if max_age is None:
            return True
        return self.age(now) <= max(0.0, float(max_age))

    def as_dict(self) -> Dict[str, Any]:
        return {
            "id": self.tag_id,
            "tag_id": self.tag_id,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
            "quat": list(self.quat),
            "stamp": self.stamp,
            "frame_id": self.frame_id,
            "received_monotonic": self.received_monotonic,
        }


class QRRecognizer:
    """订阅 AprilTag 检测，并提供带超时和新鲜度约束的扫描接口。

    Args:
        robot_io: 机器人 IO，必须提供本模块使用的正式通信接口。
        params: 完整任务配置；默认读取 ``robot_io.params``。
        auto_subscribe: 是否在构造时立即订阅二维码 topic。
    """

    def __init__(self, robot_io: Any, params: Optional[Mapping[str, Any]] = None, auto_subscribe: bool = True):
        if robot_io is None:
            raise QRRuntimeError("QRRecognizer 必须注入 robot_io")
        self.robot_io = robot_io
        inherited = robot_io.params
        self.params = dict(params if params is not None else (inherited or {}))
        qr_config = _section(self.params, "qr")
        self.detection_topic = str(qr_config.get("detection_topic", DEFAULT_DETECTION_TOPIC))
        self.freshness = _non_negative_float(
            qr_config.get("freshness", qr_config.get("max_age", DEFAULT_FRESHNESS)),
            "qr.freshness",
        )
        self.latest_tags: Dict[int, Dict[str, Any]] = {}
        self.tag_sub = None
        self._lock = threading.RLock()
        self._started = False
        if auto_subscribe:
            self.start()

    def start(self) -> "QRRecognizer":
        """开始订阅二维码检测；失败时抛出包含 topic 的明确异常。"""

        if self._started:
            return self
        try:
            self.tag_sub = self.robot_io.subscribe_tags(
                self.qr_callback,
                self.detection_topic,
            )
        except Exception as exc:
            raise QRRuntimeError(
                "无法订阅二维码 topic %s: %s" % (self.detection_topic, exc)
            ) from exc
        if self.tag_sub is None:
            raise QRRuntimeError("订阅二维码 topic %s 未返回 subscriber" % self.detection_topic)
        self._started = True
        self.robot_io.loginfo("二维码识别已订阅 %s", self.detection_topic)
        return self

    def close(self) -> None:
        """注销二维码订阅。"""

        subscriber = self.tag_sub
        self.tag_sub = None
        self._started = False
        unregister = getattr(subscriber, "unregister", None)
        if callable(unregister):
            try:
                unregister()
            except Exception as exc:
                raise QRRuntimeError("注销二维码订阅失败: %s" % exc) from exc

    def qr_callback(self, msg: Any) -> None:
        """解析 AprilTagDetectionArray，并记录来源时间和 monotonic 接收时间。"""

        received = time.monotonic()
        header = getattr(msg, "header", None)
        source_stamp = getattr(header, "stamp", None) or self.robot_io.now()
        frame_id = str(getattr(header, "frame_id", "") or "")
        updates: Dict[int, Dict[str, Any]] = {}

        for detection in list(getattr(msg, "detections", []) or []):
            try:
                tag_ids = _tag_ids(detection)
                pose = _detection_pose(detection)
                quat = (
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                )
                values = (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                )
                if not all(math.isfinite(value) for value in values + quat):
                    raise ValueError("二维码位姿包含非有限值")
                yaw = yaw_from_quat(quat)
            except (TypeError, ValueError, AttributeError) as exc:
                self.robot_io.logwarn("忽略无效二维码 detection: %s", exc)
                continue

            for tag_id in tag_ids:
                observation = QRObservation(
                    tag_id=tag_id,
                    x=values[0],
                    y=values[1],
                    z=values[2],
                    yaw=yaw,
                    quat=quat,
                    received_monotonic=received,
                    stamp=source_stamp,
                    frame_id=frame_id,
                )
                updates[tag_id] = observation.as_dict()

        with self._lock:
            self.latest_tags.update(updates)
            self._prune_locked(received)

    def clear(self, target_id: Any = None) -> None:
        """清空全部标签缓存或指定 ID。"""

        with self._lock:
            if target_id is None:
                self.latest_tags.clear()
            else:
                self.latest_tags.pop(_tag_id(target_id), None)

    def get_latest(
        self,
        target_id: Any,
        max_age: Optional[float] = None,
        required: bool = False,
    ) -> Optional[Dict[str, Any]]:
        """读取最新新鲜位姿；``required=True`` 时缺失会明确失败。"""

        tag_id = _tag_id(target_id)
        age_limit = self.freshness if max_age is None else _validate_optional_age(max_age)
        now = time.monotonic()
        with self._lock:
            cached = self.latest_tags.get(tag_id)
            if cached is not None:
                received = float(cached.get("received_monotonic", 0.0))
                if age_limit is None or max(0.0, now - received) <= age_limit:
                    return dict(cached)
        if required:
            raise QRError("二维码 ID=%s 没有新鲜观测" % tag_id)
        return None

    def scan(
        self,
        target_id: Any,
        yaw_range: float = 5.0,
        yaw_step: float = 2.5,
        pitch_center: float = 0.0,
        pitch_range: float = 10.0,
        pitch_step: float = 5.0,
        initial_yaw: Optional[float] = None,
        initial_pitch: Optional[float] = None,
        timeout: float = 15.0,
        hold: float = 1.0,
        max_age: Optional[float] = None,
    ) -> Dict[str, Any]:
        """摆头扫描指定二维码，超时或 RobotIO 关闭时抛出明确异常。"""

        if not self._started:
            self.start()
        tag_id = _tag_id(target_id)
        timeout_value = _non_negative_float(timeout, "timeout")
        hold_value = max(0.05, _non_negative_float(hold, "hold"))
        pitch_center = float(pitch_center)
        initial_yaw = 0.0 if initial_yaw is None else float(initial_yaw)
        initial_pitch = pitch_center if initial_pitch is None else float(initial_pitch)
        points = scan_head_points(
            yaw_range,
            yaw_step,
            pitch_center,
            pitch_range,
            pitch_step,
        )
        deadline = time.monotonic() + timeout_value
        self.clear(tag_id)
        self._publish_head(initial_yaw, initial_pitch)
        self.robot_io.sleep(min(0.5, timeout_value))
        self.robot_io.loginfo(
            "开始扫描二维码 ID=%s，超时 %.1fs",
            tag_id,
            timeout_value,
        )

        index = 0
        try:
            while not self.robot_io.is_shutdown():
                cached = self.get_latest(tag_id, max_age=max_age)
                if cached is not None:
                    return cached
                if time.monotonic() >= deadline:
                    raise QRScanTimeout(
                        "二维码扫描超时: ID=%s timeout=%.3fs" % (tag_id, timeout_value)
                    )
                yaw_deg, pitch_deg = points[index % len(points)]
                self._publish_head(yaw_deg, pitch_deg)
                index += 1
                remaining = max(0.0, deadline - time.monotonic())
                self.robot_io.sleep(min(hold_value, remaining))
            raise QRRuntimeError("RobotIO 已关闭，二维码扫描中断: ID=%s" % tag_id)
        finally:
            try:
                self._publish_head(0.0, pitch_center)
            except Exception as exc:
                self.robot_io.logwarn("扫描结束后头部回中失败: %s", exc)

    def scan_after_walk(
        self,
        target_id: Any,
        pitch_deg: float = 24.0,
        motion: Any = None,
        align_y: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """接近后固定俯仰复扫，并按显式策略选择是否进行 Y 向对齐。

        ``align_y=None`` 保留旧行为：目标等于 ``qr.place_qr_id`` 时不对齐；新
        代码应显式传入布尔值，避免通过二维码 ID 隐式决定运动策略。
        """

        tag_id = _tag_id(target_id)
        pitch_value = float(pitch_deg)
        scan_timeout = (
            float(_section(self.params, "qr").get("scan_timeout", 15.0))
            if timeout is None
            else _non_negative_float(timeout, "timeout")
        )
        qr = self._fixed_pitch_scan(tag_id, pitch_value, scan_timeout)

        if align_y is None:
            place_id = _optional_int(_section(self.params, "qr").get("place_qr_id"))
            align_y = motion is not None and tag_id != place_id
        if not align_y:
            return qr
        if motion is None:
            raise QRAlignmentError("请求二维码 Y 对齐，但未提供 motion controller")
        return self.align_y_after_walk(tag_id, pitch_value, qr, motion, timeout=scan_timeout)

    def align_y_after_walk(
        self,
        target_id: Any,
        pitch_deg: float,
        qr: Mapping[str, Any],
        motion: Any,
        timeout: float = 15.0,
    ) -> Dict[str, Any]:
        """横向调整并复扫；每次运动失败及最终超差都会明确抛出异常。"""

        tag_id = _tag_id(target_id)
        qr_config = _section(self.params, "qr")
        walk_config = _section(self.params, "walk")
        tolerance = _non_negative_float(qr_config.get("align_y_tolerance", 0.05), "align_y_tolerance")
        max_passes = max(1, int(qr_config.get("align_max_passes", 3)))
        robot_type = str(self.robot_io.robot_type).strip().lower()
        current = dict(qr)

        for _pass_index in range(1, max_passes + 1):
            y_error = float(current["y"])
            if abs(y_error) <= tolerance:
                return current

            if robot_type == "wheel":
                speed = min(float(walk_config.get("linear_speed", 0.15)), 0.12)
                step_y = y_error
                move_timeout = max(1.0, min(4.0, abs(y_error) / max(speed, 1e-3) + 1.0))
                kwargs = {
                    "pos_tolerance": min(tolerance * 0.5, 0.02),
                    "min_lateral_speed": 0.05,
                    "log_label": "轮臂二维码y",
                }
            else:
                speed = float(qr_config.get("align_speed", 0.08))
                max_step = _non_negative_float(qr_config.get("align_max_step", 0.12), "align_max_step")
                min_step = _non_negative_float(qr_config.get("align_min_step", 0.02), "align_min_step")
                step_y = max(-max_step, min(max_step, y_error))
                if 0.0 < abs(step_y) < min_step:
                    step_y = min_step if step_y > 0.0 else -min_step
                move_timeout = max(1.5, abs(step_y) / max(abs(speed), 0.01))
                kwargs = {}

            try:
                result = motion.lateral_adjust(step_y, speed, move_timeout, **kwargs)
            except Exception as exc:
                raise QRAlignmentError(
                    "二维码 Y 向微调异常: ID=%s y=%.3f: %s" % (tag_id, y_error, exc)
                ) from exc
            if result is False or result is None:
                raise QRAlignmentError(
                    "二维码 Y 向微调失败: ID=%s y=%.3f" % (tag_id, y_error)
                )

            self.robot_io.sleep(0.5)
            current = self._fixed_pitch_scan(tag_id, float(pitch_deg), float(timeout))

        final_error = float(current["y"])
        if abs(final_error) > tolerance:
            raise QRAlignmentError(
                "二维码 Y 对齐超过最大次数: ID=%s y=%.3f tolerance=%.3f"
                % (tag_id, final_error, tolerance)
            )
        return current

    def _fixed_pitch_scan(self, target_id: int, pitch_deg: float, timeout: float) -> Dict[str, Any]:
        self._publish_head(0.0, pitch_deg)
        self.robot_io.sleep(min(0.5, max(0.0, timeout)))
        return self.scan(
            target_id,
            yaw_range=5.0,
            yaw_step=2.5,
            pitch_center=pitch_deg,
            pitch_range=0.0,
            pitch_step=5.0,
            initial_yaw=0.0,
            initial_pitch=pitch_deg,
            timeout=timeout,
        )

    def _publish_head(self, yaw_deg: float, pitch_deg: float) -> None:
        result = self.robot_io.publish_head(float(yaw_deg), float(pitch_deg))
        if result is False:
            raise QRRuntimeError("robot_io.publish_head 返回 False")

    def _prune_locked(self, now: float) -> None:
        stale = [
            tag_id
            for tag_id, cached in self.latest_tags.items()
            if max(0.0, now - float(cached.get("received_monotonic", 0.0))) > self.freshness
        ]
        for tag_id in stale:
            self.latest_tags.pop(tag_id, None)


def _tag_ids(detection: Any) -> List[int]:
    raw = getattr(detection, "id", [])
    values = list(raw) if isinstance(raw, (list, tuple)) else [raw]
    result = [_tag_id(value) for value in values if value not in (None, "")]
    if not result:
        raise ValueError("二维码 detection 缺少 ID")
    return result


def _detection_pose(detection: Any) -> Any:
    pose = getattr(detection, "pose", None)
    for _ in range(4):
        if pose is None or (hasattr(pose, "position") and hasattr(pose, "orientation")):
            break
        pose = getattr(pose, "pose", None)
    if pose is None or not hasattr(pose, "position"):
        raise ValueError("二维码 detection 缺少 pose")
    return pose


def _section(params: Mapping[str, Any], key: str) -> Dict[str, Any]:
    value = params.get(key, {}) if isinstance(params, Mapping) else {}
    return dict(value) if isinstance(value, Mapping) else {}


def _tag_id(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("二维码 ID 无效: %r" % (value,)) from exc


def _optional_int(value: Any) -> Optional[int]:
    return None if value is None else _tag_id(value)


def _non_negative_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("%s 必须是数字" % name) from exc
    if not math.isfinite(result) or result < 0.0:
        raise ValueError("%s 必须是有限的非负数" % name)
    return result


def _validate_optional_age(value: Any) -> Optional[float]:
    return None if value is None else _non_negative_float(value, "max_age")


__all__ = [
    "DEFAULT_DETECTION_TOPIC",
    "DEFAULT_FRESHNESS",
    "DEFAULT_HEAD_TOPIC",
    "QRAlignmentError",
    "QRError",
    "QRObservation",
    "QRRecognizer",
    "QRRuntimeError",
    "QRScanTimeout",
]
