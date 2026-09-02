 # -*- coding: utf-8 -*-
"""CheckPoseToTargetMove：距离+角度门控节点。"""

import math
import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
_AMCL_TOPIC = "/move_base/amcl_pose"

try:
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    HAS_ROSPY = True
except ImportError:
    rospy = None
    PoseWithCovarianceStamped = None
    HAS_ROSPY = False


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _normalize_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


@define_manifest(
    label="检查导航距离和角度",
    category=["motion", "chassis", "jibot"],
    tree_type="studio_smoke",
    description="订阅 AMCL 位姿，距离和 yaw 偏差同时达标时返回 SUCCESS",
    params=[
        {"name": "target_x", "type": "float", "default": "0.0", "description": "目标点 x (map)"},
        {"name": "target_y", "type": "float", "default": "0.0", "description": "目标点 y (map)"},
        {"name": "target_theta", "type": "float", "default": "0.0", "description": "目标 yaw"},
        {"name": "theta_unit", "type": "string", "default": "deg", "description": "target_theta 单位: deg/rad"},
        {"name": "distance_threshold", "type": "float", "default": "0.6", "description": "欧氏距离阈值 (m)"},
        {"name": "angle_threshold_deg", "type": "float", "default": "5.0", "description": "yaw 偏差阈值 (deg)"},
    ],
    inputs=[],
    outputs=[],
)
class CheckPoseToTargetMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False
        self._pose = None
        self._sub = None

    def initialise(self):
        self._done = False
        self._pose = None
        if _DRY_RUN:
            self.feedback_message = "dry-run check_pose_to_target"
            self._done = True
            return
        if not HAS_ROSPY:
            self.feedback_message = "check_pose_to_target: rospy unavailable"
            return
        self._sub = rospy.Subscriber(_AMCL_TOPIC, PoseWithCovarianceStamped, self._cb, queue_size=1)

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._done else Status.FAILURE
        if not HAS_ROSPY or self._sub is None:
            return Status.FAILURE
        if self._done:
            return Status.SUCCESS

        rospy.sleep(0.001)
        if self._pose is None:
            return Status.RUNNING

        x, y, yaw = self._pose
        target_x = float(self.params.get("target_x", 0.0))
        target_y = float(self.params.get("target_y", 0.0))
        target_theta = float(self.params.get("target_theta", 0.0))
        if str(self.params.get("theta_unit", "deg")).lower() == "deg":
            target_theta = _deg_to_rad(target_theta)
        distance_threshold = float(self.params.get("distance_threshold", 0.6))
        angle_threshold = _deg_to_rad(float(self.params.get("angle_threshold_deg", 5.0)))

        dist = math.sqrt((x - target_x) ** 2 + (y - target_y) ** 2)
        angle_diff = abs(_normalize_angle(yaw - target_theta))

        if dist <= distance_threshold and angle_diff <= angle_threshold:
            self._done = True
            self._unsubscribe()
            self.feedback_message = (
                f"pose reached: dist={dist:.3f}/{distance_threshold:.3f}m, "
                f"yaw_diff={math.degrees(angle_diff):.2f}/{math.degrees(angle_threshold):.2f}deg"
            )
            return Status.SUCCESS

        self.feedback_message = (
            f"waiting pose: dist={dist:.3f}/{distance_threshold:.3f}m, "
            f"yaw_diff={math.degrees(angle_diff):.2f}/{math.degrees(angle_threshold):.2f}deg"
        )
        return Status.RUNNING

    def terminate(self, new_status):
        if new_status != Status.RUNNING:
            self._unsubscribe()

    def _cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (p.x, p.y, _yaw_from_quat(q))

    def _unsubscribe(self):
        if self._sub is not None:
            try:
                self._sub.unregister()
            except Exception:
                pass
            self._sub = None
