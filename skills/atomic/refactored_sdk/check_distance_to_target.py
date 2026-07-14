# -*- coding: utf-8 -*-
"""Atomic skill: check_distance_to_target.

订阅 /move_base/amcl_pose，50Hz tick 对比距离，达标开门。
"""
import math
from dataclasses import dataclass
from typing import Optional

import rospy

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

_AMCL_TOPIC = "/move_base/amcl_pose"


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass
class CheckDistanceToTargetParams(SkillParams):
    skill_name: str = "check_distance_to_target"
    target_x: float = 0.0
    target_y: float = 0.0
    threshold: float = 0.5
    timeout: float = 120.0


@define_manifest(
    label="检查导航距离",
    category=["motion", "chassis", "jibot"],
    tree_type="studio_smoke",
    description="订阅 AMCL 位姿，达标时门控打开",
    params=[
        {"name": "target_x", "type": "float", "default": "0.0", "description": "目标点 x (map)"},
        {"name": "target_y", "type": "float", "default": "0.0", "description": "目标点 y (map)"},
        {"name": "threshold", "type": "float", "default": "0.5", "description": "欧氏距离阈值 (m)"},
    ],
    inputs=[], outputs=[],
)
class CheckDistanceToTargetSkill(SkillBase):

    def __init__(self, hardware: IHardware):
        super().__init__(name="check_distance_to_target")
        self.hardware = hardware
        self.params: Optional[CheckDistanceToTargetParams] = None
        self._done = False
        self._pose = None  # (x, y)
        self._sub = None

    def _cb(self, msg):
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_initialize(self, params: CheckDistanceToTargetParams) -> Result:
        if not isinstance(params, CheckDistanceToTargetParams):
            return Result.fail("Invalid parameters for CheckDistanceToTargetSkill")
        self.params = params
        self._done = False
        self._pose = None
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self._sub = rospy.Subscriber(_AMCL_TOPIC, PoseWithCovarianceStamped, self._cb, queue_size=1)
        logger.info(f"[check_distance] 订阅 {_AMCL_TOPIC}")
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("Already within threshold")

        # 泵一次 ROS 事件让回调有机会触发
        rospy.sleep(0.001)

        pose = self._pose
        if pose is None:
            return Result.ok()

        x, y = pose
        dist = math.sqrt((x - self.params.target_x) ** 2 + (y - self.params.target_y) ** 2)

        if dist <= self.params.threshold:
            self._done = True
            self._sub.unregister()
            logger.info(f"[check_distance] ✅ dist={dist:.3f}/{self.params.threshold:.3f}m")
            return Result.ok("Reached")

        return Result.ok(f"dist={dist:.3f}/{self.params.threshold:.3f}m")

    def on_is_finished(self) -> bool:
        return self._done
