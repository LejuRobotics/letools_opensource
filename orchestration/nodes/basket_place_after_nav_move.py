# -*- coding: utf-8 -*-
"""放置：从双臂最后目标位姿话题竖直下放，再开拇指并水平外扩。"""

import math
import os
import time

import rospy
from py_trees.common import Status
from std_msgs.msg import Float64MultiArray

from core.domain.end_effector import SG100HandCommand
from core.domain.enums import ArmSide
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.basket_vision_carry_move import _require_success
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

# 仅打开拇指的现有标定指令；不要在此处再使用原来的 SG100_OPENN 全手张开。
SG100_HALF = [1.5, -3.0, -0.5, 1.5, 1.5, 1.3, 1.5, 1.3, 0.0, 1.5, 1.3]

EE_TARGET_LEFT_TOPIC = "/mobile_manipulator/ee_target_6D/point0"
EE_TARGET_RIGHT_TOPIC = "/mobile_manipulator/ee_target_6D/point1"


@define_manifest(
    label="导航到达后当前位置下放箱体",
    category=["motion", "arm"],
    tree_type="depalletize_bin",
    description="读取双臂最后目标位姿话题，竖直下放、开拇指、双臂水平外扩",
    params=[], inputs=[], outputs=[],
)
class BasketPlaceAfterNavMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._status = None

    def initialise(self):
        self._status = None

    def update(self):
        if self._status is not None:
            return self._status
        if _DRY_RUN:
            self.feedback_message = "dry-run current-pose place"
            self._status = Status.SUCCESS
            return self._status
        if not bool(self.params.get("execute_place", False)):
            self.feedback_message = "execute_place=false，未发送放置指令"
            self._status = Status.FAILURE
            return self._status
        try:
            self._execute(get_shared_hardware())
            self.feedback_message = "当前位置下放、开拇指、外扩完成"
            self._status = Status.SUCCESS
        except Exception as exc:
            self.feedback_message = "放置失败: %s" % exc
            self._status = Status.FAILURE
        return self._status

    def _execute(self, hardware, box_width=None):
        # point0/point1 是控制器最后接收的左右臂 6D 目标，不依赖未初始化的
        # 状态管理器。消息姿态为弧度，TimedCmd 命令姿态为度。
        left_current, right_current = self._target_topic_arm_poses()
        drop = float(self.params.get("place_drop_distance_m", 0.30))
        expand = float(self.params.get("place_arm_expand_distance_m", 0.20))
        if drop <= 0.0 or expand < 0.0:
            raise ValueError("place_drop_distance_m 必须大于0，place_arm_expand_distance_m 不能小于0")

        left_down = list(left_current)
        right_down = list(right_current)
        left_down[2] -= drop
        right_down[2] -= drop
        rospy.loginfo("从双臂末端目标位姿放置：双臂竖直下放 %.3fm", drop)
        _require_success(
            hardware.send_arm_ee_local_timed(left_down, right_down, desire_time=2.0),
            "双臂竖直下放",
        )
        time.sleep(2.5)

        rospy.loginfo("下放完成，打开左右手大拇指")
        _require_success(
            hardware.control_end_effector(
                ArmSide.LEFT, SG100HandCommand(positions=SG100_HALF)
            ),
            "左SG100大拇指打开",
        )
        _require_success(
            hardware.control_end_effector(
                ArmSide.RIGHT, SG100HandCommand(positions=SG100_HALF)
            ),
            "右SG100大拇指打开",
        )
        time.sleep(1.0)

        # 保持 x、z 和姿态不变；左臂向 +y、右臂向 -y 水平展开。
        left_expand = list(left_down)
        right_expand = list(right_down)
        left_expand[1] += expand
        right_expand[1] -= expand
        rospy.loginfo("开拇指后，左右臂向外水平展开 %.3fm", expand)
        _require_success(
            hardware.send_arm_ee_local_timed(left_expand, right_expand, desire_time=2.0),
            "双臂水平外扩",
        )
        time.sleep(2.5)
        # 不在这里 arm_reset；全流程下一箱会进入对应层的腰部/双臂预备位。

    def _target_topic_arm_poses(self):
        timeout = max(0.1, float(self.params.get("ee_target_topic_timeout_sec", 3.0)))
        try:
            left_msg = rospy.wait_for_message(EE_TARGET_LEFT_TOPIC, Float64MultiArray, timeout=timeout)
            right_msg = rospy.wait_for_message(EE_TARGET_RIGHT_TOPIC, Float64MultiArray, timeout=timeout)
            return (
                self._target_message_to_command_pose(left_msg, "左"),
                self._target_message_to_command_pose(right_msg, "右"),
            )
        except rospy.ROSException as exc:
            raise RuntimeError("未收到双臂末端目标话题: %s" % exc)

    @staticmethod
    def _target_message_to_command_pose(message, side):
        try:
            data = [float(value) for value in message.data]
        except (AttributeError, TypeError, ValueError) as exc:
            raise RuntimeError("%s臂末端目标话题格式错误: %s" % (side, exc))
        if len(data) < 6:
            raise RuntimeError("%s臂末端目标话题数据不足: %d，期望6" % (side, len(data)))
        # point0/point1: [x, y, z, roll, pitch, yaw]，后三项为 rad。
        pose = data[:3] + [math.degrees(value) for value in data[3:6]]
        if not all(math.isfinite(value) for value in pose):
            raise RuntimeError("%s臂末端目标话题包含非法数值: %s" % (side, pose))
        return pose
