# -*- coding: utf-8 -*-
"""ArmEEPoseJibotMove：手臂末端位姿控制（话题直发）薄节点 → arm_ee_pose_jibot 原子技能。

对齐 test_arm_ee_single_both.py：
- 单臂: hardware.send_ee_pose(side, pose, frame)
- 双臂: hardware.send_both_ee_poses(left_pose, right_pose, frame)
底层路径: /mm/two_arm_hand_pose_cmd (kuavo_msgs/twoArmHandPoseCmd)

前置条件（需在 Sequence 中本节点之前完成）:
- set_arm_control_mode(2) → 外部控制模式
- set_mpc_mode(ARM_EE_ONLY) → 仅手臂末端受控（躯干/底盘不参与解算）
- _ensure_ee_publisher()    → 预创建 Publisher/Subscriber

参数:
- side: 控制侧 ("left" / "right" / "both"), 默认 "both"
- left_x / left_y / left_z: 左手目标位置 (米)
- left_yaw / left_pitch / left_roll: 左手目标姿态 (度)
- right_x / right_y / right_z: 右手目标位置 (米)
- right_yaw / right_pitch / right_roll: 右手目标姿态 (度)
- frame: 坐标系 0=KEEP_CURRENT 1=WORLD 2=LOCAL, 默认 2
"""

import os

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_pose_jibot import (
    ArmEePoseJibotParams,
    ArmEePoseJibotSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="手臂末端位姿控制（话题直发）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "调用 hardware.send_ee_pose / send_both_ee_poses，"
        "直接发布到 /mm/two_arm_hand_pose_cmd。"
        "前置: set_arm_control_mode(2) + set_mpc_mode(ARM_EE_ONLY) + _ensure_ee_publisher()"
    ),
    params=[
        {"name": "side", "type": "string", "default": "both",
         "description": "控制侧: left / right / both"},
        {"name": "left_x", "type": "float", "default": "1.4", "description": "左手 X (米)"},
        {"name": "left_y", "type": "float", "default": "0.25", "description": "左手 Y (米)"},
        {"name": "left_z", "type": "float", "default": "1.0", "description": "左手 Z (米)"},
        {"name": "left_yaw", "type": "float", "default": "0.0", "description": "左手 yaw (度)"},
        {"name": "left_pitch", "type": "float", "default": "0.0", "description": "左手 pitch (度)"},
        {"name": "left_roll", "type": "float", "default": "0.0", "description": "左手 roll (度)"},
        {"name": "right_x", "type": "float", "default": "1.4", "description": "右手 X (米)"},
        {"name": "right_y", "type": "float", "default": "-0.25", "description": "右手 Y (米)"},
        {"name": "right_z", "type": "float", "default": "1.0", "description": "右手 Z (米)"},
        {"name": "right_yaw", "type": "float", "default": "0.0", "description": "右手 yaw (度)"},
        {"name": "right_pitch", "type": "float", "default": "0.0", "description": "右手 pitch (度)"},
        {"name": "right_roll", "type": "float", "default": "0.0", "description": "右手 roll (度)"},
        {"name": "frame", "type": "int", "default": "2",
         "description": "坐标系: 0=KEEP_CURRENT, 1=WORLD, 2=LOCAL"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEePoseJibotMove(BaseAction):
    """手臂末端位姿控制节点（话题直发）。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        side = str(self.params.get("side", "both")).lower()
        if side not in ("left", "right", "both"):
            self.feedback_message = f"arm_ee_pose_jibot: invalid side '{side}'"
            return

        if _DRY_RUN:
            self.feedback_message = f"dry-run arm_ee_pose_jibot side={side}"
            self._dry_done = True
            return

        skill_params = ArmEePoseJibotParams(
            side=side,
            left_x=float(self.params.get("left_x", 1.4)),
            left_y=float(self.params.get("left_y", 0.25)),
            left_z=float(self.params.get("left_z", 1.0)),
            left_yaw=float(self.params.get("left_yaw", 0.0)),
            left_pitch=float(self.params.get("left_pitch", 0.0)),
            left_roll=float(self.params.get("left_roll", 0.0)),
            right_x=float(self.params.get("right_x", 1.4)),
            right_y=float(self.params.get("right_y", -0.25)),
            right_z=float(self.params.get("right_z", 1.0)),
            right_yaw=float(self.params.get("right_yaw", 0.0)),
            right_pitch=float(self.params.get("right_pitch", 0.0)),
            right_roll=float(self.params.get("right_roll", 0.0)),
            frame=int(self.params.get("frame", 2)),
        )
        self._skill = ArmEePoseJibotSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_pose_jibot init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_ee_pose_jibot failed"
            return Status.FAILURE
        return Status.RUNNING
