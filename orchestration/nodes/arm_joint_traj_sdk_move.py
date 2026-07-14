# -*- coding: utf-8 -*-
"""ArmJointTrajSdkMove：手臂关节轨迹 SDK 直调薄节点 → arm_joint_traj_sdk 原子技能。"""

import os
import json

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_joint_traj_sdk import (
    ArmJointTrajSdkParams,
    ArmJointTrajSdkSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="手臂关节轨迹（SDK）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="对齐 test_arm_joint_traj.py：调用 hardware.send_arm_joint_traj_sdk(joint_traj, total_time)",
    params=[
        {
            "name": "use_board_trajectory",
            "type": "string",
            "default": "true",
            "description": "true: 从黑板 ArmJointTrajectories 读取 q_frames/times",
        },
        {"name": "total_time", "type": "float", "default": "3.0", "description": "不读黑板时使用"},
        {
            "name": "joint_traj",
            "type": "string",
            "default": "",
            "description": "use_board_trajectory=false 时使用；可传 JSON 字符串或直接传 list",
        },
    ],
    inputs=[
        {
            "name": "arm_joint_trajectories",
            "type": "dict",
            "required": False,
            "default_key": "ArmJointTrajectories",
            "description": "times + q_frames（度）",
        }
    ],
    outputs=[],
)
class ArmJointTrajSdkMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False
        self.global_blackboard.register_key(
            key="ArmJointTrajectories", access=py_trees.common.Access.READ
        )

    def initialise(self):
        self._dry_done = False
        self._skill = None
        use_board = str(self.params.get("use_board_trajectory", "true")).lower() == "true"

        joint_traj = None
        total_time = float(self.params.get("total_time", 3.0))

        if use_board:
            try:
                traj = self.global_blackboard.ArmJointTrajectories
                times = traj.get("times", [])
                q_frames = traj.get("q_frames", [])
                if q_frames:
                    joint_traj = q_frames
                    if times:
                        total_time = float(times[-1])
            except Exception:
                self.feedback_message = "ArmJointTrajectories missing on blackboard"
                return
        else:
            direct = self.params.get("joint_traj", None)
            if isinstance(direct, list):
                joint_traj = direct
            elif isinstance(direct, str) and direct.strip():
                try:
                    parsed = json.loads(direct)
                except Exception:
                    parsed = None
                if isinstance(parsed, list):
                    joint_traj = parsed

        if joint_traj is None:
            self.feedback_message = "arm_joint_traj_sdk: no joint_traj available"
            return

        if _DRY_RUN:
            self.feedback_message = f"dry-run arm_joint_traj_sdk points={len(joint_traj)}"
            self._dry_done = True
            return

        skill_params = ArmJointTrajSdkParams(
            joint_traj=list(joint_traj),
            total_time=float(total_time),
        )
        self._skill = ArmJointTrajSdkSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_joint_traj_sdk init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_joint_traj_sdk failed"
            return Status.FAILURE
        return Status.RUNNING

