# -*- coding: utf-8 -*-
"""
MoveArmBaseJointTrajectories：从黑板读取关节轨迹并驱动手臂。
use_board_trajectory=true 时薄节点 → arm_control Skill。
"""

import os
from time import sleep

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="手臂关节轨迹运动",
    category=["arm", "control"],
    tree_type="studio_smoke",
    description="从黑板 ArmJointTrajectories 读取 times/q_frames 并执行",
    params=[
        {
            "name": "use_board_trajectory",
            "type": "string",
            "default": "true",
            "description": "true 时用黑板末帧走 arm_control",
        }
    ],
    inputs=[
        {
            "name": "arm_joint_trajectories",
            "type": "dict",
            "required": True,
            "default_key": "ArmJointTrajectories",
            "description": "times + q_frames（度）",
        }
    ],
    outputs=[],
)
class MoveArmBaseJointTrajectories(Behaviour):

    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super().__init__(name)
        self.params = params
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.global_blackboard.register_key(
            key="ArmJointTrajectories", access=py_trees.common.Access.READ
        )
        self._skill = None
        self._dry_done = False
        self._legacy_init_ok = False

    def initialise(self):
        self._skill = None
        self._dry_done = False
        self._legacy_init_ok = False

        try:
            traj = self.global_blackboard.ArmJointTrajectories
        except (AttributeError, KeyError):
            self.feedback_message = "ArmJointTrajectories missing on blackboard"
            return

        times = traj["times"]
        q_frames = traj["q_frames"]

        if _DRY_RUN:
            self.feedback_message = f"dry-run arm traj frames={len(q_frames)}"
            self._dry_done = True
            return

        use_board = str(self.params.get("use_board_trajectory", "true")).lower() == "true"
        if use_board and len(q_frames) >= 1:
            target_deg = q_frames[-1]
            time_sec = float(times[-1]) if times else 2.0
            try:
                from orchestration.shared_hardware import get_shared_hardware
                from skills.atomic.manipulation.arm_control.params import ArmControlParams
                from skills.atomic.manipulation.arm_control.skill import ArmControlSkill

                arm_params = ArmControlParams(
                    joint_angles_deg=list(target_deg),
                    time_sec=time_sec,
                    enable_quick_mode=True,
                )
                self._skill = ArmControlSkill(hardware=get_shared_hardware())
                result = self._skill.initialize(arm_params)
                if result.success:
                    return
                self.feedback_message = result.message or "arm_control init failed"
            except Exception as exc:
                self.feedback_message = f"arm_control path failed: {exc}"

        try:
            from shared_robot_sdk import get_shared_robot_sdk

            robot_sdk = get_shared_robot_sdk()
            result = robot_sdk.arm.control_arm_joint_trajectory(times, q_frames)
            if not result:
                self.feedback_message = "control_arm_joint_trajectory failed"
                return
            sleep(float(times[-1]) if times else 1.0)
            self._legacy_init_ok = True
        except Exception as exc:
            self.feedback_message = str(exc)

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is not None:
            if self._skill.is_finished():
                return Status.SUCCESS
            result = self._skill.execute()
            if not result.success:
                self.feedback_message = result.message or "arm_control failed"
                return Status.FAILURE
            return Status.RUNNING

        if self._legacy_init_ok:
            return Status.SUCCESS
        return Status.FAILURE
