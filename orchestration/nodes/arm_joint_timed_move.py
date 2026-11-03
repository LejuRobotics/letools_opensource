# -*- coding: utf-8 -*-
"""ArmJointTimedMove: one 14D arm joint target through TimedCmd."""

import json
import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_joint_timed import (
    ArmJointTimedParams,
    ArmJointTimedSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="双臂关节控制（TimedCmd）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="一次下发 14 个关节目标角，并等待 TimedCmd 规划动作完成",
    params=[
        {
            "name": "joint_angles",
            "type": "floatArr",
            "default": "0,0,0,0,0,0,0,0,0,0,0,0,0,0",
            "description": "14 个关节角 [左臂 7, 右臂 7]（deg）",
        },
        {"name": "total_time", "type": "float", "default": "3.0", "description": "期望执行时间（秒）"},
        {"name": "settle_time", "type": "float", "default": "0.0", "description": "运动完成后的额外等待（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class ArmJointTimedMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False
        self._error = ""

    def initialise(self):
        self._skill = None
        self._dry_done = False
        self._error = ""

        try:
            joint_angles = self._parse_joint_angles(self.params.get("joint_angles", []))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._error = str(exc)
            self.feedback_message = self._error
            return

        if _DRY_RUN:
            self.feedback_message = f"dry-run arm_joint_timed angles={joint_angles}"
            self._dry_done = True
            return

        skill_params = ArmJointTimedParams(
            joint_angles=joint_angles,
            desire_time=float(self.params.get("total_time", 3.0)),
            settle_time=float(self.params.get("settle_time", 0.0)),
        )
        self._skill = ArmJointTimedSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self._error = result.message or "arm_joint_timed init failed"
            self.feedback_message = self._error

    def update(self):
        if self._error:
            return Status.FAILURE
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_joint_timed failed"
            return Status.FAILURE
        return Status.RUNNING

    @staticmethod
    def _parse_joint_angles(raw):
        if isinstance(raw, str):
            raw = json.loads(raw)
        if not isinstance(raw, (list, tuple)):
            raise TypeError("joint_angles must be a list")
        values = [float(value) for value in raw]
        if len(values) != 14:
            raise ValueError(f"ArmJointTimedMove expects 14 joint angles, got {len(values)}")
        return values
