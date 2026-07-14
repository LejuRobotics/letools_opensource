# -*- coding: utf-8 -*-
"""LegJointSdkMove：下肢关节 SDK 直调薄节点 → leg_joint_sdk 原子技能。"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.leg_joint_sdk import LegJointSdkParams, LegJointSdkSkill

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="下肢关节控制（SDK）",
    category=["motion", "leg"],
    tree_type="studio_smoke",
    description="对齐 test_leg_joint.py：调用 hardware.send_leg_joint_sdk(joint_angles, total_time)",
    params=[
        {"name": "j0", "type": "float", "default": "14.90", "description": "左髋俯仰 °"},
        {"name": "j1", "type": "float", "default": "-32.01", "description": "左膝俯仰 °"},
        {"name": "j2", "type": "float", "default": "18.03", "description": "右髋俯仰 °"},
        {"name": "j3", "type": "float", "default": "-90.0", "description": "右膝俯仰 °"},
        {"name": "total_time", "type": "float", "default": "3.0", "description": "秒"},
    ],
    inputs=[],
    outputs=[],
)
class LegJointSdkMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None
        if _DRY_RUN:
            return
        skill_params = LegJointSdkParams(
            joint_angles=[
                float(self.params.get("j0", 14.90)),
                float(self.params.get("j1", -32.01)),
                float(self.params.get("j2", 18.03)),
                float(self.params.get("j3", -90.0)),
            ],
            total_time=float(self.params.get("total_time", 3.0)),
        )
        self._skill = LegJointSdkSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "leg_joint_sdk init failed"

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run leg_joint_sdk"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "leg_joint_sdk failed"
            return Status.FAILURE
        return Status.RUNNING

