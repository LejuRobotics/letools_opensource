# -*- coding: utf-8 -*-
"""HeadControlSdkMove：头部 SDK 直调薄节点 → head_control_sdk 原子技能。"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.head_control_sdk import (
    HeadControlSdkParams,
    HeadControlSdkSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="头部控制（SDK）",
    category=["motion", "head"],
    tree_type="studio_smoke",
    description="对齐 test_head_control.py：调用 hardware.control_head_sdk(yaw, pitch)",
    params=[
        {"name": "yaw_deg", "type": "float", "default": "0.0", "description": "偏航角（度）"},
        {"name": "pitch_deg", "type": "float", "default": "0.0", "description": "俯仰角（度）"},
    ],
    inputs=[],
    outputs=[],
)
class HeadControlSdkMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None
        if _DRY_RUN:
            return

        skill_params = HeadControlSdkParams(
            yaw_deg=float(self.params.get("yaw_deg", 0.0)),
            pitch_deg=float(self.params.get("pitch_deg", 0.0)),
        )
        self._skill = HeadControlSdkSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "head_control_sdk init failed"

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                yaw = float(self.params.get("yaw_deg", 0.0))
                pitch = float(self.params.get("pitch_deg", 0.0))
                self.feedback_message = f"dry-run head_control_sdk yaw={yaw} pitch={pitch}"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "head_control_sdk failed"
            return Status.FAILURE
        return Status.RUNNING

