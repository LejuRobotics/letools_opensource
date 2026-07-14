# -*- coding: utf-8 -*-
"""ArmResetSdkMove：手臂归位薄节点 → arm_reset_sdk 原子技能。"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_reset_sdk import (
    ArmResetSdkParams,
    ArmResetSdkSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="手臂归位（SDK）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="调用 hardware.arm_reset()，自动管理 MPC 模式",
    params=[],
    inputs=[],
    outputs=[],
)
class ArmResetSdkMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        if _DRY_RUN:
            self.feedback_message = "dry-run arm_reset_sdk"
            self._dry_done = True
            return

        skill_params = ArmResetSdkParams()
        self._skill = ArmResetSdkSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_reset_sdk init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_reset_sdk failed"
            return Status.FAILURE
        return Status.RUNNING
