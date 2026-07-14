# -*- coding: utf-8 -*-
"""ArmEESetupJibotMove：手臂末端控制前置设置薄节点 → arm_ee_setup_jibot 原子技能。

对齐 test_arm_ee_single_both.py main() 前置设置：
- set_arm_control_mode(1→2) + set_mpc_mode(ARM_EE_ONLY) + _ensure_ee_publisher()

此节点必须放在 ArmEePoseJibotMove 之前执行。
"""

import os

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_setup_jibot import (
    ArmEESetupJibotParams,
    ArmEESetupJibotSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="手臂末端控制前置设置",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "对齐 test_arm_ee_single_both.py main()。"
        "执行 set_arm_control_mode(1→2) + set_mpc_mode(ARM_EE_ONLY) + _ensure_ee_publisher()。"
        "ARM_EE_ONLY 确保仅胳膊正逆解。"
    ),
    params=[],
    inputs=[],
    outputs=[],
)
class ArmEeSetupJibotMove(BaseAction):
    """手臂末端控制前置设置节点。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        if _DRY_RUN:
            self.feedback_message = "dry-run arm_ee_setup_jibot"
            self._dry_done = True
            return

        skill_params = ArmEESetupJibotParams()
        self._skill = ArmEESetupJibotSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_setup_jibot init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_ee_setup_jibot failed"
            return Status.FAILURE
        return Status.RUNNING
