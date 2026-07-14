# -*- coding: utf-8 -*-
"""Vacuum485Move：气泵继电器控制薄节点 → vacuum_485 原子技能。

吹气 (blow):     通道1继电器开
吸气 (suck):     通道2继电器开
断电 (power_off): 所有继电器关

对齐 adapters/vacuum_485/relay_pump_client.py
"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.vacuum_485 import (
    Vacuum485Params,
    Vacuum485Skill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="气泵485控制 (吹/吸/断电)",
    category=["vacuum", "end_effector", "ros"],
    tree_type="studio_smoke",
    description="通过 ROS Trigger 服务控制气泵继电器: 吹气/吸气/断电",
    params=[
        {"name": "action", "type": "string", "default": "suck",
         "description": "操作: 'blow'=吹气, 'suck'=吸气, 'power_off'=断电"},
    ],
    inputs=[],
    outputs=[],
)
class Vacuum485Move(BaseAction):
    """气泵继电器控制节点

    用法示例 (py_tree_child.json):
    {
      "name": "Vacuum485Move",
      "label": "vacuum_485_suck",
      "params": {
        "action": { "value": "suck", "source": "CUSTOM", "data_type": "string" }
      },
      "childs": [],
      "childBoard": []
    }
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        action = str(self.params.get("action", "suck")).strip().lower()
        valid_actions = ("blow", "suck", "power_off")
        if action not in valid_actions:
            self.feedback_message = (
                f"vacuum_485: unknown action '{action}', "
                f"expected one of {valid_actions}"
            )
            return

        if _DRY_RUN:
            self.feedback_message = f"dry-run vacuum_485 action={action}"
            self._dry_done = True
            return

        skill_params = Vacuum485Params(action=action)
        self._skill = Vacuum485Skill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "vacuum_485 init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is None:
            return Status.FAILURE

        if self._skill.is_finished():
            return Status.SUCCESS

        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "vacuum_485 failed"
            return Status.FAILURE

        return Status.RUNNING
