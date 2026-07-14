# -*- coding: utf-8 -*-
"""VacuumControlMove：气泵控制薄节点 → vacuum_control 原子技能。

吸气 (suck):  开气泵
松开 (release): 关气泵 + 破真空继电器脉冲

对齐 adapters/vacuum_control/test_vacuum.py
"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.vacuum_control import (
    VacuumControlParams,
    VacuumControlSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="气泵控制 (吸/放)",
    category=["vacuum", "end_effector"],
    tree_type="studio_smoke",
    description="控制气泵吸气或松开 (含破真空)。对齐 adapters/vacuum_control/test_vacuum.py",
    params=[
        {"name": "action", "type": "string", "default": "suck",
         "description": "操作类型: 'suck'=吸气, 'release'=松开(含破真空)"},
    ],
    inputs=[],
    outputs=[],
)
class VacuumControlMove(BaseAction):
    """气泵控制节点

    用法示例 (py_tree_child.json):
    {
      "name": "VacuumControlMove",
      "label": "vacuum_suck",
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
        if action not in ("suck", "release"):
            self.feedback_message = (
                f"vacuum_control: unknown action '{action}', expected 'suck' or 'release'"
            )
            return

        if _DRY_RUN:
            self.feedback_message = f"dry-run vacuum_control action={action}"
            self._dry_done = True
            return

        skill_params = VacuumControlParams(action=action)
        self._skill = VacuumControlSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "vacuum_control init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is None:
            return Status.FAILURE

        if self._skill.is_finished():
            return Status.SUCCESS

        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "vacuum_control failed"
            return Status.FAILURE

        return Status.RUNNING
