# -*- coding: utf-8 -*-
"""LegShortMove：腿部/躯干短动薄节点 → leg_control Skill。"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.manipulation.leg_control.params import LegControlParams
from skills.atomic.manipulation.leg_control.skill import LegControlSkill

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="腿部短动",
    category=["motion", "leg"],
    tree_type="studio_smoke",
    description="4 关节腿部控制，对接 leg_control Skill（参照 test_leg_joint）",
    params=[
        {"name": "j0", "type": "float", "default": "14.90", "description": "左髋俯仰 °"},
        {"name": "j1", "type": "float", "default": "-32.01", "description": "左膝俯仰 °"},
        {"name": "j2", "type": "float", "default": "18.03", "description": "右髋俯仰 °"},
        {"name": "j3", "type": "float", "default": "30.0", "description": "右膝俯仰 °"},
    ],
    inputs=[],
    outputs=[],
)
class LegShortMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        if _DRY_RUN:
            return
        skill_params = LegControlParams(
            joint_angles_deg=[
                float(self.params.get("j0", 14.90)),
                float(self.params.get("j1", -32.01)),
                float(self.params.get("j2", 18.03)),
                float(self.params.get("j3", 30.0)),
            ],
        )
        self._skill = LegControlSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "leg_control init failed"

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run leg short move"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "leg_control failed"
            return Status.FAILURE
        return Status.RUNNING
