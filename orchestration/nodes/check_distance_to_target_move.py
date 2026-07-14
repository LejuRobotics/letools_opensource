# -*- coding: utf-8 -*-
"""CheckDistanceToTargetMove：距离门控节点 → check_distance_to_target 原子技能。"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.check_distance_to_target import (
    CheckDistanceToTargetParams,
    CheckDistanceToTargetSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="检查导航距离",
    category=["motion", "chassis", "jibot"],
    tree_type="studio_smoke",
    description="订阅 AMCL 位姿，距离达标时返回 SUCCESS",
    params=[
        {"name": "target_x", "type": "float", "default": "0.0", "description": "目标点 x (map)"},
        {"name": "target_y", "type": "float", "default": "0.0", "description": "目标点 y (map)"},
        {"name": "threshold", "type": "float", "default": "0.5", "description": "距离阈值 (m)"},
    ],
    inputs=[], outputs=[],
)
class CheckDistanceToTargetMove(BaseAction):

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None
        if _DRY_RUN:
            self.feedback_message = "dry-run check_distance_to_target"
            self._dry_done = True
            return
        skill_params = CheckDistanceToTargetParams(
            target_x=float(self.params.get("target_x", 0.0)),
            target_y=float(self.params.get("target_y", 0.0)),
            threshold=float(self.params.get("threshold", 0.5)),
        )
        self._skill = CheckDistanceToTargetSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        self._skill.execute()
        return Status.RUNNING
