# -*- coding: utf-8 -*-
"""WaitSeconds：等待指定秒数薄节点 → wait_seconds 原子技能。"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.wait_seconds import (
    WaitSecondsParams,
    WaitSecondsSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="等待(秒)",
    category=["utility", "timing"],
    tree_type="studio_smoke",
    description="等待指定秒数后返回 SUCCESS（RUNNING 可重入/幂等）",
    params=[
        {"name": "duration_sec", "type": "float", "default": "1.0", "description": "等待秒数"},
    ],
    inputs=[],
    outputs=[],
)
class WaitSeconds(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        if _DRY_RUN:
            self._dry_done = True
            return

        skill_params = WaitSecondsParams(
            duration_sec=float(self.params.get("duration_sec", 1.0)),
        )
        self._skill = WaitSecondsSkill()
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "wait_seconds init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "wait_seconds failed"
            return Status.FAILURE
        return Status.RUNNING

