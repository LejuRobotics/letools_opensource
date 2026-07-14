# -*- coding: utf-8 -*-
"""WaitForEnter：按 Enter 后继续薄节点 → wait_for_enter 原子技能。"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.wait_for_enter import (
    WaitForEnterParams,
    WaitForEnterSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="按Enter继续",
    category=["utility", "test"],
    tree_type="studio_smoke",
    description="等待用户按下 Enter 后继续，用于调试时暂停以便查看日志",
    params=[
        {
            "name": "message",
            "type": "string",
            "default": "按 Enter 继续...",
            "description": "暂停时的提示信息",
        }
    ],
    inputs=[],
    outputs=[],
)
class WaitForEnter(BaseAction):

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

        skill_params = WaitForEnterParams(
            message=str(self.params.get("message", "按 Enter 继续...")),
        )
        self._skill = WaitForEnterSkill()
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "wait_for_enter init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "wait_for_enter failed"
            return Status.FAILURE
        return Status.RUNNING
