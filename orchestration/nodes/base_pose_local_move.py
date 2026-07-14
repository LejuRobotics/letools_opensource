# -*- coding: utf-8 -*-
"""BasePoseLocalMove：底盘本体系相对位姿薄节点 → base_pose_local 原子技能。"""

import os

from py_trees.common import Status

from core.domain.enums import FrameType
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.base_pose_local import BasePoseLocalParams, BasePoseLocalSkill

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="底盘相对位姿（本体坐标）",
    category=["motion", "chassis"],
    tree_type="studio_smoke",
    description="对齐 test_cmd_pose_base.py：调用 hardware.send_base_pose(frame=LOCAL)",
    params=[
        {"name": "x", "type": "float", "default": "0.5", "description": "前后位移（m）"},
        {"name": "y", "type": "float", "default": "0.0", "description": "左右位移（m）"},
        {"name": "yaw", "type": "float", "default": "0.0", "description": "偏航（deg，适配器会转换）"},
    ],
    inputs=[],
    outputs=[],
)
class BasePoseLocalMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None
        if _DRY_RUN:
            return

        skill_params = BasePoseLocalParams(
            x=float(self.params.get("x", 0.5)),
            y=float(self.params.get("y", 0.0)),
            yaw=float(self.params.get("yaw", 0.0)),
            frame=FrameType.LOCAL,
        )
        self._skill = BasePoseLocalSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "base_pose_local init failed"

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run base_pose_local"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "base_pose_local failed"
            return Status.FAILURE
        return Status.RUNNING

