# -*- coding: utf-8 -*-
"""ChassisShortMove：底盘短动薄节点 → chassis_velocity Skill。"""

import os

from py_trees.common import Status

from core.domain.enums import FrameType
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.motion.chassis_velocity.params import ChassisVelocityParams
from skills.atomic.motion.chassis_velocity.skill import ChassisVelocitySkill

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="底盘短动",
    category=["motion", "chassis"],
    tree_type="studio_smoke",
    description="短时底盘速度控制，对接 chassis_velocity Skill",
    params=[
        {"name": "vx", "type": "float", "default": "0.3", "description": "m/s（对齐 test_cmd_vel_base 前进）"},
        {"name": "vy", "type": "float", "default": "0.0", "description": "m/s"},
        {"name": "vyaw", "type": "float", "default": "0.0", "description": "rad/s"},
        {"name": "duration_sec", "type": "float", "default": "3.0", "description": "秒"},
    ],
    inputs=[],
    outputs=[],
)
class ChassisShortMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        if _DRY_RUN:
            return
        skill_params = ChassisVelocityParams(
            vx=float(self.params.get("vx", 0.3)),
            vy=float(self.params.get("vy", 0.0)),
            vyaw=float(self.params.get("vyaw", 0.0)),
            duration_sec=float(self.params.get("duration_sec", 3.0)),
            frame=FrameType.LOCAL,
        )
        self._skill = ChassisVelocitySkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "chassis_velocity init failed"

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run chassis short move"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "chassis_velocity failed"
            return Status.FAILURE
        return Status.RUNNING
