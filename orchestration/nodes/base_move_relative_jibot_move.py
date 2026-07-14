# -*- coding: utf-8 -*-
"""BaseMoveRelativeJibotMove：JiBot底盘相对移动薄节点 → base_move_relative_jibot 原子技能。"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.base_move_relative_jibot import BaseMoveRelativeJibotParams, BaseMoveRelativeJibotSkill

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="JiBot底盘相对移动",
    category=["motion", "chassis", "jibot"],
    tree_type="studio_smoke",
    description="对齐 test_base_move.py：调用 hardware.base_move_relative_jibot()",
    params=[
        {"name": "x", "type": "float", "default": "0.2", "description": "相对当前位置的x方向位移(m)"},
        {"name": "y", "type": "float", "default": "0.0", "description": "相对当前位置的y方向位移(m)"},
        {"name": "theta", "type": "float", "default": "0.0", "description": "相对当前位置的yaw角度变化(rad)"},
        {"name": "avoid_enabled", "type": "bool", "default": "False", "description": "是否启用避障"},
        {"name": "avoid_distance", "type": "float", "default": "0.5", "description": "避障距离(m)"},
        {"name": "linear_velocity", "type": "float", "default": "0.30", "description": "线速度(m/s)"},
        {"name": "angular_velocity", "type": "float", "default": "0.50", "description": "角速度(rad/s)"},
        {"name": "position_threshold", "type": "float", "default": "0.08", "description": "位置到达阈值(m)"},
        {"name": "angle_threshold", "type": "float", "default": "0.1", "description": "角度到达阈值(rad)"},
        {"name": "allow_rotation", "type": "bool", "default": "True", "description": "是否允许旋转"},
    ],
    inputs=[],
    outputs=[],
)
class BaseMoveRelativeJibotMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None
        if _DRY_RUN:
            return

        skill_params = BaseMoveRelativeJibotParams(
            x=float(self.params.get("x", 0.2)),
            y=float(self.params.get("y", 0.0)),
            theta=float(self.params.get("theta", 0.0)),
            avoid_enabled=bool(self.params.get("avoid_enabled", False)),
            avoid_distance=float(self.params.get("avoid_distance", 0.5)),
            linear_velocity=float(self.params.get("linear_velocity", 0.15)),
            angular_velocity=float(self.params.get("angular_velocity", 0.25)),
            position_threshold=float(self.params.get("position_threshold", 0.08)),
            angle_threshold=float(self.params.get("angle_threshold", 0.1)),
            allow_rotation=bool(self.params.get("allow_rotation", True)),
        )
        self._skill = BaseMoveRelativeJibotSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "base_move_relative_jibot init failed"

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run base_move_relative_jibot"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "base_move_relative_jibot failed"
            return Status.FAILURE
        return Status.RUNNING
