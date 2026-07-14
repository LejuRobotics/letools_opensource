# -*- coding: utf-8 -*-
"""BaseMoveToTargetJibotMove：JiBot底盘绝对目标点移动薄节点 → base_move_to_target_jibot 原子技能。"""

import os
import math

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.base_move_to_target_jibot import BaseMoveToTargetJibotParams, BaseMoveToTargetJibotSkill

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def deg_to_rad(deg):
    """将角度转换为弧度。"""
    return deg * math.pi / 180.0


@define_manifest(
    label="JiBot底盘绝对目标点移动",
    category=["motion", "chassis", "jibot"],
    tree_type="studio_smoke",
    description="对齐 test_move_to_target.py：调用 hardware.base_move_to_target_jibot()",
    params=[
        {"name": "x", "type": "float", "default": "0.0", "description": "map坐标系下目标x(m)"},
        {"name": "y", "type": "float", "default": "0.0", "description": "map坐标系下目标y(m)"},
        {"name": "theta", "type": "float", "default": "0.0", "description": "map坐标系下目标yaw(deg)"},
        {"name": "theta_unit", "type": "string", "default": "deg", "description": "theta的单位: deg或rad"},
        {"name": "avoid_enabled", "type": "bool", "default": "False", "description": "是否启用避障"},
        {"name": "avoid_distance", "type": "float", "default": "0.5", "description": "避障距离(m)"},
        {"name": "linear_velocity", "type": "float", "default": "0.30", "description": "线速度(m/s)"},
        {"name": "angular_velocity", "type": "float", "default": "0.50", "description": "角速度(rad/s)"},
        {"name": "position_threshold", "type": "float", "default": "0.08", "description": "位置到达阈值(m)"},
        {"name": "angle_threshold", "type": "float", "default": "0.1", "description": "角度到达阈值(rad)"},
        {"name": "allow_rotation", "type": "bool", "default": "True", "description": "是否允许旋转"},
        {"name": "task_id_key", "type": "string", "default": "current_task_id", "description": "保存task_id到黑板的键名"},
    ],
    inputs=[],
    outputs=[],
)
class BaseMoveToTargetJibotMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False
        self._task_id_saved = False

    def initialise(self):
        self._dry_done = False
        self._skill = None
        self._task_id_saved = False
        if _DRY_RUN:
            return

        theta = float(self.params.get("theta", 0.0))
        theta_unit = str(self.params.get("theta_unit", "deg"))
        if theta_unit == "deg":
            theta_rad = deg_to_rad(theta)
        else:
            theta_rad = theta

        skill_params = BaseMoveToTargetJibotParams(
            x=float(self.params.get("x", 0.0)),
            y=float(self.params.get("y", 0.0)),
            theta=theta_rad,
            avoid_enabled=bool(self.params.get("avoid_enabled", False)),
            avoid_distance=float(self.params.get("avoid_distance", 0.5)),
            linear_velocity=float(self.params.get("linear_velocity", 1.0)),
            angular_velocity=float(self.params.get("angular_velocity", 1.0)),
            position_threshold=float(self.params.get("position_threshold", 0.08)),
            angle_threshold=float(self.params.get("angle_threshold", 0.1)),
            allow_rotation=bool(self.params.get("allow_rotation", True)),
        )
        self._skill = BaseMoveToTargetJibotSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "base_move_to_target_jibot init failed"

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run base_move_to_target_jibot"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._skill is None:
            return Status.FAILURE
        
        if self._skill.is_finished():
            return Status.SUCCESS
        
        result = self._skill.execute()
        
        if not result.success:
            self.feedback_message = result.message or "base_move_to_target_jibot failed"
            return Status.FAILURE
        
        if result.data and "task_id" in result.data and not self._task_id_saved:
            task_id_key = str(self.params.get("task_id_key", "current_task_id"))
            try:
                self.global_blackboard.register_key(key=task_id_key, access=py_trees.common.Access.WRITE)
            except AttributeError:
                pass
            try:
                self.global_blackboard.set(task_id_key, result.data["task_id"])
            except AttributeError as e:
                pass
            self._task_id_saved = True
        
        return Status.RUNNING
