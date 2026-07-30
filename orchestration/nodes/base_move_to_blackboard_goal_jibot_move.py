# -*- coding: utf-8 -*-
"""Move JiBot chassis to a map pose stored on the blackboard."""

import math
import os

import py_trees
from py_trees.common import Status

from module_internal.bin_planner.config import NAV_GOAL_KEY
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.base_move_to_target_jibot import (
    BaseMoveToTargetJibotParams,
    BaseMoveToTargetJibotSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _deg_to_rad(deg):
    return float(deg) * math.pi / 180.0


def _as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


@define_manifest(
    label="JiBot底盘移动到黑板目标",
    category=["motion", "chassis", "jibot", "depalletize_bin_v1_internal"],
    tree_type="studio_smoke",
    description="Read x/y/theta from blackboard and call base_move_to_target_jibot.",
    params=[
        {"name": "nav_goal_key", "type": "string", "default": NAV_GOAL_KEY, "description": "blackboard key with x/y/theta"},
        {"name": "avoid_enabled", "type": "bool", "default": "False", "description": "enable obstacle avoidance"},
        {"name": "avoid_distance", "type": "float", "default": "0.5", "description": "avoidance distance"},
        {"name": "linear_velocity", "type": "float", "default": "1.0", "description": "linear velocity"},
        {"name": "angular_velocity", "type": "float", "default": "1.0", "description": "angular velocity"},
        {"name": "position_threshold", "type": "float", "default": "0.03", "description": "position threshold"},
        {"name": "angle_threshold", "type": "float", "default": "0.03", "description": "angle threshold"},
        {"name": "allow_rotation", "type": "bool", "default": "True", "description": "allow rotation"},
        {"name": "task_id_key", "type": "string", "default": "current_task_id", "description": "blackboard key for task id"},
    ],
    inputs=[],
    outputs=[],
)
class BaseMoveToBlackboardGoalJibotMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False
        self._task_id_saved = False

    def initialise(self):
        self._skill = None
        self._dry_done = False
        self._task_id_saved = False
        self.global_blackboard.register_key(
            key=str(self.params.get("nav_goal_key", NAV_GOAL_KEY)),
            access=py_trees.common.Access.READ,
        )
        if _DRY_RUN:
            return

        goal = getattr(self.global_blackboard, str(self.params.get("nav_goal_key", NAV_GOAL_KEY)))
        theta = float(goal.get("theta", goal.get("theta_deg", 0.0)))
        theta_unit = str(goal.get("theta_unit", "deg"))
        theta_rad = _deg_to_rad(theta) if theta_unit == "deg" else theta

        skill_params = BaseMoveToTargetJibotParams(
            x=float(goal["x"]),
            y=float(goal["y"]),
            theta=theta_rad,
            avoid_enabled=_as_bool(self.params.get("avoid_enabled", False)),
            avoid_distance=float(self.params.get("avoid_distance", 0.5)),
            linear_velocity=float(self.params.get("linear_velocity", 1.0)),
            angular_velocity=float(self.params.get("angular_velocity", 1.0)),
            position_threshold=float(self.params.get("position_threshold", 0.03)),
            angle_threshold=float(self.params.get("angle_threshold", 0.03)),
            allow_rotation=_as_bool(self.params.get("allow_rotation", True)),
        )
        self._skill = BaseMoveToTargetJibotSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "base_move_to_blackboard_goal_jibot init failed"

    def update(self):
        if _DRY_RUN:
            self._dry_done = True
            return Status.SUCCESS

        if self._skill is None:
            return Status.FAILURE

        if self._skill.is_finished():
            return Status.SUCCESS

        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "base_move_to_blackboard_goal_jibot failed"
            return Status.FAILURE

        if result.data and "task_id" in result.data and not self._task_id_saved:
            task_id_key = str(self.params.get("task_id_key", "current_task_id"))
            self.global_blackboard.register_key(key=task_id_key, access=py_trees.common.Access.WRITE)
            self.global_blackboard.set(task_id_key, result.data["task_id"])
            self._task_id_saved = True

        return Status.RUNNING
