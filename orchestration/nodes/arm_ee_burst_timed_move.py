# -*- coding: utf-8 -*-
"""ArmEeBurstTimedMove：连发末端航点（TimedCmd）薄节点 → arm_ee_burst_timed 原子技能。"""

import json
import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_burst_timed import (
    ArmEEBurstTimedParams,
    ArmEEBurstTimedSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="连发末端航点（TimedCmd）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="循环 send_timed_single_command 执行多航点，躯干/底盘不动，可选静差检查",
    params=[
        {"name": "side", "type": "string", "default": "left", "description": "手臂侧: 'left' / 'right'"},
        {"name": "frame", "type": "string", "default": "world", "description": "坐标系: 'world' / 'local'"},
        {"name": "waypoints", "type": "json", "default": "",
         "description": "航点 [[x,y,z,yaw,pitch,roll], ...]（米, 度）"},
        {"name": "desire_time", "type": "float", "default": "3.0", "description": "每段期望时间（秒）"},
        {"name": "settle_time", "type": "float", "default": "1.0", "description": "每段后额外等待（秒）"},
        {"name": "check_reach", "type": "bool", "default": "false", "description": "是否每段后查询静差"},
        {"name": "reach_linear_tol", "type": "float", "default": "0.01", "description": "静差位置容差（米）"},
        {"name": "reach_angular_tol", "type": "float", "default": "0.05", "description": "静差姿态容差（弧度）"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEeBurstTimedMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        side = str(self.params.get("side", "left"))
        frame = str(self.params.get("frame", "world"))
        desire_time = float(self.params.get("desire_time", 3.0))
        settle_time = float(self.params.get("settle_time", 1.0))
        check_reach = self._to_bool(self.params.get("check_reach", False))
        reach_linear_tol = float(self.params.get("reach_linear_tol", 0.01))
        reach_angular_tol = float(self.params.get("reach_angular_tol", 0.05))
        waypoints = self._resolve_json("waypoints")

        if not waypoints:
            self.feedback_message = "arm_ee_burst_timed: missing or empty waypoints"
            return

        if _DRY_RUN:
            self.feedback_message = f"dry-run arm_ee_burst_timed {side}/{frame} {len(waypoints)}点"
            self._dry_done = True
            return

        skill_params = ArmEEBurstTimedParams(
            side=side,
            frame=frame,
            waypoints=waypoints,
            desire_time=desire_time,
            settle_time=settle_time,
            check_reach=check_reach,
            reach_linear_tol=reach_linear_tol,
            reach_angular_tol=reach_angular_tol,
        )
        self._skill = ArmEEBurstTimedSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_burst_timed init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_ee_burst_timed failed"
            return Status.FAILURE
        return Status.RUNNING

    def _resolve_json(self, key: str):
        """解析参数：支持内联 list 或 JSON 字符串"""
        raw = self.params.get(key, None)
        if raw is None:
            return None
        if isinstance(raw, list):
            return raw
        if isinstance(raw, str) and raw.strip():
            try:
                return json.loads(raw)
            except Exception:
                return None
        return None

    @staticmethod
    def _to_bool(v) -> bool:
        if isinstance(v, bool):
            return v
        return str(v).lower() in ("1", "true", "yes", "on")
