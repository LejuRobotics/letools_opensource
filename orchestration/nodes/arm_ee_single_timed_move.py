# -*- coding: utf-8 -*-
"""ArmEeSingleTimedMove：单次末端位姿（TimedCmd）薄节点 → arm_ee_single_timed 原子技能。"""

import json
import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_single_timed import (
    ArmEESingleTimedParams,
    ArmEESingleTimedSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="单次末端位姿（TimedCmd）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="单次末端位姿指令（planner 4/5/6/7），躯干/底盘不动，格式 [x,y,z,yaw,pitch,roll]",
    params=[
        {"name": "side", "type": "string", "default": "left", "description": "手臂侧: 'left' / 'right'"},
        {"name": "frame", "type": "string", "default": "world", "description": "坐标系: 'world' / 'local'"},
        {"name": "pose", "type": "json", "default": "[0.3,0.25,0.5,0,0,0]",
         "description": "末端位姿 [x,y,z,yaw,pitch,roll]（米, 度）"},
        {"name": "desire_time", "type": "float", "default": "3.0", "description": "期望执行时间（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEeSingleTimedMove(BaseAction):
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
        pose = self._resolve_json("pose")

        if pose is None:
            self.feedback_message = "arm_ee_single_timed: missing or invalid pose"
            return

        if _DRY_RUN:
            self.feedback_message = f"dry-run arm_ee_single_timed {side}/{frame} pose={pose}"
            self._dry_done = True
            return

        skill_params = ArmEESingleTimedParams(
            side=side,
            frame=frame,
            pose=pose,
            desire_time=desire_time,
        )
        self._skill = ArmEESingleTimedSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_single_timed init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_ee_single_timed failed"
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
