# -*- coding: utf-8 -*-
"""ArmEeOfflineTrajMove：离线时间最优末端轨迹薄节点 → arm_ee_offline_traj 原子技能。"""

import json
import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_offline_traj import (
    ArmEEOfflineTrajParams,
    ArmEEOfflineTrajSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="离线时间最优末端轨迹",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="enable→set→sleep→enable(False) 全流程，planner 0左/1右，姿态弧度",
    params=[
        {"name": "side", "type": "string", "default": "left", "description": "手臂侧: 'left' / 'right'"},
        {"name": "frame", "type": "string", "default": "world", "description": "坐标系: 'world' / 'local'"},
        {"name": "traj", "type": "json", "default": "",
         "description": "轨迹 [[x,y,z,yaw,pitch,roll], ...]（米, 弧度！）"},
        {"name": "times", "type": "json", "default": "",
         "description": "绝对时间 [0,1,2,...]，第一帧必须 0，严格递增"},
        {"name": "total_time", "type": "float", "default": "0", "description": "总时间（秒），0=取 times[-1]"},
        {"name": "post_settle", "type": "float", "default": "0.5", "description": "关闭使能前额外等待（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEeOfflineTrajMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        side = str(self.params.get("side", "left"))
        frame = str(self.params.get("frame", "world"))
        total_time = float(self.params.get("total_time", 0.0))
        post_settle = float(self.params.get("post_settle", 0.5))
        traj = self._resolve_json("traj")
        times = self._resolve_json("times")

        if not traj or not times:
            self.feedback_message = "arm_ee_offline_traj: missing or empty traj/times"
            return

        if _DRY_RUN:
            self.feedback_message = f"dry-run arm_ee_offline_traj {side}/{frame} {len(traj)}点"
            self._dry_done = True
            return

        skill_params = ArmEEOfflineTrajParams(
            side=side,
            frame=frame,
            traj=traj,
            times=times,
            total_time=total_time,
            post_settle=post_settle,
        )
        self._skill = ArmEEOfflineTrajSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_offline_traj init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_ee_offline_traj failed"
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
