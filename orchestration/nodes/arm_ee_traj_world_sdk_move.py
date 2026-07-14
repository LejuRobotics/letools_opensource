# -*- coding: utf-8 -*-
"""ArmEETrajWorldSdkMove：手臂末端轨迹（世界系）薄节点 → arm_ee_traj_world_sdk 原子技能。"""

import json
import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_traj_world_sdk import (
    ArmEETrajWorldSdkParams,
    ArmEETrajWorldSdkSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="手臂末端轨迹（世界系/SDK）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="调用 hardware.send_arm_ee_traj_sdk(frame='world')，支持黑板或内联轨迹",
    params=[
        {"name": "total_time", "type": "float", "default": "3.0", "description": "总执行时间（秒）"},
        {
            "name": "left_traj",
            "type": "json",
            "default": "",
            "description": "左手轨迹 JSON [[x,y,z,qx,qy,qz,qw],...]",
        },
        {
            "name": "right_traj",
            "type": "json",
            "default": "",
            "description": "右手轨迹 JSON [[x,y,z,qx,qy,qz,qw],...]",
        },
    ],
    inputs=[],
    outputs=[],
)
class ArmEeTrajWorldSdkMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        total_time = float(self.params.get("total_time", 3.0))
        left_traj = self._resolve_traj("left_traj")
        right_traj = self._resolve_traj("right_traj")

        if left_traj is None or right_traj is None:
            self.feedback_message = "arm_ee_traj_world_sdk: missing left_traj or right_traj"
            return

        if _DRY_RUN:
            self.feedback_message = (
                f"dry-run arm_ee_traj_world_sdk left={len(left_traj)} right={len(right_traj)}"
            )
            self._dry_done = True
            return

        skill_params = ArmEETrajWorldSdkParams(
            left_traj=left_traj,
            right_traj=right_traj,
            total_time=total_time,
        )
        self._skill = ArmEETrajWorldSdkSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_traj_world_sdk init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_ee_traj_world_sdk failed"
            return Status.FAILURE
        return Status.RUNNING

    def _resolve_traj(self, key: str):
        """解析轨迹：支持内联 list 或 JSON 字符串"""
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
