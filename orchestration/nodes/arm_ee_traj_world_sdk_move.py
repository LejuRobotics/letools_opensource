# -*- coding: utf-8 -*-
"""ArmEETrajWorldSdkMove：手臂末端轨迹（世界系）薄节点 → arm_ee_traj_world_sdk 原子技能。

轨迹点姿态默认按四元数 [x,y,z,qx,qy,qz,qw] 给；置 orientation_format='euler'
则可按 [x,y,z,yaw,pitch,roll]（米, 度）给，节点内转成四元数后仍按原格式下发。
"""

import json
import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from orchestration.utils.pose_conversion import (
    ORIENTATION_QUAT,
    convert_traj_orientation,
)
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
            "name": "orientation_format",
            "type": "string",
            "default": ORIENTATION_QUAT,
            "description": (
                "轨迹点姿态格式：'quat'（默认）为 [x,y,z,qx,qy,qz,qw]；"
                "'euler' 为 [x,y,z,yaw,pitch,roll]（米, 度），节点内转四元数"
            ),
        },
        {
            "name": "left_traj",
            "type": "json",
            "default": "",
            "description": "左手轨迹 JSON，每点 [x,y,z,qx,qy,qz,qw] 或按 orientation_format",
        },
        {
            "name": "right_traj",
            "type": "json",
            "default": "",
            "description": "右手轨迹 JSON，每点 [x,y,z,qx,qy,qz,qw] 或按 orientation_format",
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
        orientation_format = self._orientation_format()
        try:
            left_traj = self._resolve_traj("left_traj", orientation_format)
            right_traj = self._resolve_traj("right_traj", orientation_format)
        except ValueError as exc:
            self.feedback_message = f"arm_ee_traj_world_sdk: {exc}"
            return

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

    def _orientation_format(self) -> str:
        return str(self.params.get("orientation_format", ORIENTATION_QUAT)).strip().lower()

    def _resolve_traj(self, key: str, orientation_format: str):
        """解析轨迹（支持内联 list 或 JSON 字符串）并把姿态统一为四元数"""
        raw = self.params.get(key, None)
        if raw is None:
            return None
        if isinstance(raw, str):
            if not raw.strip():
                return None
            try:
                raw = json.loads(raw)
            except Exception:
                return None
        if not isinstance(raw, list):
            return None
        return convert_traj_orientation(raw, orientation_format)
