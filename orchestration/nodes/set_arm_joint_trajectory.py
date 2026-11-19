"""SetArmJointTrajectory：发送原生带时间点轨迹，不重采样、不等待到位。"""

import json
import os
from typing import Optional

from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_arm_joint_trajectory import (
    SetArmJointTrajectoryParams, SetArmJointTrajectorySkill,
)


@define_manifest(
    label="设置双臂关节轨迹",
    category=["motion", "arm"], tree_type="studio_smoke",
    description="发送原生轨迹（deg），保留每帧时间，不重采样或等待。",
    params=[
        {"name": "times", "type": "floatArr", "required": True,
         "description": "相对轨迹起点的时间点（秒），大于0且严格递增"},
        {"name": "q_frames", "type": "json", "required": True,
         "description": "与times对应的N×14双臂关节角（deg）"},
    ], inputs=[], outputs=[],
)
class SetArmJointTrajectory(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[SetArmJointTrajectorySkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            values = {}
            for key in ("times", "q_frames"):
                board_key = self.params.get(f"{key}__board_key")
                if board_key:
                    self.global_blackboard.register_key(key=board_key, access=Access.READ)
                    value = self.global_blackboard.get(board_key)
                else:
                    value = self.params.get(key)
                values[key] = json.loads(value) if isinstance(value, str) else value
            self._skill = SetArmJointTrajectorySkill(hardware=None)
            result = self._skill.initialize(SetArmJointTrajectoryParams(**values))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"SetArmJointTrajectory 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetArmJointTrajectory 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_arm_joint_trajectory")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
