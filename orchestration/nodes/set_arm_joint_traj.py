"""SetArmJointTraj：同步执行SDK ArmAPI下发循环，会阻塞当前tick，不保证实测到位。"""

import json
import os
from typing import Optional
from py_trees.common import Access, Status
from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_arm_joint_traj import SetArmJointTrajParams, SetArmJointTrajSkill


@define_manifest(
    label="定时双臂关节轨迹", category=["motion", "arm"], tree_type="studio_smoke",
    description="SDK ArmAPI同步重采样执行，不是TimedCmd服务；不保证到位。",
    params=[
        {"name": "joint_traj", "type": "json", "required": True, "description": "至少两帧14维关节角（deg）"},
        {"name": "total_time", "type": "float", "required": True, "description": "轨迹总时间（秒），至少0.01"},
    ], inputs=[], outputs=[],
)
class SetArmJointTraj(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[SetArmJointTrajSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            values = {}
            for key, default in (("joint_traj", None), ("total_time", None)):
                board_key = self.params.get(f"{key}__board_key")
                if board_key:
                    self.global_blackboard.register_key(key=board_key, access=Access.READ)
                    value = self.global_blackboard.get(board_key)
                else:
                    value = self.params.get(key, default)
                if isinstance(value, str) and key != "frame":
                    value = json.loads(value)
                values[key] = value
            self._skill = SetArmJointTrajSkill(hardware=None)
            result = self._skill.initialize(SetArmJointTrajParams(**values))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"SetArmJointTraj 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetArmJointTraj 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_arm_joint_traj")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
