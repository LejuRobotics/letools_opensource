"""SDK TorsoAPI同步轨迹节点，会阻塞当前tick，不保证到位。"""

import json
import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_torso_joint_traj import SetTorsoJointTrajParams, SetTorsoJointTrajSkill


@define_manifest(
    label="躯干关节轨迹（SDK）", category=["motion", "torso"], tree_type="studio_smoke",
    description="SDK插入当前起点，切MPC ArmOnly同步重采样下发，不恢复模式、不保证到位。",
    params=[
        {"name": "joint_traj", "type": "json", "required": True, "description": "至少一个4维deg关键帧"},
        {"name": "total_time", "type": "float", "required": True, "description": "SDK总时长秒，至少0.01"},
    ], inputs=[], outputs=[],
)
class SetTorsoJointTraj(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[SetTorsoJointTrajSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._result = None
        self._skill = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            values = {}
            for key in ("joint_traj", "total_time"):
                board_key = self.params.get(key + "__board_key")
                if board_key:
                    self.global_blackboard.register_key(key=board_key, access=Access.READ)
                    value = self.global_blackboard.get(board_key)
                else:
                    value = self.params.get(key)
                values[key] = json.loads(value) if isinstance(value, str) else value
            self._skill = SetTorsoJointTrajSkill(hardware=None)
            initialized = self._skill.initialize(SetTorsoJointTrajParams(**values))
            if not initialized.success:
                self._result = initialized
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"SetTorsoJointTraj 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetTorsoJointTraj 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_torso_joint_traj")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
