"""单帧发送躯干4分量位姿，不自动切模式或坐标系，不等待到位。"""

import json
import os
from typing import Optional

from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_torso_pose import SetTorsoPoseParams, SetTorsoPoseSkill


@define_manifest(
    label="设置躯干位姿（单帧）", category=["motion", "torso"], tree_type="studio_smoke",
    description="仅发送4分量m/deg位姿，不做TF转换，y/roll固定为0，不保证运动完成。",
    params=[{"name": "pose", "type": "floatArr", "required": True,
             "description": "[x,z,yaw,pitch]，固定m/deg，沿用控制器当前参考语义"}],
    inputs=[], outputs=[],
)
class SetTorsoPose(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[SetTorsoPoseSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            board_key = self.params.get("pose__board_key")
            if board_key:
                self.global_blackboard.register_key(key=board_key, access=Access.READ)
                pose = self.global_blackboard.get(board_key)
            else:
                pose = self.params.get("pose")
            if isinstance(pose, str):
                pose = json.loads(pose)
            self._skill = SetTorsoPoseSkill(hardware=None)
            result = self._skill.initialize(SetTorsoPoseParams(pose=pose))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"SetTorsoPose 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetTorsoPose 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_torso_pose")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
