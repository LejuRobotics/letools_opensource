"""单帧发送躯干4关节目标，不自动切模式、不等待到位。"""

import json
import os
from typing import Optional

from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_torso_joint import SetTorsoJointParams, SetTorsoJointSkill


@define_manifest(
    label="设置躯干关节（单帧）", category=["motion", "torso"], tree_type="studio_smoke",
    description="仅发送4维deg目标，不插值、不切模式、不保证运动完成。",
    params=[{"name": "joint", "type": "floatArr", "required": True,
             "description": "[knee,leg,waist_pitch,waist_yaw]，固定deg，无默认运动目标"}],
    inputs=[], outputs=[],
)
class SetTorsoJoint(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[SetTorsoJointSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            board_key = self.params.get("joint__board_key")
            if board_key:
                self.global_blackboard.register_key(key=board_key, access=Access.READ)
                joint = self.global_blackboard.get(board_key)
            else:
                joint = self.params.get("joint")
            if isinstance(joint, str):
                joint = json.loads(joint)
            self._skill = SetTorsoJointSkill(hardware=None)
            result = self._skill.initialize(SetTorsoJointParams(joint=joint))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"SetTorsoJoint 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetTorsoJoint 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_torso_joint")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
