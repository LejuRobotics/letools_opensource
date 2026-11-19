"""SetArmJoint：单次发送双臂14维关节角（deg），不等待到位。"""

import json
import os
from typing import Optional

from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_arm_joint import SetArmJointParams, SetArmJointSkill


@define_manifest(
    label="设置双臂关节",
    category=["motion", "arm"], tree_type="studio_smoke",
    description="发送14维双臂关节目标（deg），不等待到位。",
    params=[{"name": "joint", "type": "floatArr", "required": True,
             "description": "14个关节角（deg），左臂7个、右臂7个"}],
    inputs=[], outputs=[],
)
class SetArmJoint(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[SetArmJointSkill] = None
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
            self._skill = SetArmJointSkill(hardware=None)
            result = self._skill.initialize(SetArmJointParams(joint=joint))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"SetArmJoint 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetArmJoint 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_arm_joint")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
