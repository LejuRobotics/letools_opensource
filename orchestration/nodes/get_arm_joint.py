"""读取双臂关节反馈（deg）到黑板，不伪造dry-run反馈。"""

import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.get_arm_joint import GetArmJointParams, GetArmJointSkill


@define_manifest(
    label="获取双臂关节", category=["motion", "arm"], tree_type="studio_smoke",
    description="读取14维双臂关节反馈缓存（deg），不保证新鲜度。",
    params=[{"name": "output_key", "type": "string", "default": "arm_joint",
             "description": "关节角列表写入的黑板键"}],
    inputs=[], outputs=[{"name": "arm_joint", "type": "floatArr", "description": "14维关节角（deg）"}],
)
class GetArmJoint(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._output_key = "arm_joint"
        self._dry_run = False

    def initialise(self) -> None:
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        self._output_key = str(self.params.get("output_key", "arm_joint")).strip()
        if not self._output_key:
            self._result = Result.fail("output_key 不能为空")
            return
        self.global_blackboard.register_key(key=self._output_key, access=Access.WRITE)
        self.global_blackboard.set(self._output_key, None)

    def update(self) -> Status:
        if self._result is None:
            if self._dry_run:
                self._result = Result.ok("dry-run get_arm_joint: 未读取反馈")
            else:
                try:
                    skill = GetArmJointSkill(get_shared_hardware())
                    self._result = skill.initialize(GetArmJointParams())
                    if self._result.success:
                        self._result = skill.execute()
                        if self._result.success:
                            self.global_blackboard.set(self._output_key, self._result.data)
                except Exception as exc:
                    self._result = Result.fail(f"get_arm_joint 失败: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
