"""躯干4关节反馈写黑板；dry-run不伪造反馈。"""

import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.get_torso_joint import GetTorsoJointParams, GetTorsoJointSkill


@define_manifest(
    label="获取躯干关节", category=["motion", "torso"], tree_type="studio_smoke",
    description="4关节反馈缓存（deg），不保证首帧或新鲜度。",
    params=[{"name": "output_key", "type": "string", "default": "torso_joint", "description": "反馈输出键"}],
    inputs=[], outputs=[{"name": "torso_joint", "type": "floatArr", "description": "[knee,leg,waist_pitch,waist_yaw] deg"}],
)
class GetTorsoJoint(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._output_key = "torso_joint"
        self._dry_run = False

    def initialise(self) -> None:
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            key = self.params.get("output_key", "torso_joint")
            if not isinstance(key, str) or not key.strip():
                raise ValueError("output_key 必须为非空字符串")
            self._output_key = key.strip()
            self.global_blackboard.register_key(key=self._output_key, access=Access.WRITE)
            self.global_blackboard.set(self._output_key, None)
        except Exception as exc:
            self._result = Result.fail(f"GetTorsoJoint 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._dry_run:
                self._result = Result.ok("dry-run get_torso_joint：未读取反馈")
            else:
                try:
                    skill = GetTorsoJointSkill(get_shared_hardware())
                    self._result = skill.initialize(GetTorsoJointParams())
                    if self._result.success:
                        self._result = skill.execute()
                        if self._result.success:
                            self.global_blackboard.set(self._output_key, self._result.data)
                except Exception as exc:
                    self._result = Result.fail(f"get_torso_joint 失败: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
