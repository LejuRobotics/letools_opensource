"""base_link系当前TF，返回四分量m/deg，不保证新鲜度。"""

import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.get_torso_pose import GetTorsoPoseParams, GetTorsoPoseSkill


@define_manifest(
    label="GetTorsoPose", category=["motion", "torso"], tree_type="studio_smoke",
    description="base_link系当前TF，返回四分量m/deg，不保证新鲜度。",
    params=[{"name": "output_key", "type": "string", "default": "torso_pose", "description": "结果字典的黑板键"}], inputs=[], outputs=[{"name": "torso_pose", "type": "json", "description": "pose/frame/source元数据"}],
)
class GetTorsoPose(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._dry_run = False
        self._output_key = "torso_pose"

    def initialise(self) -> None:
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            key = self.params.get("output_key", "torso_pose")
            if not isinstance(key, str) or not key.strip():
                raise ValueError("output_key 必须为非空字符串")
            self._output_key = key.strip()
            self.global_blackboard.register_key(key=self._output_key, access=Access.WRITE)
            self.global_blackboard.set(self._output_key, None)
        except Exception as exc:
            self._result = Result.fail(f"GetTorsoPose 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._dry_run:
                self._result = Result.ok("dry-run get_torso_pose：未访问硬件")
            else:
                try:
                    skill = GetTorsoPoseSkill(get_shared_hardware())
                    self._result = skill.initialize(GetTorsoPoseParams())
                    if self._result.success:
                        self._result = skill.execute()
                        if self._result.success:
                            self.global_blackboard.set(self._output_key, self._result.data)
                except Exception as exc:
                    self._result = Result.fail(f"get_torso_pose 失败: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
