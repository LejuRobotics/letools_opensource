"""ROS初始目标查询，服务无frame字段，不是当前反馈。"""

import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.get_torso_pose_initial import GetTorsoPoseInitialParams, GetTorsoPoseInitialSkill


@define_manifest(
    label="GetTorsoPoseInitial", category=["motion", "torso"], tree_type="studio_smoke",
    description="ROS初始目标查询，服务无frame字段，不是当前反馈。",
    params=[{"name": "output_key", "type": "string", "default": "torso_pose_initial", "description": "结果字典的黑板键"}], inputs=[], outputs=[{"name": "torso_pose_initial", "type": "json", "description": "pose/frame/source元数据"}],
)
class GetTorsoPoseInitial(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._dry_run = False
        self._output_key = "torso_pose_initial"

    def initialise(self) -> None:
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            key = self.params.get("output_key", "torso_pose_initial")
            if not isinstance(key, str) or not key.strip():
                raise ValueError("output_key 必须为非空字符串")
            self._output_key = key.strip()
            self.global_blackboard.register_key(key=self._output_key, access=Access.WRITE)
            self.global_blackboard.set(self._output_key, None)
        except Exception as exc:
            self._result = Result.fail(f"GetTorsoPoseInitial 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._dry_run:
                self._result = Result.ok("dry-run get_torso_pose_initial：未访问硬件")
            else:
                try:
                    skill = GetTorsoPoseInitialSkill(get_shared_hardware())
                    self._result = skill.initialize(GetTorsoPoseInitialParams())
                    if self._result.success:
                        self._result = skill.execute()
                        if self._result.success:
                            self.global_blackboard.set(self._output_key, self._result.data)
                except Exception as exc:
                    self._result = Result.fail(f"get_torso_pose_initial 失败: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
