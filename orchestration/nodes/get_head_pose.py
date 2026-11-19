"""GetHeadPose：获取头部姿态，写入指定黑板键。"""

import os
from typing import Optional

from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.get_head_pose import GetHeadPoseParams, GetHeadPoseSkill


@define_manifest(
    label="获取头部姿态",
    category=["motion", "head"],
    tree_type="studio_smoke",
    description="获取头部姿态 yaw/pitch（deg），写入黑板。",
    params=[{"name": "output_key", "type": "string", "default": "head_pose",
             "description": "获取头部姿态 yaw/pitch（deg）写入的黑板键。"}],
    inputs=[],
    outputs=[{"name": "head_pose", "type": "json", "description": "获取头部姿态 yaw/pitch（deg）。"}],
)
class GetHeadPose(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._output_key = "head_pose"
        self._dry_run = False

    def initialise(self) -> None:
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        self._output_key = str(self.params.get("output_key", "head_pose")).strip()
        if not self._output_key:
            self._result = Result.fail("output_key 不能为空")
            return
        self.global_blackboard.register_key(key=self._output_key, access=Access.WRITE)
        # 新一次读取开始时清空，失败时不留下上次的角度冒充本次结果。
        self.global_blackboard.set(self._output_key, None)

    def update(self) -> Status:
        if self._result is None:
            if self._dry_run:
                # 没有实测就不伪造零角度。
                self._result = Result.ok("dry-run get_head_pose: 未读取硬件反馈")
            else:
                try:
                    skill = GetHeadPoseSkill(hardware=get_shared_hardware())
                    self._result = skill.initialize(GetHeadPoseParams())
                    if self._result.success:
                        self._result = skill.execute()
                        if self._result.success:
                            self.global_blackboard.set(self._output_key, self._result.data)
                except Exception as exc:
                    self._result = Result.fail(f"get_head_pose 失败: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE