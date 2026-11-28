"""ROS复位请求，成功仅表示接受，不等待到位。"""

import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.reset_torso import ResetTorsoParams, ResetTorsoSkill


@define_manifest(
    label="ResetTorso", category=["motion", "torso"], tree_type="studio_smoke",
    description="ROS复位请求，成功仅表示接受，不等待到位。",
    params=[], inputs=[], outputs=[],
)
class ResetTorso(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

    def update(self) -> Status:
        if self._result is None:
            if self._dry_run:
                self._result = Result.ok("dry-run reset_torso：未访问硬件")
            else:
                try:
                    skill = ResetTorsoSkill(get_shared_hardware())
                    self._result = skill.initialize(ResetTorsoParams())
                    if self._result.success:
                        self._result = skill.execute()
                except Exception as exc:
                    self._result = Result.fail(f"reset_torso 失败: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
