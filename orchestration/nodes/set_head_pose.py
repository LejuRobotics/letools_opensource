"""SetHeadPose：设置头部姿态。"""

import os
from typing import Optional

from py_trees.common import Status

from core.common.logger import get_logger
from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.blackboard_path import read_blackboard_path
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_head_pose import SetHeadPoseParams, SetHeadPoseSkill

logger = get_logger(__name__)


@define_manifest(
    label="设置头部姿态",
    category=["motion", "head"],
    tree_type="studio_smoke",
    description="设置头部姿态 yaw/pitch（deg）。",
    params=[
        {"name": "yaw", "type": "float", "required": True, "description": "yaw（deg）。"},
        {"name": "pitch", "type": "float", "required": True, "description": "pitch（deg）。"},
    ],
    inputs=[],
    outputs=[],
)
class SetHeadPose(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[SetHeadPoseSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            values = {}
            for key in ("yaw", "pitch"):
                board_key = self.params.get(f"{key}__board_key")
                if board_key:
                    values[key] = read_blackboard_path(self.global_blackboard, board_key)
                else:
                    values[key] = self.params.get(key)
            # dry-run 仍校验参数，但不创建共享硬件。
            self._skill = SetHeadPoseSkill(hardware=None)
            result = self._skill.initialize(SetHeadPoseParams(**values))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            logger.exception("初始化 SetHeadPose 失败")
            self._result = Result.fail(str(exc))

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetHeadPose 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_head_pose")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE