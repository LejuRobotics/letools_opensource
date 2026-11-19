"""双臂末端发送节点（m/deg），默认local，必须同时提供左右臂。"""

import json
import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_arm_pose import SetArmPoseParams, SetArmPoseSkill


@define_manifest(
    label="设置双臂末端位姿", category=["motion", "arm"], tree_type="studio_smoke",
    description="双臂[x,y,z,yaw,pitch,roll]（m/deg），不等待到位。",
    params=[
        {"name": "left_pose", "type": "floatArr", "required": True, "description": "左臂6维位姿（m/deg）"},
        {"name": "right_pose", "type": "floatArr", "required": True, "description": "右臂6维位姿（m/deg）"},
        {"name": "frame", "type": "string", "default": "local", "description": "local或world"},
    ], inputs=[], outputs=[],
)
class SetArmPose(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[SetArmPoseSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            values = {}
            for key in ("left_pose", "right_pose", "frame"):
                board_key = self.params.get(f"{key}__board_key")
                if board_key:
                    self.global_blackboard.register_key(key=board_key, access=Access.READ)
                    value = self.global_blackboard.get(board_key)
                else:
                    value = self.params.get(key, "local" if key == "frame" else None)
                values[key] = json.loads(value) if key != "frame" and isinstance(value, str) else value
            self._skill = SetArmPoseSkill(hardware=None)
            result = self._skill.initialize(SetArmPoseParams(**values))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"SetArmPose 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetArmPose 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_arm_pose")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
