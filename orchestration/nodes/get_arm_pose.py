"""双臂末端反馈节点，默认local/TF，可显式选择FK及其坐标基准。"""

import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.get_arm_pose import GetArmPoseParams, GetArmPoseSkill


@define_manifest(
    label="获取双臂末端位姿", category=["motion", "arm"], tree_type="studio_smoke",
    description="双臂6维反馈（m/deg），TF或反馈关节FK，不保证新鲜度。",
    params=[
        {"name": "frame", "type": "string", "default": "local", "description": "local或world"},
        {"name": "source", "type": "string", "default": "tf", "description": "tf或fk"},
        {"name": "fk_frame", "type": "string", "description": "选择fk时必填，确认FK输出的TF frame"},
        {"name": "output_key", "type": "string", "default": "arm_pose", "description": "双臂反馈写入键"},
    ], inputs=[], outputs=[{"name": "arm_pose", "type": "json", "description": "left_pose/right_pose/frame/source"}],
)
class GetArmPose(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[GetArmPoseSkill] = None
        self._result: Optional[Result] = None
        self._output_key = "arm_pose"
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        self._output_key = str(self.params.get("output_key", "arm_pose")).strip()
        if not self._output_key:
            self._result = Result.fail("output_key 不能为空")
            return
        self.global_blackboard.register_key(key=self._output_key, access=Access.WRITE)
        self.global_blackboard.set(self._output_key, None)
        try:
            values = {}
            for key, default in (("frame", "local"), ("source", "tf"), ("fk_frame", None)):
                board_key = self.params.get(f"{key}__board_key")
                if board_key:
                    self.global_blackboard.register_key(key=board_key, access=Access.READ)
                    values[key] = self.global_blackboard.get(board_key)
                else:
                    values[key] = self.params.get(key, default)
            self._skill = GetArmPoseSkill(hardware=None)
            result = self._skill.initialize(GetArmPoseParams(**values))
            if not result.success:
                self._result = result
        except Exception as exc:
            self._result = Result.fail(f"GetArmPose 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("GetArmPose 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run get_arm_pose: 未读取反馈")
            else:
                try:
                    self._skill.hardware = get_shared_hardware()
                    self._result = self._skill.execute()
                    if self._result.success:
                        self.global_blackboard.set(self._output_key, self._result.data)
                except Exception as exc:
                    self._result = Result.fail(f"get_arm_pose 失败: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
