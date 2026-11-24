"""单/双臂multi定时关节目标，不调用SDK ArmAPI轨迹循环。"""

import json
import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.timed_arm_joint import TimedArmJointParams, TimedArmJointSkill


@define_manifest(
    label="定时单/双臂关节目标（MultiCmd）", category=["motion", "arm"], tree_type="studio_smoke",
    description="左/右7维deg目标可选，planner8/9，服务返回不等于到位。",
    params=[
        {"name": "left_arm_joint", "type": "floatArr", "description": "左臂7维deg，省略表示不提交"},
        {"name": "right_arm_joint", "type": "floatArr", "description": "右臂7维deg，至少提供一侧"},
        {"name": "desire_time", "type": "float", "required": True, "description": "期望秒数，大于0"},
        {"name": "is_sync", "type": "bool", "default": True, "description": "规划器执行时长同步，不是到位等待"},
    ], inputs=[], outputs=[],
)
class TimedArmJoint(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[TimedArmJointSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            values = {}
            for key, default in (("left_arm_joint", None), ("right_arm_joint", None),
                                 ("desire_time", None), ("is_sync", True)):
                board_key = self.params.get(f"{key}__board_key")
                if board_key:
                    self.global_blackboard.register_key(key=board_key, access=Access.READ)
                    value = self.global_blackboard.get(board_key)
                else:
                    value = self.params.get(key, default)
                values[key] = json.loads(value) if isinstance(value, str) else value
            self._skill = TimedArmJointSkill(hardware=None)
            result = self._skill.initialize(TimedArmJointParams(**values))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"TimedArmJoint 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("TimedArmJoint 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run timed_arm_joint")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        if self._result.success and isinstance(self._result.data, dict):
            self.feedback_message += f"; actual_time={self._result.data.get('actual_time')}s（不是到位确认）"
        return Status.SUCCESS if self._result.success else Status.FAILURE
