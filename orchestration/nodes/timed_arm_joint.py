"""单/双臂multi定时关节目标，不调用SDK ArmAPI轨迹循环。"""

import json
import math
import os
from typing import Any, Optional
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
        {"name": "joint_angles", "type": "floatArr",
         "description": "兼容14维deg输入[左7,右7]，与left/right_arm_joint互斥；运行期读取黑板"},
        {"name": "actual_time_output_key", "type": "string",
         "description": "可选：成功后写规划秒数，缺失时回退本次desire_time；节点自身不等待"},
        {"name": "desire_time", "type": "float", "required": True, "description": "期望秒数，大于0"},
        {"name": "is_sync", "type": "bool", "default": True, "description": "规划器执行时长同步，不是到位等待"},
        {"name": "prepare_arm_control", "type": "bool", "default": True,
         "description": "先请求模式2，失败不发令；切2可能激活旧目标，须先确认控制器状态"},
    ], inputs=[], outputs=[],
)
class TimedArmJoint(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[TimedArmJointSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False
        self._actual_time_output_key = None
        self._desire_time = None

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        self._actual_time_output_key = None
        self._desire_time = None
        try:
            output_key = self.params.get("actual_time_output_key")
            if output_key is not None:
                if not isinstance(output_key, str) or not output_key.strip():
                    raise ValueError("actual_time_output_key 必须为非空字符串")
                self._actual_time_output_key = output_key.strip()
                self.global_blackboard.register_key(key=self._actual_time_output_key, access=Access.WRITE)
                # Clear a previous invocation's duration, including on initialization/send failure.
                self.global_blackboard.set(self._actual_time_output_key, None)
            values = {
                key: self._param(key, default)
                for key, default in (("left_arm_joint", None), ("right_arm_joint", None),
                                     ("desire_time", None), ("is_sync", True), ("prepare_arm_control", True))
            }
            joint_angles = self._param("joint_angles", None)
            if joint_angles is not None:
                if values["left_arm_joint"] is not None or values["right_arm_joint"] is not None:
                    raise ValueError("joint_angles 与 left/right_arm_joint 不能同时提供")
                if not isinstance(joint_angles, (list, tuple)) or len(joint_angles) != 14:
                    raise ValueError("joint_angles 必须为14维[左7,右7]")
                values["left_arm_joint"] = list(joint_angles[:7])
                values["right_arm_joint"] = list(joint_angles[7:])
            self._desire_time = values["desire_time"]
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
            if self._result.success and self._actual_time_output_key:
                data = self._result.data if isinstance(self._result.data, dict) else {}
                duration = data.get("actual_time", self._desire_time)
                try:
                    if isinstance(duration, bool):
                        raise ValueError("actual_time 不能为bool")
                    duration = float(duration)
                    if not math.isfinite(duration) or duration < 0:
                        raise ValueError("actual_time 必须为非负有限秒数")
                    self.global_blackboard.set(self._actual_time_output_key, duration)
                except (TypeError, ValueError, OverflowError) as exc:
                    self._result = Result.fail(f"TimedArmJoint 规划时长无效: {exc}")
        self.feedback_message = self._result.message
        if self._result.success and isinstance(self._result.data, dict):
            self.feedback_message += f"; actual_time={self._result.data.get('actual_time')}s（不是到位确认）"
        return Status.SUCCESS if self._result.success else Status.FAILURE

    def _param(self, key: str, default: Any = None) -> Any:
        board_key = self.params.get(f"{key}__board_key")
        if board_key:
            self.global_blackboard.register_key(key=board_key, access=Access.READ)
            value = self.global_blackboard.get(board_key)
        else:
            value = self.params.get(key, default)
        return json.loads(value) if isinstance(value, str) else value
