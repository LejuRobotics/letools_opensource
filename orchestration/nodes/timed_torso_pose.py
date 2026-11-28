"""躯干Timed节点；仅提交，可输出规划时长用于场景层显式等待。"""

import json
import math
import os
from typing import Any, Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.timed_torso_pose import TimedTorsoPoseParams, TimedTorsoPoseSkill


@define_manifest(
    label="定时躯干位姿目标", category=["motion", "torso"], tree_type="studio_smoke",
    description="MultiCmd单次请求，不等待或检查到位，不进行客户端插值。",
    params=[
        {"name": "pose", "type": "floatArr", "required": True, "description": "4维[x,z,yaw,pitch]，固定m/deg，planner2，不暴露y/roll"},
        {"name": "desire_time", "type": "float", "required": True, "description": "期望秒数，大于0"},
        {"name": "is_sync", "type": "bool", "default": True, "description": "规划时长同步，不是等待完成"},
        {"name": "prepare_arm_control", "type": "bool", "default": True,
         "description": "先请求手臂模式2，失败不发目标；可能激活旧目标，须先确认现场状态"},
        {"name": "actual_time_output_key", "type": "string",
         "description": "可选规划时长输出键，缺字段回退desire_time；节点本身不等待"},
    ], inputs=[], outputs=[],
)
class TimedTorsoPose(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[TimedTorsoPoseSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False
        self._actual_time_output_key = None
        self._desire_time = None

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._actual_time_output_key = None
        self._desire_time = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            output_key = self.params.get("actual_time_output_key")
            if output_key is not None:
                if not isinstance(output_key, str) or not output_key.strip():
                    raise ValueError("actual_time_output_key 必须为非空字符串")
                self._actual_time_output_key = output_key.strip()
                self.global_blackboard.register_key(key=self._actual_time_output_key, access=Access.WRITE)
                self.global_blackboard.set(self._actual_time_output_key, None)
            values = {key: self._param(key, default) for key, default in (
                ("pose", None), ("desire_time", None), ("is_sync", True), ("prepare_arm_control", True))}
            self._desire_time = values["desire_time"]
            self._skill = TimedTorsoPoseSkill(hardware=None)
            initialized = self._skill.initialize(TimedTorsoPoseParams(**values))
            if not initialized.success:
                self._result = initialized
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"TimedTorsoPose 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("TimedTorsoPose 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run timed_torso_pose")
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
                    self._result = Result.fail(f"TimedTorsoPose 规划时长无效: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE

    def _param(self, key: str, default: Any = None) -> Any:
        board_key = self.params.get(key + "__board_key")
        if board_key:
            self.global_blackboard.register_key(key=board_key, access=Access.READ)
            value = self.global_blackboard.get(board_key)
        else:
            value = self.params.get(key, default)
        return json.loads(value) if isinstance(value, str) else value
