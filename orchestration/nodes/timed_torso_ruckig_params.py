"""躯干Ruckig配置节点；持久参数，不发送运动目标。"""

import json
import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.timed_torso_ruckig_params import (
    TimedTorsoRuckigParams as TorsoRuckigSkillParams, TimedTorsoRuckigParamsSkill,
)


@define_manifest(
    label="设置躯干Ruckig参数", category=["motion", "torso"], tree_type="studio_smoke",
    description="持久规划限制，非硬件安全限值；不发目标、不切模式、不自动恢复。",
    params=[
        {"name": "mode", "type": "string", "required": True, "description": "pose(planner2)或joint(planner3)"},
        *[{"name": key, "type": "floatArr", "required": True, "description": "4维正有限数，公共单位m/deg时间导数"}
          for key in ("velocity_max", "acceleration_max", "jerk_max")],
        *[{"name": key, "type": "floatArr", "description": "4维负有限数，省略则为对应-max"}
          for key in ("velocity_min", "acceleration_min")],
        {"name": "is_sync", "type": "bool", "default": True, "description": "规划器同步配置，不是等待运动"},
    ], inputs=[], outputs=[],
)
class TimedTorsoRuckigParams(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[TimedTorsoRuckigParamsSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            values = {}
            for key, default in (("mode", ""), ("velocity_max", None), ("acceleration_max", None),
                                 ("jerk_max", None), ("velocity_min", None), ("acceleration_min", None), ("is_sync", True)):
                board_key = self.params.get(key + "__board_key")
                if board_key:
                    self.global_blackboard.register_key(key=board_key, access=Access.READ)
                    value = self.global_blackboard.get(board_key)
                else:
                    value = self.params.get(key, default)
                values[key] = json.loads(value) if isinstance(value, str) and key != "mode" else value
            self._skill = TimedTorsoRuckigParamsSkill(hardware=None)
            result = self._skill.initialize(TorsoRuckigSkillParams(**values))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"TimedTorsoRuckigParams 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("TimedTorsoRuckigParams 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run timed_torso_ruckig_params")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
