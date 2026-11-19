"""读取SDK手臂模式到黑板，dry-run不伪造模式值。"""

import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.get_arm_control_mode import GetArmControlModeParams, GetArmControlModeSkill


@define_manifest(
    label="获取手臂控制模式", category=["motion", "arm"], tree_type="studio_smoke",
    description="SDK查询模式0/1/2及name，可能回退到缓存，不代表在线确认。",
    params=[{"name": "output_key", "type": "string", "default": "arm_control_mode",
             "description": "模式字典写入的黑板键"}],
    inputs=[], outputs=[{"name": "arm_control_mode", "type": "json", "description": "mode和name"}],
)
class GetArmControlMode(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._output_key = "arm_control_mode"
        self._dry_run = False

    def initialise(self) -> None:
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        raw_key = self.params.get("output_key", "arm_control_mode")
        if not isinstance(raw_key, str) or not raw_key.strip():
            self._result = Result.fail("output_key 必须为非空字符串")
            return
        self._output_key = raw_key.strip()
        self.global_blackboard.register_key(key=self._output_key, access=Access.WRITE)
        self.global_blackboard.set(self._output_key, None)

    def update(self) -> Status:
        if self._result is None:
            if self._dry_run:
                self._result = Result.ok("dry-run get_arm_control_mode: 未查询硬件")
            else:
                try:
                    skill = GetArmControlModeSkill(get_shared_hardware())
                    self._result = skill.initialize(GetArmControlModeParams())
                    if self._result.success:
                        self._result = skill.execute()
                        if self._result.success:
                            self.global_blackboard.set(self._output_key, self._result.data)
                except Exception as exc:
                    self._result = Result.fail(f"get_arm_control_mode 失败: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
