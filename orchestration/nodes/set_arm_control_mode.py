"""设置手臂控制模式薄节点：0固定、1AutoSwing、2外部控制。"""

import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_arm_control_mode import SetArmControlModeParams, SetArmControlModeSkill


@define_manifest(
    label="设置手臂控制模式", category=["motion", "arm"], tree_type="studio_smoke",
    description="通过SDK设置0固定、1AutoSwing（可能运动）、2外部控制；不等待到位。",
    params=[{"name": "control_mode", "type": "int", "required": True,
             "description": "0固定、1AutoSwing、2外部控制，无默认值"}],
    inputs=[], outputs=[],
)
class SetArmControlMode(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[SetArmControlModeSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            board_key = self.params.get("control_mode__board_key")
            if board_key:
                self.global_blackboard.register_key(key=board_key, access=Access.READ)
                mode = self.global_blackboard.get(board_key)
            else:
                mode = self.params.get("control_mode")
            self._skill = SetArmControlModeSkill(hardware=None)
            result = self._skill.initialize(SetArmControlModeParams(control_mode=mode))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"SetArmControlMode 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetArmControlMode 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_arm_control_mode")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
