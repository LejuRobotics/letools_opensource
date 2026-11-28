"""MPC独立服务节点，不自动设置其他模式；查询无缓存回退。"""

import json
import os
from typing import Optional
from py_trees.common import Access, Status
from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.set_mpc_frame import SetMpcFrameParams, SetMpcFrameSkill


@define_manifest(label="SetMpcFrame", category=["control", "mpc"], tree_type="studio_smoke",
    description="单次ROS服务；不切其他模式、不自动恢复。服务成功不证明控制行为生效。",
    params=[{"name":"frame","type":"int","required":True,"description":"frame整数，严格按公共枚举取值"}], inputs=[], outputs=[])
class SetMpcFrame(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._skill: Optional[SetMpcFrameSkill] = None
        self._dry_run = False
        self._output_key = "mpc_frame"

    def initialise(self) -> None:
        self._result = None
        self._skill = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            board_key = self.params.get("frame__board_key")
            if board_key:
                self.global_blackboard.register_key(key=board_key, access=Access.READ)
                value = self.global_blackboard.get(board_key)
            else:
                value = self.params.get("frame")
            value = json.loads(value) if isinstance(value, str) else value
            params = SetMpcFrameParams(frame=value)
            self._skill = SetMpcFrameSkill(hardware=None)
            result = self._skill.initialize(params)
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"SetMpcFrame 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("SetMpcFrame 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run set_mpc_frame：未访问硬件")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
