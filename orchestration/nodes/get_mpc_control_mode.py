"""MPC独立服务节点，不自动设置其他模式；查询无缓存回退。"""

import json
import os
from typing import Optional
from py_trees.common import Access, Status
from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.get_mpc_control_mode import GetMpcControlModeParams, GetMpcControlModeSkill


@define_manifest(label="GetMpcControlMode", category=["control", "mpc"], tree_type="studio_smoke",
    description="单次ROS服务；不切其他模式、不自动恢复。服务成功不证明控制行为生效。",
    params=[{"name":"output_key","type":"string","default":"mpc_control_mode","description":"服务查询结果输出键"}], inputs=[], outputs=[{"name":"mpc_control_mode","type":"json","description":"值与名称"}])
class GetMpcControlMode(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._skill: Optional[GetMpcControlModeSkill] = None
        self._dry_run = False
        self._output_key = "mpc_control_mode"

    def initialise(self) -> None:
        self._result = None
        self._skill = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            key = self.params.get("output_key", "mpc_control_mode")
            if not isinstance(key, str) or not key.strip():
                raise ValueError("output_key 必须为非空字符串")
            self._output_key = key.strip()
            self.global_blackboard.register_key(key=self._output_key, access=Access.WRITE)
            self.global_blackboard.set(self._output_key, None)
            params = GetMpcControlModeParams()
            self._skill = GetMpcControlModeSkill(hardware=None)
            result = self._skill.initialize(params)
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"GetMpcControlMode 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("GetMpcControlMode 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run get_mpc_control_mode：未访问硬件")
            else:
                self._result = self._skill.execute()
                if self._result.success:
                    self.global_blackboard.set(self._output_key, self._result.data)
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
