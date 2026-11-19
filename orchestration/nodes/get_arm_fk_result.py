"""双臂FK计算节点，不发送运动；dry-run不伪造结果。"""

import json
import os
from typing import Optional
from py_trees.common import Access, Status
from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.get_arm_fk_result import GetArmFkResultParams, GetArmFkResultSkill


@define_manifest(
    label="获取双臂FK计算结果", category=["motion", "arm"], tree_type="studio_smoke",
    description="纯计算，不发送运动，输入输出角度deg，位置m。",
    params=[{'name': 'joint', 'type': 'floatArr', 'required': True, 'description': '14维关节角（deg）'},
     {'name': 'frame', 'type': 'string', 'default': 'local', 'description': 'local或world'},
     {'name': 'fk_frame',
      'type': 'string',
      'required': True,
      'description': '已确认模型基准对应整机TF frame'},
     {'name': 'output_key',
      'type': 'string',
      'default': 'arm_fk_result',
      'description': '计算结果黑板键'}],
    inputs=[], outputs=[{"name": "arm_fk_result", "type": "json", "description": "计算结果，不是到位反馈"}],
)
class GetArmFkResult(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._result: Optional[Result] = None
        self._skill: Optional[GetArmFkResultSkill] = None
        self._output_key = "arm_fk_result"
        self._dry_run = False

    def initialise(self) -> None:
        self._result = None
        self._skill = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        key = self.params.get("output_key", "arm_fk_result")
        if not isinstance(key, str) or not key.strip():
            self._result = Result.fail("output_key必须为非空字符串")
            return
        self._output_key = key.strip()
        self.global_blackboard.register_key(key=self._output_key, access=Access.WRITE)
        self.global_blackboard.set(self._output_key, None)
        try:
            values = {}
            for key, default in (("joint", None), ("frame", "local"), ("fk_frame", None)):
                board_key = self.params.get(f"{key}__board_key")
                if board_key:
                    self.global_blackboard.register_key(key=board_key, access=Access.READ)
                    value = self.global_blackboard.get(board_key)
                else:
                    value = self.params.get(key, default)
                if key in ("joint", "left_pose", "right_pose") and isinstance(value, str):
                    value = json.loads(value)
                values[key] = value
            self._skill = GetArmFkResultSkill(hardware=None)
            result = self._skill.initialize(GetArmFkResultParams(**values))
            if not result.success:
                self._result = result
        except Exception as exc:
            self._result = Result.fail(f"GetArmFkResult 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("GetArmFkResult 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run get_arm_fk_result: 未计算")
            else:
                try:
                    self._skill.hardware = get_shared_hardware()
                    self._result = self._skill.execute()
                    if self._result.success:
                        self.global_blackboard.set(self._output_key, self._result.data)
                except Exception as exc:
                    self._result = Result.fail(f"get_arm_fk_result 失败: {exc}")
        self.feedback_message = self._result.message
        return Status.SUCCESS if self._result.success else Status.FAILURE
