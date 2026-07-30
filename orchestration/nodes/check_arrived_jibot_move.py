# -*- coding: utf-8 -*-
"""CheckArrivedJibotMove：JiBot底盘任务到达检查薄节点 → check_arrived_jibot 原子技能。"""

import os

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.check_arrived_jibot import CheckArrivedJibotParams, CheckArrivedJibotSkill

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="JiBot底盘任务到达检查",
    category=["motion", "chassis", "jibot"],
    tree_type="studio_smoke",
    description="对齐 test_check_arrived.py：调用 hardware.check_arrived_jibot()",
    params=[
        {"name": "task_id", "type": "string", "default": "", "description": "由base_move或move_to_target返回的任务ID"},
        {"name": "task_id_key", "type": "string", "default": "current_task_id", "description": "从黑板读取task_id的键名(优先级高于task_id参数)"},
        {"name": "blocking", "type": "bool", "default": "True", "description": "是否阻塞等待任务完成"},
        {"name": "timeout", "type": "float", "default": "20.0", "description": "超时时间(s)，blocking=True时有效"},
    ],
    inputs=[],
    outputs=[],
)
class CheckArrivedJibotMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._last_result = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None
        self._last_result = None
        if _DRY_RUN:
            return

        task_id = str(self.params.get("task_id", ""))
        task_id_key = str(self.params.get("task_id_key", "current_task_id"))
        
        if task_id_key:
            try:
                self.global_blackboard.register_key(key=task_id_key, access=py_trees.common.Access.READ)
            except AttributeError:
                pass
            try:
                task_id = self.global_blackboard.get(task_id_key)
            except (KeyError, AttributeError):
                task_id = ""
        
        if not task_id:
            task_id = str(self.params.get("task_id", ""))
        
        skill_params = CheckArrivedJibotParams(
            task_id=task_id,
            blocking=bool(self.params.get("blocking", True)),
            timeout=float(self.params.get("timeout", 20.0)),
        )
        self._skill = CheckArrivedJibotSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "check_arrived_jibot init failed"

    def _status_from_result(self, result):
        if result is None:
            self.feedback_message = "check_arrived_jibot finished without a result"
            return Status.FAILURE

        if not result.success:
            self.feedback_message = result.message or "check_arrived_jibot failed"
            return Status.FAILURE

        data = result.data if isinstance(result.data, dict) else {}
        if bool(data.get("arrived", False)):
            self.feedback_message = data.get("message") or "arrived"
            return Status.SUCCESS

        status = data.get("status", "unknown")
        message = data.get("message") or result.message or "not arrived"
        self.feedback_message = (
            f"JiBot did not arrive: status={status}, message={message}"
        )
        return Status.FAILURE

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run check_arrived_jibot"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return self._status_from_result(self._last_result)
        result = self._skill.execute()
        self._last_result = result
        if self._skill.is_finished():
            return self._status_from_result(result)
        if not result.success:
            return self._status_from_result(result)
        return Status.RUNNING
