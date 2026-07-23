# -*- coding: utf-8 -*-
"""ChassisStopMove：底盘停止薄节点 → chassis_stop 原子技能。

调用 /enable_vel_control 服务切换底盘控制权限：
- enable=True  → 停止导航运动（速度控制模式开启）
- enable=False → 交还导航控制

对齐 kuavo 原始掉落处理中的 set_enable_vel_control(True, force=True)。
"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.chassis_stop import (
    ChassisStopParams,
    ChassisStopSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="底盘停止（速度控制切换）",
    category=["motion", "chassis", "safety"],
    tree_type="studio_smoke",
    description="调用 /enable_vel_control 切换底盘控制权限，enable=True 停止导航运动",
    params=[
        {"name": "enable", "type": "bool", "default": True,
         "description": "True=停止导航(速度控制开启), False=交还导航控制"},
    ],
    inputs=[],
    outputs=[],
)
class ChassisStopMove(BaseAction):
    """底盘停止节点

    用法示例 (py_tree_child.json):
    {
      "name": "ChassisStopMove",
      "label": "chassis_stop",
      "params": {
        "enable": { "value": true, "source": "CUSTOM", "data_type": "bool" }
      },
      "childs": [],
      "childBoard": []
    }
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        raw_enable = self.params.get("enable", True)
        if isinstance(raw_enable, dict) and "value" in raw_enable:
            raw_enable = raw_enable["value"]
        if isinstance(raw_enable, str):
            enable = raw_enable.strip().lower() in ("true", "1", "yes")
        else:
            enable = bool(raw_enable)

        if _DRY_RUN:
            self.feedback_message = f"dry-run chassis_stop enable={enable}"
            self._dry_done = True
            return

        skill_params = ChassisStopParams(enable=enable)
        self._skill = ChassisStopSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "chassis_stop init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is None:
            return Status.FAILURE

        if self._skill.is_finished():
            return Status.SUCCESS

        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "chassis_stop failed"
            return Status.FAILURE

        return Status.RUNNING
