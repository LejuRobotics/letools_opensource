# -*- coding: utf-8 -*-
"""ControlRecoverMove：长流程控制模式短恢复节点。"""

import os
import time

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "on")
    return False


@define_manifest(
    label="控制模式短恢复",
    category=["utility", "control"],
    tree_type="studio_smoke",
    description="执行 set_arm_control_mode(0)、set_focus_ee(False)、set_focus_z(False)，并可等待控制服务稳定。",
    params=[
        {"name": "arm_control_mode", "type": "int", "default": "0", "description": "手臂控制模式，默认0保持当前位置控制"},
        {"name": "focus_ee", "type": "bool", "default": "False", "description": "是否启用末端焦点"},
        {"name": "focus_z", "type": "bool", "default": "False", "description": "是否启用Z轴焦点"},
        {"name": "wait_sec", "type": "float", "default": "1.0", "description": "恢复后等待时间"},
    ],
    inputs=[],
    outputs=[],
)
class ControlRecoverMove(BaseAction):
    """长流程中用于降低控制模式累积压力的短恢复节点。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False
        self._failed = False

    def initialise(self):
        self._done = False
        self._failed = False

        if _DRY_RUN:
            self._done = True
            return

        hardware = get_shared_hardware()
        mode = int(self.params.get("arm_control_mode", 0))
        focus_ee = _to_bool(self.params.get("focus_ee", False))
        focus_z = _to_bool(self.params.get("focus_z", False))
        wait_sec = float(self.params.get("wait_sec", 1.0))

        try:
            if hasattr(hardware, "set_arm_control_mode"):
                result = hardware.set_arm_control_mode(mode)
                if not result.success:
                    self.feedback_message = f"set_arm_control_mode({mode}) failed: {result.message}"
                    self._failed = True
                    self._done = True
                    return
            if hasattr(hardware, "set_focus_ee"):
                hardware.set_focus_ee(focus_ee)
            if hasattr(hardware, "set_focus_z"):
                hardware.set_focus_z(focus_z)
            if wait_sec > 0:
                time.sleep(wait_sec)
            self._done = True
        except Exception as e:
            self.feedback_message = f"control recover failed: {e}"
            self._failed = True
            self._done = True

    def update(self):
        if self._failed:
            return Status.FAILURE
        return Status.SUCCESS if self._done else Status.RUNNING
