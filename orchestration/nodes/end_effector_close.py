# -*- coding: utf-8 -*-
"""EndEffectorClose:闭合夹爪 → IHardware.control_end_effector(spec § 3.1)。"""
import os
from py_trees.common import Status

from core.domain.enums import ArmSide
from core.domain.end_effector import GripperCommand
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


# side 字符串 → ArmSide 映射
_SIDE_MAP = {
    "left": ArmSide.LEFT,
    "right": ArmSide.RIGHT,
    "both": ArmSide.BOTH,
}


class EndEffectorClose(BaseAction):
    """闭合夹爪,默认 LEFT+RIGHT 同时闭合,position=100,effort=1.0。"""

    def __init__(self, name, label, namespace, params):
        super(EndEffectorClose, self).__init__(name, label, namespace, params)

    def initialise(self):
        pass

    def update(self):
        if _is_dry_run():
            self.feedback_message = "dry-run end_effector_close"
            return Status.SUCCESS

        side_str = self.params.get("side", "both")
        position = float(self.params.get("position", 100.0))
        velocity = float(self.params.get("velocity", 50.0))
        effort = float(self.params.get("effort", 1.0))

        arm_side = _SIDE_MAP.get(side_str, ArmSide.BOTH)
        cmd = GripperCommand(position=position, velocity=velocity, effort=effort)
        hw = get_shared_hardware()
        try:
            if arm_side == ArmSide.BOTH:
                hw.control_end_effector(ArmSide.LEFT, cmd)
                hw.control_end_effector(ArmSide.RIGHT, cmd)
            else:
                hw.control_end_effector(arm_side, cmd)
            self.feedback_message = f"gripper closed (side={side_str}, pos={position})"
            return Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"end_effector_close failed: {e}"
            return Status.FAILURE
