# -*- coding: utf-8 -*-
"""NodeWheelArm：保留黑板关节控制入口；旧末端轨迹分支已移除。"""
import os
import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class NodeWheelArm(BaseAction):
    """仅保留关节分支，不将旧末端输入静默转换为关节运动。"""

    def __init__(self, name, label, namespace, params):
        super(NodeWheelArm, self).__init__(name, label, namespace, params)
        self._control_type = self.params.get("control_type", "joint")
        self._executed = False
        for k in [
            "left_arm_joint_traj", "right_arm_joint_traj",
        ]:
            self.global_blackboard.register_key(key=k, access=py_trees.common.Access.READ)

    def initialise(self):
        self._executed = False

    def _read_traj_pair(self, traj_key: str):
        try:
            l = getattr(self.global_blackboard, f"left_{traj_key}", None)
            r = getattr(self.global_blackboard, f"right_{traj_key}", None)
        except KeyError:
            return None, None
        return l, r

    def update(self):
        if self._control_type != "joint":
            self.feedback_message = "NodeWheelArm 仅支持 joint；旧末端轨迹分支已移除"
            return Status.FAILURE
        if _is_dry_run():
            return Status.SUCCESS

        if self._executed:
            return Status.SUCCESS

        hw = get_shared_hardware()
        try:
            l, r = self._read_traj_pair("arm_joint_traj")

            if l is None and r is None:
                return Status.RUNNING

            if not l or not r:
                self.feedback_message = "arm traj empty, skip"
                self._executed = True
                return Status.SUCCESS

            hw.send_arm_joint_trajectory(list(l) + list(r))

            self._executed = True
            return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"wheel_arm failed: {e}"
            return Status.FAILURE
