# -*- coding: utf-8 -*-
"""校验并执行单 Tag 双臂 joint 轨迹节点（基于 BimanualJointTrajectorySkill）。"""

from __future__ import annotations

import os
from typing import Any, Optional

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from skills.atomic.refactored_sdk.bimanual_joint_trajectory import (
    BimanualJointTrajectorySkill,
    BimanualJointTrajectoryParams,
)


class NodeWheelArmSingleTag(BaseAction):
    """场景专用双臂执行节点，通过 BimanualJointTrajectorySkill 下发与管理轨迹。"""

    def __init__(self, name: str, label: str, namespace: str, params: dict[str, Any]):
        super().__init__(name, label, namespace, params)
        self._skill: Optional[BimanualJointTrajectorySkill] = None
        self._started = False
        self._timeout = float(self.params.get("timeout_sec", 30.0))
        for key in ("left_arm_joint_traj", "right_arm_joint_traj"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.READ
            )

    def initialise(self) -> None:
        self._skill = BimanualJointTrajectorySkill(get_shared_hardware())
        self._started = False

    def update(self) -> Status:
        if os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes"):
            return Status.SUCCESS

        if self._skill is None:
            self._skill = BimanualJointTrajectorySkill(get_shared_hardware())

        if not self._started:
            left = self._read_blackboard_value("left_arm_joint_traj")
            right = self._read_blackboard_value("right_arm_joint_traj")
            if left is None and right is None:
                return Status.RUNNING

            traj_params = BimanualJointTrajectoryParams(
                left_arm_joint_traj=left,
                right_arm_joint_traj=right,
                total_time=float(self.params.get("total_time", 3.0)),
                timeout=self._timeout,
            )
            init_res = self._skill.initialize(traj_params)
            if not init_res.success:
                self.feedback_message = init_res.message
                return Status.FAILURE

            self._started = True

        exec_res = self._skill.execute()
        if not exec_res.success:
            self.feedback_message = exec_res.message
            return Status.FAILURE

        if not self._skill.is_finished():
            return Status.RUNNING

        self.feedback_message = "bimanual trajectory execution completed"
        return Status.SUCCESS

    def _read_blackboard_value(self, key: str) -> Any:
        try:
            return getattr(self.global_blackboard, key)
        except (AttributeError, KeyError):
            return None

    def terminate(self, new_status: Status) -> None:
        if new_status != Status.SUCCESS and self._started and self._skill is not None:
            self._skill.cancel()
        super().terminate(new_status)

