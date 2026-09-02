# -*- coding: utf-8 -*-
"""GraspRingMove：抓取圆环薄节点 → grasp_ring Skill。

薄节点职责：
  - 参数解析（board JSON dict → GraspRingParams dataclass）
  - py_trees 生命周期（initialise / update / terminate）
  - 委托 GraspRingSkill 执行全部业务逻辑
"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from skills.atomic.refactored_sdk.grasp_ring import (
    GraspRingParams,
    GraspRingSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class GraspRingMove(BaseAction):
    """薄节点：参数解析 + py_trees 生命周期 → GraspRingSkill。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._init_result = None

    def initialise(self):
        try:
            hw = get_shared_hardware()
        except Exception:
            hw = None

        skill_params = GraspRingParams(
            q_pre=list(self.params.get("q_pre", [])),
            grasp_offset_z=float(self.params.get("grasp_offset_z", 0.05)),
            pre_grasp_offset_z=float(self.params.get("pre_grasp_offset_z", 0.15)),
            ee_to_tip_z=float(self.params.get("ee_to_tip_z", -0.1)),
            ring_name=str(self.params.get("ring_name", "ring_01")),
            gripper_pre_position=float(self.params.get("gripper_pre_position", 50)),
            gripper_close_position=float(self.params.get("gripper_close_position", 0)),
        )
        self._skill = GraspRingSkill(hardware=hw)
        self._init_result = self._skill.initialize(skill_params)
        if not self._init_result.success:
            self.feedback_message = self._init_result.message or "init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS

        if self._skill is None:
            return Status.FAILURE

        if not self._init_result or not self._init_result.success:
            self.feedback_message = (
                self._init_result.message if self._init_result else "init failed"
            )
            return Status.FAILURE

        if self._skill.is_finished():
            return Status.SUCCESS if self._skill._success else Status.FAILURE

        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "execute failed"
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        if self._skill is not None:
            try:
                self._skill.cancel()
            except Exception:
                pass
        self._skill = None
