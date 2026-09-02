# -*- coding: utf-8 -*-
"""PlaceRingMove：放置圆环薄节点 → place_ring Skill。

薄节点职责：
  - 参数解析（board JSON dict → PlaceRingParams dataclass）
  - py_trees 生命周期（initialise / update / terminate）
  - 委托 PlaceRingSkill 执行全部业务逻辑
"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from skills.atomic.refactored_sdk.place_ring import (
    PlaceRingParams,
    PlaceRingSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class PlaceRingMove(BaseAction):
    """薄节点：参数解析 + py_trees 生命周期 → PlaceRingSkill。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._init_result = None

    def initialise(self):
        try:
            hw = get_shared_hardware()
        except Exception:
            hw = None

        skill_params = PlaceRingParams(
            place_target_x=float(self.params.get("place_target_x", 0.7)),
            place_target_y=float(self.params.get("place_target_y", -0.3)),
            place_target_z=float(self.params.get("place_target_z", 1.05)),
            gripper_pre_position=float(self.params.get("gripper_pre_position", 50)),
        )
        self._skill = PlaceRingSkill(hardware=hw)
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
