# -*- coding: utf-8 -*-
"""InferenceControl：推理控制薄节点 → inference_control Skill。

薄节点职责：
  - 参数解析（board JSON dict → InferenceControlParams dataclass）
  - py_trees 生命周期（initialise / update / terminate）
  - 委托 InferenceControlSkill 执行全部业务逻辑（子进程推理开启/监控/关闭）
"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from skills.atomic.refactored_sdk.inference_control_skill import (
    InferenceControlParams,
    InferenceControlSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class InferenceControl(BaseAction):
    """薄节点：参数解析 + py_trees 生命周期 → InferenceControlSkill。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._init_result = None

    def initialise(self):
        try:
            hw = get_shared_hardware()
        except Exception:
            hw = None

        max_steps = int(self.params.get("max_episode_steps", 200))
        skill_params = InferenceControlParams(
            inference_config_path=str(self.params.get("inference_config_path", "")),
            policy_type=str(self.params.get("policy_type", "act")),
            pretrained_path=str(self.params.get("pretrained_path", "")),
            task_prompt=str(self.params.get("task_prompt", "Pick and Place")),
            max_episode_steps=max_steps,
            device=str(self.params.get("device", "cuda")),
            inference_env=str(self.params.get("inference_env", "sim")),
            gripper_pre_position=float(self.params.get("gripper_pre_position", 50)),
            gripper_close_threshold=float(self.params.get("gripper_close_threshold", 0.3)),
            gripper_hold_frames=int(self.params.get("gripper_hold_frames", 5)),
            post_close_steps=int(self.params.get("post_close_steps", 10)),
            timeout=float(max_steps),
        )
        self._skill = InferenceControlSkill(hardware=hw)
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
            return Status.SUCCESS if self._skill._done else Status.FAILURE

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
