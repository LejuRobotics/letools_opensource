# -*- coding: utf-8 -*-
"""InferenceControl：推理控制薄节点 → inference_control Skill。

薄节点职责：
  - 参数解析（board JSON dict → InferenceControlParams dataclass）
  - py_trees 生命周期（initialise / update / terminate）
  - 委托 InferenceControlSkill 执行全部业务逻辑（子进程推理开启/监控/关闭）
"""

import os
import sys

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
        self._real_stage = None
        self._backend_error = None

    def initialise(self):
        self._real_stage = None
        self._backend_error = None
        backend = str(self.params.get("inference_backend", "act")).strip().lower()
        if backend not in ("act", "pi05", "skip"):
            self._backend_error = "Unsupported inference_backend: " + backend
            self.feedback_message = self._backend_error
            return
        if _DRY_RUN:
            self.feedback_message = "dry-run inference_control"
            self._skill = None
            self._init_result = None
            return

        mode = str(self.params.get("execution_mode", "sim")).strip().lower()
        if mode not in ("sim", "real"):
            self._backend_error = "Unsupported inference execution_mode: " + mode
            self.feedback_message = self._backend_error
            return
        real_act = backend == "act" and mode == "real"
        if mode == "sim" and backend != "act":
            self._backend_error = "PI05 and skip require execution_mode=real"
            self.feedback_message = self._backend_error
            return
        if backend in ("pi05", "skip") or real_act:
            try:
                from orchestration.nodes.real_ring_runtime import RealStage
                if real_act:
                    self._real_stage = RealStage("grasp", backend="act")
                else:
                    self._real_stage = RealStage("grasp", model=(backend == "pi05"))
                self._real_stage.start()
            except Exception as exc:
                self._backend_error = str(exc)
                self.feedback_message = self._backend_error
            return

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
            timeout=float(max_steps),
        )
        self._skill = InferenceControlSkill(hardware=hw)
        self._init_result = self._skill.initialize(skill_params)
        if not self._init_result.success:
            self.feedback_message = self._init_result.message or "init failed"

    def update(self):
        if self._backend_error:
            return Status.FAILURE
        if _DRY_RUN:
            return Status.SUCCESS

        if self._real_stage is not None:
            state = self._real_stage.update()
            if state == "failure":
                self.feedback_message = self._real_stage.error or "real grasp failed"
            return {"running": Status.RUNNING, "success": Status.SUCCESS,
                    "failure": Status.FAILURE}[state]

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
        if self._real_stage is not None:
            self._real_stage.terminate(new_status == Status.SUCCESS)
            self._real_stage = None
        elif new_status != Status.SUCCESS:
            runtime = sys.modules.get("orchestration.nodes.real_ring_runtime")
            if runtime is not None:
                runtime.reset_session()
        if self._skill is not None:
            try:
                self._skill.cancel()
            except Exception:
                pass
        self._skill = None
