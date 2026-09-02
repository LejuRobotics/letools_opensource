# -*- coding: utf-8 -*-
"""InferenceControl Skill：模型推理控制（开启→监控→关闭）。

通过子进程调用 letools conda 环境的 python 运行推理循环。
推理子进程负责创建 Kuavo 推理环境 → 加载策略 → 循环 select_action/step。
检测夹爪闭合，闭合后保持若干步再退出。

推理完成后（子进程退出码 0），行为树继续走原来的导航与放置链路。
"""

import os
import subprocess
import time
from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

_STUDIO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
_LETOOLS_PYTHON = "/home/zwl/miniforge3/envs/letools/bin/python"
_RUNNER_SCRIPT = os.path.join(_STUDIO_ROOT, "orchestration", "nodes", "inference_runner.py")
_DEFAULT_CONFIG = os.path.join(_STUDIO_ROOT, "LeTools-Learning", "configs", "deploy", "deploy.yaml")


@dataclass
class InferenceControlParams(SkillParams):
    """推理控制参数。"""
    skill_name: str = "inference_control"
    inference_config_path: str = _DEFAULT_CONFIG
    policy_type: str = "act"
    pretrained_path: str = ""
    task_prompt: str = "Pick and Place"
    max_episode_steps: int = 200
    device: str = "cuda"
    inference_env: str = "sim"
    gripper_pre_position: float = 50.0
    gripper_close_threshold: float = 0.3
    gripper_hold_frames: int = 5
    post_close_steps: int = 10
    timeout: float = 300.0


class InferenceControlSkill(SkillBase):
    """推理控制 Skill：子进程启动推理 → 监控状态 → 关闭推理。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="inference_control")
        self.hardware = hardware
        self.params: Optional[InferenceControlParams] = None
        self._proc = None
        self._done = False
        self._failed = False
        self._start_time = 0.0
        self._timeout_sec = 300.0

    def on_initialize(self, params: InferenceControlParams) -> Result:
        if not isinstance(params, InferenceControlParams):
            return Result.fail("Invalid parameters for InferenceControlSkill")
        self.params = params
        self._done = False
        self._failed = False
        self._start_time = time.time()
        self._timeout_sec = params.timeout

        try:
            self._start_inference()
            print("[InferenceControlSkill] 推理已开启")
            return Result.ok()
        except Exception as e:
            self._failed = True
            return Result.fail(f"推理启动失败: {e}")

    def _start_inference(self):
        p = self.params
        config_path = p.inference_config_path
        if not os.path.isabs(config_path):
            config_path = os.path.join(_STUDIO_ROOT, config_path)

        cmd = [
            _LETOOLS_PYTHON, "-u", _RUNNER_SCRIPT,
            "--config", config_path,
            "--pretrained-path", p.pretrained_path,
            "--policy-type", p.policy_type,
            "--task-prompt", p.task_prompt,
            "--max-episode-steps", str(p.max_episode_steps),
            "--device", p.device,
            "--inference-env", p.inference_env,
            "--gripper-pre-position", str(p.gripper_pre_position),
            "--gripper-close-threshold", str(p.gripper_close_threshold),
            "--gripper-hold-frames", str(p.gripper_hold_frames),
            "--post-close-steps", str(p.post_close_steps),
        ]

        letools_root = os.path.join(_STUDIO_ROOT, "LeTools-Learning")
        env = os.environ.copy()
        env["PYTHONPATH"] = _STUDIO_ROOT + os.pathsep + letools_root + os.pathsep + env.get("PYTHONPATH", "")

        print(f"[InferenceControlSkill] 启动推理子进程: {' '.join(cmd[:4])} ...")
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=env, text=True, bufsize=1,
        )

    def on_execute(self) -> Result:
        if self._failed:
            return Result.fail("推理启动失败")
        if self._done:
            return Result.ok("done")

        if self._proc is None:
            return Result.fail("推理子进程未启动")

        # 超时
        elapsed = time.time() - self._start_time
        if elapsed > self._timeout_sec:
            print(f"[InferenceControlSkill] 推理超时 ({elapsed:.0f}s)")
            self._stop_inference()
            return Result.fail("推理超时")

        # 读子进程输出
        line = self._proc.stdout.readline()
        if line:
            line = line.strip()
            if line.startswith("[INFER]"):
                msg = line[7:].strip()
                if msg == "done":
                    print("[InferenceControlSkill] 推理完成")
                    self._done = True
                    self._wait_process()
                    return Result.ok("done")
                elif msg == "timeout":
                    self._wait_process()
                    return Result.fail("推理超时")
                elif msg.startswith("error"):
                    self._wait_process()
                    return Result.fail(msg)
                elif msg == "gripper_closed":
                    print("[InferenceControlSkill] 检测到夹爪闭合")
                elif msg.startswith("step"):
                    pass
                else:
                    print(f"[InferenceControlSkill] {msg}")

        # 子进程已退出
        if self._proc.poll() is not None:
            remaining = self._proc.stdout.read()
            if remaining:
                for ln in remaining.strip().split("\n"):
                    print(f"[InferenceControlSkill] {ln.strip()}")
            retcode = self._proc.returncode
            if retcode == 0 and not self._done:
                self._done = True
                return Result.ok("done")
            elif not self._done:
                return Result.fail(f"推理子进程异常退出 (code={retcode})")

        return Result.ok("running")

    def on_is_finished(self) -> bool:
        return self._done or self._failed

    def on_cancel(self) -> Result:
        self._stop_inference()
        return Result.ok("cancelled")

    def _stop_inference(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
        self._proc = None
        print("[InferenceControlSkill] 推理已关闭")

    def _wait_process(self):
        if self._proc and self._proc.poll() is None:
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
        self._proc = None
        print("[InferenceControlSkill] 推理已关闭")
