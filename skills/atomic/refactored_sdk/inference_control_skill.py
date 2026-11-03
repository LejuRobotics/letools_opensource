# -*- coding: utf-8 -*-
"""InferenceControl Skill：模型推理控制（开启→监控→关闭）。

通过子进程调用 letools conda 环境的 python 运行推理循环。
推理子进程负责创建 Kuavo 推理环境 → 加载策略 → 循环 select_action/step。
推理子进程订阅夹爪驱动反馈，夹爪运动或到位后，state=3连续保持配置时长才确认抓取成功。
判断时长统一配置于场景 board.json 的 grasp_feedback.hold_seconds。

推理成功结束后（子进程退出码 0），行为树执行导航与放置阶段。
"""

import json
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
_LETOOLS_PYTHON = os.path.expanduser("~/miniforge3/envs/letools/bin/python")
_RUNNER_SCRIPT = os.path.join(_STUDIO_ROOT, "orchestration", "nodes", "inference_runner.py")
_DEFAULT_CONFIG = os.path.join(_STUDIO_ROOT, "LeTools-Learning", "configs", "deploy", "deploy.yaml")


def _load_scenario_runtime_config():
    """Load the scenario board selected by the behavior-tree launcher."""
    if "RING_SIM_CONFIG" not in os.environ:
        return None
    config_name = os.environ["RING_SIM_CONFIG"]
    if not config_name.strip():
        raise ValueError("RING_SIM_CONFIG must name a configuration file")
    config_path = os.path.abspath(os.path.expanduser(config_name))
    with open(config_path, "r", encoding="utf-8") as stream:
        config = json.load(stream)
    if not isinstance(config, dict) or not isinstance(config.get("paths"), dict):
        raise ValueError("RING_SIM_CONFIG must contain a paths object")
    return config_path, config


def _inference_python_for_environment(inference_env, loaded_config=None):
    """Use the scenario configuration for simulation only; preserve old defaults."""
    loaded_config = (
        loaded_config
        if loaded_config is not None
        else _load_scenario_runtime_config()
    )
    if loaded_config is None:
        return _LETOOLS_PYTHON
    if inference_env != "sim":
        raise ValueError("scenario runtime config is for simulation only")
    config_path, config = loaded_config
    value = config["paths"].get("learning_python")
    if not isinstance(value, str) or not value.strip():
        raise ValueError("paths.learning_python must be a nonempty path")
    if os.path.basename(value).startswith("REPLACE_WITH_"):
        raise ValueError(
            "Please replace paths.learning_python in the scenario board with "
            "your inference environment's Python interpreter"
        )
    expanded = os.path.expanduser(value)
    if not os.path.isabs(expanded):
        expanded = os.path.join(os.path.dirname(config_path), expanded)
    python_path = os.path.abspath(expanded)
    if not os.path.isfile(python_path) or not os.access(python_path, os.X_OK):
        raise ValueError("paths.learning_python must be an executable file: " + python_path)
    return python_path


def _resolve_config_path(value, config_path):
    expanded = os.path.expanduser(value)
    if not os.path.isabs(expanded):
        expanded = os.path.join(os.path.dirname(config_path), expanded)
    return os.path.abspath(expanded)


def _is_sdk_root(path):
    return os.path.isfile(os.path.join(path, "kuavo_humanoid_sdk", "__init__.py"))


def _sdk_pythonpath_entries(loaded_config):
    """Return SDK import paths, preferring the configured Kuavo workspace."""
    candidates = []
    if loaded_config is not None:
        config_path, config = loaded_config
        kuavo_root = config.get("paths", {}).get("kuavo_root")
        if (
            isinstance(kuavo_root, str)
            and kuavo_root.strip()
            and not os.path.basename(kuavo_root).startswith("REPLACE_WITH_")
        ):
            kuavo_root = _resolve_config_path(kuavo_root.strip(), config_path)
            candidates.append(os.path.join(kuavo_root, "src", "kuavo_humanoid_sdk"))

    for entry in os.environ.get("PYTHONPATH", "").split(os.pathsep):
        if entry and _is_sdk_root(entry):
            candidates.append(os.path.abspath(entry))

    candidates.append(os.path.join(
        _STUDIO_ROOT,
        "drivers", "leju", "kuavo_humanoid_sdk",
        "src", "kuavo_humanoid_sdk",
    ))

    entries = []
    for sdk_root in candidates:
        if sdk_root in entries or not _is_sdk_root(sdk_root):
            continue
        entries.append(sdk_root)
        msg_root = os.path.join(sdk_root, "kuavo_humanoid_sdk", "msg")
        if os.path.isdir(msg_root):
            entries.append(msg_root)
        kuavo_root = os.path.dirname(os.path.dirname(sdk_root))
        common_python = os.path.join(kuavo_root, "src", "kuavo_common", "python")
        if os.path.isdir(common_python):
            entries.append(common_python)
    return entries


def _build_inference_env(loaded_config, extra_pythonpath):
    env = os.environ.copy()
    entries = _sdk_pythonpath_entries(loaded_config)
    entries.extend(extra_pythonpath)
    current_entries = [
        entry for entry in env.get("PYTHONPATH", "").split(os.pathsep) if entry
    ]
    merged = []
    for entry in entries + current_entries:
        if entry and entry not in merged:
            merged.append(entry)
    env["PYTHONPATH"] = os.pathsep.join(merged)
    return env


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
        loaded_config = _load_scenario_runtime_config()
        config_path = p.inference_config_path
        if not os.path.isabs(config_path):
            config_path = os.path.join(_STUDIO_ROOT, config_path)

        cmd = [
            _inference_python_for_environment(p.inference_env, loaded_config),
            "-u", _RUNNER_SCRIPT,
            "--config", config_path,
            "--pretrained-path", p.pretrained_path,
            "--policy-type", p.policy_type,
            "--task-prompt", p.task_prompt,
            "--max-episode-steps", str(p.max_episode_steps),
            "--device", p.device,
            "--inference-env", p.inference_env,
            "--gripper-pre-position", str(p.gripper_pre_position),
        ]
        if loaded_config is not None:
            cmd.extend(["--grasp-feedback-config", loaded_config[0]])

        letools_root = os.path.join(_STUDIO_ROOT, "LeTools-Learning")
        env = _build_inference_env(loaded_config, [_STUDIO_ROOT, letools_root])

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
