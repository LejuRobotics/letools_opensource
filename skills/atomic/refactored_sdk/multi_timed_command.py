# -*- coding: utf-8 -*-
"""Atomic skill for one configurable multi-planner TimedCmd request."""

import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

PLANNER_VECTOR_SIZES = {
    0: 3,
    1: 3,
    2: 4,
    3: 4,
    4: 6,
    5: 6,
    6: 6,
    7: 6,
    8: 7,
    9: 7,
}


@dataclass
class MultiTimedCommandParams(SkillParams):
    """Commands submitted together to the TimedCmd multi-command service."""

    skill_name: str = "multi_timed_command"
    commands: List[Dict[str, Any]] = field(default_factory=list)
    desire_time: float = 3.0
    is_sync: bool = True
    focus_ee: bool = False
    settle_time: float = 0.5
    timeout: float = 60.0
    prepare_arm_control: bool = True


@define_manifest(
    label="多规划器组合控制（TimedCmd）",
    category=["motion", "timed_cmd"],
    tree_type="studio_smoke",
    description="按配置将任意一组 planner 通过一次 send_timed_multi_commands 下发",
    params=[
        {
            "name": "commands",
            "type": "json",
            "default": "[]",
            "description": "指令列表，每项包含 planner_index 和 cmd_vec，可选 desire_time",
        },
        {"name": "desire_time", "type": "float", "default": "3.0", "description": "未单独指定时的执行时间（秒）"},
        {"name": "is_sync", "type": "bool", "default": "true", "description": "是否让各 planner 同时完成"},
        {"name": "focus_ee", "type": "bool", "default": "false",
         "description": "true=专注手臂末端解算，躯干可能随动；false=尽可能不动躯干，在手臂工作区间内解算，精度可能降低。focus_z 内部跟随该值"},
        {"name": "prepare_arm_control", "type": "bool", "default": "true",
         "description": "包含躯干或手臂 planner 2–9 时，发令前切换到手臂外部控制模式 2；不复位、不自动切回 0。关闭则需外部保证模式"},
        {"name": "settle_time", "type": "float", "default": "0.5", "description": "运动后的额外等待（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class MultiTimedCommandSkill(SkillBase):
    """Validate and submit one configurable multi-planner command group."""

    def __init__(self, hardware: Optional[IHardware]):
        super().__init__(name="multi_timed_command")
        self.hardware = hardware
        self.params: Optional[MultiTimedCommandParams] = None
        self.commands: List[Dict[str, Any]] = []
        self._done = False

    def on_initialize(self, params: MultiTimedCommandParams) -> Result:
        if not isinstance(params, MultiTimedCommandParams):
            return Result.fail("Invalid parameters for MultiTimedCommandSkill")
        try:
            duration = self._positive_number(params.desire_time, "desire_time")
            if not isinstance(params.focus_ee, bool):
                raise TypeError("focus_ee must be a boolean")
            if not isinstance(params.prepare_arm_control, bool):
                raise TypeError("prepare_arm_control must be a boolean")
            settle_time = self._non_negative_number(params.settle_time, "settle_time")
            commands = self._normalize_commands(params.commands, duration)
        except (TypeError, ValueError) as exc:
            return Result.fail(str(exc))

        params.desire_time = duration
        params.settle_time = settle_time
        self.params = params
        self.commands = commands
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("MultiTimedCommandSkill already finished")

        # 躯干/手臂 TimedCmd 需要外部控制模式；只切模式，不做物理复位。
        if self.params.prepare_arm_control and any(
            command["planner_index"] >= 2 for command in self.commands
        ):
            fn = getattr(self.hardware, "set_arm_control_mode", None)
            if fn is None:
                self._done = True
                return Result.fail("Hardware does not implement set_arm_control_mode()")
            result = fn(2)
            if not result.success:
                self._done = True
                return Result.fail(f"set_arm_control_mode(2) failed: {result.message}")

        for name in ("set_focus_ee", "set_focus_z"):
            fn = getattr(self.hardware, name, None)
            if fn is None:
                self._done = True
                return Result.fail(f"Hardware does not implement {name}()")
            result = fn(self.params.focus_ee)
            if not result.success:
                self._done = True
                return Result.fail(
                    f"{name}({self.params.focus_ee}) failed: {result.message}"
                )

        fn = getattr(self.hardware, "send_timed_multi_commands", None)
        if fn is None:
            self._done = True
            return Result.fail("Hardware does not implement send_timed_multi_commands()")

        result = fn(commands=self.commands, is_sync=bool(self.params.is_sync))
        if not result.success:
            self._done = True
            return result

        fallback_time = max(command["desire_time"] for command in self.commands)
        result_data = result.data if isinstance(result.data, dict) else {}
        actual_time = float(result_data.get("actual_time", fallback_time))
        time.sleep(actual_time + self.params.settle_time)
        self._done = True
        logger.info(
            "multi_timed_command: planners=%s sync=%s focus_ee=%s focus_z=%s actual_time=%.3fs",
            [command["planner_index"] for command in self.commands],
            self.params.is_sync,
            self.params.focus_ee,
            self.params.focus_ee,
            actual_time,
        )
        return result

    def on_is_finished(self) -> bool:
        return self._done

    @classmethod
    def _normalize_commands(cls, raw_commands, default_duration):
        if not isinstance(raw_commands, Sequence) or isinstance(raw_commands, (str, bytes)):
            raise TypeError("commands must be a list")
        if not raw_commands:
            raise ValueError("commands must not be empty")

        normalized = []
        seen_planners = set()
        for position, raw_command in enumerate(raw_commands):
            if not isinstance(raw_command, Mapping):
                raise TypeError(f"commands[{position}] must be an object")

            planner_index = raw_command.get("planner_index")
            if isinstance(planner_index, bool) or not isinstance(planner_index, int):
                raise TypeError(f"commands[{position}].planner_index must be an integer")
            if planner_index not in PLANNER_VECTOR_SIZES:
                raise ValueError(f"commands[{position}].planner_index must be in [0, 9]")
            if planner_index in seen_planners:
                raise ValueError(f"planner_index {planner_index} appears more than once")

            raw_vector = raw_command.get("cmd_vec")
            if not isinstance(raw_vector, Sequence) or isinstance(raw_vector, (str, bytes)):
                raise TypeError(f"commands[{position}].cmd_vec must be a list")
            vector = [cls._finite_number(value, f"commands[{position}].cmd_vec") for value in raw_vector]
            expected_size = PLANNER_VECTOR_SIZES[planner_index]
            if len(vector) != expected_size:
                raise ValueError(
                    f"planner_index {planner_index} expects {expected_size} values, got {len(vector)}"
                )

            duration = cls._positive_number(
                raw_command.get("desire_time", default_duration),
                f"commands[{position}].desire_time",
            )
            normalized.append(
                {
                    "planner_index": planner_index,
                    "desire_time": duration,
                    "cmd_vec": vector,
                }
            )
            seen_planners.add(planner_index)
        return normalized

    @staticmethod
    def _finite_number(value, name):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"{name} values must be finite numbers")
        return number

    @classmethod
    def _positive_number(cls, value, name):
        number = cls._finite_number(value, name)
        if number <= 0:
            raise ValueError(f"{name} must be greater than 0")
        return number

    @classmethod
    def _non_negative_number(cls, value, name):
        number = cls._finite_number(value, name)
        if number < 0:
            raise ValueError(f"{name} must be greater than or equal to 0")
        return number
