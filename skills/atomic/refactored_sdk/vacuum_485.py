# -*- coding: utf-8 -*-
"""Atomic skill: vacuum_485 — 气泵继电器控制 (基于 ROS Trigger 服务)。

封装 `adapters.vacuum_485` 的 ROS 服务调用函数：
- `blow()`  — 吹气 (通道1继电器开)
- `suck()`  — 吸气 (通道2继电器开)
- `power_off()` — 断电全关


"""

from dataclasses import dataclass
from typing import Optional

from adapters.vacuum_485 import blow, suck, power_off

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


# 支持的动作列表
_VALID_ACTIONS = ("blow", "suck", "power_off")


@dataclass
class Vacuum485Params(SkillParams):
    """气泵继电器控制参数。

    action:
      - "blow"      → 吹气 (通道1开)
      - "suck"      → 吸气 (通道2开)
      - "power_off" → 断电全关
    """

    skill_name: str = "vacuum_485"
    action: str = "suck"  # "blow" | "suck" | "power_off"
    timeout: float = 10.0


@define_manifest(
    label="气泵485控制 (吹/吸/断电)",
    category=["vacuum", "end_effector", "ros"],
    tree_type="studio_smoke",
    description="通过 ROS Trigger 服务控制气泵继电器: 吹气/吸气/断电",
    params=[
        {"name": "action", "type": "string", "default": "suck",
         "description": "操作: 'blow'=吹气, 'suck'=吸气, 'power_off'=断电"},
    ],
    inputs=[],
    outputs=[],
)
class Vacuum485Skill(SkillBase):
    """气泵继电器控制 —— 一次性操作，首次 execute 后即完成。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="vacuum_485")
        self.hardware = hardware
        self.params: Optional[Vacuum485Params] = None
        self._done = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: Vacuum485Params) -> Result:
        if not isinstance(params, Vacuum485Params):
            return Result.fail("Invalid parameters for Vacuum485Skill")
        if params.action not in _VALID_ACTIONS:
            return Result.fail(
                f"Unknown action: '{params.action}', expected one of {_VALID_ACTIONS}"
            )
        self.params = params
        self._done = False
        self._result = None
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return self._result or Result.ok("Vacuum485Skill already finished")

        action = self.params.action

        try:
            if action == "blow":
                success, msg = blow()
                logger.info("[vacuum_485] 吹气: %s", "成功" if success else "失败")

            elif action == "suck":
                success, msg = suck()
                logger.info("[vacuum_485] 吸气: %s", "成功" if success else "失败")

            elif action == "power_off":
                success, msg = power_off()
                logger.info("[vacuum_485] 断电: %s", "成功" if success else "失败")

            else:
                self._done = True
                return Result.fail(f"Unknown action: {action}")

            self._done = True
            self._result = Result.ok(msg) if success else Result.fail(msg)
            return self._result

        except Exception as e:
            self._done = True
            logger.error("[vacuum_485] ❌ 异常: %s", e, exc_info=True)
            return Result.fail(f"vacuum_485 error: {e}")

    def on_is_finished(self) -> bool:
        return self._done
