# -*- coding: utf-8 -*-
"""Atomic skill: vacuum_control — 气泵吸/放控制。

直接封装 `adapters.vacuum_control` 的 Modbus 串口控制函数：
- `control_vacuum_pump(relay_index, action)` — 气泵开关
- `control_relay(relay_index, action)` — 破真空继电器

对齐 `adapters/vacuum_control/test_vacuum.py`。
"""

import time
from dataclasses import dataclass
from typing import Optional

from adapters.vacuum_control import control_relay, control_vacuum_pump

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest

logger = get_logger(__name__)


@dataclass
class VacuumControlParams(SkillParams):
    """气泵控制参数。

    action:
      - "suck"    → 吸气 (开气泵)
      - "release" → 松开 (关气泵 + 破真空)
    """

    skill_name: str = "vacuum_control"
    action: str = "suck"  # "suck" | "release"
    timeout: float = 10.0


@define_manifest(
    label="气泵控制 (吸/放)",
    category=["vacuum", "end_effector"],
    tree_type="studio_smoke",
    description="控制气泵吸气或松开 (含破真空)。对齐 adapters/vacuum_control/test_vacuum.py",
    params=[
        {"name": "action", "type": "string", "default": "suck",
         "description": "操作类型: 'suck'=吸气, 'release'=松开(含破真空)"},
    ],
    inputs=[],
    outputs=[],
)
class VacuumControlSkill(SkillBase):
    """气泵吸/放控制 —— 一次性操作，首次 execute 后即完成。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="vacuum_control")
        self.hardware = hardware  # 保留以匹配 Skill 构造签名；真空控制使用自己的 Modbus 串口
        self.params: Optional[VacuumControlParams] = None
        self._done = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: VacuumControlParams) -> Result:
        if not isinstance(params, VacuumControlParams):
            return Result.fail("Invalid parameters for VacuumControlSkill")
        if params.action not in ("suck", "release"):
            return Result.fail(
                f"Unknown action: '{params.action}', expected 'suck' or 'release'"
            )
        self.params = params
        self._done = False
        self._result = None
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return self._result or Result.ok("VacuumControlSkill already finished")

        action = self.params.action

        try:
            if action == "suck":
                success = control_vacuum_pump(relay_index=0, action="ON", duration=0.0)
                msg = "气泵已开启，正在吸气" if success else "气泵开启失败"
                logger.info("[vacuum] 吸气: %s", "成功" if success else "失败")

            elif action == "release":
                # 1. 关闭气泵
                result1 = control_vacuum_pump(relay_index=0, action="OFF")
                # 2. 触发继电器1 破真空 (脉冲式)
                result2 = control_relay(relay_index=1, action="ON")
                time.sleep(0.2)
                result3 = control_relay(relay_index=1, action="OFF")
                success = result1  # 以气泵关闭结果为准
                msg = "气泵已关闭，破真空完成" if success else "气泵关闭失败"
                logger.info(
                    "[vacuum] 松开: pump_off=%s relay_on=%s relay_off=%s",
                    result1, result2, result3,
                )

            else:
                self._done = True
                return Result.fail(f"Unknown action: {action}")

            self._done = True
            self._result = Result.ok(msg) if success else Result.fail(msg)
            return self._result

        except Exception as e:
            self._done = True
            logger.error("[vacuum] ❌ 异常: %s", e, exc_info=True)
            return Result.fail(f"vacuum_control error: {e}")

    def on_is_finished(self) -> bool:
        return self._done
