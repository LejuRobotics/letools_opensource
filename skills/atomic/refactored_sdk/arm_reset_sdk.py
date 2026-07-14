# -*- coding: utf-8 -*-
"""Atomic skill: arm_reset_sdk.

Aligns with `test_arm_reset.py` by calling `hardware.arm_reset()`.
"""

from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class ArmResetSdkParams(SkillParams):
    """对齐 test_arm_reset.py：hardware.arm_reset()。"""

    skill_name: str = "arm_reset_sdk"
    timeout: float = 30.0


@define_manifest(
    label="手臂归位（SDK）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="对齐 test_arm_reset.py：调用 hardware.arm_reset()，自动管理 MPC 模式",
    params=[],
    inputs=[],
    outputs=[],
)
class ArmResetSdkSkill(SkillBase):
    """手臂归位（SDK 直调）：hardware.arm_reset()，内部自动处理 MPC 模式设置和恢复。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_reset_sdk")
        self.hardware = hardware
        self.params: Optional[ArmResetSdkParams] = None
        self._done = False

    def on_initialize(self, params: ArmResetSdkParams) -> Result:
        if not isinstance(params, ArmResetSdkParams):
            return Result.fail("Invalid parameters for ArmResetSdkSkill")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmResetSdkSkill already finished")

        fn = getattr(self.hardware, "arm_reset", None)
        if fn is None:
            self._done = True
            return Result.fail("Hardware does not implement arm_reset()")

        result = fn()
        self._done = True
        if result.success:
            logger.info("arm_reset_sdk: 手臂归位成功")
        return result

    def on_is_finished(self) -> bool:
        return self._done
