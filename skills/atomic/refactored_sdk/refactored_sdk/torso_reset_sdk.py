# -*- coding: utf-8 -*-
"""Atomic skill: torso_reset_sdk.

Aligns with `torso_control_mixin.reset_torso_to_initial()` by calling `hardware.reset_torso_to_initial()`.
底层路径: Adapter 直调 → ROS Service /mobile_manipulator_reset_torso (SetBool)
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
class TorsoResetSdkParams(SkillParams):
    """对齐 torso_control_mixin.reset_torso_to_initial()。"""

    skill_name: str = "torso_reset_sdk"
    timeout: float = 30.0


@define_manifest(
    label="躯干复位（ROS Service）",
    category=["motion", "torso"],
    tree_type="studio_smoke",
    description="调用 hardware.reset_torso_to_initial() → ROS Service /mobile_manipulator_reset_torso",
    params=[],
    inputs=[],
    outputs=[],
)
class TorsoResetSdkSkill(SkillBase):
    """躯干复位（ROS Service 直调）：hardware.reset_torso_to_initial()。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="torso_reset_sdk")
        self.hardware = hardware
        self.params: Optional[TorsoResetSdkParams] = None
        self._done = False

    def on_initialize(self, params: TorsoResetSdkParams) -> Result:
        if not isinstance(params, TorsoResetSdkParams):
            return Result.fail("Invalid parameters for TorsoResetSdkSkill")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("TorsoResetSdkSkill already finished")

        fn = getattr(self.hardware, "reset_torso_to_initial", None)
        if fn is None:
            self._done = True
            return Result.fail("Hardware does not implement reset_torso_to_initial()")

        result = fn()
        self._done = True
        if result.success:
            logger.info("torso_reset_sdk: 躯干复位成功 — %s", result.message)
        return result

    def on_is_finished(self) -> bool:
        return self._done
