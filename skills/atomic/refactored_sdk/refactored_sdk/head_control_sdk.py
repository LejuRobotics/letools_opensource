# -*- coding: utf-8 -*-
"""Atomic skill: head_control_sdk.

Aligns with `test_head_control.py` by calling `hardware.control_head_sdk(yaw, pitch)`.
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
class HeadControlSdkParams(SkillParams):
    """对齐 test_head_control.py：调用 hardware.control_head_sdk(yaw, pitch)。"""

    skill_name: str = "head_control_sdk"
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    timeout: float = 30.0


@define_manifest(
    label="头部控制（SDK）",
    category=["motion", "head"],
    tree_type="studio_smoke",
    description="对齐 test_head_control.py：调用 hardware.control_head_sdk(yaw, pitch)",
    params=[
        {"name": "yaw_deg", "type": "float", "default": "0.0", "description": "偏航角（度）"},
        {"name": "pitch_deg", "type": "float", "default": "0.0", "description": "俯仰角（度）"},
    ],
    inputs=[],
    outputs=[],
)
class HeadControlSdkSkill(SkillBase):
    """头部控制（SDK 直调）：hardware.control_head_sdk(yaw, pitch)。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="head_control_sdk")
        self.hardware = hardware
        self.params: Optional[HeadControlSdkParams] = None
        self._done = False

    def on_initialize(self, params: HeadControlSdkParams) -> Result:
        if not isinstance(params, HeadControlSdkParams):
            return Result.fail("Invalid parameters for HeadControlSdkSkill")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("HeadControlSdkSkill already finished")

        fn = getattr(self.hardware, "control_head_sdk", None)
        if fn is None:
            self._done = True
            return Result.fail("Hardware does not implement control_head_sdk()")

        result = fn(yaw=float(self.params.yaw_deg), pitch=float(self.params.pitch_deg))
        self._done = True
        if result.success:
            logger.info(
                "head_control_sdk: yaw=%.3f pitch=%.3f deg",
                float(self.params.yaw_deg),
                float(self.params.pitch_deg),
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done

