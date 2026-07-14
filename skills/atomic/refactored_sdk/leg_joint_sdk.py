# -*- coding: utf-8 -*-
"""Atomic skill: leg_joint_sdk.

Aligns with `test_leg_joint.py` by calling `hardware.send_leg_joint_sdk(joint_angles, total_time)`.
"""

from dataclasses import dataclass, field
from typing import List, Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class LegJointSdkParams(SkillParams):
    """对齐 test_leg_joint.py：hardware.send_leg_joint_sdk(joint_angles, total_time)。"""

    skill_name: str = "leg_joint_sdk"
    joint_angles: List[float] = field(default_factory=lambda: [14.90, -32.01, 18.03, -90.0])
    total_time: float = 3.0
    timeout: float = 60.0


@define_manifest(
    label="下肢关节控制（SDK）",
    category=["motion", "leg"],
    tree_type="studio_smoke",
    description="对齐 test_leg_joint.py：调用 hardware.send_leg_joint_sdk(joint_angles, total_time)",
    params=[
        {
            "name": "joint_angles",
            "type": "floatArr",
            "default": "14.90,-32.01,18.03,-90.0",
            "description": "4 个关节角（用户单位，默认 deg）",
        },
        {"name": "total_time", "type": "float", "default": "3.0", "description": "秒"},
    ],
    inputs=[],
    outputs=[],
)
class LegJointSdkSkill(SkillBase):
    """下肢关节控制（SDK 直调）：hardware.send_leg_joint_sdk(joint_angles, total_time)。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="leg_joint_sdk")
        self.hardware = hardware
        self.params: Optional[LegJointSdkParams] = None
        self._done = False

    def on_initialize(self, params: LegJointSdkParams) -> Result:
        if not isinstance(params, LegJointSdkParams):
            return Result.fail("Invalid parameters for LegJointSdkSkill")
        if len(params.joint_angles) != 4:
            return Result.fail(
                f"leg_joint_sdk expects 4 joint angles, got {len(params.joint_angles)}"
            )
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("LegJointSdkSkill already finished")

        fn = getattr(self.hardware, "send_leg_joint_sdk", None)
        if fn is None:
            self._done = True
            return Result.fail("Hardware does not implement send_leg_joint_sdk()")

        result = fn(
            joint_angles=list(self.params.joint_angles),
            total_time=float(self.params.total_time),
        )
        self._done = True
        if result.success:
            logger.info(
                "leg_joint_sdk: joint_angles=%s total_time=%.3fs",
                list(self.params.joint_angles),
                float(self.params.total_time),
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done

