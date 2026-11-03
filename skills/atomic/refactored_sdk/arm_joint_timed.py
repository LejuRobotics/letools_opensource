# -*- coding: utf-8 -*-
"""Atomic skill for one 14D arm joint target through TimedCmd."""

import time
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
class ArmJointTimedParams(SkillParams):
    """Parameters for a single synchronized 14D arm joint command."""

    skill_name: str = "arm_joint_timed"
    joint_angles: List[float] = field(default_factory=lambda: [0.0] * 14)
    desire_time: float = 3.0
    settle_time: float = 0.0
    timeout: float = 60.0


@define_manifest(
    label="双臂关节控制（TimedCmd）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="调用 hardware.send_arm_joint_timed(joint_angles, desire_time)",
    params=[
        {
            "name": "joint_angles",
            "type": "floatArr",
            "default": "0,0,0,0,0,0,0,0,0,0,0,0,0,0",
            "description": "14 个关节角 [左臂 7, 右臂 7]（deg）",
        },
        {"name": "desire_time", "type": "float", "default": "3.0", "description": "期望执行时间（秒）"},
        {"name": "settle_time", "type": "float", "default": "0.0", "description": "运动完成后的额外等待（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class ArmJointTimedSkill(SkillBase):
    """Send one arm target and remain active until the planned motion finishes."""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_joint_timed")
        self.hardware = hardware
        self.params: Optional[ArmJointTimedParams] = None
        self._done = False

    def on_initialize(self, params: ArmJointTimedParams) -> Result:
        if not isinstance(params, ArmJointTimedParams):
            return Result.fail("Invalid parameters for ArmJointTimedSkill")
        if len(params.joint_angles) != 14:
            return Result.fail(
                f"arm_joint_timed expects 14 joint angles, got {len(params.joint_angles)}"
            )
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmJointTimedSkill already finished")

        fn = getattr(self.hardware, "send_arm_joint_timed", None)
        if fn is None:
            self._done = True
            return Result.fail("Hardware does not implement send_arm_joint_timed()")

        params = self.params
        result = fn(
            joint_angles=list(params.joint_angles),
            desire_time=float(params.desire_time),
        )
        if not result.success:
            self._done = True
            return result

        actual_time = float((result.data or {}).get("actual_time", params.desire_time))
        time.sleep(actual_time + float(params.settle_time))
        self._done = True
        logger.info(
            "arm_joint_timed: angles=%s desire_time=%.3fs actual_time=%.3fs",
            list(params.joint_angles),
            float(params.desire_time),
            actual_time,
        )
        return result

    def on_is_finished(self) -> bool:
        return self._done
