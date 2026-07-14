# -*- coding: utf-8 -*-
"""Atomic skill: arm_joint_traj_sdk.

Aligns with `test_arm_joint_traj.py` by calling `hardware.send_arm_joint_traj_sdk(joint_traj, total_time)`.
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
class ArmJointTrajSdkParams(SkillParams):
    """对齐 test_arm_joint_traj.py：hardware.send_arm_joint_traj_sdk(joint_traj, total_time)。"""

    skill_name: str = "arm_joint_traj_sdk"
    joint_traj: List[List[float]] = field(
        default_factory=lambda: [
            [0.0] * 14,
            [
                -30.0,
                20.0,
                15.0,
                -45.0,
                25.0,
                10.0,
                -35.0,
                -30.0,
                -20.0,
                -15.0,
                -45.0,
                -25.0,
                -10.0,
                -35.0,
            ],
        ]
    )
    total_time: float = 3.0
    timeout: float = 60.0


@define_manifest(
    label="手臂关节轨迹（SDK）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="对齐 test_arm_joint_traj.py：调用 hardware.send_arm_joint_traj_sdk(joint_traj, total_time)",
    params=[
        {
            "name": "joint_traj",
            "type": "json",
            "default": "",
            "description": "轨迹点列表（每点 14 关节角，用户单位）；JSON 数组",
        },
        {"name": "total_time", "type": "float", "default": "3.0", "description": "秒"},
    ],
    inputs=[],
    outputs=[],
)
class ArmJointTrajSdkSkill(SkillBase):
    """手臂关节轨迹（SDK 直调）：hardware.send_arm_joint_traj_sdk(joint_traj, total_time)。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_joint_traj_sdk")
        self.hardware = hardware
        self.params: Optional[ArmJointTrajSdkParams] = None
        self._done = False

    def on_initialize(self, params: ArmJointTrajSdkParams) -> Result:
        if not isinstance(params, ArmJointTrajSdkParams):
            return Result.fail("Invalid parameters for ArmJointTrajSdkSkill")
        if not params.joint_traj or not all(isinstance(p, list) for p in params.joint_traj):
            return Result.fail("arm_joint_traj_sdk expects joint_traj as List[List[float]]")
        for idx, point in enumerate(params.joint_traj):
            if len(point) != 14:
                return Result.fail(
                    "arm_joint_traj_sdk expects 14 joint angles per point, "
                    f"got {len(point)} at index {idx}"
                )
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmJointTrajSdkSkill already finished")

        fn = getattr(self.hardware, "send_arm_joint_traj_sdk", None)
        if fn is None:
            self._done = True
            return Result.fail("Hardware does not implement send_arm_joint_traj_sdk()")

        result = fn(
            joint_traj=self.params.joint_traj,
            total_time=float(self.params.total_time),
        )
        self._done = True
        if result.success:
            logger.info(
                "arm_joint_traj_sdk: points=%d total_time=%.3fs",
                len(self.params.joint_traj),
                float(self.params.total_time),
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done

