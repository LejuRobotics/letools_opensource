# -*- coding: utf-8 -*-
"""Atomic skill: arm_ee_traj_local_sdk.

Aligns with `test_arm_ee_traj_local.py` by calling `hardware.send_arm_ee_traj_sdk(left_traj, right_traj, total_time, frame='base_link')`.
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

# 默认轨迹点: [x, y, z, qx, qy, qz, qw]
_DEFAULT_LEFT = [[0.3, 0.25, 0.5, 0.0, 0.0, 0.0, 1.0], [0.5, 0.25, 0.5, 0.0, 0.0, 0.0, 1.0]]
_DEFAULT_RIGHT = [[0.3, -0.25, 0.5, 0.0, 0.0, 0.0, 1.0], [0.5, -0.25, 0.5, 0.0, 0.0, 0.0, 1.0]]


@dataclass
class ArmEETrajLocalSdkParams(SkillParams):
    """对齐 test_arm_ee_traj_local.py：hardware.send_arm_ee_traj_sdk(frame='base_link')。"""

    skill_name: str = "arm_ee_traj_local_sdk"
    left_traj: List[List[float]] = field(default_factory=lambda: _DEFAULT_LEFT)
    right_traj: List[List[float]] = field(default_factory=lambda: _DEFAULT_RIGHT)
    total_time: float = 3.0
    timeout: float = 60.0


@define_manifest(
    label="手臂末端轨迹（局部系/SDK）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="对齐 test_arm_ee_traj_local.py：调用 hardware.send_arm_ee_traj_sdk(frame='base_link')",
    params=[
        {
            "name": "left_traj",
            "type": "json",
            "default": "",
            "description": "左手轨迹点列表，每点 [x,y,z,qx,qy,qz,qw]；JSON 数组",
        },
        {
            "name": "right_traj",
            "type": "json",
            "default": "",
            "description": "右手轨迹点列表，每点 [x,y,z,qx,qy,qz,qw]；JSON 数组",
        },
        {"name": "total_time", "type": "float", "default": "3.0", "description": "总执行时间（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEETrajLocalSdkSkill(SkillBase):
    """手臂末端轨迹（局部系，SDK 直调）：hardware.send_arm_ee_traj_sdk(frame='base_link')。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_ee_traj_local_sdk")
        self.hardware = hardware
        self.params: Optional[ArmEETrajLocalSdkParams] = None
        self._done = False

    def _validate_traj(self, traj: List[List[float]], side: str) -> Optional[str]:
        """校验单侧轨迹格式"""
        if not traj or not all(isinstance(p, list) for p in traj):
            return f"{side} 侧轨迹需为 List[List[float]]"
        for idx, point in enumerate(traj):
            if len(point) != 7:
                return (
                    f"{side} 侧轨迹每点需要 7 个值 [x,y,z,qx,qy,qz,qw]，"
                    f"第 {idx} 个点有 {len(point)} 个值"
                )
        return None

    def on_initialize(self, params: ArmEETrajLocalSdkParams) -> Result:
        if not isinstance(params, ArmEETrajLocalSdkParams):
            return Result.fail("Invalid parameters for ArmEETrajLocalSdkSkill")

        err = self._validate_traj(params.left_traj, "左")
        if err:
            return Result.fail(err)
        err = self._validate_traj(params.right_traj, "右")
        if err:
            return Result.fail(err)

        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmEETrajLocalSdkSkill already finished")

        fn = getattr(self.hardware, "send_arm_ee_traj_sdk", None)
        if fn is None:
            self._done = True
            return Result.fail("Hardware does not implement send_arm_ee_traj_sdk()")

        result = fn(
            left_traj=self.params.left_traj,
            right_traj=self.params.right_traj,
            total_time=float(self.params.total_time),
            frame="base_link",
        )
        self._done = True
        if result.success:
            logger.info(
                "arm_ee_traj_local_sdk: left_points=%d right_points=%d total_time=%.3fs",
                len(self.params.left_traj),
                len(self.params.right_traj),
                float(self.params.total_time),
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done
