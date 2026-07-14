# -*- coding: utf-8 -*-
"""Atomic skill: base_move_to_target_jibot_sdk.

Aligns with `test_move_to_target.py` by calling `hardware.base_move_to_target_jibot()`.
"""

from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.chassis_options import MoveToTargetOptions
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class BaseMoveToTargetJibotParams(SkillParams):
    """对齐 test_move_to_target.py：hardware.base_move_to_target_jibot()。"""

    skill_name: str = "base_move_to_target_jibot_sdk"
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    avoid_enabled: bool = False
    avoid_distance: float = 0.5
    linear_velocity: float = 0.15
    angular_velocity: float = 0.25
    position_threshold: float = 0.08
    angle_threshold: float = 0.1
    allow_rotation: bool = True
    timeout: float = 60.0


@define_manifest(
    label="JiBot底盘绝对目标点移动",
    category=["motion", "chassis", "jibot"],
    tree_type="studio_smoke",
    description="对齐 test_move_to_target.py：调用 hardware.base_move_to_target_jibot()",
    params=[
        {"name": "x", "type": "float", "default": "0.0", "description": "map坐标系下目标x(m)"},
        {"name": "y", "type": "float", "default": "0.0", "description": "map坐标系下目标y(m)"},
        {"name": "theta", "type": "float", "default": "0.0", "description": "map坐标系下目标yaw(rad)"},
        {"name": "avoid_enabled", "type": "bool", "default": "False", "description": "是否启用避障"},
        {"name": "avoid_distance", "type": "float", "default": "0.5", "description": "避障距离(m)"},
        {"name": "linear_velocity", "type": "float", "default": "0.15", "description": "线速度(m/s)"},
        {"name": "angular_velocity", "type": "float", "default": "0.25", "description": "角速度(rad/s)"},
        {"name": "position_threshold", "type": "float", "default": "0.08", "description": "位置到达阈值(m)"},
        {"name": "angle_threshold", "type": "float", "default": "0.1", "description": "角度到达阈值(rad)"},
        {"name": "allow_rotation", "type": "bool", "default": "True", "description": "是否允许旋转"},
    ],
    inputs=[],
    outputs=[],
)
class BaseMoveToTargetJibotSkill(SkillBase):
    """JiBot底盘绝对目标点移动（Adapter）：hardware.base_move_to_target_jibot()。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="base_move_to_target_jibot_sdk")
        self.hardware = hardware
        self.params: Optional[BaseMoveToTargetJibotParams] = None
        self._done = False
        self._result = None

    def on_initialize(self, params: BaseMoveToTargetJibotParams) -> Result:
        if not isinstance(params, BaseMoveToTargetJibotParams):
            return Result.fail("Invalid parameters for BaseMoveToTargetJibotSkill")
        self.params = params
        self._done = False
        self._result = None
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return self._result if self._result else Result.ok("Already finished")

        options = MoveToTargetOptions(
            avoid_enabled=bool(self.params.avoid_enabled),
            avoid_distance=float(self.params.avoid_distance),
            linear_velocity=float(self.params.linear_velocity),
            angular_velocity=float(self.params.angular_velocity),
            position_threshold=float(self.params.position_threshold),
            angle_threshold=float(self.params.angle_threshold),
            allow_rotation=bool(self.params.allow_rotation),
        )

        result = self.hardware.base_move_to_target_jibot(
            x=float(self.params.x),
            y=float(self.params.y),
            theta=float(self.params.theta),
            options=options,
        )
        self._done = True
        self._result = result
        if result.success:
            logger.info(
                "base_move_to_target_jibot_sdk: x=%.3f y=%.3f theta=%.3f task_id=%s",
                float(self.params.x),
                float(self.params.y),
                float(self.params.theta),
                result.data.get("task_id") if result.data else "N/A",
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done
