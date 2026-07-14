# -*- coding: utf-8 -*-
"""Atomic skill: base_move_relative_jibot_sdk.

Aligns with `test_base_move.py` by calling `hardware.base_move_relative_jibot()`.
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
class BaseMoveRelativeJibotParams(SkillParams):
    """对齐 test_base_move.py：hardware.base_move_relative_jibot()。"""

    skill_name: str = "base_move_relative_jibot_sdk"
    x: float = 0.2
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
    label="JiBot底盘相对移动",
    category=["motion", "chassis", "jibot"],
    tree_type="studio_smoke",
    description="对齐 test_base_move.py：调用 hardware.base_move_relative_jibot()",
    params=[
        {"name": "x", "type": "float", "default": "0.2", "description": "相对当前位置的x方向位移(m)"},
        {"name": "y", "type": "float", "default": "0.0", "description": "相对当前位置的y方向位移(m)"},
        {"name": "theta", "type": "float", "default": "0.0", "description": "相对当前位置的yaw角度变化(rad)"},
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
class BaseMoveRelativeJibotSkill(SkillBase):
    """JiBot底盘相对移动（Adapter）：hardware.base_move_relative_jibot()。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="base_move_relative_jibot_sdk")
        self.hardware = hardware
        self.params: Optional[BaseMoveRelativeJibotParams] = None
        self._done = False
        self._result = None

    def on_initialize(self, params: BaseMoveRelativeJibotParams) -> Result:
        if not isinstance(params, BaseMoveRelativeJibotParams):
            return Result.fail("Invalid parameters for BaseMoveRelativeJibotSkill")
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

        result = self.hardware.base_move_relative_jibot(
            x=float(self.params.x),
            y=float(self.params.y),
            theta=float(self.params.theta),
            options=options,
        )
        self._done = True
        self._result = result
        if result.success:
            logger.info(
                "base_move_relative_jibot_sdk: x=%.3f y=%.3f theta=%.3f task_id=%s",
                float(self.params.x),
                float(self.params.y),
                float(self.params.theta),
                result.data.get("task_id") if result.data else "N/A",
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done
