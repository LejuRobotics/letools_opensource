# -*- coding: utf-8 -*-
"""Atomic skill: base_pose_local.

Aligns with `test_cmd_pose_base.py` by calling `hardware.send_base_pose(..., frame=LOCAL)`.
"""

from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.enums import FrameType
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class BasePoseLocalParams(SkillParams):
    """对齐 test_cmd_pose_base.py：hardware.send_base_pose(x, y, yaw, frame=LOCAL)。"""

    skill_name: str = "base_pose_local"
    x: float = 0.5
    y: float = 0.0
    yaw: float = 0.0
    frame: FrameType = FrameType.LOCAL
    timeout: float = 60.0


@define_manifest(
    label="底盘相对位姿（本体坐标）",
    category=["motion", "chassis"],
    tree_type="studio_smoke",
    description="对齐 test_cmd_pose_base.py：调用 hardware.send_base_pose(frame=LOCAL)",
    params=[
        {"name": "x", "type": "float", "default": "0.5", "description": "前后位移（m）"},
        {"name": "y", "type": "float", "default": "0.0", "description": "左右位移（m）"},
        {"name": "yaw", "type": "float", "default": "0.0", "description": "偏航（deg，适配器会转换）"},
    ],
    inputs=[],
    outputs=[],
)
class BasePoseLocalSkill(SkillBase):
    """底盘相对位姿（Adapter）：hardware.send_base_pose(frame=LOCAL) 并由适配器等待到达。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="base_pose_local")
        self.hardware = hardware
        self.params: Optional[BasePoseLocalParams] = None
        self._done = False

    def on_initialize(self, params: BasePoseLocalParams) -> Result:
        if not isinstance(params, BasePoseLocalParams):
            return Result.fail("Invalid parameters for BasePoseLocalSkill")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("BasePoseLocalSkill already finished")

        result = self.hardware.send_base_pose(
            x=float(self.params.x),
            y=float(self.params.y),
            yaw=float(self.params.yaw),
            frame=self.params.frame,
        )
        self._done = True
        if result.success:
            logger.info(
                "base_pose_local: x=%.3f y=%.3f yaw=%.3f frame=%s",
                float(self.params.x),
                float(self.params.y),
                float(self.params.yaw),
                getattr(self.params.frame, "name", str(self.params.frame)),
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done

