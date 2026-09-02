# -*- coding: utf-8 -*-
"""单 Tag 双臂抓取关节轨迹规划技能 (Refactored SDK)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple

from core.common.logger import get_logger
from core.domain.pose import Pose6D
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.scenarios.wheel_arm_single_tag_pick_v1.utils.source_joint_planner import (
    generate_source_keypoints,
    plan_source_joint_trajectory,
)
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class SingleTagJointPickPlanParams(SkillParams):
    """单 Tag 关节规划参数"""

    skill_name: str = "single_tag_joint_pick_plan"
    tag_pose: Optional[Pose6D] = None
    box_width: float = 0.35
    box_behind_tag: float = 0.0
    box_beneath_tag: float = 0.0
    box_left_tag: float = 0.0
    hand_pitch_degree: float = 0.0
    traj_point_num: int = 100
    ik_retry_count: int = 5
    enable_joint_mirroring: bool = True
    enable_high_position_accuracy: bool = False
    timeout: float = 10.0


@define_manifest(
    label="单Tag抓取关节规划",
    category=["planning", "arm"],
    tree_type="studio_smoke",
    description="根据Tag位姿与箱体几何尺寸解算关键点并规划双臂7-DOF轨迹",
    params=[
        {"name": "box_width", "type": "float", "default": "0.35", "description": "箱体宽度"},
        {"name": "hand_pitch_degree", "type": "float", "default": "0.0", "description": "末端俯仰角"},
    ],
    inputs=[],
    outputs=[],
)
class SingleTagJointPickPlanSkill(SkillBase):
    """
    原子技能：根据 Tag 位姿规划双臂抓取关节轨迹。
    输入：Tag 位姿及几何参数
    输出：Result.ok(data={'left_arm_joint_traj': left, 'right_arm_joint_traj': right})
    """

    def __init__(self, hardware: IHardware):
        super().__init__(name="single_tag_joint_pick_plan")
        self.hardware = hardware
        self.params: Optional[SingleTagJointPickPlanParams] = None
        self._left_traj: Optional[List[List[float]]] = None
        self._right_traj: Optional[List[List[float]]] = None
        self._is_finished = False

    def on_initialize(self, params: SingleTagJointPickPlanParams) -> Result:
        if not isinstance(params, SingleTagJointPickPlanParams):
            return Result.fail("Invalid parameters for SingleTagJointPickPlanSkill")
        if params.tag_pose is None:
            return Result.fail("tag_pose cannot be None")

        self.params = params
        self._left_traj = None
        self._right_traj = None
        self._is_finished = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._is_finished and self._left_traj is not None:
            return Result.ok(
                "Planning already finished",
                data={
                    "left_arm_joint_traj": self._left_traj,
                    "right_arm_joint_traj": self._right_traj,
                },
            )

        try:
            keypoints = generate_source_keypoints(
                self.params.box_width,
                self.params.box_behind_tag,
                self.params.box_beneath_tag,
                self.params.box_left_tag,
                self.params.hand_pitch_degree,
            )
            left, right = plan_source_joint_trajectory(
                self.hardware,
                self.params.tag_pose,
                keypoints,
                enable_joint_mirroring=self.params.enable_joint_mirroring,
                enable_high_position_accuracy=self.params.enable_high_position_accuracy,
                traj_point_num=self.params.traj_point_num,
                ik_retry_count=self.params.ik_retry_count,
            )
            self._left_traj = left
            self._right_traj = right
            self._is_finished = True
            logger.info(f"Planned bimanual trajectory: {len(left)} frames")
            return Result.ok(
                "Trajectory planned successfully",
                data={
                    "left_arm_joint_traj": left,
                    "right_arm_joint_traj": right,
                },
            )
        except Exception as exc:
            logger.exception(f"SingleTagJointPickPlanSkill planning failed: {exc}")
            return Result.fail(f"Planning failed: {exc}")

    def on_is_finished(self) -> bool:
        return self._is_finished
