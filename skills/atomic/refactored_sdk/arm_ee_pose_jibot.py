# -*- coding: utf-8 -*-
"""Atomic skill: arm_ee_pose_jibot.

对齐 `test_arm_ee_single_both.py`：
- 单臂: hardware.send_ee_pose(side, pose, frame)
- 双臂: hardware.send_both_ee_poses(left_pose, right_pose, frame)

底层路径: 直接发布到 /mm/two_arm_hand_pose_cmd (kuavo_msgs/twoArmHandPoseCmd)
依赖前置: set_arm_control_mode(2) + set_mpc_mode(ARM_EE_ONLY) + _ensure_ee_publisher()
"""

from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.pose import Pose6D
from core.domain.enums import FrameType, ArmSide
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class ArmEePoseJibotParams(SkillParams):
    """对齐 test_arm_ee_single_both.py：末端位姿控制（单臂或双臂）。

    Pose6D 参数单位：位置 米，姿态 度（适配器内部自动转弧度）。
    """

    skill_name: str = "arm_ee_pose_jibot"
    side: str = "both"  # "left" | "right" | "both"
    left_x: float = 1.4
    left_y: float = 0.25
    left_z: float = 1.0
    left_yaw: float = 0.0
    left_pitch: float = 0.0
    left_roll: float = 0.0
    right_x: float = 1.4
    right_y: float = -0.25
    right_z: float = 1.0
    right_yaw: float = 0.0
    right_pitch: float = 0.0
    right_roll: float = 0.0
    frame: int = 2  # 0=KEEP_CURRENT, 1=WORLD, 2=LOCAL
    timeout: float = 30.0


@define_manifest(
    label="手臂末端位姿控制（话题直发）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "对齐 test_arm_ee_single_both.py："
        "单臂 send_ee_pose 或双臂 send_both_ee_poses，"
        "直接发布到 /mm/two_arm_hand_pose_cmd。"
        "前置: set_arm_control_mode(2) + set_mpc_mode(ARM_EE_ONLY) + _ensure_ee_publisher()"
    ),
    params=[
        {"name": "side", "type": "string", "default": "both",
         "description": "控制侧: left / right / both"},
        {"name": "left_x", "type": "float", "default": "1.4", "description": "左手 X (米)"},
        {"name": "left_y", "type": "float", "default": "0.25", "description": "左手 Y (米)"},
        {"name": "left_z", "type": "float", "default": "1.0", "description": "左手 Z (米)"},
        {"name": "left_yaw", "type": "float", "default": "0.0", "description": "左手 yaw (度)"},
        {"name": "left_pitch", "type": "float", "default": "0.0", "description": "左手 pitch (度)"},
        {"name": "left_roll", "type": "float", "default": "0.0", "description": "左手 roll (度)"},
        {"name": "right_x", "type": "float", "default": "1.4", "description": "右手 X (米)"},
        {"name": "right_y", "type": "float", "default": "-0.25", "description": "右手 Y (米)"},
        {"name": "right_z", "type": "float", "default": "1.0", "description": "右手 Z (米)"},
        {"name": "right_yaw", "type": "float", "default": "0.0", "description": "右手 yaw (度)"},
        {"name": "right_pitch", "type": "float", "default": "0.0", "description": "右手 pitch (度)"},
        {"name": "right_roll", "type": "float", "default": "0.0", "description": "右手 roll (度)"},
        {"name": "frame", "type": "int", "default": "2",
         "description": "坐标系: 0=KEEP_CURRENT, 1=WORLD, 2=LOCAL"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEePoseJibotSkill(SkillBase):
    """手臂末端位姿控制（话题直发）→ send_ee_pose / send_both_ee_poses。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_ee_pose_jibot")
        self.hardware = hardware
        self.params: Optional[ArmEePoseJibotParams] = None
        self._done = False

    def on_initialize(self, params: ArmEePoseJibotParams) -> Result:
        if not isinstance(params, ArmEePoseJibotParams):
            return Result.fail("Invalid parameters for ArmEePoseJibotSkill")

        side = str(params.side).lower()
        if side not in ("left", "right", "both"):
            return Result.fail(f"arm_ee_pose_jibot: invalid side '{side}', expected left/right/both")

        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmEePoseJibotSkill already finished")

        side = str(self.params.side).lower()
        frame_map = {
            0: FrameType.KEEP_CURRENT,
            1: FrameType.WORLD,
            2: FrameType.LOCAL,
        }
        frame = frame_map.get(self.params.frame, FrameType.LOCAL)

        left_pose = Pose6D(
            x=self.params.left_x,
            y=self.params.left_y,
            z=self.params.left_z,
            yaw=self.params.left_yaw,
            pitch=self.params.left_pitch,
            roll=self.params.left_roll,
        )
        right_pose = Pose6D(
            x=self.params.right_x,
            y=self.params.right_y,
            z=self.params.right_z,
            yaw=self.params.right_yaw,
            pitch=self.params.right_pitch,
            roll=self.params.right_roll,
        )

        if side == "both":
            fn = getattr(self.hardware, "send_both_ee_poses", None)
            if fn is None:
                self._done = True
                return Result.fail("Hardware does not implement send_both_ee_poses()")
            result = fn(left_pose, right_pose, frame)
        else:
            fn = getattr(self.hardware, "send_ee_pose", None)
            if fn is None:
                self._done = True
                return Result.fail("Hardware does not implement send_ee_pose()")
            arm_side = ArmSide.LEFT if side == "left" else ArmSide.RIGHT
            result = fn(arm_side, left_pose if side == "left" else right_pose, frame)

        self._done = True
        if result.success:
            logger.info(
                "arm_ee_pose_jibot: side=%s frame=%s left=(%.2f,%.2f,%.2f) right=(%.2f,%.2f,%.2f)",
                side, frame.name,
                left_pose.x, left_pose.y, left_pose.z,
                right_pose.x, right_pose.y, right_pose.z,
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done
