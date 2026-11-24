"""ArmAPI定时轨迹输入校验：纯标准库，不预设运动目标。"""

import math
from typing import List, Tuple

from core.domain.joint_state import validate_arm_joint
from core.domain.pose import validate_arm_pose


def validate_total_time(total_time: float) -> float:
    """保证SDK的100Hz重采样至少生成起止两点，拒绝不足一个周期的轨迹。"""
    if isinstance(total_time, bool):
        raise ValueError("total_time必须是至少0.01秒的有限数字")
    try:
        value = float(total_time)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("total_time必须是至少0.01秒的有限数字") from exc
    if not math.isfinite(value) or value < 0.01:
        raise ValueError("total_time必须是至少0.01秒的有限数字")
    return value


def validate_set_arm_joint_traj(
    joint_traj: List[List[float]], total_time: float,
) -> Tuple[List[List[float]], float]:
    """ArmAPI按均匀关键点时间插值；至少两帧，每帧14维，返回副本。"""
    if not isinstance(joint_traj, (list, tuple)) or len(joint_traj) < 2:
        raise ValueError("joint_traj必须至少包含两帧14维关节角")
    return [validate_arm_joint(frame) for frame in joint_traj], validate_total_time(total_time)


def validate_set_arm_pose_traj(
    left_traj: List[List[float]], right_traj: List[List[float]], total_time: float,
) -> Tuple[List[List[float]], List[List[float]], float]:
    """必须提供等长双臂轨迹，防止原生SDK按较短轨迹静默截断。"""
    if (not isinstance(left_traj, (list, tuple)) or not isinstance(right_traj, (list, tuple))
            or len(left_traj) < 2 or len(left_traj) != len(right_traj)):
        raise ValueError("left_traj/right_traj必须等长且至少两帧，每帧6维位姿")
    left = [validate_arm_pose(frame) for frame in left_traj]
    right = [validate_arm_pose(frame) for frame in right_traj]
    return left, right, validate_total_time(total_time)
