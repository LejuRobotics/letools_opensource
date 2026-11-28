"""轮式躯干单帧/Timed命令校验；固定m/deg，不换算单位或猜测物理限位。"""

import math
from typing import List, Optional, Sequence, Tuple


def _validate_vector(values: Sequence[float], size: int, label: str) -> List[float]:
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError(f"{label} 必须为{size}维数值列表")
    if any(isinstance(value, bool) for value in values):
        raise ValueError(f"{label} 不接受bool")
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} 必须包含有限数字") from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} 必须包含有限数字")
    return result


def validate_torso_joint(joint: Sequence[float]) -> List[float]:
    """4维 [knee, leg, waist_pitch, waist_yaw]，固定deg，返回副本。"""
    return _validate_vector(joint, 4, "joint")


def validate_torso_pose(pose: Sequence[float]) -> List[float]:
    """4维 [x,z,yaw,pitch]，固定m/deg，返回副本。"""
    return _validate_vector(pose, 4, "pose")


def validate_torso_joint_traj(
    joint_traj: Sequence[Sequence[float]], total_time: Optional[float],
) -> Tuple[List[List[float]], float]:
    """至少一个4维deg关键帧；SDK会另插当前状态，时间不少于100Hz采样间隔。"""
    if not isinstance(joint_traj, (list, tuple)) or not joint_traj:
        raise ValueError("joint_traj 必须为至少一帧4维关节轨迹")
    frames = [validate_torso_joint(frame) for frame in joint_traj]
    duration = _validate_timing(total_time, True)
    if duration < 0.01:
        raise ValueError("total_time 必须至少为0.01秒（SDK 100Hz采样间隔）")
    return frames, duration


def _validate_timing(desire_time: Optional[float], is_sync: bool) -> float:
    if type(is_sync) is not bool:
        raise ValueError("is_sync 必须为bool，表示规划时长同步，不是到位等待")
    if isinstance(desire_time, bool):
        raise ValueError("desire_time 必须为大于0的有限秒数")
    try:
        duration = float(desire_time)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("desire_time 必须为大于0的有限秒数") from exc
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("desire_time 必须为大于0的有限秒数")
    return duration


def validate_timed_torso_joint(
    joint: Sequence[float], desire_time: Optional[float], is_sync: bool = True,
) -> Tuple[List[float], float]:
    """4关节deg目标及定时规划参数，不执行单位换算。"""
    return validate_torso_joint(joint), _validate_timing(desire_time, is_sync)


def validate_timed_torso_pose(
    pose: Sequence[float], desire_time: Optional[float], is_sync: bool = True,
) -> Tuple[List[float], float]:
    """4分量[x,z,yaw,pitch] m/deg目标及定时规划参数。"""
    return validate_torso_pose(pose), _validate_timing(desire_time, is_sync)
