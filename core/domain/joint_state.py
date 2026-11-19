# LeTools/core/domain/joint_state.py
from dataclasses import dataclass, field
import math
from typing import List, Dict, Optional, Sequence, Tuple


def validate_arm_joint(joint: Sequence[float]) -> List[float]:
    """校验双臂14维有限关节角并返回副本；不换算单位、不限幅。"""
    if not isinstance(joint, (list, tuple)) or len(joint) != 14:
        raise ValueError("joint 必须包含14个关节角，顺序为左臂7个、右臂7个")
    try:
        values = [float(value) for value in joint]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("关节角必须是有限数字") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("关节角必须是有限数字")
    return values


def validate_arm_joint_trajectory(
    times: Sequence[float], q_frames: Sequence[Sequence[float]],
) -> Tuple[List[float], List[List[float]]]:
    """校验原生轨迹：时间点为正、严格递增，与14维关节帧一一对应。"""
    if not isinstance(times, (list, tuple)) or not isinstance(q_frames, (list, tuple)):
        raise ValueError("times 和 q_frames 必须是列表")
    if not times or len(times) != len(q_frames):
        raise ValueError("times 和 q_frames 必须非空且长度一致")
    try:
        time_values = [float(value) for value in times]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("times 必须是有限数字（秒）") from exc
    if not all(math.isfinite(value) and value > 0 for value in time_values):
        raise ValueError("times 必须是大于0的有限时间点（秒）")
    if any(current <= previous for previous, current in zip(time_values, time_values[1:])):
        raise ValueError("times 必须严格递增，表示相对轨迹起点的时间")
    return time_values, [validate_arm_joint(frame) for frame in q_frames]

@dataclass
class JointState:
    """
    机器人关节状态。
    对应 ROS sensor_msgs/JointState 或 SDK 中的 QVTau 数据。
    """
    names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)  # rad
    velocities: List[float] = field(default_factory=list) # rad/s
    efforts: List[float] = field(default_factory=list)    # N*m or A

    def get_position(self, joint_name: str) -> Optional[float]:
        """根据关节名获取位置"""
        if joint_name in self.names:
            idx = self.names.index(joint_name)
            return self.positions[idx] if idx < len(self.positions) else None
        return None

@dataclass
class JointCommand:
    """
    机器人关节控制指令。
    """
    names: List[str] = field(default_factory=list)
    positions: Optional[List[float]] = None
    velocities: Optional[List[float]] = None
    efforts: Optional[List[float]] = None
    time_from_start: float = 0.0
