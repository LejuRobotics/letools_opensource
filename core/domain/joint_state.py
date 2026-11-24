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


def validate_timed_arm_joint_targets(
    left_arm_joint: Optional[List[float]], right_arm_joint: Optional[List[float]],
    desire_time: Optional[float], is_sync: bool = True,
) -> Tuple[Optional[List[float]], Optional[List[float]], float]:
    """单/双臂multi目标：None表示不提交该臂，空列表是错误，不换算单位。"""
    if left_arm_joint is None and right_arm_joint is None:
        raise ValueError("left_arm_joint/right_arm_joint 至少提供一侧")
    if type(is_sync) is not bool:
        raise ValueError("is_sync 必须为bool，表示规划器执行时长同步")
    try:
        duration = float(desire_time)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("desire_time 必须为大于0的有限秒数") from exc
    if isinstance(desire_time, bool) or not math.isfinite(duration) or duration <= 0:
        raise ValueError("desire_time 必须为大于0的有限秒数")

    def parse(values, label):
        if values is None:
            return None
        if not isinstance(values, (list, tuple)) or len(values) != 7:
            raise ValueError(f"{label} 必须为7维关节角，未控制的一侧请传None")
        try:
            result = [float(value) for value in values]
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{label} 必须包含有限数字") from exc
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{label} 必须包含有限数字")
        return result

    return parse(left_arm_joint, "left_arm_joint"), parse(right_arm_joint, "right_arm_joint"), duration


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
