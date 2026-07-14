# LeTools/core/domain/joint_state.py
from dataclasses import dataclass, field
from typing import List, Dict, Optional

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
