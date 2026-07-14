# LeTools/core/domain/end_effector.py
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional

class EndEffectorType(Enum):
    """末端执行器类型"""
    LEJU_CLAW = "leju_claw"       # 乐聚二指夹爪
    QIANGNAO_HAND = "qiangnao"    # 强脑灵巧手
    SUCTION_CUP = "suction_cup"   # 吸盘
    NONE = "none"                 # 无末端

class GripperStatus(Enum):
    """夹爪/手部状态 (对应 lejuClawState.msg)"""
    ERROR = -1
    UNKNOWN = 0
    MOVING = 1
    REACHED = 2
    GRABBED = 3

@dataclass
class GripperCommand:
    """
    通用夹爪控制指令。
    适用于二指夹爪或简单的开合动作。
    """
    position: float = 0.0   # 行程占比 [0, 100], 0为张开, 100为闭合
    velocity: float = 50.0  # 速度 [0, 100]
    effort: float = 1.0     # 力矩/电流 (A)

@dataclass
class HandFingerCommand:
    """
    灵巧手手指控制指令。
    对应 robotHandPosition.msg，通常包含6个手指关节的位置。
    """
    positions: List[float] = field(default_factory=lambda: [0.0] * 6) # [0, 100]

@dataclass
class EndEffectorState:
    """
    末端执行器实时状态。
    """
    status: GripperStatus = GripperStatus.UNKNOWN
    current_position: float = 0.0
    current_velocity: float = 0.0
    current_effort: float = 0.0
    finger_positions: Optional[List[float]] = None # 仅灵巧手有效

@dataclass
class DualEndEffectorState:
    """
    双臂末端状态封装。
    """
    left: EndEffectorState = field(default_factory=EndEffectorState)
    right: EndEffectorState = field(default_factory=EndEffectorState)
