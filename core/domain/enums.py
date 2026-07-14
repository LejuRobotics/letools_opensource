# LeTools/core/domain/enums.py
from enum import Enum

class FrameType(Enum):
    """坐标系类型 (对应 Kuavo 5-W 接口文档中的 frame 字段)"""
    KEEP_CURRENT = 0   # 保持当前坐标系
    WORLD = 1          # 世界坐标系 (基于 odom)
    LOCAL = 2          # 本地坐标系 (基座坐标系 base_link)
    JOINT_SPACE = 5    # 关节空间坐标系

class MPCControlMode(Enum):
    """轮臂 MPC 控制模式 (对应 /mobile_manipulator_mpc_control 服务)"""
    NO_CONTROL = 0     # 无控制
    ARM_ONLY = 1       # 仅控制手臂，基座固定
    BASE_ONLY = 2      # 仅控制基座，手臂固定
    BASE_ARM = 3       # 同时控制基座和手臂
    ARM_EE_ONLY = 4    # 仅控制手臂末端

class ArmSide(Enum):
    """机械臂侧别"""
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"
