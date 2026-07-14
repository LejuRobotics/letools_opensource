# kuavo_application_framework/core/domain/task.py
from dataclasses import dataclass
from .pose import Pose6D

@dataclass
class TaskPoint:
    """预定义的任务点"""
    name: str
    pose: Pose6D
    frame_id: str = "map"

@dataclass
class NavigationGoal:
    """导航执行目标"""
    target_pose: Pose6D
    tolerance_pos: float = 0.05   # 位置到达误差容忍度 (m)
    tolerance_yaw: float = 0.05   # 角度到达误差容忍度 (rad)
    max_speed: float = 0.5        # 最大移动速度 (m/s)