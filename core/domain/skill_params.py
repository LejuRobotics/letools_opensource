# kuavo_application_framework/core/domain/skill_params.py
from dataclasses import dataclass, field
from typing import Any, Dict
from .pose import Pose6D

@dataclass
class SkillParams:
    """
    所有技能参数的抽象基类。
    支持从 YAML 配置文件直接反序列化为对象。
    """
    skill_name: str = ""
    timeout: float = 30.0       # 技能执行超时时间 (秒)
    retry_count: int = 3        # 失败重试次数
    extra_kwargs: Dict[str, Any] = field(default_factory=dict)

@dataclass
class MoveToPoseParams(SkillParams):
    """移动到指定位姿的参数"""
    target_pose: Pose6D = field(default_factory=Pose6D)
    speed_factor: float = 0.5   # 速度倍率 (0.0 - 1.0)
    tolerance_pos: float = 0.01 # 位置容忍度 (米)

@dataclass
class PickObjectParams(SkillParams):
    """抓取物体的参数"""
    object_label: str = "box"
    approach_distance: float = 0.1 # 接近距离 (米)
    gripper_force: float = 10.0    # 抓取力 (N)