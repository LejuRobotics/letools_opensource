# LeTools/core/domain/chassis_options.py
"""
底盘导航选项数据模型。

与 ROS 消息包 leju_mobile_base_msgs/MoveToTargetOptions 对应，
在 Core 层提供零外部依赖的数据载体。
"""
from dataclasses import dataclass


@dataclass
class MoveToTargetOptions:
    """底盘导航选项配置。

    用于 JiBot/Jarvis 底盘的 base_move 和 move_to_target 服务调用。

    Attributes:
        avoid_enabled: 是否启用避障，False 表示完全关闭避障
        avoid_distance: 避障距离 (m)
        linear_velocity: 线速度 (m/s)
        angular_velocity: 角速度 (rad/s)
        position_threshold: 位置到达阈值 (m)
        angle_threshold: 角度到达阈值 (rad)
        allow_rotation: 是否允许旋转
    """
    avoid_enabled: bool = False
    avoid_distance: float = 0.5
    linear_velocity: float = 0.15
    angular_velocity: float = 0.25
    position_threshold: float = 0.08
    angle_threshold: float = 0.1
    allow_rotation: bool = True
