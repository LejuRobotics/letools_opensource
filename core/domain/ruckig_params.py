"""
Ruckig规划器参数数据结构

用于配置运动规划器的速度、加速度、急动度限制。
"""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class RuckigParams:
    """
    Ruckig规划器参数
    
    属性:
        velocity_max: 最大速度列表（需与规划器自由度匹配）
        acceleration_max: 最大加速度列表（需与规划器自由度匹配）
        jerk_max: 最大急动度列表（需与规划器自由度匹配）
        velocity_min: 最小速度列表（可选，默认为 -velocity_max）
        acceleration_min: 最小加速度列表（可选，默认为 -acceleration_max）
    
    示例:
        # 底盘规划器参数 (3维: x, y, yaw)
        params = RuckigParams(
            velocity_max=[0.2, 0.2, 0.6],      # m/s, m/s, rad/s
            acceleration_max=[4.0, 4.0, 4.0],   # m/s², m/s², rad/s²
            jerk_max=[20.0, 15.0, 12.0]         # m/s³, m/s³, rad/s³
        )
        
        # 手臂关节规划器参数 (7维)
        params = RuckigParams(
            velocity_max=[1.0] * 7,             # rad/s
            acceleration_max=[2.0] * 7,         # rad/s²
            jerk_max=[10.0] * 7                 # rad/s³
        )
    """
    velocity_max: List[float]
    acceleration_max: List[float]
    jerk_max: List[float]
    velocity_min: Optional[List[float]] = None
    acceleration_min: Optional[List[float]] = None
    
    def __post_init__(self):
        """初始化后处理：设置默认的最小值"""
        if self.velocity_min is None:
            self.velocity_min = [-v for v in self.velocity_max]
        
        if self.acceleration_min is None:
            self.acceleration_min = [-a for a in self.acceleration_max]
    
    def validate(self) -> bool:
        """
        验证参数的有效性
        
        :return: True如果参数有效，否则False
        """
        # 检查列表长度一致
        if not (len(self.velocity_max) == 
                len(self.acceleration_max) == 
                len(self.jerk_max)):
            return False
        
        # 检查最小值长度（如果提供）
        if self.velocity_min and len(self.velocity_min) != len(self.velocity_max):
            return False
        
        if self.acceleration_min and len(self.acceleration_min) != len(self.acceleration_max):
            return False
        
        # 检查数值合理性
        if any(v <= 0 for v in self.velocity_max):
            return False
        
        if any(a <= 0 for a in self.acceleration_max):
            return False
        
        if any(j <= 0 for j in self.jerk_max):
            return False
        
        return True
    
    def get_dimension(self) -> int:
        """获取规划器维度"""
        return len(self.velocity_max)
    
    def to_dict(self) -> dict:
        """转换为字典格式（用于序列化）"""
        return {
            'velocity_max': self.velocity_max,
            'acceleration_max': self.acceleration_max,
            'jerk_max': self.jerk_max,
            'velocity_min': self.velocity_min,
            'acceleration_min': self.acceleration_min,
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'RuckigParams':
        """从字典创建实例"""
        return cls(
            velocity_max=data['velocity_max'],
            acceleration_max=data['acceleration_max'],
            jerk_max=data['jerk_max'],
            velocity_min=data.get('velocity_min'),
            acceleration_min=data.get('acceleration_min'),
        )
    
    @staticmethod
    def create_chassis_params(
        vel_xy: float = 0.2,
        vel_yaw: float = 0.6,
        acc_xy: float = 4.0,
        acc_yaw: float = 4.0,
        jerk_xy: float = 20.0,
        jerk_yaw: float = 12.0
    ) -> 'RuckigParams':
        """
        创建底盘规划器参数
        
        :param vel_xy: X/Y轴最大速度 (m/s)
        :param vel_yaw: Yaw轴最大角速度 (rad/s)
        :param acc_xy: X/Y轴最大加速度 (m/s²)
        :param acc_yaw: Yaw轴最大角加速度 (rad/s²)
        :param jerk_xy: X/Y轴最大急动度 (m/s³)
        :param jerk_yaw: Yaw轴最大急动度 (rad/s³)
        :return: RuckigParams实例
        """
        return RuckigParams(
            velocity_max=[vel_xy, vel_xy, vel_yaw],
            acceleration_max=[acc_xy, acc_xy, acc_yaw],
            jerk_max=[jerk_xy, jerk_xy, jerk_yaw],
        )
    
    @staticmethod
    def create_arm_joint_params(
        vel: float = 1.0,
        acc: float = 2.0,
        jerk: float = 10.0,
        num_joints: int = 7
    ) -> 'RuckigParams':
        """
        创建手臂关节规划器参数
        
        :param vel: 关节最大速度 (rad/s)
        :param acc: 关节最大加速度 (rad/s²)
        :param jerk: 关节最大急动度 (rad/s³)
        :param num_joints: 关节数量
        :return: RuckigParams实例
        """
        return RuckigParams(
            velocity_max=[vel] * num_joints,
            acceleration_max=[acc] * num_joints,
            jerk_max=[jerk] * num_joints,
        )
    
    @staticmethod
    def create_ee_cartesian_params(
        vel_xyz: float = 0.5,
        vel_rpy: float = 1.0,
        acc_xyz: float = 2.0,
        acc_rpy: float = 3.0,
        jerk_xyz: float = 10.0,
        jerk_rpy: float = 15.0
    ) -> 'RuckigParams':
        """
        创建末端笛卡尔空间规划器参数
        
        :param vel_xyz: XYZ轴最大速度 (m/s)
        :param vel_rpy: RPY轴最大角速度 (rad/s)
        :param acc_xyz: XYZ轴最大加速度 (m/s²)
        :param acc_rpy: RPY轴最大角加速度 (rad/s²)
        :param jerk_xyz: XYZ轴最大急动度 (m/s³)
        :param jerk_rpy: RPY轴最大急动度 (rad/s³)
        :return: RuckigParams实例 (6维)
        """
        return RuckigParams(
            velocity_max=[vel_xyz, vel_xyz, vel_xyz, vel_rpy, vel_rpy, vel_rpy],
            acceleration_max=[acc_xyz, acc_xyz, acc_xyz, acc_rpy, acc_rpy, acc_rpy],
            jerk_max=[jerk_xyz, jerk_xyz, jerk_xyz, jerk_rpy, jerk_rpy, jerk_rpy],
        )
