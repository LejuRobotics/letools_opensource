"""
轨迹相关数据结构

用于描述运动轨迹点和离线轨迹。
"""

from dataclasses import dataclass
from typing import List


@dataclass
class TrajectoryPoint:
    """
    轨迹点
    
    属性:
        desire_time: 期望执行时间（秒），第一帧必须为0
        cmd_vec: 命令向量列表，维度取决于规划器类型
    
    示例:
        # 底盘轨迹点 (3维: x, y, yaw)
        point = TrajectoryPoint(
            desire_time=2.0,
            cmd_vec=[0.5, 0.0, 1.57]  # 前进0.5米，旋转90度
        )
        
        # 手臂末端轨迹点 (6维: x, y, z, yaw, pitch, roll)
        point = TrajectoryPoint(
            desire_time=3.0,
            cmd_vec=[0.5, 0.2, 0.8, 0.0, -1.57, 0.0]
        )
    """
    desire_time: float
    cmd_vec: List[float]
    
    def validate(self) -> bool:
        """
        验证轨迹点的有效性
        
        :return: True如果有效，否则False
        """
        if self.desire_time < 0:
            return False
        
        if not self.cmd_vec or len(self.cmd_vec) == 0:
            return False
        
        return True
    
    def get_dimension(self) -> int:
        """获取命令向量维度"""
        return len(self.cmd_vec)
    
    def to_dict(self) -> dict:
        """转换为字典格式"""
        return {
            'desire_time': self.desire_time,
            'cmd_vec': self.cmd_vec,
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'TrajectoryPoint':
        """从字典创建实例"""
        return cls(
            desire_time=data['desire_time'],
            cmd_vec=data['cmd_vec'],
        )


@dataclass
class OfflineTrajectory:
    """
    离线轨迹
    
    属性:
        planner_index: 规划器索引
            - 0: 左臂笛卡尔世界系
            - 1: 右臂笛卡尔世界系
            - 2: 躯干笛卡尔局部系
        frame: 坐标系
            - 0: 世界系
            - 1: 局部系
        timed_traj: 定时轨迹点列表
    
    示例:
        # 创建左臂笛卡尔轨迹
        trajectory = OfflineTrajectory(
            planner_index=0,
            frame=0,  # 世界系
            timed_traj=[
                TrajectoryPoint(desire_time=0.0, cmd_vec=[0.3, 0.4, 0.7, 0.0, 0.0, 0.0]),
                TrajectoryPoint(desire_time=2.0, cmd_vec=[0.5, 0.4, 0.7, 0.0, -1.57, 0.0]),
                TrajectoryPoint(desire_time=4.0, cmd_vec=[0.5, 0.2, 0.85, 0.0, -1.57, 0.0]),
            ]
        )
        
        # 创建躯干轨迹
        trajectory = OfflineTrajectory(
            planner_index=2,
            frame=1,  # 局部系
            timed_traj=[
                TrajectoryPoint(desire_time=0.0, cmd_vec=[0.0, 0.8, 0.0, 0.0]),
                TrajectoryPoint(desire_time=3.0, cmd_vec=[0.1, 0.9, 0.1, -0.2]),
            ]
        )
    """
    planner_index: int
    frame: int
    timed_traj: List[TrajectoryPoint]
    
    def validate(self) -> bool:
        """
        验证轨迹的有效性
        
        :return: True如果有效，否则False
        """
        # 检查规划器索引范围
        if self.planner_index not in [0, 1, 2]:
            return False
        
        # 检查坐标系
        if self.frame not in [0, 1]:
            return False
        
        # 检查轨迹点不为空
        if not self.timed_traj or len(self.timed_traj) == 0:
            return False
        
        # 检查第一帧时间为0
        if self.timed_traj[0].desire_time != 0.0:
            return False
        
        # 检查时间严格递增
        for i in range(1, len(self.timed_traj)):
            if self.timed_traj[i].desire_time <= self.timed_traj[i-1].desire_time:
                return False
        
        # 检查所有轨迹点有效
        for point in self.timed_traj:
            if not point.validate():
                return False
        
        return True
    
    def get_duration(self) -> float:
        """获取轨迹总时长"""
        if not self.timed_traj:
            return 0.0
        return self.timed_traj[-1].desire_time
    
    def get_num_points(self) -> int:
        """获取轨迹点数"""
        return len(self.timed_traj)
    
    def to_dict(self) -> dict:
        """转换为字典格式"""
        return {
            'planner_index': self.planner_index,
            'frame': self.frame,
            'timed_traj': [point.to_dict() for point in self.timed_traj],
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'OfflineTrajectory':
        """从字典创建实例"""
        return cls(
            planner_index=data['planner_index'],
            frame=data['frame'],
            timed_traj=[
                TrajectoryPoint.from_dict(point_data) 
                for point_data in data['timed_traj']
            ],
        )
    
    @staticmethod
    def create_from_poses(
        planner_index: int,
        frame: int,
        poses: List[List[float]],
        times: List[float]
    ) -> 'OfflineTrajectory':
        """
        从位姿和时间列表创建轨迹
        
        :param planner_index: 规划器索引
        :param frame: 坐标系
        :param poses: 位姿列表，每个位姿是一个命令向量
        :param times: 时间列表（秒）
        :return: OfflineTrajectory实例
        
        示例:
            trajectory = OfflineTrajectory.create_from_poses(
                planner_index=0,
                frame=0,
                poses=[
                    [0.3, 0.4, 0.7, 0.0, 0.0, 0.0],
                    [0.5, 0.4, 0.7, 0.0, -1.57, 0.0],
                ],
                times=[0.0, 2.0]
            )
        """
        if len(poses) != len(times):
            raise ValueError("poses and times must have the same length")
        
        timed_traj = [
            TrajectoryPoint(desire_time=t, cmd_vec=p)
            for t, p in zip(times, poses)
        ]
        
        return OfflineTrajectory(
            planner_index=planner_index,
            frame=frame,
            timed_traj=timed_traj,
        )
