"""
视觉标签(Tag)数据结构

用于描述AprilTag等视觉标记的信息。
"""

from dataclasses import dataclass
from typing import Optional
from .pose import Pose6D


@dataclass
class Tag:
    """
    视觉标签
    
    属性:
        id: Tag ID
        pose: Tag位姿（相对于某个坐标系）
        timestamp: 检测时间戳（秒），可选
    
    示例:
        # 创建一个Tag
        tag = Tag(
            id=1,
            pose=Pose6D.from_euler(
                x=0.5, y=0.0, z=0.8,
                yaw=0.0, pitch=-0.5, roll=0.0,
                degrees=False
            ),
            timestamp=1234567890.123
        )
        
        # 访问Tag信息
        print(f"Tag {tag.id} at position: {tag.pose.x}, {tag.pose.y}, {tag.pose.z}")
    """
    id: int
    pose: Pose6D
    timestamp: Optional[float] = None
    
    def validate(self) -> bool:
        """
        验证Tag的有效性
        
        :return: True如果有效，否则False
        """
        if self.id < 0:
            return False
        
        if not self.pose.validate():
            return False
        
        return True
    
    def to_dict(self) -> dict:
        """转换为字典格式"""
        return {
            'id': self.id,
            'pose': self.pose.to_dict(),
            'timestamp': self.timestamp,
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'Tag':
        """从字典创建实例"""
        return cls(
            id=data['id'],
            pose=Pose6D.from_dict(data['pose']),
            timestamp=data.get('timestamp'),
        )
    
    def get_distance_to(self, other_pose: Pose6D) -> float:
        """
        计算到另一个位姿的欧氏距离
        
        :param other_pose: 目标位姿
        :return: 距离（米）
        """
        dx = self.pose.x - other_pose.x
        dy = self.pose.y - other_pose.y
        dz = self.pose.z - other_pose.z
        return (dx**2 + dy**2 + dz**2) ** 0.5
    
    def is_fresh(self, current_time: float, max_age: float = 1.0) -> bool:
        """
        检查Tag是否新鲜（未过期）
        
        :param current_time: 当前时间戳
        :param max_age: 最大允许年龄（秒）
        :return: True如果Tag新鲜，否则False
        """
        if self.timestamp is None:
            return False
        
        return (current_time - self.timestamp) <= max_age
    
    def __str__(self) -> str:
        """字符串表示"""
        return f"Tag(id={self.id}, pos=[{self.pose.x:.3f}, {self.pose.y:.3f}, {self.pose.z:.3f}])"
    
    def __repr__(self) -> str:
        """详细字符串表示"""
        return (f"Tag(id={self.id}, "
                f"pose=Pose6D(x={self.pose.x:.3f}, y={self.pose.y:.3f}, z={self.pose.z:.3f}, "
                f"yaw={self.pose.yaw:.3f}, pitch={self.pose.pitch:.3f}, roll={self.pose.roll:.3f}), "
                f"timestamp={self.timestamp})")
