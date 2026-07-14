# LeTools/core/domain/pose.py
from dataclasses import dataclass, field
from typing import Tuple, List
import numpy as np
import math

@dataclass
class Pose6D:
    """
    标准 6D 位姿 (对应乐聚接口文档中的位姿格式)
    
    数据格式与底层测试脚本保持一致: [x, y, z, yaw, pitch, roll]
    - 位置单位：米 (m)
    - 姿态单位：弧度 (rad)
    - 欧拉角顺序：yaw-pitch-roll (ZYX)，与底层测试脚本一致
    
    优势:
    1. 直接与底层测试数据对比，便于调试
    2. 减少格式转换，降低出错风险
    3. 算法人员无需记忆多种格式
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0   # ← 与底层格式一致: yaw
    pitch: float = 0.0 # ← 与底层格式一致: pitch
    roll: float = 0.0  # ← 与底层格式一致: roll

    def to_list(self) -> List[float]:
        """转换为列表格式 [x, y, z, yaw, pitch, roll]，与底层测试脚本一致"""
        return [self.x, self.y, self.z, self.yaw, self.pitch, self.roll]

    @staticmethod
    def from_euler(x: float, y: float, z: float, 
                   yaw: float, pitch: float, roll: float, 
                   degrees: bool = False) -> 'Pose6D':
        """从欧拉角创建位姿，支持角度/弧度切换
        
        :param x, y, z: 位置坐标（米）
        :param yaw, pitch, roll: 欧拉角（弧度或角度），与底层测试脚本顺序一致
        :param degrees: 如果为True，输入为角度制，自动转换为弧度制
        :return: Pose6D 对象
        """
        if degrees:
            yaw = math.radians(yaw)
            pitch = math.radians(pitch)
            roll = math.radians(roll)
        return Pose6D(x=x, y=y, z=z, yaw=yaw, pitch=pitch, roll=roll)

    def to_quaternion(self) -> Tuple[float, float, float, float]:
        """将欧拉角转换为四元数 (x, y, z, w)
        
        使用 ZYX 欧拉角顺序 (yaw-pitch-roll)，与底层测试脚本一致
        ROS quaternion_from_euler 期望的顺序是 (roll, pitch, yaw)
        """
        # ROS 的 quaternion_from_euler 使用固定轴旋转顺序: roll(X), pitch(Y), yaw(Z)
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)

@dataclass
class TransformMatrix:
    """4x4 齐次变换矩阵"""
    matrix: np.ndarray

    def __post_init__(self):
        if self.matrix.shape != (4, 4):
            raise ValueError("TransformMatrix must be 4x4")
