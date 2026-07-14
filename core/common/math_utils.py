# LeTools/core/common/math_utils.py
import numpy as np
from typing import Tuple
from ..domain.pose import Pose6D

def calculate_distance(pose1: Pose6D, pose2: Pose6D) -> float:
    """计算两个位姿位置之间的欧氏距离 (m)"""
    p1 = np.array([pose1.x, pose1.y, pose1.z])
    p2 = np.array([pose2.x, pose2.y, pose2.z])
    return np.linalg.norm(p1 - p2)

def is_pose_reached(current: Pose6D, target: Pose6D, pos_tol: float = 0.01, angle_tol: float = 0.05) -> bool:
    """
    判断是否到达目标位姿。
    :param pos_tol: 位置容差 (m)
    :param angle_tol: 角度容差 (rad)
    """
    dist = calculate_distance(current, target)
    angle_diff = max(
        abs(current.roll - target.roll),
        abs(current.pitch - target.pitch),
        abs(current.yaw - target.yaw)
    )
    return dist < pos_tol and angle_diff < angle_tol

def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """
    将四元数转换为欧拉角 (roll, pitch, yaw)
    :param x, y, z, w: 四元数分量
    :return: (roll, pitch, yaw) 单位为弧度
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw