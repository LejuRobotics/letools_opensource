# LeTools/core/common/transform.py
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Tuple
from ..domain.pose import Pose6D

def pose6d_to_matrix(pose: Pose6D) -> np.ndarray:
    """将 Pose6D (欧拉角) 转换为 4x4 齐次矩阵"""
    mat = np.eye(4)
    mat[:3, 3] = [pose.x, pose.y, pose.z]
    
    # 欧拉角 (XYZ) 转旋转矩阵
    r = R.from_euler('xyz', [pose.roll, pose.pitch, pose.yaw])
    mat[:3, :3] = r.as_matrix()
    return mat

def matrix_to_pose6d(matrix: np.ndarray) -> Pose6D:
    """将 4x4 齐次矩阵转换为 Pose6D"""
    if matrix.shape != (4, 4):
        raise ValueError("Matrix must be 4x4")
    
    x, y, z = matrix[:3, 3]
    r = R.from_matrix(matrix[:3, :3])
    roll, pitch, yaw = r.as_euler('xyz')
    
    return Pose6D(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)

def transform_pose(pose: Pose6D, transform_matrix: np.ndarray) -> Pose6D:
    """
    对位姿进行空间变换。
    :param pose: 原始位姿
    :param transform_matrix: 变换矩阵 (例如从 base_link 到 world 的变换)
    :return: 变换后的位姿
    """
    src_mat = pose6d_to_matrix(pose)
    dst_mat = np.dot(transform_matrix, src_mat)
    return matrix_to_pose6d(dst_mat)