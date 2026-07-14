# LeTools/core/common/interpolator.py
import numpy as np
from scipy.interpolate import CubicSpline
from typing import List, Tuple
from ..domain.pose import Pose6D

def linear_interpolate(start: float, end: float, t: float) -> float:
    """
    一维线性插值。
    :param t: 插值参数 [0, 1]
    """
    return start + (end - start) * t

def slerp(q0: Tuple[float, float, float, float], 
          q1: Tuple[float, float, float, float], 
          t: float) -> Tuple[float, float, float, float]:
    """
    四元数球面线性插值 (Slerp)。
    用于手臂姿态的平滑过渡。
    """
    q0 = np.array(q0)
    q1 = np.array(q1)
    
    # 确保最短路径
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    
    dot = np.clip(dot, -1.0, 1.0)
    theta = np.arccos(dot) * t
    
    temp = q1 - q0 * dot
    temp = temp / np.linalg.norm(temp)
    
    result = q0 * np.cos(theta) + temp * np.sin(theta)
    return tuple(result.tolist())

def cubic_spline_interpolate(times: List[float], values: List[List[float]], 
                             num_points: int = 100) -> Tuple[np.ndarray, np.ndarray]:
    """
    多维三次样条插值。
    :param times: 时间点列表 [t0, t1, ...]
    :param values: 对应的数值列表 [[v0_1, v0_2], [v1_1, v1_2], ...]
    :return: (新的时间点数组, 插值后的数值数组)
    """
    times = np.array(times)
    values = np.array(values)
    
    if len(times) < 2:
        return times, values
        
    # 为每一维创建样条
    new_times = np.linspace(times[0], times[-1], num_points)
    interpolated_values = np.zeros((num_points, values.shape[1]))
    
    for i in range(values.shape[1]):
        cs = CubicSpline(times, values[:, i])
        interpolated_values[:, i] = cs(new_times)
        
    return new_times, interpolated_values

def generate_cartesian_waypoints(start_pose: Pose6D, end_pose: Pose6D, 
                                 steps: int = 50) -> List[Pose6D]:
    """
    在两个笛卡尔位姿之间生成线性插值点。
    """
    waypoints = []
    for i in range(steps + 1):
        t = i / steps
        p = Pose6D(
            x=linear_interpolate(start_pose.x, end_pose.x, t),
            y=linear_interpolate(start_pose.y, end_pose.y, t),
            z=linear_interpolate(start_pose.z, end_pose.z, t),
            roll=linear_interpolate(start_pose.roll, end_pose.roll, t),
            pitch=linear_interpolate(start_pose.pitch, end_pose.pitch, t),
            yaw=linear_interpolate(start_pose.yaw, end_pose.yaw, t)
        )
        waypoints.append(p)
    return waypoints
