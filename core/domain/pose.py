# LeTools/core/domain/pose.py
from dataclasses import dataclass, field
from typing import Tuple, List, Optional
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

def validate_arm_pose(pose: List[float]) -> List[float]:
    """校验6维末端位姿[x,y,z,yaw,pitch,roll]，不改单位或输入。"""
    if not isinstance(pose, (list, tuple)) or len(pose) != 6:
        raise ValueError("末端位姿必须为6维[x,y,z,yaw,pitch,roll]")
    try:
        values = [float(value) for value in pose]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("末端位姿必须包含有限数字") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("末端位姿必须包含有限数字")
    return values


def validate_arm_frame(frame: str) -> str:
    """新末端接口只接受local/base_link和world/odom，不接受隐式fallback。"""
    frames = {"local": "local", "base_link": "local", "world": "world", "odom": "world"}
    if not isinstance(frame, str) or frame not in frames:
        raise ValueError("frame 必须为 local/base_link 或 world/odom")
    return frames[frame]


def validate_timed_arm_pose_targets(
    left_pose: Optional[List[float]], right_pose: Optional[List[float]],
    desire_time: Optional[float], frame: str = "local", is_sync: bool = True,
) -> Tuple[Optional[List[float]], Optional[List[float]], float, str]:
    """校验单/双臂定时位姿；None为不提交，返回m/deg副本与规范坐标系。"""
    if left_pose is None and right_pose is None:
        raise ValueError("left_pose/right_pose 至少提供一侧")
    if type(is_sync) is not bool:
        raise ValueError("is_sync 必须为bool，表示规划器执行时长同步")
    if isinstance(desire_time, bool):
        raise ValueError("desire_time 必须为大于0的有限秒数")
    try:
        duration = float(desire_time)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("desire_time 必须为大于0的有限秒数") from exc
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("desire_time 必须为大于0的有限秒数")
    left = None if left_pose is None else validate_arm_pose(left_pose)
    right = None if right_pose is None else validate_arm_pose(right_pose)
    return left, right, duration, validate_arm_frame(frame)


@dataclass
class TransformMatrix:
    """4x4 齐次变换矩阵"""
    matrix: np.ndarray

    def __post_init__(self):
        if self.matrix.shape != (4, 4):
            raise ValueError("TransformMatrix must be 4x4")
