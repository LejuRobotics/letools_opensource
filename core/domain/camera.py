# core/domain/camera.py
from dataclasses import dataclass
from typing import Optional, List, Dict, Any
import numpy as np

@dataclass
class CameraIntrinsics:
    """相机内参矩阵参数"""
    fx: float  # 焦距 x
    fy: float  # 焦距 y
    cx: float  # 主点 x
    cy: float  # 主点 y
    width: int
    height: int
    distortion_coeffs: Optional[List[float]] = None  # 畸变系数 [k1, k2, p1, p2, k3]

    @property
    def matrix(self) -> np.ndarray:
        """返回3x3内参矩阵"""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

@dataclass
class CameraInfo:
    """相机信息"""
    camera_type: str
    resolution: tuple
    frame_rate: float
    intrinsics: CameraIntrinsics
    extrinsics: Optional[Dict[str, Any]] = None
    frame_id: str = ""
    distortion_model: str = ""
    rectification_matrix: Optional[List[float]] = None
    projection_matrix: Optional[List[float]] = None

@dataclass
class CameraFrame:
    """相机帧数据；color_image 统一为 RGB uint8 HWC。"""
    color_image: Any
    depth_image: Optional[Any] = None
    timestamp: float = 0.0  # 彩色图原始 ROS Header 时间戳。
    depth_timestamp: Optional[float] = None  # 深度图原始 ROS Header 时间戳。
    color_frame_id: str = ""
    depth_frame_id: str = ""
    sync_delta_sec: Optional[float] = None  # 严格配对时两路时间戳的绝对差。
    sequence: int = 0  # Adapter 内同步帧序号；0 表示普通非同步缓存帧。

@dataclass
class PointCloudData:
    """点云数据"""
    points: np.ndarray  # Nx3 数组，每行 [x, y, z]
    colors: Optional[np.ndarray] = None  # Nx3 数组，每行 [r, g, b]
    timestamp: float = 0.0
    frame_id: str = "camera_depth_optical_frame"

@dataclass
class DepthData:
    """深度图数据"""
    depth_image: np.ndarray  # 深度图像数组 (单位: 米或毫米)
    intrinsics: CameraIntrinsics
    timestamp: float = 0.0
    scale: float = 1.0  # 深度值缩放因子，用于转换单位

@dataclass
class CameraStatus:
    """相机状态"""
    is_running: bool
    frame_count: int
    fps: float
    error_message: Optional[str] = None

@dataclass
class CameraExtrinsics:
    """相机外参（相对于某个参考坐标系）"""
    translation: np.ndarray  # [x, y, z]
    rotation: np.ndarray     # 旋转矩阵 3x3
    frame_id: str            # 参考坐标系名称
