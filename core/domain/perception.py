# kuavo_application_framework/core/domain/perception.py
from dataclasses import dataclass
from typing import List, Optional
from .pose import Pose6D

@dataclass
class ObjectDetection:
    """通用物体检测结果 (参考 YOLO 等检测算法输出)"""
    label: str
    confidence: float
    bbox: Optional[List[float]] = None  # [x_min, y_min, x_max, y_max]
    pose_in_camera: Optional[Pose6D] = None  # 在相机坐标系下的位姿

@dataclass
class TagDetection:
    """标签检测结果 (参考 AprilTag/QR Code)"""
    tag_id: int
    pose_in_world: Pose6D          # 经过 TF 转换后的位姿 (通常是 base_link)
    pose_in_camera: Optional[Pose6D] = None  # 相机坐标系下的原始位姿
    size: float = 0.0              # 标签物理尺寸 (米)
    confidence: float = 1.0        # 置信度或检测质量评分
    timestamp: float = 0.0         # 检测时刻的时间戳
    corner_points: Optional[List[List[float]]] = None  # 图像像素坐标 [[u1,v1], [u2,v2], ...]
    frame_id: str = "base_link"    # 世界坐标系的参考帧

@dataclass
class PerceptionResult:
    """感知模块的统一返回结果"""
    success: bool
    objects: List[ObjectDetection] = None
    tags: List[TagDetection] = None
    message: str = ""

    def __post_init__(self):
        if self.objects is None:
            self.objects = []
        if self.tags is None:
            self.tags = []