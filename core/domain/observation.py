# kuavo_application_framework/core/domain/observation.py
from dataclasses import dataclass, field
from typing import List, Optional, Dict
import numpy as np
from .joint_state import JointState
from .pose import Pose6D

@dataclass
class CameraImage:
    """标准化的相机图像数据"""
    name: str  # 相机名称，如 'head_camera', 'wrist_camera'
    width: int
    height: int
    channels: int
    data: np.ndarray  # 形状: [H, W, C]，通常为 uint8
    timestamp: float = 0.0

@dataclass
class Observation:
    """
    策略输入的标准化观测空间。
    对应 RL/IL 中的 State Space。
    """
    # --- 本体感知 (Proprioception) ---
    joint_state: JointState           # 全身关节状态 (pos, vel, effort)
    base_pose: Pose6D                   # 基座在世界坐标系下的位姿
    end_effector_poses: Dict[str, Pose6D] # 末端位姿 {'left': Pose6D, 'right': Pose6D}
    
    # --- 环境感知 (Exteroception) ---
    images: List[CameraImage] = field(default_factory=list)
    depth_maps: List[np.ndarray] = field(default_factory=list) # 可选的深度图
    
    # --- 其它传感器 ---
    force_torque: Optional[List[float]] = None # 腕部或足端力控信息
    gripper_states: Dict[str, float] = field(default_factory=dict) # {'left': 0.5, 'right': 0.0}
    
    # --- 元数据 ---
    timestamp: float = 0.0
    task_description: Optional[str] = None # 用于 VLA 模型的文本指令