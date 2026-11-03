#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""几何原语 —— 从 LeTools `skills/atomic/perception/pallet_detect/algorithm.py`
**逐字抄来**的三个函数（`CameraIntrinsics` / `_project_px` / `_corners_mm`）。

抄的理由：本验证包要求**自包含**（新对话不装 ROS 工作区、不 import LeTools 就能跑）。
代价是**改了 LeTools 那两个函数要同步改这里**。三者的实现都很短、而且已经冻结很久。

    `frame` 的约定（**全项目统一，别各写各的**）：
        origin  台面矩形的 (0,0) 角，相机系 **毫米**
        E1      台面平面内，**投影到图像上指向画面右**（+u）的单位向量
        E2      台面平面内，**投影到图像上指向画面里更上方**（-v）的单位向量
        nrm     台面法向，`E1 × E2 = nrm`（右手系）
        W       沿 E1 的边长（毫米）
        H       沿 E2 的边长（毫米）
    四角顺序恒为 `(0,0) -> (W,0) -> (W,H) -> (0,H)`，即 `左下 → 右下 → 右上 → 左上`。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class CameraIntrinsics:
    """针孔内参。与 `rgbd_detector.CameraIntrinsics` 同构（逐字段同名同序）。"""
    fx: float
    fy: float
    cx: float
    cy: float


def _project_px(points: np.ndarray, k: CameraIntrinsics) -> np.ndarray:
    """相机系三维点 -> 像素。`z` 被夹到 1e-6 防止除零（正常帧不会触发）。"""
    z = np.where(np.abs(points[..., 2]) < 1e-6, 1e-6, points[..., 2])
    return np.stack([k.cx + k.fx * points[..., 0] / z,
                     k.cy + k.fy * points[..., 1] / z], axis=-1)


def _corners_mm(frame: dict, W: float, H: float) -> np.ndarray:
    """台面矩形四个角，相机系 mm。顺序 `(0,0) -> (W,0) -> (W,H) -> (0,H)`。"""
    o, E1, E2 = frame['origin'], frame['E1'], frame['E2']
    ab = np.array([[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]], np.float64)
    return o[None, :] + ab[:, 0:1] * E1[None, :] + ab[:, 1:2] * E2[None, :]
