# -*- coding: utf-8 -*-
"""箱高修正（`correct_box_for_plane_height`）的几何回归。

场景刻意**不正对相机**：相机有 15° 俯仰 + 20° 偏航，箱子在托盘面内再转 30°。
正视（R=I、台面法向与光轴反向）时 `plane_d <= 0`，修正会走"跳过"分支 ——
拿那种相机测等于什么都没测。
"""
import math

import numpy as np
import pytest

from skills.atomic.perception.pallet_servo_v2.algorithm import (
    BoxObservation,
    correct_box_for_plane_height,
)

pytestmark = pytest.mark.unit

FX = FY = 1000.0
CX, CY = 640.0, 400.0
K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])

W_MM, H_MM = 1200.0, 800.0
TILT_DEG = 15.0
YAW_DEG = 20.0
DIST_M = 1.0


def tilted_camera(tilt_deg=TILT_DEG, yaw_deg=YAW_DEG, dist_m=DIST_M) -> np.ndarray:
    """相机在托盘中心上方 `dist_m`，先偏航再俯仰；返回 `T_cam_pallet`。"""
    yaw = math.radians(yaw_deg)
    tilt = math.radians(tilt_deg)
    # 相机轴在托盘（世界）系下的方向
    x_cam = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    z0 = np.array([0.0, 0.0, -1.0])          # 正视时相机 +z（朝下）
    z_cam = (math.cos(tilt) * z0
             + math.sin(tilt) * np.cross(x_cam, z0))
    z_cam = z_cam / np.linalg.norm(z_cam)
    y_cam = np.cross(z_cam, x_cam)           # 右手系
    # 世界 -> 相机 的旋转（行 = 相机轴在世界系）
    R = np.vstack([x_cam, y_cam, z_cam])
    center = np.array([W_MM, H_MM, 0.0]) * 0.5
    C = center - z_cam * (dist_m * 1000.0)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ (C * 0.001)
    return T


def project(T, points_mm):
    """与算法里的无畸变路径同一套投影（毫米进、像素出）。"""
    P = np.asarray(points_mm, np.float64) * 0.001
    pc = P @ np.asarray(T, np.float64)[:3, :3].T + np.asarray(T, np.float64)[:3, 3]
    xy = pc[:, :2] / pc[:, 2:3]
    return (K @ np.hstack([xy, np.ones((len(xy), 1))]).T).T[:, :2]


def box_quad(height_mm=250.0, theta_deg=30.0, half_mm=150.0):
    """托盘中心一个方箱的**顶面**四角（pallet 系毫米），面内转 `theta_deg`。"""
    th = math.radians(theta_deg)
    c, s = math.cos(th), math.sin(th)
    center = np.array([W_MM, H_MM]) * 0.5
    local = np.array([[+1, +1], [-1, +1], [-1, -1], [+1, -1]]) * half_mm
    xy = local @ np.array([[c, s], [-s, c]]).T + center
    return np.column_stack([xy, np.full(4, height_mm)])


def test_tilted_yawed_camera_matches_ground_projection():
    """非正视 + 面内旋转：修正后四角必须等于"同一箱子贴台面放"的投影。"""
    T = tilted_camera()
    top = box_quad()
    quad_px = project(T, top)
    obs = BoxObservation(quad=quad_px.tolist(), label="box", confidence=0.9)

    diag = {}
    out = correct_box_for_plane_height(
        obs, T, K, None, 250.0, diag=diag)

    foot = top.copy()
    foot[:, 2] = 0.0
    want = project(T, foot)
    err = np.abs(np.asarray(out.quad) - want).max()
    assert diag["applied"] is True, diag
    assert diag["why"] == "ok", diag
    assert err < 1e-6, f"修正后四角与台面投影差 {err}px"
    # 而且这一步**确实动了**：正视时这条会退化，非正视时位移是几十像素
    shift = np.linalg.norm(np.asarray(out.quad) - quad_px, axis=1).max()
    assert shift > 10.0, f"箱高修正只动了 {shift}px，测试场景没有覆盖透视差"


def test_height_zero_is_a_noop():
    T = tilted_camera()
    obs = BoxObservation(quad=project(T, box_quad()).tolist())
    diag = {}
    out = correct_box_for_plane_height(obs, T, K, None, 0.0, diag=diag)
    assert out is obs
    assert diag == {"applied": False, "why": "height<=0", "height_mm": 0.0}


def test_no_quad_is_skipped_with_reason():
    T = tilted_camera()
    obs = BoxObservation(u1=1.0, v1=2.0, u2=3.0, v2=4.0)
    diag = {}
    out = correct_box_for_plane_height(obs, T, K, None, 250.0, diag=diag)
    assert out is obs
    assert diag["applied"] is False
    assert diag["why"] == "no_quad", diag


def test_camera_below_pallet_is_skipped():
    """台面在相机背后时不许产生一个镜像四角 —— 原样返回并说明原因。"""
    T = tilted_camera()
    T[2, 3] = -1.0
    obs = BoxObservation(quad=project(tilted_camera(), box_quad()).tolist())
    diag = {}
    out = correct_box_for_plane_height(obs, T, K, None, 250.0, diag=diag)
    assert out is obs
    assert diag["applied"] is False
    assert diag["why"] == "plane_behind_camera", diag
