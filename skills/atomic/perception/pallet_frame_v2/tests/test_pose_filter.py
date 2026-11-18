# -*- coding: utf-8 -*-
"""托盘位姿卡尔曼滤波的单元测试（纯函数，无 ROS）。"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from skills.atomic.perception.pallet_frame_v2.pose_filter import PoseKalmanFilter

pytestmark = pytest.mark.unit


def pose_xyz(x=0.0, y=0.0, z=1.0, yaw_deg=0.0):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def test_first_frame_is_passthrough():
    f = PoseKalmanFilter(window=5)
    T = pose_xyz(0.1, -0.2, 1.3, 12.0)
    out = f.update(T, 10.0)
    assert np.allclose(out, T)


def test_single_frame_jump_is_suppressed():
    f = PoseKalmanFilter(window=5)
    for i in range(5):
        out = f.update(pose_xyz(0.0, 0.0, 1.0), i * 0.05)
    jump = pose_xyz(1.5, 0.8, 0.4)          # 1.8m 单帧跳变
    out = f.update(jump, 5 * 0.05)
    assert abs(out[0, 3]) < 0.2
    assert abs(out[1, 3]) < 0.2
    # 下一帧回到原目标：滤波器没被单帧坏观测带走
    out = f.update(pose_xyz(0.0, 0.0, 1.0), 6 * 0.05)
    assert abs(out[0, 3]) < 0.1


def test_persistent_jump_recovers_after_window_frames():
    f = PoseKalmanFilter(window=5)
    for i in range(5):
        f.update(pose_xyz(0.0, 0.0, 1.0), i * 0.05)
    for i in range(5, 10):
        out = f.update(pose_xyz(1.5, 0.8, 0.4), i * 0.05)
    # 第 window 个连续异常帧触发重新捕获
    assert np.allclose(out[:3, 3], [1.5, 0.8, 0.4], atol=1e-6)


def test_rotation_jump_is_suppressed():
    f = PoseKalmanFilter(window=5)
    for i in range(5):
        out = f.update(pose_xyz(yaw_deg=0.0), i * 0.05)
    out = f.update(pose_xyz(yaw_deg=40.0), 5 * 0.05)
    angle = np.degrees(np.linalg.norm(Rotation.from_matrix(out[:3, :3]).as_rotvec()))
    assert angle < 5.0
    out = f.update(pose_xyz(yaw_deg=0.0), 6 * 0.05)
    angle = np.degrees(np.linalg.norm(Rotation.from_matrix(out[:3, :3]).as_rotvec()))
    assert angle < 5.0


def test_time_gap_resets():
    f = PoseKalmanFilter(window=5)
    f.update(pose_xyz(0.0, 0.0, 1.0), 1.0)
    out = f.update(pose_xyz(2.0, 0.0, 1.0), 3.0)   # 2s 空档
    assert np.allclose(out[:3, 3], [2.0, 0.0, 1.0])


def test_same_stamp_does_not_double_update():
    f = PoseKalmanFilter(window=5)
    f.update(pose_xyz(0.0, 0.0, 1.0), 1.0)
    out = f.update(pose_xyz(1.0, 0.0, 1.0), 1.0)
    assert np.allclose(out[:3, 3], [0.0, 0.0, 1.0])


def test_clock_rewind_resets():
    f = PoseKalmanFilter(window=5)
    f.update(pose_xyz(0.0, 0.0, 1.0), 10.0)
    out = f.update(pose_xyz(2.0, 0.0, 1.0), 1.0)
    assert np.allclose(out[:3, 3], [2.0, 0.0, 1.0])
