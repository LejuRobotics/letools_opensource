# -*- coding: utf-8 -*-
"""keypoints 工具函数单元测试。"""
import math
import pytest
from orchestration.nodes.utils.keypoints import generate_pick_keypoints, generate_lift_keypoints


def test_generate_pick_keypoints_returns_4_frames_per_arm():
    """源版有 4 帧(预抓取/预抓取/并拢/收臂),左/右臂各 4 帧。"""
    left, right = generate_pick_keypoints(
        box_width=0.35, box_behind_tag=0.0, box_beneath_tag=0.0, box_left_tag=0.0,
    )
    assert len(left) == 4, f"expected 4 frames, got {len(left)}"
    assert len(right) == 4


def test_generate_pick_keypoints_hand_pitch_degree():
    """hand_pitch_degree 应影响末端四元数。"""
    left_default, _ = generate_pick_keypoints(0.35, 0.0, 0.0, 0.0, hand_pitch_degree=0.0)
    left_pitched, _ = generate_pick_keypoints(0.35, 0.0, 0.0, 0.0, hand_pitch_degree=30.0)
    # pitch 变化 → quat 必然不同
    import numpy as np
    assert not np.array_equal(left_default[0].quat, left_pitched[0].quat)


def test_generate_lift_keypoints_returns_1_frame_per_arm():
    left, right = generate_lift_keypoints(
        box_width=0.35, box_behind_tag=0.0, box_beneath_tag=0.0, box_left_tag=0.0,
        z_lift=0.2,
    )
    assert len(left) == 1
    assert len(right) == 1


def test_generate_lift_keypoints_z_lifted_by_0p2m():
    """抬升关键点 z 坐标应比抓取收臂位姿最后一帧高 0.2m。"""
    pick_left, _ = generate_pick_keypoints(0.35, 0.0, 0.0, 0.0)
    lift_left, _ = generate_lift_keypoints(0.35, 0.0, 0.0, 0.0, z_lift=0.2)
    z_diff = lift_left[0].pos[2] - pick_left[-1].pos[2]
    assert math.isclose(z_diff, 0.2, abs_tol=1e-6)
