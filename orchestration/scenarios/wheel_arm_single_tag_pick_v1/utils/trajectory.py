# -*- coding: utf-8 -*-
"""wheel_arm_single_tag_pick_v1 的轨迹契约工具。"""

from __future__ import annotations

import math
from typing import List


JOINTS_PER_ARM = 7


def validate_arm_trajectory(trajectory, name: str) -> List[List[float]]:
    """校验并复制一侧至少两帧的 N×7 关节轨迹。"""
    if not isinstance(trajectory, (list, tuple)) or len(trajectory) < 2:
        raise ValueError(f"{name} must contain at least two N×7 frames")

    # 裸 [7] 轨迹必须拒绝，避免把单帧误解释成七帧。
    if any(not isinstance(frame, (list, tuple)) for frame in trajectory):
        raise ValueError(f"{name} must be a list of 7-value frames")

    checked = []
    for index, frame in enumerate(trajectory):
        if len(frame) != JOINTS_PER_ARM:
            raise ValueError(f"{name}[{index}] must contain 7 joints")
        values = [float(value) for value in frame]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name}[{index}] contains a non-finite joint")
        checked.append(values)
    return checked


def merge_bimanual_trajectory(left, right) -> List[List[float]]:
    """将左右 N×7 轨迹按帧合并成 SDK 所需的 N×14。"""
    left_checked = validate_arm_trajectory(left, "left_arm_joint_traj")
    right_checked = validate_arm_trajectory(right, "right_arm_joint_traj")
    if len(left_checked) != len(right_checked):
        raise ValueError("left and right trajectories must have equal length")
    return [list(left_frame) + list(right_frame)
            for left_frame, right_frame in zip(left_checked, right_checked)]


def validate_bimanual_14d_trajectory(trajectory) -> List[List[float]]:
    """校验并复制 N×14 双臂合并轨迹（前 7 维左臂，后 7 维右臂）。"""
    if not isinstance(trajectory, (list, tuple)) or len(trajectory) < 2:
        raise ValueError("bimanual trajectory must contain at least two N×14 frames")

    if any(not isinstance(frame, (list, tuple)) for frame in trajectory):
        raise ValueError("bimanual trajectory must be a list of 14-value frames")

    checked = []
    for index, frame in enumerate(trajectory):
        if len(frame) != 2 * JOINTS_PER_ARM:
            raise ValueError(f"bimanual trajectory[{index}] must contain 14 joints")
        values = [float(value) for value in frame]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"bimanual trajectory[{index}] contains a non-finite joint")
        checked.append(values)
    return checked


