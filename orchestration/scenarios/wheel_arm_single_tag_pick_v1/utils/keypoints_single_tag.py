# -*- coding: utf-8 -*-
"""单 Tag 场景的抓取关键点；不依赖 Kuavo SDK。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class SingleTagPose:
    """供场景 IK 适配层使用的不可变 BASE 系位姿 DTO。"""

    pos: Tuple[float, float, float]
    quat: Tuple[float, float, float, float]
    frame: str = "base_link"


def _pose_from_euler(pos, euler) -> SingleTagPose:
    quaternion = Rotation.from_euler("xyz", euler, degrees=True).as_quat()
    return SingleTagPose(
        pos=tuple(float(value) for value in pos),
        quat=tuple(float(value) for value in quaternion),
    )


def generate_pick_keypoints(
    box_width: float,
    box_behind_tag: float,
    box_beneath_tag: float,
    box_left_tag: float,
    hand_pitch_degree: float = 0.0,
) -> Tuple[List[SingleTagPose], List[SingleTagPose]]:
    """生成源脚本内嵌函数实际使用的四帧 BASE 系关键点。

    ``case_wheel_test_arm.py`` 的该版本没有使用除 ``box_width`` 外的
    参数，也没有应用 ``hand_pitch_degree``；这里保留参数仅为兼容调用签名。
    关键点不是 Tag 相对坐标，Tag 在源流程中只负责版本门禁。
    """
    del box_behind_tag, box_beneath_tag, box_left_tag, hand_pitch_degree
    left = [
        _pose_from_euler((0.3, box_width * 4 / 2, 0.1), (0, -90, 0)),
        _pose_from_euler((0.5, box_width * 3 / 2, 0.2), (0, -90, 0)),
        _pose_from_euler((0.5, box_width / 2, 0.2), (0, -90, 0)),
        _pose_from_euler((0.5, box_width / 2, 0.4), (0, -90, 0)),
    ]
    right = [
        _pose_from_euler((0.3, -box_width * 4 / 2, 0.1), (0, -90, 0)),
        _pose_from_euler((0.5, -box_width * 3 / 2, 0.2), (0, -90, 0)),
        _pose_from_euler((0.5, -box_width / 2, 0.2), (0, -90, 0)),
        _pose_from_euler((0.5, -box_width / 2, 0.4), (0, -90, 0)),
    ]
    return left, right
