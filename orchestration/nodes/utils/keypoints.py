# -*- coding: utf-8 -*-
"""keypoint 工具函数:从 case_wheel_pick_and_place.py 移植并扩展。

- generate_pick_keypoints:抓取位姿序列(4 帧:预抓取 / 预抓取 / 并拢 / 收臂)
- generate_lift_keypoints:抬升位姿(1 帧:在收臂位姿基础上 +z_lift)

源版来源:drivers/leju/kuavo_humanoid_sdk/.../pick_place_box/case_wheel_pick_and_place.py:54-99
本模块独立放置在 orchestration/nodes/utils/ 下,避免对 SDK 子模块的循环依赖。
"""
from typing import List, Tuple
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Frame


def generate_pick_keypoints(
    box_width: float,
    box_behind_tag: float,
    box_beneath_tag: float,
    box_left_tag: float,
    hand_pitch_degree: float = 0.0,
) -> Tuple[List[Pose], List[Pose]]:
    """算抓取位姿关键点(BASE 系相对偏移)。

    Args:
        box_width: 箱子宽度(米)
        box_behind_tag: 箱子在 tag 后方距离(米,源版保留参数暂未使用)
        box_beneath_tag: 箱子在 tag 下方距离(米,源版保留参数暂未使用)
        box_left_tag: 箱子在 tag 左侧距离(米)
        hand_pitch_degree: 抓取手臂 pitch(度,相比水平,下倾是正)

    Returns:
        (left_poses, right_poses):左右臂各 4 帧关键点(Frame.BASE)
    """
    pick_left_arm_poses = [
        # 1. 预抓取点位
        Pose.from_euler(pos=(0.4, box_width * 3 / 2 - box_left_tag, 0.0),
                        euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True, frame=Frame.BASE),
        # 1'. 预抓取点位(更近)
        Pose.from_euler(pos=(0.5, box_width * 3 / 2 - box_left_tag, 0.1),
                        euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True, frame=Frame.BASE),
        # 2. 并拢点位
        Pose.from_euler(pos=(0.6, box_width / 2 - box_left_tag, 0.1),
                        euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True, frame=Frame.BASE),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.7, box_width / 2, 0.2),
                        euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True, frame=Frame.BASE),
    ]

    pick_right_arm_poses = [
        # 1. 预抓取点位
        Pose.from_euler(pos=(0.4, -box_width * 3 / 2 - box_left_tag, 0.0),
                        euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True, frame=Frame.BASE),
        # 1'. 预抓取点位(更近)
        Pose.from_euler(pos=(0.5, -box_width * 3 / 2 - box_left_tag, 0.1),
                        euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True, frame=Frame.BASE),
        # 2. 并拢点位
        Pose.from_euler(pos=(0.6, -box_width / 2 - box_left_tag, 0.1),
                        euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True, frame=Frame.BASE),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.7, -box_width / 2, 0.2),
                        euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True, frame=Frame.BASE),
    ]

    return pick_left_arm_poses, pick_right_arm_poses


def generate_lift_keypoints(
    box_width: float,
    box_behind_tag: float,
    box_beneath_tag: float,
    box_left_tag: float,
    z_lift: float = 0.1,
) -> Tuple[List[Pose], List[Pose]]:
    """算抬升位姿关键点(基于抓取收臂位姿最后一帧 + z_lift)。

    Returns:
        (left_poses, right_poses):左右臂各 1 帧关键点
    """
    pick_left, pick_right = generate_pick_keypoints(
        box_width, box_behind_tag, box_beneath_tag, box_left_tag,
    )
    # 收臂位姿(pick 最后 1 帧)z + z_lift
    lifted_left = [
        Pose(
            pos=(p.pos[0], p.pos[1], p.pos[2] + z_lift),
            quat=p.quat, frame=p.frame,
        )
        for p in pick_left[-1:]
    ]
    lifted_right = [
        Pose(
            pos=(p.pos[0], p.pos[1], p.pos[2] + z_lift),
            quat=p.quat, frame=p.frame,
        )
        for p in pick_right[-1:]
    ]
    return lifted_left, lifted_right
