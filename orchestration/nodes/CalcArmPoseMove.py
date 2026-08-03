# -*- coding: utf-8 -*-
"""CalcArmPoseMove：计算搬箱抓取/放置的双臂末端关键点（tag 系），写黑板 ArmPoseAndWrench。

- 关键点在 TAG 坐标系下定义，由下游 MoveArmBaseTargetPoseMove 用 TargetTag 变换到 odom(世界)系
"""

import os

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

_ZERO_WRENCH = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def _keypoint(x, y, z, roll_deg, pitch_deg, yaw_deg):
    """tag 系关键点：[x, y, z, roll_deg, pitch_deg, yaw_deg]（角度为度）。"""
    return [x, y, z, roll_deg, pitch_deg, yaw_deg]


@define_manifest(
    label="计算搬箱手臂关键点",
    category=["manipulation", "calc"],
    tree_type="grasp_mtbf",
    description="计算搬箱抓取(box_grasp_step1)/放置(box_place)的双臂末端关键点(tag系)，写入黑板 ArmPoseAndWrench",
    params=[
        {"name": "mode", "type": "string", "default": "box_grasp_step1",
         "description": "计算模式", "options": ["box_grasp_step1", "box_place"]},
        {"name": "box_length", "type": "float", "default": "0.4", "description": "箱子长度(m)"},
        {"name": "box_width", "type": "float", "default": "0.3", "description": "箱子宽度(m)"},
        {"name": "box_height", "type": "float", "default": "0.25", "description": "箱子高度(m)"},
        {"name": "box_height_offset", "type": "float", "default": "0", "description": "抓取高度偏移(m)"},
        {"name": "box_pre_open", "type": "float", "default": "0.13", "description": "预抓取张开距离(m)"},
        {"name": "box_close_offset", "type": "float", "default": "0.03", "description": "并拢距离(m)"},
        {"name": "box_width_offset", "type": "float", "default": "0", "description": "宽度方向偏移(m)"},
        {"name": "box_pre_place", "type": "float", "default": "0", "description": "放置预张开距离(m)"},
    ],
    inputs=[],
    outputs=[
        {"name": "arm_pose_and_wrench", "type": "object", "default_key": "ArmPoseAndWrench",
         "description": "[(left_keypoints, right_keypoints), (left_wrench, right_wrench)]，关键点为 tag 系 [x,y,z,r,p,y(deg)]"},
    ],
)
class CalcArmPoseMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False
        self._success = False

    def initialise(self):
        self._done = False
        self._success = False

        try:
            self.global_blackboard.register_key(key="ArmPoseAndWrench", access=py_trees.common.Access.WRITE)
        except Exception:
            pass

        if _DRY_RUN:
            self.global_blackboard.set("ArmPoseAndWrench", None)
            self._done = True
            self._success = True
            return

        mode = str(self.params.get("mode", "box_grasp_step1"))
        box_length = float(self.params.get("box_length", 0.4))
        box_width = float(self.params.get("box_width", 0.3))
        box_height = float(self.params.get("box_height", 0.25))

        if mode == "box_grasp_step1":
            height_off = float(self.params.get("box_height_offset", 0))
            pre_open = float(self.params.get("box_pre_open", 0.13))
            close_off = float(self.params.get("box_close_offset", 0.03))
            width_off = float(self.params.get("box_width_offset", 0))

            half_l, half_w = box_length / 2, box_width / 2
            h = box_height + height_off
            w = -half_w - width_off

            left_kps = [
                _keypoint(-half_l - pre_open, h, w, 0, 0, 90),          # 1. 预抓取
                _keypoint(-half_l + close_off, h, w, 0, 0, 90),       # 2. 并拢
                _keypoint(-half_l + close_off, h, w + 0.2, 0, 0, 90), # 3. 抬起（箱厚方向 +0.2）
            ]
            right_kps = [
                _keypoint(half_l + pre_open, h, w, 0, 0, 90),
                _keypoint(half_l - close_off, h, w, 0, 0, 90),
                _keypoint(half_l - close_off, h, w + 0.2, 0, 0, 90),
            ]
            left_wrench = [list(_ZERO_WRENCH) for _ in left_kps]
            right_wrench = [list(_ZERO_WRENCH) for _ in right_kps]

        elif mode == "box_place":
            pre_place = float(self.params.get("box_pre_place", 0))
            half_l = box_length / 2

            left_kps = [
                _keypoint(-half_l - pre_place, -0.1, 0.13, 0, 0, 90),        # 1. 打开
                _keypoint(-half_l - pre_place - 0.1, -0.1, 0.13, 0, 0, 90),  # 2. 撤开
            ]
            right_kps = [
                _keypoint(half_l + pre_place, -0.1, 0.13, 0, 0, 90),
                _keypoint(half_l + pre_place + 0.1, -0.1, 0.13, 0, 0, 90),
            ]
            left_wrench = [list(_ZERO_WRENCH) for _ in left_kps]
            right_wrench = [list(_ZERO_WRENCH) for _ in right_kps]
        else:
            self.feedback_message = f"未知 mode: {mode}"
            self._done = True
            return

        self.global_blackboard.set("ArmPoseAndWrench",
                                   [(left_kps, right_kps), (left_wrench, right_wrench)])
        self.feedback_message = f"mode={mode} 关键点计算成功（左右各 {len(left_kps)} 个）"
        self._success = True
        self._done = True

    def update(self):
        if not self._done:
            return Status.RUNNING
        return Status.SUCCESS if self._success else Status.FAILURE
