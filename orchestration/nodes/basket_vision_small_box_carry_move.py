# -*- coding: utf-8 -*-
"""40 cm 小箱拆垛的左右手先抬微调动作。"""

from dataclasses import replace
from math import isfinite

from orchestration.nodes.basket_vision_carry_move import BasketVisionCarryMove
from orchestration.nodes.basket_vision_right_carry_move import BasketVisionRightCarryMove
from orchestration.utils.manifest_decorators import define_manifest


SMALL_BOX_WIDTH_M = 0.40
SMALL_BOX_GRASP_MODES = frozenset({
    "small_box_left_lift_right_carry",
    "small_box_right_lift_left_carry",
})

# 右手先抬流程的 40 cm 箱子实机标定值。未列出的轨迹点保持 build_plan() 的原值。
#
# 这组值来自右手先抬的实际调试，不能直接作为左手先抬的标定值复用。
SMALL_BOX_RIGHT_FIRST_TUNE = {
    "right_out": [0.00, 0.10, 0.00, 0, 0, 0],
    "right_grasp": [0.00, 0.10, -0.04, 0, 0, 0],
    "right_up": [0.00, 0.10, -0.05, 0, 0, 0],
    "right_pull": [0.00, 0.11, -0.05, 0, 0, 0],
    "left_out": [0.00, 0.09, 0.00, 0, 0, 0],
    "left_out2": [0.00, 0.09, 0.00, 0, 0, 0],
    "left_grasp": [-0.01, 0.09, 0.00, 0, 0, 0],
    "chest_up_left": [0.00, 0.13, 0.00, 0, 0, 0],
    "chest_up_right": [0.00, 0.13, 0.00, 0, 0, 0],
    "chest_left": [0.00, 0.13, 0.00, 0, 0, 0],
    "chest_right": [0.00, 0.13, 0.00, 0, 0, 0],
}

# 左手先抬流程的独立微调输入。
#
# 当前只写入已经针对左臂进场/抓取和双臂保持位确认过的数值；左臂的
# left_up / left_pull 没有沿用右臂的 right_up / right_pull 标定量，而是保持
# build_plan() 原值，等待单独标定后再填入本表。
SMALL_BOX_LEFT_FIRST_TUNE = {
    "left_out": [0.00, -0.10, 0.00, 0, 0, 0],
    "left_grasp": [0.00, -0.13, -0.05, 0, 0, 0],
    "left_up": [0.00, -0.13, -0.05, 0, 0, 0],
    "left_pull": [0.00, -0.11, -0.05, 0, 0, 0],
    "right_out": [0.00, -0.13, -0.05, 0, 0, 0],
    "right_out2": [0.00, -0.13, -0.05, 0, 0, 0],
    "right_grasp": [0.00, -0.07, 0.00, 0, 0, 0],
    "chest_up_left": [0.00, -0.13, 0.00, 0, 0, 0],
    "chest_up_right": [0.00, -0.13, 0.00, 0, 0, 0],
    "chest_left": [0.00, -0.13, 0.00, 0, 0, 0],
    "chest_right": [0.00, -0.13, 0.00, 0, 0, 0],
}


def _add_delta(pose, delta):
    if len(pose) != 6 or len(delta) != 6:
        raise ValueError("末端位姿和微调量都必须为 6D")
    result = [float(value) + float(offset) for value, offset in zip(pose, delta)]
    if not all(isfinite(value) for value in result):
        raise ValueError("40 cm 小箱微调后出现非法位姿: %s" % result)
    return result


def tuned_small_box_plan(plan, tune):
    """返回叠加指定微调后的新轨迹，不修改调用方传入的原计划。"""
    updates = {
        name: _add_delta(getattr(plan, name), delta)
        for name, delta in tune.items()
    }
    return replace(plan, **updates)


@define_manifest(
    label="basket_vision 40cm 左手抬箱右手搬运",
    category=["perception", "motion", "arm"],
    tree_type="depalletize_bin",
    description="40cm 小箱：叠加左手先抬实机微调后，右手加入并保持导航位",
    params=[], inputs=[], outputs=[],
)
class BasketVisionSmallBoxLeftCarryMove(BasketVisionCarryMove):
    """40 cm 小箱的左手先抬、右手加入动作。"""

    @staticmethod
    def _execute(plan, hardware):
        BasketVisionCarryMove._execute(
            tuned_small_box_plan(plan, SMALL_BOX_LEFT_FIRST_TUNE), hardware
        )


@define_manifest(
    label="basket_vision 40cm 右手抬箱左手搬运",
    category=["perception", "motion", "arm"],
    tree_type="depalletize_bin",
    description="40cm 小箱：叠加右手先抬实机微调后，左手加入并保持导航位",
    params=[], inputs=[], outputs=[],
)
class BasketVisionSmallBoxRightCarryMove(BasketVisionRightCarryMove):
    """40 cm 小箱的右手先抬、左手加入动作。"""

    @staticmethod
    def _execute(plan, hardware):
        BasketVisionRightCarryMove._execute(
            tuned_small_box_plan(plan, SMALL_BOX_RIGHT_FIRST_TUNE), hardware
        )
