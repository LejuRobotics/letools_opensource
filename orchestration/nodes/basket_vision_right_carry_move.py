# -*- coding: utf-8 -*-
"""视觉定位箱体，右手先抬、左手加入并搬到导航保持位。"""

from orchestration.nodes.basket_vision_carry_move import (
    BasketVisionCarryMove,
    _require_success,
)
from orchestration.utils.manifest_decorators import define_manifest
from core.domain.enums import ArmSide
from core.domain.end_effector import SG100HandCommand
SG100_OPEN = [0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SG100_OPENN = [0.0, -2.0, 0.0, 0.0, 0.0,3.0, 0.0, 3.0, 0.0, 0.0, 3.0]
SG100_HALF = [1.5, -2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 3.0]
SG100_CLOSE = [1.5, -1.0, 2.7, 0.0, 0.0, 3.0, 0.0, 3.0, 0.0, 0.0, 3.0]

@define_manifest(
    label="basket_vision 右手抬箱左手搬运",
    category=["perception", "motion", "arm"],
    tree_type="depalletize_bin",
    description="视觉定位后右手抬箱、左手加入，最终保持箱体供底盘导航",
    params=[], inputs=[], outputs=[],
)
class BasketVisionRightCarryMove(BasketVisionCarryMove):
    """与现有左手先抬流程对称的右手先抬流程。"""

    @staticmethod
    def _execute(plan, hardware):
        import time
        TUNE = {
            "right_out":    [0.00, -0.27, 0.03, 0, 0, 0],
            "right_grasp":  [0.00, -0.16, 0.03, 0, 0, 0],
            "right_up":     [0.00, 0.00, -0.15, 0, 0, 0],
            "right_pull":   [0.00, -0.20, -0.17, 0, 0, 0],

            "left_out":     [0.00, -0.25, 0.00, 0, 0, 0],
            "left_grasp":   [0.00, -0.25, 0.01, 0, 0, 0],

            "chest_up_left":  [0.00, 0.00, 0.00, 0, 0, 0],
            "chest_up_right": [0.00, 0.00, 0.00, 0, 0, 0],

            "chest_left":   [0.00, 0.00, 0.00, 0, 0, 0],
            "chest_right":  [0.00, 0.00, 0.00, 0, 0, 0],
        }

        def tuned(pose, name):
            delta = TUNE[name]
            return [
                pose[0] + delta[0],
                pose[1] + delta[1],
                pose[2] + delta[2],
                pose[3] + delta[3],
                pose[4] + delta[4],
                pose[5] + delta[5],
            ]

        right_out = tuned(plan.right_out, "right_out")
        right_grasp = tuned(plan.right_grasp, "right_grasp")
        right_up = tuned(plan.right_up, "right_up")
        right_pull = tuned(plan.right_pull, "right_pull")

        left_out = tuned(plan.left_out, "left_out")
        left_grasp = tuned(plan.left_grasp, "left_grasp")

        chest_up_left = tuned(plan.chest_up_left, "chest_up_left")
        chest_up_right = tuned(plan.chest_up_right, "chest_up_right")
        chest_left = tuned(plan.chest_left, "chest_left")
        chest_right = tuned(plan.chest_right, "chest_right")

        prefix_steps = (
            ("send_right_arm_ee_local_timed", (right_out,), 7.0, "右臂绕行"),
            ("send_right_arm_ee_local_timed", (right_grasp,), 7.0, "右臂抓取位"),
        )
        _require_success(hardware.control_end_effector(
                            ArmSide.RIGHT, SG100HandCommand(positions=SG100_OPEN)), "左SG100手松开")
        time.sleep(3.0)
        for method, poses, duration, label in prefix_steps:
            _require_success(getattr(hardware, method)(*poses, desire_time=duration), label)
            time.sleep(duration + 1.0)
        _require_success(hardware.control_end_effector(
                                    ArmSide.RIGHT, SG100HandCommand(positions=SG100_HALF)), "右SG100手夹紧")
        time.sleep(3.0)
        _require_success(hardware.control_end_effector(
                    ArmSide.RIGHT, SG100HandCommand(positions=SG100_CLOSE)), "右SG100手夹紧")
        time.sleep(3.0)
        if getattr(plan, "use_whole_body_ik", False):
            _require_success(hardware.send_single_arm_whole_body_ik_timed(
                False, right_up, desire_time=7.0), "右臂整身IK提起")
            time.sleep(8.0)
            _require_success(hardware.send_single_arm_whole_body_ik_timed(
                False, right_pull, desire_time=8.0), "右臂整身IK拉出")
            time.sleep(9.0)
        else:
            _require_success(hardware.send_right_arm_ee_local_timed(
                right_up, desire_time=7.0), "右臂提起")
            time.sleep(8.0)
            _require_success(hardware.send_right_arm_ee_local_timed(
                right_pull, desire_time=8.0), "右臂拉出")
            time.sleep(9.0)
        suffix_steps = (
            ("send_left_arm_ee_local_timed", (left_out,), 8.0, "左臂绕行"),
            ("send_left_arm_ee_local_timed", (left_grasp,), 7.0, "左臂抓取位"),
        )
        for method, poses, duration, label in suffix_steps:
            _require_success(
                getattr(hardware, method)(*poses, desire_time=duration), label
            )
            time.sleep(duration + 1.0)
        _require_success(hardware.control_end_effector(
                                    ArmSide.LEFT, SG100HandCommand(positions=SG100_OPENN)), "左SG100手夹紧")
        time.sleep(3.0)
        _require_success(hardware.control_end_effector(
                    ArmSide.LEFT, SG100HandCommand(positions=SG100_HALF)), "左SG100手夹紧")
        time.sleep(3.0)
        _require_success(hardware.control_end_effector(
                    ArmSide.LEFT, SG100HandCommand(positions=SG100_CLOSE)), "左SG100手夹紧")
        time.sleep(3.0)
        dual_method = (hardware.send_dual_arm_whole_body_ik_timed
                       if getattr(plan, "use_whole_body_ik", False)
                       else hardware.send_arm_ee_local_timed)
        _require_success(dual_method(chest_up_left, chest_up_right,
                                     desire_time=5.0), "双臂抬起")
        time.sleep(6.0)
        _require_success(dual_method(chest_left, chest_right,
                                     desire_time=8.0), "搬到导航保持位")
        time.sleep(9.0)