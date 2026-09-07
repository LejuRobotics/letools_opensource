# -*- coding: utf-8 -*-
"""视觉定位箱体，双手同步夹持、抬起并搬到导航保持位。"""

from orchestration.nodes.basket_vision_carry_move import (
    BasketVisionCarryMove,
    _require_success,
)
from orchestration.utils.manifest_decorators import define_manifest
from core.domain.enums import ArmSide
from core.domain.end_effector import SG100HandCommand, SG100_JOINT_COUNT
SG100_OPEN = [0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SG100_OPENN = [0.0, -2.0, 0.0, 0.0, 0.0,3.0, 0.0, 3.0, 0.0, 0.0, 3.0]
SG100_HALF = [1.5, -3.0, 0.0, 1.5, 1.5, 1.3, 1.5, 1.3, 0.0, 1.5, 1.3]
SG100_CLOSE = [1.5, -2.0, 1.5, 1.5, 1.5, 3.0, 1.5, 3.0, 0.0, 1.5, 3.0]

@define_manifest(
    label="basket_vision 双手同步抬箱搬运",
    category=["perception", "motion", "arm"],
    tree_type="depalletize_bin",
    description="视觉定位后双手同步接近、夹持、抬升并保持箱体供底盘导航",
    params=[], inputs=[], outputs=[],
)
class BasketVisionDualCarryMove(BasketVisionCarryMove):
    """所有关键阶段均通过双臂同步接口发送，避免两侧启动时间不同。"""

    @staticmethod
    def _execute(plan, hardware):
        import time
                # 每个动作的微调：
        # [dx, dy, dz, droll, dpitch, dyaw]
        # xyz 单位：米；rpy 单位：度
        TUNE = {
            "left_out":    [0.00, -0.05, 0.08, 0, 0, 0],
            "right_out":   [0.00, -0.25, -0.07, 0, 0, 0],

            "left_grasp":  [0.00, 0.00, 0.03, 0, 0, 0],
            "right_grasp": [0.00, -0.20, -0.04, 0, 0, 0],

            "left_up":     [0.00, 0.00, 0.03, 0, 0, 0],
            "right_up":    [-0.05, 0.18, 0.03, 0, 0, 0],

            "left_pull":   [0.00, 0.00, 0.00, 0, 0, 0],
            "right_pull":  [0.00, 0.00, 0.00, 0, 0, 0],

            "chest_up_left":  [0.00, -0.20, 0.00, 0, 0, 0],
            "chest_up_right": [0.00, -0.20, 0.00, 0, 0, 0],

            # 最后一下“搬到导航保持位”在这里调
            "chest_left":  [0.00, -0.2, 0.00, 0, 0, 0],
            "chest_right": [0.00, -0.2, 0.00, 0, 0, 0],
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

        left_out = tuned(plan.left_out, "left_out")
        right_out = tuned(plan.right_out, "right_out")
        left_grasp = tuned(plan.left_grasp, "left_grasp")
        right_grasp = tuned(plan.right_grasp, "right_grasp")
        left_up = tuned(plan.left_up, "left_up")
        right_up = tuned(plan.right_up, "right_up")
        left_pull = tuned(plan.left_pull, "left_pull")
        right_pull = tuned(plan.right_pull, "right_pull")
        chest_up_left = tuned(plan.chest_up_left, "chest_up_left")
        chest_up_right = tuned(plan.chest_up_right, "chest_up_right")
        chest_left = tuned(plan.chest_left, "chest_left")
        chest_right = tuned(plan.chest_right, "chest_right")
        # 1. 接近箱子前，先张开双手
        _require_success(
            hardware.control_end_effector(
                ArmSide.LEFT, SG100HandCommand(positions=SG100_OPENN)
            ),
            "左SG100手张开",
        )
        _require_success(
            hardware.control_end_effector(
                ArmSide.LEFT, SG100HandCommand(positions=SG100_HALF)
            ),
            "左SG100手半握",
        )
        time.sleep(1.0)
        _require_success(
            hardware.control_end_effector(
                ArmSide.RIGHT, SG100HandCommand(positions=SG100_HALF)
            ),
            "右SG100手半握",
        )
        # _require_success(
        #     hardware.control_end_effector(
        #         ArmSide.RIGHT, SG100HandCommand(positions=SG100_OPENN)
        #     ),
        #     "右SG100手张开",
        # )
        time.sleep(1.0)

        # 2. 双臂同步绕行、到达抓取位
        approach_steps = (
            (left_out, right_out, 3.0, "双臂同步绕行"),
            (left_grasp, right_grasp, 3.0, "双臂同步抓取"),
        )
        for left, right, duration, label in approach_steps:
            _require_success(
                hardware.send_arm_ee_local_timed(
                    left, right, desire_time=duration
                ),
                label,
            )
            time.sleep(duration + 0.5)

        
        # 3. 两只手均到抓取位后，先半握、再夹紧
        
        # time.sleep(1.0)

        

        # 4. 已夹紧后才允许提起、拉出、抱到胸前
        carry_steps = (
            (left_up, right_up, 2.0, "双臂同步提起"),
            # (plan.left_pull, plan.right_pull, 7.0, "双臂同步拉出"),
            # (chest_up_left, chest_up_right, 5.0, "双臂同步抬至胸前"),
            # (chest_left, chest_right, 8.0, "搬到导航保持位"),
        )
        for left, right, duration, label in carry_steps:
            method = (
                hardware.send_dual_arm_whole_body_ik_timed
                if getattr(plan, "use_whole_body_ik", False)
                else hardware.send_arm_ee_local_timed
            )
            _require_success(
                method(left, right, desire_time=duration),
                label,
            )
            time.sleep(duration + 1.0)
        # _require_success(
        #     hardware.control_end_effector(
        #         ArmSide.LEFT, SG100HandCommand(positions=SG100_CLOSE)
        #     ),
        #     "左SG100手夹紧",
        # )
        # time.sleep(1.0)
        # _require_success(
        #     hardware.control_end_effector(
        #         ArmSide.RIGHT, SG100HandCommand(positions=SG100_CLOSE)
        #     ),
        #     "右SG100手夹紧",
        # )
        # time.sleep(1.0)