# -*- coding: utf-8 -*-
"""到达目的导航点后放下箱体、撤离并复位。"""

import os
import rospy
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from core.domain.enums import ArmSide
from core.domain.end_effector import SG100HandCommand, SG100_JOINT_COUNT
from orchestration.nodes.basket_vision_carry_move import _require_success
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

SG100_OPEN = [0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SG100_OPENN = [0.0, -2.0, 0.0, 0.0, 0.0,3.0, 0.0, 3.0, 0.0, 0.0, 3.0]
@define_manifest(
    label="导航到达后放下箱体",
    category=["motion", "arm"],
    tree_type="depalletize_bin",
    description="箱体保持导航到目的地后，执行放下、外扩和复位",
    params=[], inputs=[], outputs=[],
)
class BasketPlaceAfterNavMove(BaseAction):
    """独立放置节点；必须放在目的地到达检查之后。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._status = None

    def initialise(self):
        self._status = None

    def update(self):
        if self._status is not None:
            return self._status
        if _DRY_RUN:
            self.feedback_message = "dry-run place after nav"
            self._status = Status.SUCCESS
            return self._status
        if not bool(self.params.get("execute_place", False)):
            self.feedback_message = "execute_place=false，未发送放置指令"
            self._status = Status.FAILURE
            return self._status
        try:
            self._execute(get_shared_hardware())
            self.feedback_message = "目的地放置与复位完成"
            self._status = Status.SUCCESS
        except Exception as exc:
            self.feedback_message = "目的地放置失败: %s" % exc
            self._status = Status.FAILURE
        return self._status

    def _pose(self, key, default):
        pose = list(self.params.get(key, default))
        if len(pose) != 6:
            raise ValueError("%s 必须是 6 维末端位姿" % key)
        return pose

    def _execute(self, hardware, box_width=None):
        import time

        # 单箱测试：读取 board 顶层 box_width
        # 全流程：由当前点位的 action["box_width"] 传进来
        width = float(
            self.params.get("box_width", 0.70)
            if box_width is None else box_width
        )
        if abs(width - 0.40) < 0.01:
            table_left = [0.85, 0.29, 0.63, 0, -80, 0]
            table_right = [0.80, -0.23, 0.50, 0, -95, 0]
            expand_left = [0.95, 0.36, 0.60, 0, -80, 0]
            expand_right = [0.90, -0.36, 0.60, 0, -95, 0]
            reset_left = [0.35, 0.25, 0.72, 0, -1, 0]
            reset_right = [0.35, -0.25, 0.72, 0, -1, 0]
        elif abs(width - 0.50) < 0.01:
            # 50 cm 小箱子放置动作
            table_left = [0.85, 0.35, 0.63, 0, -80, 0]
            table_right = [0.80, -0.29, 0.50, 0, -95, 0]
            expand_left = [0.95, 0.43, 0.60, 0, -80, 0]
            expand_right = [0.90, -0.43, 0.60, 0, -95, 0]
            reset_left = [0.35, 0.25, 0.72, 0, -1, 0]
            reset_right = [0.35, -0.25, 0.72, 0, -1, 0]
        elif abs(width - 0.70) < 0.01:
            # 70 cm 大箱子放置动作：保留你当前已调好的数值
            table_left = [0.85, 0.47, 0.63, 0, -80, 0]
            table_right = [0.80, -0.40, 0.50, 0, -95, 0]
            expand_left = [0.95, 0.60, 0.60, 0, -80, 0]
            expand_right = [0.90, -0.60, 0.60, 0, -95, 0]
            reset_left = [0.35, 0.25, 0.72, 0, -1, 0]
            reset_right = [0.35, -0.25, 0.72, 0, -1, 0]
        else:
            raise ValueError(
                "放置动作仅支持 0.50m / 0.70m 箱子，当前宽度为 %.3fm" % width
            )

        rospy.loginfo("  放置箱体宽度: %.2fm", width)

        _require_success(
            hardware.send_arm_ee_local_timed(
                table_left, table_right, desire_time=5.0
            ),
            "双臂放到目的点",
        )
        time.sleep(6.0)

        # 先按当前张开姿态松手
        _require_success(
            hardware.control_end_effector(
                ArmSide.LEFT, SG100HandCommand(positions=SG100_OPENN)
            ),
            "左SG100手松开",
        )
        _require_success(
            hardware.control_end_effector(
                ArmSide.RIGHT, SG100HandCommand(positions=SG100_OPENN)
            ),
            "右SG100手松开",
        )
        time.sleep(2.0)

        # 再到完全张开，确认手指已脱离箱体
        _require_success(
            hardware.control_end_effector(
                ArmSide.LEFT, SG100HandCommand(positions=SG100_OPEN)
            ),
            "左SG100手完全张开",
        )
        _require_success(
            hardware.control_end_effector(
                ArmSide.RIGHT, SG100HandCommand(positions=SG100_OPEN)
            ),
            "右SG100手完全张开",
        )
        time.sleep(2.0)

        _require_success(
            hardware.send_arm_ee_local_timed(
                expand_left, expand_right, desire_time=4.0
            ),
            "双臂安全撤离",
        )
        time.sleep(5.0)

        _require_success(
            hardware.send_arm_ee_local_timed(
                reset_left, reset_right, desire_time=5.0
            ),
            "双臂回到下一箱抓取准备位",
        )
        time.sleep(6.0)
        rospy.loginfo("  放置与复位完成")
