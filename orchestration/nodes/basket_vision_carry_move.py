# -*- coding: utf-8 -*-
"""视觉定位箱体，左手先抬、右手加入并搬到导航保持位。"""

import os
from types import SimpleNamespace

from py_trees.common import Status
from core.domain.end_effector import SG100HandCommand, SG100_JOINT_COUNT
from core.domain.enums import ArmSide
SG100_OPEN = [0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SG100_OPENN = [0.0, -2.0, 0.0, 1.5, 1.5, 3.0, 1.5, 3.0, 0.0, 1.5, 3.0]
SG100_HALF = [1.5, -3.0, -0.5, 1.5, 1.5, 1.3, 1.5, 1.3, 0.0, 1.5, 1.3]
SG100_CLOSE = [1.5, -3.0, -0.5, 1.5, 1.5, 3.0, 1.5, 3.0, 0.0, 1.5, 3.0]
SG100_CLOSES = [1.5, -2.0, 2.0, 1.5, 1.5, 3.0, 1.5, 3.0, 0.0, 1.5, 3.0]
from orchestration.nodes.boxcarry import StableBasketDetector, build_plan, print_plan, validate_plan
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _require_success(result, operation):
    if result is None or not getattr(result, "success", False):
        raise RuntimeError("%s失败: %s" % (
            operation, getattr(result, "message", "无返回值")
        ))


@define_manifest(
    label="basket_vision 左手抬箱右手搬运",
    category=["perception", "motion", "arm"],
    tree_type="depalletize_bin",
    description="视觉定位后左手抬箱、右手加入，最终保持箱体供底盘导航",
    params=[], inputs=[], outputs=[],
)
class BasketVisionCarryMove(BaseAction):
    """一次性视觉抓取节点。

    ``execute_grasp=false`` 时只规划并返回 FAILURE，确保后续放置点导航不会
    在机器人没有真正拿到箱子时被执行。
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._status = None

    def initialise(self):
        self._status = None

    def update(self):
        if self._status is not None:
            return self._status
        if _DRY_RUN:
            self.feedback_message = "dry-run basket vision carry"
            self._status = Status.SUCCESS
            return self._status
        try:
            args = self._make_args()
            box = StableBasketDetector(
                topic=args.tag_topic,
                requested_id=args.tag_id,
                samples=args.samples,
                timeout=args.detection_timeout,
                max_spread=args.max_position_spread,
                target_frame=args.target_frame,
                tf_timeout=args.tf_timeout,
            ).detect()
            plan = build_plan(box, args)
            plan.use_whole_body_ik = args.use_whole_body_ik or args.box_layer >= 2
            validate_plan(plan, args)
            print_plan(box, plan)
            if not args.execute_grasp:
                self.feedback_message = (
                    "轨迹验证通过；execute_grasp=false，未发送手臂指令"
                )
                self._status = Status.FAILURE
                return self._status
            self._execute(plan, get_shared_hardware())
            self.feedback_message = "左手抬箱、右手加入完成；保持箱体等待导航"
            self._status = Status.SUCCESS
        except Exception as exc:
            self.feedback_message = "视觉抓取失败: %s" % exc
            self._status = Status.FAILURE
        return self._status

    def _make_args(self):
        p = self.params
        return SimpleNamespace(
            execute_grasp=bool(p.get("execute_grasp", False)),
            tag_topic=str(p.get("tag_topic", "/tag_detections")),
            tag_id=int(p.get("tag_id", -1)),
            target_frame=str(p.get("target_frame", "base_link")),
            samples=int(p.get("samples", 1)),
            detection_timeout=float(p.get("detection_timeout", 20.0)),
            tf_timeout=float(p.get("tf_timeout", 1.0)),
            max_position_spread=float(p.get("max_position_spread", 0.015)),
            box_width=float(p.get("box_width", 0.60)),
            grasp_x_offset=float(p.get("grasp_x_offset", 0.0)),
            grasp_x_offset_right=float(p.get("grasp_x_offset_right", 0.0)),
            grasp_z_offset=float(p.get("grasp_z_offset", -0.05)),
            grasp_z_offset_right=float(p.get("grasp_z_offset_right", 0.10)),
            side_clearance=float(p.get("side_clearance", 0.025)),
            approach_side_distance=float(p.get("approach_side_distance", 0.15)),
            approach_height=float(p.get("approach_height", 0.12)),
            lift_height=float(p.get("lift_height", 0.15)),
            pull_distance=float(p.get("pull_distance", 0.20)),
            left_rpy=list(p.get("left_rpy", [0, -85, 0])),
            right_rpy=list(p.get("right_rpy", [0, -75, 0])),
            chest_up_left_z=float(p.get("chest_up_left_z", 1.13)),
            chest_up_right_z=float(p.get("chest_up_right_z", 1.13)),
            chest_left_z=float(p.get("chest_left_z", 0.90)),
            chest_right_z=float(p.get("chest_right_z", 0.90)),
            table_left_z=float(p.get("table_left_z", 0.50)),
            table_right_z=float(p.get("table_right_z", 0.50)),
            expand_left_z=float(p.get("expand_left_z", 0.60)),
            expand_right_z=float(p.get("expand_right_z", 0.60)),
            box_layer=int(p.get("box_layer", 1)),
            use_whole_body_ik=bool(p.get("use_whole_body_ik", False)),
            x_min=float(p.get("x_min", 0.25)), x_max=float(p.get("x_max", 1.0)),
            y_min=float(p.get("y_min", -0.70)), y_max=float(p.get("y_max", 0.70)),
            z_min=float(p.get("z_min", 0.30)), z_max=float(p.get("z_max", 1.30)),
        )

    @staticmethod
    def _execute(plan, hardware):
        import time
        _require_success(hardware.control_end_effector(
                    ArmSide.LEFT, SG100HandCommand(positions=SG100_OPEN)), "左SG100手松开")
        time.sleep(1.0)
        _require_success(hardware.control_end_effector(
                                    ArmSide.LEFT, SG100HandCommand(positions=SG100_HALF)), "左SG100手夹紧")
        time.sleep(1.0)
        # _require_success(
        #     hardware.send_left_arm_ee_local_timed(plan.left_out, desire_time=5.0),
        #     "左臂绕行",
        # )
        # time.sleep(5.5)

        # # 新增：绕行后、抓取位前的“手部动作”
        # _require_success(
        #     hardware.control_end_effector(
        #         ArmSide.LEFT, SG100HandCommand(positions=SG100_OPENN),
        #     ),
        #     "左SG100手部动作",
        # )
        # time.sleep(1.0)

        # # 左臂抓取位
        # _require_success(
        #     hardware.send_left_arm_ee_local_timed(plan.left_grasp, desire_time=5.0),
        #     "左臂抓取位",
        # )
        # time.sleep(5.5)
        prefix_steps = (
            ("send_left_arm_ee_local_timed", (plan.left_out,), 2.0, "左臂绕行"),
            ("send_left_arm_ee_local_timed", (plan.left_grasp,), 2.0, "左臂抓取位"),
        )
        
        for method, poses, duration, label in prefix_steps:
            _require_success(getattr(hardware, method)(*poses, desire_time=duration), label)
            time.sleep(duration + 0.5)

        
        
        # _require_success(hardware.control_end_effector(
        #                     ArmSide.LEFT, SG100HandCommand(positions=SG100_CLOSE)), "左SG100手夹紧")
       
        if getattr(plan, "use_whole_body_ik", False):
            _require_success(hardware.send_single_arm_whole_body_ik_timed(
                True, plan.left_up, desire_time=7.0), "左臂整身IK提起")
            time.sleep(8.0)
            _require_success(hardware.send_single_arm_whole_body_ik_timed(
                True, plan.left_pull, desire_time=7.0), "左臂整身IK拉出")
            time.sleep(8.0)
        else:
            _require_success(hardware.send_left_arm_ee_local_timed(
                plan.left_up, desire_time=4.0), "左臂提起")
            time.sleep(3.5)
            _require_success(hardware.control_end_effector(
                                ArmSide.LEFT, SG100HandCommand(positions=SG100_CLOSE)), "左SG100手夹紧")
            time.sleep(1.0)
            _require_success(hardware.send_left_arm_ee_local_timed(
                plan.left_pull, desire_time=4.0), "左臂拉出")
            time.sleep(2.5)

        _require_success(hardware.control_end_effector(
                                    ArmSide.RIGHT, SG100HandCommand(positions=SG100_OPENN)), "右SG100手夹紧")
        time.sleep(1.0)
        _require_success(hardware.control_end_effector(
                    ArmSide.RIGHT, SG100HandCommand(positions=SG100_HALF)), "右SG100手夹紧")
        
        suffix_steps = (
            ("send_right_arm_ee_local_timed", (plan.right_out,), 2.5, "右臂绕行"),
             ("send_right_arm_ee_local_timed", (plan.right_out2,), 2, "右臂抓取位"),
            ("send_right_arm_ee_local_timed", (plan.right_grasp,), 2.5, "右臂抓取位"),
        )
        
        for method, poses, duration, label in suffix_steps:
            result = getattr(hardware, method)(*poses, desire_time=duration)
            _require_success(result, label)
            time.sleep(duration + 0.5)
        
        
        dual_method = (hardware.send_dual_arm_whole_body_ik_timed
                       if getattr(plan, "use_whole_body_ik", False)
                       else hardware.send_arm_ee_local_timed)
        _require_success(dual_method(plan.chest_up_left, plan.chest_up_right,
                                     desire_time=2.0), "双臂抬起")
        time.sleep(2.5)
        # _require_success(hardware.control_end_effector(
        #                     ArmSide.RIGHT, SG100HandCommand(positions=SG100_CLOSE)), "右SG100手夹紧")
        # time.sleep(1.0)
        _require_success(dual_method(plan.chest_left, plan.chest_right,
                                     desire_time=1.0), "搬到导航保持位")
        time.sleep(1.5)
