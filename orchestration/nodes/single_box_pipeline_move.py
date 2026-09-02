# -*- coding: utf-8 -*-
"""单箱动作调试：6D识别 -> 固定抓取策略 -> 放置（可选）-> 停止。

该节点不调用视觉规划器。操作者手动将底盘开到目标作业点后，节点直接调用
6D 位姿服务，并按 ``grasp_mode`` 执行指定的抓取策略。

调试模式 (debug_step_mode=true):
  每步执行前暂停，等待用户按 Enter 确认，按 q 退出。

原地测试模式 (skip_navigation=true):
  跳过所有底盘导航，机器人在原地不动。
  只跑：6D位姿识别 -> 轨迹规划 -> 机械臂抓取 -> 机械臂放置。

安全等级:
  Level 0: skip_navigation=true + execute_grasp=false → 原地纯视觉+轨迹验证
  Level 1: skip_navigation=true + execute_grasp=true  → 原地手臂动作
  Level 2: 正常导航 + execute_grasp=false            → 导航+视觉验证
  Level 3: 正常导航 + execute_grasp=true             → 全流程真机
"""

import math
import os

import rospy
from py_trees.common import Status

from basket_vision.sdk.basket_vision_client import BasketVisionClient
from boxcarry import DetectedPose, build_plan, validate_plan
from core.domain.chassis_options import MoveToTargetOptions
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.basket_place_after_nav_move import BasketPlaceAfterNavMove
from orchestration.nodes.basket_vision_carry_move import BasketVisionCarryMove
from orchestration.nodes.basket_vision_dual_carry_move import BasketVisionDualCarryMove
from orchestration.nodes.basket_vision_right_carry_move import BasketVisionRightCarryMove
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

_EXECUTORS = {
    "left_lift_right_carry": BasketVisionCarryMove._execute,
    "right_lift_left_carry": BasketVisionRightCarryMove._execute,
    "dual_lift_carry": BasketVisionDualCarryMove._execute,
}


@define_manifest(
    label="单箱动作调试：6D识别->固定抓取策略->放置",
    category=["perception", "motion", "chassis", "arm"],
    tree_type="depalletize_bin",
    description=(
        "操作者手动定位后，直接调用 6D 位姿服务，按固定抓取策略执行，"
        "完成抓取、导航到放置点并放下后立即停止。"
        "skip_navigation=true 时跳过所有底盘导航，原地测试。"
    ),
    params=[], inputs=[], outputs=[],
)
class SingleBoxPipelineMove(BaseAction):
    """单箱测试节点。

    调试模式开关 (在 board.json 中配置):
      skip_navigation=true   → 跳过所有底盘导航，机器人原地不动
      debug_step_mode=true   → 每步暂停，按 Enter 继续 / q 退出
      execute_grasp=false    → 只验证导航+视觉+轨迹，不发手臂指令
      execute_place=false    → 不发放置指令
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._status = None
        self._debug = False
        self._skip_nav = False

    def initialise(self):
        self._status = None
        self._debug = bool(self.params.get("debug_step_mode", False))
        self._skip_nav = bool(self.params.get("skip_navigation", False))
        if self._skip_nav:
            rospy.loginfo("=" * 60)
            rospy.loginfo("  [原地测试] 跳过所有底盘导航，机器人原地不动")
            rospy.loginfo("=" * 60)
        if self._debug:
            rospy.loginfo("=" * 60)
            rospy.loginfo("  [调试模式] 每步暂停，按 Enter 继续 / 输入 q 退出")
            rospy.loginfo("=" * 60)

    def _debug_pause(self, step_desc):
        if not self._debug:
            return
        try:
            user_input = raw_input(
                "\n  >>> [调试] %s <<<\n"
                "  >>> 按 Enter 继续执行，输入 q 退出: " % step_desc
            )
        except NameError:
            user_input = input(
                "\n  >>> [调试] %s <<<\n"
                "  >>> 按 Enter 继续执行，输入 q 退出: " % step_desc
            )
        if user_input.strip().lower() == "q":
            raise RuntimeError("用户手动退出调试模式")

    def update(self):
        if self._status is not None:
            return self._status
        if _DRY_RUN:
            self.feedback_message = "dry-run 单箱测试"
            self._status = Status.SUCCESS
            return self._status
        try:
            self._run()
            self.feedback_message = "单箱测试完成"
            self._status = Status.SUCCESS
        except Exception as exc:
            self.feedback_message = "单箱测试失败: %s" % exc
            rospy.logerr(self.feedback_message)
            self._status = Status.FAILURE
        return self._status

    def _run(self):
        p = self.params
        hardware = get_shared_hardware()

        # ---- 步骤 1: 确认 TF 连通 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[单箱测试] 步骤 1/6: 确认 TF 连通")
        self._debug_pause("步骤 1/6: 确认 TF 连通 (base_link -> camera)")
        self._wait_camera_tf()

        # ---- 步骤 2: 使用手动指定的抓取策略 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[单箱测试] 步骤 2/6: 读取手动抓取策略（不调用视觉规划器）")
        mode = str(p.get("grasp_mode", "dual_lift_carry"))
        if mode not in _EXECUTORS:
            raise ValueError("手动 grasp_mode 无效: %s" % mode)
        step = {"target_index": int(p.get("manual_target_index", 0))}
        action = {
            "grasp_mode": mode,
            "box_layer": int(p.get("box_layer", 1)),
        }
        rospy.loginfo("  手动策略: mode=%s, box_layer=%d, 6D目标序号=%d",
                      mode, action["box_layer"], step["target_index"])

        # ---- 步骤 3: 导航到抓取作业点 ----
        rospy.loginfo("=" * 60)
        if self._skip_nav:
            rospy.loginfo("[单箱测试] 步骤 3/6: 跳过导航 (原地测试模式)")
        else:
            nav_pose = {
                "x": float(p["grasp_nav_x"]),
                "y": float(p["grasp_nav_y"]),
                "theta_deg": float(p["grasp_nav_theta_deg"]),
            }
            rospy.loginfo("[单箱测试] 步骤 3/6: 导航到手动抓取点 (%.3f, %.3f)",
                          nav_pose["x"], nav_pose["y"])
            self._debug_pause("步骤 3/6: 导航到抓取点 -> 确认底盘到达")
            self._navigate_to(hardware, nav_pose, "single_box_grasp")

        # ---- 步骤 4: 6D 位姿识别 + 轨迹规划 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[单箱测试] 步骤 4/6: 6D 位姿识别与轨迹规划 (模式=%s)", mode)
        self._debug_pause("步骤 4/6: 调用 6D 位姿服务 -> 生成抓取轨迹")
        plan = self._detect_6d_and_plan(step, action, p)
        rospy.loginfo("  6D 位姿: pos=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)",
                      plan.grasp_pose.x, plan.grasp_pose.y, plan.grasp_pose.z,
                      math.degrees(plan.grasp_pose.roll),
                      math.degrees(plan.grasp_pose.pitch),
                      math.degrees(plan.grasp_pose.yaw))

        # ---- 步骤 5: 执行抓取 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[单箱测试] 步骤 5/6: 执行抓取 (模式=%s)", mode)
        grasp_enabled = bool(p.get("execute_grasp", False))
        if grasp_enabled:
            self._debug_pause(
                "步骤 5/6: [真机] 执行抓取动作 (模式=%s) - 请确认安全!" % mode
            )
            _EXECUTORS[mode](plan, hardware)
            rospy.loginfo("抓取完成，箱体保持夹持")
        else:
            rospy.loginfo("  execute_grasp=false，跳过抓取（轨迹已验证通过）")
            raise RuntimeError(
                "轨迹验证通过，但 execute_grasp=false；安全停止。"
                "如需真机执行，请将 execute_grasp 设为 true"
            )

        # ---- 步骤 6: 导航到放置点并放下 ----
        rospy.loginfo("=" * 60)
        place_enabled = bool(p.get("execute_place", True))
        if self._skip_nav:
            rospy.loginfo("[单箱测试] 步骤 6/6: 跳过导航到放置点 (原地测试模式)")
            rospy.loginfo("  目标放置点: (%.3f, %.3f, theta=%.1f) -> %s",
                          p["place_x"], p["place_y"], p["place_theta_deg"],
                          "放下复位" if place_enabled else "跳过放置")
        else:
            rospy.loginfo("[单箱测试] 步骤 6/6: 导航到放置点并放下")
            self._debug_pause(
                "步骤 6/6: 导航到放置点 (%.3f, %.3f) -> %s" % (
                    p["place_x"], p["place_y"],
                    "放下复位" if place_enabled else "跳过放置"
                )
            )
            self._navigate_to(hardware, {
                "x": p["place_x"],
                "y": p["place_y"],
                "theta_deg": p["place_theta_deg"],
            }, "single_box_place")
        self._execute_place(hardware, p)

        rospy.loginfo("=" * 60)
        rospy.loginfo("单箱测试完成！")

    def _navigate_to(self, hardware, pose, label):
        options = MoveToTargetOptions(
            avoid_enabled=bool(self.params.get("nav_avoid_enabled", False)),
            avoid_distance=float(self.params.get("nav_avoid_distance", 0.5)),
            linear_velocity=float(self.params.get("nav_linear_velocity", 0.3)),
            angular_velocity=float(self.params.get("nav_angular_velocity", 0.5)),
            position_threshold=float(self.params.get("nav_position_threshold", 0.08)),
            angle_threshold=float(self.params.get("nav_angle_threshold", 0.1)),
            allow_rotation=bool(self.params.get("nav_allow_rotation", True)),
        )
        result = hardware.base_move_to_target_jibot(
            float(pose["x"]), float(pose["y"]),
            math.radians(float(pose["theta_deg"])), options=options,
        )
        if not result.success or not result.data or not result.data.get("task_id"):
            raise RuntimeError("%s 导航下发失败: %s" % (label, result.message))
        rospy.loginfo("  导航任务已下发: task_id=%s", result.data["task_id"])
        arrived = hardware.check_arrived_jibot(
            str(result.data["task_id"]), blocking=True,
            timeout=float(self.params.get("nav_arrival_timeout_sec", 120.0)),
        )
        if not arrived.success or not (arrived.data or {}).get("arrived", False):
            raise RuntimeError("%s 未到达: %s" % (label, arrived.message))
        rospy.loginfo("  已到达目标点")

    def _wait_camera_tf(self):
        import tf2_ros
        target = str(self.params.get("camera_tf_target_frame", "base_link"))
        source = str(self.params.get("camera_tf_source_frame",
                                     "camera_color_optical_frame"))
        timeout = float(self.params.get("camera_tf_timeout_sec", 15.0))
        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.loginfo("  等待 TF %s -> %s ...", target, source)
        try:
            tf_buffer.lookup_transform(target, source, rospy.Time(0),
                                       rospy.Duration(timeout))
            rospy.loginfo("  TF 已连通")
        except Exception as exc:
            raise RuntimeError("TF %s -> %s 超时: %s" % (target, source, exc))
        finally:
            tf_listener.unregister()

    def _detect_6d_and_plan(self, step, action, p):
        client = BasketVisionClient({
            "basket_pose_service": p.get("basket_pose_service",
                                         "/infer_basket_pose"),
            "timeout": float(p.get("detection_timeout", 20.0)),
            "save_images": bool(p.get("save_vision_images", True)),
        })
        rospy.loginfo("  调用 6D 位姿服务: %s", p.get("basket_pose_service"))
        result = client.infer_basket_pose()
        if not result.success or not result.data.get("baskets"):
            raise RuntimeError("6D 位姿识别失败: %s" % result.message)
        target_index = int(step.get("target_index", step.get("basket_index", 0)))
        baskets = result.data["baskets"]
        if target_index < 0 or target_index >= len(baskets):
            raise IndexError("6D 目标序号越界: %d/%d" % (target_index, len(baskets)))
        pose = baskets[target_index]["pose6d"]
        rospy.loginfo("  识别到箱子 #%d: pos=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)",
                      int(step.get("basket_id", target_index)),
                      pose.x, pose.y, pose.z,
                      math.degrees(pose.roll), math.degrees(pose.pitch),
                      math.degrees(pose.yaw))
        mode = str(action.get("grasp_mode", ""))

        if mode == "dual_lift_carry":
            # 双臂同步搬运：XYZ 固定，姿态角仍采用视觉识别结果
            target_x = 0.812
            target_y = -0.060
            target_z = 0.801

            rospy.loginfo(
                "  双臂模式使用固定抓取点: x=%.3f, y=%.3f, z=%.3f",
                target_x, target_y, target_z
            )
        else:
            # 左先抬 / 右先抬：使用视觉完整位置
            target_x = pose.x
            target_y = pose.y
            target_z = pose.z

        detected = DetectedPose(
            target_x, target_y, target_z,
            pose.roll, pose.pitch, pose.yaw,
            int(step.get("basket_id", target_index)),
            "camera_color_optical_frame", "base_link",
        )
        args = BasketVisionCarryMove(
            "single_box_plan", "单箱轨迹规划", "", p
        )._make_args()
        args.box_layer = int(action.get("box_layer", 1)) if isinstance(action, dict) else 1
        plan = build_plan(detected, args)
        plan.use_whole_body_ik = bool(p.get("use_whole_body_ik", False))
        plan.grasp_pose = detected
        validate_plan(plan, args)
        rospy.loginfo("  轨迹规划完成，已通过安全校验")
        return plan

    def _execute_place(self, hardware, p):
        if not bool(p.get("execute_place", True)):
            rospy.loginfo("  execute_place=false，跳过放置")
            return
        params_dict = dict(p)
        params_dict["execute_place"] = True
        place = BasketPlaceAfterNavMove(
            "single_box_place", "单箱放置", "", params_dict
        )
        place._execute(hardware)
        rospy.loginfo("  放置与复位完成")
