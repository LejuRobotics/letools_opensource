# -*- coding: utf-8 -*-
"""单箱动作调试：固定 6D 位姿 -> 固定抓取策略 -> 放置（可选）-> 停止。

该节点不调用视觉规划器。操作者手动将底盘开到目标作业点后，节点直接调用
固定的 ``base_link`` 6D 位姿，并按 ``grasp_mode`` 执行指定的抓取策略。

调试模式 (debug_step_mode=true):
  每步执行前暂停，等待用户按 Enter 确认，按 q 退出。

原地测试模式 (skip_navigation=true):
  跳过所有底盘导航，机器人在原地不动。
  只跑：固定6D位姿 -> 轨迹规划 -> 机械臂抓取 -> 机械臂放置。

安全等级:
  Level 0: skip_navigation=true + execute_grasp=false → 原地纯视觉+轨迹验证
  Level 1: skip_navigation=true + execute_grasp=true  → 原地手臂动作
  Level 2: 正常导航 + execute_grasp=false            → 导航+视觉验证
  Level 3: 正常导航 + execute_grasp=true             → 全流程真机
"""

import math
import os
from dataclasses import replace

import rospy
from py_trees.common import Status

from boxcarry import DetectedPose, build_plan, validate_plan
from core.domain.chassis_options import MoveToTargetOptions
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.basket_place_after_nav_move import BasketPlaceAfterNavMove
from orchestration.nodes.basket_vision_carry_move import BasketVisionCarryMove
from orchestration.nodes.basket_vision_dual_carry_move import BasketVisionDualCarryMove
from orchestration.nodes.basket_vision_right_carry_move import BasketVisionRightCarryMove
from orchestration.nodes.basket_vision_small_box_carry_move import (
    BasketVisionSmallBoxLeftCarryMove,
    BasketVisionSmallBoxRightCarryMove,
    SMALL_BOX_GRASP_MODES,
    SMALL_BOX_WIDTH_M,
)
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

_EXECUTORS = {
    "left_lift_right_carry": BasketVisionCarryMove._execute,
    "right_lift_left_carry": BasketVisionRightCarryMove._execute,
    "dual_lift_carry": BasketVisionDualCarryMove._execute,
    "small_box_left_lift_right_carry": BasketVisionSmallBoxLeftCarryMove._execute,
    "small_box_right_lift_left_carry": BasketVisionSmallBoxRightCarryMove._execute,
}

# 单箱静态测试使用的固定箱体 6D 位姿（base_link 坐标系）。
# 单位：m / deg；如需换点，直接修改此处，而不是 board.json。
FIXED_BOX_6D_POSE = {
    "x": 0.738,
    #0.738
    "y": -0.06,
    "z": 0.654,
    #0.654
    ###0.784
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "yaw_deg": 0.0,
    "detection_id": 0,
}


@define_manifest(
    label="单箱动作调试：固定6D位姿->固定抓取策略->放置",
    category=["perception", "motion", "chassis", "arm"],
    tree_type="depalletize_bin",
    description=(
        "操作者手动定位后，直接使用配置中的固定 6D 位姿，按固定抓取策略执行，"
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
      execute_grasp=false    → 只验证导航+固定点位+轨迹，不发手臂指令
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

        # ---- 步骤 1: 校验固定 6D 抓取位姿 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[单箱测试] 步骤 1/8: 读取固定 6D 抓取位姿")
        self._debug_pause("步骤 1/8: 确认固定 6D 抓取位姿")
        fixed_detected = self._fixed_6d_pose(p)

        # ---- 步骤 2: 读取手动抓取策略与导航库 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[单箱测试] 步骤 2/8: 读取手动抓取策略与导航库")
        mode = str(p.get("grasp_mode", "small_box_left_lift_right_carry"))
        if mode not in _EXECUTORS:
            raise ValueError("手动 grasp_mode 无效: %s" % mode)
        if (mode in SMALL_BOX_GRASP_MODES
                and abs(float(p.get("box_width", 0.0)) - SMALL_BOX_WIDTH_M) >= 0.01):
            raise ValueError(
                "40cm 小箱动作要求 box_width=%.2f，当前为 %.3f"
                % (SMALL_BOX_WIDTH_M, float(p.get("box_width", 0.0)))
            )
        step = {"target_index": int(p.get("manual_target_index", 0))}
        action = {
            "grasp_mode": mode,
            "box_layer": int(p.get("box_layer", 1)),
        }
        rospy.loginfo("  手动策略: mode=%s, box_layer=%d", mode, action["box_layer"])
        if self._skip_nav:
            nav_key, detect_nav_pose, grasp_nav_pose = None, None, None
        else:
            nav_key, detect_nav_pose, grasp_nav_pose = self._resolve_nav_poses(p)

        # ---- 步骤 3: 导航到对应的远距离检测点 ----
        rospy.loginfo("=" * 60)
        if self._skip_nav:
            rospy.loginfo("[单箱测试] 步骤 3/8: 跳过导航到检测点 (原地测试)")
        else:
            rospy.loginfo("[单箱测试] 步骤 3/8: 导航到检测点 [%s] (%.3f, %.3f)",
                          nav_key, detect_nav_pose["x"], detect_nav_pose["y"])
            self._debug_pause("步骤 3/8: 导航到检测点 -> 确认底盘到达")
            self._navigate_to(hardware, detect_nav_pose, "single_box_detect")

        # ---- 步骤 4: 使用固定 6D 位姿（不调用识别服务）----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[单箱测试] 步骤 4/8: 使用固定 6D 位姿，不调用识别服务")
        self._debug_pause("步骤 4/8: 使用固定 6D 位姿")
        detected = fixed_detected
        detected = self._apply_pose_offset(detected)

        # ---- 步骤 5: 导航到对应抓取点 ----
        rospy.loginfo("=" * 60)
        if self._skip_nav:
            rospy.loginfo("[单箱测试] 步骤 5/8: 跳过导航到抓取点 (原地测试)")
        else:
            rospy.loginfo("[单箱测试] 步骤 5/8: 导航到抓取点 [%s] (%.3f, %.3f)",
                          nav_key, grasp_nav_pose["x"], grasp_nav_pose["y"])
            self._debug_pause("步骤 5/8: 导航到抓取点 -> 确认底盘到达")
            self._navigate_to(hardware, grasp_nav_pose, "single_box_grasp")

        # ---- 步骤 6: 使用固定 6D 位姿规划 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[单箱测试] 步骤 6/8: 使用固定 6D 位姿规划轨迹 (模式=%s)", mode)
        self._debug_pause("步骤 6/8: 固定 6D 位姿 -> 生成抓取轨迹")
        plan = self._plan_from_pose(detected, action, p)
        rospy.loginfo("  抓取位姿: pos=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)",
                      plan.grasp_pose.x, plan.grasp_pose.y, plan.grasp_pose.z,
                      math.degrees(plan.grasp_pose.roll),
                      math.degrees(plan.grasp_pose.pitch),
                      math.degrees(plan.grasp_pose.yaw))

        # ---- 步骤 7: 执行抓取 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[单箱测试] 步骤 7/8: 执行抓取 (模式=%s)", mode)
        grasp_enabled = bool(p.get("execute_grasp", False))
        if grasp_enabled:
            self._debug_pause(
                "步骤 7/8: [真机] 执行抓取动作 (模式=%s) - 请确认安全!" % mode
            )
            _EXECUTORS[mode](plan, hardware)
            rospy.loginfo("抓取完成，箱体保持夹持")
        else:
            rospy.loginfo("  execute_grasp=false，跳过抓取（轨迹已验证通过）")
            raise RuntimeError(
                "轨迹验证通过，但 execute_grasp=false；安全停止。"
                "如需真机执行，请将 execute_grasp 设为 true"
            )

        # ---- 步骤 8: 导航到放置点并放下 ----
        rospy.loginfo("=" * 60)
        place_enabled = bool(p.get("execute_place", True))
        if self._skip_nav:
            rospy.loginfo("[单箱测试] 步骤 8/8: 跳过导航到放置点 (原地测试)")
            rospy.loginfo("  目标放置点: (%.3f, %.3f, theta=%.1f) -> %s",
                          p["place_x"], p["place_y"], p["place_theta_deg"],
                          "放下复位" if place_enabled else "跳过放置")
        else:
            rospy.loginfo("[单箱测试] 步骤 8/8: 导航到放置点并放下")
            self._debug_pause(
                "步骤 8/8: 导航到放置点 (%.3f, %.3f) -> %s" % (
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

    def _resolve_nav_poses(self, p):
        nav_table = dict(p.get("nav_pose_table", {}))
        detect_table = dict(p.get("detect_pose_table", {}))
        nav_key = str(p.get("nav_pose_key", "front.face_0.zone_1"))
        if nav_key.startswith("${") and nav_key.endswith("}"):
            raise RuntimeError(
                "nav_pose_key 未被黑板解析，当前值=%s。单箱流程必须使用 full_pipeline_board.json 启动"
                "（bash start_single_box_pipeline.sh 已内置该黑板），不能使用默认 board.json。" % nav_key
            )

        if nav_key not in nav_table:
            raise KeyError("导航点未在 nav_pose_table 中配置: %s" % nav_key)
        if nav_key not in detect_table:
            raise KeyError("检测点未在 detect_pose_table 中配置: %s" % nav_key)

        grasp = nav_table[nav_key]
        detect = detect_table[nav_key]

        detect_nav_pose = {
            "x": float(detect["x"]),
            "y": float(detect["y"]),
            "theta_deg": float(detect["theta_deg"]),
        }
        grasp_nav_pose = {
            "x": float(grasp["x"]),
            "y": float(grasp["y"]),
            "theta_deg": float(grasp["theta_deg"]),
        }

        rospy.loginfo("  导航键: %s", nav_key)
        rospy.loginfo("  检测点: (%.3f, %.3f, %.1f deg)",
                      detect_nav_pose["x"], detect_nav_pose["y"],
                      detect_nav_pose["theta_deg"])
        rospy.loginfo("  抓取点: (%.3f, %.3f, %.1f deg)",
                      grasp_nav_pose["x"], grasp_nav_pose["y"],
                      grasp_nav_pose["theta_deg"])
        return nav_key, detect_nav_pose, grasp_nav_pose

    def _fixed_6d_pose(self, _p):
        """使用代码中固化的 ``base_link`` 系 6D 位姿，不调用视觉服务。

        角度字段使用度：``roll_deg``、``pitch_deg``、``yaw_deg``，以免与
        ``DetectedPose`` 内部使用的弧度混淆。
        """
        raw = FIXED_BOX_6D_POSE
        required = ("x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg")
        missing = [key for key in required if key not in raw]
        if missing:
            raise ValueError("fixed_6d_pose 缺少字段: %s" % ", ".join(missing))
        pose = DetectedPose(
            float(raw["x"]), float(raw["y"]), float(raw["z"]),
            math.radians(float(raw["roll_deg"])),
            math.radians(float(raw["pitch_deg"])),
            math.radians(float(raw["yaw_deg"])),
            int(raw.get("detection_id", 0)), "base_link", "base_link",
        )
        rospy.loginfo(
            "  固定箱体 #%d: pos=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)",
            pose.detection_id, pose.x, pose.y, pose.z,
            math.degrees(pose.roll), math.degrees(pose.pitch), math.degrees(pose.yaw),
        )
        return pose

    def _apply_pose_offset(self, detected):
        """给检测到的 base_link 位姿加固定 xyz 偏移后送入轨迹规划。"""
        offset = list(self.params.get("detected_pose_offset", [0.0, 0.0, 0.0]))
        dx = float(offset[0]) if len(offset) > 0 else 0.0
        dy = float(offset[1]) if len(offset) > 1 else 0.0
        dz = float(offset[2]) if len(offset) > 2 else 0.0
        adjusted = replace(detected, x=detected.x + dx, y=detected.y + dy, z=detected.z + dz)
        rospy.loginfo("  固定偏移: dx=%.3f, dy=%.3f, dz=%.3f -> pos=(%.3f,%.3f,%.3f)",
                      dx, dy, dz, adjusted.x, adjusted.y, adjusted.z)
        return adjusted

    def _plan_from_pose(self, detected, action, p):
        """根据检测到的 base_link 位姿直接生成抓取轨迹。"""
        rospy.loginfo("  抓取点 base_link 系位姿: pos=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)",
                      detected.x, detected.y, detected.z,
                      math.degrees(detected.roll), math.degrees(detected.pitch),
                      math.degrees(detected.yaw))

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
