# -*- coding: utf-8 -*-
"""全流程：导航观测点 -> 视觉规划 -> 导航抓取点 -> 6D位姿识别 -> 抓取 -> 导航放置点 -> 放下。

完整流程：
  1. 调用视觉规划器获取抓取序列
  2. 对序列中每个抓取步骤：
     a. 导航到抓取作业点
     b. 确认底盘到达
     c. 6D 位姿识别（调用 /infer_basket_pose）
     d. 将 6D 位姿作为机械臂轨迹规划终点，生成抓取轨迹
     e. 执行抓取动作
     f. 导航到放置点
     g. 确认底盘到达
     h. 执行放下、撤离和复位
"""

import json
import math
import os
import time

import rospy
from py_trees.common import Status
from rosservice import get_service_class_by_name

from collections import defaultdict
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


def _message_dict(value):
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        decoded = json.loads(value)
        if not isinstance(decoded, dict):
            raise ValueError("规划步骤 JSON 必须是对象")
        return decoded
    slots = getattr(value, "__slots__", ())
    return {name: getattr(value, name) for name in slots}


# def _decode_sequence(response):
#     """兼容 sequence 数组、JSON 字符串以及 data/result JSON。"""
#     value = getattr(response, "sequence", None)
#     if value is None:
#         for name in ("data", "result", "json_result", "decision_json"):
#             candidate = getattr(response, name, None)
#             if candidate:
#                 value = candidate
#                 break
#     if isinstance(value, str):
#         value = json.loads(value)
#     if isinstance(value, dict):
#         value = value.get("sequence", [])
#     if value is None:
#         return []
#     return [_message_dict(item) for item in list(value)]

def _decode_sequence(response):
    """兼容 TriggerResponse(message=JSON字符串) 等格式。"""
    # 1) TriggerResponse: message 字段包含 JSON 字符串
    value = getattr(response, "message", None)
    if isinstance(value, str):
        try:
            parsed = json.loads(value)
            if isinstance(parsed, dict) and "sequence" in parsed:
                value = parsed
        except Exception:
            pass
    # 2) 直接属性 sequence
    if not isinstance(value, dict) or "sequence" not in value:
        value = getattr(response, "sequence", None)
    # 3) 常见字段名
    if value is None or (isinstance(value, dict) and "sequence" not in value):
        for name in ("json_result", "data", "result", "decision_json"):
            candidate = getattr(response, name, None)
            if candidate:
                if isinstance(candidate, str):
                    try:
                        candidate = json.loads(candidate)
                    except Exception:
                        pass
                if isinstance(candidate, dict) and "sequence" in candidate:
                    value = candidate
                    break
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except Exception:
            pass
    if isinstance(value, dict):
        value = value.get("sequence", [])
    if not value:
        return []
    return [_message_dict(item) for item in list(value)]

def _nav_key(step, default_side="front"):
    """从规划步骤提取导航键，格式如 'front.face_0.zone_1'。"""
    direct = step.get("nav_pose_key") or step.get("nav_key")
    if direct:
        return str(direct)
    result = step.get("result", {})
    if isinstance(result, dict):
        side = step.get("side", result.get("side", default_side))
        face = step.get("face", step.get("face_id", result.get("face", result.get("face_id"))))
        zone = step.get("zone", step.get("zone_id", result.get("zone", result.get("zone_id"))))
    else:
        side = step.get("side", default_side)
        face = step.get("face", step.get("face_id"))
        zone = step.get("zone", step.get("zone_id"))
    if face is None or zone is None:
        raise ValueError("规划步骤缺少 nav_pose_key，且无法从 side/face/zone 生成")
    face = str(face)
    zone = str(zone)
    if not face.startswith("face_"):
        face = "face_%s" % face
    if not zone.startswith("zone_"):
        zone = "zone_%s" % zone
    return "%s.%s.%s" % (side, face, zone)


@define_manifest(
    label="全流程：观测->规划->导航->6D->抓取->放置",
    category=["perception", "motion", "chassis", "arm"],
    tree_type="depalletize_bin",
    description=(
        "完整单箱搬箱流程：导航到观测点后调用视觉规划器，"
        "按返回序列逐点导航、6D识别、抓取、搬运到放置点并放下"
    ),
    params=[], inputs=[], outputs=[],
)
class FullPipelineGraspMove(BaseAction):
    """全流程视觉规划-导航-6D抓取-导航-放置节点。

    参数从 board.json 黑板读取，与现有 py_tree 接口兼容。
    每一步都有清晰的日志输出，便于调试和监控。
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
            self.feedback_message = "dry-run 全流程视觉抓取放置"
            self._status = Status.SUCCESS
            return self._status
        try:
            self._run()
            self.feedback_message = "全流程完成"
            self._status = Status.SUCCESS
        except Exception as exc:
            self.feedback_message = "全流程失败: %s" % exc
            rospy.logerr(self.feedback_message)
            self._status = Status.FAILURE
        return self._status

    # ==================================================================
    # 主流程
    # ==================================================================
    def _run(self):
        p = self.params
        hardware = get_shared_hardware()

        # ---- 步骤 1: 确认 TF 连通 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[步骤 1/8] 确认 TF 连通: base_link -> camera_color_optical_frame")
        self._wait_camera_tf()

        # ---- 步骤 2: 调用视觉规划器 ----
        rospy.loginfo("=" * 60)
        rospy.loginfo("[步骤 2/8] 调用视觉规划器: %s", p.get("binplanner_service_name"))
        sequence = self._call_vision_planner(p)
        rospy.loginfo("视觉规划器返回 %d 个抓取步骤", len(sequence))
        if not sequence:
            rospy.loginfo("视觉规划器返回空序列，无需抓取")
            return

        nav_table = dict(p.get("nav_pose_table", {})) or {
            "front.face_0.zone_1": { "x": 1.731, "y": 0.607, "theta_deg": -90.0 },
            "front.face_0.zone_2": { "x": 1.207, "y": 0.627, "theta_deg": -90.0 },
            "front.face_1.zone_2": { "x": 0.307, "y": 0.016, "theta_deg": -1.0 },
            "front.face_2.zone_1": { "x": 1.731, "y": 0.607, "theta_deg": -90.0 },
            "rear.face_0.zone_1": { "x": 1.731, "y": -1.152, "theta_deg": 90.0 },
            "rear.face_0.zone_2": { "x": 1.227, "y": -1.102, "theta_deg": 90.0 },
            "rear.face_1.zone_2": { "x": 0.267, "y": -0.452, "theta_deg": 0.0 },
            "rear.face_2.zone_1": { "x": 1.731, "y": -1.152, "theta_deg": 90.0 },
        }
        action_library = dict(p.get("grasp_action_library", {})) or {
            "front.face_0.zone_1": { "grasp_mode": "left_lift_right_carry", "box_layer": 1 ,"box_width": 0.70},
            "front.face_0.zone_2": { "grasp_mode": "dual_lift_carry", "box_layer": 1 ,"box_width": 0.40},
            "front.face_1.zone_2": { "grasp_mode": "left_lift_right_carry", "box_layer": 1 ,"box_width": 0.50},
            "front.face_2.zone_1": { "grasp_mode": "left_lift_right_carry", "box_layer": 1 ,"box_width": 0.70},
            "rear.face_0.zone_1": { "grasp_mode": "left_lift_right_carry", "box_layer": 1 ,"box_width": 0.70},
            "rear.face_0.zone_2": { "grasp_mode": "dual_lift_carry", "box_layer": 1,"box_width": 0.40 },
            "rear.face_1.zone_2": { "grasp_mode": "right_lift_left_carry", "box_layer": 1 ,"box_width": 0.50},
            "rear.face_2.zone_1": { "grasp_mode": "right_lift_left_carry", "box_layer": 1 ,"box_width": 0.70},
        }

        key_occurrences = defaultdict(int)
        remap_by_occurrence = {
            ("front.face_1.zone_2", 2): "front.face_0.zone_2",
            ("rear.face_1.zone_2", 2): "rear.face_0.zone_2",
        }
        visited_nav_keys = set()
        # ---- 对每个抓取步骤执行完整流程 ----
        for index, step in enumerate(sequence):
            rospy.loginfo("=" * 60)
            rospy.loginfo(">>> 处理第 %d/%d 个箱子 <<<", index + 1, len(sequence))

            # key = _nav_key(step, default_side=p.get("nav_side", "front"))
            source_key = _nav_key(
                step, default_side=p.get("nav_side", "front")
            )
            key_occurrences[source_key] += 1
            occurrence = key_occurrences[source_key]

            key = remap_by_occurrence.get(
                (source_key, occurrence), source_key
            )
            rospy.loginfo(
                "  原规划键=%s，第%d次；实际导航/抓取键=%s",
                source_key, occurrence, key,
            )
                        # 同一最终导航点只停留一次。
            # 注意：这里使用重映射后的 key，而不是 source_key。
            if key in visited_nav_keys:
                rospy.loginfo(
                    "  跳过第 %d/%d 个箱子：原规划键=%s，第%d次；"
                    "目标点=%s 已处理过",
                    index + 1, len(sequence),
                    source_key, occurrence, key,
                )
                continue

            visited_nav_keys.add(key)
            if key not in nav_table:
                raise KeyError("导航点未在 nav_pose_table 中配置: %s" % key)
            if key not in action_library:
                raise KeyError("导航点动作未在 grasp_action_library 中配置: %s" % key)

            action = action_library[key]
            mode = action.get("grasp_mode") if isinstance(action, dict) else action
            if mode not in _EXECUTORS:
                raise ValueError("导航点 %s 的抓取模式无效: %s" % (key, mode))

            nav_pose = nav_table[key]
            rospy.loginfo("  导航键: %s", key)

            # 根据面决定箱子宽度：正面/背面(face_0)用0.6m，侧面(face_1/face_2)用0.4m

            rospy.loginfo("  抓取模式: %s", mode)
            rospy.loginfo("  抓取模式: %s", mode)
            rospy.loginfo("  导航坐标: x=%.3f, y=%.3f, theta=%.1f deg",
                          nav_pose["x"], nav_pose["y"], nav_pose["theta_deg"])

            # ---- 步骤 3: 导航到抓取作业点 ----
            rospy.loginfo("[步骤 3/8] 导航到抓取作业点: %s", key)
            self._navigate_to(hardware, nav_pose, "grasp_%d" % index)

            # ---- 步骤 4: 确认底盘到达抓取点 ----
            rospy.loginfo("[步骤 4/8] 确认底盘到达抓取点")

            # ---- 步骤 5: 6D 位姿识别 ----
            rospy.loginfo("[步骤 5/8] 6D 位姿识别: 调用 %s", p.get("basket_pose_service"))
            plan = self._detect_6d_and_plan(step, action, p)

            # ---- 步骤 6: 执行抓取（6D位姿作为轨迹终点） ----
            rospy.loginfo("[步骤 6/8] 执行抓取: 模式=%s, 层数=%d",
                          mode, action.get("box_layer", 1))
            rospy.loginfo("  6D 位姿: x=%.3f, y=%.3f, z=%.3f, roll=%.1f, pitch=%.1f, yaw=%.1f",
                          plan.grasp_pose.x, plan.grasp_pose.y, plan.grasp_pose.z,
                          math.degrees(plan.grasp_pose.roll),
                          math.degrees(plan.grasp_pose.pitch),
                          math.degrees(plan.grasp_pose.yaw))
            # if not bool(p.get("execute_grasp", False)):
            #     raise RuntimeError(
            #         "轨迹规划验证通过，但 execute_grasp=false；安全停止。"
            #         "如需真机执行，请将 execute_grasp 设为 true"
            #     )
            if not bool(p.get("execute_grasp", False)):
                rospy.loginfo("  execute_grasp=false，跳过抓取，继续下一个箱子")
                continue
            _EXECUTORS[mode](plan, hardware)
            rospy.loginfo("抓取完成，箱体保持夹持，双臂进入导航保持位")

            # ---- 步骤 7: 导航到放置点 ----
            rospy.loginfo("[步骤 7/8] 导航到放置点")
            self._navigate_to(hardware, {
                "x": p["place_x"],
                "y": p["place_y"],
                "theta_deg": p["place_theta_deg"],
            }, "place_%d" % index)

            # ---- 步骤 8: 放下箱体 ----
            rospy.loginfo("[步骤 8/8] 放下箱体并复位")
            box_width = float(action.get("box_width", p.get("box_width", 0.40)))
            self._execute_place(hardware, p, box_width)

            rospy.loginfo(">>> 第 %d/%d 个箱子完成 <<<", index + 1, len(sequence))

        rospy.loginfo("=" * 60)
        rospy.loginfo("全流程完成！共处理 %d 个箱子", len(sequence))

    # ==================================================================
    # 导航
    # ==================================================================
    def _navigate_to(self, hardware, pose, label):
        """导航到目标点并阻塞等待到达。"""
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
            math.radians(float(pose["theta_deg"])),
            options=options,
        )
        if not result.success or not result.data or not result.data.get("task_id"):
            raise RuntimeError("%s 导航下发失败: %s" % (label, result.message))

        rospy.loginfo("  导航任务已下发: task_id=%s", result.data["task_id"])
        rospy.sleep(1)
        arrived = hardware.check_arrived_jibot(
            str(result.data["task_id"]),
            blocking=True,
            timeout=float(self.params.get("nav_arrival_timeout_sec", 120.0)),
        )
        if not arrived.success or not (arrived.data or {}).get("arrived", False):
            msg = getattr(arrived, "message", "") or ""
            if "interrupted" in str(msg).lower():
                rospy.logwarn("  导航被中断，重试一次...")
                time.sleep(2.0)
                result2 = hardware.base_move_to_target_jibot(
                    float(pose["x"]), float(pose["y"]),
                    math.radians(float(pose["theta_deg"])),
                    options=options,
                )
                if not result2.success or not result2.data or not result2.data.get("task_id"):
                    raise RuntimeError("%s 重试导航下发失败: %s" % (label, result2.message))
                arrived2 = hardware.check_arrived_jibot(
                    str(result2.data["task_id"]),
                    blocking=True,
                    timeout=float(self.params.get("nav_arrival_timeout_sec", 120.0)),
                )
                if not arrived2.success or not (arrived2.data or {}).get("arrived", False):
                    raise RuntimeError("%s 重试后仍未到达: %s" % (label, arrived2.message))
            else:
                raise RuntimeError("%s 未到达: %s" % (label, arrived.message))
        rospy.loginfo("  已到达目标点")

    # ==================================================================
    # 视觉规划器
    # ==================================================================
    def _call_vision_planner(self, p):
        """调用视觉规划器服务，返回抓取步骤序列。"""
        service_name = str(p.get("binplanner_service_name",
                                 "/lingbot/run_decide_with_stack_pose"))
        timeout = float(p.get("binplanner_timeout_sec", 120.0))

        # 设置规划器请求参数
        rospy.set_param(service_name + "/request/front_x",
                        float(p.get("decide_front_x", 1.05)))
        rospy.set_param(service_name + "/request/front_y",
                        float(p.get("decide_front_y", -0.15)))
        rospy.set_param(service_name + "/request/yaw_deg",
                        float(p.get("decide_yaw_deg", 0.0)))

        rospy.loginfo("  等待视觉规划器服务: %s (timeout=%.1fs)", service_name, timeout)
        rospy.wait_for_service(service_name, timeout=timeout)

        service_class = get_service_class_by_name(service_name)
        if service_class is None:
            raise RuntimeError("无法解析视觉规划器服务类型: %s" % service_name)

        rospy.loginfo("  调用视觉规划器...")
        response = rospy.ServiceProxy(service_name, service_class)()
        sequence = _decode_sequence(response)
        rospy.loginfo("  视觉规划器返回 %d 个步骤", len(sequence))
        return sequence

    # ==================================================================
    # TF 检查
    # ==================================================================
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
            rospy.loginfo("  TF %s -> %s 已连通", target, source)
        except Exception as exc:
            raise RuntimeError("TF %s -> %s 等待超时: %s" % (target, source, exc))
        finally:
            tf_listener.unregister()

    # ==================================================================
    # 6D 位姿识别 + 轨迹规划
    # ==================================================================
    def _detect_6d_and_plan(self, step, action, p):
        """6D 位姿识别，将结果作为机械臂轨迹规划终点。"""
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
        rospy.loginfo("  识别到箱子 #%d, 6D位姿: pos=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)",
                      int(step.get("basket_id", target_index)),
                      pose.x, pose.y, pose.z,
                      math.degrees(pose.roll), math.degrees(pose.pitch),
                      math.degrees(pose.yaw))

        # 将 6D 位姿转换为 DetectedPose，用于轨迹规划
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

        # 构建机械臂轨迹规划参数
        args = BasketVisionCarryMove(
            "full_pipeline_plan", "全流程轨迹规划", "", p
        )._make_args()
        args.box_layer = int(action.get("box_layer", 1)) if isinstance(action, dict) else 1

        args.box_layer = int(action.get("box_layer", 1))

        # 优先使用当前点位的箱子宽度
        args.box_width = float(
            action.get("box_width", p.get("box_width", 0.40))
        )

        rospy.loginfo(
            "  当前点位动作: mode=%s, box_width=%.2fm",
            action.get("grasp_mode"),
            args.box_width,
        )


        # 以 6D 位姿为终点生成抓取轨迹
        plan = build_plan(detected, args)
        plan.use_whole_body_ik = bool(p.get("use_whole_body_ik", False))
        plan.grasp_pose = detected  # 记录 6D 位姿

        # 验证轨迹安全性
        validate_plan(plan, args)
        rospy.loginfo("  轨迹规划完成，已通过安全校验")
        return plan

    # ==================================================================
    # 放置
    # ==================================================================
    def _execute_place(self, hardware, p, box_width):
        if not bool(p.get("execute_place", True)):
            rospy.loginfo("  execute_place=false，跳过放置")
            return

        place = BasketPlaceAfterNavMove(
            "full_pipeline_place", "全流程放置", "", p
        )
        place._execute(hardware, box_width=box_width)
        rospy.loginfo("  放置与复位完成")
