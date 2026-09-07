# -*- coding: utf-8 -*-
"""机器人版本：两层顶部多箱识别、批量规划、直接抓取搬运。

直接替换机器人原 ``full_pipeline_vision_grasp_move.py``。
不再调用视觉规划器、检测点或 ``/infer_basket_pose``：每个观测面只调用一次
``/infer_top_basket_ids``，用其确认箱子数量和索引；轨迹规划使用按层、按抓取模式
指定的固定抓取基准位姿。
"""

import math
import os
import time
from dataclasses import dataclass, replace

import rospy
from py_trees.common import Access, Status

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

# 顶部服务仍用于确认当前面识别到的箱子数量，以及 detected_index 的有效性。
# 但不直接使用其 pose6d 规划：实机标定后的固定基准点由这里统一管理。
# 单位：m / rad。
_LAYER1_FIXED_GRASP_POSE = (0.738, -0.060, 0.854)
_LAYER2_LEFT_FIRST_FIXED_GRASP_POSE = (0.738, -0.060, 0.654)
_LAYER2_RIGHT_FIRST_FIXED_GRASP_POSE = (0.688, -0.060, 0.624)

# 机器人已安装小箱/第二层大箱动作时，自动支持；没有这些文件也不会影响基本左右先抬模式。
try:
    from orchestration.nodes.basket_vision_small_box_carry_move import (
        BasketVisionSmallBoxLeftCarryMove, BasketVisionSmallBoxRightCarryMove,
    )
    _EXECUTORS.update({
        "small_box_left_lift_right_carry": BasketVisionSmallBoxLeftCarryMove._execute,
        "small_box_right_lift_left_carry": BasketVisionSmallBoxRightCarryMove._execute,
    })
except ImportError:
    pass
try:
    from orchestration.nodes.basket_vision_layer2_large_box_carry_move import (
        BasketVisionLayer2LargeBoxLeftCarryMove,
        BasketVisionLayer2LargeBoxRightCarryMove,
    )
    _EXECUTORS.update({
        "layer2_large_box_left_lift_right_carry": BasketVisionLayer2LargeBoxLeftCarryMove._execute,
        "layer2_large_box_right_lift_left_carry": BasketVisionLayer2LargeBoxRightCarryMove._execute,
    })
except ImportError:
    pass


@dataclass
class _Job:
    name: str
    config: dict
    plan: object


@define_manifest(
    label="两层顶部识别批量抓取搬运",
    category=["perception", "motion", "chassis", "arm"],
    tree_type="depalletize_bin",
    description="每面一次顶部多箱6D识别，批量规划后直接抓取、放置和下一箱循环",
    params=[], inputs=[], outputs=[],
)
class FullPipelineGraspMove(BaseAction):
    """替代机器人旧单箱检测点流程的两层批量流程。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._status = None

    def initialise(self):
        self._status = None

    def update(self):
        if self._status is not None:
            return self._status
        if _DRY_RUN:
            self.feedback_message = "dry-run 两层顶部批量抓取"
            self._status = Status.SUCCESS
            return self._status
        try:
            self._run()
            self.feedback_message = "两层顶部批量抓取完成"
            self._status = Status.SUCCESS
        except Exception as exc:
            self.feedback_message = "两层顶部批量抓取失败: %s" % exc
            rospy.logerr(self.feedback_message)
            self._status = Status.FAILURE
        return self._status

    def _run(self):
        jobs_by_layer = self._board_value("top_basket_jobs_by_layer", {})
        if not isinstance(jobs_by_layer, dict) or not jobs_by_layer:
            raise ValueError("缺少 top_basket_jobs_by_layer 配置")
        if bool(self.params.get("execute_motion", False)) and bool(self.params.get("skip_navigation", False)):
            raise ValueError("真机 execute_motion=true 时 skip_navigation 必须为 false")

        hardware = get_shared_hardware()
        for raw_layer in self._board_value("layer_execution_order", [1, 2]):
            layer = int(raw_layer)
            side_jobs = jobs_by_layer.get(str(layer), jobs_by_layer.get(layer))
            if not isinstance(side_jobs, dict):
                raise KeyError("top_basket_jobs_by_layer 缺少第 %d 层" % layer)
            for side in ("front", "rear"):
                specs = side_jobs.get(side, [])
                if specs:
                    self._run_one_side(hardware, layer, side, specs)

    def _run_one_side(self, hardware, layer, side, specs):
        # 从正面切到背面前，先收回腰部到相机识别姿态；识别完成后会由
        # _move_to_layer_ready_pose() 重新抬到当前层的抓取预备位。
        if side == "rear":
            self._move_to_observation_torso(hardware, side)
        self._navigate_to(hardware, self._observation_pose(side), "layer%d_%s_observe" % (layer, side))
        self._wait_camera_tf()
        baskets = self._infer_top_baskets()
        jobs = self._build_all_jobs(layer, side, specs, baskets)
        rospy.loginfo("第%d层%s面：%d 个箱子已全部完成分类、补偿和轨迹校验", layer, side, len(jobs))

        if not bool(self.params.get("execute_motion", False)):
            rospy.logwarn("execute_motion=false：只做识别与批量规划校验，不发送任何运动指令")
            return
        # 在腰臂动作前先验证所有导航键，避免抬升后才发现配置缺失。
        for job in jobs:
            self._nav_pose(job.config["nav_pose_key"])
            self._place_pose(job.config)

        self._move_to_layer_ready_pose(hardware, layer)
        nav_key_occurrences = {}
        for index, job in enumerate(jobs):
            configured_nav_key = str(job.config["nav_pose_key"])
            occurrence = nav_key_occurrences.get(configured_nav_key, 0) + 1
            nav_key_occurrences[configured_nav_key] = occurrence
            actual_nav_key = self._remap_nav_key_for_occurrence(configured_nav_key, occurrence)
            grasp_nav_pose = self._nav_pose(actual_nav_key)
            if actual_nav_key != configured_nav_key:
                rospy.loginfo(
                    "%s: 第%d次使用 %s，二次重定位导航到 %s；抓取动作仍使用原导航点动作库配置",
                    job.name, occurrence, configured_nav_key, actual_nav_key,
                )
            self._navigate_to(hardware, grasp_nav_pose, "grasp_%s" % job.name)
            if bool(self.params.get("execute_grasp", False)):
                _EXECUTORS[job.config["grasp_mode"]](job.plan, hardware)

                # 只有实际抓住箱子后，才需要退出垛体、前往放置点和执行放置。
                self._navigate_to(
                    hardware,
                    self._post_grasp_retreat_pose(grasp_nav_pose),
                    "retreat_%s" % job.name,
                )
                self._navigate_to(hardware, self._place_pose(job.config), "place_%s" % job.name)
                if bool(self.params.get("execute_place", False)):
                    self._execute_place(hardware, float(job.config.get("box_width", self.params.get("box_width", 0.70))))
                else:
                    rospy.loginfo("execute_place=false：跳过 %s 的放置/撤离/复位动作", job.name)
            else:
                rospy.loginfo("execute_grasp=false：跳过 %s 的抓取、后退、放置及放置导航", job.name)
            if index < len(jobs) - 1:
                # 放置后只回本层预备位，直接去下一抓取点；不再前往检测点。
                self._move_to_layer_ready_pose(hardware, layer)

    def _infer_top_baskets(self):
        client = BasketVisionClient({
            "top_basket_service": self.params.get("top_basket_service", "/infer_top_basket_ids"),
            "timeout": float(self.params.get("top_detection_timeout_s", 20.0)),
            "save_images": bool(self.params.get("save_vision_images", True)),
        })
        result = client.infer_top_basket()
        baskets = (getattr(result, "data", None) or {}).get("baskets", [])
        if not getattr(result, "success", False) or not baskets:
            raise RuntimeError("/infer_top_basket_ids 失败: %s" % getattr(result, "message", "无箱体"))
        return baskets

    def _build_all_jobs(self, layer, side, specs, baskets):
        jobs, used = [], set()
        for ordinal, config in enumerate(specs):
            for key in ("detected_index", "nav_pose_key"):
                if key not in config:
                    raise ValueError("第%d层%s面任务%d缺少 %s" % (layer, side, ordinal, key))
            # 抓取模式、箱宽和层动作参数由导航点动作库决定；视觉任务只负责
            # detected_index、导航点和位姿补偿，不可直接覆盖抓取模式。
            config = self._config_from_nav_pose(config, layer, side, ordinal)
            index = int(config["detected_index"])
            if index in used or index < 0 or index >= len(baskets):
                raise ValueError("第%d层%s面 detected_index=%d 无效或重复；服务返回%d个箱子" % (layer, side, index, len(baskets)))
            if (config["grasp_mode"] not in _EXECUTORS
                    and bool(self.params.get("execute_grasp", False))):
                raise ValueError("机器人未安装或未注册抓取模式: %s" % config["grasp_mode"])
            used.add(index)
            # fixed = self._fixed_pose_for_job(layer, config, index)
            use_vision_pose = self._bool_board_value(
                "use_vision_grasp_pose",
                False,
            )

            if use_vision_pose:
                # 使用 /infer_top_basket_ids 返回的第 index 个 6D 位姿
                pose6d = baskets[index].get("pose6d")
                if pose6d is None:
                    raise ValueError(
                        "顶部服务第%d项没有 pose6d，无法使用视觉抓取位姿" % index
                    )

                source_pose = DetectedPose(
                    float(pose6d.x),
                    float(pose6d.y),
                    float(pose6d.z),
                    float(pose6d.roll),
                    float(pose6d.pitch),
                    float(pose6d.yaw),
                    index,
                    "base_link",
                    "base_link",
                )
                pose_source = "vision_pose"
            else:
                # 保留你当前的分层固定点逻辑
                source_pose = self._fixed_pose_for_job(layer, config, index)
                pose_source = "fixed_pose"

            adjusted = self._apply_offset(
                source_pose,
                config.get("pose_offset", [0, 0, 0, 0, 0, 0]),
            )

            plan = self._plan_from_pose(adjusted, config, layer)
            name = str(config.get("name", "%s_%d" % (side, index)))
            rospy.loginfo(
                "  %s: index=%d class=%s %s -> (%.3f, %.3f, %.3f)",
                name,
                index,
                config.get("class_name", "未分类"),
                pose_source,
                adjusted.x,
                adjusted.y,
                adjusted.z,
            )
            jobs.append(_Job(name, config, plan))
        return jobs

    def _config_from_nav_pose(self, spec, layer, side, ordinal):
        """以 nav_pose_key 为唯一动作选择键，合并导航点对应的动作参数。"""
        library = self._board_value("grasp_action_library", {})
        if not isinstance(library, dict):
            raise ValueError("grasp_action_library 必须是字典")
        nav_key = str(spec["nav_pose_key"])
        action = library.get(nav_key)
        if not isinstance(action, dict):
            raise KeyError("grasp_action_library 缺少导航点 %s（第%d层%s面任务%d）" % (nav_key, layer, side, ordinal))
        if not action.get("grasp_mode"):
            raise ValueError("grasp_action_library.%s 缺少 grasp_mode" % nav_key)

        # spec 中的 pose_offset / detected_index 保持任务自身设置；动作相关字段
        # 统一以导航点动作库为准，避免与视觉任务条目发生两份配置冲突。
        config = dict(spec)
        config.update(action)
        config["nav_pose_key"] = nav_key
        return config

    def _remap_nav_key_for_occurrence(self, nav_key, occurrence):
        """支持 ``原导航键#第几次`` 到二次重定位导航键的映射。"""
        table = self._board_value("nav_key_remap_by_occurrence", {})
        if table is None:
            return nav_key
        if not isinstance(table, dict):
            raise ValueError("nav_key_remap_by_occurrence 必须是字典")
        remap_key = "%s#%d" % (nav_key, occurrence)
        remapped = table.get(remap_key, nav_key)
        if not isinstance(remapped, str) or not remapped:
            raise ValueError("nav_key_remap_by_occurrence.%s 必须是非空导航键" % remap_key)
        return remapped

    @staticmethod
    def _fixed_pose_for_job(layer, config, detected_index):
        """按层和左右先抬动作选择固定抓取基准；pose_offset 仍在其后叠加。"""
        if layer == 1:
            x, y, z = _LAYER1_FIXED_GRASP_POSE
        elif layer == 2:
            grasp_mode = str(config.get("grasp_mode", ""))
            if grasp_mode == "left_lift_right_carry":
                x, y, z = _LAYER2_LEFT_FIRST_FIXED_GRASP_POSE
            elif grasp_mode == "right_lift_left_carry":
                x, y, z = _LAYER2_RIGHT_FIRST_FIXED_GRASP_POSE
            else:
                raise ValueError("第2层不支持的固定抓取模式: %s" % grasp_mode)
        else:
            raise ValueError("第%d层没有配置固定抓取基准位姿" % layer)
        return DetectedPose(x, y, z, 0.0, 0.0, 0.0, detected_index, "base_link", "base_link")

    @staticmethod
    def _apply_offset(pose, offset):
        if not isinstance(offset, (list, tuple)) or len(offset) not in (3, 6):
            raise ValueError("pose_offset 必须是 [dx,dy,dz] 或 [dx,dy,dz,droll_deg,dpitch_deg,dyaw_deg]")
        value = [float(x) for x in offset] + [0.0] * (6 - len(offset))
        return replace(pose, x=pose.x + value[0], y=pose.y + value[1], z=pose.z + value[2], roll=pose.roll + math.radians(value[3]), pitch=pose.pitch + math.radians(value[4]), yaw=pose.yaw + math.radians(value[5]))

    def _plan_from_pose(self, pose, config, layer):
        args = BasketVisionCarryMove("top_plan", "顶部多箱轨迹规划", "", self.params)._make_args()
        args.box_layer = int(config.get("box_layer", layer))
        for key in ("box_width", "grasp_x_offset", "grasp_x_offset_right", "grasp_z_offset", "grasp_y_offset", "grasp_y_offset_right", "side_clearance", "approach_side_distance", "approach_height", "lift_height", "pull_distance"):
            if key in config:
                setattr(args, key, float(config[key]))
        plan = build_plan(pose, args)
        plan.use_whole_body_ik = bool(config.get("use_whole_body_ik", self.params.get("use_whole_body_ik", False)))
        plan.grasp_pose = pose
        validate_plan(plan, args)
        return plan

    def _move_to_layer_ready_pose(self, hardware, layer):
        table = self._board_value("layer_ready_poses", {})
        config = table.get(str(layer), table.get(layer)) if isinstance(table, dict) else None
        if not isinstance(config, dict):
            raise KeyError("layer_ready_poses 缺少第%d层" % layer)
        torso = self._torso(config.get("torso"), "layer_ready_poses.%d.torso" % layer)
        left = self._arm(config.get("left_arm"), "layer_ready_poses.%d.left_arm" % layer)
        right = self._arm(config.get("right_arm"), "layer_ready_poses.%d.right_arm" % layer)
        torso_time, arm_time = float(config.get("torso_duration_s", 4.0)), float(config.get("arm_duration_s", 4.0))
        settle = max(0.0, float(config.get("settle_s", 0.5)))
        moved_ready_pose = False
        if bool(self.params.get("execute_torso_ready", True)):
            result = hardware.send_torso_pose_timed(x=torso["x"], z=torso["z"], yaw=torso["yaw"], pitch=torso["pitch"], desire_time=torso_time)
            self._wait_result(result, torso_time, settle, "第%d层腰部预备位" % layer)
            moved_ready_pose = True
        else:
            rospy.loginfo("execute_torso_ready=false：跳过第%d层腰部预备位", layer)
        if bool(self.params.get("execute_arm_ready", True)):
            result = hardware.send_arm_ee_local_timed(left, right, desire_time=arm_time)
            self._wait_result(result, arm_time, settle, "第%d层双臂预备位" % layer)
            moved_ready_pose = True
        else:
            rospy.loginfo("execute_arm_ready=false：跳过第%d层双臂预备位", layer)
        if moved_ready_pose:
            result = hardware.set_focus_ee(focus_ee=False)
            if result is None or not getattr(result, "success", False):
                raise RuntimeError("set_focus_ee(False)失败: %s" % getattr(result, "message", "无返回值"))

    def _move_to_observation_torso(self, hardware, side):
        """在指定观测面识别前切换到相机标定时的腰部姿态。"""
        table = self._board_value("observation_torso_poses", {})
        config = table.get(side) if isinstance(table, dict) else None
        if config is None:
            return
        torso = self._torso(config, "observation_torso_poses.%s" % side)
        if not bool(self.params.get("execute_motion", False)):
            rospy.loginfo("execute_motion=false：跳过%s观测腰部姿态", side)
            return
        if not bool(self.params.get("execute_torso_ready", True)):
            rospy.loginfo("execute_torso_ready=false：跳过%s观测腰部姿态", side)
            return
        duration = float(config.get("duration_s", 4.0))
        settle = max(0.0, float(config.get("settle_s", 0.5)))
        result = hardware.send_torso_pose_timed(
            x=torso["x"], z=torso["z"], yaw=torso["yaw"], pitch=torso["pitch"],
            desire_time=duration,
        )
        self._wait_result(result, duration, settle, "%s面观测腰部姿态" % side)

    def _navigate_to(self, hardware, pose, label):
        if not bool(self.params.get("execute_motion", False)):
            rospy.loginfo("execute_motion=false，跳过导航 %s", label)
            return
        if bool(self.params.get("skip_navigation", False)):
            rospy.logwarn("skip_navigation=true，跳过导航 %s", label)
            return
        options = MoveToTargetOptions(avoid_enabled=bool(self.params.get("nav_avoid_enabled", False)), avoid_distance=float(self.params.get("nav_avoid_distance", 0.5)), linear_velocity=float(self.params.get("nav_linear_velocity", 0.3)), angular_velocity=float(self.params.get("nav_angular_velocity", 0.5)), position_threshold=float(self.params.get("nav_position_threshold", 0.08)), angle_threshold=float(self.params.get("nav_angle_threshold", 0.1)), allow_rotation=bool(self.params.get("nav_allow_rotation", True)))
        retry_count = max(0, int(self.params.get("nav_retry_count", 2)))
        retry_interval = max(0.0, float(self.params.get("nav_retry_interval_sec", 3.0)))
        total_attempts = retry_count + 1
        last_error = "无返回值"
        for attempt in range(1, total_attempts + 1):
            result = hardware.base_move_to_target_jibot(float(pose["x"]), float(pose["y"]), math.radians(float(pose["theta_deg"])), options=options)
            if not getattr(result, "success", False) or not (getattr(result, "data", None) or {}).get("task_id"):
                last_error = "导航下发失败: %s" % getattr(result, "message", "无返回值")
            else:
                arrived = hardware.check_arrived_jibot(str(result.data["task_id"]), blocking=True, timeout=float(self.params.get("nav_arrival_timeout_sec", 120.0)))
                if getattr(arrived, "success", False) and (getattr(arrived, "data", None) or {}).get("arrived", False):
                    if attempt > 1:
                        rospy.loginfo("%s：第%d次导航尝试到达成功", label, attempt)
                    return
                last_error = "未到达: %s" % getattr(arrived, "message", "无返回值")

            if attempt < total_attempts:
                rospy.logwarn("%s：第%d/%d次导航失败（%s），%.1f秒后重试", label, attempt, total_attempts, last_error, retry_interval)
                time.sleep(retry_interval)

        raise RuntimeError("%s 导航连续%d次失败: %s" % (label, total_attempts, last_error))

    def _observation_pose(self, side):
        table = self._board_value("observation_pose_table", {})
        return self._nav(table.get(side), "observation_pose_table.%s" % side)

    def _nav_pose(self, key):
        table = self._board_value("nav_pose_table", {})
        if key not in table:
            raise KeyError("nav_pose_table 缺少 %s" % key)
        return self._nav(table[key], "nav_pose_table.%s" % key)

    def _place_pose(self, config):
        key = config.get("place_pose_key")
        if key:
            table = self._board_value("place_pose_table", {})
            if key not in table:
                raise KeyError("place_pose_table 缺少 %s" % key)
            return self._nav(table[key], "place_pose_table.%s" % key)
        return self._nav(self._board_value("place_pose"), "place_pose")

    def _board_value(self, key, default=None):
        """兼容字典参数被旧版行为树展开后的运行环境。

        新版行为树会把 board 中的字典原样传入 ``self.params``；部分旧版
        工厂只保留 ``a.b.c`` 形式的叶子参数，使根键变成 ``${a}`` 或缺失。
        这里优先使用节点参数，必要时回退到全局黑板中的原始 board 值。
        """
        missing = object()
        value = self.params.get(key, missing)
        unresolved_macro = isinstance(value, str) and value == "${%s}" % key
        if value is not missing and not unresolved_macro:
            return value
        try:
            self.global_blackboard.register_key(key=key, access=Access.READ)
            if self.global_blackboard.exists(key):
                return self.global_blackboard.get(key)
        except Exception as exc:
            rospy.logwarn("读取 board 参数 %s 失败: %s", key, exc)
        return default
    def _bool_board_value(self, key, default=False):
        """读取 board 布尔开关，兼容旧树未展开的 ${key} 宏。"""
        value = self._board_value(key, default)

        if isinstance(value, bool):
            return value

        if isinstance(value, (int, float)):
            return bool(value)

        if isinstance(value, str):
            normalized = value.strip().lower()

            if normalized in ("true", "1", "yes", "on"):
                return True

            if normalized in ("false", "0", "no", "off", ""):
                return False

            if normalized == "${%s}" % key:
                return bool(default)

        raise ValueError("%s 必须是布尔值，实际为 %r" % (key, value))
    def _post_grasp_retreat_pose(self, grasp_nav_pose):
        """以抓取导航点朝向为前方，后退到垛体外。"""
        distance = float(self.params.get("post_grasp_retreat_distance_m", 0.60))
        if distance < 0.0:
            raise ValueError("post_grasp_retreat_distance_m 不能小于 0")
        heading = math.radians(float(grasp_nav_pose["theta_deg"]))
        return {
            "x": float(grasp_nav_pose["x"]) - distance * math.cos(heading),
            "y": float(grasp_nav_pose["y"]) - distance * math.sin(heading),
            "theta_deg": float(grasp_nav_pose["theta_deg"]),
        }

    @staticmethod
    def _nav(value, name):
        if not isinstance(value, dict) or any(k not in value for k in ("x", "y", "theta_deg")):
            raise ValueError("%s 必须为 {x,y,theta_deg}" % name)
        try:
            pose = {k: float(value[k]) for k in ("x", "y", "theta_deg")}
        except (TypeError, ValueError):
            raise ValueError("%s 尚未填写；请填入 x、y、theta_deg 后再启用第二层真机导航" % name)
        if not all(math.isfinite(item) for item in pose.values()):
            raise ValueError("%s 包含非有限数值" % name)
        return pose

    @staticmethod
    def _torso(value, name):
        if not isinstance(value, dict) or any(k not in value for k in ("x", "z", "yaw", "pitch")):
            raise ValueError("%s 必须为 {x,z,yaw,pitch}" % name)
        return {k: float(value[k]) for k in ("x", "z", "yaw", "pitch")}

    @staticmethod
    def _arm(value, name):
        if not isinstance(value, (list, tuple)) or len(value) != 6:
            raise ValueError("%s 必须为6D末端位姿" % name)
        return [float(x) for x in value]

    @staticmethod
    def _wait_result(result, fallback, settle, label):
        if result is None or not getattr(result, "success", False):
            raise RuntimeError("%s失败: %s" % (label, getattr(result, "message", "无返回值")))
        actual = (getattr(result, "data", None) or {}).get("actual_time", fallback)
        time.sleep(max(0.0, float(actual)) + settle)

    def _execute_place(self, hardware, box_width):
        place = BasketPlaceAfterNavMove("full_pipeline_place", "全流程放置", "", self.params)
        place._execute(hardware, box_width=box_width)

    def _wait_camera_tf(self):
        import tf2_ros
        target = str(self.params.get("camera_tf_target_frame", "base_link"))
        source = str(self.params.get("camera_tf_source_frame", "camera_color_optical_frame"))
        buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        listener = tf2_ros.TransformListener(buffer)
        try:
            buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(float(self.params.get("camera_tf_timeout_sec", 15.0))))
        finally:
            listener.unregister()
