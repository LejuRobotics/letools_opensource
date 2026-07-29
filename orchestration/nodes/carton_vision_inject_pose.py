# -*- coding: utf-8 -*-
"""CartonVisionInjectPose：调用视觉服务获取纸箱位姿，动态注入 waypoints 到黑板。

在 timed_grasp 节点之前执行，调用 GDRNPP 视觉服务获取 embodied_compat.t_base，
根据可配置的偏移参数构造两段 waypoints（接近 + 抓取），写入黑板覆盖静态值。
"""

import os

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.nodes.carton_sequence import build_pick_summary
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

try:
    import rospy
    HAS_ROSPY = True
except ImportError:
    HAS_ROSPY = False


@define_manifest(
    label="视觉位姿注入",
    category=["perception", "vision"],
    tree_type="studio_smoke",
    description=(
        "调用 GDRNPP 视觉服务获取纸箱位姿(t_base)，"
        "构造两段 waypoints(接近+抓取)写入黑板，覆盖 timed_grasp 的静态值。"
    ),
    params=[
        {"name": "left_board_key", "type": "string",
         "default": "timed_grasp_1_left_waypoints",
         "description": "左手 waypoints 写入的黑板 key"},
        {"name": "right_board_key", "type": "string",
         "default": "timed_grasp_1_right_waypoints",
         "description": "右手 waypoints 写入的黑板 key"},
        {"name": "approach_z_offset", "type": "float", "default": "0.08",
         "description": "接近点相对抓取点的 z 抬高量(米)"},
        {"name": "grasp_z_offset", "type": "float", "default": "0.24",
         "description": "法兰到吸盘尖端的 z 补偿量(米)"},
        {"name": "x_offset", "type": "float", "default": "0.1",
         "description": "x 方向前移补偿量(米)"},
        {"name": "left_y_offset", "type": "float", "default": "0.1",
         "description": "左手臂 y 方向偏移(米)"},
        {"name": "right_y_offset", "type": "float", "default": "-0.1",
         "description": "右手臂 y 方向偏移(米)"},
        {"name": "yaw", "type": "float", "default": "0.0",
         "description": "waypoints 姿态 yaw(度)"},
        {"name": "pitch", "type": "float", "default": "-90.0",
         "description": "waypoints 姿态 pitch(度)"},
        {"name": "roll", "type": "float", "default": "0.0",
         "description": "waypoints 姿态 roll(度)"},
        # ---- 不满垛字段注入开关与黑板 key ----
        {"name": "enable_top_carton_fields", "type": "bool", "default": "true",
         "description": "是否同时把 estimated_total/top_ids/pick_sequence 等不满垛字段写入黑板"},
        {"name": "box_type_override", "type": "string", "default": "",
         "description": "可选箱型覆盖(如 type1/type3)；留空则从 message 解析 carton_type"},
        {"name": "top_carton_message_key", "type": "string",
         "default": "top_carton_message",
         "description": "原始 message 字符串写入的黑板 key"},
        {"name": "carton_type_key", "type": "string",
         "default": "carton_type",
         "description": "箱型写入的黑板 key"},
        {"name": "top_orientation_key", "type": "string",
         "default": "top_orientation",
         "description": "顶部朝向写入的黑板 key"},
        {"name": "top_ids_key", "type": "string",
         "default": "top_ids",
         "description": "顶层纸箱局部 id 列表写入的黑板 key"},
        {"name": "estimated_total_key", "type": "string",
         "default": "estimated_total",
         "description": "视觉识别总数写入的黑板 key"},
        {"name": "empty_slots_key", "type": "string",
         "default": "empty_slots",
         "description": "缺箱数写入的黑板 key"},
        {"name": "is_partial_stack_key", "type": "string",
         "default": "is_partial_stack",
         "description": "是否不满垛标志写入的黑板 key"},
        {"name": "pick_sequence_key", "type": "string",
         "default": "pick_sequence",
         "description": "完整抓取序列(box_index 列表)写入的黑板 key"},
    ],
    inputs=[],
    outputs=[],
)
class CartonVisionInjectPose(BaseAction):
    """视觉位姿注入节点

    执行流程:
      1. 调用 get_shared_hardware().infer_carton_pose() (ROS 服务 /infer_carton_pose)
      2. 从 result.data["embodied_compat"]["t_base"] 提取 [x, y, z]
      3. 构造 left_waypoints = [[x, y+left_y_offset, z+approach_z_offset, yaw, pitch, roll],
                                 [x, y+left_y_offset, z, yaw, pitch, roll]]
      4. 同理构造 right_waypoints
      5. 写入黑板覆盖 timed_grasp 的静态值
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False
        self._success = False

    def initialise(self):
        self._done = False
        self._success = False

    def update(self):
        if self._done:
            return Status.SUCCESS if self._success else Status.FAILURE

        if _DRY_RUN:
            self._done = True
            self._success = True
            self.feedback_message = "dry-run carton_vision_inject_pose"
            return Status.SUCCESS

        # --- 1. 调用视觉服务 ---
        try:
            hw = get_shared_hardware()
            result = hw.infer_carton_pose()
        except Exception as e:
            self.feedback_message = f"carton_vision_inject: hardware call failed: {e}, 回退到静态值"
            self._done = True
            self._success = True
            if HAS_ROSPY:
                rospy.logwarn(f"[CartonVisionInjectPose] 视觉服务调用失败，回退到静态值: {e}")
            return Status.SUCCESS

        if not result.success:
            self.feedback_message = (
                result.message or "carton_vision_inject: infer failed, 回退到静态值"
            )
            self._done = True
            self._success = True
            if HAS_ROSPY:
                rospy.logwarn(f"[CartonVisionInjectPose] 视觉推理失败，回退到静态值: {result.message}")
            return Status.SUCCESS

        # --- 2. 提取 embodied_compat.t_base ---
        data = result.data or {}
        embodied = data.get("embodied_compat", {})
        t_base = embodied.get("t_base")

        if t_base is None or len(t_base) < 3:
            self.feedback_message = (
                f"carton_vision_inject: no valid t_base, "
                f"source={embodied.get('source', 'unknown')}, 回退到静态值"
            )
            self._done = True
            self._success = True
            if HAS_ROSPY:
                rospy.logwarn(f"[CartonVisionInjectPose] 无有效 t_base，回退到静态值")
            return Status.SUCCESS

        tx, ty, tz = float(t_base[0]), float(t_base[1]), float(t_base[2])
        # x 方向前移补偿
        tx += float(self.params.get("x_offset", 0.18))

        left_y_offset = float(self.params.get("left_y_offset", 0.1))
        right_y_offset = float(self.params.get("right_y_offset", -0.1))
        yaw = float(self.params.get("yaw", 0.0))
        pitch = float(self.params.get("pitch", -90.0))
        roll = float(self.params.get("roll", 0.0))

        # 法兰到吸盘尖端的 z 补偿
        grasp_z = tz + float(self.params.get("grasp_z_offset", 0.24))
        approach_z = grasp_z + float(self.params.get("approach_z_offset", 0.08))

        left_waypoints = [
            [tx, ty + left_y_offset, approach_z, yaw, pitch, roll],
            [tx, ty + left_y_offset, grasp_z, yaw, pitch, roll],
        ]
        right_waypoints = [
            [tx, ty + right_y_offset, approach_z, yaw, pitch, roll],
            [tx, ty + right_y_offset, grasp_z, yaw, pitch, roll],
        ]

        # --- 5. 写入黑板 ---
        left_key = str(self.params.get("left_board_key", "timed_grasp_1_left_waypoints"))
        right_key = str(self.params.get("right_board_key", "timed_grasp_1_right_waypoints"))

        for key, value in [(left_key, left_waypoints), (right_key, right_waypoints)]:
            try:
                self.global_blackboard.register_key(
                    key=key, access=py_trees.common.Access.WRITE
                )
            except AttributeError:
                pass
            try:
                self.global_blackboard.set(key, value)
            except AttributeError as e:
                self.feedback_message = f"carton_vision_inject: blackboard set failed: {e}"
                self._done = True
                self._success = False
                return Status.FAILURE

        # --- 5b. 写入不满垛字段（estimated_total / top_ids / pick_sequence 等） ---
        self._inject_top_carton_fields(result, data)

        # --- 6. 写入完成（ArmEeTimedCmdMove.initialise() 会通过 board_key 运行时读取）---

        if HAS_ROSPY:
            rospy.loginfo(
                "[CartonVisionInjectPose] t_base=({:.4f}, {:.4f}, {:.4f}) -> "
                "left_key={}, right_key={}".format(tx, ty, tz, left_key, right_key)
            )

        self.feedback_message = (
            f"carton_vision_inject: t_base=({tx:.4f}, {ty:.4f}, {tz:.4f}), "
            f"injected to {left_key} & {right_key}"
        )
        self._done = True
        self._success = True
        return Status.SUCCESS

    def _inject_top_carton_fields(self, result, data) -> None:
        """把 /infer_top_carton_ids 返回的 message 解析为不满垛摘要并写入黑板。

        仅当 message 包含不满垛信息(top ids / estimated total)时才解析，
        /infer_carton_pose (single 模式) 的 message 不含这些字段，会跳过。

        解析失败时仅告警，不影响主流程（保持与现有视觉失败回退一致的容错哲学）。
        """
        if str(self.params.get("enable_top_carton_fields", "true")).lower() not in (
            "1", "true", "yes",
        ):
            return

        message = ""
        try:
            message = str((data or {}).get("message", "") or "")
        except Exception:
            message = ""

        if not message:
            if HAS_ROSPY:
                rospy.logwarn(
                    "[CartonVisionInjectPose] result.message 为空，跳过不满垛字段注入"
                )
            return

        # single 模式(/infer_carton_pose)的 message 不含 top ids/estimated total，
        # 跳过不满垛字段注入
        if "top ids" not in message and "estimated total" not in message:
            return

        # box_type_override 留空 → None → build_pick_summary 从 message 解析
        box_type_override = str(self.params.get("box_type_override", "") or "").strip()
        box_type_arg = box_type_override if box_type_override else None

        try:
            summary = build_pick_summary(message, box_type_arg)
        except Exception as e:
            if HAS_ROSPY:
                rospy.logwarn(
                    f"[CartonVisionInjectPose] 不满垛摘要解析失败: {e}，"
                    f"仅保留 message 字符串"
                )
            self._write_blackboard_safe(
                str(self.params.get("top_carton_message_key", "top_carton_message")),
                message,
            )
            return

        field_map = [
            ("top_carton_message_key", "top_carton_message", message),
            ("carton_type_key", "carton_type", summary["carton_type"]),
            ("top_orientation_key", "top_orientation", summary["top_orientation"]),
            ("top_ids_key", "top_ids", summary["top_ids"]),
            ("estimated_total_key", "estimated_total", summary["estimated_total"]),
            ("empty_slots_key", "empty_slots", summary["empty_slots"]),
            ("is_partial_stack_key", "is_partial_stack", summary["is_partial_stack"]),
            ("pick_sequence_key", "pick_sequence", summary["pick_sequence"]),
        ]
        for param_key, default_key, value in field_map:
            board_key = str(self.params.get(param_key, default_key))
            self._write_blackboard_safe(board_key, value)

        if HAS_ROSPY:
            rospy.loginfo(
                "[CartonVisionInjectPose] 不满垛: carton_type={}, "
                "estimated_total={}, empty_slots={}, is_partial_stack={}, "
                "pick_sequence(len={})={}".format(
                    summary["carton_type"],
                    summary["estimated_total"],
                    summary["empty_slots"],
                    summary["is_partial_stack"],
                    len(summary["pick_sequence"]),
                    summary["pick_sequence"],
                )
            )

    def _write_blackboard_safe(self, key: str, value) -> None:
        """安全写入黑板：注册 key（若已存在忽略异常）后 set value。"""
        try:
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE
            )
        except Exception:
            pass
        try:
            self.global_blackboard.set(key, value)
        except Exception as e:
            if HAS_ROSPY:
                rospy.logwarn(
                    f"[CartonVisionInjectPose] blackboard set failed: key={key}, err={e}"
                )
