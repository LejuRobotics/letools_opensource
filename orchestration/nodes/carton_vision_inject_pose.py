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
        {"name": "x_offset", "type": "float", "default": "0.18",
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
    ],
    inputs=[],
    outputs=[],
)
class CartonVisionInjectPose(BaseAction):
    """视觉位姿注入节点

    执行流程:
      1. 调用 get_shared_hardware().infer_top_carton() (ROS 服务 /infer_top_carton_ids)
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
            result = hw.infer_top_carton()
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
