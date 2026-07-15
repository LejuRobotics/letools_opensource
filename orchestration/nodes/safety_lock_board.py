# -*- coding: utf-8 -*-
"""SafetyLockBoard：安全锁节点，打印黑板中的 waypoints 并等待 Enter 确认。

在 timed_grasp 之前执行，读取黑板中已注入的 left/right waypoints，
打印手臂目标位置，等待用户按 Enter 后才放行后续节点。
"""

import json
import os

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

try:
    import rospy
    HAS_ROSPY = True
except ImportError:
    HAS_ROSPY = False


def _format_waypoints(wps):
    """将 waypoints 格式化为可读字符串。"""
    if wps is None:
        return "(无数据)"
    if isinstance(wps, str):
        try:
            wps = json.loads(wps)
        except Exception:
            return wps
    if not isinstance(wps, list):
        return str(wps)
    lines = []
    for i, wp in enumerate(wps):
        if isinstance(wp, list) and len(wp) >= 6:
            lines.append(
                f"  [{i}] x={wp[0]:.4f}  y={wp[1]:.4f}  z={wp[2]:.4f}  "
                f"yaw={wp[3]:.1f}  pitch={wp[4]:.1f}  roll={wp[5]:.1f}"
            )
        else:
            lines.append(f"  [{i}] {wp}")
    return "\n".join(lines)


@define_manifest(
    label="安全锁(打印+等待确认)",
    category=["utility", "safety"],
    tree_type="studio_smoke",
    description=(
        "读取黑板中的 left/right waypoints 并打印，"
        "等待用户按 Enter 确认后才放行后续节点。"
    ),
    params=[
        {"name": "left_board_key", "type": "string",
         "default": "timed_grasp_1_left_waypoints",
         "description": "左手 waypoints 的黑板 key"},
        {"name": "right_board_key", "type": "string",
         "default": "timed_grasp_1_right_waypoints",
         "description": "右手 waypoints 的黑板 key"},
        {"name": "message", "type": "string",
         "default": "确认后按 Enter 继续...",
         "description": "等待确认时的提示信息"},
    ],
    inputs=[],
    outputs=[],
)
class SafetyLockBoard(BaseAction):
    """安全锁节点

    执行流程:
      1. 从黑板读取 left_board_key / right_board_key 对应的 waypoints
      2. 打印手臂目标位置
      3. 阻塞等待用户按 Enter
      4. 返回 SUCCESS 放行后续节点
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False

    def initialise(self):
        self._done = False

    def update(self):
        if self._done:
            return Status.SUCCESS

        if _DRY_RUN:
            self._done = True
            self.feedback_message = "dry-run safety_lock_board (skipped)"
            return Status.SUCCESS

        # --- 1. 从黑板读取 waypoints ---
        left_key = str(self.params.get("left_board_key", "timed_grasp_1_left_waypoints"))
        right_key = str(self.params.get("right_board_key", "timed_grasp_1_right_waypoints"))

        left_wps = None
        right_wps = None
        try:
            self.global_blackboard.register_key(
                key=left_key, access=py_trees.common.Access.READ
            )
            left_wps = self.global_blackboard.get(left_key)
        except Exception:
            pass
        try:
            self.global_blackboard.register_key(
                key=right_key, access=py_trees.common.Access.READ
            )
            right_wps = self.global_blackboard.get(right_key)
        except Exception:
            pass

        # --- 2. 打印手臂目标位置 ---
        msg = str(self.params.get("message", "确认后按 Enter 继续..."))
        banner = "=" * 60
        output = (
            f"\n{banner}\n"
            f" [安全锁] {self.label}\n"
            f"{banner}\n"
            f" 左手目标位置 (key={left_key}):\n"
            f"{_format_waypoints(left_wps)}\n"
            f"\n"
            f" 右手目标位置 (key={right_key}):\n"
            f"{_format_waypoints(right_wps)}\n"
            f"\n"
            f" {msg}\n"
            f"{banner}\n"
        )
        print(output)
        if HAS_ROSPY:
            rospy.loginfo(output)

        # --- 3. 等待 Enter ---
        try:
            input()
        except EOFError:
            pass

        self._done = True
        self.feedback_message = "safety_lock_board: user confirmed"
        return Status.SUCCESS
