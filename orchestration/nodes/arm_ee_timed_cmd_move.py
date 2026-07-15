# -*- coding: utf-8 -*-
"""ArmEETimedCmdMove：TimedCmd 末端位姿控制薄节点 → arm_ee_timed_cmd 原子技能。

通过 TimedCmd 服务(/mobile_manipulator_timed_single_cmd) 控制手臂末端位姿。
desireTime 让 C++ Ruckig 规划器在指定时间内平滑规划轨迹，
避免 topic 直发"马上到"导致的急加速。

对齐 test_arm_ee_local.py
格式: [x, y, z, yaw, pitch, roll] (米, 度)
"""

import json
import os

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_timed_cmd import (
    ArmEETimedCmdParams,
    ArmEETimedCmdSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="TimedCmd 末端位姿 (desireTime 平滑)",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "通过 TimedCmd 服务控制手臂末端，desireTime 字段让规划器平滑执行。"
        "格式: [x,y,z,yaw,pitch,roll] (米, 度)"
    ),
    params=[
        {"name": "left_waypoints", "type": "json", "default": "",
         "description": "左手关键点 [[x,y,z,yaw,pitch,roll], ...]"},
        {"name": "right_waypoints", "type": "json", "default": "",
         "description": "右手关键点 [[x,y,z,yaw,pitch,roll], ...]"},
        {"name": "desire_time", "type": "float", "default": "3.0",
         "description": "每段执行时间(秒)"},
        {"name": "frame", "type": "string", "default": "local",
         "description": "坐标系: 'world' / 'local'"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEeTimedCmdMove(BaseAction):
    """TimedCmd 末端位姿控制节点

    用法示例 (py_tree_child.json):
    {
      "name": "ArmEeTimedCmdMove",
      "label": "timed_grasp",
      "params": {
        "left_waypoints": {
          "value": [[1.0, 0.25, 1.2, 0, 0, 0]],
          "source": "CUSTOM", "data_type": "json"
        },
        "right_waypoints": {
          "value": [[1.0, -0.25, 1.2, 0, 0, 0]],
          "source": "CUSTOM", "data_type": "json"
        },
        "desire_time": { "value": "3.0", "source": "CUSTOM", "data_type": "float" }
      }
    }
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        # 优先从黑板运行时读取（支持动态注入）
        left_wps = self._resolve_waypoints_from_board("left_waypoints")
        right_wps = self._resolve_waypoints_from_board("right_waypoints")

        # 回退到构建时冻结的静态值
        if left_wps is None:
            left_wps = self._resolve_waypoints("left_waypoints")
        if right_wps is None:
            right_wps = self._resolve_waypoints("right_waypoints")

        if left_wps is None or right_wps is None:
            self.feedback_message = "arm_ee_timed_cmd: missing left_waypoints or right_waypoints"
            return

        if _DRY_RUN:
            self.feedback_message = (
                f"dry-run arm_ee_timed_cmd left={len(left_wps)}wp right={len(right_wps)}wp"
            )
            self._dry_done = True
            return

        skill_params = ArmEETimedCmdParams(
            left_waypoints=left_wps,
            right_waypoints=right_wps,
            desire_time=float(self.params.get("desire_time", 3.0)),
            frame=str(self.params.get("frame", "local")),
        )
        self._skill = ArmEETimedCmdSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_timed_cmd init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_ee_timed_cmd failed"
            return Status.FAILURE
        return Status.RUNNING

    def _resolve_waypoints(self, key: str):
        raw = self.params.get(key, None)
        return self._resolve_waypoints_value(raw)

    def _resolve_waypoints_value(self, raw):
        """解析 waypoints 原始值，支持 list / JSON string。"""
        if raw is None:
            return None
        if isinstance(raw, list):
            if len(raw) > 0 and isinstance(raw[0], (int, float)):
                # 单个 waypoint [x,y,z,yaw,pitch,roll] → 包装为单点列表
                return [raw]
            return raw
        if isinstance(raw, str) and raw.strip():
            try:
                parsed = json.loads(raw)
                if isinstance(parsed, list):
                    if len(parsed) > 0 and isinstance(parsed[0], (int, float)):
                        return [parsed]
                    return parsed
            except Exception:
                pass
        return None

    def _resolve_waypoints_from_board(self, key: str):
        """运行时从黑板读取 waypoints（优先于静态值）。

        key: 'left_waypoints' 或 'right_waypoints'
        查找 params 中工厂自动注入的 {key}__board_key，从黑板读取最新值。
        """
        board_key = str(self.params.get(f"{key}__board_key", "")).strip()
        if not board_key:
            return None
        try:
            self.global_blackboard.register_key(
                key=board_key, access=py_trees.common.Access.READ
            )
            raw = self.global_blackboard.get(board_key)
        except Exception:
            return None
        return self._resolve_waypoints_value(raw)
