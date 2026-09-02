# -*- coding: utf-8 -*-
"""ArmEeDualTimedMove：双臂单次末端位姿（TimedCmd）薄节点 → arm_ee_dual_timed 原子技能。

对齐 apps/test_kuavo_5w_sdk_adapter/timed/04_arm/test_arm_ee_dual_timed.py 的双臂末端指令。

安全约束：
- 默认 focus_ee=False、focus_z=False，躯干不参与末端结算；
- 执行前只切手臂外部控制 set_arm_control_mode(2)，不做手臂物理复位；
- 执行完成后保持手臂当前位置，不复位手臂；
- 不调用 reset_torso_to_initial，躯干不复位。
"""

import json
import os

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_dual_timed import (
    ArmEEDualTimedParams,
    ArmEEDualTimedSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "on")
    return False


@define_manifest(
    label="双臂单次末端位姿（TimedCmd/躯干不参与）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "双臂单次末端位姿指令（planner 4+5/6+7）。"
        "默认 focus_ee=False/focus_z=False，躯干不参与结算；执行后不复位手臂、不复位躯干。"
    ),
    params=[
        {"name": "frame", "type": "string", "default": "world", "description": "坐标系: 'world' / 'local'"},
        {"name": "left_pose", "type": "json", "default": "[0.5,0.25,0.7,0,-90,0]",
         "description": "左臂末端位姿 [x,y,z,yaw,pitch,roll]（米, 度）；单点模式使用"},
        {"name": "right_pose", "type": "json", "default": "[0.5,-0.25,0.7,0,-90,0]",
         "description": "右臂末端位姿 [x,y,z,yaw,pitch,roll]（米, 度）；单点模式使用"},
        {"name": "left_waypoints", "type": "json", "default": "",
         "description": "兼容原 ArmEeTimedCmdMove: 左臂两段 waypoints [[approach], [grasp]]，优先从黑板读取"},
        {"name": "right_waypoints", "type": "json", "default": "",
         "description": "兼容原 ArmEeTimedCmdMove: 右臂两段 waypoints [[approach], [grasp]]，优先从黑板读取"},
        {"name": "waypoint_index", "type": "int", "default": "0",
         "description": "使用 waypoints 中的第几个点：0=上方接近点，1=下探抓取点"},
        {"name": "desire_time", "type": "float", "default": "2.0", "description": "期望执行时间（秒）"},
        {"name": "focus_ee", "type": "bool", "default": "False",
         "description": "False=躯干优先/躯干不被末端带动；默认 False"},
        {"name": "focus_z", "type": "bool", "default": "False",
         "description": "False=禁用 Z 轴焦点跟随；默认 False"},
        {"name": "prepare_arm_control", "type": "bool", "default": "True",
         "description": "执行前切换到手臂外部控制模式 set_arm_control_mode(2)，不做手臂复位"},
        {"name": "release_arm_control", "type": "bool", "default": "True",
         "description": "执行完成后释放外部末端保持 set_arm_control_mode(0)，不复位手臂"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEeDualTimedMove(BaseAction):
    """双臂单次末端位姿 TimedCmd 节点。

    用法示例 (py_tree_child.json):
    {
      "name": "ArmEeDualTimedMove",
      "label": "dual_ee_pose",
      "params": {
        "frame": { "value": "world", "source": "CUSTOM", "data_type": "string" },
        "left_pose": {
          "value": [0.5, 0.25, 0.7, 0, -90, 0],
          "source": "CUSTOM", "data_type": "json"
        },
        "right_pose": {
          "value": [0.5, -0.25, 0.7, 0, -90, 0],
          "source": "CUSTOM", "data_type": "json"
        },
        "desire_time": { "value": "2.0", "source": "CUSTOM", "data_type": "float" },
        "focus_ee": { "value": "False", "source": "CUSTOM", "data_type": "bool" },
        "focus_z": { "value": "False", "source": "CUSTOM", "data_type": "bool" },
        "prepare_arm_control": { "value": "True", "source": "CUSTOM", "data_type": "bool" }
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

        frame = str(self.params.get("frame", "world"))
        desire_time = float(self.params.get("desire_time", 2.0))
        waypoint_index = int(self.params.get("waypoint_index", 0))
        left_pose, right_pose = self._resolve_pose_pair(waypoint_index)
        focus_ee = _to_bool(self.params.get("focus_ee", False))
        focus_z = _to_bool(self.params.get("focus_z", False))
        prepare_arm_control = _to_bool(self.params.get("prepare_arm_control", True))
        release_arm_control = _to_bool(self.params.get("release_arm_control", True))

        if left_pose is None or right_pose is None:
            self.feedback_message = "arm_ee_dual_timed: missing or invalid left/right pose or waypoints"
            return

        if _DRY_RUN:
            self.feedback_message = (
                f"dry-run arm_ee_dual_timed {frame} left={left_pose} right={right_pose} "
                f"focus_ee={focus_ee} focus_z={focus_z} prepare_arm_control={prepare_arm_control} "
                f"release_arm_control={release_arm_control}"
            )
            self._dry_done = True
            return

        skill_params = ArmEEDualTimedParams(
            frame=frame,
            left_pose=left_pose,
            right_pose=right_pose,
            desire_time=desire_time,
            focus_ee=focus_ee,
            focus_z=focus_z,
            prepare_arm_control=prepare_arm_control,
            release_arm_control=release_arm_control,
        )
        self._skill = ArmEEDualTimedSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_dual_timed init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_ee_dual_timed failed"
            return Status.FAILURE
        return Status.RUNNING

    def _resolve_pose_pair(self, waypoint_index: int):
        left_wps = self._resolve_waypoints_from_board("left_waypoints")
        right_wps = self._resolve_waypoints_from_board("right_waypoints")
        if left_wps is None:
            left_wps = self._resolve_waypoints("left_waypoints")
        if right_wps is None:
            right_wps = self._resolve_waypoints("right_waypoints")

        if left_wps is not None or right_wps is not None:
            if left_wps is None or right_wps is None:
                return None, None
            if len(left_wps) <= waypoint_index or len(right_wps) <= waypoint_index:
                return None, None
            left_pose = self._resolve_pose_value(left_wps[waypoint_index])
            right_pose = self._resolve_pose_value(right_wps[waypoint_index])
            return left_pose, right_pose

        return self._resolve_pose("left_pose"), self._resolve_pose("right_pose")

    def _resolve_pose(self, key: str):
        return self._resolve_pose_value(self.params.get(key, None))

    def _resolve_pose_value(self, raw):
        if raw is None:
            return None
        if isinstance(raw, list):
            return raw if len(raw) == 6 else None
        if isinstance(raw, str) and raw.strip():
            try:
                parsed = json.loads(raw)
                if isinstance(parsed, list) and len(parsed) == 6:
                    return parsed
            except Exception:
                return None
        return None

    def _resolve_waypoints(self, key: str):
        return self._resolve_waypoints_value(self.params.get(key, None))

    def _resolve_waypoints_value(self, raw):
        if raw is None:
            return None
        if isinstance(raw, list):
            if len(raw) > 0 and isinstance(raw[0], (int, float)):
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
