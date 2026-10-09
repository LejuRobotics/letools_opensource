# -*- coding: utf-8 -*-
"""乐聚二指夹爪控制节点：按指定侧、位置、速度和力度下发一次指令。"""

import json
import math
import os

from py_trees.common import Access, Status

from core.domain.end_effector import DualGripperCommand
from core.domain.enums import ArmSide
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in (
        "1",
        "true",
        "yes",
    )


@define_manifest(
    label="乐聚夹爪控制",
    category=["end_effector", "control"],
    # 通用节点统一进入公共 Studio 节点分组；具体业务只负责在场景 JSON 中编排。
    tree_type="studio_smoke",
    description=(
        "通过通用硬件接口控制左、右或双侧乐聚二指夹爪；单侧控制会根据"
        "实时状态补齐另一侧，状态不可用时不会下发命令"
    ),
    params=[
        {
            "name": "enabled",
            "type": "bool",
            "default": True,
            "description": (
                "False 时跳过夹爪控制；绑定阶段命令对象时读取其中的 enabled"
            ),
        },
        {
            "name": "command",
            "type": "json",
            "default": [0.0, 50.0, 1.0],
            "description": (
                "格式为 [活动手位置,速度,电流]，需配合 active_arm；"
                "位置 0 张开，100 闭合"
            ),
        },
        {
            "name": "command_key",
            "type": "string",
            "default": "",
            "description": "command 为阶段命令对象时选择其中的键，例如 servo、pick",
        },
        {
            "name": "active_arm",
            "type": "json",
            "default": "",
            "description": (
                "必填活动手：left/right/both，或包含 active_arm 的对象；"
                "可通过 READ_BOARD 直接绑定关键点包"
            ),
        },
    ],
    inputs=[],
    outputs=[],
)
class LejuClawControl(BaseAction):
    """经共享 Hardware Adapter 安全控制单侧或双侧夹爪。"""

    def update(self):
        try:
            enabled = self._as_bool(self._param("enabled", True), "enabled")
            if not enabled:
                self.feedback_message = "夹爪控制已禁用，跳过当前节点"
                return Status.SUCCESS

            raw_command = self._param("command", [0.0, 50.0, 1.0])
            command_key = str(self.params.get("command_key", "")).strip()
            if command_key:
                raw_command = self._select_command(raw_command, command_key)
            active_arm_raw = self._param("active_arm", "")
            active_arm = self._find_active_arm(active_arm_raw)
            command = self._parse_command(raw_command, active_arm=active_arm)
            positions = [command.left_position, command.right_position]

            if _is_dry_run():
                self.feedback_message = (
                    f"dry-run leju claw positions={positions}, "
                    f"velocity={command.velocity}, effort={command.effort}"
                )
                return Status.SUCCESS

            result = get_shared_hardware().control_end_effector(
                ArmSide.BOTH, command
            )
            if not result.success:
                self.feedback_message = (
                    "gripper command failed: "
                    f"{result.message or 'unknown error'}"
                )
                return Status.FAILURE

            self.feedback_message = (
                f"gripper command succeeded: positions={positions}"
            )
            return Status.SUCCESS
        except (TypeError, ValueError) as exc:
            self.feedback_message = f"invalid gripper command: {exc}"
            return Status.FAILURE
        except Exception as exc:
            self.feedback_message = f"gripper control failed: {exc}"
            return Status.FAILURE

    def _param(self, key: str, default=None):
        """优先从 READ_BOARD 绑定键读取，保留嵌套 JSON 对象。"""
        board_key = str(self.params.get(f"{key}__board_key", "")).strip()
        if not board_key:
            return self.params.get(key, default)
        self.global_blackboard.register_key(key=board_key, access=Access.READ)
        if not self.global_blackboard.exists(board_key):
            raise ValueError(f"黑板不存在夹爪参数键: {board_key}")
        return self.global_blackboard.get(board_key)

    @classmethod
    def _parse_command(cls, raw, active_arm: str) -> DualGripperCommand:
        """按 active_arm 将单位置命令转换为双侧驱动命令。"""
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise ValueError(f"command 不是合法 JSON: {exc}")

        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            raise ValueError("command 必须是 [活动手位置,速度,电流]")

        raw_position, raw_velocity, raw_effort = raw
        if isinstance(raw_position, (list, tuple, dict)):
            raise ValueError("command 的活动手位置必须是单个数值")
        if active_arm not in ("left", "right", "both"):
            raise ValueError("command 必须提供 active_arm: left/right/both")

        position = cls._position(raw_position)
        if active_arm == "left":
            positions = [position, None]
        elif active_arm == "right":
            positions = [None, position]
        else:
            positions = [position, position]

        velocity = cls._bounded(raw_velocity, "速度", 0.0, 100.0)
        effort = float(raw_effort)
        if not math.isfinite(effort) or effort < 0.0:
            raise ValueError("电流必须是大于等于 0 的有限数")

        return DualGripperCommand(
            left_position=positions[0],
            right_position=positions[1],
            velocity=velocity,
            effort=effort,
        )

    @staticmethod
    def _select_command(raw, command_key: str):
        """从 board 中集中配置的阶段命令对象选择一条命令。"""
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise ValueError(f"command 不是合法 JSON: {exc}")
        if not isinstance(raw, dict):
            raise ValueError("设置 command_key 时，command 必须是阶段命令对象")
        if command_key not in raw:
            raise ValueError(f"command 缺少阶段键: {command_key}")
        return raw[command_key]

    @staticmethod
    def _find_active_arm(raw) -> str:
        """解析活动手字符串或嵌套对象中的唯一 ``active_arm``。"""
        if isinstance(raw, str):
            arm = raw.strip().lower()
            if arm in ("left", "right", "both"):
                return arm
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError:
                raise ValueError(
                    "活动手必须为 left/right/both 或包含 active_arm 的对象"
                )

        found = set()

        def _collect(value):
            if isinstance(value, dict):
                arm = str(value.get("active_arm", "")).strip().lower()
                if arm:
                    found.add(arm)
                for nested in value.values():
                    _collect(nested)
            elif isinstance(value, (list, tuple)):
                for nested in value:
                    _collect(nested)

        _collect(raw)
        invalid = found - {"left", "right", "both"}
        if invalid:
            raise ValueError(f"关键点包包含非法 active_arm: {sorted(invalid)}")
        if len(found) != 1:
            raise ValueError(
                "关键点包必须包含唯一的 active_arm，"
                f"实际找到: {sorted(found)}"
            )
        return next(iter(found))

    @staticmethod
    def _position(raw):
        value = float(raw)
        if not math.isfinite(value) or not 0.0 <= value <= 100.0:
            raise ValueError("活动手位置必须在 [0,100] 范围内")
        return value

    @staticmethod
    def _as_bool(raw, name):
        """解析布尔值；配置对象则读取其 enabled 字段。"""
        if isinstance(raw, dict):
            if "enabled" not in raw:
                raise ValueError(f"{name} 配置对象缺少 enabled")
            raw = raw["enabled"]
        if isinstance(raw, bool):
            return raw
        if isinstance(raw, str):
            value = raw.strip().lower()
            if value in ("true", "1", "yes"):
                return True
            if value in ("false", "0", "no"):
                return False
        if isinstance(raw, (int, float)) and raw in (0, 1):
            return bool(raw)
        raise ValueError(f"{name} 必须是布尔值")

    @staticmethod
    def _bounded(raw, name: str, minimum: float, maximum: float) -> float:
        value = float(raw)
        if not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"{name} 必须在 [{minimum}, {maximum}] 范围内")
        return value
