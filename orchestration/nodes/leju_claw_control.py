# -*- coding: utf-8 -*-
"""乐聚二指夹爪控制节点：按指定侧、位置、速度和力度下发一次指令。"""

import math
import os
import json

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
    description="通过通用硬件接口控制左、右或双侧乐聚二指夹爪",
    params=[
        {
            "name": "enabled",
            "type": "bool",
            "default": True,
            "description": "False 时跳过夹爪控制并直接返回 SUCCESS",
        },
        {
            "name": "command",
            "type": "json",
            "default": [[0.0, 0.0], 50.0, 1.0],
            "description": (
                "[[左位置,右位置],速度,电流]；位置 -1 表示该侧不控制，"
                "0 张开，100 闭合"
            ),
        },
        {
            "name": "command_key",
            "type": "string",
            "default": "",
            "description": "command 为阶段命令对象时选择其中的键，例如 servo、pick",
        },
        {
            "name": "active_arm_board_key",
            "type": "string",
            "default": "",
            "description": (
                "可选黑板键；值为 left/right 或包含 active_arm 的关键点包。"
                "设置后只控制活动手"
            ),
        },
    ],
    inputs=[],
    outputs=[],
)
class LejuClawControl(BaseAction):
    """经共享 Hardware Adapter 控制夹爪，并映射行为树状态。"""

    def update(self):
        try:
            enabled = self._as_bool(
                self._param("enabled", True), "enabled"
            )
            if not enabled:
                self.feedback_message = "夹爪控制已禁用，跳过当前节点"
                return Status.SUCCESS

            raw_command = self._param(
                "command", [[0.0, 0.0], 50.0, 1.0]
            )
            command_key = str(self.params.get("command_key", "")).strip()
            if command_key:
                raw_command = self._select_command(raw_command, command_key)
            command = self._parse_command(
                raw_command
            )
            active_arm_board_key = str(
                self.params.get("active_arm_board_key", "")
            ).strip()
            if active_arm_board_key:
                active_arm = self._read_active_arm(active_arm_board_key)
                command = self._mask_to_active_arm(command, active_arm)
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
        board_key = str(
            self.params.get(f"{key}__board_key", "")
        ).strip()
        if not board_key:
            return self.params.get(key, default)
        self.global_blackboard.register_key(key=board_key, access=Access.READ)
        if not self.global_blackboard.exists(board_key):
            raise ValueError(f"黑板不存在夹爪参数键: {board_key}")
        return self.global_blackboard.get(board_key)

    @classmethod
    def _parse_command(cls, raw) -> DualGripperCommand:
        """解析 ``[[左位置,右位置],速度,电流]`` 紧凑命令。"""
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise ValueError(f"command 不是合法 JSON: {exc}")

        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            raise ValueError("command 必须是 [[左位置,右位置],速度,电流]")

        raw_positions, raw_velocity, raw_effort = raw
        if not isinstance(raw_positions, (list, tuple)) or len(raw_positions) != 2:
            raise ValueError("command 的位置必须是 [左位置,右位置]")

        positions = [cls._position(value) for value in raw_positions]
        if positions == [None, None]:
            raise ValueError("左右位置不能同时为 -1")

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

    def _read_active_arm(self, board_key: str) -> str:
        """从黑板字符串或关键点包中解析当前活动手。"""
        self.global_blackboard.register_key(key=board_key, access=Access.READ)
        if not self.global_blackboard.exists(board_key):
            raise ValueError(f"黑板不存在活动手来源键: {board_key}")
        return self._find_active_arm(self.global_blackboard.get(board_key))

    @classmethod
    def _find_active_arm(cls, raw) -> str:
        """递归查找关键点包中的唯一 ``active_arm``。"""
        if isinstance(raw, str):
            arm = raw.strip().lower()
            if arm in ("left", "right"):
                return arm
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError:
                raise ValueError("活动手必须为 left/right 或包含 active_arm 的对象")

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
        invalid = found - {"left", "right"}
        if invalid:
            raise ValueError(f"关键点包包含非法 active_arm: {sorted(invalid)}")
        if len(found) != 1:
            raise ValueError(
                "关键点包必须包含唯一的 active_arm，"
                f"实际找到: {sorted(found)}"
            )
        return next(iter(found))

    @staticmethod
    def _mask_to_active_arm(
        command: DualGripperCommand,
        active_arm: str,
    ) -> DualGripperCommand:
        """保留活动手位置，把另一侧转换为不控制。"""
        if active_arm == "left":
            if command.left_position is None:
                raise ValueError("command 未提供活动左手的位置")
            left_position, right_position = command.left_position, None
        elif active_arm == "right":
            if command.right_position is None:
                raise ValueError("command 未提供活动右手的位置")
            left_position, right_position = None, command.right_position
        else:
            raise ValueError("active_arm 必须为 left 或 right")
        return DualGripperCommand(
            left_position=left_position,
            right_position=right_position,
            velocity=command.velocity,
            effort=command.effort,
        )

    @staticmethod
    def _position(raw):
        value = float(raw)
        if value == -1.0:
            return None
        if not math.isfinite(value) or not 0.0 <= value <= 100.0:
            raise ValueError("左右位置必须为 -1 或 [0,100]")
        return value

    @staticmethod
    def _as_bool(raw, name):
        """兼容 JSON 布尔值和行为树编辑器产生的布尔字符串。"""
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
