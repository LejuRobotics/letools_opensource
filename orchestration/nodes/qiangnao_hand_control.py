# -*- coding: utf-8 -*-
"""qiangnao 灵巧手控制节点：按指定侧与 6 个手指关节位置（[0,100]）下发一次指令。

手指关节顺序与 ``robotHandPosition.msg`` 一侧数组一致（由手部固件定义），
位置为行程占比 [0, 100]，0=张开，100=闭合。

``command`` 支持两种 JSON 格式：
    1) 紧凑列表：
       [[关节0..5 共 6 个位置], "left"|"right"|"both"]
    2) 可读字典：
       {"positions": [6 个 float], "side": "left"|"right"|"both"}

底层经 ``hardware.control_end_effector`` 发到话题 ``/control_robot_hand_position``。
"""

import json
import math
import os
from typing import List

from py_trees.common import Status

from core.domain.end_effector import HAND_JOINT_COUNT, HandFingerCommand
from core.domain.enums import ArmSide
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_SIDE_MAP = {
    "left": ArmSide.LEFT,
    "right": ArmSide.RIGHT,
    "both": ArmSide.BOTH,
}

_DEFAULT_COMMAND = [[0.0] * HAND_JOINT_COUNT, "left"]


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in (
        "1",
        "true",
        "yes",
    )


@define_manifest(
    label="强脑灵巧手控制",
    category=["end_effector", "control"],
    tree_type="studio_smoke",
    description="通过通用硬件接口控制左、右或双侧强脑 qiangnao 6-DOF 灵巧手开合",
    params=[
        {
            "name": "command",
            "type": "json",
            "default": _DEFAULT_COMMAND,
            "description": (
                "紧凑 [[6 位置], side] 或可读 dict：{positions, side}；"
                "位置为行程占比 [0,100]，0=张开，100=闭合"
            ),
        },
        {
            "name": "command_key",
            "type": "string",
            "default": "",
            "description": "command 为阶段命令对象时选择其中的键，例如 open、grasp",
        },
    ],
    inputs=[],
    outputs=[],
)
class QiangnaoHandControl(BaseAction):
    """经共享 Hardware Adapter 控制 qiangnao 灵巧手，并映射行为树状态。"""

    def update(self):
        try:
            raw_command = self.params.get("command", _DEFAULT_COMMAND)
            command_key = str(self.params.get("command_key", "")).strip()
            if command_key:
                raw_command = self._select_command(raw_command, command_key)
            side, cmd = self._parse_command(raw_command)

            if _is_dry_run():
                self.feedback_message = (
                    f"dry-run qiangnao side={side}, positions={cmd.positions}"
                )
                return Status.SUCCESS

            result = get_shared_hardware().control_end_effector(
                _SIDE_MAP[side], cmd
            )
            if not result.success:
                self.feedback_message = (
                    "qiangnao command failed: "
                    f"{result.message or 'unknown error'}"
                )
                return Status.FAILURE

            self.feedback_message = f"qiangnao command succeeded: side={side}"
            return Status.SUCCESS
        except (TypeError, ValueError) as exc:
            self.feedback_message = f"invalid qiangnao command: {exc}"
            return Status.FAILURE
        except Exception as exc:
            self.feedback_message = f"qiangnao control failed: {exc}"
            return Status.FAILURE

    @classmethod
    def _parse_command(cls, raw):
        """解析紧凑列表或可读 dict，返回 (side_str, HandFingerCommand)。"""
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise ValueError(f"command 不是合法 JSON: {exc}")

        if isinstance(raw, (list, tuple)):
            if len(raw) != 2:
                raise ValueError("紧凑 command 必须是 [[6 位置], side]")
            raw_positions, raw_side = raw
        elif isinstance(raw, dict):
            if "positions" not in raw:
                raise ValueError("command dict 缺少 positions")
            raw_positions, raw_side = raw["positions"], raw.get("side", "")
        else:
            raise ValueError("command 必须是 [[6 位置], side] 或 dict")

        return cls._side(raw_side), HandFingerCommand(
            positions=cls._positions(raw_positions)
        )

    @staticmethod
    def _side(raw) -> str:
        side = str(raw).strip().lower()
        if side not in _SIDE_MAP:
            raise ValueError("side 必须为 left/right/both")
        return side

    @staticmethod
    def _positions(raw) -> List[float]:
        """校验并返回 6 元素 [0,100] 行程占比数组。"""
        if not isinstance(raw, (list, tuple)) or len(raw) != HAND_JOINT_COUNT:
            raise ValueError(f"positions 必须是 {HAND_JOINT_COUNT} 个元素列表")
        try:
            values = [float(v) for v in raw]
        except (TypeError, ValueError):
            raise ValueError(f"positions 含无法转为 float 的值: {raw}")
        for value in values:
            if not math.isfinite(value) or not 0.0 <= value <= 100.0:
                raise ValueError(
                    "positions 元素必须在 [0, 100] 行程占比范围内"
                    f"（0=张开, 100=闭合），实际含 {value}"
                )
        return values

    @staticmethod
    def _select_command(raw, command_key: str):
        """从阶段命令对象选择一条命令。"""
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
