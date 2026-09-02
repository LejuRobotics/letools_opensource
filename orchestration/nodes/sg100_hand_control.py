# -*- coding: utf-8 -*-
"""SG100 11-DOF 灵巧手控制节点：按指定侧与关节位置（弧度）下发一次指令。

支持两种控制模式：
    - 位置控制（MODE_JOINT_POSITION=7）：仅下发 11 关节目标位置（rad）。
    - 阻抗/力控（MODE_JOINT_IMPEDANCE=9）：位置 + 每关节 kp/kd/torque_ff。

关节顺序（每只手 11 个）：
    [0]  TH_CMC_ABD   [1] TH_MCP_FLEX  [2] TH_IP_FLEX
    [3]  IF_MCP_ABD   [4] IF_MCP_FLEX  [5] IF_PIP_FLEX
    [6]  MF_MCP_FLEX  [7] MF_PIP_FLEX
    [8]  LF_MCP_ABD   [9] LF_MCP_FLEX [10] LF_PIP_FLEX

参数 ``command`` 支持两种 JSON 格式：
    1) 紧凑列表（位置模式，默认）：
       [[关节0..10 共 11 个位置], "left"|"right"]
    2) 可读字典：
       {"positions": [11 个 float], "side": "left"|"right",
        "control_mode": "position"|"impedance",  # 可选，默认 position
        "enable_mask": 2047,                      # 可选，默认全关节
        "kp": [11 float], "kd": [11 float], "torque_ff": [11 float]}  # 阻抗可选
"""

import math
import os
import json

from py_trees.common import Access, Status

from core.domain.end_effector import (
    SG100ControlMode,
    SG100HandCommand,
    SG100_JOINT_COUNT,
    SG100_POSITION_LIMIT,
)
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
    label="SG100灵巧手控制",
    category=["end_effector", "control"],
    tree_type="studio_smoke",
    description="通过通用硬件接口控制左、右单侧黑漫 SG100 11-DOF 灵巧手（位置/阻抗）",
    params=[
        {
            "name": "command",
            "type": "json",
            "default": [[0.0] * SG100_JOINT_COUNT, "left"],
            "description": (
                "紧凑 [[11 位置rad], side] 或可读 dict："
                "{positions, side, control_mode, enable_mask, kp, kd, torque_ff}"
            ),
        },
        {
            "name": "command_key",
            "type": "string",
            "default": "",
            "description": "command 为阶段命令对象时选择其中的键，例如 open、grasp",
        },
        {
            "name": "active_arm_board_key",
            "type": "string",
            "default": "",
            "description": (
                "可选黑板键；值为 left/right 或包含 active_arm 的关键点包。"
                "设置后覆盖 command 中的 side"
            ),
        },
    ],
    inputs=[],
    outputs=[],
)
class SG100HandControl(BaseAction):
    """经共享 Hardware Adapter 控制 SG100 灵巧手，并映射行为树状态。"""

    def update(self):
        try:
            raw_command = self.params.get(
                "command", [[0.0] * SG100_JOINT_COUNT, "left"]
            )
            command_key = str(self.params.get("command_key", "")).strip()
            if command_key:
                raw_command = self._select_command(raw_command, command_key)
            side, cmd = self._parse_command(raw_command)

            active_arm_board_key = str(
                self.params.get("active_arm_board_key", "")
            ).strip()
            if active_arm_board_key:
                side = self._read_active_arm(active_arm_board_key)

            if _is_dry_run():
                self.feedback_message = (
                    f"dry-run sg100 side={side}, mode={cmd.control_mode.name}, "
                    f"positions={cmd.positions}"
                )
                return Status.SUCCESS

            arm_side = ArmSide.LEFT if side == "left" else ArmSide.RIGHT
            result = get_shared_hardware().control_end_effector(arm_side, cmd)
            if not result.success:
                self.feedback_message = (
                    "sg100 command failed: "
                    f"{result.message or 'unknown error'}"
                )
                return Status.FAILURE

            self.feedback_message = (
                f"sg100 command succeeded: side={side}, "
                f"mode={cmd.control_mode.name}"
            )
            return Status.SUCCESS
        except (TypeError, ValueError) as exc:
            self.feedback_message = f"invalid sg100 command: {exc}"
            return Status.FAILURE
        except Exception as exc:
            self.feedback_message = f"sg100 control failed: {exc}"
            return Status.FAILURE

    @classmethod
    def _parse_command(cls, raw):
        """解析紧凑列表或可读 dict，返回 (side_str, SG100HandCommand)。"""
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise ValueError(f"command 不是合法 JSON: {exc}")

        if isinstance(raw, (list, tuple)):
            return cls._parse_compact(raw)
        if isinstance(raw, dict):
            return cls._parse_dict(raw)
        raise ValueError("command 必须是 [[11 位置], side] 或 dict")

    @classmethod
    def _parse_compact(cls, raw):
        """解析 [[11 位置], side] 紧凑格式（仅位置模式）。"""
        if len(raw) != 2:
            raise ValueError("紧凑 command 必须是 [[11 位置], side]")
        raw_positions, raw_side = raw
        positions = cls._joint_array(raw_positions, "positions")
        side = cls._side(raw_side)
        return side, SG100HandCommand(positions=positions)

    @classmethod
    def _parse_dict(cls, raw):
        """解析可读 dict 格式。"""
        positions = cls._joint_array(raw.get("positions", [0.0] * SG100_JOINT_COUNT), "positions")
        side = cls._side(raw.get("side", "left"))

        mode_str = str(raw.get("control_mode", "position")).strip().lower()
        if mode_str in ("position", "7", "joint_position"):
            control_mode = SG100ControlMode.JOINT_POSITION
        elif mode_str in ("impedance", "9", "joint_impedance"):
            control_mode = SG100ControlMode.JOINT_IMPEDANCE
        else:
            raise ValueError(f"control_mode 不支持: {mode_str}")

        enable_mask = int(raw.get("enable_mask", 0x07FF))
        if not 0 <= enable_mask <= 0xFFFF:
            raise ValueError("enable_mask 必须在 [0, 65535] 范围内")

        kp = raw.get("kp")
        kd = raw.get("kd")
        torque_ff = raw.get("torque_ff")
        # 阻抗模式下增益可选；若提供则校验长度与有限性
        kp = cls._joint_array(kp, "kp") if kp is not None else None
        kd = cls._joint_array(kd, "kd") if kd is not None else None
        torque_ff = cls._joint_array(torque_ff, "torque_ff") if torque_ff is not None else None

        # 混合模式：每关节独立控制模式（可选）
        per_joint_modes = cls._parse_per_joint_modes(raw.get("per_joint_modes"))

        return side, SG100HandCommand(
            positions=positions,
            control_mode=control_mode,
            enable_mask=enable_mask,
            kp=kp,
            kd=kd,
            torque_ff=torque_ff,
            per_joint_modes=per_joint_modes,
        )

    @staticmethod
    def _parse_per_joint_modes(raw):
        """解析 per_joint_modes：支持字符串列表或整数列表。"""
        if raw is None:
            return None
        if not isinstance(raw, (list, tuple)) or len(raw) != SG100_JOINT_COUNT:
            raise ValueError(
                f"per_joint_modes 必须是 {SG100_JOINT_COUNT} 个元素列表"
            )
        modes = []
        for i, item in enumerate(raw):
            s = str(item).strip().lower()
            if s in ("position", "7", "joint_position"):
                modes.append(SG100ControlMode.JOINT_POSITION)
            elif s in ("impedance", "9", "joint_impedance"):
                modes.append(SG100ControlMode.JOINT_IMPEDANCE)
            else:
                raise ValueError(
                    f"per_joint_modes[{i}]={item!r} 不支持，仅 position(7)/impedance(9)"
                )
        return modes

    @staticmethod
    def _joint_array(raw, name: str):
        """校验并返回 11 元素有限浮点数组（位置另做弧度软限位校验）。"""
        if not isinstance(raw, (list, tuple)) or len(raw) != SG100_JOINT_COUNT:
            raise ValueError(f"{name} 必须是 {SG100_JOINT_COUNT} 个 float")
        try:
            values = [float(v) for v in raw]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} 含无法转为 float 的元素: {exc}")
        for i, v in enumerate(values):
            if not math.isfinite(v):
                raise ValueError(f"{name}[{i}]={v!r} 非有限值")
            if name == "positions" and abs(v) > SG100_POSITION_LIMIT:
                raise ValueError(
                    f"positions[{i}]={v} rad 超限（±{SG100_POSITION_LIMIT} rad）；"
                    "请确认单位为弧度而非 [0,100] 归一化值"
                )
        return values

    @staticmethod
    def _side(raw) -> str:
        arm = str(raw).strip().lower()
        if arm not in ("left", "right"):
            raise ValueError("side 必须为 left 或 right")
        return arm

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


# ``_snake_to_pascal`` 对含全大写缩写的文件名（sg100→Sg100）无法还原原类名，
# 索引键为 "Sg100HandControl" 而类名是 "SG100HandControl"。
# 此别名使场景 JSON 用 "Sg100HandControl" 时也能 getattr 命中该类。
Sg100HandControl = SG100HandControl
