# -*- coding: utf-8 -*-
"""SG100HandControl 节点单元测试（三层测试：核心 dataclass + 节点解析 + dry-run）。"""

import os
import pytest
from unittest.mock import MagicMock, patch

from py_trees.common import Status

from core.domain.end_effector import (
    SG100ControlMode,
    SG100HandCommand,
    SG100_JOINT_COUNT,
)
from core.domain.enums import ArmSide
from core.domain.result import Result
from orchestration.nodes.sg100_hand_control import SG100HandControl

pytestmark = pytest.mark.unit


# ---------- 第一层：核心 dataclass 校验 ----------

class TestSG100HandCommand:
    """SG100HandCommand 领域模型校验。"""

    def test_default_is_position_mode_all_joints(self):
        cmd = SG100HandCommand()
        assert cmd.control_mode == SG100ControlMode.JOINT_POSITION
        assert cmd.enable_mask == 0x07FF
        assert len(cmd.positions) == SG100_JOINT_COUNT
        assert cmd.kp is None and cmd.kd is None and cmd.torque_ff is None

    def test_wrong_position_count_raises(self):
        with pytest.raises(ValueError, match="positions 长度"):
            SG100HandCommand(positions=[0.0] * 5)

    def test_wrong_gain_count_raises(self):
        with pytest.raises(ValueError, match="kp 长度"):
            SG100HandCommand(
                positions=[0.0] * SG100_JOINT_COUNT,
                control_mode=SG100ControlMode.JOINT_IMPEDANCE,
                kp=[0.1] * 3,
            )

    def test_control_mode_from_int(self):
        # 兼容从 JSON 反序列化得到的 int 值（阻抗模式须同时提供 kp/kd）
        cmd = SG100HandCommand(
            positions=[0.0] * SG100_JOINT_COUNT,
            control_mode=9,  # type: ignore[arg-type]
            kp=[0.5] * SG100_JOINT_COUNT,
            kd=[0.01] * SG100_JOINT_COUNT,
        )
        assert cmd.control_mode == SG100ControlMode.JOINT_IMPEDANCE

    def test_impedance_without_gains_raises(self):
        """阻抗模式无 kp/kd 必须在构造时拒绝（防零刚度失控下垂）。"""
        with pytest.raises(ValueError, match="kp 与 kd"):
            SG100HandCommand(
                positions=[0.0] * SG100_JOINT_COUNT,
                control_mode=SG100ControlMode.JOINT_IMPEDANCE,
            )

    def test_impedance_only_kp_raises(self):
        """阻抗模式只给 kp 不给 kd（无阻尼弹簧，振荡风险）必须拒绝。"""
        with pytest.raises(ValueError, match="kd"):
            SG100HandCommand(
                positions=[0.0] * SG100_JOINT_COUNT,
                control_mode=SG100ControlMode.JOINT_IMPEDANCE,
                kp=[0.5] * SG100_JOINT_COUNT,
            )

    def test_position_out_of_range_raises(self):
        """超限弧度值（如旧 [0,100] 归一化误用）必须在构造时拒绝。"""
        with pytest.raises(ValueError, match="超限"):
            SG100HandCommand(positions=[100.0] + [0.0] * (SG100_JOINT_COUNT - 1))

    def test_per_joint_modes_wrong_length_raises(self):
        """per_joint_modes 长度必须为 11。"""
        with pytest.raises(ValueError, match="per_joint_modes 长度"):
            SG100HandCommand(
                positions=[0.0] * SG100_JOINT_COUNT,
                per_joint_modes=[SG100ControlMode.JOINT_POSITION] * 3,
            )

    def test_per_joint_modes_invalid_value_raises(self):
        """per_joint_modes 含无效模式值必须拒绝。"""
        with pytest.raises(ValueError, match="不是有效控制模式"):
            SG100HandCommand(
                positions=[0.0] * SG100_JOINT_COUNT,
                per_joint_modes=[99] * SG100_JOINT_COUNT,  # type: ignore[list-item]
            )

    def test_per_joint_modes_normalizes_int(self):
        """per_joint_modes 接受 int 值并归一化为枚举。"""
        modes = [7] * SG100_JOINT_COUNT  # type: ignore[list-item]
        cmd = SG100HandCommand(
            positions=[0.0] * SG100_JOINT_COUNT,
            per_joint_modes=modes,
        )
        assert cmd.per_joint_modes is not None
        assert all(m == SG100ControlMode.JOINT_POSITION for m in cmd.per_joint_modes)

    def test_mixed_mode_impedance_without_gains_raises(self):
        """混合模式含阻抗关节但未提供 kp/kd 必须拒绝（防零刚度失控）。"""
        modes = [SG100ControlMode.JOINT_POSITION] * SG100_JOINT_COUNT
        modes[0] = SG100ControlMode.JOINT_IMPEDANCE  # 拇指走阻抗
        with pytest.raises(ValueError, match="kp 与 kd"):
            SG100HandCommand(
                positions=[0.0] * SG100_JOINT_COUNT,
                per_joint_modes=modes,
            )

    def test_mixed_mode_impedance_only_kp_raises(self):
        """混合模式含阻抗关节但只给 kp 不给 kd 必须拒绝。"""
        modes = [SG100ControlMode.JOINT_POSITION] * SG100_JOINT_COUNT
        modes[0] = SG100ControlMode.JOINT_IMPEDANCE
        with pytest.raises(ValueError, match="kd"):
            SG100HandCommand(
                positions=[0.0] * SG100_JOINT_COUNT,
                per_joint_modes=modes,
                kp=[0.5] * SG100_JOINT_COUNT,
            )

    def test_mixed_mode_impedance_with_gains_ok(self):
        """混合模式含阻抗关节且提供 kp/kd 应通过校验。"""
        modes = [SG100ControlMode.JOINT_POSITION] * SG100_JOINT_COUNT
        modes[0] = SG100ControlMode.JOINT_IMPEDANCE  # 仅拇指阻抗
        cmd = SG100HandCommand(
            positions=[0.0] * SG100_JOINT_COUNT,
            per_joint_modes=modes,
            kp=[0.5] * SG100_JOINT_COUNT,
            kd=[0.05] * SG100_JOINT_COUNT,
        )
        impedance_joints = cmd.impedance_joint_indices()
        assert impedance_joints == [0]

    def test_impedance_joint_indices_scalar_position_returns_empty(self):
        """标量位置模式无阻抗关节。"""
        cmd = SG100HandCommand(positions=[0.0] * SG100_JOINT_COUNT)
        assert cmd.impedance_joint_indices() == []

    def test_impedance_joint_indices_scalar_impedance_returns_all(self):
        """标量阻抗模式所有关节都走阻抗。"""
        cmd = SG100HandCommand(
            positions=[0.0] * SG100_JOINT_COUNT,
            control_mode=SG100ControlMode.JOINT_IMPEDANCE,
            kp=[0.5] * SG100_JOINT_COUNT,
            kd=[0.05] * SG100_JOINT_COUNT,
        )
        assert cmd.impedance_joint_indices() == list(range(SG100_JOINT_COUNT))


# ---------- 第二层：节点参数解析 ----------

class TestSG100ParseCommand:
    """SG100HandControl._parse_command 解析逻辑。"""

    def test_compact_list_position_mode(self):
        positions = [0.0] * SG100_JOINT_COUNT
        side, cmd = SG100HandControl._parse_command([positions, "left"])
        assert side == "left"
        assert cmd.positions == positions
        assert cmd.control_mode == SG100ControlMode.JOINT_POSITION

    def test_dict_position_mode(self):
        positions = [0.1] * SG100_JOINT_COUNT
        side, cmd = SG100HandControl._parse_command(
            {"positions": positions, "side": "right"}
        )
        assert side == "right"
        assert cmd.positions == positions
        assert cmd.control_mode == SG100ControlMode.JOINT_POSITION

    def test_dict_impedance_mode_with_gains(self):
        positions = [0.0] * SG100_JOINT_COUNT
        kp = [0.5] * SG100_JOINT_COUNT
        kd = [0.01] * SG100_JOINT_COUNT
        torque = [0.2] * SG100_JOINT_COUNT
        side, cmd = SG100HandControl._parse_command(
            {
                "positions": positions,
                "side": "left",
                "control_mode": "impedance",
                "kp": kp,
                "kd": kd,
                "torque_ff": torque,
            }
        )
        assert cmd.control_mode == SG100ControlMode.JOINT_IMPEDANCE
        assert cmd.kp == kp
        assert cmd.kd == kd
        assert cmd.torque_ff == torque

    def test_dict_mode_alias_numeric(self):
        positions = [0.0] * SG100_JOINT_COUNT
        _, cmd = SG100HandControl._parse_command(
            {
                "positions": positions,
                "control_mode": "9",
                "kp": [0.5] * SG100_JOINT_COUNT,
                "kd": [0.01] * SG100_JOINT_COUNT,
            }
        )
        assert cmd.control_mode == SG100ControlMode.JOINT_IMPEDANCE

    def test_invalid_side_raises(self):
        positions = [0.0] * SG100_JOINT_COUNT
        with pytest.raises(ValueError, match="side"):
            SG100HandControl._parse_command([positions, "middle"])

    def test_wrong_position_count_raises(self):
        with pytest.raises(ValueError, match="positions"):
            SG100HandControl._parse_command([[0.0] * 3, "left"])

    def test_position_out_of_range_in_parse_raises(self):
        """解析阶段对超限弧度值（100 rad）拒绝。"""
        with pytest.raises(ValueError, match="超限"):
            SG100HandControl._parse_command([[100.0] + [0.0] * 10, "left"])

    def test_impedance_dict_without_gains_raises(self):
        """阻抗模式 dict 不提供 kp/kd 在构造时拒绝。"""
        positions = [0.0] * SG100_JOINT_COUNT
        with pytest.raises(ValueError, match="kp 与 kd"):
            SG100HandControl._parse_command(
                {"positions": positions, "control_mode": "impedance"}
            )

    def test_impedance_dict_only_kp_raises(self):
        """阻抗模式 dict 只给 kp 不给 kd 必须拒绝。"""
        positions = [0.0] * SG100_JOINT_COUNT
        with pytest.raises(ValueError, match="kd"):
            SG100HandControl._parse_command(
                {
                    "positions": positions,
                    "control_mode": "impedance",
                    "kp": [0.5] * SG100_JOINT_COUNT,
                }
            )

    def test_mixed_mode_dict_parses_per_joint_modes(self):
        """混合模式 dict：per_joint_modes 字符串列表解析为枚举列表。"""
        positions = [0.0] * SG100_JOINT_COUNT
        # 仅第 11 个关节（索引 10）走阻抗，其余位置控制
        pjm = ["position"] * 10 + ["impedance"]
        side, cmd = SG100HandControl._parse_command(
            {
                "side": "left",
                "positions": positions,
                "per_joint_modes": pjm,
                "kp": [0.5] * SG100_JOINT_COUNT,
                "kd": [0.05] * SG100_JOINT_COUNT,
            }
        )
        assert side == "left"
        assert cmd.per_joint_modes is not None
        assert cmd.per_joint_modes[10] == SG100ControlMode.JOINT_IMPEDANCE
        assert cmd.per_joint_modes[0] == SG100ControlMode.JOINT_POSITION

    def test_mixed_mode_dict_int_modes(self):
        """混合模式 dict：per_joint_modes 接受整数列表 (7=position/9=impedance)。"""
        positions = [0.0] * SG100_JOINT_COUNT
        pjm = [9] + [7] * (SG100_JOINT_COUNT - 1)
        side, cmd = SG100HandControl._parse_command(
            {
                "side": "right",
                "positions": positions,
                "per_joint_modes": pjm,
                "kp": [0.5] * SG100_JOINT_COUNT,
                "kd": [0.05] * SG100_JOINT_COUNT,
            }
        )
        assert side == "right"
        assert cmd.per_joint_modes is not None
        assert cmd.per_joint_modes[0] == SG100ControlMode.JOINT_IMPEDANCE

    def test_mixed_mode_dict_invalid_mode_raises(self):
        """混合模式 dict：不支持的字符串模式必须报错。"""
        positions = [0.0] * SG100_JOINT_COUNT
        with pytest.raises(ValueError, match="不支持"):
            SG100HandControl._parse_command(
                {
                    "side": "left",
                    "positions": positions,
                    "per_joint_modes": ["bogus"] * SG100_JOINT_COUNT,
                }
            )

    def test_mixed_mode_dict_wrong_length_raises(self):
        """混合模式 dict：per_joint_modes 长度错必须报错。"""
        positions = [0.0] * SG100_JOINT_COUNT
        with pytest.raises(ValueError, match="per_joint_modes 必须是"):
            SG100HandControl._parse_command(
                {
                    "side": "left",
                    "positions": positions,
                    "per_joint_modes": ["position"] * 5,
                }
            )

    def test_invalid_mode_raises(self):
        positions = [0.0] * SG100_JOINT_COUNT
        with pytest.raises(ValueError, match="control_mode"):
            SG100HandControl._parse_command(
                {"positions": positions, "control_mode": "bogus"}
            )

    def test_select_command_key(self):
        positions = [0.0] * SG100_JOINT_COUNT
        stage = {"open": [positions, "left"], "grasp": [positions, "right"]}
        raw = SG100HandControl._select_command(stage, "grasp")
        side, _ = SG100HandControl._parse_command(raw)
        assert side == "right"

    def test_enable_mask_custom(self):
        positions = [0.0] * SG100_JOINT_COUNT
        _, cmd = SG100HandControl._parse_command(
            {"positions": positions, "enable_mask": 0b00000000100}
        )
        assert cmd.enable_mask == 0b00000000100


# ---------- 第三层：节点 update dry-run + 分发 ----------

def _make_node(params):
    node = SG100HandControl("sg100", "sg100", "ns", params)
    node.initialise()
    return node


def test_dry_run_returns_success():
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        positions = [0.0] * SG100_JOINT_COUNT
        node = _make_node({"command": [positions, "left"]})
        assert node.update() == Status.SUCCESS
        assert "dry-run" in node.feedback_message
    finally:
        del os.environ["STUDIO_DRY_RUN"]


def test_dispatch_left_position_command():
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    positions = [0.2] * SG100_JOINT_COUNT
    mock_hw = MagicMock()
    mock_hw.control_end_effector.return_value = Result.ok()
    with patch(
        "orchestration.nodes.sg100_hand_control.get_shared_hardware",
        return_value=mock_hw,
    ):
        node = _make_node({"command": [positions, "left"]})
        assert node.update() == Status.SUCCESS
    mock_hw.control_end_effector.assert_called_once()
    side, cmd = mock_hw.control_end_effector.call_args.args
    assert side == ArmSide.LEFT
    assert isinstance(cmd, SG100HandCommand)
    assert cmd.positions == positions
    assert cmd.control_mode == SG100ControlMode.JOINT_POSITION


def test_dispatch_right_impedance_command():
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    positions = [0.0] * SG100_JOINT_COUNT
    mock_hw = MagicMock()
    mock_hw.control_end_effector.return_value = Result.ok()
    command = {
        "positions": positions,
        "side": "right",
        "control_mode": "impedance",
        "kp": [0.5] * SG100_JOINT_COUNT,
        "kd": [0.0] * SG100_JOINT_COUNT,
        "torque_ff": [0.0] * SG100_JOINT_COUNT,
    }
    with patch(
        "orchestration.nodes.sg100_hand_control.get_shared_hardware",
        return_value=mock_hw,
    ):
        node = _make_node({"command": command})
        assert node.update() == Status.SUCCESS
    side, cmd = mock_hw.control_end_effector.call_args.args
    assert side == ArmSide.RIGHT
    assert cmd.control_mode == SG100ControlMode.JOINT_IMPEDANCE
    assert cmd.kp == [0.5] * SG100_JOINT_COUNT


def test_hardware_failure_returns_failure_status():
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    positions = [0.0] * SG100_JOINT_COUNT
    mock_hw = MagicMock()
    mock_hw.control_end_effector.return_value = Result.fail("timeout")
    with patch(
        "orchestration.nodes.sg100_hand_control.get_shared_hardware",
        return_value=mock_hw,
    ):
        node = _make_node({"command": [positions, "left"]})
        assert node.update() == Status.FAILURE
    assert "timeout" in node.feedback_message


def test_active_arm_board_key_overrides_side():
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    positions = [0.0] * SG100_JOINT_COUNT
    mock_hw = MagicMock()
    mock_hw.control_end_effector.return_value = Result.ok()
    import py_trees
    with patch(
        "orchestration.nodes.sg100_hand_control.get_shared_hardware",
        return_value=mock_hw,
    ):
        node = _make_node(
            {"command": [positions, "left"], "active_arm_board_key": "ee"}
        )
        node.global_blackboard.register_key(
            key="ee", access=py_trees.common.Access.WRITE
        )
        node.global_blackboard.ee = "right"
        assert node.update() == Status.SUCCESS
    side, _ = mock_hw.control_end_effector.call_args.args
    assert side == ArmSide.RIGHT
