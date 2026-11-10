# -*- coding: utf-8 -*-
"""EndEffectorMixin SG100 分发单元测试（mock IHardware）。"""

import pytest
from unittest.mock import MagicMock

from core.domain.end_effector import (
    GripperCommand,
    HandFingerCommand,
    SG100ControlMode,
    SG100HandCommand,
    SG100_JOINT_COUNT,
)
from core.domain.enums import ArmSide
from core.domain.result import Result
from adapters.hardware.leju_wheeled.mixins.end_effector_mixin import EndEffectorMixin


@pytest.mark.unit
class _Mixin(EndEffectorMixin):
    """最小 mixin 实例，注入 mock 末端驱动。"""

    def __init__(self, end_effector):
        self._end_effector = end_effector


def _ok(msg="ok", data=None):
    return Result.ok(msg, data=data)


class TestSG100Dispatch:
    """SG100HandCommand 经 mixin 分发到 send_sg100_command。"""

    def test_left_dispatches_to_send_sg100(self):
        ee = MagicMock()
        ee.send_sg100_command.return_value = _ok("published")
        hw = _Mixin(ee)
        positions = [0.1] * SG100_JOINT_COUNT
        cmd = SG100HandCommand(positions=positions)

        result = hw.control_end_effector(ArmSide.LEFT, cmd)

        assert result.success
        ee.send_sg100_command.assert_called_once()
        side_str, dispatched_cmd = ee.send_sg100_command.call_args.args
        assert side_str == "left"
        assert dispatched_cmd is cmd

    def test_right_dispatches_to_send_sg100(self):
        ee = MagicMock()
        ee.send_sg100_command.return_value = _ok()
        hw = _Mixin(ee)
        # JOINT_IMPEDANCE 必须同时提供 kp/kd，否则 SG100HandCommand 构造时 ValueError
        cmd = SG100HandCommand(
            control_mode=SG100ControlMode.JOINT_IMPEDANCE,
            kp=[0.3] * SG100_JOINT_COUNT,
            kd=[0.05] * SG100_JOINT_COUNT,
        )

        result = hw.control_end_effector(ArmSide.RIGHT, cmd)

        assert result.success
        side_str, _ = ee.send_sg100_command.call_args.args
        assert side_str == "right"

    def test_sg100_does_not_call_legacy_hand_command(self):
        """SG100 分支不应误触 send_command / send_hand_command。"""
        ee = MagicMock()
        ee.send_sg100_command.return_value = _ok()
        hw = _Mixin(ee)
        cmd = SG100HandCommand()

        hw.control_end_effector(ArmSide.LEFT, cmd)

        ee.send_command.assert_not_called()
        ee.send_hand_command.assert_not_called()

    def test_legacy_gripper_still_routes_to_send_command(self):
        """回归：GripperCommand 仍走 send_command 分支。"""
        ee = MagicMock()
        ee.send_command.return_value = _ok()
        hw = _Mixin(ee)
        cmd = GripperCommand(position=50.0)

        result = hw.control_end_effector(ArmSide.BOTH, cmd)

        assert result.success
        ee.send_command.assert_called_once()
        ee.send_sg100_command.assert_not_called()

    def test_legacy_hand_finger_still_routes_to_send_hand_command(self):
        """回归：HandFingerCommand 仍走 send_hand_command 分支。"""
        ee = MagicMock()
        ee.send_hand_command.return_value = _ok()
        hw = _Mixin(ee)
        cmd = HandFingerCommand()

        result = hw.control_end_effector(ArmSide.LEFT, cmd)

        assert result.success
        ee.send_hand_command.assert_called_once()
        ee.send_sg100_command.assert_not_called()

    def test_driver_failure_propagates_as_fail(self):
        ee = MagicMock()
        ee.send_sg100_command.return_value = Result.fail("driver not ready")
        hw = _Mixin(ee)
        cmd = SG100HandCommand()

        result = hw.control_end_effector(ArmSide.LEFT, cmd)

        assert not result.success
        assert "driver not ready" in result.message

    def test_mixed_mode_dispatches_unchanged(self):
        """混合模式 SG100HandCommand 经 mixin 透传到 send_sg100_command。"""
        ee = MagicMock()
        ee.send_sg100_command.return_value = _ok()
        hw = _Mixin(ee)
        # 仅拇指（索引 0）走阻抗，其余位置控制
        modes = [SG100ControlMode.JOINT_POSITION] * SG100_JOINT_COUNT
        modes[0] = SG100ControlMode.JOINT_IMPEDANCE
        cmd = SG100HandCommand(
            positions=[0.1] * SG100_JOINT_COUNT,
            per_joint_modes=modes,
            kp=[0.5] * SG100_JOINT_COUNT,
            kd=[0.05] * SG100_JOINT_COUNT,
        )

        result = hw.control_end_effector(ArmSide.LEFT, cmd)

        assert result.success
        ee.send_sg100_command.assert_called_once()
        side_str, dispatched_cmd = ee.send_sg100_command.call_args.args
        assert side_str == "left"
        assert dispatched_cmd is cmd
        assert dispatched_cmd.per_joint_modes is not None
        assert dispatched_cmd.per_joint_modes[0] == SG100ControlMode.JOINT_IMPEDANCE
