# -*- coding: utf-8 -*-
"""LejuClawControl 单元测试。"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

# 直接执行本文件时，Python 默认只把 tests 目录放进模块搜索路径。
if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from py_trees.blackboard import Client
from py_trees.common import Access, Status

from core.domain.end_effector import DualGripperCommand
from core.domain.enums import ArmSide
from core.domain.result import Result
from orchestration.engine.behavior_tree_factory import ParamsWrapper
from orchestration.nodes.leju_claw_control import LejuClawControl


def test_both_grippers_receive_command():
    params = {"command": [[0, 90], 50, 1.0]}
    node = LejuClawControl("gripper", "gripper", None, params)
    hardware = MagicMock()
    hardware.control_end_effector.return_value = Result.ok()
    with patch(
        "orchestration.nodes.leju_claw_control.get_shared_hardware",
        return_value=hardware,
    ):
        assert node.update() == Status.SUCCESS
    hardware.control_end_effector.assert_called_once()
    side, command = hardware.control_end_effector.call_args.args
    assert side == ArmSide.BOTH
    assert isinstance(command, DualGripperCommand)
    assert command.left_position == 0.0
    assert command.right_position == 90.0
    assert command.velocity == 50.0
    assert command.effort == 1.0


def test_disabled_skips_invalid_command_and_hardware_access():
    node = LejuClawControl(
        "gripper",
        "gripper",
        None,
        {
            "enabled": False,
            "command": "not-json",
            "active_arm_board_key": "missing_key",
        },
    )
    with patch(
        "orchestration.nodes.leju_claw_control.get_shared_hardware"
    ) as get_hardware:
        assert node.update() == Status.SUCCESS
        assert "已禁用" in node.feedback_message
        get_hardware.assert_not_called()


def test_minus_one_skips_left_gripper():
    command = LejuClawControl._parse_command([[-1, 50], 60, 0.8])
    assert command.left_position is None
    assert command.right_position == 50.0
    assert command.velocity == 60.0
    assert command.effort == 0.8


def test_key_points_select_only_active_right_gripper():
    raw_command = LejuClawControl._parse_command([[0, 0], 50, 1.0])
    key_points = {
        "eef_pick": {"active_arm": "right"},
        "eef_pick_lift": {"active_arm": "right"},
    }
    active_arm = LejuClawControl._find_active_arm(key_points)
    command = LejuClawControl._mask_to_active_arm(raw_command, active_arm)
    assert command.left_position is None
    assert command.right_position == 0.0
    assert command.velocity == 50.0
    assert command.effort == 1.0


def test_select_command_from_board_mapping():
    commands = {
        "servo": [[90, 90], 50, 1.0],
        "pick": [[0, 0], 50, 1.0],
    }
    raw = LejuClawControl._select_command(commands, "pick")
    command = LejuClawControl._parse_command(raw)
    assert command.left_position == 0.0
    assert command.right_position == 0.0
    assert command.velocity == 50.0
    assert command.effort == 1.0


def test_nested_stage_commands_are_read_back_from_board():
    board_key = "test_leju_claw_stage_commands"
    writer = Client(name="test_leju_claw_writer", namespace="/")
    writer.register_key(key=board_key, access=Access.WRITE)
    writer.set(
        board_key,
        {
            "servo": [[90, 90], 50, 1.0],
            "pick": [[0, 0], 40, 0.8],
        },
    )
    params = ParamsWrapper(
        {
            # ParamsWrapper 会展开该对象；节点必须通过 __board_key 回读原值。
            "command": writer.get(board_key),
            "command__board_key": board_key,
            "command_key": "servo",
        }
    )
    node = LejuClawControl("gripper", "gripper", None, params)
    hardware = MagicMock()
    hardware.control_end_effector.return_value = Result.ok()
    with patch(
        "orchestration.nodes.leju_claw_control.get_shared_hardware",
        return_value=hardware,
    ):
        assert node.update() == Status.SUCCESS
    _, command = hardware.control_end_effector.call_args.args
    assert command.left_position == 90.0
    assert command.right_position == 90.0


def test_driver_failure_returns_failure():
    node = LejuClawControl(
        "gripper",
        "gripper",
        None,
        {"command": [[0, -1], 50, 1.0]},
    )
    hardware = MagicMock()
    hardware.control_end_effector.return_value = Result.fail("service rejected")
    with patch(
        "orchestration.nodes.leju_claw_control.get_shared_hardware",
        return_value=hardware,
    ):
        assert node.update() == Status.FAILURE
        assert "service rejected" in node.feedback_message


def test_both_sides_skipped_returns_failure_without_hardware_access():
    node = LejuClawControl(
        "gripper",
        "gripper",
        None,
        {"command": [[-1, -1], 50, 1.0]},
    )
    with patch(
        "orchestration.nodes.leju_claw_control.get_shared_hardware"
    ) as get_hardware:
        assert node.update() == Status.FAILURE
        get_hardware.assert_not_called()


if __name__ == "__main__":
    # 兼容没有安装 pytest 的机器人运行环境。这里直接调用纯单元测试函数，
    # 所有硬件访问均已 mock，不会连接 ROS 或控制真实夹爪。
    tests = (
        test_both_grippers_receive_command,
        test_disabled_skips_invalid_command_and_hardware_access,
        test_minus_one_skips_left_gripper,
        test_key_points_select_only_active_right_gripper,
        test_select_command_from_board_mapping,
        test_nested_stage_commands_are_read_back_from_board,
        test_driver_failure_returns_failure,
        test_both_sides_skipped_returns_failure_without_hardware_access,
    )
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"LejuClawControl: {len(tests)} tests passed")
