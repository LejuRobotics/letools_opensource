"""LejuClawControl 单元测试。"""

from unittest.mock import MagicMock, patch

from py_trees.blackboard import Client
from py_trees.common import Access, Status

from core.domain.end_effector import DualGripperCommand
from core.domain.enums import ArmSide
from core.domain.result import Result
from orchestration.engine.behavior_tree_factory import ParamsWrapper
from orchestration.nodes.leju_claw_control import LejuClawControl


def test_both_target_sends_same_position_to_both_grippers():
    params = {"command": [0, 50, 1.0], "active_arm": "both"}
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
    assert command.right_position == 0.0
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
        },
    )
    with patch(
        "orchestration.nodes.leju_claw_control.get_shared_hardware"
    ) as get_hardware:
        assert node.update() == Status.SUCCESS
        assert "已禁用" in node.feedback_message
        get_hardware.assert_not_called()


def test_enabled_can_be_read_from_central_command_config():
    board_key = "test_leju_claw_enabled_config"
    writer = Client(name="test_leju_claw_enabled_writer", namespace="/")
    writer.register_key(key=board_key, access=Access.WRITE)
    writer.set(board_key, {"enabled": False, "pick": [100, 50, 1.0]})
    node = LejuClawControl(
        "gripper",
        "gripper",
        None,
        ParamsWrapper(
            {
                "enabled__board_key": board_key,
                "command__board_key": board_key,
                "command_key": "pick",
            }
        ),
    )

    with patch(
        "orchestration.nodes.leju_claw_control.get_shared_hardware"
    ) as get_hardware:
        assert node.update() == Status.SUCCESS
        assert "已禁用" in node.feedback_message
        get_hardware.assert_not_called()


def test_key_points_select_only_active_right_gripper():
    board_key = "test_leju_claw_pick_key_points"
    writer = Client(name="test_leju_claw_key_points_writer", namespace="/")
    writer.register_key(key=board_key, access=Access.WRITE)
    key_points = {
        "ee_pick": {"active_arm": "right"},
        "ee_pick_lift": {"active_arm": "right"},
    }
    writer.set(board_key, key_points)
    node = LejuClawControl(
        "gripper",
        "gripper",
        None,
        ParamsWrapper(
            {
                "command": [100, 50, 1.0],
                "active_arm__board_key": board_key,
            }
        ),
    )
    hardware = MagicMock()
    hardware.control_end_effector.return_value = Result.ok()

    with patch(
        "orchestration.nodes.leju_claw_control.get_shared_hardware",
        return_value=hardware,
    ):
        assert node.update() == Status.SUCCESS

    _, command = hardware.control_end_effector.call_args.args
    assert command.left_position is None
    assert command.right_position == 100.0
    assert command.velocity == 50.0
    assert command.effort == 1.0


def test_single_position_command_targets_active_arm():
    left = LejuClawControl._parse_command([60, 50, 1.0], active_arm="left")
    right = LejuClawControl._parse_command([90, 40, 0.8], active_arm="right")

    assert left.left_position == 60.0
    assert left.right_position is None
    assert right.left_position is None
    assert right.right_position == 90.0


def test_open_and_close_positions_are_not_inverted():
    opened = LejuClawControl._parse_command([0, 50, 1.0], active_arm="both")
    closed = LejuClawControl._parse_command([100, 50, 1.0], active_arm="both")

    assert [opened.left_position, opened.right_position] == [0.0, 0.0]
    assert [closed.left_position, closed.right_position] == [100.0, 100.0]


def test_nested_stage_commands_are_read_back_from_board():
    board_key = "test_leju_claw_stage_commands"
    writer = Client(name="test_leju_claw_writer", namespace="/")
    writer.register_key(key=board_key, access=Access.WRITE)
    writer.set(
        board_key,
        {
            "servo": [90, 50, 1.0],
            "pick": [100, 40, 0.8],
        },
    )
    params = ParamsWrapper(
        {
            # ParamsWrapper 会展开该对象；节点必须通过 __board_key 回读原值。
            "command": writer.get(board_key),
            "command__board_key": board_key,
            "command_key": "servo",
            "active_arm": "both",
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


def test_hardware_failure_returns_failure():
    node = LejuClawControl(
        "gripper",
        "gripper",
        None,
        {"command": [0, 50, 1.0], "active_arm": "left"},
    )
    hardware = MagicMock()
    hardware.control_end_effector.return_value = Result.fail("service rejected")
    with patch(
        "orchestration.nodes.leju_claw_control.get_shared_hardware",
        return_value=hardware,
    ):
        assert node.update() == Status.FAILURE
        assert "service rejected" in node.feedback_message
