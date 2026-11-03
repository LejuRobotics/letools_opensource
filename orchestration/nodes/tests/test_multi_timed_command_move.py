# -*- coding: utf-8 -*-
"""MultiTimedCommandMove 模式准备及失败传播的离线测试。"""

from unittest.mock import MagicMock

import py_trees
from py_trees.blackboard import Blackboard
from py_trees.common import Status
import pytest

from core.domain.result import Result
import orchestration.nodes.multi_timed_command_move as motion

pytestmark = pytest.mark.unit


@pytest.fixture
def hardware(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.setattr(motion, "_DRY_RUN", False)
    hw = MagicMock()
    hw.set_arm_control_mode.return_value = Result.ok()
    hw.set_focus_ee.return_value = Result.ok()
    hw.set_focus_z.return_value = Result.ok()
    hw.send_timed_multi_commands.return_value = Result.ok()
    monkeypatch.setattr(motion, "get_shared_hardware", lambda: hw)
    monkeypatch.setattr("skills.atomic.refactored_sdk.multi_timed_command.time.sleep", lambda _: None)
    return hw


@pytest.mark.parametrize("value,expected", [(None, True), (True, True), (False, False),
                                            ("true", True), ("false", False)])
def test_mode_preparation_parameter_reaches_skill(hardware, value, expected):
    params = {"commands": [{"planner_index": 6, "cmd_vec": [0.0] * 6}]}
    if value is not None:
        params["prepare_arm_control"] = value
    node = motion.MultiTimedCommandMove("move", "move", "test", params)

    node.initialise()
    assert node._skill.params.prepare_arm_control is expected
    hardware.set_arm_control_mode.assert_not_called()
    assert node.update() == Status.RUNNING
    assert node.update() == Status.SUCCESS
    if expected:
        hardware.set_arm_control_mode.assert_called_once_with(2)
    else:
        hardware.set_arm_control_mode.assert_not_called()
    hardware.send_timed_multi_commands.assert_called_once()


def test_mode_failure_propagates_and_blocks_next_action(hardware):
    hardware.set_arm_control_mode.return_value = Result.fail("mode unavailable")
    node = motion.MultiTimedCommandMove("move", "move", "test", {
        "commands": [{"planner_index": 8, "cmd_vec": [0.0] * 7}],
    })
    next_action = py_trees.behaviours.Success("next_action")
    root = py_trees.composites.Sequence("sequence", memory=True, children=[node, next_action])

    py_trees.trees.BehaviourTree(root).tick()

    assert node.status == root.status == Status.FAILURE
    assert "set_arm_control_mode(2) failed: mode unavailable" in node.feedback_message
    assert next_action.status == Status.INVALID
    hardware.set_arm_control_mode.assert_called_once_with(2)
    hardware.set_focus_ee.assert_not_called()
    hardware.send_timed_multi_commands.assert_not_called()


def test_dry_run_does_not_access_hardware_or_execute_skill(hardware, monkeypatch):
    monkeypatch.setattr(motion, "_DRY_RUN", True)
    getter = MagicMock(side_effect=AssertionError("unexpected hardware access"))
    monkeypatch.setattr(motion, "get_shared_hardware", getter)
    node = motion.MultiTimedCommandMove("move", "move", "test", {
        "commands": [{"planner_index": 6, "cmd_vec": [0.0] * 6}],
        "prepare_arm_control": True,
    })

    node.initialise()
    assert node.update() == Status.SUCCESS
    getter.assert_not_called()
    hardware.set_arm_control_mode.assert_not_called()
    hardware.send_timed_multi_commands.assert_not_called()
