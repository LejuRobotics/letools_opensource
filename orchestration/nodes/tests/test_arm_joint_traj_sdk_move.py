# -*- coding: utf-8 -*-
"""ArmJointTrajSdkMove 参数校验与 dry-run 行为的离线测试。"""

import json
from unittest.mock import MagicMock

import py_trees
from py_trees.blackboard import Blackboard
from py_trees.common import Status
import pytest

from core.domain.result import Result
import orchestration.nodes.arm_joint_traj_sdk_move as motion

pytestmark = pytest.mark.unit

ZERO = [0.0] * 14
FORWARD = [-30.0] + [0.0] * 6 + [-30.0] + [0.0] * 6


@pytest.fixture
def hardware(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.setattr(motion, "_DRY_RUN", False)
    hw = MagicMock()
    hw.send_arm_joint_traj_sdk.return_value = Result.ok()
    monkeypatch.setattr(motion, "get_shared_hardware", lambda: hw)
    return hw


def _node(joint_traj, total_time=3.0):
    return motion.ArmJointTrajSdkMove("move", "move", "test", {
        "use_board_trajectory": "false",
        "joint_traj": joint_traj,
        "total_time": total_time,
    })


@pytest.mark.parametrize("joint_traj", [[ZERO, FORWARD], json.dumps([ZERO, FORWARD])],
                         ids=["list", "json"])
def test_inline_trajectory_reaches_hardware(hardware, joint_traj):
    node = _node(joint_traj)

    node.initialise()
    assert node.update() == Status.RUNNING
    assert node.update() == Status.SUCCESS
    hardware.send_arm_joint_traj_sdk.assert_called_once_with(
        joint_traj=[ZERO, FORWARD], total_time=3.0
    )


def test_wrong_point_size_fails_without_touching_hardware(hardware):
    node = _node([ZERO, FORWARD[:13]])

    node.initialise()
    assert node.update() == Status.FAILURE
    assert "expects 14 joint angles per point, got 13 at index 1" in node.feedback_message
    hardware.send_arm_joint_traj_sdk.assert_not_called()


def test_dry_run_validates_point_size_without_hardware(hardware, monkeypatch):
    monkeypatch.setattr(motion, "_DRY_RUN", True)
    getter = MagicMock(side_effect=AssertionError("unexpected hardware access"))
    monkeypatch.setattr(motion, "get_shared_hardware", getter)

    good = _node([ZERO, FORWARD])
    good.initialise()
    assert good.update() == Status.SUCCESS

    bad = _node([ZERO, FORWARD[:13]])
    bad.initialise()
    assert bad.update() == Status.FAILURE
    assert "got 13 at index 1" in bad.feedback_message

    getter.assert_not_called()
    hardware.send_arm_joint_traj_sdk.assert_not_called()


@pytest.mark.parametrize("dry_run", [False, True], ids=["mock-hardware", "dry-run"])
@pytest.mark.parametrize("joint_traj", [None, "", "[", "{}", [], ZERO, [ZERO, FORWARD[:13]]],
                         ids=["missing", "empty-string", "invalid-json", "object", "empty-list",
                              "flat-list", "wrong-point-size"])
def test_invalid_trajectory_blocks_sequence(hardware, monkeypatch, dry_run, joint_traj):
    monkeypatch.setattr(motion, "_DRY_RUN", dry_run)
    node = _node(joint_traj)
    next_action = py_trees.behaviours.Success("next_action")
    root = py_trees.composites.Sequence("sequence", memory=True, children=[node, next_action])

    py_trees.trees.BehaviourTree(root).tick()

    assert node.status == root.status == Status.FAILURE
    assert node.feedback_message
    assert next_action.status == Status.INVALID
    hardware.send_arm_joint_traj_sdk.assert_not_called()


@pytest.mark.parametrize("dry_run", [False, True], ids=["mock-hardware", "dry-run"])
@pytest.mark.parametrize("valid", [False, True], ids=["invalid", "valid"])
def test_fixed_blackboard_trajectory_is_validated(hardware, monkeypatch, dry_run, valid):
    monkeypatch.setattr(motion, "_DRY_RUN", dry_run)
    getter = MagicMock(return_value=hardware)
    monkeypatch.setattr(motion, "get_shared_hardware", getter)
    trajectory = [ZERO, FORWARD if valid else FORWARD[:13]]
    Blackboard.set("ArmJointTrajectories", {"q_frames": trajectory, "times": [0.0, 4.5]})
    node = motion.ArmJointTrajSdkMove("move", "move", "test", {})

    node.initialise()

    if not valid:
        assert node.update() == Status.FAILURE
        assert "got 13 at index 1" in node.feedback_message
    elif dry_run:
        assert node.update() == Status.SUCCESS
    else:
        assert node.update() == Status.RUNNING
        assert node.update() == Status.SUCCESS
        hardware.send_arm_joint_traj_sdk.assert_called_once_with(
            joint_traj=trajectory, total_time=4.5
        )
    if dry_run:
        getter.assert_not_called()
    if dry_run or not valid:
        hardware.send_arm_joint_traj_sdk.assert_not_called()


@pytest.mark.parametrize("dry_run", [False, True], ids=["mock-hardware", "dry-run"])
def test_reinitialization_clears_success_and_recovers(hardware, monkeypatch, dry_run):
    monkeypatch.setattr(motion, "_DRY_RUN", dry_run)
    node = _node([ZERO, FORWARD])

    for trajectory, valid in [([ZERO, FORWARD], True),
                              ([ZERO, FORWARD[:13]], False),
                              ([FORWARD, ZERO], True)]:
        node.params["joint_traj"] = trajectory
        node.initialise()
        if not valid:
            assert node.update() == Status.FAILURE
        else:
            if not dry_run:
                assert node.update() == Status.RUNNING
            assert node.update() == Status.SUCCESS

    assert hardware.send_arm_joint_traj_sdk.call_count == (0 if dry_run else 2)


def test_hardware_failure_blocks_sequence(hardware):
    hardware.send_arm_joint_traj_sdk.return_value = Result.fail("trajectory unavailable")
    node = _node([ZERO, FORWARD])
    next_action = py_trees.behaviours.Success("next_action")
    root = py_trees.composites.Sequence("sequence", memory=True, children=[node, next_action])

    py_trees.trees.BehaviourTree(root).tick()

    assert node.status == root.status == Status.FAILURE
    assert node.feedback_message == "trajectory unavailable"
    assert next_action.status == Status.INVALID
    hardware.send_arm_joint_traj_sdk.assert_called_once()
