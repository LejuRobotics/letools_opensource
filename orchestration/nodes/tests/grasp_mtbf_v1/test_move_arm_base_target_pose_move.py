# -*- coding: utf-8 -*-
"""双臂节点的时序回归测试；所有硬件为 mock，不连接 ROS 服务。"""

import importlib
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

pytestmark = pytest.mark.unit
from py_trees.common import Status

from core.domain.enums import ArmSide
from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction

arm_module = importlib.import_module("orchestration.nodes.grasp_mtbf_v1.GraspMtbfMoveArm")
GraspMtbfMoveArm = arm_module.GraspMtbfMoveArm


class ArmHarness:
    def __init__(self):
        self.now = 100.0
        self.events = []
        self.releases = []
        self.stopped = threading.Event()
        self.hardware = MagicMock()
        self.hardware.send_timed_multi_commands.side_effect = self.send_motion
        self.hardware.control_end_effector.side_effect = self.send_gripper
        self.hardware.set_motion_control_enabled.side_effect = self.stop
        self.actual_time = 2.0

    def send_motion(self, commands, is_sync, cancel_event=None):
        self.events.append(("motion", commands, is_sync))
        return Result.ok(data={"actual_time": self.actual_time})

    def send_gripper(self, side, command):
        self.events.append(("gripper", side, command.position))
        return Result.ok()

    def stop(self, enabled):
        self.events.append(("stop", enabled))
        self.stopped.set()
        return Result.ok()

    def node(self, **overrides):
        params = {
            "prep_only": True,
            "prep_time": 1.0,
            "tag_id": -1,
            "total_time": 1.0,
            "settle_time": 0.25,
            "cmd_interval": 0.5,
            "service_timeout": 6.0,
            "motion_timeout": 60.0,
        }
        params.update(overrides)
        blackboard = MagicMock()
        blackboard.get.return_value = [
            (
                [[0.4, 0.3, 0.2, 0, 0, 0], [0.5, 0.2, 0.3, 0, 0, 0]],
                [[0.4, -0.3, 0.2, 0, 0, 0], [0.5, -0.2, 0.3, 0, 0, 0]],
            ),
            None,
        ]
        with patch.object(
            BaseAction, "attach_blackboard_client", create=True, return_value=blackboard
        ):
            return GraspMtbfMoveArm("arm", "arm", None, params)

    def complete_call(self, node):
        assert node._pending["done"].wait(1.0), "mock service worker did not finish"
        return node.update()

    def block_motion(self):
        entered = threading.Event()
        release = threading.Event()
        self.releases.append(release)

        def send(commands, is_sync, cancel_event=None):
            self.events.append(("motion", commands, is_sync))
            entered.set()
            assert release.wait(2.0), "test must release the mock service"
            return Result.ok(data={"actual_time": self.actual_time})

        self.hardware.send_timed_multi_commands.side_effect = send
        return entered, release


@pytest.fixture
def arm(monkeypatch):
    harness = ArmHarness()
    GraspMtbfMoveArm._uncertain_motion.clear()
    monkeypatch.setattr(arm_module, "_DRY_RUN", False)
    monkeypatch.setattr(arm_module, "get_scene_io", lambda: harness.hardware)
    # Replace only this module's time object; Event.wait retains its real clock.
    monkeypatch.setattr(
        arm_module, "time", SimpleNamespace(monotonic=lambda: harness.now)
    )
    yield harness
    for release in harness.releases:
        release.set()
    GraspMtbfMoveArm._uncertain_motion.clear()


def test_prep_acceptance_stays_running_until_actual_time_and_settle(arm):
    node = arm.node()
    node.initialise()
    arm.hardware.send_timed_multi_commands.assert_not_called()
    assert node.update() == Status.RUNNING
    assert arm.complete_call(node) == Status.RUNNING
    arm.hardware.send_timed_multi_commands.assert_called_once()
    commands = arm.events[0][1]
    assert [command["planner_index"] for command in commands] == [6, 7]
    assert commands[0]["cmd_vec"] == [0.4, 0.35, 0.13, 0.0, -90.0, 0.0]
    assert commands[1]["cmd_vec"] == [0.4, -0.35, 0.13, 0.0, -90.0, 0.0]
    assert arm.events[0][2] is True
    node.global_blackboard.set.assert_called_with("ArmMoveResult", False)

    arm.now += 2.24
    assert node.update() == Status.RUNNING
    node.global_blackboard.set.assert_called_with("ArmMoveResult", False)
    arm.now += 0.02
    assert node.update() == Status.SUCCESS
    node.global_blackboard.set.assert_called_with("ArmMoveResult", True)
    arm.hardware.control_end_effector.assert_not_called()


def test_short_actual_time_cannot_skip_requested_duration(arm):
    arm.actual_time = 0.1
    node = arm.node(prep_time=2.0, settle_time=0.0)
    node.initialise()
    assert node.update() == Status.RUNNING
    assert arm.complete_call(node) == Status.RUNNING
    arm.now += 0.5
    assert node.update() == Status.RUNNING
    arm.now += 1.5
    assert node.update() == Status.SUCCESS


def test_two_waypoints_wait_then_grip_at_the_configured_indices(arm):
    arm.actual_time = 1.0
    node = arm.node(
        prep_only=False,
        settle_time=0.0,
        gripper_close_indices="0",
        gripper_open_indices="1",
        gripper_position=80.0,
    )
    node.initialise()
    assert node.update() == Status.RUNNING
    assert arm.complete_call(node) == Status.RUNNING
    assert [event[0] for event in arm.events] == ["motion"]
    assert arm.events[0][1][0]["cmd_vec"][:3] == [0.4, 0.3, 0.2]
    assert arm.events[0][1][1]["cmd_vec"][:3] == [0.4, -0.3, 0.2]

    arm.now += 1.0
    assert node.update() == Status.RUNNING
    assert arm.complete_call(node) == Status.RUNNING
    assert arm.complete_call(node) == Status.RUNNING
    assert arm.events[1:3] == [
        ("gripper", ArmSide.LEFT, 80.0),
        ("gripper", ArmSide.RIGHT, 80.0),
    ]
    assert arm.hardware.send_timed_multi_commands.call_count == 1
    arm.now += 0.4
    assert node.update() == Status.RUNNING
    assert arm.hardware.send_timed_multi_commands.call_count == 1
    arm.now += 0.2
    assert node.update() == Status.RUNNING
    assert arm.complete_call(node) == Status.RUNNING
    assert arm.events[3][0] == "motion"
    assert arm.events[3][1][0]["cmd_vec"][:3] == [0.5, 0.2, 0.3]
    assert arm.events[3][1][1]["cmd_vec"][:3] == [0.5, -0.2, 0.3]
    assert arm.hardware.control_end_effector.call_count == 2

    arm.now += 1.0
    assert node.update() == Status.RUNNING
    assert arm.complete_call(node) == Status.RUNNING
    assert arm.complete_call(node) == Status.SUCCESS
    assert arm.events[4:6] == [
        ("gripper", ArmSide.LEFT, 0.0),
        ("gripper", ArmSide.RIGHT, 0.0),
    ]
    node.global_blackboard.set.assert_called_with("ArmMoveResult", True)


def test_timeout_freezes_and_late_success_cannot_restart_or_advance(arm):
    entered, release = arm.block_motion()
    node = arm.node(prep_only=False, gripper_close_indices="0")
    node.initialise()
    assert node.update() == Status.RUNNING
    assert entered.wait(1.0)
    flight = node._pending
    arm.now += 6.1
    assert node.update() == Status.FAILURE
    assert "超时" in node.feedback_message
    assert arm.stopped.wait(1.0)
    arm.hardware.set_motion_control_enabled.assert_called_once_with(False)
    release.set()
    assert flight["done"].wait(1.0)
    assert node.update() == Status.FAILURE
    node.initialise()
    assert node.update() == Status.FAILURE
    arm.hardware.send_timed_multi_commands.assert_called_once()
    arm.hardware.control_end_effector.assert_not_called()
    node.global_blackboard.set.assert_called_with("ArmMoveResult", False)

    another_node = arm.node()
    another_node.initialise()
    assert another_node.update() == Status.FAILURE
    arm.hardware.send_timed_multi_commands.assert_called_once()


def test_finished_service_response_after_deadline_is_rejected(arm):
    entered, release = arm.block_motion()
    node = arm.node()
    node.initialise()
    assert node.update() == Status.RUNNING
    assert entered.wait(1.0)
    flight = node._pending
    arm.now += 6.1
    release.set()
    assert flight["done"].wait(1.0)
    assert node.update() == Status.FAILURE
    assert "迟到" in node.feedback_message
    assert arm.stopped.wait(1.0)
    arm.hardware.control_end_effector.assert_not_called()


@pytest.mark.parametrize("cancel_during", ["call", "wait"])
def test_cancellation_never_sends_next_waypoint_or_gripper(arm, cancel_during):
    entered, release = arm.block_motion()
    node = arm.node(prep_only=False, gripper_close_indices="0")
    node.initialise()
    assert node.update() == Status.RUNNING
    assert entered.wait(1.0)
    flight = node._pending
    if cancel_during == "wait":
        release.set()
        assert arm.complete_call(node) == Status.RUNNING

    node.terminate(Status.INVALID)
    assert arm.stopped.wait(1.0)
    release.set()
    assert flight["done"].wait(1.0)
    arm.now += 100.0
    assert node.update() == Status.FAILURE
    node.initialise()
    assert node.update() == Status.FAILURE
    arm.hardware.send_timed_multi_commands.assert_called_once()
    arm.hardware.control_end_effector.assert_not_called()
    node.global_blackboard.set.assert_called_with("ArmMoveResult", False)


@pytest.mark.parametrize("actual_time", [float("nan"), float("inf"), -1.0, 61.0])
def test_invalid_actual_time_fails_without_followup_commands(arm, actual_time):
    arm.actual_time = actual_time
    node = arm.node(prep_only=False, gripper_close_indices="0")
    node.initialise()
    assert node.update() == Status.RUNNING
    assert arm.complete_call(node) == Status.FAILURE
    assert "无效执行时长" in node.feedback_message
    assert arm.stopped.wait(1.0)
    arm.hardware.send_timed_multi_commands.assert_called_once()
    arm.hardware.control_end_effector.assert_not_called()
    node.global_blackboard.set.assert_called_with("ArmMoveResult", False)


def test_gripper_failure_prevents_other_gripper_and_next_waypoint(arm):
    arm.actual_time = 1.0
    arm.hardware.control_end_effector.return_value = Result.fail("gripper refused")
    arm.hardware.control_end_effector.side_effect = None
    node = arm.node(prep_only=False, gripper_close_indices="0", settle_time=0.0)
    node.initialise()
    assert node.update() == Status.RUNNING
    assert arm.complete_call(node) == Status.RUNNING
    arm.now += 1.0
    assert node.update() == Status.RUNNING
    assert arm.complete_call(node) == Status.FAILURE
    assert arm.stopped.wait(1.0)
    assert arm.hardware.control_end_effector.call_count == 1
    assert arm.hardware.send_timed_multi_commands.call_count == 1
    assert "gripper refused" in node.feedback_message
