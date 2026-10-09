"""Submission, recovery and arrival sequencing for the private JiBot nodes."""
import importlib
import threading
import time
from unittest.mock import MagicMock, patch

import pytest
from py_trees.common import Status
from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.grasp_mtbf_v1.GraspMtbfMoveToTarget import GraspMtbfMoveToTarget
from orchestration.nodes.grasp_mtbf_v1.GraspMtbfRelativeMove import GraspMtbfRelativeMove
from orchestration.nodes.grasp_mtbf_v1.GraspMtbfChassisControl import GraspMtbfChassisControl

pytestmark = pytest.mark.unit
move_module = importlib.import_module("orchestration.nodes.grasp_mtbf_v1.GraspMtbfMoveToTarget")
control_module = importlib.import_module("orchestration.nodes.grasp_mtbf_v1.GraspMtbfChassisControl")


def node(cls=GraspMtbfMoveToTarget, **params):
    with patch.object(BaseAction, "attach_blackboard_client", return_value=MagicMock()):
        return cls("test", "test", "", params)


def run(node, duration=0.5):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        status = node.update()
        if status != Status.RUNNING:
            return status
        time.sleep(0.001)
    return Status.RUNNING


def wait_until(predicate):
    deadline = time.monotonic() + 0.4
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert predicate()


@pytest.fixture
def io(monkeypatch):
    io = MagicMock()
    io.base_move_to_target_jibot.return_value = Result.ok(data={"task_id": "absolute"})
    io.base_move_relative_jibot.return_value = Result.ok(data={"task_id": "relative"})
    io.enable_vel_control_jibot.return_value = Result.ok()
    monkeypatch.setattr(move_module, "get_scene_io", lambda: io)
    monkeypatch.setattr(control_module, "get_scene_io", lambda: io)
    monkeypatch.setattr(move_module, "_DRY_RUN", False)
    monkeypatch.setattr(control_module, "_DRY_RUN", False)
    return io


def test_target_preserves_units_and_saves_only_accepted_task(io):
    action = node(theta=180, avoid_enabled="false", allow_rotation="True")
    action.initialise()
    assert run(action) == Status.SUCCESS
    kwargs = io.base_move_to_target_jibot.call_args.kwargs
    assert kwargs["theta"] == pytest.approx(3.141592653589793)
    assert kwargs["options"].avoid_enabled is False
    assert kwargs["options"].allow_rotation is True
    assert isinstance(kwargs["cancel_event"], threading.Event)
    action.global_blackboard.set.assert_called_once_with("current_task_id", "absolute")
    io.enable_vel_control_jibot.assert_not_called()


def test_failed_task_never_becomes_success_on_tick_or_reinitialise(io):
    io.base_move_to_target_jibot.return_value = Result.fail("service failed")
    action = node()
    action.initialise()
    assert run(action) == Status.FAILURE
    wait_until(lambda: io.enable_vel_control_jibot.called)
    action.initialise()
    assert action.update() == Status.FAILURE
    assert io.base_move_to_target_jibot.call_count == 1
    assert io.enable_vel_control_jibot.call_args.args == (True,)


def test_missing_task_id_restores_control(io):
    io.base_move_to_target_jibot.return_value = Result.ok()
    action = node()
    action.initialise()
    assert run(action) == Status.FAILURE
    wait_until(lambda: io.enable_vel_control_jibot.called)
    action.global_blackboard.set.assert_not_called()


def test_timeout_cancels_flight_and_ignores_late_success(io):
    release = threading.Event()
    captured = []

    def blocked(**kwargs):
        captured.append(kwargs["cancel_event"])
        release.wait(0.5)
        return Result.ok(data={"task_id": "late"})

    io.base_move_to_target_jibot.side_effect = blocked
    action = node(service_call_timeout=0.02)
    action.initialise()
    try:
        assert run(action) == Status.FAILURE
        assert captured[0].is_set()
        release.set()
        time.sleep(0.02)
        assert action.update() == Status.FAILURE
        action.initialise()
        assert action.update() == Status.FAILURE
        assert io.base_move_to_target_jibot.call_count == 1
        action.global_blackboard.set.assert_not_called()
    finally:
        release.set()


def test_relative_waits_for_arrival_and_defaults_to_radians(io):
    io.check_arrived_jibot.side_effect = [
        Result.ok(data={"arrived": False, "success": False, "status": 0, "message": "running"}),
        Result.ok(data={"arrived": True, "success": True, "status": 2, "message": "arrived"}),
    ]
    action = node(GraspMtbfRelativeMove, x=-0.3, theta=0.5, poll_interval=0.001)
    action.initialise()
    assert run(action) == Status.SUCCESS
    assert io.base_move_relative_jibot.call_args.kwargs["theta"] == 0.5
    assert io.check_arrived_jibot.call_count == 2
    assert all(call.kwargs["blocking"] is False for call in io.check_arrived_jibot.call_args_list)
    io.enable_vel_control_jibot.assert_not_called()


def test_relative_arrival_failure_latches_and_restores(io):
    io.check_arrived_jibot.return_value = Result.ok(data={
        "arrived": False, "success": False, "status": 0, "message": "interrupted"})
    action = node(GraspMtbfRelativeMove)
    action.initialise()
    assert run(action) == Status.FAILURE
    wait_until(lambda: io.enable_vel_control_jibot.called)
    assert action.update() == Status.FAILURE
    assert io.base_move_relative_jibot.call_count == 1


def test_failed_control_release_attempts_independent_restore(io):
    io.enable_vel_control_jibot.side_effect = lambda enable, **kwargs: (
        Result.ok() if enable else Result.fail("release failed"))
    action = node(GraspMtbfChassisControl, enable="False")
    action.initialise()
    assert run(action) == Status.FAILURE
    wait_until(lambda: io.enable_vel_control_jibot.call_count == 2)
    assert [call.args[0] for call in io.enable_vel_control_jibot.call_args_list] == [False, True]
    action.initialise()
    assert action.update() == Status.FAILURE
    assert io.enable_vel_control_jibot.call_count == 2


def test_failed_restore_does_not_retry_and_report_success(io):
    io.enable_vel_control_jibot.return_value = Result.fail("restore failed")
    action = node(GraspMtbfChassisControl, enable=True)
    action.initialise()
    assert run(action) == Status.FAILURE
    assert action.update() == Status.FAILURE
    assert io.enable_vel_control_jibot.call_count == 1
