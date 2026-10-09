# -*- coding: utf-8 -*-
"""GraspMtbfCalcLegMove 安全互锁单元测试；所有硬件和 TF 均为 mock。"""

import time
import pytest

pytestmark = pytest.mark.unit
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from py_trees.common import Status

from core.domain.pose import Pose6D
from core.domain.result import Result
from orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove import GraspMtbfCalcLegMove
from orchestration.nodes.base_node import BaseAction


def _hardware(mode=3, base_velocity=None, motor_errors=None):
    hardware = MagicMock()
    hardware.set_mpc_mode.return_value = Result.ok()
    hardware.get_mpc_control_mode.return_value = mode
    hardware.get_base_cmd_vel.return_value = base_velocity or {
        "vx": 0.0, "vy": 0.0, "wz": 0.0,
    }
    hardware.get_motor_error_codes.return_value = (
        [0.0] * 20 if motor_errors is None else motor_errors
    )
    hardware.get_torso_target_6d.return_value = {
        "values": [0.15, 0.0, 1.0, 0.0, 0.0, 0.0]
    }
    hardware.send_torso_pose_timed.return_value = Result.ok(
        data={"actual_time": 1.0}
    )
    hardware.set_motion_control_enabled.return_value = Result.ok()
    return hardware


def _node(hardware, blackboard=None, **overrides):
    params = {
        "leg_mode": "leg",
        "offset_x": 0.15,
        "offset_y": 0.0,
        "offset_z": 1.0,
        "control_base": False,
        "total_time": 1.0,
        "settle_time": 0.0,
        "mode_timeout": 0.1,
    }
    params.update(overrides)
    blackboard = blackboard or MagicMock()
    with patch.object(
        BaseAction, "attach_blackboard_client", create=True, return_value=blackboard
    ):
        node = GraspMtbfCalcLegMove("leg", "leg", None, params)
    return node


def test_base_arm_waits_for_command_readiness_and_motion_completion():
    hardware = _hardware()
    node = _node(hardware)
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io",
        return_value=hardware,
    ):
        node.initialise()
        assert node.update() == Status.RUNNING

    hardware.send_torso_pose_timed.assert_called_once()
    command = hardware.send_torso_pose_timed.call_args.kwargs
    assert command["x"] == 0.15
    assert command["z"] == 1.0

    node._target_confirmed = True
    node._target_confirm_at = float("inf")
    node._motion_finish_at = time.monotonic() - 1.0
    assert node.update() == Status.SUCCESS


def test_fixed_base_arm_submits_without_requesting_or_waiting_for_old_modes():
    hardware = _hardware(mode=3)
    node = _node(hardware)
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io", return_value=hardware):
        node.initialise()
        assert node.update() == Status.RUNNING
    hardware.send_torso_pose_timed.assert_called_once()
    hardware.set_mpc_mode.assert_not_called()
    hardware.get_mpc_control_mode.assert_not_called()
    hardware.set_motion_control_enabled.assert_not_called()


def test_nonzero_base_velocity_command_is_rejected():
    hardware = _hardware(
        mode=1,
        base_velocity={"vx": 0.08, "vy": 0.0, "wz": 0.0},
    )
    node = _node(hardware)
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io",
        return_value=hardware,
    ):
        node.initialise()
    node._deadline = time.monotonic() - 1.0

    assert node.update() == Status.FAILURE
    hardware.send_torso_pose_timed.assert_not_called()


def test_missing_base_velocity_feedback_never_submits_command():
    hardware = _hardware(mode=1)
    hardware.get_base_cmd_vel.return_value = None
    node = _node(hardware)
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io",
        return_value=hardware,
    ):
        node.initialise()
    node._deadline = time.monotonic() - 1.0

    assert node.update() == Status.FAILURE
    assert "base_cmd_vel" in node.feedback_message
    hardware.send_torso_pose_timed.assert_not_called()


def test_missing_motor_error_feedback_never_submits_command():
    hardware = _hardware(mode=1)
    hardware.get_motor_error_codes.return_value = None
    node = _node(hardware)
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io",
        return_value=hardware,
    ):
        node.initialise()
    node._deadline = time.monotonic() - 1.0

    assert node.update() == Status.FAILURE
    assert "电机错误码反馈超时" in node.feedback_message
    hardware.send_torso_pose_timed.assert_not_called()


def test_missing_torso_target_feedback_never_reports_success():
    hardware = _hardware(mode=1)
    node = _node(hardware)
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io",
        return_value=hardware,
    ):
        node.initialise()
        assert node.update() == Status.RUNNING

    hardware.get_torso_target_6d.return_value = None
    node._target_confirm_at = time.monotonic() - 1.0
    node._motion_finish_at = time.monotonic() - 1.0
    node._deadline = time.monotonic() - 0.5
    assert node.update() == Status.FAILURE
    assert "终点反馈" in node.feedback_message


def test_mismatched_torso_target_feedback_triggers_safety_stop():
    hardware = _hardware(mode=1)
    hardware.get_torso_target_6d.side_effect = [
        {"values": [0.15, 0.0, 1.0, 0.0, 0.0, 0.0]},
        {"values": [0.15, 0.0, 1.0, 0.0, 0.0, 0.0]},
        {"values": [0.28, 0.0, 1.0, 0.0, 0.0, 0.0]},
    ]
    node = _node(hardware)
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io",
        return_value=hardware,
    ):
        node.initialise()
        assert node.update() == Status.RUNNING

    node._target_confirm_at = time.monotonic() - 1.0
    assert node.update() == Status.FAILURE
    assert "反馈的躯干目标与下发值不一致" in node.feedback_message
    hardware.set_motion_control_enabled.assert_called_once_with(False)


def test_existing_motor_error_blocks_motion_before_command():
    hardware = _hardware(motor_errors=[1.0, 0.0, 0.0, 0.0])
    node = _node(hardware)
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io",
        return_value=hardware,
    ):
        node.initialise()

    assert node.update() == Status.FAILURE
    hardware.send_torso_pose_timed.assert_not_called()
    hardware.set_motion_control_enabled.assert_called_once_with(False)


def test_dangerous_direct_torso_x_is_rejected_by_workspace_guard():
    hardware = _hardware()
    node = _node(hardware, offset_x=0.657541)
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io",
        return_value=hardware,
    ):
        node.initialise()

    assert node.update() == Status.FAILURE
    assert "超出安全范围" in node.feedback_message
    hardware.set_mpc_mode.assert_not_called()
    hardware.send_torso_pose_timed.assert_not_called()


def test_target_mode_never_uses_tag_transform_as_torso_x():
    hardware = _hardware()
    blackboard = MagicMock()
    blackboard.latest_tag_0 = SimpleNamespace(
        pose_in_world=Pose6D(x=1.0, y=0.0, z=1.0, yaw=0.0)
    )
    node = _node(
        hardware,
        blackboard=blackboard,
        leg_mode="target",
        tag_id=0,
        fixed_torso_x=0.15,
        offset_z=0.45,
    )
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io",
        return_value=hardware,
    ), patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove._base_from_odom",
        return_value=Pose6D(x=0.657541, y=0.0, z=0.997504),
    ):
        node.initialise()
        assert node.update() == Status.RUNNING

    command = hardware.send_torso_pose_timed.call_args.kwargs
    assert command["x"] == 0.15
    assert command["z"] == 0.997504


@pytest.mark.parametrize("duration", [float("nan"), float("inf"), -1.0, 61.0, "invalid"])
def test_invalid_duration_stops_and_blocks_retry(duration):
    hardware = _hardware()
    hardware.send_torso_pose_timed.return_value = Result.ok(data={"actual_time": duration})
    node = _node(hardware)
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io", return_value=hardware):
        node.initialise()
        assert node.update() == Status.FAILURE
        node.initialise()
        assert node.update() == Status.FAILURE
    hardware.send_torso_pose_timed.assert_called_once()
    hardware.set_motion_control_enabled.assert_called_once_with(False)
    hardware.set_mpc_mode.assert_not_called()


def test_ready_deadline_prevents_late_submission():
    hardware = _hardware()
    node = _node(hardware)
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io", return_value=hardware):
        node.initialise()
    node._deadline = time.monotonic() - 1.0
    assert node.update() == Status.FAILURE
    hardware.send_torso_pose_timed.assert_not_called()


def test_duration_starts_after_service_response(monkeypatch):
    import importlib
    module = importlib.import_module("orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove")
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock.now))
    hardware = _hardware()
    def send(**kwargs):
        clock.now += 1.0
        return Result.ok(data={"actual_time": 3.0})
    hardware.send_torso_pose_timed.side_effect = send
    node = _node(hardware)
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io", return_value=hardware):
        node.initialise()
        assert node.update() == Status.RUNNING
    assert node._motion_finish_at == 104.0
    clock.now = 103.5
    assert node.update() == Status.RUNNING
    clock.now = 104.0
    assert node.update() == Status.SUCCESS


def test_service_exception_fails_without_escaping_tree():
    hardware = _hardware()
    hardware.send_torso_pose_timed.side_effect = RuntimeError("connection lost")
    node = _node(hardware)
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io", return_value=hardware):
        node.initialise()
        assert node.update() == Status.FAILURE
    assert "connection lost" in node.feedback_message
    hardware.set_motion_control_enabled.assert_called_once_with(False)


def test_cold_start_waits_for_torso_feedback_without_sending():
    hardware = _hardware()
    hardware.get_torso_target_6d.return_value = None
    node = _node(hardware)
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io", return_value=hardware):
        node.initialise()
    assert node.update() == Status.RUNNING
    hardware.send_torso_pose_timed.assert_not_called()
    node._deadline = time.monotonic() - 1
    assert node.update() == Status.FAILURE
    hardware.send_torso_pose_timed.assert_not_called()


def test_late_initial_torso_feedback_is_checked_before_sending():
    hardware = _hardware()
    hardware.get_torso_target_6d.return_value = None
    node = _node(hardware)
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io", return_value=hardware):
        node.initialise()
    assert node.update() == Status.RUNNING
    hardware.get_torso_target_6d.return_value = {"values": [-0.1, 0, 1, 0, 0, 0]}
    assert node.update() == Status.FAILURE
    assert "x跳变过大" in node.feedback_message
    hardware.send_torso_pose_timed.assert_not_called()


def test_valid_initial_torso_feedback_releases_ready_wait():
    hardware = _hardware()
    hardware.get_torso_target_6d.return_value = None
    node = _node(hardware)
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfCalcLegMove.get_scene_io", return_value=hardware):
        node.initialise()
    assert node.update() == Status.RUNNING
    hardware.get_torso_target_6d.return_value = {"values": [0.15, 0, 1, 0, 0, 0]}
    assert node.update() == Status.RUNNING
    hardware.send_torso_pose_timed.assert_called_once()
    assert node._phase == "wait_motion"
