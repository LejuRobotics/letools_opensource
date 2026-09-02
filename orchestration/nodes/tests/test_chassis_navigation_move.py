# -*- coding: utf-8 -*-
"""ChassisNavigationMove 单元测试；所有硬件访问均为 mock。"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from py_trees.common import Status

from core.domain.result import Result
from orchestration.nodes.chassis_navigation_move import ChassisNavigationMove


def _node(**overrides):
    params = {
        "mode": "relative",
        "command": [0.1, 0.0, 0.0],
        "poll_interval": 0.2,
        "timeout": 60.0,
    }
    params.update(overrides)
    return ChassisNavigationMove("move", "move", None, params)


def _navigation_hardware():
    hardware = MagicMock()
    hardware.get_chassis_external_control_state.return_value = Result.ok(
        data={"state": False}
    )
    hardware.move_chassis_relative.return_value = Result.ok(
        data={"task_id": "relative-1"}
    )
    hardware.move_chassis_to_target.return_value = Result.ok(
        data={"task_id": "map-1"}
    )
    return hardware


def test_relative_move_submits_and_waits_without_blocking():
    node = _node(command=[0.1, -0.2, 0.3])
    hardware = _navigation_hardware()
    hardware.check_chassis_arrived.return_value = Result.ok(
        data={"arrived": False, "status": 1}
    )
    with patch(
        "orchestration.nodes.chassis_navigation_move.get_shared_hardware",
        return_value=hardware,
    ):
        node.initialise()
        assert node.update() == Status.RUNNING

    call = hardware.move_chassis_relative.call_args.kwargs
    assert (call["x"], call["y"], call["theta"]) == (0.1, -0.2, 0.3)
    hardware.move_chassis_to_target.assert_not_called()
    hardware.check_chassis_arrived.assert_called_once_with(
        task_id="relative-1", blocking=False, timeout=0.0
    )


def test_map_move_returns_success_after_arrival():
    node = _node(mode="map", command=[1.0, 2.0, -1.57])
    hardware = _navigation_hardware()
    hardware.check_chassis_arrived.return_value = Result.ok(
        data={"arrived": True, "status": 2}
    )
    with patch(
        "orchestration.nodes.chassis_navigation_move.get_shared_hardware",
        return_value=hardware,
    ):
        node.initialise()
        assert node.update() == Status.SUCCESS

    call = hardware.move_chassis_to_target.call_args.kwargs
    assert (call["x"], call["y"], call["theta"]) == (1.0, 2.0, -1.57)
    hardware.move_chassis_relative.assert_not_called()


def test_external_mode_is_automatically_switched_to_navigation():
    node = _node()
    hardware = _navigation_hardware()
    hardware.get_chassis_external_control_state.return_value = Result.ok(
        data={"state": True}
    )
    hardware.set_chassis_external_control.return_value = Result.ok(
        data={"state_after": False}
    )
    with patch(
        "orchestration.nodes.chassis_navigation_move.get_shared_hardware",
        return_value=hardware,
    ):
        node.initialise()

    hardware.set_chassis_external_control.assert_called_once_with(False)
    hardware.move_chassis_relative.assert_called_once()


def test_external_mode_can_be_rejected_without_submitting_motion():
    node = _node(auto_switch_navigation=False)
    hardware = _navigation_hardware()
    hardware.get_chassis_external_control_state.return_value = Result.ok(
        data={"state": True}
    )
    with patch(
        "orchestration.nodes.chassis_navigation_move.get_shared_hardware",
        return_value=hardware,
    ):
        node.initialise()
        assert node.update() == Status.FAILURE

    hardware.set_chassis_external_control.assert_not_called()
    hardware.move_chassis_relative.assert_not_called()


def test_invalid_command_fails_before_hardware_access():
    node = _node(command=[0.1, 0.0])
    with patch(
        "orchestration.nodes.chassis_navigation_move.get_shared_hardware"
    ) as get_hardware:
        node.initialise()
        assert node.update() == Status.FAILURE
    get_hardware.assert_not_called()


def test_disabled_node_succeeds_without_parsing_or_hardware_access():
    node = _node(enabled=False, command="not-json")
    with patch(
        "orchestration.nodes.chassis_navigation_move.get_shared_hardware"
    ) as get_hardware:
        node.initialise()
        assert node.update() == Status.SUCCESS
    get_hardware.assert_not_called()


def test_missing_task_id_is_failure():
    node = _node()
    hardware = _navigation_hardware()
    hardware.move_chassis_relative.return_value = Result.ok(data={})
    with patch(
        "orchestration.nodes.chassis_navigation_move.get_shared_hardware",
        return_value=hardware,
    ):
        node.initialise()
        assert node.update() == Status.FAILURE


def test_boolean_string_false_is_not_treated_as_true():
    options = ChassisNavigationMove._parse_options(
        {
            "avoid_enabled": "false",
            "allow_rotation": "false",
        }
    )
    assert options.avoid_enabled is False
    assert options.allow_rotation is False


def test_interrupted_running_task_stops_navigation():
    node = _node()
    hardware = _navigation_hardware()
    hardware.set_chassis_external_control.return_value = Result.ok()
    with patch(
        "orchestration.nodes.chassis_navigation_move.get_shared_hardware",
        return_value=hardware,
    ):
        node.initialise()
        node.terminate(Status.INVALID)

    hardware.set_chassis_external_control.assert_called_once_with(True)


def test_completed_task_does_not_switch_control_mode():
    node = _node()
    hardware = _navigation_hardware()
    hardware.check_chassis_arrived.return_value = Result.ok(
        data={"arrived": True, "status": 2}
    )
    with patch(
        "orchestration.nodes.chassis_navigation_move.get_shared_hardware",
        return_value=hardware,
    ):
        node.initialise()
        assert node.update() == Status.SUCCESS
        node.terminate(Status.SUCCESS)

    hardware.set_chassis_external_control.assert_not_called()


if __name__ == "__main__":
    tests = (
        test_relative_move_submits_and_waits_without_blocking,
        test_map_move_returns_success_after_arrival,
        test_external_mode_is_automatically_switched_to_navigation,
        test_external_mode_can_be_rejected_without_submitting_motion,
        test_invalid_command_fails_before_hardware_access,
        test_disabled_node_succeeds_without_parsing_or_hardware_access,
        test_missing_task_id_is_failure,
        test_boolean_string_false_is_not_treated_as_true,
        test_interrupted_running_task_stops_navigation,
        test_completed_task_does_not_switch_control_mode,
    )
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"ChassisNavigationMove: {len(tests)} tests passed")
