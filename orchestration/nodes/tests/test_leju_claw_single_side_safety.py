"""乐聚夹爪单侧安全下发测试；所有 ROS 服务调用均已 mock。"""

from types import SimpleNamespace
from unittest.mock import MagicMock

from core.common.ros_environment import ensure_local_ros_python_path
from core.domain.end_effector import DualGripperCommand, EndEffectorType, GripperCommand
from drivers.leju.end_effector import LejuEndEffector


def _driver(state_timeout=0.01):
    ensure_local_ros_python_path()
    driver = LejuEndEffector(
        {"type": EndEffectorType.LEJU_CLAW.value, "claw_state_timeout": state_timeout}
    )
    driver._connected = True
    response = SimpleNamespace(success=True, message="success")
    driver._claw_service = MagicMock(return_value=response)
    return driver


def _state(left, right, names=("left_claw", "right_claw")):
    return SimpleNamespace(
        data=SimpleNamespace(name=list(names), position=[left, right])
    )


def test_single_left_close_preserves_actual_right_position():
    driver = _driver()
    driver._on_claw_state(_state(25.0, 80.0))

    result = driver.send_command(
        "both",
        DualGripperCommand(
            left_position=100.0,
            right_position=None,
            velocity=40.0,
            effort=1.2,
        ),
    )

    assert result.success
    request = driver._claw_service.call_args.args[0]
    assert list(request.data.name) == ["left_claw", "right_claw"]
    assert list(request.data.position) == [100.0, 80.0]
    assert list(request.data.velocity) == [40.0, 40.0]
    assert list(request.data.effort) == [1.2, 1.2]


def test_single_right_open_uses_names_to_read_reordered_state():
    driver = _driver()
    driver._on_claw_state(
        _state(70.0, 30.0, names=("right_claw", "left_claw"))
    )

    result = driver.send_command(
        "right",
        GripperCommand(position=0.0, velocity=50.0, effort=1.0),
    )

    assert result.success
    request = driver._claw_service.call_args.args[0]
    assert list(request.data.position) == [30.0, 0.0]


def test_single_side_command_fails_when_state_is_unavailable():
    driver = _driver()

    result = driver.send_command(
        "both",
        DualGripperCommand(left_position=None, right_position=100.0),
    )

    assert not result.success
    assert "/leju_claw_state" in result.message
    driver._claw_service.assert_not_called()


def test_dual_command_does_not_require_state_feedback():
    driver = _driver()

    result = driver.send_command(
        "both",
        DualGripperCommand(left_position=20.0, right_position=90.0),
    )

    assert result.success
    request = driver._claw_service.call_args.args[0]
    assert list(request.data.position) == [20.0, 90.0]
