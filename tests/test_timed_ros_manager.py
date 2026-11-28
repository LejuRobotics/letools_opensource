"""MultiCmd ROS后端与兼容桥离线测试，不初始化SDK或ROS。"""

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from core.domain.result import Result
from adapters.hardware.leju_wheeled.services.ros_manager import timed_ros_manager as module

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.unit
COMMANDS = [{"planner_index": 2, "desire_time": 2.0, "cmd_vec": [0.1, 0.2, 0.3, 0.4]}]


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setattr(module.rospy.core, "is_initialized", lambda: True)
    wait = Mock()
    client = Mock(return_value=SimpleNamespace(isSuccess=True, actualTime=3.0))
    factory = Mock(return_value=client)
    monkeypatch.setattr(module.rospy, "wait_for_service", wait)
    monkeypatch.setattr(module.rospy, "ServiceProxy", factory)
    monkeypatch.setattr(module.rospy, "init_node", Mock(side_effect=AssertionError("不能init ROS")))
    return wait, factory, client


def test_manager_has_no_sdk_dependency_or_base_class(ros):
    tree = ast.parse(Path(module.__file__).read_text())
    imports = [n.module or "" for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
    assert not any("sdk" in name for name in imports)
    assert module.TimedRosManager.__bases__ == (object,)
    manager = module.TimedRosManager()
    assert manager.initialize().success
    assert manager.is_initialized()
    for mock in ros:
        mock.assert_not_called()
    assert manager.shutdown().success
    assert not manager.is_initialized()
    assert not manager.send_multi_commands(COMMANDS).success


@pytest.mark.parametrize("sync", [False, True])
def test_request_fields_and_result_are_preserved(ros, sync):
    manager = module.TimedRosManager()
    manager.initialize()
    commands = COMMANDS + [{"planner_index": 8, "desire_time": 4.0, "cmd_vec": [0.2] * 7}]
    result = manager.send_multi_commands(commands, sync)
    assert result.success and result.data == {"actual_time": 3.0}
    wait, factory, client = ros
    wait.assert_called_once_with("/mobile_manipulator_timed_multi_cmd", timeout=5.0)
    assert factory.call_count == client.call_count == 1
    request = client.call_args.args[0]
    assert request.isSync is sync
    assert len(request.timedCmdVec) == 2
    for actual, expected in zip(request.timedCmdVec, commands):
        assert actual.planner_index == expected["planner_index"]
        assert actual.desireTime == expected["desire_time"]
        assert actual.cmdVec == expected["cmd_vec"]


@pytest.mark.parametrize("stage", ["wait", "service", "response"])
def test_failure_does_not_retry(ros, stage):
    wait, _, client = ros
    if stage == "wait":
        wait.side_effect = module.rospy.ROSException("timeout")
    elif stage == "service":
        client.side_effect = module.rospy.ServiceException("failure")
    else:
        client.return_value = SimpleNamespace(isSuccess=False, message="refused")
    manager = module.TimedRosManager()
    manager.initialize()
    result = manager.send_multi_commands(COMMANDS)
    assert not result.success
    assert wait.call_count == 1
    assert client.call_count == (0 if stage == "wait" else 1)
    if stage == "response":
        assert result.message == "refused"


def test_no_ros_and_invalid_input_fail_before_io(ros, monkeypatch):
    manager = module.TimedRosManager()
    monkeypatch.setattr(module.rospy.core, "is_initialized", lambda: False)
    assert not manager.initialize().success
    assert not manager.send_multi_commands(COMMANDS).success
    monkeypatch.setattr(module.rospy.core, "is_initialized", lambda: True)
    manager.initialize()
    assert not manager.send_multi_commands([]).success
    assert not manager.send_multi_commands(COMMANDS, "true").success
    assert not manager.send_multi_commands([{}]).success
    for mock in ros:
        mock.assert_not_called()


@pytest.mark.parametrize("unit", ["deg", "rad"])
def test_shared_mixin_uses_ros_manager_and_preserves_unit_conversion(unit):
    import math
    spec = importlib.util.spec_from_file_location("shared_timed_test", ROOT / "adapters/hardware/leju_wheeled/mixins/timed_command_mixin.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    hw = mod.TimedCommandMixin()
    hw._timed_cmd_manager = Mock(side_effect=AssertionError("不得经SDK后端"))
    hw._timed_ros_manager = Mock()
    hw._to_rad = (lambda value: math.radians(value)) if unit == "deg" else float
    result = Result.ok(data={"actual_time": 4.0})
    hw._timed_ros_manager.send_multi_commands.return_value = result
    assert hw.send_timed_multi_commands(COMMANDS, is_sync=True) is result
    converted = COMMANDS[0]["cmd_vec"][:2] + [hw._to_rad(v) for v in COMMANDS[0]["cmd_vec"][2:]]
    hw._timed_ros_manager.send_multi_commands.assert_called_once_with(
        commands=[dict(COMMANDS[0], cmd_vec=converted)], is_sync=True)
    assert not hw._timed_cmd_manager.mock_calls
    hw._timed_ros_manager = None
    assert not hw.send_timed_multi_commands(COMMANDS).success


def test_legacy_sdk_manager_delegates_only_multi_to_ros_manager(ros):
    from adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager import TimedCmdManager
    manager = TimedCmdManager()
    manager._initialized = True
    manager.robot_sdk = Mock()  # 不调用SDK.initialize。
    try:
        assert manager.send_multi_commands(COMMANDS, True).success
        assert isinstance(manager._multi_ros_manager, module.TimedRosManager)
        backend = manager._multi_ros_manager
        manager.shutdown()
        assert not backend.is_initialized()
        assert manager._multi_ros_manager is None
        assert not manager.send_multi_commands(COMMANDS).success
    finally:
        manager._initialized = False
