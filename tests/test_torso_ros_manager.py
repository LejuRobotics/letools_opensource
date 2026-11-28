"""躯干 ROS 服务与共享生命周期离线测试；禁止真实 ROS 通信及 SDK。"""

import ast
import builtins
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from adapters.hardware.leju_wheeled import hardware as hardware_module
from adapters.hardware.leju_wheeled.mixins import lifecycle_mixin as lifecycle
from adapters.hardware.leju_wheeled.services.ros_manager import torso_ros_manager as module
from core.domain.result import Result

pytestmark = pytest.mark.unit
POSE_SERVICE = "/mobile_manipulator_get_torso_initial_pose"
RESET_SERVICE = "/mobile_manipulator_reset_torso"
METHODS = ["get_torso_pose_initial", "reset_torso"]


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setattr(module.rospy.core, "is_initialized", Mock(return_value=True))
    forbidden = {}
    for name in ("init_node", "sleep", "Publisher", "Subscriber"):
        forbidden[name] = Mock(side_effect=AssertionError("unexpected ROS call: " + name))
        monkeypatch.setattr(module.rospy, name, forbidden[name])
    wait = Mock()
    client = Mock()
    factory = Mock(return_value=client)
    monkeypatch.setattr(module.rospy, "wait_for_service", wait)
    monkeypatch.setattr(module.rospy, "ServiceProxy", factory)
    yield SimpleNamespace(wait=wait, factory=factory, client=client)
    for operation in forbidden.values():
        operation.assert_not_called()


@pytest.fixture
def no_sdk(monkeypatch):
    original_import = builtins.__import__
    attempts = []

    def guarded_import(name, *args, **kwargs):
        if "sdk_manager" in name or name.startswith("kuavo_humanoid_sdk"):
            attempts.append(name)
            raise AssertionError("SDK import forbidden: " + name)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    yield
    assert not attempts


def make_manager():
    manager = module.TorsoRosManager()
    assert manager.initialize().success
    return manager


def pose_response():
    return SimpleNamespace(
        result=True, message="pose ready",
        linear=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        angular=SimpleNamespace(x=0.4, y=0.5, z=0.6),
    )


def test_lifecycle_has_no_io_or_sdk_dependency(ros, no_sdk):
    tree = ast.parse(Path(module.__file__).read_text())
    imports = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            imports.append(node.module or "")
        elif isinstance(node, ast.Import):
            imports.extend(alias.name for alias in node.names)
    assert not any("sdk" in name.lower() for name in imports)
    assert module.TorsoRosManager.__bases__ == (object,)
    manager = make_manager()
    assert manager.is_initialized()
    assert manager.shutdown().success
    assert not manager.is_initialized()
    for method in METHODS:
        assert not getattr(manager, method)().success
    ros.wait.assert_not_called()
    ros.factory.assert_not_called()


@pytest.mark.parametrize("state", ["new", "ros_uninitialized", "ros_lost", "shutdown"])
@pytest.mark.parametrize("method", METHODS)
def test_uninitialized_fails_before_io(ros, monkeypatch, state, method):
    manager = module.TorsoRosManager()
    if state in ("ros_lost", "shutdown"):
        assert manager.initialize().success
    if state in ("ros_uninitialized", "ros_lost"):
        monkeypatch.setattr(module.rospy.core, "is_initialized", lambda: False)
    if state == "ros_uninitialized":
        assert not manager.initialize().success
    if state == "shutdown":
        assert manager.shutdown().success
    result = getattr(manager, method)()
    assert not result.success
    assert "未初始化" in result.message
    ros.wait.assert_not_called()
    ros.factory.assert_not_called()


def test_get_pose_request_and_raw_units(ros, no_sdk):
    from kuavo_msgs.srv import getLbTorsoInitialPose, getLbTorsoInitialPoseRequest

    ros.client.return_value = pose_response()
    result = make_manager().get_torso_pose_initial()
    assert result.success
    assert result.message == "pose ready"
    assert result.data == {"position": [0.1, 0.2, 0.3], "euler": [0.6, 0.5, 0.4]}
    ros.wait.assert_called_once_with(POSE_SERVICE, timeout=5.0)
    ros.factory.assert_called_once_with(POSE_SERVICE, getLbTorsoInitialPose)
    ros.client.assert_called_once()
    request = ros.client.call_args.args[0]
    assert isinstance(request, getLbTorsoInitialPoseRequest)
    assert request.isNeed is True


@pytest.mark.parametrize("field", ["linear.x", "linear.y", "linear.z", "angular.x", "angular.y", "angular.z"])
@pytest.mark.parametrize("value", [True, False, float("nan"), float("inf"), float("-inf"), "0.1", None])
def test_get_pose_rejects_nonfinite_and_non_numeric_components(ros, field, value):
    response = pose_response()
    vector, axis = field.split(".")
    setattr(getattr(response, vector), axis, value)
    ros.client.return_value = response
    result = make_manager().get_torso_pose_initial()
    assert not result.success
    assert result.data is None
    ros.client.assert_called_once()


@pytest.mark.parametrize("message,estimated", [
    ("2.5", 2.5), ("0", 0.0), (" 1.25 ", 1.25), ("1e2", 100.0),
    ("accepted", None), ("", None), ("-1", None), ("nan", None),
    ("inf", None), ("-inf", None), ("1e999", None),
    ("True", None), ("False", None), ("true", None), ("false", None),
    ("time=2.5", None), ("2 seconds", None),
])
def test_reset_preserves_acceptance_and_message(ros, no_sdk, message, estimated):
    from std_srvs.srv import SetBool, SetBoolRequest

    ros.client.return_value = SimpleNamespace(success=True, message=message)
    result = make_manager().reset_torso()
    assert result.success
    assert result.message == message
    assert result.data == {"accepted": True, "estimated_time": estimated}
    if estimated is not None:
        assert type(result.data["estimated_time"]) is float
    ros.wait.assert_called_once_with(RESET_SERVICE, timeout=5.0)
    ros.factory.assert_called_once_with(RESET_SERVICE, SetBool)
    ros.client.assert_called_once()
    request = ros.client.call_args.args[0]
    assert isinstance(request, SetBoolRequest)
    assert request.data is True


@pytest.mark.parametrize("method", METHODS)
@pytest.mark.parametrize("stage", ["wait", "service", "response", "unexpected"])
def test_errors_are_not_retried(ros, method, stage):
    if stage == "wait":
        ros.wait.side_effect = module.rospy.ROSException("discovery timeout")
        message = "discovery timeout"
    elif stage == "service":
        ros.client.side_effect = module.rospy.ServiceException("transport failed")
        message = "transport failed"
    elif stage == "unexpected":
        ros.client.side_effect = RuntimeError("unexpected failure")
        message = "unexpected failure"
    else:
        ros.client.return_value = SimpleNamespace(result=False, success=False, message="refused verbatim")
        message = "refused verbatim"
    result = getattr(make_manager(), method)()
    assert not result.success
    assert message in result.message
    if stage == "response":
        assert result.message == message
    assert ros.wait.call_count == 1
    assert ros.factory.call_count == ros.client.call_count == (0 if stage == "wait" else 1)


@pytest.fixture
def hardware_runtime(ros, monkeypatch):
    driver = Mock()
    monkeypatch.setattr(hardware_module, "LejuEndEffector", Mock(return_value=driver))
    monkeypatch.setattr(lifecycle.os, "makedirs", Mock())

    def create(**options):
        config = {
            "skip_camera": True, "skip_end_effector": True,
            "skip_state_manager": True, "skip_force_publishers": True,
            "skip_sdk_managers": True,
        }
        config.update(options)
        return hardware_module.LejuWheeledArmHardware(config)

    yield create
    ros.wait.assert_not_called()
    ros.factory.assert_not_called()
    driver.connect.assert_not_called()


@pytest.mark.parametrize("options,enabled", [
    ({}, {"timed", "torso"}),
    ({"ros_managers_whitelist": None}, {"timed", "torso"}),
    ({"ros_managers_whitelist": ["timed"]}, {"timed"}),
    ({"ros_managers_whitelist": ["torso"]}, {"torso"}),
    ({"ros_managers_whitelist": ["timed", "torso"]}, {"timed", "torso"}),
    ({"ros_managers_whitelist": []}, set()),
    ({"ros_managers_whitelist": ["unknown"]}, set()),
    ({"skip_ros_managers": True}, set()),
    ({"skip_ros_managers": True, "ros_managers_whitelist": ["torso"]}, set()),
    ({"skip_sdk_managers": False, "sdk_managers_whitelist": []}, {"timed", "torso"}),
])
def test_shared_lifecycle_routes_ros_whitelist(hardware_runtime, no_sdk, options, enabled):
    hardware = hardware_runtime(**options)
    assert hardware._torso_ros_manager is None
    assert hardware.initialize().success
    managers = {}
    for key in ("timed", "torso"):
        manager = getattr(hardware, "_" + key + "_ros_manager")
        assert (manager is not None) == (key in enabled)
        if manager is not None:
            assert manager.is_initialized()
            managers[key] = manager
    assert hardware._timed_cmd_manager is None
    assert hardware._arm_sdk_manager is None
    assert hardware._low_level_sdk_manager is None
    assert hardware.shutdown().success
    for manager in managers.values():
        assert not manager.is_initialized()
    assert hardware._timed_ros_manager is None
    assert hardware._torso_ros_manager is None


@pytest.mark.parametrize("raises", [False, True])
def test_torso_initialization_failure_remains_optional(hardware_runtime, no_sdk, monkeypatch, raises):
    manager = Mock()
    if raises:
        manager.initialize.side_effect = RuntimeError("torso init failed")
    else:
        manager.initialize.return_value = Result.fail("torso init failed")
    factory = Mock(return_value=manager)
    monkeypatch.setattr(module, "TorsoRosManager", factory)
    hardware = hardware_runtime()
    assert hardware.initialize().success
    assert hardware._torso_ros_manager is None
    assert hardware._timed_ros_manager.is_initialized()
    factory.assert_called_once_with()
    manager.initialize.assert_called_once_with()
    assert hardware.shutdown().success
    manager.shutdown.assert_not_called()
