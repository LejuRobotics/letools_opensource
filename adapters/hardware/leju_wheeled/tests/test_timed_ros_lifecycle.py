"""ROS/SDK 生命周期隔离；所有 ROS 通信与硬件驱动均由 Mock 截断。"""

import builtins
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

from adapters.hardware.leju_wheeled import hardware as hardware_module
from adapters.hardware.leju_wheeled.mixins import lifecycle_mixin as lifecycle
from core.domain.result import Result

pytestmark = pytest.mark.unit

SDK_PACKAGE = "adapters.hardware.leju_wheeled.services.sdk_manager"
ROS_MODULE = "adapters.hardware.leju_wheeled.services.ros_manager.timed_ros_manager"
SDK_MANAGERS = {
    "timed": ("timed_cmd_manager", "TimedCmdManager", "_timed_cmd_manager"),
    "arm": ("arm_sdk_manager", "ArmSDKManager", "_arm_sdk_manager"),
    "low": ("low_level_sdk_manager", "LowLevelSDKManager", "_low_level_sdk_manager"),
}


@pytest.fixture
def runtime(monkeypatch):
    """实际 Hardware 生命周期，但不允许创建 ROS 通信端点或真实驱动。"""
    driver = Mock()
    monkeypatch.setattr(hardware_module, "LejuEndEffector", Mock(return_value=driver))
    monkeypatch.setattr(lifecycle.rospy.core, "is_initialized", Mock(return_value=True))
    calls = {}
    for name in ("init_node", "wait_for_service", "ServiceProxy", "Publisher", "Subscriber"):
        calls[name] = Mock(side_effect=AssertionError("unexpected ROS call: " + name))
        monkeypatch.setattr(lifecycle.rospy, name, calls[name])
    monkeypatch.setattr(lifecycle, "init_logging", Mock())
    monkeypatch.setattr(lifecycle.os, "makedirs", Mock())

    def create(**config):
        settings = {
            "skip_camera": True,
            "skip_end_effector": True,
            "skip_state_manager": True,
            "skip_force_publishers": True,
            "sdk_managers_whitelist": [],
        }
        settings.update(config)
        return hardware_module.LejuWheeledArmHardware(settings)

    yield SimpleNamespace(create=create, driver=driver, calls=calls)
    for name, operation in calls.items():
        if name != "init_node":
            operation.assert_not_called()


@pytest.fixture
def no_sdk_imports(monkeypatch):
    original_import = builtins.__import__
    attempts = []

    def guarded_import(name, *args, **kwargs):
        if "sdk_manager" in name or name.startswith("kuavo_humanoid_sdk"):
            attempts.append(name)
            raise AssertionError("SDK import forbidden in ROS-only lifecycle: " + name)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    yield attempts
    # initialize() treats optional failures as non-fatal, so also check swallowed imports.
    assert attempts == []


@pytest.fixture
def ros_manager(monkeypatch):
    module = ModuleType(ROS_MODULE)
    manager = Mock()
    manager.initialize.return_value = Result.ok()
    manager.shutdown.return_value = Result.ok()
    module.TimedRosManager = Mock(return_value=manager)
    monkeypatch.setitem(sys.modules, ROS_MODULE, module)
    return SimpleNamespace(factory=module.TimedRosManager, manager=manager)


@pytest.fixture
def sdk_managers(monkeypatch):
    package = ModuleType(SDK_PACKAGE)
    package.__path__ = []
    monkeypatch.setitem(sys.modules, SDK_PACKAGE, package)
    managers = {}
    for key, (module_name, class_name, _) in SDK_MANAGERS.items():
        module = ModuleType(SDK_PACKAGE + "." + module_name)
        manager = Mock()
        manager.initialize.return_value = Result.ok()
        manager.shutdown.return_value = Result.ok()
        factory = Mock(return_value=manager)
        setattr(module, class_name, factory)
        monkeypatch.setitem(sys.modules, module.__name__, module)
        managers[key] = SimpleNamespace(factory=factory, manager=manager)
    return managers


@pytest.mark.parametrize("options", [
    {"sdk_managers_whitelist": []},
    {"skip_sdk_managers": True, "sdk_managers_whitelist": None},
    {"skip_sdk_managers": True, "sdk_managers_whitelist": ["timed", "arm", "low"]},
])
def test_ros_only_initialize_and_shutdown(runtime, no_sdk_imports, options):
    hardware = runtime.create(**options)
    assert hardware._timed_ros_manager is None
    assert hardware.initialize().success
    manager = hardware._timed_ros_manager
    assert manager is not None
    assert manager.is_initialized()
    assert hardware._timed_cmd_manager is None
    assert hardware._arm_sdk_manager is None
    assert hardware._low_level_sdk_manager is None
    runtime.calls["init_node"].assert_not_called()
    runtime.driver.connect.assert_not_called()
    assert hardware.shutdown().success
    assert not manager.is_initialized()
    assert hardware._timed_ros_manager is None
    assert not hardware.is_connected
    runtime.driver.disconnect.assert_called_once_with()


@pytest.mark.parametrize("whitelist,enabled", [
    (None, True), (["timed"], True), ([], False), (["unknown"], False),
])
def test_ros_whitelist(runtime, no_sdk_imports, ros_manager, whitelist, enabled):
    hardware = runtime.create(ros_managers_whitelist=whitelist)
    assert hardware.initialize().success
    assert ros_manager.factory.call_count == int(enabled)
    if enabled:
        assert hardware._timed_ros_manager is ros_manager.manager
        ros_manager.manager.initialize.assert_called_once_with()
        ros_manager.manager.send_multi_commands.assert_not_called()
    else:
        assert hardware._timed_ros_manager is None
    assert hardware.shutdown().success
    assert ros_manager.manager.shutdown.call_count == int(enabled)


def test_skip_ros_does_not_skip_sdk(runtime, ros_manager, sdk_managers):
    hardware = runtime.create(skip_ros_managers=True, sdk_managers_whitelist=["low"])
    assert hardware.initialize().success
    ros_manager.factory.assert_not_called()
    assert hardware._timed_ros_manager is None
    assert hardware._low_level_sdk_manager is sdk_managers["low"].manager
    sdk_managers["timed"].factory.assert_not_called()
    sdk_managers["arm"].factory.assert_not_called()
    assert hardware.shutdown().success
    sdk_managers["low"].manager.shutdown.assert_called_once_with()


@pytest.mark.parametrize("whitelist,expected", [
    (None, {"timed", "arm", "low"}),
    (["timed"], {"timed"}),
    (["arm"], {"arm"}),
    (["low"], {"low"}),
    ([], set()),
])
def test_sdk_whitelist_preserves_legacy_timed(runtime, ros_manager, sdk_managers, whitelist, expected):
    hardware = runtime.create(sdk_managers_whitelist=whitelist)
    assert hardware.initialize().success
    assert hardware._timed_ros_manager is ros_manager.manager
    for key, (_, _, attribute) in SDK_MANAGERS.items():
        assert sdk_managers[key].factory.call_count == int(key in expected)
        assert getattr(hardware, attribute) is (sdk_managers[key].manager if key in expected else None)
    assert hardware.shutdown().success
    for key in SDK_MANAGERS:
        assert sdk_managers[key].manager.shutdown.call_count == int(key in expected)


@pytest.mark.parametrize("raises", [False, True])
def test_ros_initialization_failure_remains_optional(runtime, no_sdk_imports, ros_manager, monkeypatch, raises):
    warning = Mock()
    monkeypatch.setattr(lifecycle.logger, "warning", warning)
    if raises:
        ros_manager.manager.initialize.side_effect = RuntimeError("ROS init failed")
    else:
        ros_manager.manager.initialize.return_value = Result.fail("ROS init failed")
    hardware = runtime.create()
    assert hardware.initialize().success
    assert hardware._timed_ros_manager is None
    warning.assert_called()
    assert "ROS init failed" in str(warning.call_args)
    assert hardware.shutdown().success
    ros_manager.manager.shutdown.assert_not_called()


@pytest.mark.parametrize("key", ["timed", "arm", "low"])
@pytest.mark.parametrize("raises", [False, True])
def test_sdk_initialization_failure_not_installed(runtime, ros_manager, sdk_managers, key, raises):
    if raises:
        sdk_managers[key].manager.initialize.side_effect = RuntimeError("SDK init failed")
    else:
        sdk_managers[key].manager.initialize.return_value = Result.fail("SDK init failed")
    hardware = runtime.create(sdk_managers_whitelist=[key])
    assert hardware.initialize().success
    assert getattr(hardware, SDK_MANAGERS[key][2]) is None
    assert hardware._timed_ros_manager is ros_manager.manager
    assert hardware.shutdown().success
    sdk_managers[key].manager.shutdown.assert_not_called()


def test_initialization_order_ros_node_then_ros_manager_then_sdk(runtime, ros_manager, sdk_managers, monkeypatch):
    from adapters.hardware.leju_wheeled.services.ros_manager import torso_ros_manager, mpc_ros_manager

    events = []
    torso = Mock()
    torso.initialize.side_effect = lambda: events.append("torso") or Result.ok()
    monkeypatch.setattr(torso_ros_manager, "TorsoRosManager", Mock(return_value=torso))
    mpc = Mock()
    mpc.initialize.side_effect = lambda: events.append("mpc") or Result.ok()
    monkeypatch.setattr(mpc_ros_manager, "MpcRosManager", Mock(return_value=mpc))
    monkeypatch.setattr(lifecycle.rospy.core, "is_initialized", lambda: False)
    runtime.calls["init_node"].side_effect = lambda *a, **kw: events.append("node")
    ros_manager.manager.initialize.side_effect = lambda: events.append("ros") or Result.ok()
    sdk_managers["low"].manager.initialize.side_effect = lambda: events.append("sdk") or Result.ok()
    hardware = runtime.create(sdk_managers_whitelist=["low"])
    assert hardware.initialize().success
    assert hardware._torso_ros_manager is torso
    assert hardware._mpc_ros_manager is mpc
    assert events == ["node", "ros", "torso", "mpc", "sdk"]


def test_timed_ros_manager_requires_existing_ros_node(runtime, no_sdk_imports, monkeypatch):
    from adapters.hardware.leju_wheeled.services.ros_manager.timed_ros_manager import TimedRosManager

    monkeypatch.setattr(lifecycle.rospy.core, "is_initialized", lambda: False)
    manager = TimedRosManager()
    assert not manager.initialize().success
    assert not manager.is_initialized()
    runtime.calls["init_node"].assert_not_called()
    assert manager.shutdown().success
