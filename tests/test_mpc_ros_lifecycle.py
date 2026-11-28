"""MPC ROS 生命周期与缓存兼容性；全部硬件和 ROS 通信均被 Mock 截断。"""

import builtins
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from adapters.hardware.leju_wheeled import hardware as hardware_module
from adapters.hardware.leju_wheeled.mixins import MpcMixin, lifecycle_mixin as lifecycle
from adapters.hardware.leju_wheeled.services.ros_manager import mpc_ros_manager as module
from core.domain.result import Result

pytestmark = pytest.mark.unit


@pytest.fixture
def runtime(monkeypatch):
    driver = Mock()
    monkeypatch.setattr(hardware_module, "LejuEndEffector", Mock(return_value=driver))
    monkeypatch.setattr(lifecycle.rospy.core, "is_initialized", Mock(return_value=True))
    operations = {}
    for name in ("init_node", "wait_for_service", "ServiceProxy", "Publisher", "Subscriber", "sleep"):
        operations[name] = Mock(side_effect=AssertionError("unexpected ROS call: " + name))
        monkeypatch.setattr(lifecycle.rospy, name, operations[name])
    monkeypatch.setattr(lifecycle.os, "makedirs", Mock())
    original_import = builtins.__import__
    imports = []

    def guarded_import(name, *args, **kwargs):
        if "sdk_manager" in name or name.startswith("kuavo_humanoid_sdk"):
            imports.append(name)
            raise AssertionError("SDK import forbidden: " + name)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)

    def create(**options):
        config = {
            "skip_camera": True, "skip_end_effector": True,
            "skip_state_manager": True, "skip_force_publishers": True,
            "skip_sdk_managers": True,
        }
        config.update(options)
        return hardware_module.LejuWheeledArmHardware(config)

    yield SimpleNamespace(create=create, driver=driver)
    assert imports == []  # Optional initialization must not swallow forbidden imports.
    for operation in operations.values():
        operation.assert_not_called()
    driver.connect.assert_not_called()


@pytest.mark.parametrize("options,enabled", [
    ({}, {"timed", "torso", "mpc"}),
    ({"ros_managers_whitelist": None}, {"timed", "torso", "mpc"}),
    ({"ros_managers_whitelist": ["mpc"]}, {"mpc"}),
    ({"ros_managers_whitelist": ["timed", "torso"]}, {"timed", "torso"}),
    ({"ros_managers_whitelist": []}, set()),
    ({"ros_managers_whitelist": ["unknown"]}, set()),
    ({"skip_ros_managers": True, "ros_managers_whitelist": ["mpc"]}, set()),
    ({"skip_sdk_managers": False, "sdk_managers_whitelist": []}, {"timed", "torso", "mpc"}),
])
def test_independent_ros_whitelist_and_shutdown(runtime, options, enabled):
    hardware = runtime.create(**options)
    assert hardware._mpc_ros_manager is None
    assert hardware.initialize().success
    managers = []
    for key in ("timed", "torso", "mpc"):
        manager = getattr(hardware, "_" + key + "_ros_manager")
        assert (manager is not None) == (key in enabled)
        if manager is not None:
            assert manager.is_initialized()
            managers.append(manager)
    assert hardware._timed_cmd_manager is None
    assert hardware._arm_sdk_manager is None
    assert hardware._low_level_sdk_manager is None
    assert hardware.shutdown().success
    assert all(not manager.is_initialized() for manager in managers)
    for key in ("timed", "torso", "mpc"):
        assert getattr(hardware, "_" + key + "_ros_manager") is None
    runtime.driver.disconnect.assert_called_once_with()


@pytest.mark.parametrize("raises", [False, True])
def test_mpc_initialization_failure_is_optional(runtime, monkeypatch, raises):
    manager = Mock()
    if raises:
        manager.initialize.side_effect = RuntimeError("MPC init failed")
    else:
        manager.initialize.return_value = Result.fail("MPC init failed")
    factory = Mock(return_value=manager)
    monkeypatch.setattr(module, "MpcRosManager", factory)
    hardware = runtime.create()
    assert hardware.initialize().success
    assert hardware._mpc_ros_manager is None
    assert hardware._timed_ros_manager.is_initialized()
    assert hardware._torso_ros_manager.is_initialized()
    factory.assert_called_once_with()
    manager.initialize.assert_called_once_with()
    assert hardware.shutdown().success
    manager.shutdown.assert_not_called()


def test_mpc_requires_existing_ros_node_without_io(runtime, monkeypatch):
    monkeypatch.setattr(lifecycle.rospy.core, "is_initialized", lambda: False)
    manager = module.MpcRosManager()
    assert not manager.initialize().success
    assert not manager.is_initialized()
    assert manager.shutdown().success


def test_mpc_shutdown_called_once_and_reference_cleared(runtime, monkeypatch):
    manager = Mock()
    manager.initialize.return_value = Result.ok()
    monkeypatch.setattr(module, "MpcRosManager", Mock(return_value=manager))
    hardware = runtime.create(ros_managers_whitelist=["mpc"])
    assert hardware.initialize().success
    assert hardware.shutdown().success
    assert hardware.shutdown().success
    manager.shutdown.assert_called_once_with()
    assert hardware._mpc_ros_manager is None


def test_new_getter_is_separate_from_cached_feedback(runtime):
    hardware = runtime.create()
    assert hardware_module.LejuWheeledArmHardware.get_mpc_control_mode is MpcMixin.get_mpc_control_mode
    assert hardware.get_mpc_control_mode_cached() is None
    hardware.state_manager = Mock()
    hardware.state_manager.get_state.return_value = 0
    assert hardware.get_mpc_control_mode_cached() == 0
    hardware.state_manager.get_state.assert_called_once_with("mpc_control_mode")
    hardware.state_manager.get_state.reset_mock()
    assert not hardware.get_mpc_control_mode().success
    hardware.state_manager.get_state.assert_not_called()
    hardware._mpc_ros_manager = Mock()
    result = Result.ok(data={"mode": 1, "name": "ArmOnly"})
    hardware._mpc_ros_manager.get_mpc_control_mode.return_value = result
    assert hardware.get_mpc_control_mode() is result
    hardware._mpc_ros_manager.get_mpc_control_mode.assert_called_once_with()
    hardware.state_manager.get_state.assert_not_called()
