"""九个双臂反馈/末端示例的离线测试，不执行实际SDK初始化或运动。"""

import importlib
import math
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

from apps.internal import _arm_test as common
from core.domain.result import Result

LAYERS = ("adapters", "skills", "nodes")
ACTIONS = ("get_arm_joint", "set_arm_pose", "get_arm_pose")
# 仅用于Mock，不作为真机目标或坐标系配置建议。
LEFT = [0.1, 0.2, 0.3, 30.0, -20.0, 10.0]
RIGHT = [0.1, -0.2, 0.3, -30.0, 20.0, -10.0]


def prepare(module, action, monkeypatch):
    monkeypatch.setattr(module, "init_logging", lambda: None)
    if action == "set_arm_pose":
        monkeypatch.setattr(module, "LEFT_POSE", list(LEFT))
        monkeypatch.setattr(module, "RIGHT_POSE", list(RIGHT))
    if action != "get_arm_joint":
        monkeypatch.setattr(module, "FRAME", "local")
    if action == "get_arm_pose":
        monkeypatch.setattr(module, "SOURCE", "tf")
        monkeypatch.setattr(module, "FK_FRAME", None)
    if hasattr(module, "OUTPUT_KEY"):
        monkeypatch.setattr(module, "OUTPUT_KEY", "app_arm_feedback_test")


def expected_result(action, source="tf"):
    if action == "set_arm_pose":
        return Result.ok("sent")
    if action == "get_arm_joint":
        return Result.ok(data=[30.0] * 14)
    return Result.ok(data={"left_pose": LEFT, "right_pose": RIGHT, "frame": "local", "source": source})


def expected_kwargs(action):
    if action == "get_arm_joint":
        return {}
    if action == "set_arm_pose":
        return {"left_pose": LEFT, "right_pose": RIGHT, "frame": "local"}
    return {"frame": "local", "source": "tf", "fk_frame": None}


@pytest.mark.parametrize("layer", LAYERS)
@pytest.mark.parametrize("action", ACTIONS)
@pytest.mark.parametrize("success", [True, False])
def test_layer_calls_and_output(layer, action, success, monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    module = importlib.import_module(f"apps.internal.{layer}.{action}")
    prepare(module, action, monkeypatch)
    hardware = Mock()
    getattr(hardware, action).return_value = expected_result(action) if success else Result.fail("mock failure")
    if layer == "nodes":
        node_module = importlib.import_module(f"orchestration.nodes.{action}")
        monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hardware)
        actual = getattr(module, f"test_{action}")()
    else:
        actual = getattr(module, f"test_{action}")(hardware)
    assert actual is success
    getattr(hardware, action).assert_called_once_with(**expected_kwargs(action))
    hardware.arm_reset.assert_not_called()
    hardware.set_arm_control_mode.assert_not_called()
    hardware.reset_torso_to_initial.assert_not_called()


@pytest.mark.parametrize("layer", ("adapters", "skills"))
@pytest.mark.parametrize("action", ACTIONS)
@pytest.mark.parametrize("initialized", [True, False])
def test_main_lifecycle_mocked(layer, action, initialized, monkeypatch):
    module = importlib.import_module(f"apps.internal.{layer}.{action}")
    prepare(module, action, monkeypatch)
    hardware = Mock()
    hardware.initialize.return_value = Result.ok() if initialized else Result.fail("mock init failure")
    getattr(hardware, action).return_value = expected_result(action)
    factory = Mock(return_value=hardware)
    factory_module = ModuleType("adapters.hardware.factory")
    factory_module.HardwareFactory = SimpleNamespace(create_hardware=factory)
    monkeypatch.setitem(sys.modules, "adapters.hardware.factory", factory_module)
    assert module.main() == (0 if initialized else 1)
    factory.assert_called_once_with(config=common.HARDWARE_CONFIG)
    hardware.initialize.assert_called_once_with()
    hardware.shutdown.assert_called_once_with()
    assert getattr(hardware, action).call_count == int(initialized)


@pytest.mark.parametrize("action", ACTIONS)
def test_node_main_owns_shared_lifecycle(action, monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    from orchestration import shared_hardware
    module = importlib.import_module(f"apps.internal.nodes.{action}")
    prepare(module, action, monkeypatch)
    configured, reset, tested = Mock(), Mock(), Mock(return_value=True)
    monkeypatch.setattr(shared_hardware, "set_hardware_config", configured)
    monkeypatch.setattr(shared_hardware, "reset_shared_hardware", reset)
    monkeypatch.setattr(module, f"test_{action}", tested)
    assert module.main() == 0
    configured.assert_called_once_with(common.HARDWARE_CONFIG)
    reset.assert_called_once_with()
    tested.assert_called_once_with()


@pytest.mark.parametrize("layer", LAYERS)
def test_empty_pose_target_and_invalid_fk_fail_before_hardware(layer, monkeypatch):
    setter = importlib.import_module(f"apps.internal.{layer}.set_arm_pose")
    getter = importlib.import_module(f"apps.internal.{layer}.get_arm_pose")
    monkeypatch.setattr(setter, "init_logging", lambda: None)
    monkeypatch.setattr(getter, "init_logging", lambda: None)
    monkeypatch.setattr(setter, "LEFT_POSE", [])
    monkeypatch.setattr(setter, "RIGHT_POSE", [])
    monkeypatch.setattr(getter, "SOURCE", "fk")
    monkeypatch.setattr(getter, "FK_FRAME", None)
    assert setter.main() == 1
    assert getter.main() == 1


@pytest.mark.parametrize("layer", LAYERS)
def test_fk_selection_is_forwarded(layer, monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    module = importlib.import_module(f"apps.internal.{layer}.get_arm_pose")
    prepare(module, "get_arm_pose", monkeypatch)
    monkeypatch.setattr(module, "SOURCE", "fk")
    monkeypatch.setattr(module, "FK_FRAME", "test_model_frame")
    hardware = Mock()
    hardware.get_arm_pose.return_value = expected_result("get_arm_pose", "fk")
    if layer == "nodes":
        from orchestration.nodes import get_arm_pose
        monkeypatch.setattr(get_arm_pose, "get_shared_hardware", lambda: hardware)
        assert module.test_get_arm_pose()
    else:
        assert module.test_get_arm_pose(hardware)
    hardware.get_arm_pose.assert_called_once_with(frame="local", source="fk", fk_frame="test_model_frame")


@pytest.mark.parametrize("action", ACTIONS)
def test_node_main_rejects_dry_run(action, monkeypatch):
    module = importlib.import_module(f"apps.internal.nodes.{action}")
    prepare(module, action, monkeypatch)
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    assert module.main() == 1


def test_helpers_reject_invalid_feedback_or_wrong_source():
    assert not common.check_joint_result(Result.ok(data=[math.nan] * 14), feedback=True)
    assert common.check_joint_result(Result.ok(data=[30.0] * 14), feedback=True)
    assert not common.check_pose_result(Result.ok(data=None), frame="local", source="tf")
    assert not common.check_pose_result(expected_result("get_arm_pose"), frame="world", source="tf")
    assert not common.check_pose_result(expected_result("get_arm_pose"), frame="local", source="fk")
    assert not common.validate_pose_target(LEFT, [], "local")
    assert not common.validate_query("local", "fk", None)
