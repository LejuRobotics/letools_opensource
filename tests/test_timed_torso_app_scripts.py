"""人工脚本离线守卫；所有物理目标均为monkeypatch注入的合成Mock输入。"""

import importlib
import sys
from types import SimpleNamespace
from unittest.mock import Mock, call

import pytest
from py_trees.blackboard import Blackboard

from apps.test_internal import _arm_test, _timed_test
from core.domain.result import Result

LAYERS = ("adapters", "skills", "nodes")
KINDS = ("joint", "pose")
TARGET = [1.0, 2.0, 3.0, 4.0]  # 仅Mock校验数据，不是真机推荐值。


@pytest.fixture(params=[(layer, kind) for layer in LAYERS for kind in KINDS])
def script(request, monkeypatch):
    layer, kind = request.param
    module = importlib.import_module(f"apps.test_internal.{layer}.timed_torso_{kind}")
    monkeypatch.setattr(module, kind.upper(), list(TARGET))
    monkeypatch.setattr(module, "DESIRE_TIME", 2.5)
    monkeypatch.setattr(module, "IS_SYNC", True)
    if layer != "adapters":
        monkeypatch.setattr(module, "PREPARE_ARM_CONTROL", True)
    if layer == "nodes":
        monkeypatch.setattr(module, "ACTUAL_TIME_OUTPUT_KEY", f"test_torso_{kind}_duration")
    monkeypatch.setattr(module, "init_logging", lambda: None)
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)

    hardware = Mock()
    hardware.initialize.return_value = Result.ok()
    hardware.set_arm_control_mode.return_value = Result.ok()
    getattr(hardware, f"timed_torso_{kind}").return_value = Result.ok(data={"actual_time": 3.5})
    factory = Mock(return_value=hardware)
    monkeypatch.setitem(sys.modules, "adapters.hardware.factory", SimpleNamespace(
        HardwareFactory=SimpleNamespace(create_hardware=factory)))
    shared = importlib.import_module("orchestration.shared_hardware")
    configure, reset, getter = Mock(), Mock(), Mock(return_value=hardware)
    monkeypatch.setattr(shared, "set_hardware_config", configure)
    monkeypatch.setattr(shared, "reset_shared_hardware", reset)
    monkeypatch.setattr(shared, "get_shared_hardware", getter)
    if layer == "nodes":
        node_module = importlib.import_module(f"orchestration.nodes.timed_torso_{kind}")
        monkeypatch.setattr(node_module, "get_shared_hardware", getter)
    yield SimpleNamespace(module=module, layer=layer, kind=kind, hardware=hardware,
                          factory=factory, configure=configure, reset=reset, getter=getter)
    if layer == "nodes":
        Blackboard.unset(f"test_torso_{kind}_duration")


def assert_no_hardware(script):
    script.factory.assert_not_called()
    script.configure.assert_not_called()
    script.getter.assert_not_called()
    script.reset.assert_not_called()
    assert script.hardware.mock_calls == []


@pytest.mark.parametrize("field,value", [
    ("target", []), ("target", None), ("target", [1, 2, 3]),
    ("target", [1, 2, 3, float("nan")]), ("target", [True, 2, 3, 4]),
    ("DESIRE_TIME", None), ("DESIRE_TIME", 0), ("DESIRE_TIME", float("inf")),
    ("DESIRE_TIME", True), ("IS_SYNC", 1),
])
def test_invalid_inputs_stop_before_hardware(script, monkeypatch, field, value):
    monkeypatch.setattr(script.module, script.kind.upper() if field == "target" else field, value)
    assert script.module.main() == 1
    assert_no_hardware(script)


def test_invalid_prepare_policy_stops_before_hardware(script, monkeypatch):
    if script.layer == "adapters":
        return
    monkeypatch.setattr(script.module, "PREPARE_ARM_CONTROL", "true")
    assert script.module.main() == 1
    assert_no_hardware(script)


@pytest.mark.parametrize("dry_run", ["1", "true", "YES"])
def test_node_rejects_dry_run_before_shared_hardware(script, monkeypatch, dry_run):
    if script.layer != "nodes":
        return
    monkeypatch.setenv("STUDIO_DRY_RUN", dry_run)
    assert script.module.main() == 1
    assert_no_hardware(script)


@pytest.mark.parametrize("key", ["", "  ", 1, False])
def test_invalid_node_output_key_stops_before_hardware(script, monkeypatch, key):
    if script.layer != "nodes":
        return
    monkeypatch.setattr(script.module, "ACTUAL_TIME_OUTPUT_KEY", key)
    assert script.module.main() == 1
    assert_no_hardware(script)


@pytest.mark.parametrize("prepare", [True, False])
def test_success_dispatch_and_cleanup(script, monkeypatch, prepare):
    if script.layer != "adapters":
        monkeypatch.setattr(script.module, "PREPARE_ARM_CONTROL", prepare)
    monkeypatch.setattr(script.module, "IS_SYNC", False)
    assert script.module.main() == 0
    expected = [] if script.layer == "nodes" else [call.initialize()]
    if script.layer != "adapters" and prepare:
        expected.append(call.set_arm_control_mode(2))
    expected.append(getattr(call, f"timed_torso_{script.kind}")(**{
        script.kind: TARGET, "desire_time": 2.5, "is_sync": False,
    }))
    if script.layer != "nodes":
        expected.append(call.shutdown())
    assert script.hardware.mock_calls == expected  # 不读反馈、不设置focus、不复位。
    if script.layer == "nodes":
        script.configure.assert_called_once_with(script.module.MULTI_CONFIG)
        script.getter.assert_called_once_with()
        script.reset.assert_called_once_with()
        assert Blackboard.get(script.module.ACTUAL_TIME_OUTPUT_KEY) == 3.5
    else:
        script.factory.assert_called_once_with(config=script.module.MULTI_CONFIG)
    assert script.module.MULTI_CONFIG["sdk_managers_whitelist"] == (
        [] if script.layer == "adapters" else ["low"])


@pytest.mark.parametrize("raises", [False, True])
def test_mode_failure_never_submits_and_cleans_up(script, raises):
    if script.layer == "adapters":
        return
    if raises:
        script.hardware.set_arm_control_mode.side_effect = RuntimeError("mode unavailable")
    else:
        script.hardware.set_arm_control_mode.return_value = Result.fail("mode unavailable")
    assert script.module.main() == 1
    getattr(script.hardware, f"timed_torso_{script.kind}").assert_not_called()
    script.hardware.set_arm_control_mode.assert_called_once_with(2)
    if script.layer == "nodes":
        script.reset.assert_called_once_with()
    else:
        script.hardware.shutdown.assert_called_once_with()


@pytest.mark.parametrize("raises", [False, True])
def test_hardware_initialization_failure_cleans_up(script, raises):
    if script.layer == "nodes":
        script.getter.side_effect = RuntimeError("shared hardware unavailable")
    elif raises:
        script.hardware.initialize.side_effect = RuntimeError("initialize unavailable")
    else:
        script.hardware.initialize.return_value = Result.fail("initialize unavailable")
    assert script.module.main() == 1
    getattr(script.hardware, f"timed_torso_{script.kind}").assert_not_called()
    if script.layer == "nodes":
        script.reset.assert_called_once_with()
    else:
        script.hardware.shutdown.assert_called_once_with()


@pytest.mark.parametrize("outcome", ["failure", "exception", "bad_time"])
def test_target_failure_or_invalid_result_cleans_up(script, outcome):
    send = getattr(script.hardware, f"timed_torso_{script.kind}")
    if outcome == "exception":
        send.side_effect = RuntimeError("service unavailable")
    elif outcome == "failure":
        send.return_value = Result.fail("service unavailable")
    else:
        send.return_value = Result.ok(data={"actual_time": float("nan")})
    assert script.module.main() == 1
    send.assert_called_once()
    if script.layer == "nodes":
        script.reset.assert_called_once_with()
    else:
        script.hardware.shutdown.assert_called_once_with()


def test_node_output_can_be_disabled(script, monkeypatch):
    if script.layer != "nodes":
        return
    monkeypatch.setattr(script.module, "ACTUAL_TIME_OUTPUT_KEY", None)
    assert script.module.main() == 0
    script.reset.assert_called_once_with()


@pytest.mark.parametrize("data", [None, {}, [], {"actual_time": None},
    {"actual_time": True}, {"actual_time": False}, {"actual_time": -1},
    {"actual_time": float("nan")}, {"actual_time": float("inf")}, {"actual_time": "invalid"}])
def test_multi_result_rejects_malformed_time(data):
    assert not _timed_test.check_multi_result(Result.ok(data=data))


@pytest.mark.parametrize("actual_time", [0, 3.5, "3.5"])
def test_multi_result_accepts_planned_time(actual_time):
    assert _timed_test.check_multi_result(Result.ok(data={"actual_time": actual_time}))
    assert not _timed_test.check_multi_result(Result.fail("failed", data={"actual_time": actual_time}))


def test_sdk_trajectory_result_contract_remains_separate():
    assert _arm_test.check_timed_result(Result.ok())
    assert not _arm_test.check_timed_result(Result.ok(data={"actual_time": 3.5}))
    assert not _arm_test.check_timed_result(Result.fail("failed"))
    assert not hasattr(_arm_test, "check_multi_joint_result")
    assert not hasattr(_arm_test, "validate_multi_joint_input")


@pytest.mark.parametrize("layer", LAYERS)
@pytest.mark.parametrize("kind", KINDS)
def test_existing_arm_scripts_use_timed_helpers_and_validate_before_hardware(layer, kind, monkeypatch):
    module = importlib.import_module(f"apps.test_internal.{layer}.timed_arm_{kind}")
    left, right = ("LEFT_ARM_JOINT", "RIGHT_ARM_JOINT") if kind == "joint" else ("LEFT_POSE", "RIGHT_POSE")
    monkeypatch.setattr(module, left, None)
    monkeypatch.setattr(module, right, None)
    monkeypatch.setattr(module, "DESIRE_TIME", None)
    monkeypatch.setattr(module, "init_logging", lambda: None)
    factory = Mock(side_effect=AssertionError("invalid input must not create hardware"))
    monkeypatch.setitem(sys.modules, "adapters.hardware.factory", SimpleNamespace(
        HardwareFactory=SimpleNamespace(create_hardware=factory)))
    shared = importlib.import_module("orchestration.shared_hardware")
    configure = Mock(side_effect=AssertionError("invalid input must not configure shared hardware"))
    monkeypatch.setattr(shared, "set_hardware_config", configure)
    assert module.main() == 1
    factory.assert_not_called()
    configure.assert_not_called()
    assert module.HARDWARE_CONFIG is _timed_test.HARDWARE_CONFIG
    if layer != "nodes":
        assert module.check_multi_result is _timed_test.check_multi_result
