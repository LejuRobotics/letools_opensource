"""躯干反馈/同步轨迹人工脚本回归；只使用Mock，不执行真机初始化。

所有运动参数均monkeypatch注入合成数据，不约束操作者填写的脚本配置。
"""

import builtins
import importlib
from unittest.mock import Mock

import pytest

from core.domain.result import Result

pytestmark = pytest.mark.unit
LAYERS = ("adapters", "skills", "nodes")
OPERATIONS = ("get_torso_joint", "set_torso_joint_traj")
# 仅供Mock；不是推荐真机目标或安全时长。
FRAMES = [[1.0, 2.0, 3.0, 4.0]]
DURATION = 0.01


def load(layer, operation):
    return importlib.import_module(f"apps.test_internal.{layer}.{operation}")


def configure(monkeypatch, module):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    if hasattr(module, "JOINT_TRAJECTORY"):
        monkeypatch.setattr(module, "JOINT_TRAJECTORY", [list(frame) for frame in FRAMES])
        monkeypatch.setattr(module, "TOTAL_TIME", DURATION)
    if hasattr(module, "OUTPUT_KEY"):
        monkeypatch.setattr(module, "OUTPUT_KEY", "test_torso_feedback_traj_apps")


def invoke(monkeypatch, layer, operation, hardware):
    module = load(layer, operation)
    configure(monkeypatch, module)
    if layer == "nodes":
        node_module = importlib.import_module("orchestration.nodes." + operation)
        monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hardware)
        return getattr(module, "test_" + operation)()
    return getattr(module, "test_" + operation)(hardware)


@pytest.mark.parametrize("layer", LAYERS)
@pytest.mark.parametrize("operation", OPERATIONS)
@pytest.mark.parametrize("success", [True, False])
def test_one_call_and_failure_propagation(monkeypatch, layer, operation, success):
    hw = Mock()
    call = getattr(hw, operation)
    data = list(FRAMES[0]) if operation == "get_torso_joint" else None
    call.return_value = Result.ok(data=data) if success else Result.fail("mock refused")
    assert invoke(monkeypatch, layer, operation, hw) is success
    if operation == "get_torso_joint":
        call.assert_called_once_with()
    else:
        call.assert_called_once_with(joint_traj=FRAMES, total_time=DURATION)
    # 禁止附加模式切换、停止、到位轮询或第二次发送。
    assert len(hw.mock_calls) == 1


@pytest.mark.parametrize("layer", LAYERS)
@pytest.mark.parametrize("data", [None, [], [1, 2, 3], [1, 2, 3, 4, 5], [1, 2, 3, float("nan")],
                                  [1, 2, 3, float("inf")], [1, 2, 3, True], "1234"])
def test_getter_rejects_bad_feedback(monkeypatch, layer, data):
    hw = Mock()
    hw.get_torso_joint.return_value = Result.ok(data=data)
    assert invoke(monkeypatch, layer, "get_torso_joint", hw) is False


@pytest.mark.parametrize("data", [(1, 2, 3, 4), {"joint": [1, 2, 3, 4]}])
def test_feedback_result_contract_requires_list(data):
    from apps.test_internal._torso_test import check_joint_result
    assert check_joint_result(Result.ok(data=data)) is False


@pytest.mark.parametrize("layer", ["adapters", "skills"])
def test_trajectory_rejects_unexpected_result_data(monkeypatch, layer):
    hw = Mock()
    hw.set_torso_joint_traj.return_value = Result.ok(data={"arrived": True})
    assert invoke(monkeypatch, layer, "set_torso_joint_traj", hw) is False


@pytest.mark.parametrize("layer", LAYERS)
@pytest.mark.parametrize("frames,duration", [
    ([], DURATION), (None, DURATION), ([1, 2, 3, 4], DURATION), ([[1, 2, 3]], DURATION),
    ([[1, 2, 3, True]], DURATION), ([[1, 2, 3, float("nan")]], DURATION),
    (FRAMES, None), (FRAMES, True), (FRAMES, float("inf")), (FRAMES, float("nan")),
    (FRAMES, 0.009), (FRAMES, 0.0), (FRAMES, -1.0),
])
def test_invalid_trajectory_blocks_initialization_and_submission(monkeypatch, layer, frames, duration):
    module = load(layer, "set_torso_joint_traj")
    configure(monkeypatch, module)
    monkeypatch.setattr(module, "JOINT_TRAJECTORY", frames)
    monkeypatch.setattr(module, "TOTAL_TIME", duration)
    original = builtins.__import__

    def guarded(name, *args, **kwargs):
        assert name not in ("adapters.hardware.factory", "orchestration.shared_hardware")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)
    assert module.main() == 1
    hw = Mock()
    if layer == "nodes":
        node = Mock(side_effect=AssertionError("invalid input must not construct node"))
        monkeypatch.setattr(module, "SetTorsoJointTraj", node)
        assert module.test_set_torso_joint_traj() is False
        node.assert_not_called()
    else:
        assert module.test_set_torso_joint_traj(hw) is False
        assert hw.mock_calls == []


@pytest.mark.parametrize("operation", OPERATIONS)
@pytest.mark.parametrize("value", ["1", "true", "YES"])
def test_nodes_reject_dry_run(monkeypatch, operation, value):
    module = load("nodes", operation)
    configure(monkeypatch, module)
    monkeypatch.setenv("STUDIO_DRY_RUN", value)
    name = "GetTorsoJoint" if operation == "get_torso_joint" else "SetTorsoJointTraj"
    node = Mock(side_effect=AssertionError("dry-run must not construct node"))
    monkeypatch.setattr(module, name, node)
    assert module.main() == 1
    assert getattr(module, "test_" + operation)() is False
    node.assert_not_called()


@pytest.mark.parametrize("key", ["", "  ", None, 123])
def test_getter_node_rejects_invalid_output_key(monkeypatch, key):
    module = load("nodes", "get_torso_joint")
    configure(monkeypatch, module)
    monkeypatch.setattr(module, "OUTPUT_KEY", key)
    node = Mock(side_effect=AssertionError("invalid key must not construct node"))
    monkeypatch.setattr(module, "GetTorsoJoint", node)
    assert module.main() == 1
    assert module.test_get_torso_joint() is False
    node.assert_not_called()


@pytest.mark.parametrize("layer", ["adapters", "skills"])
@pytest.mark.parametrize("operation", OPERATIONS)
@pytest.mark.parametrize("outcome", ["ok", "init_fail", "init_exception", "call_fail", "call_exception"])
def test_main_hardware_lifecycle_is_mocked(monkeypatch, layer, operation, outcome):
    module = load(layer, operation)
    configure(monkeypatch, module)
    factory = importlib.import_module("adapters.hardware.factory")
    hw = Mock()
    hw.initialize.return_value = Result.fail("mock init failed") if outcome == "init_fail" else Result.ok()
    if outcome == "init_exception":
        hw.initialize.side_effect = RuntimeError("mock init exception")
    data = list(FRAMES[0]) if operation == "get_torso_joint" else None
    call = getattr(hw, operation)
    call.return_value = Result.fail("mock call failed") if outcome == "call_fail" else Result.ok(data=data)
    if outcome == "call_exception":
        call.side_effect = RuntimeError("mock call exception")
    create = Mock(return_value=hw)
    monkeypatch.setattr(factory.HardwareFactory, "create_hardware", create)
    assert module.main() == (0 if outcome == "ok" else 1)
    create.assert_called_once_with(config=module.HARDWARE_CONFIG)
    assert module.HARDWARE_CONFIG["sdk_managers_whitelist"] == ["low"]
    hw.initialize.assert_called_once_with()
    hw.shutdown.assert_called_once_with()
    if outcome in ("init_fail", "init_exception"):
        call.assert_not_called()
    else:
        assert call.call_count == 1
    assert {entry[0] for entry in hw.mock_calls} <= {"initialize", "shutdown", operation}


@pytest.mark.parametrize("operation", OPERATIONS)
@pytest.mark.parametrize("outcome", ["ok", "failure", "exception", "config_exception"])
def test_node_main_always_resets_mock_shared_hardware(monkeypatch, operation, outcome):
    module = load("nodes", operation)
    configure(monkeypatch, module)
    shared = importlib.import_module("orchestration.shared_hardware")
    setup = Mock()
    if outcome == "config_exception":
        setup.side_effect = RuntimeError("mock config exception")
    reset = Mock()
    run = Mock(return_value=outcome == "ok")
    if outcome == "exception":
        run.side_effect = RuntimeError("mock node exception")
    monkeypatch.setattr(shared, "set_hardware_config", setup)
    monkeypatch.setattr(shared, "reset_shared_hardware", reset)
    monkeypatch.setattr(shared, "get_shared_hardware", Mock(side_effect=AssertionError("must not initialize hardware")))
    monkeypatch.setattr(module, "test_" + operation, run)
    assert module.main() == (0 if outcome == "ok" else 1)
    setup.assert_called_once_with(module.HARDWARE_CONFIG)
    assert module.HARDWARE_CONFIG["sdk_managers_whitelist"] == ["low"]
    reset.assert_called_once_with()
