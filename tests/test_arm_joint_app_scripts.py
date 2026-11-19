"""apps/internal 双臂关节示例的离线验证；所有硬件调用均使用Mock。"""

import importlib
import os
import subprocess
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

from apps.internal import _arm_test as common
from core.domain.result import Result

ROOT = Path(__file__).resolve().parents[1]
LAYERS = ("adapters", "skills", "nodes")
ACTIONS = ("set_arm_joint", "set_arm_joint_trajectory")
# 只用于Mock验证，不是建议真机目标。
JOINT = [float(index) for index in range(14)]
TIMES = [0.5, 2.0]
FRAMES = [JOINT, [-value for value in JOINT]]


@pytest.fixture(autouse=True)
def preserve_logging_and_live_mode(monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    for layer in LAYERS:
        for action in ACTIONS:
            module = importlib.import_module(f"apps.internal.{layer}.{action}")
            monkeypatch.setattr(module, "init_logging", lambda: None)


def fill_mock_params(module, action, monkeypatch):
    if action == "set_arm_joint":
        monkeypatch.setattr(module, "TARGET_JOINT", list(JOINT))
    else:
        monkeypatch.setattr(module, "TRAJECTORY_TIMES", list(TIMES))
        monkeypatch.setattr(module, "TRAJECTORY_FRAMES", [list(frame) for frame in FRAMES])


@pytest.mark.parametrize("layer", LAYERS)
@pytest.mark.parametrize("action", ACTIONS)
def test_unfilled_parameters_exit_without_hardware(layer, action, monkeypatch):
    module = importlib.import_module(f"apps.internal.{layer}.{action}")
    # 操作人员可能已填写脚本目标；测试主动置空，不能执行真实硬件入口。
    if action == "set_arm_joint":
        monkeypatch.setattr(module, "TARGET_JOINT", [])
    else:
        monkeypatch.setattr(module, "TRAJECTORY_TIMES", [])
        monkeypatch.setattr(module, "TRAJECTORY_FRAMES", [])
    assert module.main() == 1


@pytest.mark.parametrize("layer", LAYERS)
@pytest.mark.parametrize("action", ACTIONS)
@pytest.mark.parametrize("success", [True, False])
def test_each_layer_calls_requested_method_once(layer, action, success, monkeypatch):
    module = importlib.import_module(f"apps.internal.{layer}.{action}")
    fill_mock_params(module, action, monkeypatch)
    hardware = Mock()
    getattr(hardware, action).return_value = Result.ok() if success else Result.fail("mock failure")
    if layer == "nodes":
        node_module = importlib.import_module(f"orchestration.nodes.{action}")
        monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hardware)
        result = getattr(module, f"test_{action}")()
    else:
        result = getattr(module, f"test_{action}")(hardware)
    assert result is success
    expected = {"joint": JOINT} if action == "set_arm_joint" else {"times": TIMES, "q_frames": FRAMES}
    getattr(hardware, action).assert_called_once_with(**expected)
    hardware.arm_reset.assert_not_called()
    hardware.set_arm_control_mode.assert_not_called()
    hardware.reset_torso_to_initial.assert_not_called()


@pytest.mark.parametrize("layer", ("adapters", "skills"))
@pytest.mark.parametrize("action", ACTIONS)
@pytest.mark.parametrize("initialized", [True, False])
def test_main_initializes_and_closes_hardware(layer, action, initialized, monkeypatch):
    module = importlib.import_module(f"apps.internal.{layer}.{action}")
    fill_mock_params(module, action, monkeypatch)
    hardware = Mock()
    hardware.initialize.return_value = Result.ok() if initialized else Result.fail("mock init failure")
    getattr(hardware, action).return_value = Result.ok()
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
def test_node_main_sets_shared_config_and_releases_hardware(action, monkeypatch):
    from orchestration import shared_hardware
    module = importlib.import_module(f"apps.internal.nodes.{action}")
    fill_mock_params(module, action, monkeypatch)
    configured, reset = Mock(), Mock()
    monkeypatch.setattr(shared_hardware, "set_hardware_config", configured)
    monkeypatch.setattr(shared_hardware, "reset_shared_hardware", reset)
    test_call = Mock(return_value=True)
    monkeypatch.setattr(module, f"test_{action}", test_call)
    assert module.main() == 0
    configured.assert_called_once_with(common.HARDWARE_CONFIG)
    test_call.assert_called_once_with()
    reset.assert_called_once_with()


@pytest.mark.parametrize("action", ACTIONS)
def test_node_dry_run_is_not_reported_as_live_success(action, monkeypatch):
    module = importlib.import_module(f"apps.internal.nodes.{action}")
    fill_mock_params(module, action, monkeypatch)
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    assert module.main() == 1


@pytest.mark.parametrize("layer", LAYERS)
@pytest.mark.parametrize("action", ACTIONS)
def test_standalone_script_rejects_unfilled_parameters_from_any_directory(layer, action):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    script = ROOT / "apps/internal" / layer / f"{action}.py"
    code = (
        "import runpy, sys; namespace = runpy.run_path(sys.argv[1]); "
        "namespace['main'].__globals__.update(TARGET_JOINT=[], TRAJECTORY_TIMES=[], TRAJECTORY_FRAMES=[]); "
        "sys.exit(namespace['main']())"
    )
    result = subprocess.run([sys.executable, "-B", "-c", code, str(script)], cwd="/tmp", env=env,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 1, result.stdout + result.stderr
    assert "KuavoSDK" not in result.stdout + result.stderr


def test_common_helpers_do_not_accept_invalid_targets_or_false_success():
    assert not common.validate_joint_target([])
    assert not common.validate_trajectory([], [])
    assert not common.check_joint_result(Result.fail("failure"))
    assert not common.check_joint_result(Result.ok(data={"arrived": True}))
    assert common.check_joint_result(Result.ok())
