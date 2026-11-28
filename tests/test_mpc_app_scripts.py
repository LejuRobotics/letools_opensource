"""MPC人工脚本离线回归；人工目标通过monkeypatch注入。"""

import builtins
import importlib
from unittest.mock import Mock

import pytest
from core.domain.result import Result
from core.domain.mpc import MPC_VALUES

pytestmark = pytest.mark.unit
KINDS = [("control_mode", "mode"), ("control_flow", "flow"), ("frame", "frame")]


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
@pytest.mark.parametrize("suffix,kind", KINDS)
def test_invalid_setter_blocks_initialization(layer, suffix, kind, monkeypatch):
    module = importlib.import_module(f"apps.test_internal.{layer}.set_mpc_{suffix}")
    monkeypatch.setattr(module, "TARGET_" + kind.upper(), None)
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        assert name not in ("adapters.hardware.factory", "orchestration.shared_hardware")
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", guarded)
    assert module.main() == 1
    assert module.HARDWARE_CONFIG["sdk_managers_whitelist"] == []
    assert module.HARDWARE_CONFIG["ros_managers_whitelist"] == ["mpc"]


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
@pytest.mark.parametrize("suffix,kind", KINDS)
@pytest.mark.parametrize("action", ["set", "get"])
@pytest.mark.parametrize("success", [True, False])
def test_dispatch_only_selected_service(layer, suffix, kind, action, success, monkeypatch):
    name = f"{action}_mpc_{suffix}"
    module = importlib.import_module(f"apps.test_internal.{layer}.{name}")
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    if action == "set": monkeypatch.setattr(module, "TARGET_" + kind.upper(), 1)
    hw = Mock(spec=[name])
    getattr(hw, name).return_value = Result.ok(data=None if action == "set" else {kind: 1, "name": MPC_VALUES[kind][1]}) if success else Result.fail("refused")
    if layer == "nodes":
        node_module = importlib.import_module("orchestration.nodes." + name)
        monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hw)
        result = getattr(module, "test_" + name)()
    else:
        result = getattr(module, "test_" + name)(hw)
    assert result is success
    getattr(hw, name).assert_called_once_with(**{kind: 1} if action == "set" else {})
    assert len(hw.mock_calls) == 1


@pytest.mark.parametrize("suffix,kind", KINDS)
@pytest.mark.parametrize("action", ["set", "get"])
def test_node_dryrun_not_live_success(suffix, kind, action, monkeypatch):
    module = importlib.import_module(f"apps.test_internal.nodes.{action}_mpc_{suffix}")
    if action == "set": monkeypatch.setattr(module, "TARGET_" + kind.upper(), 1)
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    assert module.main() == 1
    assert not getattr(module, f"test_{action}_mpc_{suffix}")()
