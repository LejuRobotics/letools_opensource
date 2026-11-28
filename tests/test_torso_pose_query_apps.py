"""新躯干查询/复位人工脚本，注入Mock，不执行真实初始化。"""

import builtins
import importlib
from unittest.mock import Mock

import pytest

from core.domain.result import Result

pytestmark = pytest.mark.unit


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
def test_reset_requires_explicit_confirmation_before_importing_hardware(layer, monkeypatch):
    module = importlib.import_module("apps.test_internal." + layer + ".reset_torso")
    monkeypatch.setattr(module, "CONFIRM_RESET", False)
    original = builtins.__import__

    def guarded(name, *args, **kwargs):
        assert name not in ("adapters.hardware.factory", "orchestration.shared_hardware")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)
    assert module.main() == 1
    if layer == "nodes":
        assert not module.test_reset_torso()
    else:
        hw = Mock()
        assert not module.test_reset_torso(hw)
        assert not hw.mock_calls


@pytest.mark.parametrize("name", ["get_torso_pose", "get_torso_pose_initial", "reset_torso"])
@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
@pytest.mark.parametrize("success", [True, False])
def test_dispatch_and_failure_with_mock_only(name, layer, success, monkeypatch):
    module = importlib.import_module(f"apps.test_internal.{layer}.{name}")
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    if name == "reset_torso":
        monkeypatch.setattr(module, "CONFIRM_RESET", True)
    hw = Mock(spec=[name])
    method = getattr(hw, name)
    method.return_value = Result.ok(data={"pose": [0.1, 0.2, 0, 0]}) if success else Result.fail("refused")
    if layer == "nodes":
        node_module = importlib.import_module("orchestration.nodes." + name)
        monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hw)
        result = getattr(module, "test_" + name)()
    else:
        result = getattr(module, "test_" + name)(hw)
    assert result is success
    method.assert_called_once_with()
    assert module.CONFIG["sdk_managers_whitelist"] == (["low"] if name == "get_torso_pose" else [])
    assert module.CONFIG["ros_managers_whitelist"] == ([] if name == "get_torso_pose" else ["torso"])


@pytest.mark.parametrize("name", ["get_torso_pose", "get_torso_pose_initial", "reset_torso"])
def test_nodes_reject_dryrun_as_live_read_or_reset(name, monkeypatch):
    module = importlib.import_module("apps.test_internal.nodes." + name)
    if name == "reset_torso":
        monkeypatch.setattr(module, "CONFIRM_RESET", True)
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    assert module.main() == 1
    assert not getattr(module, "test_" + name)()
