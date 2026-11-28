"""人工躯干脚本回归；全部Mock，禁止真实硬件初始化。"""

import importlib
from unittest.mock import Mock

import pytest

from core.domain.result import Result

pytestmark = pytest.mark.unit
CASES = [(layer, kind) for layer in ("adapters", "skills", "nodes") for kind in ("joint", "pose")]


def load(layer, kind):
    return importlib.import_module(f"apps.test_internal.{layer}.set_torso_{kind}")


@pytest.mark.parametrize("layer,kind", CASES)
def test_empty_target_blocks_hardware_import(monkeypatch, layer, kind):
    module = load(layer, kind)
    # 人工脚本允许操作者填写目标；此处注入空值验证守卫，不依赖现场配置。
    monkeypatch.setattr(module, "TARGET_" + kind.upper(), [])
    import builtins
    original = builtins.__import__

    def guarded(name, *args, **kwargs):
        assert name not in ("adapters.hardware.factory", "orchestration.shared_hardware")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)
    assert module.main() == 1
    assert module.HARDWARE_CONFIG["sdk_managers_whitelist"] == ["low"]


@pytest.mark.parametrize("layer", ["adapters", "skills"])
@pytest.mark.parametrize("kind", ["joint", "pose"])
@pytest.mark.parametrize("success", [True, False])
def test_single_submission_and_failure_propagation(monkeypatch, layer, kind, success):
    module = load(layer, kind)
    target = [0.0] * 4  # Mock数据，非真机目标。
    monkeypatch.setattr(module, "TARGET_" + kind.upper(), target)
    hw = Mock()
    send = getattr(hw, "set_torso_" + kind)
    send.return_value = Result.ok() if success else Result.fail("refused")
    assert getattr(module, "test_set_torso_" + kind)(hw) is success
    send.assert_called_once_with(**{kind: target})
    assert len(hw.mock_calls) == 1


@pytest.mark.parametrize("kind", ["joint", "pose"])
@pytest.mark.parametrize("success", [True, False])
def test_node_script_uses_real_node_with_mock_hardware(monkeypatch, kind, success):
    module = load("nodes", kind)
    node_module = importlib.import_module("orchestration.nodes.set_torso_" + kind)
    target = [0.0] * 4
    monkeypatch.setattr(module, "TARGET_" + kind.upper(), target)
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hw = Mock()
    send = getattr(hw, "set_torso_" + kind)
    send.return_value = Result.ok() if success else Result.fail("refused")
    monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hw)
    assert getattr(module, "test_set_torso_" + kind)() is success
    send.assert_called_once_with(**{kind: target})
    assert len(hw.mock_calls) == 1


@pytest.mark.parametrize("kind", ["joint", "pose"])
def test_node_script_dry_run_cannot_report_live_success(monkeypatch, kind):
    module = load("nodes", kind)
    monkeypatch.setattr(module, "TARGET_" + kind.upper(), [0.0] * 4)
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    node_cls = Mock(side_effect=AssertionError("不应构造节点"))
    monkeypatch.setattr(module, "SetTorso" + kind.title(), node_cls)
    assert module.main() == 1
    assert getattr(module, "test_set_torso_" + kind)() is False
    node_cls.assert_not_called()
