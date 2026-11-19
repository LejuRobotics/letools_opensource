"""模式查询全层Mock测试；不初始化SDK，不下发模式切换或运动指令。"""

import importlib
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock
import sys

import pytest
from py_trees.common import Status

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from apps.internal import _arm_test
from skills.atomic.refactored_sdk.get_arm_control_mode import GetArmControlModeParams, GetArmControlModeSkill
from orchestration.nodes import get_arm_control_mode as node_module
from orchestration.nodes import set_arm_control_mode as setter_node
from skills.atomic.refactored_sdk.set_arm_control_mode import SetArmControlModeParams, SetArmControlModeSkill

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("mode_arm_mixin_test", ROOT / "adapters/hardware/leju_wheeled/mixins/arm_mixin.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
DATA = {"mode": 2, "name": "ExternalControl"}


@pytest.mark.parametrize("mode", [0, 1, 2])
@pytest.mark.parametrize("keyword", [False, True])
def test_setter_keeps_positional_and_keyword_calls(mode, keyword):
    hardware = MODULE.ArmMixin()
    hardware._low_level_sdk_manager = Mock()
    expected = Result.ok("SDK mode set")
    hardware._low_level_sdk_manager.set_arm_control_mode.return_value = expected
    result = (hardware.set_arm_control_mode(control_mode=mode) if keyword
              else hardware.set_arm_control_mode(mode))
    assert result is expected
    hardware._low_level_sdk_manager.set_arm_control_mode.assert_called_once_with(control_mode=mode)


@pytest.mark.parametrize("mode", [None, -1, 3, True, 2.0, "2"])
def test_setter_rejects_invalid_input_before_manager(mode):
    hardware = MODULE.ArmMixin()
    hardware._low_level_sdk_manager = Mock()
    assert not hardware.set_arm_control_mode(mode).success
    hardware._low_level_sdk_manager.set_arm_control_mode.assert_not_called()


def test_setter_requires_manager_and_preserves_sdk_failure():
    hardware = MODULE.ArmMixin()
    hardware._low_level_sdk_manager = None
    assert not hardware.set_arm_control_mode(2).success
    hardware._low_level_sdk_manager = Mock()
    failure = Result.fail("SDK refused", error_code="MODE_ERROR")
    hardware._low_level_sdk_manager.set_arm_control_mode.return_value = failure
    assert hardware.set_arm_control_mode(2) is failure
    hardware._low_level_sdk_manager.set_arm_control_mode.side_effect = RuntimeError("SDK error")
    assert not hardware.set_arm_control_mode(2).success


def test_hardware_resolves_setter_from_arm_mixin():
    from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
    from adapters.hardware.leju_wheeled.mixins.arm_mixin import ArmMixin
    from adapters.hardware.leju_wheeled.mixins.mode_service_mixin import ModeServiceMixin
    assert LejuWheeledArmHardware.set_arm_control_mode is ArmMixin.set_arm_control_mode
    assert 'set_arm_control_mode' not in ModeServiceMixin.__dict__
    assert 'set_arm_control_mode' not in IHardware.__abstractmethods__


def test_mixin_preserves_result_and_requires_manager():
    hardware = MODULE.ArmMixin()
    hardware._low_level_sdk_manager = Mock()
    for result in (Result.ok(data=DATA), Result.fail("SDK unavailable")):
        hardware._low_level_sdk_manager.get_arm_control_mode.return_value = result
        assert hardware.get_arm_control_mode() is result
    hardware._low_level_sdk_manager = None
    assert not hardware.get_arm_control_mode().success
    assert "get_arm_control_mode" not in IHardware.__abstractmethods__


@pytest.mark.parametrize("success", [True, False])
def test_skill_calls_once_and_reinitializes(success):
    hardware = Mock()
    result = Result.ok(data=DATA) if success else Result.fail("query failed")
    hardware.get_arm_control_mode.return_value = result
    skill = GetArmControlModeSkill(hardware)
    assert skill.initialize(GetArmControlModeParams()).success
    assert skill.execute() is result
    assert skill.execute() is result
    assert skill.is_finished()
    hardware.get_arm_control_mode.assert_called_once_with()
    assert skill.initialize(GetArmControlModeParams()).success
    assert not skill.is_finished()
    assert not skill.initialize(SkillParams()).success
    assert not skill.execute().success


@pytest.mark.parametrize("success", [True, False])
def test_node_reads_once_and_writes_only_success(monkeypatch, success):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hardware = Mock()
    hardware.get_arm_control_mode.return_value = Result.ok(data=DATA) if success else Result.fail("no mode")
    monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hardware)
    node = node_module.GetArmControlMode("mode", "mode", "ns", {"output_key": "mode_test_output"})
    node.initialise()
    expected = Status.SUCCESS if success else Status.FAILURE
    assert node.update() == expected
    assert node.update() == expected
    assert node.global_blackboard.get("mode_test_output") == (DATA if success else None)
    hardware.get_arm_control_mode.assert_called_once_with()


def test_node_dry_run_and_invalid_output_do_not_use_hardware(monkeypatch):
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    unexpected = Mock(side_effect=AssertionError("must not access hardware"))
    monkeypatch.setattr(node_module, "get_shared_hardware", unexpected)
    node = node_module.GetArmControlMode("dry", "mode", "ns", {})
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert node.global_blackboard.get("arm_control_mode") is None
    for key in (None, "", " "):
        bad = node_module.GetArmControlMode("bad", "mode", "ns", {"output_key": key})
        bad.initialise()
        assert bad.update() == Status.FAILURE
    unexpected.assert_not_called()


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
@pytest.mark.parametrize("success", [True, False])
def test_example_calls_query_without_setter(layer, success, monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    module = importlib.import_module(f"apps.internal.{layer}.get_arm_control_mode")
    hardware = Mock()
    hardware.get_arm_control_mode.return_value = Result.ok(data=DATA) if success else Result.fail("query failed")
    if layer == "nodes":
        monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hardware)
        actual = module.test_get_arm_control_mode()
    else:
        actual = module.test_get_arm_control_mode(hardware)
    assert actual is success
    hardware.get_arm_control_mode.assert_called_once_with()
    hardware.set_arm_control_mode.assert_not_called()
    hardware.arm_reset.assert_not_called()


@pytest.mark.parametrize("layer", ["adapters", "skills"])
def test_example_lifecycle_is_mocked(layer, monkeypatch):
    module = importlib.import_module(f"apps.internal.{layer}.get_arm_control_mode")
    monkeypatch.setattr(module, "init_logging", lambda: None)
    hardware = Mock()
    hardware.initialize.return_value = Result.ok()
    hardware.get_arm_control_mode.return_value = Result.ok(data=DATA)
    factory = Mock(return_value=hardware)
    fake = ModuleType("adapters.hardware.factory")
    fake.HardwareFactory = SimpleNamespace(create_hardware=factory)
    monkeypatch.setitem(sys.modules, "adapters.hardware.factory", fake)
    assert module.main() == 0
    factory.assert_called_once_with(config=_arm_test.HARDWARE_CONFIG)
    hardware.shutdown.assert_called_once_with()


def test_node_example_lifecycle_and_discovery(monkeypatch):
    from orchestration import shared_hardware
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    module = importlib.import_module("apps.internal.nodes.get_arm_control_mode")
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    monkeypatch.setattr(module, "init_logging", lambda: None)
    configured, reset, tested = Mock(), Mock(), Mock(return_value=True)
    monkeypatch.setattr(shared_hardware, "set_hardware_config", configured)
    monkeypatch.setattr(shared_hardware, "reset_shared_hardware", reset)
    monkeypatch.setattr(module, "test_get_arm_control_mode", tested)
    assert module.main() == 0
    configured.assert_called_once_with(_arm_test.HARDWARE_CONFIG)
    reset.assert_called_once_with()
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    assert module.main() == 1
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index["GetArmControlMode"] == "orchestration.nodes.get_arm_control_mode"


@pytest.mark.parametrize("mode", [0, 1, 2])
@pytest.mark.parametrize("success", [True, False])
def test_setter_skill_single_call_and_preserved_result(mode, success):
    hardware = Mock()
    expected = Result.ok() if success else Result.fail("mode rejected")
    hardware.set_arm_control_mode.return_value = expected
    skill = SetArmControlModeSkill(hardware)
    assert skill.initialize(SetArmControlModeParams(control_mode=mode)).success
    assert skill.execute() is expected
    assert skill.execute() is expected
    assert skill.is_finished()
    hardware.set_arm_control_mode.assert_called_once_with(control_mode=mode)
    assert skill.initialize(SetArmControlModeParams(control_mode=mode)).success
    assert not skill.is_finished()


@pytest.mark.parametrize("mode", [None, -1, 3, True, 2.0, "2"])
def test_setter_skill_rejects_invalid_mode(mode):
    hardware = Mock()
    skill = SetArmControlModeSkill(hardware)
    assert not skill.initialize(SetArmControlModeParams(control_mode=mode)).success
    assert not skill.execute().success
    hardware.set_arm_control_mode.assert_not_called()


@pytest.mark.parametrize("success", [True, False])
def test_setter_node_executes_once(monkeypatch, success):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hardware = Mock()
    hardware.set_arm_control_mode.return_value = Result.ok() if success else Result.fail("rejected")
    monkeypatch.setattr(setter_node, "get_shared_hardware", lambda: hardware)
    node = setter_node.SetArmControlMode("set_mode", "mode", "ns", {"control_mode": 2})
    node.initialise()
    expected = Status.SUCCESS if success else Status.FAILURE
    assert node.update() == expected
    assert node.update() == expected
    hardware.set_arm_control_mode.assert_called_once_with(control_mode=2)


def test_setter_node_runtime_board_and_dry_run(monkeypatch):
    from py_trees.common import Access
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    unexpected = Mock(side_effect=AssertionError("dry-run cannot access hardware"))
    monkeypatch.setattr(setter_node, "get_shared_hardware", unexpected)
    node = setter_node.SetArmControlMode("board_mode", "mode", "ns", {"control_mode__board_key": "mode_input_test"})
    node.global_blackboard.register_key("mode_input_test", access=Access.WRITE)
    node.global_blackboard.set("mode_input_test", 0)
    node.initialise()
    assert node.update() == Status.SUCCESS
    invalid = setter_node.SetArmControlMode("invalid", "mode", "ns", {})
    invalid.initialise()
    assert invalid.update() == Status.FAILURE
    unexpected.assert_not_called()
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index["SetArmControlMode"] == "orchestration.nodes.set_arm_control_mode"


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
@pytest.mark.parametrize("success", [True, False])
def test_setter_examples_only_request_configured_mode(layer, success, monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    module = importlib.import_module(f"apps.internal.{layer}.set_arm_control_mode")
    monkeypatch.setattr(module, "CONTROL_MODE", 2)
    hardware = Mock()
    hardware.set_arm_control_mode.return_value = Result.ok() if success else Result.fail("rejected")
    if layer == "nodes":
        monkeypatch.setattr(setter_node, "get_shared_hardware", lambda: hardware)
        actual = module.test_set_arm_control_mode()
    else:
        actual = module.test_set_arm_control_mode(hardware)
    assert actual is success
    hardware.set_arm_control_mode.assert_called_once_with(control_mode=2)
    hardware.arm_reset.assert_not_called()
    hardware.set_mpc_mode.assert_not_called()


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
def test_unfilled_setter_example_never_initializes_hardware(layer, monkeypatch):
    module = importlib.import_module(f"apps.internal.{layer}.set_arm_control_mode")
    monkeypatch.setattr(module, "init_logging", lambda: None)
    monkeypatch.setattr(module, "CONTROL_MODE", None)
    assert module.main() == 1


@pytest.mark.parametrize("layer", ["adapters", "skills"])
@pytest.mark.parametrize("initialized", [True, False])
def test_setter_example_main_mocked_lifecycle(layer, initialized, monkeypatch):
    module = importlib.import_module(f"apps.internal.{layer}.set_arm_control_mode")
    monkeypatch.setattr(module, "init_logging", lambda: None)
    monkeypatch.setattr(module, "CONTROL_MODE", 2)
    hardware = Mock()
    hardware.initialize.return_value = Result.ok() if initialized else Result.fail("init failure")
    hardware.set_arm_control_mode.return_value = Result.ok()
    fake = ModuleType("adapters.hardware.factory")
    fake.HardwareFactory = SimpleNamespace(create_hardware=Mock(return_value=hardware))
    monkeypatch.setitem(sys.modules, "adapters.hardware.factory", fake)
    assert module.main() == (0 if initialized else 1)
    hardware.shutdown.assert_called_once()
    assert hardware.set_arm_control_mode.call_count == int(initialized)


def test_setter_example_node_main_mocked_lifecycle(monkeypatch):
    from orchestration import shared_hardware
    module = importlib.import_module("apps.internal.nodes.set_arm_control_mode")
    monkeypatch.setattr(module, "init_logging", lambda: None)
    monkeypatch.setattr(module, "CONTROL_MODE", 2)
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    configured, reset, tested = Mock(), Mock(), Mock(return_value=True)
    monkeypatch.setattr(shared_hardware, "set_hardware_config", configured)
    monkeypatch.setattr(shared_hardware, "reset_shared_hardware", reset)
    monkeypatch.setattr(module, "test_set_arm_control_mode", tested)
    assert module.main() == 0
    configured.assert_called_once_with(_arm_test.HARDWARE_CONFIG)
    reset.assert_called_once()
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    assert module.main() == 1


def test_mode_result_validation():
    for value, name in ((0, "ArmFixed"), (1, "AutoSwing"), (2, "ExternalControl")):
        assert _arm_test.check_mode_result(Result.ok(data={"mode": value, "name": name}))
    for data in (None, {}, {"mode": True, "name": "AutoSwing"}, {"mode": 2, "name": "wrong"}):
        assert not _arm_test.check_mode_result(Result.ok(data=data))
    assert not _arm_test.check_mode_result(Result.fail("offline"))
