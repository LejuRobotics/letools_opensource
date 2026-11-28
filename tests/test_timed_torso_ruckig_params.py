"""躯干规划限制离线回归，合成数值仅给Mock，不配置真机。"""

import builtins
import importlib
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from core.domain.ruckig_params import validate_torso_ruckig_params
from core.domain.result import Result
from core.interfaces.i_hardware import IHardware
from skills.atomic.refactored_sdk.timed_torso_ruckig_params import TimedTorsoRuckigParams, TimedTorsoRuckigParamsSkill
from orchestration.nodes import timed_torso_ruckig_params as node_module
from adapters.hardware.leju_wheeled.services.ros_manager import timed_ros_manager as ros_module

SPEC = importlib.util.spec_from_file_location("torso_ruckig_test", Path(__file__).resolve().parents[1] / "adapters/hardware/leju_wheeled/mixins/torso_mixin.py")
MIXIN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MIXIN)
pytestmark = pytest.mark.unit


def query(mode="pose"):
    return dict(mode=mode, velocity_max=[1., 2., 3., 4.], acceleration_max=[2., 3., 4., 5.], jerk_max=[3., 4., 5., 6.])


@pytest.mark.parametrize("mode", ["pose", "joint"])
@pytest.mark.parametrize("unit", ["deg", "rad"])
@pytest.mark.parametrize("custom_min", [True, False])
def test_adapter_converts_every_derivative_and_sets_defaults(mode, unit, custom_min):
    values = query(mode)
    if custom_min:
        values.update(velocity_min=[-2., -3., -4., -5.], acceleration_min=[-3., -4., -5., -6.])
    expected = validate_torso_ruckig_params(**values)
    for key in ("velocity_max", "acceleration_max", "jerk_max", "velocity_min", "acceleration_min"):
        expected[key] = [math.radians(v) if mode == "joint" or i >= 2 else v for i, v in enumerate(expected[key])]
    hw = MIXIN.TorsoMixin()
    hw._angle_unit = unit
    hw._timed_ros_manager = Mock()
    result = Result.ok()
    hw._timed_ros_manager.timed_torso_ruckig_params.return_value = result
    assert hw.timed_torso_ruckig_params(**values) is result
    hw._timed_ros_manager.timed_torso_ruckig_params.assert_called_once_with(**expected)


@pytest.mark.parametrize("field,bad", [
    ("mode", ""), ("mode", "arm"), ("mode", None), ("mode", 2),
    ("is_sync", 1), ("is_sync", "true"),
    ("velocity_max", []), ("velocity_max", [1.] * 3), ("velocity_max", [True] * 4),
    ("velocity_max", [0.] * 4), ("acceleration_max", [-1.] * 4), ("jerk_max", [math.nan] * 4),
    ("jerk_max", [math.inf] * 4), ("jerk_max", [None] * 4),
    ("velocity_min", []), ("velocity_min", [0.] * 4), ("velocity_min", [1.] * 4),
    ("acceleration_min", [False] * 4), ("acceleration_min", [-math.inf] * 4),
])
def test_invalid_values_rejected_before_io(field, bad):
    values = dict(query(), **{field: bad})
    with pytest.raises(ValueError):
        validate_torso_ruckig_params(**values)
    hw = MIXIN.TorsoMixin()
    hw._timed_ros_manager = Mock()
    assert not hw.timed_torso_ruckig_params(**values).success
    assert not hw._timed_ros_manager.mock_calls
    skill = TimedTorsoRuckigParamsSkill(Mock())
    assert not skill.initialize(TimedTorsoRuckigParams(**values)).success
    assert not skill.execute().success
    assert not skill.hardware.mock_calls


@pytest.mark.parametrize("mode", ["pose", "joint"])
@pytest.mark.parametrize("is_sync", [True, False])
def test_ros_request_fields_and_one_call(monkeypatch, mode, is_sync):
    monkeypatch.setattr(ros_module.rospy.core, "is_initialized", lambda: True)
    wait = Mock()
    client = Mock(return_value=SimpleNamespace(result=True, message="accepted"))
    factory = Mock(return_value=client)
    monkeypatch.setattr(ros_module.rospy, "wait_for_service", wait)
    monkeypatch.setattr(ros_module.rospy, "ServiceProxy", factory)
    manager = ros_module.TimedRosManager()
    assert manager.initialize().success
    values = dict(query(mode), is_sync=is_sync)
    result = manager.timed_torso_ruckig_params(**values)
    assert result.success and result.message == "accepted" and result.data is None
    wait.assert_called_once_with("/mobile_manipulator_set_ruckig_planner_params", timeout=5.0)
    client.assert_called_once()
    req = client.call_args.args[0]
    expected = validate_torso_ruckig_params(**values)
    assert req.planner_index == (2 if mode == "pose" else 3)
    for field, value in expected.items():
        if field != "mode": assert getattr(req, field) == value


@pytest.mark.parametrize("fault", ["uninitialized", "invalid", "refused", "exception"])
def test_ros_failures_do_not_retry(monkeypatch, fault):
    monkeypatch.setattr(ros_module.rospy.core, "is_initialized", lambda: True)
    wait = Mock()
    client = Mock(return_value=SimpleNamespace(result=False, message="refused"))
    factory = Mock(return_value=client)
    monkeypatch.setattr(ros_module.rospy, "wait_for_service", wait)
    monkeypatch.setattr(ros_module.rospy, "ServiceProxy", factory)
    manager = ros_module.TimedRosManager()
    if fault != "uninitialized": manager.initialize()
    values = query()
    if fault == "invalid": values["velocity_max"] = []
    if fault == "exception": client.side_effect = ros_module.rospy.ServiceException("lost")
    assert not manager.timed_torso_ruckig_params(**values).success
    assert client.call_count == (1 if fault in ("refused", "exception") else 0)
    assert wait.call_count == client.call_count


@pytest.mark.parametrize("fault", ["success", "failure", "exception"])
def test_skill_result_cache_and_parameter_snapshot(fault):
    hw = Mock(spec=["timed_torso_ruckig_params"])
    result = Result.ok() if fault == "success" else Result.fail("refused")
    hw.timed_torso_ruckig_params.return_value = result
    if fault == "exception": hw.timed_torso_ruckig_params.side_effect = RuntimeError("lost")
    values = query()
    expected = validate_torso_ruckig_params(**values)
    skill = TimedTorsoRuckigParamsSkill(hw)
    assert skill.initialize(TimedTorsoRuckigParams(**values)).success
    values["velocity_max"][0] = 999.
    outcome = skill.execute()
    assert outcome.success == (fault == "success")
    assert skill.execute() is outcome
    assert skill.is_finished()
    hw.timed_torso_ruckig_params.assert_called_once_with(**expected)
    assert skill.initialize(TimedTorsoRuckigParams(**query())).success
    assert not skill.is_finished()


def test_node_runtime_board_cache_and_dryrun(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hw = Mock()
    hw.timed_torso_ruckig_params.return_value = Result.ok()
    getter = Mock(return_value=hw)
    monkeypatch.setattr(node_module, "get_shared_hardware", getter)
    values = query()
    node = node_module.TimedTorsoRuckigParams("config", "config", "ns", {
        **values, "velocity_max__board_key": "v", "is_sync": "false"})
    Blackboard.set("v", "[2,3,4,5]")
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert node.update() == Status.SUCCESS
    hw.timed_torso_ruckig_params.assert_called_once()
    assert hw.timed_torso_ruckig_params.call_args.kwargs["velocity_min"] == [-2., -3., -4., -5.]
    assert hw.timed_torso_ruckig_params.call_args.kwargs["is_sync"] is False
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    getter.reset_mock()
    node.initialise()
    assert node.update() == Status.SUCCESS
    Blackboard.set("v", [])
    node.initialise()
    assert node.update() == Status.FAILURE
    getter.assert_not_called()


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
def test_manual_scripts_invalid_values_block_hardware_import(layer, monkeypatch):
    module = importlib.import_module(f"apps.test_internal.{layer}.timed_torso_ruckig_params")
    monkeypatch.setattr(module, "VELOCITY_MAX", [])
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        assert name not in ("adapters.hardware.factory", "orchestration.shared_hardware")
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", guarded)
    assert module.main() == 1
    assert module.CONFIG["sdk_managers_whitelist"] == []
    assert module.CONFIG["ros_managers_whitelist"] == ["timed"]


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
@pytest.mark.parametrize("success", [True, False])
def test_manual_dispatch_uses_only_configuration_method(layer, success, monkeypatch):
    module = importlib.import_module(f"apps.test_internal.{layer}.timed_torso_ruckig_params")
    for key, value in query().items():
        monkeypatch.setattr(module, key.upper(), value)
    monkeypatch.setattr(module, "VELOCITY_MIN", None)
    monkeypatch.setattr(module, "ACCELERATION_MIN", None)
    monkeypatch.setattr(module, "IS_SYNC", True)
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hw = Mock(spec=["timed_torso_ruckig_params"])
    hw.timed_torso_ruckig_params.return_value = Result.ok() if success else Result.fail("refused")
    if layer == "nodes":
        monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hw)
        result = module.test_timed_torso_ruckig_params()
    else:
        result = module.test_timed_torso_ruckig_params(hw)
    assert result is success
    hw.timed_torso_ruckig_params.assert_called_once_with(**validate_torso_ruckig_params(**query()))
    assert len(hw.mock_calls) == 1


def test_optional_contract_and_node_discovery():
    assert "timed_torso_ruckig_params" not in IHardware.__abstractmethods__
    assert not IHardware.timed_torso_ruckig_params(object(), **query()).success
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(Path(__file__).resolve().parents[1] / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index["TimedTorsoRuckigParams"] == "orchestration.nodes.timed_torso_ruckig_params"
