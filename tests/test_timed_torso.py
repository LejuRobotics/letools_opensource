"""躯干Timed接口/技能/节点离线回归；仅Mock与合成目标。"""

import importlib
import importlib.util
import math
from pathlib import Path
from unittest.mock import Mock, call

import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from core.domain.result import Result
from core.domain.torso import validate_timed_torso_joint, validate_timed_torso_pose
from core.interfaces.i_hardware import IHardware

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "timed_torso_mixin_test", ROOT / "adapters/hardware/leju_wheeled/mixins/timedcmd_mixin.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
pytestmark = pytest.mark.unit
TARGET = [0.1, 0.2, 30.0, -40.0]  # 合成Mock数据，不是真机目标。


@pytest.fixture(params=["joint", "pose"])
def case(request, monkeypatch):
    kind = request.param
    name = "timed_torso_" + kind
    cls = "TimedTorso" + kind.title()
    skill_module = importlib.import_module("skills.atomic.refactored_sdk." + name)
    node_module = importlib.import_module("orchestration.nodes." + name)
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    monkeypatch.setattr("time.sleep", Mock(side_effect=AssertionError("不得sleep")))
    return kind, name, cls, skill_module, node_module


def params(case, **overrides):
    kind, _, cls, module, _ = case
    return getattr(module, cls + "Params")(**dict({kind: list(TARGET), "desire_time": 2.0}, **overrides))


def hardware(case, result=None):
    hw = Mock(spec=["set_arm_control_mode", case[1]])
    hw.set_arm_control_mode.return_value = Result.ok()
    getattr(hw, case[1]).return_value = Result.ok(data={"actual_time": 3.0}) if result is None else result
    return hw


@pytest.mark.parametrize("unit", ["deg", "rad"])
@pytest.mark.parametrize("is_sync", [True, False])
def test_adapter_uses_single_multi_request_and_fixed_units(case, unit, is_sync):
    kind, name, *_ = case
    hw = MODULE.TimedCmdMixin()
    hw._angle_unit = unit
    hw._timed_ros_manager = Mock(spec=["send_multi_commands"])
    expected = Result.ok(data={"actual_time": 999.0})
    hw._timed_ros_manager.send_multi_commands.return_value = expected
    assert getattr(hw, name)(**{kind: TARGET, "desire_time": 2.0, "is_sync": is_sync}) is expected
    vector = [math.radians(v) for v in TARGET] if kind == "joint" else TARGET[:2] + [math.radians(v) for v in TARGET[2:]]
    hw._timed_ros_manager.send_multi_commands.assert_called_once_with(
        commands=[{"planner_index": 3 if kind == "joint" else 2, "desire_time": 2.0, "cmd_vec": vector}],
        is_sync=is_sync)


@pytest.mark.parametrize("field,value", [
    ("target", None), ("target", []), ("target", [0.0] * 6),
    ("target", [True] * 4), ("target", [float("nan")] * 4),
    ("target", [float("inf")] * 4), ("target", [None] * 4),
    ("desire_time", None), ("desire_time", 0), ("desire_time", -1),
    ("desire_time", True), ("desire_time", float("nan")),
    ("desire_time", float("inf")), ("desire_time", "bad"),
    ("is_sync", 1), ("is_sync", "true"), ("is_sync", None),
])
def test_invalid_inputs_fail_before_any_io(case, field, value):
    kind, name, cls, module, _ = case
    kw = {kind: list(TARGET), "desire_time": 2.0, "is_sync": True}
    kw[kind if field == "target" else field] = value
    hw = MODULE.TimedCmdMixin()
    hw._timed_ros_manager = Mock()
    assert not getattr(hw, name)(**kw).success
    assert hw._timed_ros_manager.mock_calls == []
    fake = hardware(case)
    skill = getattr(module, cls + "Skill")(fake)
    assert not skill.initialize(getattr(module, cls + "Params")(**kw)).success
    assert not skill.execute().success
    assert fake.mock_calls == []


def test_adapter_missing_manager_and_failures(case):
    kind, name, *_ = case
    hw = MODULE.TimedCmdMixin()
    for manager in (None, Mock()):
        hw._timed_ros_manager = manager
        if manager is None:
            assert not getattr(hw, name)(TARGET, 2.0).success
        else:
            expected = Result.fail("refused", error_code="SDK")
            manager.send_multi_commands.return_value = expected
            assert getattr(hw, name)(TARGET, 2.0) is expected
            manager.send_multi_commands.side_effect = RuntimeError("failed")
            assert not getattr(hw, name)(TARGET, 2.0).success


@pytest.mark.parametrize("prepare", [False, True])
@pytest.mark.parametrize("success", [False, True])
def test_skill_prepares_once_and_caches_result(case, prepare, success):
    kind, name, cls, module, _ = case
    expected = Result.ok(data={"actual_time": 999.0}) if success else Result.fail("refused")
    hw = hardware(case, expected)
    skill = getattr(module, cls + "Skill")(hw)
    assert skill.initialize(params(case, prepare_arm_control=prepare)).success
    assert not skill.is_finished()
    assert skill.execute() is expected
    assert skill.is_finished()
    assert skill.execute() is expected
    expected_calls = [call.set_arm_control_mode(2)] if prepare else []
    expected_calls += [getattr(call, name)(**{kind: TARGET, "desire_time": 2.0, "is_sync": True})]
    assert hw.mock_calls == expected_calls
    assert skill.initialize(params(case, prepare_arm_control=prepare)).success
    assert not skill.is_finished()
    skill.execute()
    assert getattr(hw, name).call_count == 2


@pytest.mark.parametrize("failure_stage", ["mode", "send"])
@pytest.mark.parametrize("raises", [False, True])
def test_skill_failure_is_terminal_without_resubmit(case, failure_stage, raises):
    _, name, cls, module, _ = case
    hw = hardware(case)
    method = hw.set_arm_control_mode if failure_stage == "mode" else getattr(hw, name)
    if raises:
        method.side_effect = RuntimeError("failure")
    else:
        method.return_value = Result.fail("failure")
    skill = getattr(module, cls + "Skill")(hw)
    assert skill.initialize(params(case)).success
    failure = skill.execute()
    assert not failure.success and skill.is_finished()
    calls = list(hw.mock_calls)
    assert skill.execute() is failure
    assert hw.mock_calls == calls
    if failure_stage == "mode":
        getattr(hw, name).assert_not_called()


def test_skill_validates_prepare_before_io(case):
    hw = hardware(case)
    skill = getattr(case[3], case[2] + "Skill")(hw)
    assert not skill.initialize(params(case, prepare_arm_control="true")).success
    assert not skill.execute().success
    assert not hw.mock_calls


@pytest.mark.parametrize("data,expected", [({"actual_time": 3.0}, 3.0), ({}, 2.0), (None, 2.0)])
def test_node_terminal_and_optional_duration_output(case, monkeypatch, data, expected):
    kind, name, cls, _, module = case
    hw = hardware(case, Result.ok(data=data))
    monkeypatch.setattr(module, "get_shared_hardware", lambda: hw)
    node = getattr(module, cls)("test", "test", "ns", {
        kind: TARGET, "desire_time": 2.0, "actual_time_output_key": "duration"})
    Blackboard.set("duration", 100)
    node.initialise()
    assert Blackboard.get("duration") is None
    assert node.update() == Status.SUCCESS
    assert Blackboard.get("duration") == expected
    calls = list(hw.mock_calls)
    assert node.update() == Status.SUCCESS
    assert hw.mock_calls == calls


@pytest.mark.parametrize("actual", [None, True, -1, float("nan"), float("inf"), "bad"])
def test_invalid_duration_does_not_leave_stale_output(case, monkeypatch, actual):
    kind, name, cls, _, module = case
    hw = hardware(case, Result.ok(data={"actual_time": actual}))
    monkeypatch.setattr(module, "get_shared_hardware", lambda: hw)
    node = getattr(module, cls)("test", "test", "ns", {
        kind: TARGET, "desire_time": 2.0, "actual_time_output_key": "duration"})
    Blackboard.set("duration", 100)
    node.initialise()
    assert node.update() == Status.FAILURE
    assert Blackboard.get("duration") is None
    assert node.update() == Status.FAILURE
    getattr(hw, name).assert_called_once()


def test_node_reads_board_at_initialise_and_dry_run_avoids_hardware(case, monkeypatch):
    import json
    kind, name, cls, _, module = case
    getter = Mock(return_value=hardware(case))
    monkeypatch.setattr(module, "get_shared_hardware", getter)
    node = getattr(module, cls)("test", "test", "ns", {
        kind + "__board_key": "target", "desire_time__board_key": "duration",
        "prepare_arm_control": "false", "is_sync": "false"})
    Blackboard.set("target", json.dumps(TARGET))
    Blackboard.set("duration", 2.0)
    node.initialise()
    assert node.update() == Status.SUCCESS
    getattr(getter.return_value, name).assert_called_once_with(**{kind: TARGET, "desire_time": 2.0, "is_sync": False})
    getter.return_value.set_arm_control_mode.assert_not_called()
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    getter.reset_mock()
    node.initialise()
    assert node.update() == Status.SUCCESS
    Blackboard.set("target", [])
    node.initialise()
    assert node.update() == Status.FAILURE
    getter.assert_not_called()


def test_optional_interface_and_node_discovery(case):
    _, name, cls, _, _ = case
    assert name not in IHardware.__abstractmethods__
    assert not getattr(IHardware, name)(object(), TARGET, 2.0).success
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index[cls] == "orchestration.nodes." + name
