"""躯干单目标接口、技能与节点离线测试；绝不初始化 SDK 或发送真机指令。"""

import importlib.util
import json
import math
from pathlib import Path
from unittest.mock import Mock, call

import pytest
from py_trees.common import Access, Status

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.torso import validate_torso_joint, validate_torso_pose
from core.interfaces.i_hardware import IHardware
from orchestration.nodes import set_torso_joint as joint_node
from orchestration.nodes import set_torso_pose as pose_node
from skills.atomic.refactored_sdk.set_torso_joint import SetTorsoJointParams, SetTorsoJointSkill
from skills.atomic.refactored_sdk.set_torso_pose import SetTorsoPoseParams, SetTorsoPoseSkill

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "torso_mixin_under_test",
    ROOT / "adapters/hardware/leju_wheeled/mixins/torso_mixin.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

# 仅用于 Mock 的合成样本，不是建议的真机目标。
JOINT = [11, -22, 33, -44]
POSE = [0.12, 0.34, 15, -26]
CASES = [
    pytest.param(
        ("joint", JOINT, validate_torso_joint, SetTorsoJointParams,
         SetTorsoJointSkill, joint_node, joint_node.SetTorsoJoint,
         "control_wheel_lower_joint"), id="joint",
    ),
    pytest.param(
        ("pose", POSE, validate_torso_pose, SetTorsoPoseParams,
         SetTorsoPoseSkill, pose_node, pose_node.SetTorsoPose,
         "control_torso_6dof"), id="pose",
    ),
]


@pytest.fixture(params=CASES)
def target(request):
    keys = ("param", "values", "validate", "params_cls", "skill_cls", "module", "node_cls", "sdk_method")
    return dict(zip(keys, request.param))


@pytest.fixture(autouse=True)
def forbid_ros_initialization_and_sleep(monkeypatch):
    """错误路径也不可连接 ROS、创建发布者或等待运动完成。"""
    forbidden = Mock(side_effect=AssertionError("offline single-target tests must not initialize ROS or wait"))
    for name in ("init_node", "Publisher", "ServiceProxy", "wait_for_service", "sleep"):
        monkeypatch.setattr(MODULE.rospy, name, forbidden)
    monkeypatch.setattr("time.sleep", forbidden)
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    yield
    forbidden.assert_not_called()


class FakeHardware(MODULE.TorsoMixin):
    def __init__(self, unit="deg"):
        self._angle_unit = unit
        self._low_level_sdk_manager = Mock(spec=["control_wheel_lower_joint", "control_torso_6dof"])
        self._low_level_sdk_manager.control_wheel_lower_joint.return_value = Result.ok()
        self._low_level_sdk_manager.control_torso_6dof.return_value = Result.ok()

    def _to_rad(self, value):
        return math.radians(value) if self._angle_unit == "deg" else value


def hardware_mock(target, result=None):
    hardware = Mock(spec=["set_torso_joint", "set_torso_pose"])
    method = getattr(hardware, "set_torso_" + target["param"])
    method.return_value = Result.ok() if result is None else result
    return hardware, method


def make_node(target, params):
    return target["node_cls"]("torso_test_" + target["param"], "torso", "test", params)


@pytest.mark.parametrize("container", [list, tuple])
def test_validation_returns_float_copy_without_changing_units(target, container):
    original = container(target["values"])
    result = target["validate"](original)
    assert isinstance(result, list)
    assert result == list(original)
    assert all(type(value) is float for value in result)
    assert result is not original
    result[0] += 1
    assert list(original) == target["values"]


INVALID_KINDS = [
    "none", "empty", "short", "long", "string", "mapping", "range",
    "true", "false", "element_none", "nan", "inf", "negative_inf", "nonnumeric", "nested",
]


def invalid_value(target, kind):
    size = len(target["values"])
    containers = {
        "none": None, "empty": [], "short": [0] * (size - 1),
        "long": [0] * (size + 1), "string": "0" * size,
        "mapping": dict.fromkeys(range(size), 0), "range": range(size),
    }
    if kind in containers:
        return containers[kind]
    bad_elements = {
        "true": True, "false": False, "element_none": None,
        "nan": math.nan, "inf": math.inf, "negative_inf": -math.inf,
        "nonnumeric": "not-a-number", "nested": [0],
    }
    values = list(target["values"])
    values[-1] = bad_elements[kind]
    return values


@pytest.mark.parametrize("kind", INVALID_KINDS)
def test_invalid_values_fail_validation_and_never_reach_sdk(target, kind):
    values = invalid_value(target, kind)
    with pytest.raises(ValueError):
        target["validate"](values)
    hardware = FakeHardware()
    result = getattr(hardware, "set_torso_" + target["param"])(values)
    assert not result.success
    assert hardware._low_level_sdk_manager.mock_calls == []


@pytest.mark.parametrize("unit", ["deg", "rad"])
@pytest.mark.parametrize("container", [list, tuple])
def test_fixed_units_and_pose_angle_reordering(target, unit, container):
    hardware = FakeHardware(unit)
    values = container(target["values"])
    result = getattr(hardware, "set_torso_" + target["param"])(values)
    assert result.success
    if target["param"] == "joint":
        expected = call.control_wheel_lower_joint(joint_traj=[float(value) for value in JOINT])
    else:
        x, z, yaw, pitch = POSE
        expected = call.control_torso_6dof(
            torso_pose=(x, 0.0, z, 0.0, math.radians(pitch), math.radians(yaw)))
    assert hardware._low_level_sdk_manager.mock_calls == [expected]
    assert list(values) == target["values"]


def test_six_component_pose_is_rejected_without_sdk_call():
    old_pose = [0.12, -0.23, 0.34, 15.0, -26.0, 37.0]  # Mock数据。
    with pytest.raises(ValueError):
        validate_torso_pose(old_pose)
    hardware = FakeHardware()
    assert not hardware.set_torso_pose(old_pose).success
    assert hardware._low_level_sdk_manager.mock_calls == []
    skill = SetTorsoPoseSkill(hardware)
    assert not skill.initialize(SetTorsoPoseParams(pose=old_pose)).success
    assert not skill.execute().success
    assert hardware._low_level_sdk_manager.mock_calls == []


def test_sdk_failure_is_preserved_without_truthiness_conversion(target):
    hardware = FakeHardware()
    failure = Result.fail("SDK rejected target", error_code="SDK_ERROR")
    getattr(hardware._low_level_sdk_manager, target["sdk_method"]).return_value = failure
    assert getattr(hardware, "set_torso_" + target["param"])(target["values"]) is failure


@pytest.mark.parametrize("missing_attribute", [False, True])
def test_missing_manager_fails_instead_of_raising(target, missing_attribute):
    hardware = FakeHardware()
    if missing_attribute:
        del hardware._low_level_sdk_manager
    else:
        hardware._low_level_sdk_manager = None
    assert not getattr(hardware, "set_torso_" + target["param"])(target["values"]).success


@pytest.mark.parametrize("success", [True, False])
def test_skill_sends_once_caches_result_and_reinitializes(target, success):
    expected = Result.ok() if success else Result.fail("rejected")
    hardware, method = hardware_mock(target, expected)
    skill = target["skill_cls"](hardware)
    values = list(target["values"])
    params = target["params_cls"](**{target["param"]: values})
    assert skill.initialize(params).success
    assert not skill.is_finished()
    values[0] += 1  # 初始化必须保存副本，而不是引用调用方的可变目标。
    assert skill.execute() is expected
    assert skill.execute() is expected
    assert skill.is_finished()
    method.assert_called_once_with(**{target["param"]: target["values"]})
    assert hardware.mock_calls == [
        getattr(call, "set_torso_" + target["param"])(**{target["param"]: target["values"]})
    ]

    assert skill.initialize(params).success
    assert not skill.is_finished()
    assert skill.execute() is expected
    assert skill.execute() is expected
    assert method.call_count == 2
    assert method.call_args == call(**{target["param"]: values})


def test_skill_exception_is_cached_and_reinitialization_allows_retry(target):
    hardware, method = hardware_mock(target)
    method.side_effect = RuntimeError("offline SDK exception")
    skill = target["skill_cls"](hardware)
    params = target["params_cls"](**{target["param"]: target["values"]})
    assert skill.initialize(params).success
    failure = skill.execute()
    assert not failure.success
    assert "offline SDK exception" in failure.message
    assert skill.execute() is failure
    assert skill.is_finished()
    method.assert_called_once()
    method.side_effect = None
    assert skill.initialize(params).success
    assert not skill.is_finished()
    assert skill.execute().success
    assert method.call_count == 2


def test_default_skill_params_are_empty_and_independent(target):
    first, second = target["params_cls"](), target["params_cls"]()
    assert getattr(first, target["param"]) == []
    assert getattr(second, target["param"]) == []
    getattr(first, target["param"]).append(1)
    assert getattr(second, target["param"]) == []
    hardware, method = hardware_mock(target)
    skill = target["skill_cls"](hardware)
    assert not skill.initialize(second).success
    assert not skill.execute().success
    assert not skill.execute().success
    method.assert_not_called()


@pytest.mark.parametrize("kind", INVALID_KINDS)
def test_invalid_skill_reinitialization_discards_previous_target(target, kind):
    hardware, method = hardware_mock(target)
    skill = target["skill_cls"](hardware)
    assert skill.initialize(target["params_cls"](**{target["param"]: target["values"]})).success
    invalid = target["params_cls"](**{target["param"]: invalid_value(target, kind)})
    assert not skill.initialize(invalid).success
    assert not skill.execute().success
    method.assert_not_called()


def test_wrong_skill_params_do_not_send(target):
    hardware, method = hardware_mock(target)
    skill = target["skill_cls"](hardware)
    assert not skill.initialize(SkillParams()).success
    assert not skill.execute().success
    method.assert_not_called()


@pytest.mark.parametrize("success", [True, False])
@pytest.mark.parametrize("json_input", [False, True])
def test_node_parses_input_caches_terminal_status_and_reinitializes(monkeypatch, target, success, json_input):
    result = Result.ok() if success else Result.fail("SDK rejected target")
    hardware, method = hardware_mock(target, result)
    monkeypatch.setattr(target["module"], "get_shared_hardware", lambda: hardware)
    values = json.dumps(target["values"]) if json_input else list(target["values"])
    node = make_node(target, {target["param"]: values})
    node.initialise()
    method.assert_not_called()
    expected = Status.SUCCESS if success else Status.FAILURE
    assert node.update() == expected
    assert node.update() == expected
    method.assert_called_once_with(**{target["param"]: target["values"]})
    node.initialise()
    assert node.update() == expected
    assert node.update() == expected
    assert method.call_count == 2


def test_node_hardware_exception_has_cached_failure(monkeypatch, target):
    hardware, method = hardware_mock(target)
    method.side_effect = RuntimeError("offline SDK exception")
    monkeypatch.setattr(target["module"], "get_shared_hardware", lambda: hardware)
    node = make_node(target, {target["param"]: target["values"]})
    node.initialise()
    assert node.update() == Status.FAILURE
    assert node.update() == Status.FAILURE
    method.assert_called_once()


@pytest.mark.parametrize("json_input", [False, True])
def test_dry_run_validates_without_getting_hardware(monkeypatch, target, json_input):
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    get_hardware = Mock(side_effect=AssertionError("dry-run must not obtain hardware"))
    monkeypatch.setattr(target["module"], "get_shared_hardware", get_hardware)
    values = json.dumps(target["values"]) if json_input else list(target["values"])
    node = make_node(target, {target["param"]: values})
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert node.update() == Status.SUCCESS
    get_hardware.assert_not_called()


@pytest.mark.parametrize("dry_run", [False, True])
@pytest.mark.parametrize("kind", INVALID_KINDS + ["missing", "bad_json"])
def test_invalid_node_never_gets_hardware(monkeypatch, target, dry_run, kind):
    if dry_run:
        monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    get_hardware = Mock(side_effect=AssertionError("invalid input must not obtain hardware"))
    monkeypatch.setattr(target["module"], "get_shared_hardware", get_hardware)
    if kind == "missing":
        params = {}
    elif kind == "bad_json":
        params = {target["param"]: "[broken JSON"}
    else:
        params = {target["param"]: invalid_value(target, kind)}
    node = make_node(target, params)
    node.initialise()
    assert node.update() == Status.FAILURE
    assert node.update() == Status.FAILURE
    get_hardware.assert_not_called()


@pytest.mark.parametrize("dry_run", [False, True])
def test_node_resolves_runtime_board_key_again_after_reinitialization(monkeypatch, target, dry_run):
    hardware, method = hardware_mock(target)
    get_hardware = Mock(return_value=hardware)
    if dry_run:
        monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    monkeypatch.setattr(target["module"], "get_shared_hardware", get_hardware)
    key = "test_set_torso_runtime_" + target["param"]
    node = make_node(target, {target["param"]: [], target["param"] + "__board_key": key})
    node.global_blackboard.register_key(key, access=Access.WRITE)
    try:
        for index in range(2):
            values = list(target["values"])
            values[0] += index
            node.global_blackboard.set(key, json.dumps(values) if index else values)
            node.initialise()
            assert node.update() == Status.SUCCESS
            assert node.update() == Status.SUCCESS
            if not dry_run:
                assert method.call_count == index + 1
                assert method.call_args == call(**{target["param"]: values})
        if dry_run:
            get_hardware.assert_not_called()
            method.assert_not_called()
    finally:
        node.global_blackboard.unset(key)


def test_missing_board_key_fails_without_hardware(monkeypatch, target):
    get_hardware = Mock(side_effect=AssertionError("missing board value must not obtain hardware"))
    monkeypatch.setattr(target["module"], "get_shared_hardware", get_hardware)
    key = "test_set_torso_missing_" + target["param"]
    node = make_node(target, {target["param"] + "__board_key": key})
    node.initialise()
    assert node.update() == Status.FAILURE
    assert node.update() == Status.FAILURE
    get_hardware.assert_not_called()


def test_interfaces_are_optional_and_default_to_failure(target):
    method_name = "set_torso_" + target["param"]
    assert method_name not in IHardware.__abstractmethods__
    assert not getattr(IHardware, method_name)(object(), target["values"]).success


def test_nodes_are_discovered_by_factory():
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory

    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index["SetTorsoJoint"] == "orchestration.nodes.set_torso_joint"
    assert factory._node_index["SetTorsoPose"] == "orchestration.nodes.set_torso_pose"
