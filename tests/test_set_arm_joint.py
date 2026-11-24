"""新双臂关节接口、技能、节点的离线测试；不初始化SDK或发真机指令。"""

import importlib.util
import math
from pathlib import Path
from unittest.mock import Mock

import pytest
from py_trees.common import Access, Status

from core.domain.joint_state import validate_arm_joint
from core.domain.result import Result
from core.interfaces.i_hardware import IHardware
from skills.atomic.refactored_sdk.set_arm_joint import SetArmJointParams, SetArmJointSkill
from orchestration.nodes import set_arm_joint as joints_node

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "arm_sdk_mixin_under_test", ROOT / "adapters/hardware/leju_wheeled/mixins/sdk_control_mixin.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
ARM_SPEC = importlib.util.spec_from_file_location(
    "arm_mixin_under_test", ROOT / "adapters/hardware/leju_wheeled/mixins/arm_mixin.py")
ARM_MODULE = importlib.util.module_from_spec(ARM_SPEC)
ARM_SPEC.loader.exec_module(ARM_MODULE)

# 仅用于Mock的样本，不是建议真机目标。
JOINT = [float(index) for index in range(14)]


class FakeHardware(ARM_MODULE.ArmMixin, MODULE.SDKControlMixin):
    def __init__(self, unit="deg"):
        self._angle_unit = unit
        self._low_level_sdk_manager = Mock()
        self._arm_sdk_manager = Mock()
        self._low_level_sdk_manager.control_arm_joint_positions.return_value = Result.ok()

    def _to_rad(self, value):
        return math.radians(value) if self._angle_unit == "deg" else value


@pytest.mark.parametrize("unit", ["deg", "rad"])
def test_new_target_fixed_degrees_and_reuses_existing_manager(unit):
    hardware = FakeHardware(unit)
    assert hardware.set_arm_joint(JOINT).success
    hardware._low_level_sdk_manager.control_arm_joint_positions.assert_called_once_with(
        joint_positions_rad=[math.radians(value) for value in JOINT])


@pytest.mark.parametrize("unit", ["deg", "rad"])
def test_old_target_still_uses_configured_unit(unit):
    hardware = FakeHardware(unit)
    assert hardware.send_arm_joint_positions_sdk(JOINT).success
    expected = [math.radians(value) for value in JOINT] if unit == "deg" else JOINT
    hardware._low_level_sdk_manager.control_arm_joint_positions.assert_called_once_with(joint_positions_rad=expected)


@pytest.mark.parametrize("method,kwargs,manager_method", [
    ("set_arm_joint", {"joint": JOINT}, "control_arm_joint_positions"),
])
def test_failure_result_is_not_treated_as_truthy_success(method, kwargs, manager_method):
    hardware = FakeHardware()
    failure = Result.fail("SDK failure", error_code="SDK_ERROR")
    getattr(hardware._low_level_sdk_manager, manager_method).return_value = failure
    assert getattr(hardware, method)(**kwargs) is failure
    hardware._low_level_sdk_manager = None
    assert not getattr(hardware, method)(**kwargs).success


@pytest.mark.parametrize("joint", [None, [], [0.0] * 13, [0.0] * 15, [math.nan] * 14, [math.inf] * 14, [None] * 14])
def test_invalid_joint_fail_before_sending(joint):
    hardware = FakeHardware()
    assert not hardware.set_arm_joint(joint).success
    hardware._low_level_sdk_manager.control_arm_joint_positions.assert_not_called()
    with pytest.raises(ValueError):
        validate_arm_joint(joint)


@pytest.mark.parametrize("skill_class,params,method", [
    (SetArmJointSkill, SetArmJointParams(joint=JOINT), "set_arm_joint"),
])
@pytest.mark.parametrize("success", [True, False])
def test_skill_runs_once_and_preserves_result(skill_class, params, method, success):
    hardware = Mock()
    expected = Result.ok() if success else Result.fail("failed")
    getattr(hardware, method).return_value = expected
    skill = skill_class(hardware)
    assert skill.initialize(params).success
    assert not skill.is_finished()
    assert skill.execute() is expected
    assert skill.execute() is expected
    assert skill.is_finished()
    getattr(hardware, method).assert_called_once()
    assert skill.initialize(params).success
    assert not skill.is_finished()


@pytest.mark.parametrize("skill_class,params,method", [
    (SetArmJointSkill, SetArmJointParams(), "set_arm_joint"),
])
def test_empty_skill_params_have_no_motion_defaults(skill_class, params, method):
    hardware = Mock()
    skill = skill_class(hardware)
    assert not skill.initialize(params).success
    assert not skill.execute().success
    getattr(hardware, method).assert_not_called()


@pytest.mark.parametrize("module,node_class,params,method", [
    (joints_node, joints_node.SetArmJoint, {"joint": JOINT}, "set_arm_joint"),
])
@pytest.mark.parametrize("success", [True, False])
def test_nodes_return_terminal_status_and_send_once(monkeypatch, module, node_class, params, method, success):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hardware = Mock()
    getattr(hardware, method).return_value = Result.ok() if success else Result.fail("failed")
    monkeypatch.setattr(module, "get_shared_hardware", lambda: hardware)
    node = node_class("arm_test", "arm", "ns", params)
    node.initialise()
    expected = Status.SUCCESS if success else Status.FAILURE
    assert node.update() == expected
    assert node.update() == expected
    getattr(hardware, method).assert_called_once_with(**params)


@pytest.mark.parametrize("module,node_class,params", [
    (joints_node, joints_node.SetArmJoint, {"joint": JOINT}),
])
def test_dry_run_checks_params_without_hardware(monkeypatch, module, node_class, params):
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    unexpected = Mock(side_effect=AssertionError("dry-run should not initialize hardware"))
    monkeypatch.setattr(module, "get_shared_hardware", unexpected)
    node = node_class("arm_dry", "arm", "ns", params)
    node.initialise()
    assert node.update() == Status.SUCCESS
    invalid = node_class("invalid", "arm", "ns", {})
    invalid.initialise()
    assert invalid.update() == Status.FAILURE
    unexpected.assert_not_called()


def test_single_joint_node_reads_renamed_runtime_board_key(monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hardware = Mock()
    hardware.set_arm_joint.return_value = Result.ok()
    monkeypatch.setattr(joints_node, "get_shared_hardware", lambda: hardware)
    node = joints_node.SetArmJoint("joint_board", "arm", "ns", {"joint__board_key": "arm_joint_input_test"})
    node.global_blackboard.register_key("arm_joint_input_test", access=Access.WRITE)
    node.global_blackboard.set("arm_joint_input_test", JOINT)
    node.initialise()
    assert node.update() == Status.SUCCESS
    hardware.set_arm_joint.assert_called_once_with(joint=JOINT)


def test_new_interfaces_are_optional_and_nodes_are_discovered():
    assert "set_arm_joint" not in IHardware.__abstractmethods__
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index["SetArmJoint"] == "orchestration.nodes.set_arm_joint"
