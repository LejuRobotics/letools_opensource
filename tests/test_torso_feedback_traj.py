"""躯干反馈/轨迹离线测试；真实SDK对象不初始化，仅使用Mock。"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from core.domain.result import Result
from core.domain.torso import validate_torso_joint_traj
from core.interfaces.i_hardware import IHardware
from skills.atomic.refactored_sdk.get_torso_joint import GetTorsoJointParams, GetTorsoJointSkill
from skills.atomic.refactored_sdk.set_torso_joint_traj import SetTorsoJointTrajParams, SetTorsoJointTrajSkill
from orchestration.nodes import get_torso_joint as getter_node, set_torso_joint_traj as traj_node
from adapters.hardware.leju_wheeled.services.sdk_manager.low_level_sdk_manager import LowLevelSDKManager

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("torso_feedback_test", ROOT / "adapters/hardware/leju_wheeled/mixins/torso_mixin.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
pytestmark = pytest.mark.unit
FRAMES = [[1.0, 2.0, 3.0, 4.0], [4.0, 3.0, 2.0, 1.0]]  # 仅Mock数据。


@pytest.fixture
def manager():
    value = LowLevelSDKManager()
    value._initialized = True
    value.robot_sdk = Mock()
    value.robot_sdk.state.joint_state = SimpleNamespace(position=[0.1, -0.2, 0.3, -0.4] + [9.0] * 14)
    value._torso_api = Mock()
    value._torso_api.move_wheel_lower_joint.return_value = None
    yield value
    value.shutdown()


@pytest.mark.parametrize("unit", ["rad", "deg"])
def test_getter_converts_first_four_rad_to_fixed_deg(manager, unit):
    hw = MODULE.TorsoMixin()
    hw._angle_unit = unit
    hw._low_level_sdk_manager = manager
    result = hw.get_torso_joint()
    assert result.success
    assert result.data == pytest.approx([math.degrees(v) for v in [0.1, -0.2, 0.3, -0.4]])
    assert manager.robot_sdk.control.mock_calls == []


@pytest.mark.parametrize("positions", [None, [], [0.0] * 3, [math.nan] * 4, [math.inf] * 4, [True] * 4, [None] * 4])
def test_feedback_invalid_blocks_trajectory_before_sdk(manager, positions):
    manager.robot_sdk.state.joint_state = SimpleNamespace(position=positions)
    assert not manager.get_torso_joint().success
    assert not manager.set_torso_joint_traj(FRAMES, 1.0).success
    manager._torso_api.move_wheel_lower_joint.assert_not_called()
    assert manager.robot_sdk.control.mock_calls == []


@pytest.mark.parametrize("frames", [FRAMES, [FRAMES[0]]])
@pytest.mark.parametrize("unit", ["deg", "rad"])
def test_trajectory_passes_degrees_once_without_inserting_own_start(manager, frames, unit):
    hw = MODULE.TorsoMixin()
    hw._angle_unit = unit
    hw._low_level_sdk_manager = manager
    result = hw.set_torso_joint_traj(frames, 1.0)
    assert result.success and result.data is None
    manager._torso_api.move_wheel_lower_joint.assert_called_once_with(
        joint_traj=frames, total_time=1.0, asynchronous=False)
    assert manager.robot_sdk.control.mock_calls == []  # 模式准备只属于SDK内部。


@pytest.mark.parametrize("frames,time", [
    ([], 1.0), ([0.0] * 4, 1.0), ([[0.0] * 3], 1.0), ([[True] * 4], 1.0),
    ([[math.nan] * 4], 1.0), (FRAMES, None), (FRAMES, True), (FRAMES, 0),
    (FRAMES, 0.009), (FRAMES, math.inf), (FRAMES, math.nan),
])
def test_invalid_trajectory_is_rejected_before_sdk(manager, frames, time):
    with pytest.raises(ValueError):
        validate_torso_joint_traj(frames, time)
    assert not manager.set_torso_joint_traj(frames, time).success
    manager._torso_api.move_wheel_lower_joint.assert_not_called()


def test_trajectory_minimum_and_copy():
    source = [list(FRAMES[0])]
    result, duration = validate_torso_joint_traj(source, 0.01)
    assert duration == 0.01 and result == source
    source[0][0] += 1
    assert result != source


def test_sdk_exception_and_uninitialized_manager(manager):
    manager._torso_api.move_wheel_lower_joint.side_effect = RuntimeError("failed")
    assert not manager.set_torso_joint_traj(FRAMES, 1.0).success
    manager._initialized = False
    assert not manager.get_torso_joint().success
    assert not manager.set_torso_joint_traj(FRAMES, 1.0).success


def test_native_torso_api_prepends_feedback_and_requests_arm_only(manager, monkeypatch):
    from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes import api
    resample = Mock()
    monkeypatch.setattr(api, "resample_and_execute_traj", resample)
    manager._torso_api = api.TorsoAPI(manager.robot_sdk)
    assert manager.set_torso_joint_traj(FRAMES, 1.0).success
    resample.assert_called_once()
    frames, duration, publish = resample.call_args.args
    assert frames[0] == pytest.approx([math.degrees(v) for v in [0.1, -0.2, 0.3, -0.4]])
    assert frames[1:] == FRAMES
    assert duration == 1.0
    manager.robot_sdk.control.set_manipulation_mpc_mode.assert_called_once_with(
        api.KuavoManipulationMpcCtrlMode.ArmOnly)
    publish(FRAMES[0])
    manager.robot_sdk.control.control_wheel_lower_joint.assert_called_once_with(FRAMES[0])


def test_torso_api_lazy_cache_and_shutdown(manager, monkeypatch):
    from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes import api
    factory = Mock()
    monkeypatch.setattr(api, "TorsoAPI", factory)
    manager._torso_api = None
    assert manager._get_torso_api() is factory.return_value
    assert manager._get_torso_api() is factory.return_value
    factory.assert_called_once_with(manager.robot_sdk)
    manager.shutdown()
    factory.return_value._pool.shutdown.assert_called_once_with(wait=True)
    assert manager._torso_api is None


@pytest.mark.parametrize("kind", ["getter", "traj"])
@pytest.mark.parametrize("fault", ["success", "failure", "exception"])
def test_skills_cache_terminal_result_without_repeating(kind, fault):
    hw = Mock()
    if kind == "getter":
        skill = GetTorsoJointSkill(hw)
        params = GetTorsoJointParams()
        method = hw.get_torso_joint
        expected = Result.ok(data=FRAMES[0])
    else:
        skill = SetTorsoJointTrajSkill(hw)
        params = SetTorsoJointTrajParams(joint_traj=FRAMES, total_time=1.0)
        method = hw.set_torso_joint_traj
        expected = Result.ok()
    if fault == "exception":
        method.side_effect = RuntimeError("failed")
    else:
        method.return_value = Result.fail("refused") if fault == "failure" else expected
    assert skill.initialize(params).success
    result = skill.execute()
    assert result.success == (fault == "success")
    assert skill.is_finished()
    assert skill.execute() is result
    method.assert_called_once()
    skill.initialize(params)
    assert not skill.is_finished()
    skill.execute()
    assert method.call_count == 2


def test_malformed_feedback_and_invalid_trajectory_never_report_success():
    hw = Mock()
    hw.get_torso_joint.return_value = Result.ok(data=[0.0] * 3)
    skill = GetTorsoJointSkill(hw)
    skill.initialize(GetTorsoJointParams())
    result = skill.execute()
    assert not result.success and skill.execute() is result
    trajectory = SetTorsoJointTrajSkill(hw)
    assert not trajectory.initialize(SetTorsoJointTrajParams()).success
    assert not trajectory.execute().success
    hw.set_torso_joint_traj.assert_not_called()


def test_getter_node_clears_output_failure_dryrun_and_reinitialization(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hw = Mock()
    hw.get_torso_joint.return_value = Result.ok(data=FRAMES[0])
    getter = Mock(return_value=hw)
    monkeypatch.setattr(getter_node, "get_shared_hardware", getter)
    node = getter_node.GetTorsoJoint("read", "read", "ns", {"output_key": "feedback"})
    Blackboard.set("feedback", "old")
    node.initialise()
    assert Blackboard.get("feedback") is None
    assert node.update() == Status.SUCCESS
    assert Blackboard.get("feedback") == FRAMES[0]
    assert node.update() == Status.SUCCESS
    hw.get_torso_joint.assert_called_once()
    hw.get_torso_joint.return_value = Result.fail("unavailable")
    node.initialise()
    assert node.update() == Status.FAILURE
    assert Blackboard.get("feedback") is None
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    getter.reset_mock()
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert Blackboard.get("feedback") is None
    getter.assert_not_called()


@pytest.mark.parametrize("key", [None, "", " ", 1])
def test_getter_invalid_output_never_acquires_hardware(monkeypatch, key):
    getter = Mock(side_effect=AssertionError("no hardware"))
    monkeypatch.setattr(getter_node, "get_shared_hardware", getter)
    node = getter_node.GetTorsoJoint("read", "read", "ns", {"output_key": key})
    node.initialise()
    assert node.update() == Status.FAILURE
    getter.assert_not_called()


def test_trajectory_node_runtime_inputs_failure_cache_and_dryrun(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hw = Mock()
    hw.set_torso_joint_traj.return_value = Result.fail("refused")
    getter = Mock(return_value=hw)
    monkeypatch.setattr(traj_node, "get_shared_hardware", getter)
    node = traj_node.SetTorsoJointTraj("traj", "traj", "ns", {
        "joint_traj__board_key": "frames", "total_time__board_key": "duration"})
    Blackboard.set("frames", FRAMES)
    Blackboard.set("duration", "1.0")
    node.initialise()
    assert node.update() == Status.FAILURE
    assert node.update() == Status.FAILURE
    hw.set_torso_joint_traj.assert_called_once_with(joint_traj=FRAMES, total_time=1.0)
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    getter.reset_mock()
    node.initialise()
    assert node.update() == Status.SUCCESS
    Blackboard.set("frames", [])
    node.initialise()
    assert node.update() == Status.FAILURE
    getter.assert_not_called()


def test_optional_contracts_and_discovery():
    assert not IHardware.get_torso_joint(object()).success
    assert not IHardware.set_torso_joint_traj(object(), FRAMES, 1.0).success
    assert not {"get_torso_joint", "set_torso_joint_traj"} & IHardware.__abstractmethods__
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index["GetTorsoJoint"] == "orchestration.nodes.get_torso_joint"
    assert factory._node_index["SetTorsoJointTraj"] == "orchestration.nodes.set_torso_joint_traj"
