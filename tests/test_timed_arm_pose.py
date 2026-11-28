"""timed_arm_pose Adapter/Skill/Node离线回归，不初始化SDK、ROS或发送真机指令。"""

import copy
import importlib.util
import json
import math
from pathlib import Path
from unittest.mock import Mock, call

import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from core.domain.pose import validate_timed_arm_pose_targets
from core.domain.result import Result
from core.interfaces.i_hardware import IHardware
from orchestration.nodes import timed_arm_pose as node_module
from skills.atomic.refactored_sdk.timed_arm_pose import TimedArmPoseParams, TimedArmPoseSkill

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "timedcmd_under_test", ROOT / "adapters/hardware/leju_wheeled/mixins/timedcmd_mixin.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
pytestmark = pytest.mark.unit

# 仅为Mock测试样本，不是现场运动目标。
LEFT = [0.12, 0.23, 0.34, 90.0, -45.0, 30.0]
RIGHT = [0.21, -0.32, 0.43, -60.0, 15.0, -20.0]


class FakeHardware(MODULE.TimedCmdMixin):
    def __init__(self, angle_unit="deg"):
        self._angle_unit = angle_unit
        self._timed_ros_manager = Mock()
        self._timed_ros_manager.send_multi_commands.return_value = Result.ok(data={"actual_time": 4.0})
        self._to_rad = Mock(side_effect=AssertionError("must not convert by legacy angle_unit"))
        self.set_arm_control_mode = Mock(side_effect=AssertionError("must not switch modes"))
        self.set_focus_ee = Mock(return_value=Result.ok())
        self.set_focus_z = Mock(return_value=Result.ok())


@pytest.mark.parametrize("unit", ["deg", "rad"])
@pytest.mark.parametrize("frame,planners", [("local", (6, 7)), ("base_link", (6, 7)),
                                          ("world", (4, 5)), ("odom", (4, 5))])
@pytest.mark.parametrize("sides", ["left", "right", "both"])
@pytest.mark.parametrize("is_sync", [False, True])
def test_adapter_maps_planners_and_converts_only_angles(unit, frame, planners, sides, is_sync):
    hw = FakeHardware(unit)
    left = list(LEFT) if sides in ("left", "both") else None
    right = list(RIGHT) if sides in ("right", "both") else None
    original = copy.deepcopy((left, right))
    expected = []
    for planner, pose in zip(planners, (left, right)):
        if pose is not None:
            expected.append({"planner_index": planner, "desire_time": 2.5,
                             "cmd_vec": pose[:3] + [math.radians(v) for v in pose[3:]]})
    result = hw.timed_arm_pose(left, right, 2.5, frame, is_sync)
    assert result is hw._timed_ros_manager.send_multi_commands.return_value
    hw._timed_ros_manager.send_multi_commands.assert_called_once_with(commands=expected, is_sync=is_sync)
    assert (left, right) == original
    hw._to_rad.assert_not_called()
    hw.set_arm_control_mode.assert_not_called()
    hw.set_focus_ee.assert_not_called()
    hw.set_focus_z.assert_not_called()


BAD_INPUTS = [
    {}, {"left_pose": []}, {"left_pose": LEFT[:5]}, {"left_pose": LEFT + [0]},
    {"left_pose": [math.nan] * 6}, {"left_pose": [math.inf] * 6},
    {"left_pose": [None] * 6}, {"left_pose": "[1,2,3,4,5,6]"},
    {"left_pose": LEFT, "right_pose": []}, {"left_pose": LEFT, "desire_time": None},
    {"left_pose": LEFT, "desire_time": True}, {"left_pose": LEFT, "desire_time": 0},
    {"left_pose": LEFT, "desire_time": -1}, {"left_pose": LEFT, "desire_time": math.nan},
    {"left_pose": LEFT, "desire_time": math.inf}, {"left_pose": LEFT, "desire_time": "bad"},
    {"left_pose": LEFT, "frame": "map"}, {"left_pose": LEFT, "frame": None},
    {"left_pose": LEFT, "frame": 2}, {"left_pose": LEFT, "is_sync": 1},
    {"left_pose": LEFT, "is_sync": "true"},
]


@pytest.mark.parametrize("overrides", BAD_INPUTS)
def test_bad_inputs_fail_before_adapter_send_and_skill_execution(overrides):
    query = {"desire_time": 2.5}
    query.update(copy.deepcopy(overrides))
    hw = FakeHardware()
    assert not hw.timed_arm_pose(**query).success
    hw._timed_ros_manager.send_multi_commands.assert_not_called()
    skill_hw = Mock()
    skill = TimedArmPoseSkill(skill_hw)
    assert not skill.initialize(TimedArmPoseParams(**query)).success
    assert not skill.execute().success
    skill_hw.timed_arm_pose.assert_not_called()


def test_validation_copies_inputs_and_accepts_tuples_and_small_positive_duration():
    left, right, duration, frame = validate_timed_arm_pose_targets(tuple(LEFT), None, 0.001, "base_link")
    assert left == LEFT and left is not LEFT
    assert right is None and duration == 0.001 and frame == "local"


def test_adapter_handles_missing_manager_failure_and_exception():
    hw = FakeHardware()
    failure = Result.fail("service refused", error_code="SDK_ERROR")
    hw._timed_ros_manager.send_multi_commands.return_value = failure
    assert hw.timed_arm_pose(left_pose=LEFT, desire_time=2.0) is failure
    hw._timed_ros_manager.send_multi_commands.side_effect = RuntimeError("unavailable")
    assert "unavailable" in hw.timed_arm_pose(right_pose=RIGHT, desire_time=2.0).message
    hw._timed_ros_manager = None
    assert not hw.timed_arm_pose(left_pose=LEFT, desire_time=2.0).success


@pytest.mark.parametrize("success", [False, True])
def test_skill_sends_once_snapshots_input_and_resets(success):
    hw = Mock()
    expected = Result.ok(data={"actual_time": 3.0}) if success else Result.fail("refused")
    hw.timed_arm_pose.return_value = expected
    hw.set_arm_control_mode.return_value = Result.ok()
    skill = TimedArmPoseSkill(hw)
    params = TimedArmPoseParams(left_pose=list(LEFT), desire_time=2.5, frame="odom")
    assert skill.initialize(params).success
    params.left_pose[0] = 999
    assert not skill.is_finished()
    assert skill.execute() is expected
    assert skill.execute() is expected
    assert skill.is_finished()
    hw.timed_arm_pose.assert_called_once_with(left_pose=LEFT, right_pose=None,
                                            desire_time=2.5, frame="world", is_sync=True)
    assert not skill.initialize(TimedArmPoseParams()).success
    assert not skill.execute().success
    assert not skill.is_finished()
    assert skill.initialize(TimedArmPoseParams(right_pose=RIGHT, desire_time=1.0)).success
    assert skill.execute() is expected
    assert hw.timed_arm_pose.call_count == 2
    assert not skill.initialize(object()).success


@pytest.fixture
def node_hardware(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hw = Mock()
    hw.timed_arm_pose.return_value = Result.ok(data={"actual_time": 4.0})
    hw.set_arm_control_mode.return_value = Result.ok()
    getter = Mock(return_value=hw)
    monkeypatch.setattr(node_module, "get_shared_hardware", getter)
    return hw, getter


def node(params):
    return node_module.TimedArmPose("timed_pose", "timed_pose", "test", params)


@pytest.mark.parametrize("success", [True, False])
def test_node_returns_service_status_once_without_wait(node_hardware, success):
    hw, getter = node_hardware
    hw.timed_arm_pose.return_value = Result.ok(data={"actual_time": 999.0}) if success else Result.fail("failed")
    motion = node({"left_pose": json.dumps(LEFT), "desire_time": "2.5", "frame": "world", "is_sync": "false"})
    motion.initialise()
    status = Status.SUCCESS if success else Status.FAILURE
    assert motion.update() == status
    assert motion.update() == status
    hw.timed_arm_pose.assert_called_once_with(left_pose=LEFT, right_pose=None,
                                            desire_time=2.5, frame="world", is_sync=False)
    getter.assert_called_once()
    if success:
        assert "actual_time=999.0" in motion.feedback_message
        assert "不是到位确认" in motion.feedback_message


@pytest.mark.parametrize("dry_run", [False, True])
@pytest.mark.parametrize("query", [q for q in BAD_INPUTS
                                  if not isinstance(q.get("left_pose"), str)
                                  and not isinstance(q.get("is_sync"), str)])
def test_node_invalid_inputs_do_not_get_hardware(node_hardware, monkeypatch, dry_run, query):
    hw, getter = node_hardware
    monkeypatch.setenv("STUDIO_DRY_RUN", "1" if dry_run else "0")
    motion = node({"desire_time": 2.5})
    motion.params.update(query)
    motion.initialise()
    assert motion.update() == Status.FAILURE
    assert motion.feedback_message
    getter.assert_not_called()
    hw.timed_arm_pose.assert_not_called()


def test_node_dry_run_and_reinitialization(node_hardware, monkeypatch):
    hw, getter = node_hardware
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    motion = node({"right_pose": RIGHT, "desire_time": 2.5})
    motion.initialise()
    assert motion.update() == Status.SUCCESS
    getter.assert_not_called()
    hw.timed_arm_pose.assert_not_called()
    monkeypatch.delenv("STUDIO_DRY_RUN")
    motion.initialise()
    assert motion.update() == Status.SUCCESS
    hw.timed_arm_pose.assert_called_once()
    motion.params["right_pose"] = None
    motion.initialise()
    assert motion.update() == Status.FAILURE
    motion.params["left_pose"] = LEFT
    motion.initialise()
    assert motion.update() == Status.SUCCESS
    assert hw.timed_arm_pose.call_count == 2


def test_node_reads_runtime_board_keys_with_precedence(node_hardware):
    hw, _ = node_hardware
    motion = node({"left_pose": [], "left_pose__board_key": "target",
                   "desire_time": -1, "desire_time__board_key": "duration",
                   "frame": "map", "frame__board_key": "frame",
                   "is_sync": 1, "is_sync__board_key": "sync"})
    Blackboard.set("target", json.dumps(LEFT))
    Blackboard.set("duration", 2.5)
    Blackboard.set("frame", "base_link")
    Blackboard.set("sync", False)
    motion.initialise()
    assert motion.update() == Status.SUCCESS
    hw.timed_arm_pose.assert_called_once_with(left_pose=LEFT, right_pose=None,
                                            desire_time=2.5, frame="local", is_sync=False)


def test_node_missing_board_or_invalid_json_fails_closed(node_hardware):
    hw, getter = node_hardware
    for params in ({"left_pose__board_key": "missing", "desire_time": 2.5},
                   {"left_pose": "[", "desire_time": 2.5}):
        motion = node(params)
        motion.initialise()
        assert motion.update() == Status.FAILURE
    getter.assert_not_called()
    hw.timed_arm_pose.assert_not_called()


def test_interface_is_optional_and_node_is_discoverable():
    assert "timed_arm_pose" not in IHardware.__abstractmethods__
    assert not IHardware.timed_arm_pose(object(), left_pose=LEFT, desire_time=2.5).success
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index["TimedArmPose"] == "orchestration.nodes.timed_arm_pose"


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
def test_manual_scripts_have_no_default_motion_and_reject_before_hardware(layer, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "manual_timed_pose_" + layer, ROOT / f"apps/test_internal/{layer}/timed_arm_pose.py")
    script = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(script)
    # 人工脚本可能已有现场参数；离线测试只覆盖空输入，不改写脚本文件。
    monkeypatch.setattr(script, "LEFT_POSE", None)
    monkeypatch.setattr(script, "RIGHT_POSE", None)
    monkeypatch.setattr(script, "DESIRE_TIME", None)
    assert script.MULTI_CONFIG["sdk_managers_whitelist"] == ([] if layer == "adapters" else ["low"])
    if layer == "adapters":
        assert not hasattr(script, "FOCUS_EE") and not hasattr(script, "FOCUS_Z")
    else:
        assert script.FOCUS_EE is script.FOCUS_Z is None
    monkeypatch.setattr(script, "init_logging", lambda: None)
    # main必须在导入硬件工厂/共享硬件前因空目标退出。
    import builtins
    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        if name in ("adapters.hardware.factory", "orchestration.shared_hardware"):
            pytest.fail("empty manual input must not load hardware")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    assert script.main() == 1


def test_new_pose_entry_does_not_change_timed_joint_dispatch():
    hw = FakeHardware("rad")
    values = [10.0] * 7
    assert hw.timed_arm_joint(left_arm_joint=values, desire_time=2.0).success
    hw._timed_ros_manager.send_multi_commands.assert_called_once_with(
        commands=[{"planner_index": 8, "desire_time": 2.0,
                   "cmd_vec": [math.radians(v) for v in values]}], is_sync=True)


@pytest.mark.parametrize("focus_ee", [None, False, True])
@pytest.mark.parametrize("focus_z", [None, False, True])
def test_focus_is_independent_and_set_before_single_motion(focus_ee, focus_z):
    hw = FakeHardware()
    events = Mock()
    events.attach_mock(hw.set_focus_ee, "ee")
    events.attach_mock(hw.set_focus_z, "z")
    events.attach_mock(hw._timed_ros_manager.send_multi_commands, "send")
    skill = TimedArmPoseSkill(hw)
    assert skill.initialize(TimedArmPoseParams(
        left_pose=LEFT, desire_time=2.5, focus_ee=focus_ee, focus_z=focus_z, prepare_arm_control=False)).success
    assert events.mock_calls == []
    result = skill.execute()
    assert skill.execute() is result
    assert result.success
    expected = []
    if focus_ee is not None:
        expected.append(call.ee(focus_ee))
    if focus_z is not None:
        expected.append(call.z(focus_z))
    expected.append(call.send(commands=[{
        "planner_index": 6, "desire_time": 2.5,
        "cmd_vec": LEFT[:3] + [math.radians(v) for v in LEFT[3:]],
    }], is_sync=True))
    assert events.mock_calls == expected  # 不自动恢复或额外设置另一focus。


@pytest.mark.parametrize("flag", ["focus_ee", "focus_z"])
@pytest.mark.parametrize("value", [0, 1, "false", "true", [], {}])
def test_invalid_focus_fails_all_layers_before_any_side_effect(node_hardware, monkeypatch, flag, value):
    query = dict(left_pose=LEFT, desire_time=2.5, focus_ee=False, focus_z=False)
    query[flag] = value
    hw = FakeHardware()
    skill = TimedArmPoseSkill(hw)
    assert not skill.initialize(TimedArmPoseParams(**query)).success
    assert not skill.execute().success
    hw.set_focus_ee.assert_not_called()
    hw.set_focus_z.assert_not_called()
    hw._timed_ros_manager.send_multi_commands.assert_not_called()
    _, getter = node_hardware
    # Node允许JSON布尔文本，但JSON数字/容器及双引号包裹的字符串仍须拒绝。
    if isinstance(value, str):
        query[flag] = json.dumps(value)
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    motion = node(query)
    motion.initialise()
    assert motion.update() == Status.FAILURE
    getter.assert_not_called()


@pytest.mark.parametrize("failure_at", ["ee", "z", "send"])
@pytest.mark.parametrize("raises", [False, True])
def test_focus_failure_blocks_motion_and_never_rolls_back(failure_at, raises):
    hw = FakeHardware()
    events = Mock()
    events.attach_mock(hw.set_focus_ee, "ee")
    events.attach_mock(hw.set_focus_z, "z")
    events.attach_mock(hw._timed_ros_manager.send_multi_commands, "send")
    failure = Result.fail("focus/service failed", error_code="SETUP_FAILED")
    target = getattr(events, failure_at)
    if raises:
        target.side_effect = RuntimeError("publish failed")
    else:
        target.return_value = failure
    skill = TimedArmPoseSkill(hw)
    assert skill.initialize(TimedArmPoseParams(
        right_pose=RIGHT, desire_time=2.5, focus_ee=False, focus_z=True, prepare_arm_control=False)).success
    result = skill.execute()
    assert skill.execute() is result
    assert not result.success
    if not raises:
        assert result is failure
    expected_names = {"ee": ["ee"], "z": ["ee", "z"], "send": ["ee", "z", "send"]}
    assert [c[0] for c in events.mock_calls] == expected_names[failure_at]
    assert hw.set_focus_ee.call_args_list == [call(False)]
    if failure_at == "ee":
        hw.set_focus_z.assert_not_called()
    else:
        hw.set_focus_z.assert_called_once_with(True)
    hw.set_arm_control_mode.assert_not_called()


def test_missing_manager_or_bad_target_does_not_set_focus():
    hw = FakeHardware()
    skill = TimedArmPoseSkill(hw)
    assert not skill.initialize(TimedArmPoseParams(left_pose=[], desire_time=2.5, focus_ee=False)).success
    assert not skill.execute().success
    hw._timed_ros_manager = None
    assert not hw.timed_arm_pose(left_pose=LEFT, desire_time=2.5).success
    hw.set_focus_ee.assert_not_called()
    hw.set_focus_z.assert_not_called()


@pytest.mark.parametrize("dry_run", [False, True])
def test_node_runtime_focus_reaches_adapter_once_or_dry_run_skips(node_hardware, monkeypatch, dry_run):
    hw, getter = node_hardware
    adapter = FakeHardware()
    adapter.set_arm_control_mode.side_effect = None
    adapter.set_arm_control_mode.return_value = Result.ok()
    hw.timed_arm_pose.side_effect = adapter.timed_arm_pose
    hw.set_focus_ee.side_effect = adapter.set_focus_ee
    hw.set_focus_z.side_effect = adapter.set_focus_z
    monkeypatch.setenv("STUDIO_DRY_RUN", "1" if dry_run else "0")
    motion = node({"left_pose": LEFT, "desire_time": 2.5,
                   "focus_ee": True, "focus_ee__board_key": "focus_ee_test",
                   "focus_z": False, "focus_z__board_key": "focus_z_test"})
    Blackboard.set("focus_ee_test", "false")
    Blackboard.set("focus_z_test", True)
    motion.initialise()
    assert motion.update() == Status.SUCCESS
    assert motion.update() == Status.SUCCESS
    if dry_run:
        getter.assert_not_called()
        adapter.set_focus_ee.assert_not_called()
        adapter.set_focus_z.assert_not_called()
        adapter._timed_ros_manager.send_multi_commands.assert_not_called()
    else:
        hw.timed_arm_pose.assert_called_once_with(
            left_pose=LEFT, right_pose=None, desire_time=2.5, frame="local", is_sync=True)
        adapter.set_focus_ee.assert_called_once_with(False)
        adapter.set_focus_z.assert_called_once_with(True)
        adapter._timed_ros_manager.send_multi_commands.assert_called_once()
        # 重新初始化允许再次设置；先前失败/成功结果不可沿用。
        motion.initialise()
        assert motion.update() == Status.SUCCESS
        assert adapter.set_focus_ee.call_count == 2


def test_focus_setup_failure_is_cached_through_node(node_hardware):
    hw, _ = node_hardware
    adapter = FakeHardware()
    adapter.set_arm_control_mode.side_effect = None
    adapter.set_arm_control_mode.return_value = Result.ok()
    adapter.set_focus_z.return_value = Result.fail("z failed")
    hw.timed_arm_pose.side_effect = adapter.timed_arm_pose
    hw.set_focus_ee.side_effect = adapter.set_focus_ee
    hw.set_focus_z.side_effect = adapter.set_focus_z
    motion = node({"left_pose": LEFT, "desire_time": 2.5, "focus_ee": False, "focus_z": False})
    motion.initialise()
    assert motion.update() == Status.FAILURE
    assert motion.update() == Status.FAILURE
    assert motion.feedback_message == "z failed"
    adapter.set_focus_ee.assert_called_once_with(False)
    adapter.set_focus_z.assert_called_once_with(False)
    adapter._timed_ros_manager.send_multi_commands.assert_not_called()


def test_focus_is_skill_only_not_a_motion_adapter_parameter():
    import inspect
    for method in (MODULE.TimedCmdMixin.timed_arm_pose, IHardware.timed_arm_pose,
                   validate_timed_arm_pose_targets):
        parameters = inspect.signature(method).parameters
        assert "focus_ee" not in parameters and "focus_z" not in parameters
    hw = FakeHardware()
    assert hw.timed_arm_pose(left_pose=LEFT, desire_time=2.5).success
    hw.set_focus_ee.assert_not_called()
    hw.set_focus_z.assert_not_called()


def test_skill_snapshots_focus_and_recovers_from_setup_failure():
    hw = Mock()
    hw.set_focus_ee.return_value = Result.ok()
    hw.set_arm_control_mode.return_value = Result.ok()
    failure = Result.fail("focus unavailable")
    hw.set_focus_z.return_value = failure
    hw.timed_arm_pose.return_value = Result.ok(data={"actual_time": 2.5})
    skill = TimedArmPoseSkill(hw)
    params = TimedArmPoseParams(left_pose=LEFT, desire_time=2.5, focus_ee=False, focus_z=True)
    assert skill.initialize(params).success
    params.focus_ee = True
    params.focus_z = False
    assert skill.execute() is failure
    assert skill.execute() is failure
    hw.set_focus_ee.assert_called_once_with(False)
    hw.set_focus_z.assert_called_once_with(True)
    hw.timed_arm_pose.assert_not_called()
    assert skill.initialize(TimedArmPoseParams(left_pose=LEFT, desire_time=2.5)).success
    assert skill.execute().success
    hw.timed_arm_pose.assert_called_once_with(left_pose=LEFT, right_pose=None,
                                            desire_time=2.5, frame="local", is_sync=True)
    # 默认None重新初始化不沿用前一轮focus，不触发额外设置。
    hw.set_focus_ee.assert_called_once()
    hw.set_focus_z.assert_called_once()


@pytest.mark.parametrize("prepare", [True, False])
def test_skill_mode_preparation_precedes_focus_and_send(prepare):
    hw = Mock()
    hw.set_arm_control_mode.return_value = Result.ok()
    hw.set_focus_ee.return_value = Result.ok()
    hw.set_focus_z.return_value = Result.ok()
    result = Result.ok(data={"actual_time": 2.5})
    hw.timed_arm_pose.return_value = result
    skill = TimedArmPoseSkill(hw)
    params = TimedArmPoseParams(left_pose=LEFT, desire_time=2.5, focus_ee=False, focus_z=True,
                                prepare_arm_control=prepare)
    assert skill.initialize(params).success
    assert hw.mock_calls == []
    params.prepare_arm_control = not prepare  # 初始化后参数变化不修改本轮策略。
    assert skill.execute() is result
    assert skill.execute() is result
    expected = [call.set_arm_control_mode(2)] if prepare else []
    expected += [call.set_focus_ee(False), call.set_focus_z(True),
                 call.timed_arm_pose(left_pose=LEFT, right_pose=None,
                                     desire_time=2.5, frame="local", is_sync=True)]
    assert hw.mock_calls == expected
    assert TimedArmPoseParams().prepare_arm_control is True


@pytest.mark.parametrize("raises", [False, True])
def test_mode_preparation_failure_blocks_focus_and_goal_and_is_cached(raises):
    hw = Mock()
    failure = Result.fail("mode unavailable", error_code="MODE_FAILED")
    if raises:
        hw.set_arm_control_mode.side_effect = RuntimeError("mode error")
    else:
        hw.set_arm_control_mode.return_value = failure
    skill = TimedArmPoseSkill(hw)
    assert skill.initialize(TimedArmPoseParams(left_pose=LEFT, desire_time=2.5, focus_ee=False)).success
    result = skill.execute()
    assert not result.success
    if not raises:
        assert result is failure
    assert skill.execute() is result
    hw.set_arm_control_mode.assert_called_once_with(2)
    hw.set_focus_ee.assert_not_called()
    hw.set_focus_z.assert_not_called()
    hw.timed_arm_pose.assert_not_called()
    assert skill.is_finished()


@pytest.mark.parametrize("value", [None, 0, 1, "true", [], {}])
def test_invalid_mode_policy_rejected_before_hardware(node_hardware, monkeypatch, value):
    hw, getter = node_hardware
    skill = TimedArmPoseSkill(hw)
    assert not skill.initialize(TimedArmPoseParams(
        left_pose=LEFT, desire_time=2.5, prepare_arm_control=value)).success
    assert not skill.execute().success
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    motion = node({"left_pose": LEFT, "desire_time": 2.5,
                   "prepare_arm_control": json.dumps(value) if isinstance(value, str) else value})
    motion.initialise()
    assert motion.update() == Status.FAILURE
    getter.assert_not_called()
    assert hw.mock_calls == []


@pytest.mark.parametrize("dry_run", [True, False])
def test_node_default_mode_and_runtime_opt_out(node_hardware, monkeypatch, dry_run):
    hw, getter = node_hardware
    monkeypatch.setenv("STUDIO_DRY_RUN", "1" if dry_run else "0")
    motion = node({"left_pose": LEFT, "desire_time": 2.5})
    motion.initialise()
    assert motion.update() == Status.SUCCESS
    assert motion.update() == Status.SUCCESS
    if dry_run:
        getter.assert_not_called()
        hw.set_arm_control_mode.assert_not_called()
    else:
        hw.set_arm_control_mode.assert_called_once_with(2)
    hw.reset_mock()
    motion.params["prepare_arm_control__board_key"] = "prepare_mode_test"
    Blackboard.set("prepare_mode_test", "false")
    motion.initialise()
    assert motion.update() == Status.SUCCESS
    hw.set_arm_control_mode.assert_not_called()


def test_node_mode_failure_returns_failure_without_motion(node_hardware):
    hw, _ = node_hardware
    hw.set_arm_control_mode.return_value = Result.fail("mode failed")
    motion = node({"left_pose": LEFT, "desire_time": 2.5, "focus_z": True})
    motion.initialise()
    assert motion.update() == Status.FAILURE
    assert motion.update() == Status.FAILURE
    hw.set_arm_control_mode.assert_called_once_with(2)
    hw.set_focus_z.assert_not_called()
    hw.timed_arm_pose.assert_not_called()
