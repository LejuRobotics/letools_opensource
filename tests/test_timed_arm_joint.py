"""定时关节Skill模式准备和Node透传的离线测试；不连接ROS/SDK。"""

import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock, call

import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from core.domain.result import Result
from orchestration.nodes import timed_arm_joint as motion
from skills.atomic.refactored_sdk.timed_arm_joint import TimedArmJointParams, TimedArmJointSkill

pytestmark = pytest.mark.unit
ROOT = Path(__file__).resolve().parents[1]
JOINT = [float(i) for i in range(7)]  # 仅Mock样本，不是推荐运动数值。


@pytest.fixture
def hardware(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hw = Mock()
    hw.set_arm_control_mode.return_value = Result.ok()
    hw.timed_arm_joint.return_value = Result.ok(data={"actual_time": 3.0})
    getter = Mock(return_value=hw)
    monkeypatch.setattr(motion, "get_shared_hardware", getter)
    return hw, getter


@pytest.mark.parametrize("prepare", [True, False])
@pytest.mark.parametrize("side", ["left", "right", "both"])
def test_skill_prepares_mode_before_target_once_and_snapshots(prepare, side, hardware):
    hw, _ = hardware
    left = list(JOINT) if side in ("left", "both") else None
    right = list(JOINT) if side in ("right", "both") else None
    params = TimedArmJointParams(left_arm_joint=left, right_arm_joint=right,
                                 desire_time=2.5, is_sync=False, prepare_arm_control=prepare)
    skill = TimedArmJointSkill(hw)
    assert skill.initialize(params).success
    assert hw.mock_calls == []
    params.prepare_arm_control = not prepare
    if left is not None:
        left[0] = 999
    result = skill.execute()
    assert result is hw.timed_arm_joint.return_value
    assert skill.execute() is result
    assert skill.is_finished()
    expected = [call.set_arm_control_mode(2)] if prepare else []
    expected += [call.timed_arm_joint(left_arm_joint=JOINT if side in ("left", "both") else None,
                                    right_arm_joint=JOINT if side in ("right", "both") else None,
                                    desire_time=2.5, is_sync=False)]
    assert hw.mock_calls == expected
    assert TimedArmJointParams().prepare_arm_control is True


@pytest.mark.parametrize("at", ["set_arm_control_mode", "timed_arm_joint"])
@pytest.mark.parametrize("raises", [False, True])
def test_failure_and_exception_cached_without_retry_or_restore(hardware, at, raises):
    hw, _ = hardware
    failure = Result.fail("failed", error_code="CONTROL_FAILED")
    if raises:
        getattr(hw, at).side_effect = RuntimeError("unavailable")
    else:
        getattr(hw, at).return_value = failure
    skill = TimedArmJointSkill(hw)
    assert skill.initialize(TimedArmJointParams(left_arm_joint=JOINT, desire_time=2.5)).success
    result = skill.execute()
    assert not result.success
    if not raises:
        assert result is failure
    assert skill.execute() is result
    hw.set_arm_control_mode.assert_called_once_with(2)
    if at == "set_arm_control_mode":
        hw.timed_arm_joint.assert_not_called()
    else:
        hw.timed_arm_joint.assert_called_once()
    hw.set_focus_ee.assert_not_called()
    hw.set_focus_z.assert_not_called()


@pytest.mark.parametrize("value", [None, 0, 1, "true", [], {}])
def test_invalid_policy_rejected_before_hardware(hardware, value, monkeypatch):
    hw, getter = hardware
    params = dict(left_arm_joint=JOINT, desire_time=2.5, prepare_arm_control=value)
    skill = TimedArmJointSkill(hw)
    assert not skill.initialize(TimedArmJointParams(**params)).success
    assert not skill.execute().success
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    if isinstance(value, str):
        params["prepare_arm_control"] = json.dumps(value)
    node = motion.TimedArmJoint("joint", "joint", "test", params)
    node.initialise()
    assert node.update() == Status.FAILURE
    assert hw.mock_calls == []
    getter.assert_not_called()


@pytest.mark.parametrize("dry_run", [True, False])
def test_node_defaults_and_runtime_policy(hardware, monkeypatch, dry_run):
    hw, getter = hardware
    monkeypatch.setenv("STUDIO_DRY_RUN", "1" if dry_run else "0")
    node = motion.TimedArmJoint("joint", "joint", "test", {
        "left_arm_joint": json.dumps(JOINT), "desire_time": "2.5"})
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert node.update() == Status.SUCCESS
    if dry_run:
        getter.assert_not_called()
        assert hw.mock_calls == []
    else:
        hw.set_arm_control_mode.assert_called_once_with(2)
        hw.timed_arm_joint.assert_called_once()
    hw.reset_mock()
    node.params["prepare_arm_control__board_key"] = "prepare_joint_test"
    Blackboard.set("prepare_joint_test", "false")
    node.initialise()
    assert node.update() == Status.SUCCESS
    hw.set_arm_control_mode.assert_not_called()


def test_node_mode_failure_blocks_target(hardware):
    hw, _ = hardware
    hw.set_arm_control_mode.return_value = Result.fail("mode failed")
    node = motion.TimedArmJoint("joint", "joint", "test", {"right_arm_joint": JOINT, "desire_time": 2.5})
    node.initialise()
    assert node.update() == Status.FAILURE
    assert node.update() == Status.FAILURE
    assert node.feedback_message == "mode failed"
    hw.set_arm_control_mode.assert_called_once_with(2)
    hw.timed_arm_joint.assert_not_called()


def test_reinitialization_recovers_and_invalid_input_cannot_send(hardware):
    hw, _ = hardware
    skill = TimedArmJointSkill(hw)
    assert skill.initialize(TimedArmJointParams(left_arm_joint=JOINT, desire_time=2.5)).success
    assert skill.execute().success
    assert not skill.initialize(TimedArmJointParams()).success
    assert not skill.execute().success
    assert not skill.is_finished()
    assert skill.initialize(TimedArmJointParams(right_arm_joint=JOINT, desire_time=1.0,
                                               prepare_arm_control=False)).success
    assert skill.execute().success
    hw.set_arm_control_mode.assert_called_once_with(2)
    assert hw.timed_arm_joint.call_count == 2


@pytest.mark.parametrize("layer", ["skills", "nodes"])
def test_manual_scripts_validate_policy_before_getting_hardware(layer, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "manual_joint_" + layer, ROOT / f"apps/test_internal/{layer}/timed_arm_joint.py")
    script = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(script)
    assert script.MULTI_CONFIG["sdk_managers_whitelist"] == ["low"]
    assert script.PREPARE_ARM_CONTROL is True
    monkeypatch.setattr(script, "init_logging", lambda: None)
    monkeypatch.setattr(script, "LEFT_ARM_JOINT", JOINT)
    monkeypatch.setattr(script, "DESIRE_TIME", 2.5)
    monkeypatch.setattr(script, "PREPARE_ARM_CONTROL", "invalid")
    import builtins
    original = builtins.__import__

    def guarded(name, *args, **kwargs):
        if name in ("adapters.hardware.factory", "orchestration.shared_hardware"):
            pytest.fail("invalid policy accessed hardware")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)
    assert script.main() == 1
