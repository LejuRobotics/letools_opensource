# -*- coding: utf-8 -*-
"""MPC 搬运关节快照恢复的离线测试；不访问 ROS 或真实硬件。"""

import math

import pytest

from core.domain.result import Result
from core.domain.transform import Transform
from skills.atomic.refactored_sdk import palletizing_line_mpc_restore as restore_module
from skills.atomic.refactored_sdk.palletizing_line_mpc import PalletizingLineMpcSkill
from skills.atomic.refactored_sdk.palletizing_line_mpc_restore import (
    PalletizingLineMpcRestoreParams,
    PalletizingLineMpcRestoreSkill,
)


class _FakeHardware:
    def __init__(self):
        self.initial_arms = [math.radians(index + 1) for index in range(14)]
        self.initial_legs = [10.0, 20.0, 30.0, 40.0]
        self.arms = list(self.initial_arms)
        self.legs = list(self.initial_legs)
        self.mode_calls = []
        self.arm_commands = []
        self.leg_commands = []
        self.fail_arm_command = False
        self.apply_arm_command = True
        self.apply_leg_command = True

    def get_arm_joint_positions(self):
        return Result.ok(data=list(self.arms))

    def get_leg_joint_positions(self):
        return Result.ok(data=list(self.legs))

    def set_arm_control_mode(self, mode):
        self.mode_calls.append(mode)
        return Result.ok()

    def send_arm_joint_trajectory(self, positions, time_sec):
        self.arm_commands.append((list(positions), time_sec))
        if self.fail_arm_command:
            return Result.fail("mock 手臂命令失败")
        if self.apply_arm_command:
            self.arms = [math.radians(value) for value in positions]
        return Result.ok()

    def send_leg_joint_command(self, positions):
        self.leg_commands.append(list(positions))
        if self.apply_leg_command:
            self.legs = list(positions)
        return Result.ok()

    def move_away_from_snapshot(self):
        self.arms = [value + 0.2 for value in self.initial_arms]
        self.legs = [value + 5.0 for value in self.initial_legs]


def _make_skill(hardware, params=None):
    skill = PalletizingLineMpcRestoreSkill(hardware=hardware)
    if params is None:
        params = PalletizingLineMpcRestoreParams()
    assert skill.initialize(params).success
    return skill


def _patch_sleep(monkeypatch):
    monkeypatch.setattr(restore_module.time, "sleep", lambda _: None)


def test_default_does_not_prepare_pose():
    params = PalletizingLineMpcRestoreParams()
    assert params.prepare_pose is False
    assert params.roundtrip_axis == "full"


@pytest.mark.parametrize("axis,changed_index,distance", [
    ("x", 0, 0.4),
    ("y", 1, 0.3),
    ("z", 2, 0.2),
])
def test_axis_roundtrip_only_changes_selected_axis_and_restores(
        monkeypatch, axis, changed_index, distance):
    hardware = _FakeHardware()
    start = (0.4, 0.5, 0.6)
    hardware.get_transform = lambda *args: Result.ok(
        data=Transform.from_lists(position=start, orientation=[0.0, 0.0, 0.0, 1.0]))
    params = PalletizingLineMpcRestoreParams(
        arms="left",
        roundtrip_axis=axis,
        x_distance=0.4,
        y_offset=0.3,
        z_height=0.2,
    )
    skill = _make_skill(hardware, params)
    _patch_sleep(monkeypatch)
    segments = []

    def capture_segment(segment, trajectories):
        segments.append(segment)
        for point in trajectories[0]["timed_traj"]:
            for index in range(3):
                if index != changed_index:
                    assert point["cmd_vec"][index] == pytest.approx(start[index])
        hardware.move_away_from_snapshot()
        return Result.ok()

    monkeypatch.setattr(skill, "_run_segment", capture_segment)
    result = skill.execute()

    assert result.success, result.message
    assert [segment.name for segment in segments] == [
        f"roundtrip_{axis}_out", f"roundtrip_{axis}_back"]
    assert segments[0].start == pytest.approx(start)
    assert segments[0].end[changed_index] == pytest.approx(start[changed_index] + distance)
    assert segments[1].start == pytest.approx(segments[0].end)
    assert segments[1].end == pytest.approx(start)
    assert "18 关节已恢复并验证" in result.message


def test_invalid_roundtrip_axis_is_rejected():
    skill = PalletizingLineMpcRestoreSkill(hardware=_FakeHardware())

    result = skill.initialize(PalletizingLineMpcRestoreParams(roundtrip_axis="xy"))

    assert not result.success
    assert "roundtrip_axis" in result.message


@pytest.mark.parametrize("params", [
    PalletizingLineMpcRestoreParams(roundtrip_axis="x", x_distance=0.0),
    PalletizingLineMpcRestoreParams(roundtrip_axis="y", y_offset=0.0),
])
def test_axis_roundtrip_distance_must_be_positive(params):
    skill = PalletizingLineMpcRestoreSkill(hardware=_FakeHardware())

    result = skill.initialize(params)

    assert not result.success
    assert "往返距离必须为正" in result.message


def test_success_restores_and_verifies_all_eighteen_joints(monkeypatch):
    hardware = _FakeHardware()
    _patch_sleep(monkeypatch)

    def move_then_succeed(skill):
        hardware.move_away_from_snapshot()
        return Result.ok("mock 搬运完成")

    monkeypatch.setattr(PalletizingLineMpcSkill, "on_execute", move_then_succeed)
    skill = _make_skill(hardware)

    result = skill.execute()

    assert result.success
    assert "18 关节已恢复并验证" in result.message
    assert hardware.mode_calls == [2], "恢复不得调用 mode=1 触发固件默认姿态复位"
    assert hardware.arm_commands[0][0] == pytest.approx(
        [math.degrees(value) for value in hardware.initial_arms])
    assert hardware.arm_commands[0][1] == 5.0
    assert hardware.leg_commands == [hardware.initial_legs]
    assert hardware.arms == pytest.approx(hardware.initial_arms)
    assert hardware.legs == pytest.approx(hardware.initial_legs)
    assert skill.is_finished()


def test_movement_failure_still_restores_snapshot(monkeypatch):
    hardware = _FakeHardware()
    _patch_sleep(monkeypatch)

    def fail_after_motion(skill):
        hardware.move_away_from_snapshot()
        return Result.fail("mock 搬运失败")

    monkeypatch.setattr(PalletizingLineMpcSkill, "on_execute", fail_after_motion)
    skill = _make_skill(hardware)

    result = skill.execute()

    assert not result.success
    assert "mock 搬运失败" in result.message
    assert "18 关节已恢复并验证" in result.message
    assert hardware.arms == pytest.approx(hardware.initial_arms)
    assert hardware.legs == pytest.approx(hardware.initial_legs)


def test_movement_exception_is_reported_after_restore(monkeypatch):
    hardware = _FakeHardware()
    _patch_sleep(monkeypatch)

    def raise_after_motion(skill):
        hardware.move_away_from_snapshot()
        raise RuntimeError("mock 搬运异常")

    monkeypatch.setattr(PalletizingLineMpcSkill, "on_execute", raise_after_motion)
    skill = _make_skill(hardware)

    result = skill.execute()

    assert not result.success
    assert "mock 搬运异常" in result.message
    assert hardware.arms == pytest.approx(hardware.initial_arms)
    assert hardware.legs == pytest.approx(hardware.initial_legs)


def test_send_success_does_not_replace_position_verification(monkeypatch):
    hardware = _FakeHardware()
    hardware.apply_arm_command = False
    _patch_sleep(monkeypatch)

    def move_then_succeed(skill):
        hardware.move_away_from_snapshot()
        return Result.ok("mock 搬运完成")

    monkeypatch.setattr(PalletizingLineMpcSkill, "on_execute", move_then_succeed)
    skill = _make_skill(hardware)

    result = skill.execute()

    assert not result.success
    assert "双臂未恢复到快照" in result.message
    assert hardware.leg_commands, "手臂验证失败不能阻止下肢恢复"


def test_leg_send_success_does_not_replace_position_verification(monkeypatch):
    hardware = _FakeHardware()
    hardware.apply_leg_command = False
    _patch_sleep(monkeypatch)

    def move_then_succeed(skill):
        hardware.move_away_from_snapshot()
        return Result.ok("mock 搬运完成")

    monkeypatch.setattr(PalletizingLineMpcSkill, "on_execute", move_then_succeed)
    skill = _make_skill(hardware)

    result = skill.execute()

    assert not result.success
    assert "下肢未恢复到快照" in result.message
    assert hardware.arms == pytest.approx(hardware.initial_arms)


def test_arm_command_failure_does_not_skip_leg_restore(monkeypatch):
    hardware = _FakeHardware()
    hardware.fail_arm_command = True
    _patch_sleep(monkeypatch)

    def move_then_fail(skill):
        hardware.move_away_from_snapshot()
        return Result.fail("mock 搬运失败")

    monkeypatch.setattr(PalletizingLineMpcSkill, "on_execute", move_then_fail)
    skill = _make_skill(hardware)

    result = skill.execute()

    assert not result.success
    assert "双臂恢复命令失败" in result.message
    assert hardware.leg_commands == [hardware.initial_legs]
    assert hardware.legs == pytest.approx(hardware.initial_legs)


def test_incomplete_snapshot_prevents_any_motion_or_restore(monkeypatch):
    hardware = _FakeHardware()
    hardware.get_leg_joint_positions = lambda: Result.fail("mock 下肢状态不可用")
    movement_called = []

    def movement(skill):
        movement_called.append(True)
        return Result.ok()

    monkeypatch.setattr(PalletizingLineMpcSkill, "on_execute", movement)
    skill = _make_skill(hardware)

    result = skill.execute()

    assert not result.success
    assert "已取消搬运" in result.message
    assert not movement_called
    assert not hardware.arm_commands
    assert not hardware.leg_commands


def test_restore_parameters_are_validated():
    skill = PalletizingLineMpcRestoreSkill(hardware=_FakeHardware())

    result = skill.initialize(PalletizingLineMpcRestoreParams(restore_time=0.0))

    assert not result.success
    assert "restore_time" in result.message
