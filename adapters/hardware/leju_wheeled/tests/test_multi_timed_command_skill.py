# -*- coding: utf-8 -*-
"""Unit tests for the configurable multi-planner TimedCmd skill."""

import time
from unittest.mock import MagicMock

import pytest

from core.domain.result import Result
from skills.atomic.refactored_sdk.multi_timed_command import (
    MultiTimedCommandParams,
    MultiTimedCommandSkill,
)


@pytest.mark.unit
class TestMultiTimedCommandSkill:
    def test_sends_configured_planners_in_one_request(self, monkeypatch):
        sleeps = []
        monkeypatch.setattr(time, "sleep", lambda seconds: sleeps.append(seconds))
        hardware = MagicMock()
        hardware.set_arm_control_mode.return_value = Result.ok()
        hardware.send_timed_multi_commands.return_value = Result.ok(
            data={"actual_time": 2.5}
        )
        commands = [
            {"planner_index": 2, "cmd_vec": [0.0, 0.1, 20.0, 10.0]},
            {"planner_index": 8, "cmd_vec": [0.0] * 7},
            {"planner_index": 9, "cmd_vec": [0.0] * 7, "desire_time": 4.0},
        ]
        skill = MultiTimedCommandSkill(hardware)

        assert skill.initialize(
            MultiTimedCommandParams(
                commands=commands,
                desire_time=3.0,
                is_sync=True,
                settle_time=0.5,
            )
        ).success
        assert skill.execute().success

        hardware.send_timed_multi_commands.assert_called_once_with(
            commands=[
                {"planner_index": 2, "desire_time": 3.0, "cmd_vec": [0.0, 0.1, 20.0, 10.0]},
                {"planner_index": 8, "desire_time": 3.0, "cmd_vec": [0.0] * 7},
                {"planner_index": 9, "desire_time": 4.0, "cmd_vec": [0.0] * 7},
            ],
            is_sync=True,
        )
        assert sleeps == [3.0]
        assert skill.is_finished()

    @pytest.mark.parametrize("focus_ee", [False, True])
    def test_sets_matching_focus_before_sending(self, focus_ee, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda _: None)
        hardware = MagicMock()
        hardware.set_arm_control_mode.return_value = Result.ok()
        hardware.set_focus_ee.return_value = Result.ok()
        hardware.set_focus_z.return_value = Result.ok()
        hardware.send_timed_multi_commands.return_value = Result.ok()
        skill = MultiTimedCommandSkill(hardware)
        skill.initialize(MultiTimedCommandParams(
            commands=[{"planner_index": 6, "cmd_vec": [0.0] * 6}],
            focus_ee=focus_ee,
        ))

        assert skill.execute().success
        hardware.set_focus_ee.assert_called_once_with(focus_ee)
        hardware.set_focus_z.assert_called_once_with(focus_ee)

    def test_does_not_send_when_focus_setup_fails(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda _: None)
        hardware = MagicMock()
        hardware.set_arm_control_mode.return_value = Result.ok()
        hardware.set_focus_ee.return_value = Result.ok()
        hardware.set_focus_z.return_value = Result.fail("unavailable")
        skill = MultiTimedCommandSkill(hardware)
        skill.initialize(MultiTimedCommandParams(
            commands=[{"planner_index": 6, "cmd_vec": [0.0] * 6}]
        ))

        result = skill.execute()

        assert not result.success
        assert "set_focus_z(False) failed" in result.message
        hardware.send_timed_multi_commands.assert_not_called()
        assert skill.is_finished()

    def test_rejects_missing_focus_interface(self):
        hardware = MagicMock(spec=["set_arm_control_mode", "send_timed_multi_commands"])
        hardware.set_arm_control_mode.return_value = Result.ok()
        skill = MultiTimedCommandSkill(hardware)
        skill.initialize(MultiTimedCommandParams(
            commands=[{"planner_index": 6, "cmd_vec": [0.0] * 6}]
        ))

        result = skill.execute()

        assert not result.success
        assert "does not implement set_focus_ee" in result.message
        hardware.send_timed_multi_commands.assert_not_called()

    @pytest.mark.parametrize(
        "commands,message",
        [
            ([], "must not be empty"),
            ([{"planner_index": 8, "cmd_vec": [0.0] * 6}], "expects 7"),
            ([{"planner_index": 10, "cmd_vec": [0.0]}], "must be in [0, 9]"),
            (
                [
                    {"planner_index": 3, "cmd_vec": [0.0] * 4},
                    {"planner_index": 3, "cmd_vec": [0.0] * 4},
                ],
                "appears more than once",
            ),
        ],
    )
    def test_rejects_invalid_commands(self, commands, message):
        hardware = MagicMock()
        hardware.set_arm_control_mode.return_value = Result.ok()
        skill = MultiTimedCommandSkill(hardware)

        result = skill.initialize(MultiTimedCommandParams(commands=commands))

        assert not result.success
        assert message in result.message
        hardware.send_timed_multi_commands.assert_not_called()

    def test_accepts_planners_3_6_7(self):
        skill = MultiTimedCommandSkill(MagicMock())

        result = skill.initialize(
            MultiTimedCommandParams(
                commands=[
                    {"planner_index": 3, "cmd_vec": [0.0] * 4},
                    {"planner_index": 6, "cmd_vec": [0.0] * 6},
                    {"planner_index": 7, "cmd_vec": [0.0] * 6},
                ]
            )
        )

        assert result.success

    def test_does_not_wait_after_send_failure(self, monkeypatch):
        sleeps = []
        monkeypatch.setattr(time, "sleep", lambda seconds: sleeps.append(seconds))
        hardware = MagicMock()
        hardware.set_arm_control_mode.return_value = Result.ok()
        hardware.send_timed_multi_commands.return_value = Result.fail("busy")
        skill = MultiTimedCommandSkill(hardware)
        skill.initialize(
            MultiTimedCommandParams(
                commands=[{"planner_index": 3, "cmd_vec": [0.0] * 4}]
            )
        )

        result = skill.execute()

        assert not result.success
        assert sleeps == []
        assert skill.is_finished()

    @pytest.mark.parametrize("planner,size", [(0, 3), (1, 3), (2, 4), (3, 4),
                                               (4, 6), (5, 6), (6, 6), (7, 6),
                                               (8, 7), (9, 7)])
    def test_prepares_external_mode_only_for_torso_or_arm_planners(self, planner, size, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda _: None)
        hardware = MagicMock()
        hardware.set_arm_control_mode.return_value = Result.ok()
        hardware.set_focus_ee.return_value = Result.ok()
        hardware.set_focus_z.return_value = Result.ok()
        hardware.send_timed_multi_commands.return_value = Result.ok()
        skill = MultiTimedCommandSkill(hardware)
        params = MultiTimedCommandParams(commands=[{"planner_index": planner, "cmd_vec": [0.0] * size}])

        assert skill.initialize(params).success
        hardware.set_arm_control_mode.assert_not_called()
        assert skill.execute().success
        calls = [call[0] for call in hardware.mock_calls]
        if planner >= 2:
            hardware.set_arm_control_mode.assert_called_once_with(2)
            assert calls == ["set_arm_control_mode", "set_focus_ee", "set_focus_z",
                             "send_timed_multi_commands"]
        else:
            hardware.set_arm_control_mode.assert_not_called()
            assert calls == ["set_focus_ee", "set_focus_z", "send_timed_multi_commands"]
        assert skill.is_finished()
        assert skill.execute().success
        hardware.send_timed_multi_commands.assert_called_once()
        assert hardware.set_arm_control_mode.call_count == int(planner >= 2)

    def test_mode_failure_stops_before_focus_and_command_submission(self, monkeypatch):
        sleeps = []
        monkeypatch.setattr(time, "sleep", lambda seconds: sleeps.append(seconds))
        hardware = MagicMock()
        hardware.set_arm_control_mode.return_value = Result.fail("mode service unavailable")
        skill = MultiTimedCommandSkill(hardware)
        assert skill.initialize(MultiTimedCommandParams(
            commands=[{"planner_index": 2, "cmd_vec": [0.0] * 4},
                      {"planner_index": 6, "cmd_vec": [0.0] * 6}]
        )).success

        result = skill.execute()

        assert not result.success
        assert "set_arm_control_mode(2) failed: mode service unavailable" in result.message
        hardware.set_arm_control_mode.assert_called_once_with(2)
        hardware.set_focus_ee.assert_not_called()
        hardware.set_focus_z.assert_not_called()
        hardware.send_timed_multi_commands.assert_not_called()
        assert sleeps == []
        assert skill.is_finished()

    def test_rejects_missing_mode_interface(self):
        hardware = MagicMock(spec=["set_focus_ee", "set_focus_z", "send_timed_multi_commands"])
        skill = MultiTimedCommandSkill(hardware)
        assert skill.initialize(MultiTimedCommandParams(
            commands=[{"planner_index": 8, "cmd_vec": [0.0] * 7}]
        )).success

        result = skill.execute()

        assert not result.success
        assert "does not implement set_arm_control_mode()" in result.message
        hardware.set_focus_ee.assert_not_called()
        hardware.send_timed_multi_commands.assert_not_called()
        assert skill.is_finished()

    def test_can_disable_mode_preparation_with_external_management(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda _: None)
        hardware = MagicMock(spec=["set_focus_ee", "set_focus_z", "send_timed_multi_commands"])
        hardware.set_focus_ee.return_value = Result.ok()
        hardware.set_focus_z.return_value = Result.ok()
        hardware.send_timed_multi_commands.return_value = Result.ok()
        skill = MultiTimedCommandSkill(hardware)
        assert skill.initialize(MultiTimedCommandParams(
            commands=[{"planner_index": 6, "cmd_vec": [0.0] * 6}],
            prepare_arm_control=False,
        )).success

        assert skill.execute().success
        assert [call[0] for call in hardware.mock_calls] == [
            "set_focus_ee", "set_focus_z", "send_timed_multi_commands"]

    def test_reinitialization_prepares_mode_again(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda _: None)
        hardware = MagicMock()
        hardware.set_arm_control_mode.return_value = Result.ok()
        hardware.set_focus_ee.return_value = Result.ok()
        hardware.set_focus_z.return_value = Result.ok()
        hardware.send_timed_multi_commands.return_value = Result.ok()
        skill = MultiTimedCommandSkill(hardware)
        params = MultiTimedCommandParams(commands=[{"planner_index": 7, "cmd_vec": [0.0] * 6}])

        for _ in range(2):
            assert skill.initialize(params).success
            assert skill.execute().success

        assert [call.args for call in hardware.set_arm_control_mode.call_args_list] == [(2,), (2,)]
        assert hardware.send_timed_multi_commands.call_count == 2

    @pytest.mark.parametrize("value", [0, 1, "false", None])
    def test_rejects_non_boolean_mode_preparation(self, value):
        hardware = MagicMock()
        skill = MultiTimedCommandSkill(hardware)
        result = skill.initialize(MultiTimedCommandParams(
            commands=[{"planner_index": 6, "cmd_vec": [0.0] * 6}],
            prepare_arm_control=value,
        ))

        assert not result.success
        assert "prepare_arm_control must be a boolean" in result.message
        hardware.set_arm_control_mode.assert_not_called()
        hardware.send_timed_multi_commands.assert_not_called()
