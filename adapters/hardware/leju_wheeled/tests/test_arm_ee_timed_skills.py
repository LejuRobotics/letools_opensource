# -*- coding: utf-8 -*-
"""arm_ee_single_timed / arm_ee_burst_timed / arm_ee_offline_traj 技能单元测试（mock IHardware）。"""

import time
from unittest.mock import MagicMock

import pytest

from skills.atomic.refactored_sdk.arm_ee_single_timed import (
    ArmEESingleTimedParams,
    ArmEESingleTimedSkill,
)
from skills.atomic.refactored_sdk.arm_ee_burst_timed import (
    ArmEEBurstTimedParams,
    ArmEEBurstTimedSkill,
)
from skills.atomic.refactored_sdk.arm_ee_offline_traj import (
    ArmEEOfflineTrajParams,
    ArmEEOfflineTrajSkill,
)


def _ok(msg="ok", data=None):
    from core.domain.result import Result
    return Result.ok(msg, data=data)


def _fail(msg="fail"):
    from core.domain.result import Result
    return Result.fail(msg)


# ---------------------------------------------------------------------------
# ArmEESingleTimedSkill
# ---------------------------------------------------------------------------
@pytest.mark.unit
class TestArmEESingleTimedSkill:
    def test_success_calls_correct_hardware_method(self):
        hw = MagicMock()
        hw.send_timed_left_arm_ee_world.return_value = _ok("下发成功")
        skill = ArmEESingleTimedSkill(hardware=hw)
        params = ArmEESingleTimedParams(side="left", frame="world",
                                        pose=[0.3, 0.25, 0.5, 0, 0, 0], desire_time=3.0)
        assert skill.initialize(params).success
        result = skill.execute()
        assert result.success
        hw.send_timed_left_arm_ee_world.assert_called_once_with(
            pose=[0.3, 0.25, 0.5, 0, 0, 0], desire_time=3.0)
        assert skill.is_finished()

    def test_right_local_uses_right_local_method(self):
        hw = MagicMock()
        hw.send_timed_right_arm_ee_local.return_value = _ok()
        skill = ArmEESingleTimedSkill(hardware=hw)
        params = ArmEESingleTimedParams(side="right", frame="local",
                                        pose=[0.2, -0.3, 0.6, 0, -45, 0], desire_time=2.0)
        assert skill.initialize(params).success
        skill.execute()
        hw.send_timed_right_arm_ee_local.assert_called_once()

    def test_invalid_pose_length_fails_init(self):
        hw = MagicMock()
        skill = ArmEESingleTimedSkill(hardware=hw)
        params = ArmEESingleTimedParams(pose=[0.3, 0.25])  # 只有2个
        assert not skill.initialize(params).success

    def test_invalid_side_fails_init(self):
        hw = MagicMock()
        skill = ArmEESingleTimedSkill(hardware=hw)
        params = ArmEESingleTimedParams(side="middle")
        assert not skill.initialize(params).success

    def test_missing_hardware_method_fails(self):
        hw = MagicMock(spec=[])  # 无任何方法
        skill = ArmEESingleTimedSkill(hardware=hw)
        params = ArmEESingleTimedParams()
        assert skill.initialize(params).success
        result = skill.execute()
        assert not result.success


# ---------------------------------------------------------------------------
# ArmEEBurstTimedSkill
# ---------------------------------------------------------------------------
@pytest.mark.unit
class TestArmEEBurstTimedSkill:
    def test_burst_sends_each_waypoint(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_timed_left_arm_ee_world.return_value = _ok()
        skill = ArmEEBurstTimedSkill(hardware=hw)
        wps = [[0.3, 0.25, 0.5, 0, 0, 0], [0.5, 0.25, 0.5, 0, 0, 0], [0.3, 0.25, 0.7, 0, 0, 0]]
        params = ArmEEBurstTimedParams(side="left", frame="world", waypoints=wps, desire_time=2.0)
        assert skill.initialize(params).success
        result = skill.execute()
        assert result.success
        assert hw.send_timed_left_arm_ee_world.call_count == 3

    def test_burst_stops_on_failure(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_timed_left_arm_ee_world.side_effect = [_ok(), _fail("下发失败"), _ok()]
        skill = ArmEEBurstTimedSkill(hardware=hw)
        wps = [[0.3, 0.25, 0.5, 0, 0, 0]] * 3
        params = ArmEEBurstTimedParams(waypoints=wps)
        assert skill.initialize(params).success
        result = skill.execute()
        assert not result.success
        assert hw.send_timed_left_arm_ee_world.call_count == 2  # 第2点失败即中止

    def test_check_reach_within_tolerance(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_timed_left_arm_ee_world.return_value = _ok()
        hw.get_ee_pose_reach_error.return_value = _ok(
            "到位", data={"err_vector": [0.001, 0.001, 0.001, 0.01, 0.01, 0.01]})
        skill = ArmEEBurstTimedSkill(hardware=hw)
        params = ArmEEBurstTimedParams(waypoints=[[0.3, 0.25, 0.5, 0, 0, 0]], check_reach=True)
        assert skill.initialize(params).success
        assert skill.execute().success
        hw.get_ee_pose_reach_error.assert_called_once_with(is_left=True)

    def test_check_reach_out_of_tolerance_fails(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_timed_left_arm_ee_world.return_value = _ok()
        hw.get_ee_pose_reach_error.return_value = _ok(
            "未到位", data={"err_vector": [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]})  # 线误差0.5m 超差
        skill = ArmEEBurstTimedSkill(hardware=hw)
        params = ArmEEBurstTimedParams(waypoints=[[0.3, 0.25, 0.5, 0, 0, 0]], check_reach=True)
        assert skill.initialize(params).success
        assert not skill.execute().success

    def test_empty_waypoints_fails_init(self):
        hw = MagicMock()
        skill = ArmEEBurstTimedSkill(hardware=hw)
        params = ArmEEBurstTimedParams(waypoints=[])
        assert not skill.initialize(params).success


# ---------------------------------------------------------------------------
# ArmEEOfflineTrajSkill
# ---------------------------------------------------------------------------
@pytest.mark.unit
class TestArmEEOfflineTrajSkill:
    def test_offline_sequence_enable_set_sleep_disable(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.enable_offline_trajectory.return_value = _ok()
        hw.set_offline_trajectory.return_value = _ok()
        skill = ArmEEOfflineTrajSkill(hardware=hw)
        params = ArmEEOfflineTrajParams(
            side="left", frame="world",
            traj=[[0.3, 0.25, 0.5, 0, 0, 0], [0.5, 0.25, 0.5, 0, 0, 0]],
            times=[0.0, 2.0])
        assert skill.initialize(params).success
        result = skill.execute()
        assert result.success
        # enable(True) → set → enable(False)
        assert hw.enable_offline_trajectory.call_args_list[0][0][0] is True
        assert hw.enable_offline_trajectory.call_args_list[-1][0][0] is False
        # 校验提交的轨迹结构
        sent = hw.set_offline_trajectory.call_args[0][0]
        assert sent[0]['planner_index'] == 0  # 左臂
        assert sent[0]['frame'] == 0          # 世界系
        assert len(sent[0]['timed_traj']) == 2

    def test_right_local_maps_planner1_frame1(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.enable_offline_trajectory.return_value = _ok()
        hw.set_offline_trajectory.return_value = _ok()
        skill = ArmEEOfflineTrajSkill(hardware=hw)
        params = ArmEEOfflineTrajParams(
            side="right", frame="local",
            traj=[[0.3, -0.25, 0.5, 0, 0, 0], [0.5, -0.25, 0.5, 0, 0, 0]],
            times=[0.0, 2.0])
        skill.initialize(params)
        skill.execute()
        sent = hw.set_offline_trajectory.call_args[0][0]
        assert sent[0]['planner_index'] == 1  # 右臂
        assert sent[0]['frame'] == 1          # 局部系

    def test_set_failure_rolls_back_enable(self, monkeypatch):
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.enable_offline_trajectory.return_value = _ok()
        hw.set_offline_trajectory.return_value = _fail("提交失败")
        skill = ArmEEOfflineTrajSkill(hardware=hw)
        params = ArmEEOfflineTrajParams(
            traj=[[0.3, 0.25, 0.5, 0, 0, 0], [0.5, 0.25, 0.5, 0, 0, 0]], times=[0.0, 2.0])
        skill.initialize(params)
        result = skill.execute()
        assert not result.success
        # 失败后应回滚 enable(False)
        assert hw.enable_offline_trajectory.call_args_list[-1][0][0] is False

    def test_first_time_not_zero_fails_init(self):
        hw = MagicMock()
        skill = ArmEEOfflineTrajSkill(hardware=hw)
        params = ArmEEOfflineTrajParams(
            traj=[[0.3, 0.25, 0.5, 0, 0, 0], [0.5, 0.25, 0.5, 0, 0, 0]], times=[1.0, 2.0])
        assert not skill.initialize(params).success

    def test_non_increasing_times_fails_init(self):
        hw = MagicMock()
        skill = ArmEEOfflineTrajSkill(hardware=hw)
        params = ArmEEOfflineTrajParams(
            traj=[[0.3, 0.25, 0.5, 0, 0, 0]] * 3, times=[0.0, 2.0, 1.0])
        assert not skill.initialize(params).success

    def test_traj_times_length_mismatch_fails_init(self):
        hw = MagicMock()
        skill = ArmEEOfflineTrajSkill(hardware=hw)
        params = ArmEEOfflineTrajParams(
            traj=[[0.3, 0.25, 0.5, 0, 0, 0]] * 3, times=[0.0, 2.0])
        assert not skill.initialize(params).success
