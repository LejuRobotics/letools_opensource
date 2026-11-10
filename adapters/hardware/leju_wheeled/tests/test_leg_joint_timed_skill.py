# -*- coding: utf-8 -*-
"""leg_joint_timed 技能单元测试（mock IHardware，无需 ROS/真机）。

被测对象: skills.atomic.refactored_sdk.leg_joint_timed.LegJointTimedSkill
底层路径: on_execute → hardware.send_leg_joint_timed(joint_angles, desire_time)
         → TimedCmdManager（planner_index=3）

关键语义（区别于 leg_joint_sdk 的阻塞路径）:
- /mobile_manipulator_timed_single_cmd 服务规划完成即返回，不等待运动执行
- 技能需在发送成功后 sleep(actual_time + settle_time) 等待运动完成
"""

import time
from unittest.mock import MagicMock

import pytest

from skills.atomic.refactored_sdk.leg_joint_timed import (
    LegJointTimedParams,
    LegJointTimedSkill,
)


def _ok(msg="ok", data=None):
    from core.domain.result import Result
    return Result.ok(msg, data=data)


def _fail(msg="fail"):
    from core.domain.result import Result
    return Result.fail(msg)


_ANGLES = [29.00, -41.11, 12.21, 0.0]  # [knee, leg, waist_pitch, waist_yaw]，度


@pytest.mark.unit
class TestLegJointTimedSkill:

    def test_success_calls_hardware_with_desire_time(self, monkeypatch):
        """成功时按 timed 接口调用：joint_angles + desire_time（非 total_time）"""
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_leg_joint_timed.return_value = _ok("下发成功", data={"actual_time": 2.5})

        skill = LegJointTimedSkill(hardware=hw)
        params = LegJointTimedParams(joint_angles=_ANGLES, desire_time=2.5)
        assert skill.initialize(params).success

        result = skill.execute()
        assert result.success
        hw.send_leg_joint_timed.assert_called_once_with(
            joint_angles=_ANGLES, desire_time=2.5)
        assert skill.is_finished()

    def test_sleeps_actual_time_plus_settle_after_send(self, monkeypatch):
        """服务非阻塞：发送成功后需 sleep(actual_time + settle_time) 等待运动完成"""
        sleeps = []
        monkeypatch.setattr(time, "sleep", lambda s: sleeps.append(s))
        hw = MagicMock()
        hw.send_leg_joint_timed.return_value = _ok("下发成功", data={"actual_time": 2.5})

        skill = LegJointTimedSkill(hardware=hw)
        skill.initialize(LegJointTimedParams(
            joint_angles=_ANGLES, desire_time=2.0, settle_time=0.5))
        skill.execute()

        assert sleeps == [3.0]  # actual_time(2.5) + settle_time(0.5)

    def test_sleep_falls_back_to_desire_time_when_actual_missing(self, monkeypatch):
        """Result.data 无 actual_time 时回退 desire_time"""
        sleeps = []
        monkeypatch.setattr(time, "sleep", lambda s: sleeps.append(s))
        hw = MagicMock()
        hw.send_leg_joint_timed.return_value = _ok("下发成功")

        skill = LegJointTimedSkill(hardware=hw)
        skill.initialize(LegJointTimedParams(
            joint_angles=_ANGLES, desire_time=3.0, settle_time=0.5))
        skill.execute()

        assert sleeps == [3.5]  # desire_time(3.0) + settle_time(0.5)

    def test_no_sleep_on_send_failure(self, monkeypatch):
        """发送失败时不等待，直接返回失败"""
        sleeps = []
        monkeypatch.setattr(time, "sleep", lambda s: sleeps.append(s))
        hw = MagicMock()
        hw.send_leg_joint_timed.return_value = _fail("服务超时")

        skill = LegJointTimedSkill(hardware=hw)
        skill.initialize(LegJointTimedParams(joint_angles=_ANGLES, desire_time=2.0))
        result = skill.execute()

        assert not result.success
        assert sleeps == []
        assert skill.is_finished()

    def test_missing_hardware_method_fails(self):
        """硬件未实现 send_leg_joint_timed 时失败"""
        hw = MagicMock(spec=[])  # 无任何方法
        skill = LegJointTimedSkill(hardware=hw)
        skill.initialize(LegJointTimedParams(joint_angles=_ANGLES, desire_time=2.0))
        result = skill.execute()
        assert not result.success
        assert "send_leg_joint_timed" in result.message

    def test_invalid_joint_count_fails_init(self):
        """关节数 ≠ 4 时初始化失败"""
        hw = MagicMock()
        skill = LegJointTimedSkill(hardware=hw)
        result = skill.initialize(LegJointTimedParams(
            joint_angles=[1.0, 2.0], desire_time=2.0))
        assert not result.success
        hw.send_leg_joint_timed.assert_not_called()

    def test_execute_idempotent_after_done(self, monkeypatch):
        """完成后重复 execute 返回成功且不重复下发"""
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_leg_joint_timed.return_value = _ok("下发成功", data={"actual_time": 2.0})

        skill = LegJointTimedSkill(hardware=hw)
        skill.initialize(LegJointTimedParams(joint_angles=_ANGLES, desire_time=2.0))
        assert skill.execute().success
        assert skill.execute().success  # 二次调用幂等

        assert hw.send_leg_joint_timed.call_count == 1
