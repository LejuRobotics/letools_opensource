"""arm_ee_timed_cmd 技能单元测试（mock IHardware）：模式前置/后置与失败不静默。"""

import time
from unittest.mock import MagicMock

import pytest

from core.domain.result import Result
from skills.atomic.refactored_sdk.arm_ee_timed_cmd import (
    ArmEETimedCmdParams,
    ArmEETimedCmdSkill,
)


def _ok(msg="ok", data=None):
    return Result.ok(msg, data=data)


def _fail(msg="fail"):
    return Result.fail(msg)


def _params(**kwargs):
    base = dict(
        left_waypoints=[[1.0, 0.25, 1.2, 0, 0, 0]],
        right_waypoints=[[1.0, -0.25, 1.2, 0, 0, 0]],
        desire_time=3.0,
        frame="local",
    )
    base.update(kwargs)
    return ArmEETimedCmdParams(**base)


@pytest.fixture(autouse=True)
def _no_sleep(monkeypatch):
    monkeypatch.setattr(time, "sleep", lambda *_: None)


def _hardware(mode_result=None):
    hw = MagicMock()
    hw.send_arm_ee_local_timed.return_value = _ok()
    hw.set_arm_control_mode.return_value = _ok() if mode_result is None else mode_result
    return hw


@pytest.mark.unit
class TestArmEETimedCmdSkill:
    def test_prepare_mode_before_send_and_release_after(self):
        """发令前切模式 2、发令后切模式 0，且顺序正确。"""
        hw = _hardware()
        skill = ArmEETimedCmdSkill(hardware=hw)
        assert skill.initialize(_params()).success

        assert skill.execute().success

        calls = [c.args[0] for c in hw.set_arm_control_mode.call_args_list]
        assert calls == [2, 0]
        hw.send_arm_ee_local_timed.assert_called_once()

    def test_mode_not_drivable_fails_instead_of_silent_drop(self):
        """模式切不到 2 时必须失败返回，且不下发指令（避免服务假成功、手臂不动）。"""
        hw = _hardware(mode_result=_fail("无效的手臂控制模式: 9"))
        skill = ArmEETimedCmdSkill(hardware=hw)
        assert skill.initialize(_params()).success

        result = skill.execute()

        assert not result.success
        assert "set_arm_control_mode(2) failed" in result.message
        hw.send_arm_ee_local_timed.assert_not_called()
        assert skill.is_finished()

    def test_release_failure_propagates(self):
        """后置释放失败不应被吞掉。"""
        hw = _hardware()
        hw.set_arm_control_mode.side_effect = [_ok(), _fail("释放失败")]
        skill = ArmEETimedCmdSkill(hardware=hw)
        assert skill.initialize(_params()).success

        result = skill.execute()

        assert not result.success
        assert "set_arm_control_mode(0) failed" in result.message

    def test_prepare_disabled_skips_mode_switch(self):
        """prepare_arm_control=False 时不自作主张切模式（调用方已自行保证）。"""
        hw = _hardware()
        skill = ArmEETimedCmdSkill(hardware=hw)
        assert skill.initialize(_params(prepare_arm_control=False,
                                       release_arm_control=False)).success

        assert skill.execute().success

        hw.set_arm_control_mode.assert_not_called()

    def test_missing_mode_method_reports_not_implemented(self):
        """硬件未实现 set_arm_control_mode 时应显式报错，而非跳过。"""
        hw = _hardware()
        del hw.set_arm_control_mode
        skill = ArmEETimedCmdSkill(hardware=hw)
        assert skill.initialize(_params()).success

        result = skill.execute()

        assert not result.success
        assert "does not implement set_arm_control_mode" in result.message
        hw.send_arm_ee_local_timed.assert_not_called()

    def test_waypoint_count_mismatch_fails_init(self):
        hw = _hardware()
        skill = ArmEETimedCmdSkill(hardware=hw)
        params = _params(right_waypoints=[[1.0, -0.25, 1.2, 0, 0, 0]] * 2)
        assert not skill.initialize(params).success
