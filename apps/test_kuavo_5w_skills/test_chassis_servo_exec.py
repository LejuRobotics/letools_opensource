#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘伺服执行：技能层单测（使用伪硬件，无 ROS）。

飞书 §3.3 / §4.3 的「执行」模块验证：ServoCommand → base_move_relative_jibot。
用伪硬件记录调用参数，验证：
    - 正常指令 → 下发 dx/dy/dtheta + options
    - stop_reason 非空（终止）→ 跳过移动，返回 skipped=True
    - 硬约束 position_threshold=0.08 不可改大
    - 速度模式（DESIGN §4.3/§4.5）→ send_base_velocity_tick 单帧非阻塞；
      终止/失败路径末发零速；position 模式不碰速度通路

运行：
    python3 apps/test_kuavo_5w_skills/test_chassis_servo_exec.py
退出码 0 通过、1 失败。
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from core.domain.result import Result                            # noqa: E402
from core.domain.chassis_options import MoveToTargetOptions      # noqa: E402
from core.domain.servo_types import ServoCommand                 # noqa: E402
from skills.atomic.refactored_sdk.chassis_servo_exec import (    # noqa: E402
    ChassisServoExecParams,
    ChassisServoExecSkill,
)


class _FakeHardware:
    """伪硬件：记录 base_move_relative_jibot 调用参数，返回 task_id。

    速度模式还要记录 send_base_velocity_tick 的调用序列（含零速兜底），
    故用两个列表分别记录两条通路的调用；底盘原生通道另记 direct_ticks。
    """

    def __init__(self, task_id="task-123", success=True, message="ok",
                 tick_success=True, tick_message="tick ok",
                 direct_success=True, direct_message="direct tick ok"):
        self.task_id = task_id
        self.success = success
        self.message = message
        self.tick_success = tick_success
        self.tick_message = tick_message
        self.direct_success = direct_success
        self.direct_message = direct_message
        self.last_call = None  # (x, y, theta, options)
        self.ticks = []        # [(vx, vy, vyaw_rad), ...]
        self.direct_ticks = []  # [(vx, vy, wz, topic), ...]

    def base_move_relative_jibot(self, x, y, theta, options):
        self.last_call = (x, y, theta, options)
        if self.success:
            return Result.ok(self.message, data={"task_id": self.task_id})
        return Result.fail(self.message)

    def send_base_velocity_tick(self, vx, vy, vyaw_rad):
        self.ticks.append((vx, vy, vyaw_rad))
        if self.tick_success:
            return Result.ok(self.tick_message)
        return Result.fail(self.tick_message)

    def send_base_cmd_vel_tick(self, vx, vy, wz, topic="/move_base/base_cmd_vel"):
        self.direct_ticks.append((vx, vy, wz, topic))
        if self.direct_success:
            return Result.ok(self.direct_message)
        return Result.fail(self.direct_message)


def _vcmd(vx=0.03, vy=-0.01, wz=0.05, iter_index=0,
          cumulative=0.005, stop_reason=""):
    """速度模式 ServoCommand。"""
    return ServoCommand(
        mode="velocity", vx_mps=vx, vy_mps=vy, wz_radps=wz,
        iter_index=iter_index, cumulative_xy_m=cumulative, stop_reason=stop_reason,
    )


def _cmd(dx=0.005, dy=0.0, dtheta=0.0, iter_index=0,
         cumulative=0.005, stop_reason=""):
    return ServoCommand(
        dx_m=dx, dy_m=dy, dtheta_rad=dtheta,
        iter_index=iter_index, cumulative_xy_m=cumulative, stop_reason=stop_reason,
    )


def _params(cmd, **over):
    base = dict(
        command=cmd,
        position_threshold=0.08,
        linear_velocity=0.15, angular_velocity=0.25,
        allow_rotation=True, avoid_enabled=False, avoid_distance=0.5,
        angle_threshold=0.1, timeout=60.0,
    )
    base.update(over)
    return ChassisServoExecParams(**base)


def test_normal_command_dispatched():
    """正常 ServoCommand → 下发 dx/dy/dtheta + options。"""
    hw = _FakeHardware()
    skill = ChassisServoExecSkill(hardware=hw)
    skill.initialize(_params(_cmd(dx=0.006, dy=0.003, dtheta=0.01)))
    res = skill.execute()
    assert res.success
    x, y, theta, options = hw.last_call
    assert abs(x - 0.006) < 1e-9
    assert abs(y - 0.003) < 1e-9
    assert abs(theta - 0.01) < 1e-9
    assert isinstance(options, MoveToTargetOptions)
    assert abs(options.position_threshold - 0.08) < 1e-9
    assert res.data["task_id"] == "task-123"
    print("    正常指令正确下发到硬件")


def test_stop_reason_skips_move():
    """stop_reason 非空 → 跳过移动，skipped=True。"""
    for reason in ("converged", "cum_limit", "iter_limit", "guard"):
        hw = _FakeHardware()
        skill = ChassisServoExecSkill(hardware=hw)
        skill.initialize(_params(_cmd(stop_reason=reason)))
        res = skill.execute()
        assert res.success, f"reason={reason} 应成功"
        assert res.data["skipped"] is True, f"reason={reason} 应 skipped"
        assert res.data["stop_reason"] == reason
        assert hw.last_call is None, f"reason={reason} 不应下发移动"
    print("    所有终止原因都跳过移动")


def test_empty_command_rejected():
    """command=None → Result.fail。"""
    hw = _FakeHardware()
    skill = ChassisServoExecSkill(hardware=hw)
    skill.initialize(_params(None))
    res = skill.execute()
    assert not res.success
    assert hw.last_call is None
    print("    空指令被拒")


def test_hardware_failure_propagates():
    """硬件返回失败 → 透传。"""
    hw = _FakeHardware(success=False, message="ros down")
    skill = ChassisServoExecSkill(hardware=hw)
    skill.initialize(_params(_cmd()))
    res = skill.execute()
    assert not res.success
    assert "ros down" in res.message
    print("    硬件失败透传")


def test_position_threshold_hardcoded_constraint():
    """position_threshold 默认 0.08 且传给 options（飞书硬约束）。"""
    hw = _FakeHardware()
    skill = ChassisServoExecSkill(hardware=hw)
    skill.initialize(_params(_cmd()))  # 默认 position_threshold=0.08
    skill.execute()
    _, _, _, options = hw.last_call
    assert abs(options.position_threshold - 0.08) < 1e-9
    print("    position_threshold=0.08 硬约束传入")


def test_is_finished_and_idempotent():
    """execute 后 is_finished，重复 execute 幂等。"""
    hw = _FakeHardware()
    skill = ChassisServoExecSkill(hardware=hw)
    assert not skill.is_finished()
    skill.initialize(_params(_cmd()))
    r1 = skill.execute()
    assert skill.is_finished()
    r2 = skill.execute()
    assert r1 is r2
    print("    is_finished 置真 + execute 幂等")


def _run(cmd, hw, **over):
    """用给定伪硬件跑一次技能，返回 (skill, result)。"""
    skill = ChassisServoExecSkill(hardware=hw)
    skill.initialize(_params(cmd, **over))
    return skill, skill.execute()


# --- 速度模式（DESIGN §4.3 / §4.5） ---

def test_velocity_mode_dispatches_tick_once():
    """velocity 模式 → 调 send_base_velocity_tick 一次（非阻塞），量纲原样透传。"""
    hw = _FakeHardware()
    _, res = _run(_vcmd(vx=0.03, vy=-0.01, wz=0.05), hw)
    assert res.success
    assert len(hw.ticks) == 1, f"非阻塞应只发一次，得到 {len(hw.ticks)} 次"
    vx, vy, wz = hw.ticks[0]
    assert abs(vx - 0.03) < 1e-9 and abs(vy - (-0.01)) < 1e-9
    assert abs(wz - 0.05) < 1e-9, f"wz 应为 rad/s 原样透传，得到 {wz}"
    assert hw.last_call is None, "速度模式不应走 base_move_relative_jibot"
    assert res.data.get("velocity") is True
    print("    velocity 模式单次下发 tick，量纲原样透传")


def test_velocity_stop_reason_sends_zero_last():
    """velocity + stop_reason 非空 → 不发速度、末发为零速、成功 skipped。"""
    hw = _FakeHardware()
    _, res = _run(_vcmd(vx=0.03, stop_reason="converged"), hw)
    assert res.success and res.data.get("skipped") is True
    assert hw.ticks, "终止路径必须发一帧零速"
    assert hw.ticks[-1] == (0.0, 0.0, 0.0), f"末发应零速，得到 {hw.ticks[-1]}"
    assert hw.last_call is None
    print("    velocity 终止 → 末发零速")


def test_velocity_tick_failure_sends_zero_last():
    """velocity 下发失败（异常路径）→ 返回失败且末发为零速。"""
    hw = _FakeHardware(tick_success=False, tick_message="pub failed")
    _, res = _run(_vcmd(vx=0.03), hw)
    assert not res.success and "pub failed" in res.message
    assert hw.ticks, "失败路径必须发零速兜底"
    assert hw.ticks[-1] == (0.0, 0.0, 0.0), f"末发应零速，得到 {hw.ticks[-1]}"
    print("    velocity 下发失败 → 末发零速")


def test_velocity_zero_command_single_frame():
    """速度全零（正常指令，非终止）→ 仍只发一帧，不额外补零速。"""
    hw = _FakeHardware()
    _, res = _run(_vcmd(vx=0.0, vy=0.0, wz=0.0), hw)
    assert res.success
    assert hw.ticks == [(0.0, 0.0, 0.0)], f"应只发一帧，得到 {hw.ticks}"
    print("    速度全零指令单帧下发")


def test_position_mode_does_not_touch_velocity_tick():
    """position 模式 → 走 base_move_relative_jibot，完全不碰 send_base_velocity_tick。"""
    hw = _FakeHardware()
    _, res = _run(_cmd(dx=0.006, dy=0.003, dtheta=0.01), hw)
    assert res.success
    assert hw.last_call is not None, "位置模式应下发 base_move_relative_jibot"
    assert hw.ticks == [], f"位置模式不应碰速度通路，得到 {hw.ticks}"
    print("    position 模式不碰速度通路")


# --- 速度模式通道分派（command_channel，父议题 Q7/Q16） ---

def test_chassis_direct_dispatches_tick_once():
    """chassis_direct → 只调 send_base_cmd_vel_tick 一次，topic 等于参数值。"""
    hw = _FakeHardware()
    _, res = _run(_vcmd(vx=0.03, vy=-0.01, wz=0.05), hw,
                  command_channel="chassis_direct",
                  cmd_vel_topic="/test/base_cmd_vel")
    assert res.success
    assert hw.ticks == [], f"chassis_direct 不应走 humanoid 通路，得到 {hw.ticks}"
    assert len(hw.direct_ticks) == 1, f"应只发一次，得到 {hw.direct_ticks}"
    vx, vy, wz, topic = hw.direct_ticks[0]
    assert abs(vx - 0.03) < 1e-9 and abs(vy - (-0.01)) < 1e-9
    assert abs(wz - 0.05) < 1e-9
    assert topic == "/test/base_cmd_vel", f"topic 应透传参数值，得到 {topic}"
    assert hw.last_call is None
    assert res.data.get("channel") == "chassis_direct"
    print("    chassis_direct 单次下发 cmd_vel tick，topic 透传")


def test_chassis_direct_stop_reason_skips_move():
    """chassis_direct + stop_reason → 不下发移动，零速走当前通道，skipped。"""
    hw = _FakeHardware()
    _, res = _run(_vcmd(vx=0.03, stop_reason="converged"), hw,
                  command_channel="chassis_direct",
                  cmd_vel_topic="/test/base_cmd_vel")
    assert res.success and res.data.get("skipped") is True
    assert res.data.get("stop_reason") == "converged"
    assert hw.ticks == [], "不应串到 humanoid 通路"
    assert hw.direct_ticks == [(0.0, 0.0, 0.0, "/test/base_cmd_vel")], \
        f"只应发零速，得到 {hw.direct_ticks}"
    assert hw.last_call is None, "不应下发移动"
    print("    chassis_direct 终止 → 不下发移动、零速走当前通道")


def test_chassis_direct_failure_sends_zero_direct():
    """chassis_direct 下发失败 → 返回失败且零速走当前通道。"""
    hw = _FakeHardware(direct_success=False, direct_message="pub failed")
    _, res = _run(_vcmd(vx=0.03), hw,
                  command_channel="chassis_direct",
                  cmd_vel_topic="/test/base_cmd_vel")
    assert not res.success and "pub failed" in res.message
    assert hw.ticks == [], "异常路径不应串到 humanoid 通路"
    assert hw.direct_ticks[-1] == (0.0, 0.0, 0.0, "/test/base_cmd_vel"), \
        f"末发应零速走当前通道，得到 {hw.direct_ticks[-1]}"
    print("    chassis_direct 下发失败 → 零速走当前通道")


def test_humanoid_default_only_calls_velocity_tick():
    """默认 command_channel=humanoid → 只调 send_base_velocity_tick，从不调新方法。"""
    hw = _FakeHardware()
    _, res = _run(_vcmd(vx=0.03, vy=-0.01, wz=0.05), hw)
    assert res.success
    assert len(hw.ticks) == 1
    assert hw.direct_ticks == [], f"humanoid 不应调底盘原生通道，得到 {hw.direct_ticks}"
    print("    humanoid 默认路径只走 send_base_velocity_tick")


def test_unknown_channel_rejected_before_publish():
    """未知 command_channel → initialise 直接失败，一帧都不发（L-2）。"""
    hw = _FakeHardware()
    skill = ChassisServoExecSkill(hardware=hw)
    init = skill.initialize(_params(_vcmd(vx=0.03), command_channel="bogus"))
    assert not init.success
    assert init.error_code == "INVALID_COMMAND_CHANNEL"
    assert hw.ticks == [], f"未知通道不得发 /cmd_vel，得到 {hw.ticks}"
    assert hw.direct_ticks == [], f"未知通道不得发 base_cmd_vel，得到 {hw.direct_ticks}"
    assert hw.last_call is None, "未知通道不得下发移动"
    print("    未知通道在 initialise 阶段被拒，零帧下发")


def main() -> int:
    cases = [
        ("正常指令下发", test_normal_command_dispatched),
        ("终止原因跳过", test_stop_reason_skips_move),
        ("空指令被拒", test_empty_command_rejected),
        ("硬件失败透传", test_hardware_failure_propagates),
        ("position_threshold 硬约束", test_position_threshold_hardcoded_constraint),
        ("幂等与完成", test_is_finished_and_idempotent),
        ("velocity 单次下发", test_velocity_mode_dispatches_tick_once),
        ("velocity 终止零速", test_velocity_stop_reason_sends_zero_last),
        ("velocity 失败零速", test_velocity_tick_failure_sends_zero_last),
        ("velocity 全零单帧", test_velocity_zero_command_single_frame),
        ("position 不碰速度通路", test_position_mode_does_not_touch_velocity_tick),
        ("chassis_direct 单次下发", test_chassis_direct_dispatches_tick_once),
        ("chassis_direct 终止跳过", test_chassis_direct_stop_reason_skips_move),
        ("chassis_direct 失败零速", test_chassis_direct_failure_sends_zero_direct),
        ("humanoid 默认只走旧方法", test_humanoid_default_only_calls_velocity_tick),
        ("未知通道 initialise 即拒", test_unknown_channel_rejected_before_publish),
    ]
    failed = 0
    for title, fn in cases:
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {title}: {exc}")
        except Exception as exc:                       # noqa: BLE001
            failed += 1
            print(f"  ERROR {title}: {type(exc).__name__}: {exc}")

    print()
    if failed:
        print(f"FAILED: {failed}/{len(cases)}")
        return 1
    print(f"OK: {len(cases)}/{len(cases)} 全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
