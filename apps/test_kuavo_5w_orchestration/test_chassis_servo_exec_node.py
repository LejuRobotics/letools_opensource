#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘伺服执行节点测试（编排层，硬件节点，伪硬件注入）。

被测对象: orchestration.nodes.node_chassis_servo_exec.NodeChassisServoExec
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE/RUNNING
底层路径: 节点 get_shared_hardware() → ChassisServoExecSkill
         → hardware.base_move_relative_jibot()（jibot/chassis_mixin，ROS 服务）

测试策略（模块单独测试，不依赖真机 / kuavo_msgs 在线）:
节点 update() 调 get_shared_hardware() 取硬件单例后构造 ChassisServoExecSkill。
本测试 monkeypatch 节点模块的 get_shared_hardware，注入伪硬件记录
base_move_relative_jibot 调用参数，从而：
  - 不需要 kuavo_msgs / kuavo_humanoid_sdk / 真机 ROS 服务
  - 仍完整验证 节点 读黑板 ServoCommand → 构造 skill → 下发 / 跳过 → 写回
    task_id 的接线

测试用例说明:
- test_dry_run: STUDIO_DRY_RUN=1 时 update() 直接返 SUCCESS
- test_wait_for_command: 黑板无 ServoCommand 时持续 RUNNING
- test_invalid_command_failure: cmd 不是 ServoCommand → FAILURE
- test_normal_dispatch: 正常指令 → 下发 base_move_relative_jibot → RUNNING
- test_skip_with_stop_reason: stop_reason=converged → 跳过移动 → SUCCESS
- test_skip_cum_limit_failure: stop_reason=cum_limit → 未对准 → FAILURE
- test_velocity_stop_converged_success: velocity + stop_reason=converged → SUCCESS
- test_velocity_stop_cum_limit_failure: velocity + stop_reason=cum_limit → FAILURE
- test_dedup_same_command: 同一指令对象不重复下发（RUNNING）
- test_velocity_dispatch_idle_true: velocity 模式单帧下发 + servo_exec_idle 恒 True
- test_velocity_obs_watchdog_zero: 观测超时 → 末发零速 → FAILURE
- test_velocity_duration_limit_zero: 时长超限 → 末发零速 → FAILURE
- test_velocity_tick_failure_zero: 下发失败（异常路径）→ 末发零速 → FAILURE
- test_velocity_terminate_sends_zero: terminate() → 零速已发
- test_direct_stream_started_once_setpoint_each_tick: chassis_direct 起流一次、每 tick 更新 setpoint、idle 恒 True
- test_direct_gate_absent_does_not_block: 从未收到 BaseCmdVelStatus → 门控待命，不阻断
- test_direct_gate_five_frames_then_stop: 连续 5 帧无效 → 置零停流 FAILURE（第 4 帧不停、闩锁不恢复）
- test_direct_stop_sequence_order: 退出时序 setpoint(0,0,0) → stop_cmd_vel_stream
- test_direct_terminate_stops_stream_once: terminate() 停流且幂等
- test_direct_start_failure_no_leak: 起流失败仍走停流清理
- test_humanoid_channel_never_touches_stream: humanoid 通道从不碰流方法
- test_position_does_not_touch_velocity: position 模式不碰 send_base_velocity_tick
- test_position_terminate_no_velocity: position 模式 terminate() 不发零速（行为不变）

运行:
    python3 apps/test_kuavo_5w_orchestration/test_chassis_servo_exec_node.py
退出码 0 通过、1 失败。
"""
import sys
import time
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "apps"))

from core.common.logger import init_logging, get_logger
init_logging()

import py_trees
from py_trees.blackboard import Client
from py_trees.common import Access, Status

from core.domain.result import Result
from core.domain.chassis_options import MoveToTargetOptions
from core.domain.servo_types import ServoCommand
from orchestration.nodes.node_chassis_servo_exec import NodeChassisServoExec
import orchestration.nodes.node_chassis_servo_exec as exec_mod
from _blackboard_isolation import reset_blackboard

logger = get_logger(__name__)

CMD_KEY = "latest_servo_command"
TASK_KEY = "current_task_id"
IDLE_KEY = "servo_exec_idle"
VER_KEY = "latest_servo_error_version"


class _FakeHardware:
    """伪硬件：记录两条通路的调用参数，返回 task_id / 速度发布结果。

    `events` 是**有序**调用流水，用于断言 chassis_direct 的停车时序
    （setpoint 置零 → stop_cmd_vel_stream）；`gate` 为 None 表示「从未收到
    BaseCmdVelStatus」（Q23A：门控待命）。
    """

    def __init__(self, task_id="task-xyz", success=True, message="ok",
                 tick_success=True, tick_message="tick ok", gate=None,
                 start_success=True, stop_fail_times=0):
        self.task_id = task_id
        self.success = success
        self.message = message
        self.tick_success = tick_success
        self.tick_message = tick_message
        self.gate = gate
        self.start_success = start_success
        self.stop_fail_times = stop_fail_times
        self.calls = []   # [(x, y, theta, options), ...]
        self.ticks = []   # [(vx, vy, vyaw_rad), ...]
        # 有序流水：[("start", topic, rate_hz) | ("setpoint", vx, vy, wz) | ("stop",)]
        self.events = []

    def base_move_relative_jibot(self, x, y, theta, options):
        self.calls.append((x, y, theta, options))
        if self.success:
            return Result.ok(self.message, data={"task_id": self.task_id})
        return Result.fail(self.message)

    def send_base_velocity_tick(self, vx, vy, vyaw_rad):
        self.ticks.append((vx, vy, vyaw_rad))
        if self.tick_success:
            return Result.ok(self.tick_message)
        return Result.fail(self.tick_message)

    # --- chassis_direct 通道（Q7/Q8） ---

    def start_cmd_vel_stream(self, topic="/move_base/base_cmd_vel", rate_hz=100.0):
        self.events.append(("start", topic, rate_hz))
        if not self.start_success:
            return Result.fail("start failed")
        return Result.ok("cmd_vel stream started",
                         data={"topic": topic, "rate_hz": rate_hz})

    def update_cmd_vel_setpoint(self, vx, vy, wz):
        self.events.append(("setpoint", float(vx), float(vy), float(wz)))
        return Result.ok("cmd_vel setpoint updated")

    def stop_cmd_vel_stream(self, zero_hold_s=0.2):
        self.events.append(("stop",))
        if self.stop_fail_times > 0:
            self.stop_fail_times -= 1
            return Result.fail("stop failed")
        return Result.ok("cmd_vel stream stopped")

    def get_motion_gate(self):
        if self.gate is None:
            return {"cmd_vel_effective": True, "reason": 0,
                    "description": "未收到状态，按有效处理", "effective_age_s": None}
        return dict(self.gate)


def _cmd(dx=0.005, dy=0.0, dtheta=0.0, iter_index=0,
         cumulative=0.005, stop_reason=""):
    return ServoCommand(
        dx_m=dx, dy_m=dy, dtheta_rad=dtheta,
        iter_index=iter_index, cumulative_xy_m=cumulative, stop_reason=stop_reason,
    )


def _vcmd(vx=0.03, vy=0.0, wz=0.0, iter_index=0,
          cumulative=0.005, stop_reason=""):
    """速度模式 ServoCommand。"""
    return ServoCommand(
        mode="velocity", vx_mps=vx, vy_mps=vy, wz_radps=wz,
        iter_index=iter_index, cumulative_xy_m=cumulative, stop_reason=stop_reason,
    )


def _make_node(obs_timeout_s=0.5, velocity_timeout_s=10.0,
               command_channel="humanoid",
               cmd_vel_topic="/move_base/base_cmd_vel", publish_rate_hz=100.0):
    """构造执行节点。"""
    return NodeChassisServoExec(
        "t_servo_exec", "底盘伺服执行", "ns",
        {"command_key": CMD_KEY, "task_id_key": TASK_KEY,
         "position_threshold": 0.08,
         "linear_velocity": 0.15, "angular_velocity": 0.25,
         "allow_rotation": True,
         "obs_timeout_s": obs_timeout_s,
         "velocity_timeout_s": velocity_timeout_s,
         "command_channel": command_channel,
         "cmd_vel_topic": cmd_vel_topic,
         "publish_rate_hz": publish_rate_hz},
    )


def _make_writer():
    """writer client：模拟控制器节点写 ServoCommand。"""
    w = Client(name="test_writer")
    w.register_key(key=CMD_KEY, access=Access.WRITE)
    return w


def _make_reader():
    """reader client：读节点写回的 task_id。"""
    r = Client(name="test_reader")
    r.register_key(key=TASK_KEY, access=Access.READ)
    return r


def _patch_hw(fake_hw):
    """monkeypatch 节点模块的 get_shared_hardware 返回伪硬件。"""
    orig = exec_mod.get_shared_hardware
    exec_mod.get_shared_hardware = lambda: fake_hw
    return orig


def _restore(orig):
    exec_mod.get_shared_hardware = orig


def _write_cmd(writer, cmd):
    writer.set(CMD_KEY, cmd)


def _read_task_id(reader):
    try:
        return getattr(reader, TASK_KEY)
    except KeyError:
        return None


def test_dry_run():
    """STUDIO_DRY_RUN=1 时 update() 直接返 SUCCESS（不碰 hardware）。

    _DRY_RUN 是模块级常量，monkeypatch 模块属性即可（reload 会破坏
    零参数 super() 的 __class__ cell 绑定）。
    """
    orig = exec_mod._DRY_RUN
    exec_mod._DRY_RUN = True
    try:
        node = _make_node()
        node.initialise()
        status = node.update()
        assert status == Status.SUCCESS, f"dry-run 应 SUCCESS，得到 {status}"
        logger.info("    dry_run 短路 SUCCESS")
    finally:
        exec_mod._DRY_RUN = orig


def test_wait_for_command():
    """黑板无 ServoCommand 时持续 RUNNING（不碰 hardware）。"""
    node = _make_node()
    node.initialise()
    status = node.update()
    assert status == Status.RUNNING, f"无指令应 RUNNING，得到 {status}"
    logger.info("    无指令时持续 RUNNING")


def test_invalid_command_failure():
    """cmd 不是 ServoCommand → FAILURE（不碰 hardware）。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    _write_cmd(writer, "not_a_command")  # 字符串，不是 ServoCommand
    status = node.update()
    assert status == Status.FAILURE, f"非 ServoCommand 应 FAILURE，得到 {status}"
    logger.info("    非 ServoCommand 拒收 FAILURE")


def test_normal_dispatch():
    """正常指令 → 下发 base_move_relative_jibot → RUNNING，task_id 写回。"""
    fake_hw = _FakeHardware(task_id="task-abc")
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        reader = _make_reader()
        _write_cmd(writer, _cmd(dx=0.006, dy=0.003, dtheta=0.01))
        status = node.update()
        assert status == Status.RUNNING, f"正常指令应 RUNNING，得到 {status}"
        # 伪硬件应被调用一次
        assert len(fake_hw.calls) == 1, f"应调用 1 次，得到 {len(fake_hw.calls)}"
        x, y, theta, options = fake_hw.calls[0]
        assert abs(x - 0.006) < 1e-9
        assert abs(y - 0.003) < 1e-9
        assert abs(theta - 0.01) < 1e-9
        assert isinstance(options, MoveToTargetOptions)
        assert abs(options.position_threshold - 0.08) < 1e-9
        # task_id 写回黑板
        assert _read_task_id(reader) == "task-abc"
        logger.info("    正常指令下发 + task_id 写回正确")
    finally:
        _restore(orig)


def test_skip_with_stop_reason():
    """stop_reason=converged → 跳过移动 → SUCCESS，不调 base_move_relative_jibot。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        reader = _make_reader()
        _write_cmd(writer, _cmd(stop_reason="converged"))
        status = node.update()
        assert status == Status.SUCCESS, f"终止指令应 SUCCESS，得到 {status}"
        # 伪硬件不应被调用
        assert fake_hw.calls == [], "skipped 时不应调用 base_move_relative_jibot"
        # task_id 应为空（未下发移动）
        assert _read_task_id(reader) == "", f"skipped 时 task_id 应空"
        logger.info("    stop_reason 非空 → 跳过移动 SUCCESS")
    finally:
        _restore(orig)


def test_skip_cum_limit_failure():
    """stop_reason=cum_limit → 跳过移动但明确 FAILURE（未对准）。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _cmd(stop_reason="cum_limit"))
        status = node.update()
        assert status == Status.FAILURE, f"累积量超限应 FAILURE，得到 {status}"
        assert fake_hw.calls == [], "skipped 时不应调用 base_move_relative_jibot"
        logger.info("    stop_reason=cum_limit → FAILURE")
    finally:
        _restore(orig)


def test_velocity_stop_converged_success():
    """velocity + stop_reason=converged → 发零速 → SUCCESS。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(stop_reason="converged"))
        status = node.update()
        assert status == Status.SUCCESS, f"收敛终止应 SUCCESS，得到 {status}"
        assert fake_hw.ticks[-1] == (0.0, 0.0, 0.0), "收敛终止应发零速"
        logger.info("    velocity stop_reason=converged → SUCCESS")
    finally:
        _restore(orig)


def test_velocity_stop_cum_limit_failure():
    """velocity + stop_reason=cum_limit → 发零速但 FAILURE（未对准）。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(stop_reason="cum_limit"))
        status = node.update()
        assert status == Status.FAILURE, f"累积量超限应 FAILURE，得到 {status}"
        assert fake_hw.ticks[-1] == (0.0, 0.0, 0.0), "终止时应发零速"
        logger.info("    velocity stop_reason=cum_limit → FAILURE")
    finally:
        _restore(orig)


def test_dedup_same_command():
    """同一指令对象不重复下发（第二个 tick RUNNING，不重复调用）。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        cmd = _cmd(dx=0.006)
        _write_cmd(writer, cmd)
        node.update()  # 首次下发
        assert len(fake_hw.calls) == 1
        # 同一对象再 update（不换新指令）
        status = node.update()
        assert status == Status.RUNNING
        assert len(fake_hw.calls) == 1, "同一指令不应重复下发"
        logger.info("    同一指令对象去重（不重复下发）")
    finally:
        _restore(orig)


def _make_idle_reader():
    """reader client：读节点写回的 servo_exec_idle。"""
    r = Client(name="test_idle_reader")
    r.register_key(key=IDLE_KEY, access=Access.READ)
    return r


def _read_idle(reader):
    try:
        return bool(getattr(reader, IDLE_KEY))
    except KeyError:
        return None


def _make_version_writer():
    """writer client：模拟视觉模块推进观测版本号（看门狗刷新信号）。"""
    w = Client(name="test_version_writer")
    w.register_key(key=VER_KEY, access=Access.WRITE)
    w.set(VER_KEY, 1)
    return w


def _count_nonzero_ticks(fake_hw):
    """统计非零速度 tick 的次数（零速兜底不计）。"""
    return sum(1 for vx, vy, wz in fake_hw.ticks if vx != 0.0 or vy != 0.0 or wz != 0.0)


def test_velocity_watchdog_latch_no_republish():
    """C1 回归：看门狗 FAILURE 之后的连续 tick 不得再发非零速度（闩锁）。

    复现自相抵消：失败路径若清 `_vel_since`，下一 tick 会以「首次进入」姿态重发
    同一条非零速度，看门狗/时长上限双重防线全废。修复后状态保留，elapsed/stale 只增。
    """
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(obs_timeout_s=0.5)
        node.initialise()
        writer = _make_writer()
        ver_writer = _make_version_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))

        # tick1：正常发非零速度
        assert node.update() == Status.RUNNING
        assert _count_nonzero_ticks(fake_hw) == 1, "首帧应发一次非零速度"

        # 观测版本停滞：回拨「上次观测更新时刻」越过 obs_timeout_s
        node._vel_last_obs_time = time.monotonic() - 1.0
        ver_writer.set(VER_KEY, 1)   # 版本不变
        assert node.update() == Status.FAILURE, "观测超时应 FAILURE"
        assert fake_hw.ticks[-1] == (0.0, 0.0, 0.0), "超时末发应零速"

        baseline = _count_nonzero_ticks(fake_hw)
        # 继续 tick 若干次，观测版本始终不推进 → 不得再发任何非零速度
        for _ in range(5):
            status = node.update()
            assert status == Status.FAILURE, \
                f"闩锁后应持续 FAILURE，得到 {status}"
        assert _count_nonzero_ticks(fake_hw) == baseline, \
            f"看门狗闩锁后不应再发非零速度：基线 {baseline}，现有 {_count_nonzero_ticks(fake_hw)}"
        assert fake_hw.ticks[-1] == (0.0, 0.0, 0.0), "闩锁期间末发应始终零速"

        # 观测恢复：版本推进 → 重新允许非零速度（证明不是永久锁死）
        ver_writer.set(VER_KEY, 2)
        status = node.update()
        assert status == Status.RUNNING, f"观测恢复后应 RUNNING，得到 {status}"
        assert _count_nonzero_ticks(fake_hw) == baseline + 1, \
            "观测恢复后应能恢复非零速度发布"
        logger.info("    看门狗闩锁：失败后不再重发非零速度，观测恢复后恢复")
    finally:
        _restore(orig)


def test_velocity_dispatch_idle_true():
    """velocity 模式 → 单帧下发 tick + servo_exec_idle 恒 True（无在途概念）。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        idle_reader = _make_idle_reader()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        status = node.update()
        assert status == Status.RUNNING, f"速度模式应 RUNNING，得到 {status}"
        assert fake_hw.calls == [], "速度模式不应走 base_move_relative_jibot"
        assert fake_hw.ticks, "速度模式应下发 send_base_velocity_tick"
        assert fake_hw.ticks[-1][0] != 0.0 or fake_hw.ticks[-1][2] != 0.0
        assert _read_idle(idle_reader) is True, "速度模式 servo_exec_idle 应恒 True"
        logger.info("    velocity 单帧下发 + idle 恒 True")
    finally:
        _restore(orig)


def test_velocity_obs_watchdog_zero():
    """观测看门狗超时（版本不更新）→ 末发零速 + FAILURE。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(obs_timeout_s=0.5)
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        assert node.update() == Status.RUNNING
        # 模拟 1s 没有新观测：把「上次观测更新时刻」回拨超过看门狗阈值
        node._vel_last_obs_time = time.monotonic() - 1.0
        status = node.update()
        assert status == Status.FAILURE, f"观测超时应 FAILURE，得到 {status}"
        assert fake_hw.ticks[-1] == (0.0, 0.0, 0.0), \
            f"观测超时末发应零速，得到 {fake_hw.ticks[-1]}"
        logger.info("    观测看门狗超时 → 末发零速 FAILURE")
    finally:
        _restore(orig)


def test_velocity_duration_limit_zero():
    """速度模式时长超限 → 末发零速 + FAILURE。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(velocity_timeout_s=10.0)
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        assert node.update() == Status.RUNNING
        # 模拟速度模式已持续 11s
        node._vel_since = time.monotonic() - 11.0
        status = node.update()
        assert status == Status.FAILURE, f"时长超限应 FAILURE，得到 {status}"
        assert fake_hw.ticks[-1] == (0.0, 0.0, 0.0), \
            f"时长超限末发应零速，得到 {fake_hw.ticks[-1]}"
        logger.info("    速度时长超限 → 末发零速 FAILURE")
    finally:
        _restore(orig)


def test_velocity_tick_failure_zero():
    """速度下发失败（异常路径）→ 末发零速 + FAILURE。"""
    fake_hw = _FakeHardware(tick_success=False, tick_message="pub down")
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        status = node.update()
        assert status == Status.FAILURE, f"下发失败应 FAILURE，得到 {status}"
        assert fake_hw.ticks[-1] == (0.0, 0.0, 0.0), \
            f"下发失败末发应零速，得到 {fake_hw.ticks[-1]}"
        logger.info("    速度下发失败 → 末发零速 FAILURE")
    finally:
        _restore(orig)


def test_velocity_terminate_sends_zero():
    """树终止钩子 terminate() → 零速已发（Ctrl-C 的唯一兜底机会）。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        assert node.update() == Status.RUNNING
        before = len(fake_hw.ticks)
        node.terminate(Status.INVALID)
        assert len(fake_hw.ticks) == before + 1, "terminate 应补发一帧零速"
        assert fake_hw.ticks[-1] == (0.0, 0.0, 0.0), \
            f"terminate 末发应零速，得到 {fake_hw.ticks[-1]}"
        logger.info("    terminate() → 零速已发")
    finally:
        _restore(orig)


def _find_event(fake_hw, kind, predicate=None):
    """返回第一条匹配事件的下标；没有则 -1。"""
    for index, event in enumerate(fake_hw.events):
        if event[0] != kind:
            continue
        if predicate is None or predicate(event):
            return index
    return -1


def _is_zero_setpoint(event):
    return event[1] == 0.0 and event[2] == 0.0 and event[3] == 0.0


def test_direct_stream_started_once_setpoint_each_tick():
    """chassis_direct 正常：起流一次、每 tick 更新 setpoint、idle 恒 True。"""
    fake_hw = _FakeHardware()          # gate=None → 从未收到 BaseCmdVelStatus
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(command_channel="chassis_direct")
        reader = _make_idle_reader()
        node.initialise()

        starts = [e for e in fake_hw.events if e[0] == "start"]
        assert len(starts) == 1, f"initialise 应起流一次，得到 {starts}"
        assert starts[0][1] == "/move_base/base_cmd_vel", f"话题不符：{starts[0]}"
        assert abs(starts[0][2] - 100.0) < 1e-9, f"频率不符：{starts[0]}"

        writer = _make_writer()
        for i in range(3):
            _write_cmd(writer, _vcmd(vx=0.03, wz=0.05, iter_index=i))
            assert node.update() == Status.RUNNING
            assert _read_idle(reader) is True, "速度模式 servo_exec_idle 应恒 True"

        assert len([e for e in fake_hw.events if e[0] == "start"]) == 1, \
            "流只应起一次"
        setpoints = [e for e in fake_hw.events if e[0] == "setpoint"]
        assert len(setpoints) == 3, f"每 tick 应更新一次 setpoint，得到 {setpoints}"
        assert abs(setpoints[-1][1] - 0.03) < 1e-9
        assert abs(setpoints[-1][3] - 0.05) < 1e-9
        assert fake_hw.ticks == [], "chassis_direct 不得逐帧调用单帧发布"
        logger.info("    chassis_direct 起流一次 + 每 tick 更新 setpoint + idle 恒 True")
    finally:
        _restore(orig)


def test_direct_gate_absent_does_not_block():
    """从未收到 BaseCmdVelStatus（effective_age_s=None）→ 正常发起运动，不判失败。"""
    fake_hw = _FakeHardware(gate=None)
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(command_channel="chassis_direct")
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        assert node.update() == Status.RUNNING, "门控待命时不得阻断运动"
        moving = [e for e in fake_hw.events
                  if e[0] == "setpoint" and not _is_zero_setpoint(e)]
        assert moving, f"应下发非零目标速度，得到 {fake_hw.events}"
        logger.info("    无状态消息 → 门控待命，正常起运动")
    finally:
        _restore(orig)


def test_direct_gate_five_frames_then_stop():
    """连续 5 帧 cmd_vel_effective=false → 置零停流 + FAILURE；第 4 帧不停。"""
    gate = {"cmd_vel_effective": False, "reason": 1,
            "description": "manual mode", "effective_age_s": 0.01}
    fake_hw = _FakeHardware(gate=gate)
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(command_channel="chassis_direct")
        node.initialise()
        writer = _make_writer()

        for i in range(4):
            _write_cmd(writer, _vcmd(vx=0.03, wz=0.05, iter_index=i))
            status = node.update()
            assert status == Status.RUNNING, \
                f"第 {i + 1} 帧不应判失败，得到 {status}"
        assert not any(e[0] == "stop" for e in fake_hw.events), "第 4 帧不应停流"
        last = [e for e in fake_hw.events if e[0] == "setpoint"][-1]
        assert not _is_zero_setpoint(last), f"第 4 帧仍应下发非零目标：{last}"

        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05, iter_index=4))
        assert node.update() == Status.FAILURE, "第 5 帧应 FAILURE"

        zero_idx = _find_event(fake_hw, "setpoint", _is_zero_setpoint)
        stop_idx = _find_event(fake_hw, "stop")
        assert zero_idx >= 0 and stop_idx >= 0, f"缺停车事件：{fake_hw.events}"
        assert zero_idx < stop_idx, f"应先置零再停流：{fake_hw.events}"

        # Q11：不自动恢复 —— 后续 tick 保持 FAILURE 且不再下发任何指令
        before = len(fake_hw.events)
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05, iter_index=5))
        assert node.update() == Status.FAILURE, "门控失败后应保持 FAILURE"
        assert len(fake_hw.events) == before, \
            f"闩锁后不得再下发：{fake_hw.events[before:]}"
        logger.info("    连续 5 帧无效 → 置零停流 FAILURE，闩锁不自动恢复")
    finally:
        _restore(orig)


def test_direct_stop_sequence_order():
    """退出调用顺序：update_cmd_vel_setpoint(0,0,0) 之后才 stop_cmd_vel_stream。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(command_channel="chassis_direct")
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(stop_reason="converged"))
        assert node.update() == Status.SUCCESS, "converged 应 SUCCESS"

        zero_idx = _find_event(fake_hw, "setpoint", _is_zero_setpoint)
        stop_idx = _find_event(fake_hw, "stop")
        assert zero_idx >= 0, f"缺置零事件：{fake_hw.events}"
        assert stop_idx >= 0, f"缺停流事件：{fake_hw.events}"
        assert zero_idx < stop_idx, f"时序不可调换：{fake_hw.events}"
        assert stop_idx == len(fake_hw.events) - 1, \
            f"停流应是最后一步：{fake_hw.events}"
        logger.info("    置零 → 停流 顺序正确（下标 %d < %d）", zero_idx, stop_idx)
    finally:
        _restore(orig)


def test_direct_terminate_stops_stream_once():
    """terminate() 停流，且幂等（重复 terminate 不重复停流）。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(command_channel="chassis_direct")
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        assert node.update() == Status.RUNNING

        node.terminate(Status.INVALID)
        stops = [e for e in fake_hw.events if e[0] == "stop"]
        assert len(stops) == 1, f"terminate 应停流一次，得到 {stops}"

        node.terminate(Status.INVALID)
        assert len([e for e in fake_hw.events if e[0] == "stop"]) == 1, \
            "重复 terminate 不应重复停流（幂等）"
        logger.info("    terminate() 停流一次且幂等")
    finally:
        _restore(orig)


def test_direct_start_failure_no_leak():
    """起流失败 → FAILURE，但仍走停流清理，且不再下发非零速度。"""
    fake_hw = _FakeHardware(start_success=False)
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(command_channel="chassis_direct")
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        assert node.update() == Status.FAILURE, "起流失败应 FAILURE"
        assert any(e[0] == "stop" for e in fake_hw.events), \
            f"起流失败也必须清理线程：{fake_hw.events}"
        moving = [e for e in fake_hw.events
                  if e[0] == "setpoint" and not _is_zero_setpoint(e)]
        assert not moving, f"失败后不得下发非零速度：{moving}"
        logger.info("    起流失败 → FAILURE + 停流清理，无泄漏")
    finally:
        _restore(orig)


def test_humanoid_channel_never_touches_stream():
    """humanoid 通道：从不调用 start_cmd_vel_stream / update_cmd_vel_setpoint。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()            # 默认 command_channel="humanoid"
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        assert node.update() == Status.RUNNING
        assert fake_hw.events == [], \
            f"humanoid 不得调用任何流方法，得到 {fake_hw.events}"
        assert fake_hw.ticks, "humanoid 仍应走单帧 /cmd_vel"
        logger.info("    humanoid 通道不碰流方法")
    finally:
        _restore(orig)


def test_position_does_not_touch_velocity():
    """position 模式仍走 base_move_relative_jibot，完全不碰速度通路。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _cmd(dx=0.006))
        status = node.update()
        assert status == Status.RUNNING
        assert len(fake_hw.calls) == 1, "position 模式应走 base_move_relative_jibot"
        assert fake_hw.ticks == [], f"position 模式不应碰速度通路，得到 {fake_hw.ticks}"
        logger.info("    position 模式不碰速度通路")
    finally:
        _restore(orig)


def test_position_terminate_no_velocity():
    """position 模式下 terminate() **不**发零速（保持现有场景行为逐字不变）。

    本节点位置模式从不经 /cmd_vel 驱动底盘，终止时多发一帧零速会平白改变
    chassis_servo_v1 / real_v1 的现有行为；零速兜底只对速度模式生效。
    """
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _cmd(dx=0.006))
        assert node.update() == Status.RUNNING
        node.terminate(Status.INVALID)
        assert fake_hw.ticks == [], f"position 模式 terminate 不应发零速，得到 {fake_hw.ticks}"
        logger.info("    position 模式 terminate 不发零速")
    finally:
        _restore(orig)


def test_unknown_channel_rejected_before_any_publish():
    """未知 command_channel → FAILURE，且不调用任何硬件方法（L-2）。"""
    fake_hw = _FakeHardware()
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(command_channel="bogus")
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        assert node.update() == Status.FAILURE, "未知通道应直接 FAILURE"
        assert fake_hw.events == [], f"不得碰流方法，得到 {fake_hw.events}"
        assert fake_hw.ticks == [], f"不得发 /cmd_vel，得到 {fake_hw.ticks}"
        assert fake_hw.calls == [], f"不得下发移动，得到 {fake_hw.calls}"
        logger.info("    未知通道 → FAILURE，零帧下发")
    finally:
        _restore(orig)


def test_direct_stop_failure_keeps_retry_path():
    """停流失败时保留待停标志，terminate() 能再停一次（M-2）。"""
    fake_hw = _FakeHardware(stop_fail_times=1)
    orig = _patch_hw(fake_hw)
    try:
        node = _make_node(command_channel="chassis_direct")
        node.initialise()
        writer = _make_writer()
        _write_cmd(writer, _vcmd(vx=0.03, wz=0.05))
        assert node.update() == Status.RUNNING

        # 第一次停车：置零成功、停流失败
        node._stop_direct_channel()
        assert node._stream_stop_pending is True, "停流失败必须保留待停标志"
        assert node._direct_streaming is True, "流可能还活着，不得清 _direct_streaming"

        stops_before = len([e for e in fake_hw.events if e[0] == "stop"])
        node.terminate(Status.INVALID)
        stops_after = len([e for e in fake_hw.events if e[0] == "stop"])

        assert stops_after == stops_before + 1, \
            f"terminate 必须重试停流，得到 {fake_hw.events}"
        assert node._stream_stop_pending is False, "停流成功后应清待停标志"
        assert node._direct_streaming is False, "停流成功后应清运行标志"
        logger.info("    停流失败 → 保留待停标志 → terminate 重试成功")
    finally:
        _restore(orig)


def main() -> int:
    cases = [
        ("dry_run 短路", test_dry_run),
        ("无指令等待 RUNNING", test_wait_for_command),
        ("非 ServoCommand 拒收", test_invalid_command_failure),
        ("正常指令下发", test_normal_dispatch),
        ("终止指令跳过移动", test_skip_with_stop_reason),
        ("累积量超限 FAILURE", test_skip_cum_limit_failure),
        ("velocity 收敛终止 SUCCESS", test_velocity_stop_converged_success),
        ("velocity 累积量超限 FAILURE", test_velocity_stop_cum_limit_failure),
        ("同一指令去重", test_dedup_same_command),
        ("velocity 单帧 + idle 恒 True", test_velocity_dispatch_idle_true),
        ("velocity 观测看门狗零速", test_velocity_obs_watchdog_zero),
        ("velocity 时长超限零速", test_velocity_duration_limit_zero),
        ("velocity 下发失败零速", test_velocity_tick_failure_zero),
        ("velocity terminate 零速", test_velocity_terminate_sends_zero),
        ("direct 起流一次 + 每 tick 更新", test_direct_stream_started_once_setpoint_each_tick),
        ("direct 无状态门控待命", test_direct_gate_absent_does_not_block),
        ("direct 连续 5 帧无效停车", test_direct_gate_five_frames_then_stop),
        ("direct 停车时序", test_direct_stop_sequence_order),
        ("direct terminate 停流幂等", test_direct_terminate_stops_stream_once),
        ("direct 起流失败无泄漏", test_direct_start_failure_no_leak),
        ("humanoid 不碰流", test_humanoid_channel_never_touches_stream),
        ("position 不碰速度通路", test_position_does_not_touch_velocity),
        ("position terminate 不发零速", test_position_terminate_no_velocity),
        ("未知通道零帧 FAILURE", test_unknown_channel_rejected_before_any_publish),
        ("停流失败保留重试路径", test_direct_stop_failure_keeps_retry_path),
    ]
    failed = 0
    for title, fn in cases:
        reset_blackboard()
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            logger.error("  FAIL  %s: %s", title, exc)
        except Exception as exc:                     # noqa: BLE001
            failed += 1
            logger.error("  ERROR %s: %s: %s", title, type(exc).__name__, exc)

    logger.info("")
    if failed:
        logger.error("FAILED: %d/%d", failed, len(cases))
        return 1
    logger.info("OK: %d/%d 全部通过 —— 底盘伺服执行（编排层，伪硬件注入）测试完成",
                len(cases), len(cases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
