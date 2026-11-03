#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘伺服模式选择节点测试（编排层，硬件节点，伪硬件注入）。

被测对象: orchestration.nodes.node_chassis_servo_mode.NodeChassisServoMode
节点生命周期: initialise() → 循环 update() → RUNNING（源节点口径，从不收口）
底层路径: 节点 get_shared_hardware() → enable_vel_control_jibot（权限）
                                    → send_base_velocity_tick（零速兜底）
测试策略（模块单独测试，不依赖真机 / ROS 服务）:
    monkeypatch 节点模块的 get_shared_hardware，注入伪硬件记录
    enable_vel_control_jibot / send_base_velocity_tick 的调用序列，从而：
    - 不需要 kuavo_msgs / jibot 下位机服务
    - 完整验证 读观测 → 双阈值滞回判决 → 切权限 → 写 servo_mode 的接线

测试用例说明:
- test_dry_run: STUDIO_DRY_RUN=1 时 update() 短路 RUNNING（不碰硬件）
- test_large_error_stays_position: 大误差不切换，保持 position
- test_hysteresis_band_no_switch: 误差落在 enter~exit 之间（滞回带）不切换
- test_enter_velocity_and_restore: 误差 < enter 切 velocity（收到 True），
                                   误差又涨过 exit 切回 position（收到 False）
- test_max_switch_locks: 切换次数到 max_mode_switch 后锁死，不再调权限服务
- test_terminate_restores: terminate() 后零速已发 + enable_vel_control_jibot(False)
- test_missing_observation_running: 无观测时 RUNNING 且不切换
- test_invalid_observation_running: 观测无效时 RUNNING 且不切换
- test_in_flight_defers_switch: 位置动作在途（servo_exec_idle=False）时暂缓切权限，
                                到达（idle=True）后才切——硬门控会掐断在途任务

运行:
    python3 apps/test_kuavo_5w_orchestration/test_chassis_servo_mode_node.py
退出码 0 通过、1 失败。
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "apps"))

from core.common.logger import init_logging, get_logger
init_logging()

from py_trees.blackboard import Client
from py_trees.common import Access, Status

from core.domain.result import Result
from core.domain.servo_types import AlignObservation
from orchestration.nodes.node_chassis_servo_mode import NodeChassisServoMode
import orchestration.nodes.node_chassis_servo_mode as mode_mod
from _blackboard_isolation import reset_blackboard

logger = get_logger(__name__)

OBS_KEY = "latest_servo_error"
MODE_KEY = "servo_mode"
IDLE_KEY = "servo_exec_idle"

# 与场景默认一致的阈值（ticket 05）
ENTER_PX = 12.0
EXIT_PX = 25.0


class _FakeHardware:
    """伪硬件：记录权限切换与速度发布的完整调用序列。"""

    def __init__(self):
        self.enable_calls = []      # enable_vel_control_jibot 的入参序列
        self.vel_calls = []         # send_base_velocity_tick 的入参序列

    def enable_vel_control_jibot(self, enable):
        self.enable_calls.append(enable)
        return Result.ok(f"enable_vel_control({enable})", data={"state_after": enable})

    def send_base_velocity_tick(self, vx, vy, vyaw_rad):
        self.vel_calls.append((vx, vy, vyaw_rad))
        return Result.ok("velocity tick")


def _obs(e_bottom=0.0, e_right=0.0, theta_rad=0.0, valid=True, reason=""):
    """构造一个 AlignObservation（偏差由参数给定）。"""
    return AlignObservation(
        e_bottom_px=e_bottom, e_right_px=e_right, theta_rad=theta_rad,
        valid=valid, reject=reason,
    )


def _make_node(max_mode_switch=2, enter_px=ENTER_PX, exit_px=EXIT_PX, settle_s=0.0):
    """构造模式节点；settle_s 在测试里给 0（不真 sleep，权限层已有自己的 sleep）。"""
    return NodeChassisServoMode(
        "t_servo_mode", "底盘伺服模式选择", "ns",
        {"mode_key": MODE_KEY,
         "enter_px": str(enter_px), "exit_px": str(exit_px),
         "max_mode_switch": str(max_mode_switch), "settle_s": str(settle_s)},
    )


def _patch_hw(fake_hw):
    """monkeypatch 节点模块的 get_shared_hardware 返回伪硬件。"""
    orig = mode_mod.get_shared_hardware
    mode_mod.get_shared_hardware = lambda: fake_hw
    return orig


def _restore(orig):
    mode_mod.get_shared_hardware = orig


def _make_writer(initial_mode="position"):
    """writer client：模拟视觉感知模块写 AlignObservation。

    黑板是进程内全局单例，跨用例不隔离，故这里每次都把模式键显式复位成
    initial_mode，用例之间不互相污染（同 NOTES §2 的跨文件污染教训）。
    """
    w = Client(name=f"test_mode_writer_{id(initial_mode)}")
    w.register_key(key=OBS_KEY, access=Access.WRITE)
    w.register_key(key=MODE_KEY, access=Access.WRITE)
    w.set(MODE_KEY, initial_mode)
    return w


def _make_reader():
    """reader client：读节点写回的模式键。"""
    r = Client(name="test_mode_reader")
    r.register_key(key=MODE_KEY, access=Access.READ)
    return r


def _write_obs(writer, obs):
    writer.set(OBS_KEY, obs)


def _read_mode(reader):
    try:
        return getattr(reader, MODE_KEY)
    except KeyError:
        return None


def test_dry_run():
    """STUDIO_DRY_RUN=1 时 update() 短路 RUNNING（不碰硬件）。"""
    orig = mode_mod._DRY_RUN
    mode_mod._DRY_RUN = True
    try:
        node = _make_node()
        node.initialise()
        status = node.update()
        assert status == Status.RUNNING, f"dry-run 应 RUNNING，得到 {status}"
        logger.info("    dry_run 短路 RUNNING")
    finally:
        mode_mod._DRY_RUN = orig


def test_large_error_stays_position():
    """大误差（> exit）不切换，保持 position，不调权限服务。"""
    fake = _FakeHardware()
    orig = _patch_hw(fake)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        reader = _make_reader()
        _write_obs(writer, _obs(e_bottom=40.0, e_right=5.0))   # e_px = 40 > exit
        status = node.update()
        assert status == Status.RUNNING, f"应 RUNNING，得到 {status}"
        assert fake.enable_calls == [], f"不应调权限服务，得到 {fake.enable_calls}"
        assert _read_mode(reader) == "position", "模式应仍为 position（未被改写）"
        logger.info("    大误差维持 position、不调权限服务")
    finally:
        _restore(orig)


def test_hysteresis_band_no_switch():
    """误差落在滞回带（enter < e_px < exit）不切换 —— 滞回的核心。"""
    fake = _FakeHardware()
    orig = _patch_hw(fake)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        reader = _make_reader()
        # e_px = 18，12 < 18 < 25：两侧条件都不成立
        _write_obs(writer, _obs(e_bottom=18.0, e_right=3.0))
        status = node.update()
        assert status == Status.RUNNING, f"应 RUNNING，得到 {status}"
        assert fake.enable_calls == [], f"滞回带内不应切换，得到 {fake.enable_calls}"
        assert _read_mode(reader) == "position", "模式应仍为 position"
        # 反向也验一次：已经在 velocity 时落在同一带内不退回
        node._mode = "velocity"
        node.update()
        assert fake.enable_calls == [], f"velocity 侧也不应切换，得到 {fake.enable_calls}"
        logger.info("    滞回带内（enter < e < exit）不切换")
    finally:
        _restore(orig)


def test_enter_velocity_and_restore():
    """误差 < enter 切 velocity（权限收到 True）；误差又涨过 exit 切回 position（False）。"""
    fake = _FakeHardware()
    orig = _patch_hw(fake)
    try:
        node = _make_node(max_mode_switch=5)
        node.initialise()
        writer = _make_writer()
        reader = _make_reader()

        # 1) e_px = 5 < enter → 切 velocity
        _write_obs(writer, _obs(e_bottom=5.0, e_right=-3.0))
        status = node.update()
        assert status == Status.RUNNING, f"应 RUNNING，得到 {status}"
        assert fake.enable_calls == [True], f"应收到 True，得到 {fake.enable_calls}"
        assert _read_mode(reader) == "velocity", f"模式应为 velocity，得到 {_read_mode(reader)}"

        # 2) e_px = 30 > exit → 切回 position
        _write_obs(writer, _obs(e_bottom=30.0))
        node.update()
        assert fake.enable_calls == [True, False], f"应再收到 False，得到 {fake.enable_calls}"
        assert _read_mode(reader) == "position", f"模式应为 position，得到 {_read_mode(reader)}"
        logger.info("    <enter 进 velocity(True) / >exit 回 position(False)")
    finally:
        _restore(orig)


def test_max_switch_locks():
    """切换次数到 max_mode_switch 后锁死，不再调权限服务。"""
    fake = _FakeHardware()
    orig = _patch_hw(fake)
    try:
        node = _make_node(max_mode_switch=2)
        node.initialise()
        writer = _make_writer()
        reader = _make_reader()

        # 第 1 次切换：position → velocity（计数 1，未到上限）
        _write_obs(writer, _obs(e_bottom=5.0))
        node.update()
        assert fake.enable_calls == [True]
        assert _read_mode(reader) == "velocity"

        # 第 2 次切换：velocity → position（计数 2 = 上限 → 锁定）
        _write_obs(writer, _obs(e_bottom=40.0))
        node.update()
        assert fake.enable_calls == [True, False], \
            f"应恰好两次切换，得到 {fake.enable_calls}"
        assert _read_mode(reader) == "position"

        # 之后误差再变小，本应切 velocity，但已锁定 → 不再调服务
        for _ in range(3):
            _write_obs(writer, _obs(e_bottom=3.0))
            node.update()
        assert fake.enable_calls == [True, False], \
            f"锁定后不应再调权限服务，得到 {fake.enable_calls}"
        assert _read_mode(reader) == "position", "模式应锁在 position"
        logger.info("    达 max_mode_switch 后锁定、不再调权限服务")
    finally:
        _restore(orig)


def test_terminate_restores():
    """terminate() 后：零速已发 + enable_vel_control_jibot(False)。"""
    fake = _FakeHardware()
    orig = _patch_hw(fake)
    try:
        node = _make_node(max_mode_switch=5)
        node.initialise()
        writer = _make_writer()

        # 先切到 velocity，再终止，看还原
        _write_obs(writer, _obs(e_bottom=5.0))
        node.update()
        assert fake.enable_calls == [True]
        zero_before = len(fake.vel_calls)

        node.terminate(Status.SUCCESS)
        assert fake.vel_calls[-1] == (0.0, 0.0, 0.0), \
            f"terminate 应发零速，得到 {fake.vel_calls[-1]}"
        assert len(fake.vel_calls) > zero_before, "terminate 应至少再发一次零速"
        assert fake.enable_calls[-1] is False, \
            f"terminate 应还原权限 False，得到 {fake.enable_calls}"
        logger.info("    terminate 发零速 + 还原权限 False")
    finally:
        _restore(orig)


def test_missing_observation_running():
    """黑板无观测时 RUNNING 且不切换（源节点等数据）。"""
    fake = _FakeHardware()
    orig = _patch_hw(fake)
    try:
        node = _make_node()
        node.initialise()
        status = node.update()
        assert status == Status.RUNNING, f"无观测应 RUNNING，得到 {status}"
        assert fake.enable_calls == []
        logger.info("    无观测维持 RUNNING、不切换")
    finally:
        _restore(orig)


def test_invalid_observation_running():
    """观测无效时 RUNNING 且不切换（零速兜底归 exec）。"""
    fake = _FakeHardware()
    orig = _patch_hw(fake)
    try:
        node = _make_node()
        node.initialise()
        writer = _make_writer()
        _write_obs(writer, _obs(e_bottom=1.0, valid=False, reason="no_edge"))
        status = node.update()
        assert status == Status.RUNNING, f"无效观测应 RUNNING，得到 {status}"
        assert fake.enable_calls == [], "无效观测不应切换"
        logger.info("    无效观测维持 RUNNING、不切换")
    finally:
        _restore(orig)


def test_in_flight_defers_switch():
    """位置动作在途（idle=False）时暂缓切权限，到达（idle=True）后才切。

    2026-10-15 真机：`_switch_to()` 在 `base_move_relative_jibot` 的任务还没到达时
    就 `enable_vel_control_jibot(True)`，硬门控把在途任务从导航栈手里抢走，
    `check_arrived` 从此恒返回 `moving`，exec 卡在 `_poll_arrival()`、judge 被
    `servo_exec_idle=False` 挡着不判决 —— 底盘静止而终端一直刷屏。
    """
    fake = _FakeHardware()
    orig = _patch_hw(fake)
    try:
        node = _make_node(max_mode_switch=5)
        node.initialise()
        writer = _make_writer()
        reader = _make_reader()
        idle = Client(name="test_mode_idle_writer")
        idle.register_key(key=IDLE_KEY, access=Access.WRITE)

        _write_obs(writer, _obs(e_bottom=5.0))          # e_px < enter，本应切 velocity
        idle.set(IDLE_KEY, False)                        # 位置动作在途
        for _ in range(3):
            assert node.update() == Status.RUNNING
        assert fake.enable_calls == [], \
            f"在途时不应切权限（会掐断在途任务），得到 {fake.enable_calls}"
        assert _read_mode(reader) == "position", "在途时模式应保持 position"

        idle.set(IDLE_KEY, True)                         # 位置动作到达，底盘空闲
        assert node.update() == Status.RUNNING
        assert fake.enable_calls == [True], \
            f"到达后应切 velocity，得到 {fake.enable_calls}"
        assert _read_mode(reader) == "velocity", "到达后模式应切到 velocity"
        logger.info("    在途暂缓 / 到达后切换")
    finally:
        _restore(orig)


def main() -> int:
    cases = [
        ("dry_run 短路", test_dry_run),
        ("大误差维持 position", test_large_error_stays_position),
        ("滞回带内不切换", test_hysteresis_band_no_switch),
        ("<enter 进 velocity / >exit 回 position", test_enter_velocity_and_restore),
        ("切换上限后锁定", test_max_switch_locks),
        ("terminate 还原权限", test_terminate_restores),
        ("无观测保持 RUNNING", test_missing_observation_running),
        ("无效观测保持 RUNNING", test_invalid_observation_running),
        ("在途暂缓切权限", test_in_flight_defers_switch),
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
    logger.info("OK: %d/%d 全部通过 —— 底盘伺服模式选择（编排层，伪硬件注入）测试完成",
                len(cases), len(cases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
