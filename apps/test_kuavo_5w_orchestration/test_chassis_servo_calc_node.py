#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘伺服计算节点测试（编排层，纯计算无硬件）。

被测对象: orchestration.nodes.node_chassis_servo_calc.NodeChassisServoCalc
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE/RUNNING
底层路径: 节点不访问硬件（纯计算）→ ChassisServoCalcSkill（无 get_shared_hardware()）
         → 控制律计算 ServoCommand
本节点为纯计算，可脱离 ROS / 硬件测试：用一个独立的 writer client 模拟上游
"视觉感知模块"往黑板写 AlignObservation，验证节点读观测 → 算指令 → 写回
ServoCommand 的完整闭环。

测试用例说明:
- test_dry_run: STUDIO_DRY_RUN=1 时 update() 直接返 SUCCESS
- test_calc_writes_command: 写入 AlignObservation → 节点算出 ServoCommand 写回黑板
- test_iter_index_increments: 第二轮 iter_index 从上一轮 cmd 取 +1 递推
- test_version_gate: 观测版本未变时不重复计算（RUNNING，cmd 不变）
- test_wait_for_observation: 黑板无观测时持续 RUNNING
- test_invalid_observation_failure: 观测无效 → FAILURE
- test_velocity_mode_writes_command: 静态 mode=velocity：速度模式参数量透传到 skill
- test_mode_from_blackboard_velocity: 黑板 servo_mode=velocity 覆盖静态 mode=position → 速度指令
- test_mode_from_blackboard_position: 黑板 servo_mode=position 覆盖静态 mode=velocity → 位置指令
- test_mode_fallback_when_key_missing_position: 黑板无该键 → 回退静态 mode=position
- test_mode_fallback_when_key_missing_velocity: 黑板无该键 → 回退静态 mode=velocity
- test_mode_illegal_value_warns_and_falls_back: 黑板写入非法值 → 告警 + 回退静态 mode，不崩

运行:
    python3 apps/test_kuavo_5w_orchestration/test_chassis_servo_calc_node.py
退出码 0 通过、1 失败。
"""
import os
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "apps"))

from core.common.logger import init_logging, get_logger
init_logging()

import py_trees
from py_trees.blackboard import Client
from py_trees.common import Access, Status

from core.domain.servo_types import AlignObservation, ServoCommand
from orchestration.nodes.node_chassis_servo_calc import NodeChassisServoCalc
import orchestration.nodes.node_chassis_servo_calc as calc_mod
from _blackboard_isolation import reset_blackboard

logger = get_logger(__name__)

OBS_KEY = "latest_servo_error"
CMD_KEY = "latest_servo_command"


def _obs(e_bottom=1.0, e_right=2.0, theta_rad=0.01, valid=True, reason=""):
    """构造一个 AlignObservation（目标=当前=0，偏差由参数给定）。"""
    return AlignObservation(
        e_bottom_px=e_bottom, e_right_px=e_right, theta_rad=theta_rad,
        valid=valid, reject=reason,
    )


def _make_node():
    """构造节点（纯计算，不需要硬件配置）。"""
    return NodeChassisServoCalc(
        "t_servo_calc", "底盘伺服计算", "ns",
        {"obs_key": OBS_KEY, "command_key": CMD_KEY,
         "gain": 0.6, "max_step_m": 0.015,
         "servo_gain_theta": 0.6, "servo_max_dtheta_rad": 0.05,
         "servo_cumulative_limit_m": 0.10, "servo_max_iters": 8},
    )


def _make_vel_node():
    """构造速度模式节点（mode=velocity，其余同 _make_node）。"""
    return NodeChassisServoCalc(
        "t_servo_calc_vel", "底盘伺服计算(速度)", "ns",
        {"obs_key": OBS_KEY, "command_key": CMD_KEY, "mode": "velocity",
         "gain": 0.6, "max_step_m": 0.015,
         "servo_gain_theta": 0.6, "servo_max_dtheta_rad": 0.05,
         "servo_gain_vel": 0.6, "servo_gain_vel_theta": 0.6,
         "v_max_mps": 0.05, "w_max_radps": 0.15, "v_min_mps": 0.02,
         "servo_cumulative_limit_m": 0.10, "servo_max_iters": 8},
    )


def _make_writer():
    """独立 writer client，模拟上游视觉感知模块写 AlignObservation + 版本号。"""
    w = Client(name="test_writer")
    w.register_key(key=OBS_KEY, access=Access.WRITE)
    w.register_key(key=f"{OBS_KEY}_version", access=Access.WRITE)
    return w


def _make_mode_node(mode, mode_key):
    """构造节点：静态 mode 为初值/回退，mode_key 是读运行模式的黑板键。"""
    return NodeChassisServoCalc(
        "t_servo_calc_mode", "底盘伺服计算(模式接线)", "ns",
        {"obs_key": OBS_KEY, "command_key": CMD_KEY,
         "mode": mode, "mode_key": mode_key,
         "gain": 0.6, "max_step_m": 0.015,
         "servo_gain_theta": 0.6, "servo_max_dtheta_rad": 0.05,
         "servo_gain_vel": 0.6, "servo_gain_vel_theta": 0.6,
         "v_max_mps": 0.05, "w_max_radps": 0.15, "v_min_mps": 0.02,
         "servo_cumulative_limit_m": 0.10, "servo_max_iters": 8},
    )


def _make_mode_writer(key):
    """独立 writer client，模拟模式选择节点往黑板写运行模式。"""
    w = Client(name=f"test_mode_writer_{key}")
    w.register_key(key=key, access=Access.WRITE)
    return w


def _make_reader():
    """独立 reader client，模拟下游执行节点读 ServoCommand。"""
    r = Client(name="test_reader")
    r.register_key(key=CMD_KEY, access=Access.READ)
    return r


def _write_obs(writer, obs, version):
    """上游写入 AlignObservation 并 bump 版本号。"""
    writer.set(OBS_KEY, obs)
    setattr(writer, f"{OBS_KEY}_version", version)


def _read_command(reader):
    """读节点写回的 ServoCommand（不存在返回 None）。"""
    try:
        return getattr(reader, CMD_KEY)
    except KeyError:
        return None


def test_dry_run():
    """STUDIO_DRY_RUN=1 时 update() 直接返 SUCCESS。

    节点模块的 _DRY_RUN 是模块级常量（import 时读一次 os.environ），
    update() 运行时从模块全局查找 _DRY_RUN，故直接 monkeypatch 模块属性即可，
    无需 reload（reload 会破坏零参数 super() 的 __class__ cell 绑定）。
    """
    orig = calc_mod._DRY_RUN
    calc_mod._DRY_RUN = True
    try:
        node = _make_node()
        node.initialise()
        status = node.update()
        assert status == Status.SUCCESS, f"dry-run 应 SUCCESS，得到 {status}"
        logger.info("    dry_run 短路 SUCCESS")
    finally:
        calc_mod._DRY_RUN = orig


def test_calc_writes_command():
    """写入 AlignObservation → 节点算出 ServoCommand 写回黑板。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    _write_obs(writer, _obs(e_bottom=1.0, e_right=2.0, theta_rad=0.01), version=1)
    status = node.update()
    # 纯计算节点算完写回后仍 RUNNING（等下一帧），但黑板应有 ServoCommand
    cmd = _read_command(reader)
    assert cmd is not None, "黑板应写入 ServoCommand"
    assert isinstance(cmd, ServoCommand)
    # dx = 0.6 * 1.0 * 10 / 1000 = 0.006
    assert abs(cmd.dx_m - 0.006) < 1e-9, f"dx={cmd.dx_m}"
    # dy = 0.6 * 2.0 * 10 / 1000 = 0.012
    assert abs(cmd.dy_m - 0.012) < 1e-9, f"dy={cmd.dy_m}"
    assert cmd.iter_index == 0  # 首轮 iter=0
    logger.info("    伺服计算节点写回 ServoCommand 正确（dx=0.006, dy=0.012）")


def test_iter_index_increments():
    """第二轮 iter_index 递增（从上一轮 cmd 取 iter+1）。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    _write_obs(writer, _obs(e_bottom=1.0), version=1)
    node.update()
    cmd1 = _read_command(reader)
    assert cmd1.iter_index == 0
    # 第二轮：换一个新观测对象 + bump 版本
    _write_obs(writer, _obs(e_bottom=0.5), version=2)
    node.update()
    cmd2 = _read_command(reader)
    assert cmd2.iter_index == 1, f"第二轮 iter 应=1，得到 {cmd2.iter_index}"
    # 累积量应累加
    assert cmd2.cumulative_xy_m > cmd1.cumulative_xy_m
    logger.info("    iter_index / 累积量跨帧递推正确")


def test_version_gate():
    """观测版本未变时不重复计算（仍 RUNNING，cmd 不变）。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    _write_obs(writer, _obs(e_bottom=1.0), version=1)
    node.update()
    cmd1 = _read_command(reader)
    # 同一版本号再 update（不 bump）
    status = node.update()
    assert status == Status.RUNNING
    cmd2 = _read_command(reader)
    assert cmd1 is cmd2, "版本未变不应重算，cmd 应同一对象"
    logger.info("    版本门禁生效（观测未更新不重算）")


def test_wait_for_observation():
    """黑板无观测时持续 RUNNING。"""
    node = _make_node()
    node.initialise()
    status = node.update()
    assert status == Status.RUNNING, f"无观测应 RUNNING，得到 {status}"
    logger.info("    无观测时持续 RUNNING")


def test_invalid_observation_failure():
    """观测无效 → FAILURE。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    _write_obs(writer, _obs(valid=False, reason="no_edge"), version=1)
    status = node.update()
    assert status == Status.FAILURE, f"无效观测应 FAILURE，得到 {status}"
    logger.info("    无效观测 FAILURE")


def test_velocity_mode_writes_command():
    """mode=velocity：节点把速度模式参数量透传给 skill，写回 ServoCommand。"""
    node = _make_vel_node()
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    # e_bottom=5 → vx = 0.6*5*10/1000 = 0.030（> v_min，未触上限）
    _write_obs(writer, _obs(e_bottom=5.0, e_right=-5.0, theta_rad=0.02), version=1)
    node.update()
    cmd = _read_command(reader)
    assert cmd is not None, "黑板应写入 ServoCommand"
    assert cmd.mode == "velocity", f"mode={cmd.mode}"
    assert abs(cmd.vx_mps - 0.030) < 1e-9, f"vx={cmd.vx_mps}"
    assert abs(cmd.vy_mps - (-0.030)) < 1e-9, f"vy={cmd.vy_mps}"
    # wz = 0.6 * 0.02 = 0.012
    assert abs(cmd.wz_radps - 0.012) < 1e-9, f"wz={cmd.wz_radps}"
    # 位置字段归零
    assert cmd.dx_m == 0.0 and cmd.dy_m == 0.0 and cmd.dtheta_rad == 0.0
    logger.info("    速度模式节点参数量透传正确（vx=0.030, vy=-0.030, wz=0.012）")


def test_mode_from_blackboard_velocity():
    """黑板 servo_mode=velocity 覆盖静态 mode=position → 产出速度指令。"""
    key = "bb_servo_mode_vel"
    node = _make_mode_node("position", key)
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    _make_mode_writer(key).set(key, "velocity")
    # e_bottom=5 → vx = 0.6*5*10/1000 = 0.030
    _write_obs(writer, _obs(e_bottom=5.0, e_right=-5.0, theta_rad=0.02), version=1)
    node.update()
    cmd = _read_command(reader)
    assert cmd is not None, "黑板应写入 ServoCommand"
    assert cmd.mode == "velocity", f"黑板 mode 应覆盖静态参数，得到 {cmd.mode}"
    assert abs(cmd.vx_mps - 0.030) < 1e-9, f"vx={cmd.vx_mps}"
    logger.info("    黑板 servo_mode=velocity 覆盖静态 position 生效")


def test_mode_from_blackboard_position():
    """黑板 servo_mode=position 覆盖静态 mode=velocity → 产出位置指令。"""
    key = "bb_servo_mode_pos"
    node = _make_mode_node("velocity", key)
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    _make_mode_writer(key).set(key, "position")
    # e_bottom=1 → dx = 0.6*1*10/1000 = 0.006
    _write_obs(writer, _obs(e_bottom=1.0, e_right=2.0, theta_rad=0.01), version=1)
    node.update()
    cmd = _read_command(reader)
    assert cmd is not None, "黑板应写入 ServoCommand"
    assert cmd.mode == "position", f"黑板 mode 应覆盖静态参数，得到 {cmd.mode}"
    assert abs(cmd.dx_m - 0.006) < 1e-9, f"dx={cmd.dx_m}"
    logger.info("    黑板 servo_mode=position 覆盖静态 velocity 生效")


def test_mode_fallback_when_key_missing_position():
    """黑板无该键 → 回退到静态 mode=position。"""
    key = "bb_servo_mode_absent_pos"
    node = _make_mode_node("position", key)
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    # 不写 mode 键（键未注册 / 未写过）
    _write_obs(writer, _obs(e_bottom=1.0, e_right=2.0, theta_rad=0.01), version=1)
    node.update()
    cmd = _read_command(reader)
    assert cmd is not None, "黑板应写入 ServoCommand"
    assert cmd.mode == "position", f"应回退静态 position，得到 {cmd.mode}"
    assert abs(cmd.dx_m - 0.006) < 1e-9, f"dx={cmd.dx_m}"
    logger.info("    黑板无 mode 键 → 回退静态 position")


def test_mode_fallback_when_key_missing_velocity():
    """黑板无该键 → 回退到静态 mode=velocity。"""
    key = "bb_servo_mode_absent_vel"
    node = _make_mode_node("velocity", key)
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    _write_obs(writer, _obs(e_bottom=5.0, e_right=-5.0, theta_rad=0.02), version=1)
    node.update()
    cmd = _read_command(reader)
    assert cmd is not None, "黑板应写入 ServoCommand"
    assert cmd.mode == "velocity", f"应回退静态 velocity，得到 {cmd.mode}"
    assert abs(cmd.vx_mps - 0.030) < 1e-9, f"vx={cmd.vx_mps}"
    logger.info("    黑板无 mode 键 → 回退静态 velocity")


def test_mode_illegal_value_warns_and_falls_back():
    """黑板写入非法值 → 告警 + 回退静态 mode，不崩。"""
    key = "bb_servo_mode_illegal"
    node = _make_mode_node("position", key)
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    _make_mode_writer(key).set(key, "foo")

    warnings = []
    orig_warning = calc_mod.logger.warning
    calc_mod.logger.warning = lambda *a, **kw: warnings.append(a)
    try:
        _write_obs(writer, _obs(e_bottom=1.0, e_right=2.0, theta_rad=0.01), version=1)
        status = node.update()
    finally:
        calc_mod.logger.warning = orig_warning

    assert status == Status.RUNNING, f"非法 mode 不应导致节点失败，得到 {status}"
    cmd = _read_command(reader)
    assert cmd is not None, "非法 mode 应仍能算出指令"
    assert cmd.mode == "position", f"非法值应回退静态 position，得到 {cmd.mode}"
    assert len(warnings) == 1, f"应告警一次，实际 {len(warnings)} 次"
    logger.info("    黑板非法 mode 值 → 告警一次并回退静态 position")


def main() -> int:
    cases = [
        ("dry_run 短路", test_dry_run),
        ("写回 ServoCommand", test_calc_writes_command),
        ("iter/累积量递推", test_iter_index_increments),
        ("版本门禁", test_version_gate),
        ("等观测 RUNNING", test_wait_for_observation),
        ("无效观测 FAILURE", test_invalid_observation_failure),
        ("速度模式写回", test_velocity_mode_writes_command),
        ("黑板 mode=velocity 生效", test_mode_from_blackboard_velocity),
        ("黑板 mode=position 生效", test_mode_from_blackboard_position),
        ("缺键回退 position", test_mode_fallback_when_key_missing_position),
        ("缺键回退 velocity", test_mode_fallback_when_key_missing_velocity),
        ("非法 mode 回退+告警", test_mode_illegal_value_warns_and_falls_back),
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
    logger.info("OK: %d/%d 全部通过 —— 底盘伺服计算（编排层，纯计算）测试完成",
                len(cases), len(cases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
