#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘伺服判断节点测试（编排层，纯计算无硬件）。

被测对象: orchestration.nodes.node_chassis_servo_judge.NodeChassisServoJudge
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE/RUNNING
底层路径: 节点不访问硬件（纯计算）→ ChassisServoJudgeSkill → 收敛/继续/超限失败
本节点为纯计算，可脱离 ROS / 硬件测试：用独立 writer client 模拟上游"视觉感知
模块"写 AlignObservation、模拟"控制器节点"写 ServoCommand，验证节点读观测 +
读指令 + 读上一轮判决 → 算判决 → 写回 latest_servo_verdict 的完整闭环。

测试用例说明:
- test_dry_run: STUDIO_DRY_RUN=1 时 update() 直接返 SUCCESS
- test_converged_after_two_frames: 连续两帧满足收敛 → 第二帧 SUCCESS
- test_continue_when_error_above: 偏差超阈值 → RUNNING（continue）
- test_failed_cum_priority: 累积量超限优先 → FAILURE（failed_cum）
- test_failed_iter_when_not_converged: 迭代超限未收敛 → FAILURE（failed_iter）
- test_version_gate: 观测版本未变时不重判（RUNNING）
- test_wait_for_observation: 黑板无观测时持续 RUNNING
- test_invalid_observation_failure: 观测无效 → FAILURE

运行:
    python3 apps/test_kuavo_5w_orchestration/test_chassis_servo_judge_node.py
退出码 0 通过、1 失败。
"""
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
from orchestration.nodes.node_chassis_servo_judge import NodeChassisServoJudge
import orchestration.nodes.node_chassis_servo_judge as judge_mod
from _blackboard_isolation import reset_blackboard

logger = get_logger(__name__)

OBS_KEY = "latest_servo_error"
CMD_KEY = "latest_servo_command"
VERDICT_KEY = "latest_servo_verdict"


def _obs(e_bottom=0.0, e_right=0.0, theta_rad=0.0, 
         valid=True, reason=""):
    """构造一个 AlignObservation（目标=当前=0，偏差由参数给定）。"""
    return AlignObservation(
        e_bottom_px=e_bottom, e_right_px=e_right, theta_rad=theta_rad,
        valid=valid, reject=reason,
    )


def _cmd(iter_index=0, cumulative=0.0, stop_reason=""):
    return ServoCommand(
        dx_m=0.0, dy_m=0.0, dtheta_rad=0.0,
        iter_index=iter_index, cumulative_xy_m=cumulative, stop_reason=stop_reason,
    )


def _make_node():
    """构造判断节点（纯计算，不需要硬件配置）。"""
    return NodeChassisServoJudge(
        "t_servo_judge", "底盘伺服判断", "ns",
        {"obs_key": OBS_KEY, "command_key": CMD_KEY,
         "verdict_key": VERDICT_KEY,
         "servo_stop_threshold_px": 10.0,
         "servo_stop_theta_rad": 0.02,
         "servo_converge_frames": 2,
         "servo_max_iters": 8,
         "servo_cumulative_limit_m": 0.10},
    )


def _make_writer():
    """writer client：模拟上游视觉感知模块写 AlignObservation + 版本号，
    以及控制器节点写 ServoCommand（判断节点读 iter/cum）。"""
    w = Client(name="test_writer")
    w.register_key(key=OBS_KEY, access=Access.WRITE)
    w.register_key(key=f"{OBS_KEY}_version", access=Access.WRITE)
    w.register_key(key=CMD_KEY, access=Access.WRITE)
    return w


def _make_reader():
    """reader client：读判断节点写回的 verdict。"""
    r = Client(name="test_reader")
    r.register_key(key=VERDICT_KEY, access=Access.READ)
    return r


def _write_obs(writer, obs, version):
    writer.set(OBS_KEY, obs)
    setattr(writer, f"{OBS_KEY}_version", version)


def _write_cmd(writer, cmd):
    writer.set(CMD_KEY, cmd)


def _read_verdict(reader):
    try:
        return getattr(reader, VERDICT_KEY)
    except KeyError:
        return None


def test_dry_run():
    """STUDIO_DRY_RUN=1 时 update() 直接返 SUCCESS。

    _DRY_RUN 是模块级常量，monkeypatch 模块属性即可（reload 会破坏
    零参数 super() 的 __class__ cell 绑定）。
    """
    orig = judge_mod._DRY_RUN
    judge_mod._DRY_RUN = True
    try:
        node = _make_node()
        node.initialise()
        status = node.update()
        assert status == Status.SUCCESS, f"dry-run 应 SUCCESS，得到 {status}"
        logger.info("    dry_run 短路 SUCCESS")
    finally:
        judge_mod._DRY_RUN = orig


def test_converged_after_two_frames():
    """连续两帧满足收敛 → 第二帧 SUCCESS，verdict=converged。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    # 第一帧：偏差为 0（满足），prev_count 默认 0 → count=1 → continue（RUNNING）
    _write_obs(writer, _obs(), version=1)
    _write_cmd(writer, _cmd())
    s1 = node.update()
    v1 = _read_verdict(reader)
    assert s1 == Status.RUNNING, f"第一帧应 RUNNING，得到 {s1}"
    assert v1["verdict"] == "continue"
    assert v1["converge_count"] == 1
    # 第二帧：仍满足 + version bump → count=2 → converged（SUCCESS）
    _write_obs(writer, _obs(), version=2)
    _write_cmd(writer, _cmd())
    s2 = node.update()
    v2 = _read_verdict(reader)
    assert s2 == Status.SUCCESS, f"第二帧应 SUCCESS，得到 {s2}"
    assert v2["verdict"] == "converged"
    assert v2["converge_count"] == 2
    logger.info("    连续两帧收敛 → SUCCESS（converged）")


def test_continue_when_error_above():
    """偏差超阈值 → RUNNING（continue）。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    _write_obs(writer, _obs(e_bottom=15.0), version=1)  # > 10.0 阈值
    _write_cmd(writer, _cmd())
    status = node.update()
    assert status == Status.RUNNING
    logger.info("    偏差超阈值 → RUNNING（continue）")


def test_failed_cum_priority():
    """累积量超限优先 → FAILURE（failed_cum，即便迭代也超）。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    _write_obs(writer, _obs(e_bottom=15.0), version=1)
    _write_cmd(writer, _cmd(iter_index=8, cumulative=0.11))  # iter 与 cum 都超
    status = node.update()
    v = _read_verdict(_make_reader())
    assert status == Status.FAILURE, f"累积超限应 FAILURE，得到 {status}"
    # 节点写回的 verdict（通过另一 reader 读）
    assert v["verdict"] == "failed_cum"
    logger.info("    累积量超限优先 → FAILURE（failed_cum）")


def test_failed_iter_when_not_converged():
    """迭代超限且未收敛 → FAILURE（failed_iter）。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    _write_obs(writer, _obs(e_bottom=15.0), version=1)  # 未收敛
    _write_cmd(writer, _cmd(iter_index=8, cumulative=0.05))  # cum 未超，iter 超
    status = node.update()
    assert status == Status.FAILURE
    logger.info("    迭代超限未收敛 → FAILURE（failed_iter）")


def test_version_gate():
    """观测版本未变时不重判（RUNNING，verdict 不变）。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer()
    reader = _make_reader()
    _write_obs(writer, _obs(e_bottom=15.0), version=1)
    _write_cmd(writer, _cmd())
    node.update()
    v1 = _read_verdict(reader)
    # 同版本再 update（不 bump）
    status = node.update()
    assert status == Status.RUNNING
    v2 = _read_verdict(reader)
    assert v1 is v2, "版本未变不应重判，verdict 应同一对象"
    logger.info("    版本门禁生效（观测未更新不重判）")


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
    _write_cmd(writer, _cmd())
    status = node.update()
    assert status == Status.FAILURE, f"无效观测应 FAILURE，得到 {status}"
    logger.info("    无效观测 FAILURE")


def main() -> int:
    cases = [
        ("dry_run 短路", test_dry_run),
        ("连续两帧收敛", test_converged_after_two_frames),
        ("偏差超阈值 continue", test_continue_when_error_above),
        ("累积量超限优先", test_failed_cum_priority),
        ("迭代超限 failed_iter", test_failed_iter_when_not_converged),
        ("版本门禁", test_version_gate),
        ("等观测 RUNNING", test_wait_for_observation),
        ("无效观测 FAILURE", test_invalid_observation_failure),
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
    logger.info("OK: %d/%d 全部通过 —— 底盘伺服判断（编排层，纯计算）测试完成",
                len(cases), len(cases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
