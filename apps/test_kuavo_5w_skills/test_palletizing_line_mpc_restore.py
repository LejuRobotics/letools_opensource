#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""带双臂与下肢关节快照恢复的轮臂码垛直线搬运（MPC 路径）验收。

被测技能:
``skills.atomic.refactored_sdk.palletizing_line_mpc_restore.PalletizingLineMpcRestoreSkill``

**会动**：切换 MPC 模式并下发离线轨迹。默认执行原 10 段码垛路径，也可通过
``--roundtrip-axis x/y/z`` 选择单轴两段往返。技能在运动前记录双臂 14 关节和下肢
4 关节，正常完成、失败或异常后均尝试恢复全部 18 个关节并读回验证。``--arms`` 只
决定参与搬运和轨迹验收的手臂，不缩小快照及恢复范围。第一次请在仿真或空载环境运行，
真机需要有人的手放在急停上。

运行::

    source /opt/ros/noetic/setup.bash
    source infrastructure/ros_packages/devel/setup.bash
    export PYTHONPATH=$(pwd):$PYTHONPATH

    python3 apps/test_kuavo_5w_skills/test_palletizing_line_mpc_restore.py
    python3 apps/test_kuavo_5w_skills/test_palletizing_line_mpc_restore.py --arms left
    python3 apps/test_kuavo_5w_skills/test_palletizing_line_mpc_restore.py --arms right
    python3 apps/test_kuavo_5w_skills/test_palletizing_line_mpc_restore.py --arms left --roundtrip-axis y

本入口有意不复用 ``skill_setup`` / ``skill_teardown``：前者会在快照前复位手臂和躯干，
后者会在快照恢复后再次复位躯干，都会破坏本案例的关节快照语义。搬运统计和阈值判定与
``test_palletizing_line_mpc.py`` 一致，恢复验证失败时入口退出码为 1。
"""
import argparse
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from apps.test_kuavo_5w_skills._eval.line_metrics import LineMetricsRecorder
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, run_skill, selected_arms, selected_ee_frames)
from apps.test_kuavo_5w_skills.test_palletizing_line_mpc import (
    EE_FRAMES, SAMPLE_RATE, WORLD_FRAME, evaluate, report, switch_to_mpc_base_arm)
from skills.atomic.refactored_sdk.palletizing_line_mpc_restore import (
    PalletizingLineMpcRestoreParams, PalletizingLineMpcRestoreSkill)

logger = get_logger(__name__)


def build_params(selection: str, roundtrip_axis: str = "full") -> PalletizingLineMpcRestoreParams:
    """构造技能参数；``--arms`` 只限制运动手臂，恢复快照始终覆盖双臂和下肢。"""
    return PalletizingLineMpcRestoreParams(
        arms=selection,
        roundtrip_axis=roundtrip_axis,
    )


def prepare_external_arm_control(hardware) -> bool:
    """只切外部控制，不调用 mode=1，避免在技能记录快照前复位手臂。"""
    result = hardware.set_arm_control_mode(2)
    if not result.success:
        logger.error("切换手臂外部控制失败: %s", result.message)
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arms", choices=("both", "left", "right"), default="both",
                        help="参与验收的手臂：both（默认）/ left / right")
    parser.add_argument("--roundtrip-axis", choices=("full", "x", "y", "z"),
                        default="full", help="full=原码垛路径；x/y/z=对应单轴往返")
    args = parser.parse_args()
    selection = args.arms
    arms = selected_arms(selection)
    logger.info("验收手臂: %s（--arms %s）；路径: %s；关节恢复范围: 双臂 + 下肢",
                ", ".join(arms), selection, args.roundtrip_axis)

    # low 提供下肢状态；arm/timed 分别提供关节轨迹与 MPC 离线轨迹服务。
    hardware = build_hardware(whitelist=['arm', 'timed', 'low'])
    passed = False
    try:
        hardware.initialize()
        if not prepare_external_arm_control(hardware):
            logger.error("手臂外部控制前置失败，取消搬运")
        elif not switch_to_mpc_base_arm(hardware):
            logger.error("MPC 前置失败，取消搬运")
        else:
            skill = PalletizingLineMpcRestoreSkill(hardware=hardware)
            ee_frames = selected_ee_frames(EE_FRAMES, arms)
            with LineMetricsRecorder(hardware, skill, WORLD_FRAME, arms, ee_frames,
                                     rate=SAMPLE_RATE) as recorder:
                run_ok = run_skill(
                    skill, build_params(selection, args.roundtrip_axis))
            failures = evaluate(recorder)
            report(recorder, failures)
            passed = run_ok and not failures
    finally:
        # 关节恢复已由技能完成并验证；这里不能再 reset_torso_to_initial 或 arm_reset。
        hardware.shutdown()

    if not passed:
        logger.error("❌ MPC 搬运或 18 关节快照恢复验收未通过")
        sys.exit(1)
    logger.info("🎉 MPC 搬运及 18 关节快照恢复验收通过")


if __name__ == '__main__':
    main()
