#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮臂码垛直线搬运（MPC 路径）验收。

被测技能: ``skills.atomic.refactored_sdk.palletizing_line_mpc.PalletizingLineMpcSkill``

**会动**：切换 MPC 模式、下发 10 段离线轨迹（双臂或单臂真实运动），最后把
**选中手臂**恢复到本脚本启动前的关节角，未选手臂保持当前角度不动。第一次请在
仿真或空载环境跑，真机需要有人的手放在急停上。

运行::

    source /opt/ros/noetic/setup.bash
    source infrastructure/ros_packages/devel/setup.bash
    export PYTHONPATH=$(pwd):$PYTHONPATH

    python3 apps/test_kuavo_5w_skills/test_palletizing_line_mpc.py                # 双臂（默认）
    python3 apps/test_kuavo_5w_skills/test_palletizing_line_mpc.py --arms left    # 只验收左臂
    python3 apps/test_kuavo_5w_skills/test_palletizing_line_mpc.py --arms right   # 只验收右臂

``--arms`` 同时决定技能参数、TF 采样范围、指标判据与收尾还原的关节范围——
未选中的手臂不采样、不参与判据、收尾时也不动。

判定口径与 kuavo-ros-control 的 ``test_palletizing_line_mpc_ik.py`` 一致：每段实测
TCP 到本段直线的最大垂距 ≤2mm、姿态相对冻结姿态漂移 ≤1°、水平段高度波动 ≤3mm；
给出箱面坐标系时再看垂直度 ≤2°。``apps/`` 不进 CI，数值是否算达标由人看日志判断——
仿真里关节跟踪滞后会让直线度超过 2mm，是「滞后」还是「真失效」需要人判断。
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from apps.test_kuavo_5w_skills._eval.line_metrics import LineMetricsRecorder
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, capture_arm_joints, parse_arms_arg, restore_selected_arms,
    run_skill, selected_arms, selected_ee_frames, skill_setup, skill_teardown)
from core.domain.enums import MPCControlMode
from skills.atomic.refactored_sdk.palletizing_line_mpc import (
    PalletizingLineMpcParams, PalletizingLineMpcSkill)

logger = get_logger(__name__)

# --- 判定阈值（与源脚本一致）---
LINE_LIMIT_MM = 2.0
ORI_LIMIT_DEG = 1.0
HEIGHT_LIMIT_MM = 3.0
PERP_LIMIT_DEG = 2.0

# --- 采样与坐标系 ---
WORLD_FRAME = 'odom'
EE_FRAMES = {'left': 'zarm_l7_end_effector', 'right': 'zarm_r7_end_effector'}
SAMPLE_RATE = 50.0
MIN_SAMPLES = 10

# --- 垂直度（可选）---
# 三个都填上才做垂直度验收，例如箱面坐标系 'box_link' + 工具轴 'z' + 箱面法向 'z'。
# 默认不测，与源脚本 --box-frame 默认关闭一致。
BOX_FRAME = None
TOOL_NORMAL_AXIS = None
BOX_NORMAL_AXIS = None

RESTORE_TIME_SEC = 5.0


def switch_to_mpc_base_arm(hardware) -> bool:
    """MPC 模式 3 + focus_ee=True + focus_z=False（源脚本的 MPC 前置）。"""
    result = hardware.set_mpc_mode(MPCControlMode.BASE_ARM)
    if not result.success:
        logger.error("切换 MPC 模式 BASE_ARM 失败: %s", result.message)
        return False
    for name, setter, value in (("focus_ee", hardware.set_focus_ee, True),
                                ("focus_z", hardware.set_focus_z, False)):
        focus = setter(value)
        if not focus.success:
            logger.error("设置 %s(%s) 失败: %s", name, value, focus.message)
            return False
    logger.info("MPC 前置完成: mode=BASE_ARM, focus_ee=True, focus_z=False")
    return True


def build_params(selection: str) -> PalletizingLineMpcParams:
    """按 ``--arms`` 选择构造技能参数（默认值保持 ``arms='both'`` 的既有行为）。"""
    return PalletizingLineMpcParams(arms=selection)


def evaluate(recorder):
    """按阈值判定，返回失败描述列表（空表示通过）。"""
    failures = []
    for (stage, arm), stats in recorder.stats.items():
        if stats.count < MIN_SAMPLES:
            failures.append(f"{stage} {arm}: 样本不足（{stats.count}）")
            continue
        if stats.line_mm > LINE_LIMIT_MM:
            failures.append(f"{stage} {arm}: 直线度 {stats.line_mm:.2f}mm > {LINE_LIMIT_MM}mm")
        if stats.ori_deg > ORI_LIMIT_DEG:
            failures.append(f"{stage} {arm}: 姿态漂移 {stats.ori_deg:.2f}° > {ORI_LIMIT_DEG}°")
        height = recorder.height_range_mm(stats)
        if height is not None and height > HEIGHT_LIMIT_MM:
            failures.append(f"{stage} {arm}: 高度波动 {height:.2f}mm > {HEIGHT_LIMIT_MM}mm")

    perpendicular = recorder.perpendicularity(BOX_FRAME, TOOL_NORMAL_AXIS, BOX_NORMAL_AXIS)
    if perpendicular is None:
        logger.info("箱面垂直度未测（未配置 BOX_FRAME / 两根轴）")
    else:
        for arm, angle in perpendicular.items():
            if angle > PERP_LIMIT_DEG:
                failures.append(f"箱面垂直度 {arm}: {angle:.2f}° > {PERP_LIMIT_DEG}°")
    return failures


def report(recorder, failures):
    logger.info("--- 实测统计（每段每臂）---")
    for line in recorder.summary():
        logger.info("  %s", line)
    if failures:
        logger.error("--- 未达标 %d 项 ---", len(failures))
        for item in failures:
            logger.error("  %s", item)
    else:
        logger.info("全部段落在阈值内")


def main():
    selection = parse_arms_arg()
    arms = selected_arms(selection)
    logger.info("验收手臂: %s（--arms %s）", ", ".join(arms), selection)
    hardware = build_hardware(whitelist=['arm', 'timed'])
    initial = []
    restored = False
    passed = False
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        try:
            initial = capture_arm_joints(hardware)
        except RuntimeError as exc:
            logger.error("%s；取消本次验收，未下发任何搬运指令", exc)

        if not initial:
            logger.error("缺少启动前关节角基准，跳过搬运")
        elif not switch_to_mpc_base_arm(hardware):
            logger.error("MPC 前置失败，跳过搬运")
        else:
            skill = PalletizingLineMpcSkill(hardware=hardware)
            ee_frames = selected_ee_frames(EE_FRAMES, arms)
            with LineMetricsRecorder(hardware, skill, WORLD_FRAME, arms, ee_frames,
                                     rate=SAMPLE_RATE) as recorder:
                run_ok = run_skill(skill, build_params(selection))
            failures = evaluate(recorder)
            report(recorder, failures)
            passed = run_ok and not failures
    finally:
        if initial:
            restored = restore_selected_arms(hardware, initial, arms, RESTORE_TIME_SEC)
        skill_teardown(hardware, need_arm=not restored)
        hardware.shutdown()

    # 还原也是验收的一部分：搬运结果再好，选中手臂没放回启动前角度就不算通过，
    # 否则「ERROR 还原失败」会和「🎉 验收通过」同时出现，操作者容易读反。
    if initial and not restored:
        logger.error("选中手臂未还原到启动前角度，已回退 skill_teardown 复位；本次验收判为未通过")
        passed = False

    if not passed:
        logger.error("❌ MPC 直线搬运验收未通过")
        sys.exit(1)
    logger.info("🎉 MPC 直线搬运验收通过")


if __name__ == '__main__':
    main()
