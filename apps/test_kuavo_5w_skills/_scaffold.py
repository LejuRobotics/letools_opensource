#!/usr/bin/env python3
"""skills 层测试脚手架 — 技能生命周期运行器 + 硬件构建。

纯度原则: 本模块只面向 IHardware / SkillBase 的公开接口（initialize → execute →
is_finished、关节读写），不 import 适配器实现——体现「技能层」测试，
区别于 T4（适配器层）。

前置/后置逻辑复用 T4 已验证的 factory_setup/factory_teardown
（MPC 模式、躯干复位等安全管理），但通过各层独立 _scaffold 暴露，
不跨目录 import T4 的 _scaffold（避免 skills 层耦合适配器层测试）。

另承载两个码垛直线验收入口（MPC / IK）共用的入口辅助：``--arms`` 选择、
TF 采样范围、启动前关节角读取与「只还原选中手臂」的收尾逻辑。

使用方式::

    from apps.test_kuavo_5w_skills._scaffold import build_hardware, run_skill, skill_setup, skill_teardown
    from skills.atomic.refactored_sdk.arm_reset_sdk import ArmResetSdkParams, ArmResetSdkSkill

    hardware = build_hardware(whitelist=['arm'])
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        skill = ArmResetSdkSkill(hardware=hardware)
        run_skill(skill, ArmResetSdkParams())
        skill_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()
"""

import argparse
import math
import time
from typing import Dict, List, Mapping, Optional, Sequence

from core.common.logger import get_logger
from core.domain.skill_params import SkillParams
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

__all__ = ['build_hardware', 'skill_setup', 'skill_teardown', 'run_skill',
           'parse_arms_arg', 'selected_arms', 'selected_ee_frames',
           'capture_arm_joints', 'restore_selected_arms']

# 双臂关节数组约定：左臂 7 个在前，右臂 7 个在后，共 14 个。
ARM_SIDES = ('left', 'right')
ARM_JOINTS_PER_SIDE = 7
ARM_JOINT_COUNT = ARM_JOINTS_PER_SIDE * len(ARM_SIDES)


def build_hardware(whitelist: Optional[List[str]] = None,
                   skip_end_effector: bool = True,
                   skip_camera: bool = True,
                   skip_state_manager: bool = True,
                   skip_force_publishers: bool = True):
    """构建 IHardware 实例（config 约定对齐 T4 真机脚本）。

    Args:
        whitelist: sdk_managers_whitelist（如 ['arm']、['low']）；None 表示不限制
        skip_*: 与 T4 一致的精简开关，默认全 True（只跑被测技能所需子系统）
    """
    from adapters.hardware.factory import HardwareFactory

    config = {
        'robot_type': 'leju_wheeled',
        'skip_end_effector': skip_end_effector,
        'skip_camera': skip_camera,
        'skip_state_manager': skip_state_manager,
        'skip_force_publishers': skip_force_publishers,
    }
    if whitelist is not None:
        config['sdk_managers_whitelist'] = whitelist
    return HardwareFactory.create_hardware(config=config)


def skill_setup(hardware, need_arm: bool = False, need_torso_reset: bool = True):
    """技能层前置设置（复用 T4 factory_setup 的安全逻辑）。

    Args:
        hardware: IHardware 实例（已 initialize）
        need_arm: 是否重置手臂并切换到外部控制
        need_torso_reset: 是否重置躯干
    """
    logger.info("--- 技能层: 前置设置 ---")

    if need_torso_reset:
        result = hardware.reset_torso_to_initial()
        if result.success:
            logger.info(f"躯干已重置: {result.message}")
            time.sleep(2.0)
        else:
            logger.warning(f"躯干复位警告: {result.message}")

    if need_arm:
        result = hardware.set_arm_control_mode(1)  # 重置
        if result.success:
            logger.info("手臂已重置到初始位置")
            time.sleep(1.0)

    # 切到外部控制模式（避免上次 teardown 后控制模式不明导致指令被静默忽略）
    result = hardware.set_arm_control_mode(2)
    if result.success:
        logger.info("已切换到外部控制器模式")
    else:
        logger.warning(f"切换外部控制模式警告: {result.message}")

    logger.info("--- 前置设置完成 ---")


def skill_teardown(hardware, need_arm: bool = False):
    """技能层后置复位（复用 T4 factory_teardown 的安全逻辑）。

    Args:
        hardware: IHardware 实例
        need_arm: 是否重置手臂
    """
    logger.info("--- 技能层: 后置复位 ---")

    if need_arm:
        result = hardware.arm_reset()
        if result.success:
            logger.info("手臂已复位")
            time.sleep(2.0)
        else:
            logger.warning(f"手臂复位警告: {result.message}")
            hardware.set_arm_control_mode(1)  # 降级复位
            time.sleep(2.0)

    result = hardware.reset_torso_to_initial()
    if result.success:
        logger.info(f"躯干已重置: {result.message}")
        time.sleep(2.0)
    else:
        logger.warning(f"躯干复位警告: {result.message}")

    logger.info("--- 后置复位完成 ---")


def run_skill(skill: SkillBase, params: SkillParams, tick_interval: float = 0.05) -> bool:
    """运行一个已构造技能的完整生命周期，返回是否成功。

    生命周期: initialize(params) → 循环 execute() + is_finished() 直至完成/超时。
    超时阈值取自 params.timeout（SkillBase.execute 内部亦做超时检查）。
    """
    name = getattr(params, "skill_name", skill.name)
    logger.info("--- 技能运行：%s ---", name)

    result = skill.initialize(params)
    if not result.success:
        logger.error("❌ %s initialize 失败: %s", name, result.message)
        return False

    deadline = time.time() + float(getattr(params, "timeout", 30.0))
    while not skill.is_finished():
        if time.time() > deadline:
            logger.error("❌ %s 超时（%ss）", name, getattr(params, "timeout", 30.0))
            return False
        result = skill.execute()
        if not result.success:
            logger.error("❌ %s execute 失败: %s", name, result.message)
            return False
        time.sleep(tick_interval)

    logger.info("✅ %s 完成", name)
    return True


# ----------------------------------------------------------------------
# 入口辅助：单臂选择 / 采样范围 / 关节还原
# ----------------------------------------------------------------------
def parse_arms_arg(argv: Optional[Sequence[str]] = None) -> str:
    """解析入口脚本的 ``--arms``（both / left / right，默认 both）。

    Args:
        argv: 参数列表；None 表示取 ``sys.argv[1:]``，测试可直接传列表。
    """
    parser = argparse.ArgumentParser(description="选择参与验收的手臂")
    parser.add_argument('--arms', choices=('both', 'left', 'right'), default='both',
                        help="参与验收的手臂：both（默认）/ left / right")
    return parser.parse_args(argv).arms


def selected_arms(selection: str) -> List[str]:
    """``'both'`` / ``'left'`` / ``'right'`` → 参与验收的手臂列表（顺序固定为左、右）。"""
    if selection == 'both':
        return list(ARM_SIDES)
    if selection in ARM_SIDES:
        return [selection]
    raise ValueError(f"arms 必须是 'both'/'left'/'right'，收到: {selection}")


def selected_ee_frames(ee_frames: Mapping[str, str],
                       arms: Sequence[str]) -> Dict[str, str]:
    """把完整的 EE 坐标系表裁成只含参与验收的手臂（采样与判据都据此收窄）。"""
    return {arm: ee_frames[arm] for arm in arms}


def capture_arm_joints(hardware) -> List[float]:
    """读取启动前的 14 关节角（弧度），作为结束时的还原目标。

    读不到就抛 ``RuntimeError``：没有还原基准还继续下发运动指令，结束后就放不回
    原处，这种失败必须当场让人看见，不能降级成一句警告。
    """
    result = hardware.get_arm_joint_positions()
    if not result.success:
        raise RuntimeError(f"读取启动前手臂关节角失败: {result.message}")
    joints = [float(value) for value in result.data]
    if len(joints) != ARM_JOINT_COUNT:
        raise RuntimeError(
            f"启动前关节角应为 {ARM_JOINT_COUNT} 个，收到 {len(joints)} 个")
    return joints


def restore_selected_arms(hardware, initial: Sequence[float], arms: Sequence[str],
                          time_sec: float) -> bool:
    """只把选中手臂还原为启动前角度，未选手臂保持当前角度。

    ``send_arm_joint_trajectory`` 只收 14 关节整组命令，所以先读当前 14 关节角，
    只替换选中臂的 7 个再整组下发——未选手臂的命令值就是它此刻的角度，不会被
    拽回启动前的姿态。

    Returns:
        True 表示已下发并到位；False 表示未还原（读取失败或下发失败），
        调用方应回退到 ``skill_teardown`` 的复位兜底。
    """
    if not initial:
        logger.error("没有启动前关节角，无法单独还原选中手臂")
        return False
    unknown = [arm for arm in arms if arm not in ARM_SIDES]
    if unknown:
        logger.error("未知手臂名: %s（只支持 left/right），不下发还原指令", unknown)
        return False

    current = hardware.get_arm_joint_positions()
    if not current.success:
        logger.error("还原前读取当前关节角失败: %s", current.message)
        return False
    current_joints = [float(value) for value in current.data]
    if len(current_joints) != ARM_JOINT_COUNT or len(initial) != ARM_JOINT_COUNT:
        logger.error("还原需要 %d 个关节角，启动前 %d 个 / 当前 %d 个",
                     ARM_JOINT_COUNT, len(initial), len(current_joints))
        return False

    goal = list(current_joints)
    for arm in arms:
        offset = 0 if arm == 'left' else ARM_JOINTS_PER_SIDE
        goal[offset:offset + ARM_JOINTS_PER_SIDE] = \
            initial[offset:offset + ARM_JOINTS_PER_SIDE]

    logger.info("还原选中手臂: %s（未选手臂保持当前角度）", ", ".join(arms))
    # 单位契约 —— 这里的 math.degrees 依赖两处接口约定，改换算前必须先核对：
    #   读侧 get_arm_joint_positions 返回**弧度**（sdk_control_mixin.py:539，
    #     直接把 SDK 的关节位置透出来）；
    #   写侧 send_arm_joint_trajectory 收**度**并按原值发布到 /kuavo_arm_traj
    #     （arm_control_mixin.py:234，「控制器期望角度单位（不做弧度转换）」）。
    # 两侧都不经 angle_unit 配置换算，所以改硬件单位配置不会影响这里；但若哪天真
    # 让读侧返回度、或写侧开始接受弧度，未选手臂的角度会被放大/缩小 57.3 倍下发。
    result = hardware.send_arm_joint_trajectory(
        [math.degrees(value) for value in goal], time_sec=float(time_sec))
    if not result.success:
        logger.error("恢复选中手臂关节角失败: %s", result.message)
        return False
    return True
