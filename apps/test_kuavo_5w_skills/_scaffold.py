#!/usr/bin/env python3
"""skills 层测试脚手架 — 技能生命周期运行器 + 硬件构建。

纯度原则: 本模块只面向 SkillBase 子类（initialize → execute → is_finished），
不直接调用 adapter 方法——体现「技能层」测试，区别于 T4（适配器层）。

前置/后置逻辑复用 T4 已验证的 factory_setup/factory_teardown
（MPC 模式、躯干复位等安全管理），但通过各层独立 _scaffold 暴露，
不跨目录 import T4 的 _scaffold（避免 skills 层耦合适配器层测试）。

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

import time
from typing import List, Optional

from core.common.logger import get_logger
from core.domain.skill_params import SkillParams
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

__all__ = ['build_hardware', 'skill_setup', 'skill_teardown', 'run_skill']


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
