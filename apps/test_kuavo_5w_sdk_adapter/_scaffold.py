#!/usr/bin/env python3
"""Tier 4 (test_kuavo_5w_refactored) 脚手架 — Factory 层通用前置/后置逻辑

脚手架纯度原则: 本模块只使用 HardwareFactory 创建的实例方法，
不直接使用 ROS API、适配器类或 SDK。

使用方式:
    hardware = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled'})
    try:
        hardware.initialize()
        factory_setup(hardware, need_arm=True)

        test_xxx(hardware)

        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()
"""

import time
from core.common.logger import get_logger

logger = get_logger(__name__)

__all__ = ['factory_setup', 'factory_teardown']


def factory_setup(hardware, need_arm: bool = False,
                  need_torso_reset: bool = True):
    """Factory 层前置设置

    Args:
        hardware: IHardware 实例（由 HardwareFactory 创建）
        need_arm: 是否重置手臂并切换到外部控制
        need_torso_reset: 是否重置躯干
    """
    logger.info("--- 脚手架: 前置设置 ---")

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
        result = hardware.set_arm_control_mode(2)  # 外部控制
        if result.success:
            logger.info("已切换到外部控制器模式")

    logger.info("--- 前置设置完成 ---")


def factory_teardown(hardware, need_arm: bool = False):
    """Factory 层后置复位

    Args:
        hardware: IHardware 实例
        need_arm: 是否重置手臂
    """
    logger.info("--- 脚手架: 后置复位 ---")

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
