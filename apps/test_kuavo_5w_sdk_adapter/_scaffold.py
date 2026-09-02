#!/usr/bin/env python3
"""Tier 4 (test_kuavo_5w_refactored) 脚手架 — Factory 层通用前置/后置逻辑

脚手架纯度原则: 本模块只使用 HardwareFactory 创建的实例方法，
不直接使用 ROS API、适配器类或 SDK。

使用方式:
    hardware = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled'})
    try:
        hardware.initialize()
        factory_setup(hardware, need_arm_reset=True)

        test_xxx(hardware)

    finally:
        factory_teardown(hardware, need_arm_reset=True)
        hardware.shutdown()
"""

import time
from core.common.logger import get_logger
from core.domain.enums import MPCControlMode

logger = get_logger(__name__)

__all__ = ['factory_setup', 'factory_teardown']


def factory_setup(hardware,
                  need_arm_reset: bool = False,
                  need_torso_reset: bool = True,
                  focus_ee: bool = None,
                  focus_z: bool = None,
                  mpc_mode: MPCControlMode = None):
    """Factory 层前置设置

    Args:
        hardware: IHardware 实例（由 HardwareFactory 创建）
        need_arm_reset: 是否将手臂复位到初始姿态
        need_torso_reset: 是否重置躯干
        focus_ee: 若不为 None，设置笛卡尔跟踪焦点（末端独立控制场景传 False=躯干优先）
        focus_z: 若不为 None，设置 Z 轴跟随焦点（通常传 False）
        mpc_mode: 若不为 None，设置指定的 MPC 控制模式；None 表示不修改
    """
    logger.info("--- 脚手架: 前置设置 ---")

    if mpc_mode is not None:
        result = hardware.set_mpc_mode(mpc_mode)
        if result.success:
            logger.info(f"已设置 MPC 模式: {mpc_mode.name}")
        else:
            logger.warning(f"设置 MPC 模式 {mpc_mode.name} 警告: {result.message}")

    if focus_ee is not None:
        result = hardware.set_focus_ee(focus_ee)
        if result.success:
            logger.info(f"已设置 focus_ee={focus_ee}（False=躯干优先，末端不可扭曲躯干）")
        else:
            logger.warning(f"设置 focus_ee 警告: {result.message}")

    if focus_z is not None:
        result = hardware.set_focus_z(focus_z)
        if result.success:
            logger.info(f"已设置 focus_z={focus_z}")
        else:
            logger.warning(f"设置 focus_z 警告: {result.message}")

    if need_torso_reset:
        result = hardware.reset_torso_to_initial()
        if result.success:
            logger.info(f"躯干已复位: {result.message}")
            time.sleep(2.0)
        else:
            logger.warning(f"躯干复位警告: {result.message}")

    if need_arm_reset:
        result = hardware.set_arm_control_mode(1)
        if result.success:
            logger.info("手臂已复位")
            time.sleep(2.0)
        else:
            logger.warning(f"手臂复位警告: {result.message}")

    # 无论是否物理复位，都确保切到外部控制模式
    # （--no-reset-arm 只跳过物理复位，不跳过模式准备；
    #   否则上次 teardown 后控制模式不明，planner 4/5/6/7 指令会被静默忽略）
    result = hardware.set_arm_control_mode(2)
    if result.success:
        logger.info("已切换到外部控制器模式")
    else:
        logger.warning(f"切换外部控制模式警告: {result.message}")

    logger.info("--- 前置设置完成 ---")


def factory_teardown(hardware,
                     need_arm_reset: bool = False,
                     need_torso_reset: bool = False):
    """Factory 层后置复位

    Args:
        hardware: IHardware 实例
        need_arm_reset: 是否将手臂复位到初始姿态
        need_torso_reset: 是否将躯干复位到初始姿态
    """
    logger.info("--- 脚手架: 后置复位 ---")

    if need_torso_reset:
        result = hardware.reset_torso_to_initial()
        if result.success:
            logger.info(f"躯干已重置: {result.message}")
            time.sleep(2.0)
        else:
            logger.warning(f"躯干复位警告: {result.message}")

    if need_arm_reset:
        result = hardware.set_arm_control_mode(1)
        if result.success:
            logger.info("手臂已复位")
            time.sleep(2.0)
        else:
            logger.warning(f"手臂复位警告: {result.message}")

    result = hardware.set_arm_control_mode(0)
    if result.success:
        logger.info("已切换到手臂保持模式")
    else:
        logger.warning(f"切换手臂保持模式警告: {result.message}")

    result = hardware.set_mpc_mode(MPCControlMode.NO_CONTROL)
    if result.success:
        logger.info("MPC 控制已释放: NO_CONTROL")
    else:
        logger.warning(f"释放 MPC 控制警告: {result.message}")

    logger.info("--- 后置复位完成 ---")
