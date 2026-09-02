#!/usr/bin/env python3
"""orchestration 层测试脚手架 — 节点生命周期运行器 + 共享硬件配置。

纯度原则: 本模块只面向 BaseAction 子类节点（initialise → update），
节点内部通过 get_shared_hardware() 单例取硬件并驱动技能——体现「编排层」测试，
区别于 skills 层（直接调 SkillBase）和适配器层 T4（直接调 hardware 方法）。

节点不接收 hardware 参数（构造签名是 (name, label, namespace, params)），
硬件通过全局单例 get_shared_hardware() 注入；因此本 scaffold 提供
set_hardware_config() 在节点首次取硬件前覆盖配置（skip_*/whitelist），
而非像 skills 层那样传 hardware 给节点。

使用方式::

    from apps.test_kuavo_5w_orchestration._scaffold import (
        set_hardware_config, run_node, node_setup, node_teardown)
    from orchestration.nodes.arm_reset_sdk_move import ArmResetSdkMove

    set_hardware_config(whitelist=['arm'])
    node = ArmResetSdkMove("t", "手臂归位", "ns", {})
    run_node(node)  # initialise → 循环 update → SUCCESS/FAILURE
"""

import time
from typing import List, Optional

from core.common.logger import get_logger
from py_trees.common import Status

logger = get_logger(__name__)

__all__ = ['set_hardware_config', 'node_setup', 'node_teardown', 'run_node']


def set_hardware_config(whitelist: Optional[List[str]] = None,
                        skip_end_effector: bool = True,
                        skip_camera: bool = True,
                        skip_state_manager: bool = True,
                        skip_force_publishers: bool = True) -> None:
    """在节点首次取硬件前覆盖共享硬件配置（config 约定对齐 T4）。

    必须在节点 initialise()/update() 之前调用——节点内部首次
    get_shared_hardware() 时会读取此配置创建单例。

    Args:
        whitelist: sdk_managers_whitelist（如 ['arm']、['low']）；None 表示不限制
        skip_*: 与 T4 一致的精简开关，默认全 True
    """
    from orchestration.shared_hardware import set_hardware_config as _set_cfg

    config = {
        'robot_type': 'leju_wheeled',
        'skip_end_effector': skip_end_effector,
        'skip_camera': skip_camera,
        'skip_state_manager': skip_state_manager,
        'skip_force_publishers': skip_force_publishers,
    }
    if whitelist is not None:
        config['sdk_managers_whitelist'] = whitelist
    _set_cfg(config)
    logger.info("--- 编排层: 已设置共享硬件配置 whitelist=%s ---", whitelist)


def _get_shared_hardware():
    """获取共享硬件单例（首次调用触发创建+initialize）。"""
    from orchestration.shared_hardware import get_shared_hardware
    return get_shared_hardware()


def node_setup(need_arm: bool = False, need_torso_reset: bool = True) -> None:
    """编排层前置设置（复用 T4 factory_setup 的安全逻辑，作用于共享单例）。

    Args:
        need_arm: 是否重置手臂并切换到外部控制
        need_torso_reset: 是否重置躯干
    """
    hardware = _get_shared_hardware()
    logger.info("--- 编排层: 前置设置 ---")

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

    result = hardware.set_arm_control_mode(2)
    if result.success:
        logger.info("已切换到外部控制器模式")
    else:
        logger.warning(f"切换外部控制模式警告: {result.message}")

    logger.info("--- 前置设置完成 ---")


def node_teardown(need_arm: bool = False) -> None:
    """编排层后置复位（复用 T4 factory_teardown 的安全逻辑）。

    Args:
        need_arm: 是否重置手臂
    """
    hardware = _get_shared_hardware()
    logger.info("--- 编排层: 后置复位 ---")

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


def run_node(node, tick_interval: float = 0.05, max_ticks: int = 2000) -> bool:
    """运行一个已构造节点的完整生命周期，返回是否成功。

    生命周期: initialise() → 循环 update() 直至 SUCCESS/FAILURE 或达 max_ticks。
    返回 True 当且仅当 update() 返回 SUCCESS。
    """
    name = node.name
    logger.info("--- 节点运行：%s ---", name)

    node.initialise()
    for i in range(max_ticks):
        status = node.update()
        if status == Status.SUCCESS:
            logger.info("✅ %s 完成（%d ticks）", name, i + 1)
            return True
        if status == Status.FAILURE:
            logger.error("❌ %s 失败（%d ticks）: %s", name, i + 1, node.feedback_message)
            return False
        time.sleep(tick_interval)

    logger.error("❌ %s 超时（%d ticks 未达终态）", name, max_ticks)
    return False
