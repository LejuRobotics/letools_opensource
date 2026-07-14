#!/usr/bin/env python3
"""Tier 2 (test_kuavo_5w_app) 脚手架 — 适配器层通用前置/后置逻辑

脚手架纯度原则: 本模块只使用 LejuWheeledArmHardware 适配器方法，
不直接使用 ROS API 或 SDK API。

使用方式:
    class TestXxx(unittest.TestCase):
        @classmethod
        def setUpClass(cls):
            cls.hardware = LejuWheeledArmHardware()
            cls.hardware.initialize()
            adapter_setup(cls.hardware, need_arm=True, mpc_mode=MPCControlMode.ARM_ONLY)

        @classmethod
        def tearDownClass(cls):
            adapter_teardown(cls.hardware, need_arm=True)
            cls.hardware.shutdown()
"""

import time
import rospy
from core.domain.enums import MPCControlMode

__all__ = ['adapter_setup', 'adapter_teardown', 'check_services_available']


def check_services_available(service_names, timeout=3.0):
    """检查 ROS 服务是否可用，不可用时打印明确错误并返回 False

    用于在 setUpClass 中提前检测环境，避免后续调用时逐个超时。

    Args:
        service_names: 需要检查的 ROS 服务名列表
        timeout: 每个服务的等待超时（秒）

    Returns:
        (bool, list): (全部可用, 不可用的服务名列表)
    """
    unavailable = []
    for name in service_names:
        try:
            rospy.wait_for_service(name, timeout=timeout)
        except (rospy.ROSInterruptException, rospy.ROSException):
            unavailable.append(name)

    if unavailable:
        print(f"\n{'='*70}")
        print(f"❌ 环境检测失败: 以下 ROS 服务不可用")
        print(f"{'='*70}")
        for name in unavailable:
            print(f"   - {name}")
        print(f"\n💡 请确认控制器进程已启动:")
        print(f"   roslaunch humanoid_wheel_interface_ros manipulator_kuavo_s60.launch")
        print(f"{'='*70}\n")
        return False, unavailable
    return True, []


def adapter_setup(hardware, need_arm: bool = False,
                  mpc_mode: MPCControlMode = None,
                  need_torso_reset: bool = True,
                  need_focus_ee: bool = False):
    """适配器层测试前置设置

    Args:
        hardware: LejuWheeledArmHardware 实例
        need_arm: 是否需要手臂控制（重置+外部控制模式序列）
        mpc_mode: MPC 模式枚举（None 表示不设置）
        need_torso_reset: 是否重置躯干
        need_focus_ee: 是否设置末端跟踪焦点
    """
    print("\n--- 脚手架: 前置设置 ---")

    # 1. 躯干复位
    if need_torso_reset:
        result = hardware.reset_torso_to_initial()
        if result.success:
            print(f"  ✓ {result.message}")
            time.sleep(2.0)
        else:
            print(f"  ⚠ 躯干复位警告: {result.message}")

    # 2. 设置 MPC 模式（need_arm 时默认 ARM_ONLY，与参考脚本一致）
    if mpc_mode is not None or need_arm:
        mode = mpc_mode if mpc_mode is not None else MPCControlMode.ARM_ONLY
        result = hardware.set_mpc_mode(mode)
        if result.success:
            print(f"  ✓ MPC 模式设置成功 ({mode.name})")
        else:
            print(f"  ⚠ MPC 模式设置警告: {result.message}")
        time.sleep(0.5)

    # 3. 手臂控制模式序列: 重置(1) → 外部控制(2)（必须在 focus 之前）
    if need_arm:
        result = hardware.set_arm_control_mode(1)
        if result.success:
            print("  ✓ 手臂已重置到初始位置")
            time.sleep(1.0)
        result = hardware.set_arm_control_mode(2)
        if result.success:
            print("  ✓ 已切换到外部控制器模式")
        else:
            print(f"  ✗ 外部控制器模式切换失败: {result.message}")

    # 4. 设置焦点（必须在 arm_control_mode(2) 之后，与参考脚本一致）
    if need_focus_ee:
        hardware.set_focus_ee(focus_ee=True)
        hardware.set_focus_z(focus_z=False)
        print("  ✓ 焦点已设置")

    print("--- 前置设置完成 ---\n")


def adapter_teardown(hardware, need_arm: bool = False,
                     restore_mpc: bool = True):
    """适配器层测试后置复位

    Args:
        hardware: LejuWheeledArmHardware 实例
        need_arm: 是否重置手臂
        restore_mpc: 是否恢复 MPC 模式为 NoControl
    """
    print("\n--- 脚手架: 后置复位 ---")

    # 1. 手臂复位（通过 arm_reset SDK 路径）
    if need_arm:
        result = hardware.arm_reset()
        if result.success:
            print("  ✓ 手臂已复位")
            time.sleep(2.0)
        else:
            print(f"  ⚠ 手臂复位警告: {result.message}")
            # 降级: 通过模式切换复位
            hardware.set_arm_control_mode(1)
            time.sleep(2.0)

    # 2. 躯干复位
    result = hardware.reset_torso_to_initial()
    if result.success:
        print(f"  ✓ {result.message}")
        time.sleep(2.0)
    else:
        print(f"  ⚠ 躯干复位警告: {result.message}")

    # 3. 恢复 MPC 模式
    if restore_mpc:
        hardware.set_mpc_mode(MPCControlMode.NO_CONTROL)
        print("  ✓ MPC 模式已恢复为 NoControl")

    print("--- 后置复位完成 ---\n")
