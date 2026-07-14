#!/usr/bin/env python3
"""JiBot 适配器层测试脚手架 — 前置/后置逻辑

JiBot 底盘测试无需手臂/躯干/MPC，只需检查底盘服务可用性。

使用方式:
    class TestXxx(unittest.TestCase):
        @classmethod
        def setUpClass(cls):
            cls.hardware = LejuWheeledArmHardware(config={'skip_sdk_managers': True})
            cls.hardware.initialize()
            jibot_setup()

        @classmethod
        def tearDownClass(cls):
            cls.hardware.shutdown()
"""

import rospy

__all__ = ['jibot_setup', 'check_jibot_services']


def check_jibot_services(timeout=5.0):
    """检查 JiBot 底盘 ROS 服务是否可用。

    依赖底盘机 Jarvis 服务已启动：
    - /move_base/base_move
    - /move_base/check_arrived
    - /move_base/move_to_target

    Args:
        timeout: 每个服务的等待超时（秒）

    Returns:
        (bool, list): (全部可用, 不可用的服务名列表)
    """
    required_services = [
        "/move_base/base_move",
        "/move_base/check_arrived",
        "/move_base/move_to_target",
    ]

    unavailable = []
    for name in required_services:
        try:
            rospy.wait_for_service(name, timeout=timeout)
        except (rospy.ROSInterruptException, rospy.ROSException):
            unavailable.append(name)

    if unavailable:
        print(f"\n{'='*70}")
        print(f"❌ 环境检测失败: 以下 JiBot 底盘服务不可用")
        print(f"{'='*70}")
        for name in unavailable:
            print(f"   - {name}")
        print(f"\n💡 请确认底盘 Jarvis 服务已启动:")
        print(f"   1. 底盘机 ROS_MASTER_URI 指向本机")
        print(f"   2. 在底盘机执行: sudo systemctl restart urobot.service")
        print(f"{'='*70}\n")
        return False, unavailable
    return True, []


def jibot_setup():
    """JiBot 适配器层测试前置设置。

    检查 JiBot 底盘 ROS 服务可用性，不操作手臂/躯干/MPC。
    """
    print("\n--- JiBot 脚手架: 前置设置 ---")

    # 检查底盘服务
    ok, unavailable = check_jibot_services(timeout=5.0)
    if not ok:
        names = ", ".join(unavailable)
        raise EnvironmentError(f"JiBot 底盘服务不可用: {names}")

    print("  ✓ JiBot 底盘服务全部可用 (/move_base/*)")
    print("--- 前置设置完成 ---\n")
