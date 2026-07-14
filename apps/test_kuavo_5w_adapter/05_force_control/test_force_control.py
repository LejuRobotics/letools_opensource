#!/usr/bin/env python3
"""
T2 标准力控方法测试

使用的适配器方法（标准方法，无后缀）:
- set_ee_force(side, force_kg, torque)       → /desired_ee_force/{left,right} 话题
- set_ee_force_both(left_force_kg, right_force_kg) → 同上，双手
- clear_ee_force(side)                       → 清除期望力
- enable_force_empty_detect(enable)          → /enable_force_empty_detact 话题
- set_contact_force_params(transition_time, interpolation_speed) → 服务调用

对齐源脚本:
- armContactForce/cmd_arm_force_test.py (Path A, LBForceController)
- armContactForce/ee_force_control_cli.py (Path A, 交互式 CLI)

架构约束: 使用 HardwareFactory + adapter_setup/adapter_teardown 脚手架，
不直接使用 rospy.Publisher/ServiceProxy。
"""

import sys
import os
import time

# 添加项目根目录到 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.insert(0, project_root)

from adapters.hardware.factory import HardwareFactory
from core.domain.enums import ArmSide, MPCControlMode
from apps.test_kuavo_5w_adapter._scaffold import adapter_setup, adapter_teardown


def test_left_hand_force(hardware):
    """左手 z 方向 -29.4N（-3kg）期望力 → clear"""
    print("\n--- 测试：左手 z-3kg 期望力 ---")
    result = hardware.set_ee_force(
        side=ArmSide.LEFT,
        force_kg=(0.0, 0.0, -3.0)
    )
    assert result.success, f"set_ee_force LEFT failed: {result.message}"
    print(f"  {result.message}")
    time.sleep(2.0)

    result = hardware.clear_ee_force(side=ArmSide.LEFT)
    assert result.success, f"clear_ee_force LEFT failed: {result.message}"
    print(f"  {result.message}")
    time.sleep(1.0)


def test_right_hand_force(hardware):
    """右手 z 方向 -29.4N（-3kg）期望力 → clear"""
    print("\n--- 测试：右手 z-3kg 期望力 ---")
    result = hardware.set_ee_force(
        side=ArmSide.RIGHT,
        force_kg=(0.0, 0.0, -3.0)
    )
    assert result.success, f"set_ee_force RIGHT failed: {result.message}"
    print(f"  {result.message}")
    time.sleep(2.0)

    result = hardware.clear_ee_force(side=ArmSide.RIGHT)
    assert result.success, f"clear_ee_force RIGHT failed: {result.message}"
    print(f"  {result.message}")
    time.sleep(1.0)


def test_both_hands_force(hardware):
    """双手 z 方向 -3kg 期望力 → clear"""
    print("\n--- 测试：双手 z-3kg 期望力 ---")
    result = hardware.set_ee_force_both(
        left_force_kg=(0.0, 0.0, -3.0),
        right_force_kg=(0.0, 0.0, -3.0)
    )
    assert result.success, f"set_ee_force_both failed: {result.message}"
    print(f"  {result.message}")
    time.sleep(2.0)

    result = hardware.clear_ee_force(side=ArmSide.BOTH)
    assert result.success, f"clear_ee_force BOTH failed: {result.message}"
    print(f"  {result.message}")
    time.sleep(1.0)


def test_force_empty_detect_disable_restore(hardware):
    """关闭挥空检测 → 等待 → 恢复"""
    print("\n--- 测试：挥空检测 disable → restore ---")
    result = hardware.enable_force_empty_detect(enable=False)
    assert result.success, f"disable force_empty_detect failed: {result.message}"
    print(f"  {result.message}")
    time.sleep(2.0)

    result = hardware.enable_force_empty_detect(enable=True)
    assert result.success, f"enable force_empty_detect failed: {result.message}"
    print(f"  {result.message}")
    time.sleep(1.0)


def test_contact_force_params(hardware):
    """设置接触力插值参数（服务不可用时跳过）"""
    print("\n--- 测试：接触力插值参数 ---")
    result = hardware.set_contact_force_params(
        transition_time=0.5,
        interpolation_speed=1.0
    )
    if result.success:
        print(f"  {result.message}")
    else:
        print(f"  [SKIP] 服务不可用: {result.message}")
    time.sleep(0.5)


def main():
    print("\n" + "=" * 60)
    print("Kuavo 5-W 应用层测试 - 标准力控方法 (T2)")
    print("=" * 60)

    hardware = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled'})
    try:
        hardware.initialize()
        adapter_setup(hardware, need_arm=True, mpc_mode=MPCControlMode.ARM_ONLY)

        test_left_hand_force(hardware)
        test_right_hand_force(hardware)
        test_both_hands_force(hardware)
        test_force_empty_detect_disable_restore(hardware)
        test_contact_force_params(hardware)

        print("\n✅ 标准力控方法测试完成")
        adapter_teardown(hardware, need_arm=True)
    except AssertionError as e:
        print(f"\n❌ 测试断言失败: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
