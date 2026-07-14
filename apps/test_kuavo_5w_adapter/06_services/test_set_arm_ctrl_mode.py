#!/usr/bin/env python3
"""
测试手臂控制模式服务调用

适配器接口: LejuWheeledArmHardware.set_arm_control_mode()
ROS 服务: /wheel_arm_change_arm_ctrl_mode (kuavo_msgs/changeArmCtrlMode)

功能说明:
- 测试3种手臂控制模式的切换
- 验证模式0（保持当前位置）、模式1（重置到初始位置）、模式2（外部控制器）
- 手臂轨迹控制执行前必须切换到模式2（外部控制）

注意事项:
1. 手臂控制模式定义:
   - 0: 保持当前位置控制 (Keep current control position)
   - 1: 重置手臂到初始目标位置 (Reset arm to initial target)
   - 2: 使用外部控制器 (Using external controller)
2. 模式2（外部控制）是手臂轨迹控制的前提条件
3. 模式切换间建议加入短暂延迟避免冲突
"""

import sys
import os
import time
import unittest

# 添加项目根目录到 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.insert(0, project_root)

import rospy
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from core.domain.result import Result


class TestSetArmCtrlMode(unittest.TestCase):
    """手臂控制模式服务调用测试类"""

    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接"""
        print("\n" + "="*70)
        print("测试套件初始化: 手臂控制模式服务调用")
        print("="*70)

        cls.hardware = LejuWheeledArmHardware(config={
            'skip_sdk_managers': True,#手臂控制模式切换不需要 SDK 管理器
            'skip_end_effector': True,#手臂控制模式切换不需要末端执行器
            'skip_camera': True,#手臂控制模式切换不需要相机
            'skip_state_manager': True,#手臂控制模式切换不需要状态管理器
            'skip_force_publishers': True,#手臂控制模式切换不需要力控发布器
        })
        result = cls.hardware.initialize()

        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")

        print("✅ 硬件初始化成功")

        # === 脚手架: 前置设置 ===
        from apps.test_kuavo_5w_adapter._scaffold import adapter_setup
        adapter_setup(cls.hardware, need_arm=False, need_torso_reset=True)

        print("\n💡 提示: 手臂控制模式服务使用 /wheel_arm_change_arm_ctrl_mode")
        print("💡 手臂控制模式定义:")
        print("   0: 保持当前位置控制")
        print("   1: 重置手臂到初始目标位置")
        print("   2: 使用外部控制器")

    @classmethod
    def tearDownClass(cls):
        """测试类清理：关闭硬件连接"""
        print("\n" + "="*70)
        print("测试套件清理: 关闭硬件连接")
        print("="*70)

        if hasattr(cls, 'hardware'):
            # === 脚手架: 后置复位 ===
            from apps.test_kuavo_5w_adapter._scaffold import adapter_teardown
            adapter_teardown(cls.hardware, need_arm=False)

            result = cls.hardware.shutdown()
            if result.success:
                print("✅ 硬件已关闭")
            else:
                print(f"⚠️  关闭警告: {result.message}")

    def setUp(self):
        """每个测试用例前的准备"""
        print(f"\n--- 开始测试: {self._testMethodName} ---")

    def tearDown(self):
        """每个测试用例后的清理"""
        print(f"--- 结束测试: {self._testMethodName} ---\n")

    def test_01_keep_pose(self):
        """测试1: 切换到保持当前位置控制模式"""
        print("  测试目标: 设置手臂控制模式为 0（保持当前位置）")

        result = self.hardware.set_arm_control_mode(0)

        self.assertTrue(result.success, f"切换到保持位置模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        print("  ✅ 保持当前位置模式设置完成")

    def test_02_auto_swing(self):
        """测试2: 切换到重置手臂到初始目标位置模式"""
        print("  测试目标: 设置手臂控制模式为 1（重置到初始位置）")
        print("  说明: 手臂将自动摆动到初始目标位置")

        result = self.hardware.set_arm_control_mode(1)

        self.assertTrue(result.success, f"切换到自动摆动模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")

        # 等待手臂运动到位
        time.sleep(2.0)
        print("  ✅ 自动摆动模式设置完成")

    def test_03_external_control(self):
        """测试3: 切换到外部控制器模式"""
        print("  测试目标: 设置手臂控制模式为 2（外部控制器）")
        print("  说明: 切换到该模式后方可接受外部手臂轨迹指令")

        result = self.hardware.set_arm_control_mode(2)

        self.assertTrue(result.success, f"切换到外部控制模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        print("  ✅ 外部控制器模式设置完成")

    def test_04_sequence(self):
        """测试4: 按顺序切换所有手臂控制模式"""
        print("  测试目标: 依次切换手臂控制模式 0 → 1 → 2 → 0")

        mode_sequence = [
            (0, "保持当前位置"),
            (1, "重置到初始位置"),
            (2, "外部控制器"),
            (0, "回到保持位置"),
        ]

        for idx, (mode, desc) in enumerate(mode_sequence, 1):
            print(f"  步骤{idx}: 切换到 {desc} (mode={mode})...")
            result = self.hardware.set_arm_control_mode(mode)

            self.assertTrue(result.success, f"步骤{idx}切换失败: {result.message}")
            print(f"  ✅ 步骤{idx}成功: {result.message}")

            if mode == 1:
                time.sleep(2.0)  # 等待手臂运动到位
            else:
                time.sleep(0.3)

        print("  ✅ 手臂控制模式序列切换完成")


def run_tests():
    """运行测试套件"""
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('test_set_arm_ctrl_mode', anonymous=True)

    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSetArmCtrlMode)

    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # 返回测试结果
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - 手臂控制模式服务调用")
    print("使用 LejuWheeledArmHardware 适配器")
    print("="*70)

    try:
        success = run_tests()

        if success:
            print("\n" + "="*70)
            print("🎉 所有测试通过！")
            print("="*70)
            sys.exit(0)
        else:
            print("\n" + "="*70)
            print("⚠️  部分测试失败")
            print("="*70)
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\n⚠️  测试被用户中断")
        sys.exit(130)
    except Exception as e:
        print(f"\n\n❌ 测试执行出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
