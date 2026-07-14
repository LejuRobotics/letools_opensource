#!/usr/bin/env python3
"""
测试快速模式服务调用

对应底层测试: wheel_arm_demo/cmd_arm_quickMode_test.py
适配器接口: LejuWheeledArmHardware.enable_quick_mode()
ROS 服务: /enable_lb_arm_quick_mode (kuavo_msgs/changeLbQuickModeSrv)

功能说明:
- 测试启用/禁用快速模式
- 验证快速模式对关节轨迹的影响
- 使用适配器层的 enable_quick_mode() 方法

注意事项:
1. 快速模式类型: 0=关闭, 1=下肢快, 2=上肢快, 3=上下肢快
2. 本测试使用模式3（上下肢快速）
3. 建议在手臂轨迹控制前启用快速模式
4. 测试结束后需要禁用快速模式
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


class TestEnableQuickMode(unittest.TestCase):
    """快速模式服务调用测试类"""
    
    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接"""
        print("\n" + "="*70)
        print("测试套件初始化: 快速模式服务调用")
        print("="*70)

        cls.hardware = LejuWheeledArmHardware(config={
            'skip_sdk_managers': True,# 快速模式服务调用不需要 SDK 管理器
            'skip_end_effector': True,# 快速模式服务调用不需要末端执行器
            'skip_camera': True,# 快速模式服务调用不需要相机
            'skip_state_manager': True,# 快速模式服务调用不需要状态管理器
            'skip_force_publishers': True,# 快速模式服务调用不需要力控发布器
        })
        result = cls.hardware.initialize()

        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")

        print("✅ 硬件初始化成功")

        # === 脚手架: 前置设置 ===
        from apps.test_kuavo_5w_adapter._scaffold import adapter_setup
        adapter_setup(cls.hardware, need_arm=False, need_torso_reset=False, mpc_mode=None)

        print("\n💡 提示: 快速模式服务使用 /enable_lb_arm_quick_mode")

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
    
    def test_01_enable_quick_mode(self):
        """测试1: 启用快速模式"""
        print("  测试目标: 启用上下肢快速模式")
        print("  快速模式值: 3 (上下肢快)")
        
        result = self.hardware.enable_quick_mode(enable=True)
        
        # 断言
        self.assertTrue(result.success, f"启用快速模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 快速模式已启用")
    
    def test_02_disable_quick_mode(self):
        """测试2: 禁用快速模式"""
        print("  测试目标: 禁用快速模式")
        print("  快速模式值: 0 (关闭)")
        
        result = self.hardware.enable_quick_mode(enable=False)
        
        self.assertTrue(result.success, f"禁用快速模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 快速模式已禁用")
    
    def test_03_toggle_quick_mode(self):
        """测试3: 切换快速模式（启用→禁用→启用）"""
        print("  测试目标: 多次切换快速模式")
        
        # 第一次：启用
        print("  步骤1: 启用快速模式...")
        result1 = self.hardware.enable_quick_mode(enable=True)
        self.assertTrue(result1.success, f"第1次启用失败: {result1.message}")
        print(f"  ✅ 第1次启用成功: {result1.message}")
        time.sleep(0.5)
        
        # 第二次：禁用
        print("  步骤2: 禁用快速模式...")
        result2 = self.hardware.enable_quick_mode(enable=False)
        self.assertTrue(result2.success, f"禁用失败: {result2.message}")
        print(f"  ✅ 禁用成功: {result2.message}")
        time.sleep(0.5)
        
        # 第三次：再次启用
        print("  步骤3: 再次启用快速模式...")
        result3 = self.hardware.enable_quick_mode(enable=True)
        self.assertTrue(result3.success, f"第2次启用失败: {result3.message}")
        print(f"  ✅ 第2次启用成功: {result3.message}")
        time.sleep(0.5)
        
        # 最后：禁用以恢复初始状态
        print("  步骤4: 最终禁用快速模式...")
        result4 = self.hardware.enable_quick_mode(enable=False)
        self.assertTrue(result4.success, f"最终禁用失败: {result4.message}")
        print(f"  ✅ 最终禁用成功: {result4.message}")
        
        print("  ✅ 快速模式切换测试完成")


def run_tests():
    """运行测试套件"""
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('test_enable_quick_mode', anonymous=True)
    
    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestEnableQuickMode)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - 快速模式服务调用")
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
