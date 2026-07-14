#!/usr/bin/env python3
"""
测试 MPC 模式服务调用

对应底层参考: test_kuavo_wheel_real/lb_ctrl_api.py::set_control_mode()
适配器接口: LejuWheeledArmHardware.set_mpc_mode()
ROS 服务: /mobile_manipulator_mpc_control (kuavo_msgs/changeTorsoCtrlMode)

功能说明:
- 测试5种MPC控制模式的切换
- 验证模式切换的响应和状态
- 使用适配器层的 set_mpc_mode() 方法
- 使用枚举类型 MPCControlMode

注意事项:
1. MPC模式定义:
   - NO_CONTROL (0): 无控制
   - ARM_ONLY (1): 仅控制手臂，基座固定
   - BASE_ONLY (2): 仅控制基座，手臂固定
   - BASE_ARM (3): 同时控制基座和手臂
   - ARM_EE_ONLY (4): 仅控制手臂末端
2. 模式切换是互斥的，同一时间只能有一个模式生效
3. 建议在手臂/底盘控制前设置正确的MPC模式
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
from core.domain.enums import MPCControlMode


class TestSetMpcMode(unittest.TestCase):
    """MPC模式服务调用测试类"""
    
    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接"""
        print("\n" + "="*70)
        print("测试套件初始化: MPC模式服务调用")
        print("="*70)

        cls.hardware = LejuWheeledArmHardware(config={
            'skip_sdk_managers': True,
            'skip_end_effector': True,
            'skip_camera': True,
            'skip_state_manager': True,
            'skip_force_publishers': True,
        })
        result = cls.hardware.initialize()

        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")

        print("✅ 硬件初始化成功")

        # === 脚手架: 前置设置 ===
        from apps.test_kuavo_5w_adapter._scaffold import adapter_setup
        adapter_setup(cls.hardware, need_arm=False, mpc_mode=None, need_torso_reset=True)

        print("\n💡 提示: MPC模式服务使用 /mobile_manipulator_mpc_control")
        print("💡 MPC模式定义:")
        print("   0: NO_CONTROL - 无控制")
        print("   1: ARM_ONLY - 仅控制手臂")
        print("   2: BASE_ONLY - 仅控制基座")
        print("   3: BASE_ARM - 同时控制基座和手臂")
        print("   4: ARM_EE_ONLY - 仅控制手臂末端")

    @classmethod
    def tearDownClass(cls):
        """测试类清理：关闭硬件连接"""
        print("\n" + "="*70)
        print("测试套件清理: 关闭硬件连接")
        print("="*70)

        if hasattr(cls, 'hardware'):
            # === 脚手架: 后置复位 ===
            from apps.test_kuavo_5w_adapter._scaffold import adapter_teardown
            adapter_teardown(cls.hardware, need_arm=False, restore_mpc=False)

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
    
    def test_01_mpc_no_control(self):
        """测试1: 切换到无控制模式"""
        print("  测试目标: 切换到 NO_CONTROL 模式")
        print("  模式值: 0")
        
        result = self.hardware.set_mpc_mode(MPCControlMode.NO_CONTROL)
        
        # 断言
        self.assertTrue(result.success, f"切换到NO_CONTROL模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ NO_CONTROL 模式设置完成")
    
    def test_02_mpc_arm_only(self):
        """测试2: 切换到仅手臂控制模式"""
        print("  测试目标: 切换到 ARM_ONLY 模式")
        print("  模式值: 1")
        print("  说明: 仅控制手臂，基座固定")
        
        result = self.hardware.set_mpc_mode(MPCControlMode.ARM_ONLY)
        
        self.assertTrue(result.success, f"切换到ARM_ONLY模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ ARM_ONLY 模式设置完成")
    
    def test_03_mpc_base_only(self):
        """测试3: 切换到仅基座控制模式"""
        print("  测试目标: 切换到 BASE_ONLY 模式")
        print("  模式值: 2")
        print("  说明: 仅控制基座，手臂固定")
        
        result = self.hardware.set_mpc_mode(MPCControlMode.BASE_ONLY)
        
        self.assertTrue(result.success, f"切换到BASE_ONLY模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ BASE_ONLY 模式设置完成")
    
    def test_04_mpc_base_arm(self):
        """测试4: 切换到基座+手臂控制模式"""
        print("  测试目标: 切换到 BASE_ARM 模式")
        print("  模式值: 3")
        print("  说明: 同时控制基座和手臂")
        
        result = self.hardware.set_mpc_mode(MPCControlMode.BASE_ARM)
        
        self.assertTrue(result.success, f"切换到BASE_ARM模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ BASE_ARM 模式设置完成")
    
    def test_05_mpc_arm_ee_only(self):
        """测试5: 切换到仅手臂末端控制模式"""
        print("  测试目标: 切换到 ARM_EE_ONLY 模式")
        print("  模式值: 4")
        print("  说明: 仅控制手臂末端（笛卡尔空间）")
        
        result = self.hardware.set_mpc_mode(MPCControlMode.ARM_EE_ONLY)
        
        self.assertTrue(result.success, f"切换到ARM_EE_ONLY模式失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ ARM_EE_ONLY 模式设置完成")
    
    def test_06_mpc_mode_sequence(self):
        """测试6: 按顺序切换所有MPC模式"""
        print("  测试目标: 依次切换所有MPC模式")
        
        # 定义模式序列
        mode_sequence = [
            (MPCControlMode.NO_CONTROL, "无控制"),
            (MPCControlMode.ARM_ONLY, "仅手臂"),
            (MPCControlMode.BASE_ONLY, "仅基座"),
            (MPCControlMode.BASE_ARM, "基座+手臂"),
            (MPCControlMode.ARM_EE_ONLY, "仅手臂末端"),
            (MPCControlMode.ARM_ONLY, "回到仅手臂"),  # 最后回到常用模式
        ]
        
        for idx, (mode, desc) in enumerate(mode_sequence, 1):
            print(f"  步骤{idx}: 切换到 {desc} (mode={mode.value})...")
            result = self.hardware.set_mpc_mode(mode)
            
            self.assertTrue(result.success, f"步骤{idx}切换失败: {result.message}")
            print(f"  ✅ 步骤{idx}成功: {result.message}")
            time.sleep(0.3)  # 短暂延迟
        
        print("  ✅ MPC模式序列切换完成")


def run_tests():
    """运行测试套件"""
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('test_set_mpc_mode', anonymous=True)
    
    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSetMpcMode)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - MPC模式服务调用")
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
