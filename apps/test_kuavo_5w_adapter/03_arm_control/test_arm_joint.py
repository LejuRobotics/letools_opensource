#!/usr/bin/env python3
"""
测试手臂关节轨迹控制（14个关节角度）

对应底层测试: test_kuavo_5w/03_arm_control/test_arm_joint.py
适配器接口: LejuWheeledArmHardware.send_arm_joint_trajectory()
ROS 话题: /kuavo_arm_traj (sensor_msgs/JointState)
反馈话题: /lb_arm_joint_reach_time/left, /lb_arm_joint_reach_time/right

功能说明:
- 测试双臂14个关节的轨迹控制
- 验证展开、弯曲、回零等动作
- 使用适配器层的 send_arm_joint_trajectory() 方法
- 自动等待到达时间反馈

注意事项:
1. **需要设置快速模式**：通过 enable_quick_mode() 控制
2. 关节角度单位：度（degrees）
3. 适配器会自动订阅到达时间反馈并等待
4. 适用于精确的关节空间控制
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


class TestArmJointTrajectory(unittest.TestCase):
    """手臂关节轨迹控制测试类"""
    
    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接"""
        print("\n" + "="*70)
        print("测试套件初始化: 手臂关节轨迹控制")
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

        # 环境检测: 检查必要的 ROS 服务是否可用
        from apps.test_kuavo_5w_adapter._scaffold import check_services_available, adapter_setup
        ok, missing = check_services_available([
            '/enable_lb_arm_quick_mode',
        ])
        if not ok:
            raise unittest.SkipTest(f"ROS 服务不可用: {missing}，请启动控制器进程")

        print("✅ 硬件初始化成功")

        # === 脚手架: 前置设置 ===
        adapter_setup(cls.hardware, need_arm=True, mpc_mode=MPCControlMode.ARM_ONLY)

        # 启用快速模式
        print("\n启用手臂快速模式...")
        quick_result = cls.hardware.enable_quick_mode(True)
        if not quick_result.success:
            print(f"⚠️  快速模式设置警告: {quick_result.message}")
        else:
            print("✅ 快速模式启用成功")
    
    @classmethod
    def tearDownClass(cls):
        """测试类清理：关闭硬件连接"""
        print("\n" + "="*70)
        print("测试套件清理: 关闭硬件连接")
        print("="*70)
        
        # 禁用快速模式
        if hasattr(cls, 'hardware'):
            # === 脚手架: 后置复位 ===
            from apps.test_kuavo_5w_adapter._scaffold import adapter_teardown
            adapter_teardown(cls.hardware, need_arm=True, restore_mpc=True)

            print("\n禁用手臂快速模式...")
            quick_result = cls.hardware.enable_quick_mode(False)
            if quick_result.success:
                print("✅ 快速模式已禁用")
            else:
                print(f"⚠️  快速模式禁用警告: {quick_result.message}")
            
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
        # 每次测试后回到零位
        try:
            zero_positions = [0.0] * 14
            result = self.hardware.send_arm_joint_trajectory(zero_positions, time_sec=2.0)
            if result.success:
                print("  ✅ 回到零位")
                time.sleep(2.5)
            else:
                print(f"  ⚠️  回零警告: {result.message}")
        except Exception as e:
            print(f"⚠️  回零时出错: {e}")
    
    def test_01_spread_arms(self):
        """测试1: 展开双臂"""
        print("  测试目标: 左右臂展开姿势")
        
        # 14个关节角度（左臂7个 + 右臂7个）
        positions = [
            -30.0, 20.0, 15.0, -45.0, 25.0, 10.0, -35.0,   # 左臂
            -30.0, -20.0, -15.0, -45.0, -25.0, -10.0, -35.0  # 右臂
        ]
        
        print(f"  目标角度: {positions}")
        
        result = self.hardware.send_arm_joint_trajectory(positions, time_sec=3.0)
        
        # 断言：检查返回结果
        self.assertTrue(result.success, f"发送关节轨迹指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        # 等待运动完成（适配器内部会等待，这里额外等待确保稳定）
        time.sleep(0.5)
        print("  ✅ 展开双臂完成")
    
    def test_02_bend_arms(self):
        """测试2: 弯曲收回双臂"""
        print("  测试目标: 双臂弯曲收回姿势")
        
        # 14个关节角度（左臂7个 + 右臂7个）
        positions = [
            -20.0, 30.0, -25.0, -20.0, 40.0, -15.0, 25.0,   # 左臂
            -20.0, -30.0, 25.0, -20.0, -40.0, 15.0, 25.0     # 右臂
        ]
        
        print(f"  目标角度: {positions}")
        
        result = self.hardware.send_arm_joint_trajectory(positions, time_sec=3.0)
        
        # 断言：检查返回结果
        self.assertTrue(result.success, f"发送关节轨迹指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        # 等待运动完成
        time.sleep(0.5)
        print("  ✅ 弯曲收回完成")
    
    def test_03_zero_position(self):
        """测试3: 回到零位"""
        print("  测试目标: 所有关节回到零位")
        
        # 14个关节都设为0度
        zero_positions = [0.0] * 14
        
        print(f"  目标角度: {zero_positions}")
        
        result = self.hardware.send_arm_joint_trajectory(zero_positions, time_sec=2.0)
        
        # 断言：检查返回结果
        self.assertTrue(result.success, f"发送零位指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        # 等待运动完成
        time.sleep(0.5)
        print("  ✅ 回到零位完成")


def run_tests():
    """运行测试套件"""
    if not rospy.core.is_initialized():
        rospy.init_node('test_arm_joint_trajectory', anonymous=True)
    
    suite = unittest.TestLoader().loadTestsFromTestCase(TestArmJointTrajectory)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - 手臂关节轨迹控制")
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
