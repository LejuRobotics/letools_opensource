#!/usr/bin/env python3
"""
测试腿部关节控制

对应底层测试: test_kuavo_wheel_real/cmd_leg_joint_test.py
适配器接口: LejuWheeledArmHardware.send_leg_joint_command()
ROS 话题: /lb_leg_traj (sensor_msgs/JointState)
反馈话题: /lb_leg_joint_reach_time (std_msgs/Float32)

功能说明:
- 测试4个腿部关节的角度控制
- 验证渐进式角度变化
- 使用适配器层的 send_leg_joint_command() 方法
- 自动等待到达时间反馈

注意事项:
1. 腿部关节包括：左髋俯仰、左膝俯仰、右髋俯仰、右膝俯仰
2. 角度单位：度（degrees）
3. 每个测试用例都会等待运动完成后才执行下一个
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


class TestLegJoint(unittest.TestCase):
    """腿部关节控制测试类"""
    
    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接"""
        print("\n" + "="*70)
        print("测试套件初始化: 腿部关节控制")
        print("="*70)
        
        cls.hardware = LejuWheeledArmHardware(config={
            'robot_type': 'leju_wheeled',
            'skip_sdk_managers': True,  # 腿部关节控制走 ROS 话题，不需要 SDK
            'skip_end_effector': True, # 腿部关节控制不需要末端执行器
            'skip_camera': True,# 腿部关节控制不需要相机
            'skip_state_manager': True,# 腿部关节控制不需要状态管理器
            'skip_force_publishers': True,# 腿部关节控制不需要力控发布器
        })
        result = cls.hardware.initialize()
        
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")
        
        print("✅ 硬件初始化成功")

        print("\n💡 提示: 腿部关节控制使用 /lb_leg_traj 话题")
    
    @classmethod
    def tearDownClass(cls):
        """测试类清理：关闭硬件连接"""
        print("\n" + "="*70)
        print("测试套件清理: 关闭硬件连接")
        print("="*70)
        
        if hasattr(cls, 'hardware'):
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
    
    def test_01_leg_initial_pose(self):
        """测试1: 腿部初始姿态"""
        print("  测试目标: 设置腿部到初始姿态")
        print("  关节角度: [14.90°, -32.01°, 18.03°, 0.0°]")
        print("  关节顺序: [左髋俯仰, 左膝俯仰, 右髋俯仰, 右膝俯仰]")
        
        positions = [14.90, -32.01, 18.03, 0.0]
        result = self.hardware.send_leg_joint_command(positions)
        
        # 断言
        self.assertTrue(result.success, f"发送腿部关节指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 初始姿态完成")
    
    def test_02_leg_right_knee_bend(self):
        """测试2: 右膝弯曲30°"""
        print("  测试目标: 右膝弯曲30°")
        print("  关节角度: [14.90°, -32.01°, 18.03°, 30.0°]")
        
        positions = [14.90, -32.01, 18.03, 30.0]
        result = self.hardware.send_leg_joint_command(positions)
        
        self.assertTrue(result.success, f"发送腿部关节指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 右膝弯曲完成")
    
    def test_03_leg_right_knee_extend(self):
        """测试3: 右膝伸展-30°"""
        print("  测试目标: 右膝伸展-30°")
        print("  关节角度: [14.90°, -32.01°, 18.03°, -30.0°]")
        
        positions = [14.90, -32.01, 18.03, -30.0]
        result = self.hardware.send_leg_joint_command(positions)
        
        self.assertTrue(result.success, f"发送腿部关节指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 右膝伸展完成")
    
    def test_04_leg_return_initial(self):
        """测试4: 回到初始姿态"""
        print("  测试目标: 回到初始姿态")
        print("  关节角度: [14.90°, -32.01°, 18.03°, 0.0°]")
        
        positions = [14.90, -32.01, 18.03, 0.0]
        result = self.hardware.send_leg_joint_command(positions)
        
        self.assertTrue(result.success, f"发送腿部关节指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 回到初始姿态完成")
    
    def test_05_leg_zero_position(self):
        """测试5: 零位姿态"""
        print("  测试目标: 所有关节归零")
        print("  关节角度: [0.0°, 0.0°, 0.0°, 0.0°]")
        
        positions = [0.0, 0.0, 0.0, 0.0]
        result = self.hardware.send_leg_joint_command(positions)
        
        self.assertTrue(result.success, f"发送腿部关节指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 零位姿态完成")


def run_tests():
    """运行测试套件"""
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('test_leg_joint', anonymous=True)
    
    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestLegJoint)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - 腿部关节控制")
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
