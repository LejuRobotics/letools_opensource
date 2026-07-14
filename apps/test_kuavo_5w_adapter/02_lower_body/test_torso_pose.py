#!/usr/bin/env python3
"""
测试躯干位姿控制

对应底层测试: test_kuavo_wheel_real/cmd_torso_pose_test.py
适配器接口: LejuWheeledArmHardware.send_torso_pose()
ROS 话题: /cmd_lb_torso_pose (geometry_msgs/Twist)
反馈话题: /lb_torso_pose_reach_time (std_msgs/Float32)

功能说明:
- 测试躯干相对基座的位姿控制（x, z, pitch, yaw）
- 验证渐进式位姿变化（抬高、前移、偏航、俯仰）
- 使用适配器层的 send_torso_pose() 方法
- 自动等待到达时间反馈

注意事项:
1. 躯干控制需要关闭Z轴焦点跟踪（set_focus_z(False)）
2. 建议先重置躯干到初始位置
3. 位姿数据是绝对坐标（相对于base_link坐标系），非增量
4. 角度单位：弧度（rad），与底层 Twist 消息一致
"""

import sys
import os
import time
import unittest
import math

# 添加项目根目录到 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.insert(0, project_root)

import rospy
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from core.domain.pose import Pose6D
from core.domain.result import Result
from core.domain.enums import MPCControlMode


class TestTorsoPose(unittest.TestCase):
    """躯干位姿控制测试类"""
    
    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接，重置躯干"""
        print("\n" + "="*70)
        print("测试套件初始化: 躯干位姿控制")
        print("="*70)

        cls.hardware = LejuWheeledArmHardware(config={
            'robot_type': 'leju_wheeled',
            'skip_sdk_managers': True,  # 躯干控制走 ROS 话题/服务，不需要 SDK
            'skip_end_effector': True, # 躯干控制不需要末端执行器
            'skip_camera': True,# 躯干控制不需要相机
            'skip_state_manager': True,# 躯干控制不需要状态管理器
            'skip_force_publishers': True,# 躯干控制不需要力控发布器
        })
        result = cls.hardware.initialize()

        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")

        print("✅ 硬件初始化成功")

        # 设置MPC模式（与T1脚本 setup_before_test(need_mpc_mode=3) 一致）
        cls.hardware.set_mpc_mode(MPCControlMode.BASE_ARM)
        time.sleep(0.5)

        # 躯干复位
        reset_result = cls.hardware.reset_torso_to_initial()
        if reset_result.success:
            print(f"  ✓ 躯干已复位: {reset_result.message}")
        time.sleep(2.5)

        # 关闭Z轴焦点跟踪（与源脚本 cmd_torso_pose_test.py 一致）
        cls.hardware.set_focus_z(False)
        time.sleep(0.3)

        # 获取躯干初始位姿（动态获取，与源脚本一致）
        pose_result = cls.hardware.get_torso_initial_pose()
        if pose_result.success:
            cls.initial_torso_pose = pose_result.data  # {'position': [x,y,z], 'euler': [yaw,pitch,roll]}
            print(f"  ✓ 初始位姿: 位置={cls.initial_torso_pose['position']}, 欧拉角={cls.initial_torso_pose['euler']}")
        else:
            raise RuntimeError(f"无法获取躯干初始位姿: {pose_result.message}")

        print("\n💡 提示: 躯干位姿使用 /cmd_lb_torso_pose 话题")
    
    @classmethod
    def tearDownClass(cls):
        """测试类清理：恢复Z轴焦点，恢复MPC模式，关闭硬件连接"""
        print("\n" + "="*70)
        print("测试套件清理: 恢复Z轴焦点，恢复MPC模式，关闭硬件连接")
        print("="*70)

        if hasattr(cls, 'hardware'):
            # 恢复Z轴焦点
            cls.hardware.set_focus_z(True)
            time.sleep(0.3)
            # 恢复MPC模式为 NoControl
            cls.hardware.set_mpc_mode(MPCControlMode.NO_CONTROL)
            time.sleep(0.5)
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
    
    def test_01_torso_lift(self):
        """测试1: 躯干抬高0.3m"""
        init = self.initial_torso_pose['position']
        pose = Pose6D(x=init[0] + 0.0, y=0.0, z=init[2] + 0.3, yaw=0.0, pitch=0.0, roll=0.0)
        print(f"  绝对位姿: x={pose.x:.3f}m, z={pose.z:.3f}m, pitch=0rad, yaw=0rad")

        result = self.hardware.send_torso_pose(pose)
        
        # 断言
        self.assertTrue(result.success, f"发送躯干位姿指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 躯干抬高完成")
    
    def test_02_torso_forward(self):
        """测试2: 躯干前移0.2m（保持抬高）"""
        init = self.initial_torso_pose['position']
        pose = Pose6D(x=init[0] + 0.2, y=0.0, z=init[2] + 0.3, yaw=0.0, pitch=0.0, roll=0.0)
        print(f"  绝对位姿: x={pose.x:.3f}m, z={pose.z:.3f}m, pitch=0rad, yaw=0rad")

        result = self.hardware.send_torso_pose(pose)
        
        self.assertTrue(result.success, f"发送躯干位姿指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 躯干前移完成")
    
    def test_03_torso_yaw_positive(self):
        """测试3: 偏航+30°"""
        init = self.initial_torso_pose['position']
        # 30° = π/6 ≈ 0.52356 rad
        pose = Pose6D(x=init[0] + 0.2, y=0.0, z=init[2] + 0.3, yaw=0.52356, pitch=0.0, roll=0.0)
        print(f"  绝对位姿: x={pose.x:.3f}m, z={pose.z:.3f}m, yaw=+0.52356rad")

        result = self.hardware.send_torso_pose(pose)
        
        self.assertTrue(result.success, f"发送躯干位姿指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 偏航+30°完成")
    
    def test_04_torso_yaw_negative(self):
        """测试4: 偏航-30°"""
        init = self.initial_torso_pose['position']
        # -30° = -π/6 ≈ -0.52356 rad
        pose = Pose6D(x=init[0] + 0.2, y=0.0, z=init[2] + 0.3, yaw=-0.52356, pitch=0.0, roll=0.0)
        print(f"  绝对位姿: x={pose.x:.3f}m, z={pose.z:.3f}m, yaw=-0.52356rad")

        result = self.hardware.send_torso_pose(pose)
        
        self.assertTrue(result.success, f"发送躯干位姿指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 偏航-30°完成")
    
    def test_05_torso_pitch_negative(self):
        """测试5: 俯仰-10°"""
        init = self.initial_torso_pose['position']
        # -10° ≈ -0.1745 rad
        pose = Pose6D(x=init[0] + 0.2, y=0.0, z=init[2] + 0.3, yaw=0.0, pitch=-0.1745, roll=0.0)
        print(f"  绝对位姿: x={pose.x:.3f}m, z={pose.z:.3f}m, pitch=-0.1745rad")

        result = self.hardware.send_torso_pose(pose)
        
        self.assertTrue(result.success, f"发送躯干位姿指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 俯仰-10°完成")
    
    def test_06_torso_pitch_positive(self):
        """测试6: 俯仰+30°"""
        init = self.initial_torso_pose['position']
        # 30° ≈ 0.524 rad
        pose = Pose6D(x=init[0] + 0.2, y=0.0, z=init[2] + 0.3, yaw=0.0, pitch=0.524, roll=0.0)
        print(f"  绝对位姿: x={pose.x:.3f}m, z={pose.z:.3f}m, pitch=+0.524rad")

        result = self.hardware.send_torso_pose(pose)
        
        self.assertTrue(result.success, f"发送躯干位姿指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 俯仰+30°完成")
    
    def test_07_torso_reset(self):
        """测试7: 复位到初始位置"""
        init = self.initial_torso_pose['position']
        pose = Pose6D(x=init[0], y=0.0, z=init[2], yaw=0.0, pitch=0.0, roll=0.0)
        print(f"  绝对位姿: x={pose.x:.3f}m, z={pose.z:.3f}m, pitch=0rad, yaw=0rad")

        result = self.hardware.send_torso_pose(pose)
        
        self.assertTrue(result.success, f"发送躯干位姿指令失败: {result.message}")
        print(f"  ✅ 指令成功: {result.message}")
        
        print("  ✅ 复位完成")


def run_tests():
    """运行测试套件"""
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('test_torso_pose', anonymous=True)
    
    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestTorsoPose)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - 躯干位姿控制")
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
