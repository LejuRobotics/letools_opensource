#!/usr/bin/env python3
"""
测试手臂关节空间控制（14个关节角度）

对应底层测试: test_kuavo_5w/03_arm_control/test_arm_ee_joint.py
适配器接口: LejuWheeledArmHardware.send_ee_pose() + 关节空间模式
ROS 话题: /mm/two_arm_hand_pose_cmd (kuavo_msgs/twoArmHandPoseCmd, frame=5)
反馈话题: /lb_arm_joint_reach_time/left, /lb_arm_joint_reach_time/right

功能说明:
- 测试关节空间模式下的双臂控制（frame=5）
- 验证左右臂各7个关节角度的直接控制
- 使用适配器层的 send_ee_pose() 方法（需要扩展支持关节空间）
- 自动等待到达时间反馈

注意事项:
1. 源脚本(cmd_arm_ee_joint_test.py)不设置 MPC 模式，控制器内部自动管理
2. **必须设置手臂控制模式**: Mode 2（外部控制），不需要先 Mode 1
3. **frame=5 表示关节空间数据**，位置和姿态信息被忽略
4. 关节角度单位：度（degrees）
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
from core.domain.enums import FrameType
from core.domain.result import Result


class TestArmJointSpace(unittest.TestCase):
    """手臂关节空间控制测试类"""
    
    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接"""
        print("\n" + "="*70)
        print("测试套件初始化: 手臂关节空间控制")
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
        from apps.test_kuavo_5w_adapter._scaffold import check_services_available
        ok, missing = check_services_available([
            '/wheel_arm_change_arm_ctrl_mode',
        ])
        if not ok:
            raise unittest.SkipTest(f"ROS 服务不可用: {missing}，请启动控制器进程")

        print("✅ 硬件初始化成功")

        # === 前置设置（与源脚本 cmd_arm_ee_joint_test.py 一致）===
        # 源脚本时序：创建 Publisher → sleep(1.0) → set_arm_control_mode(2)
        # 关键：Publisher 必须在 set_arm_control_mode(2) 之前创建并等待连接
        cls.hardware._ensure_ee_publisher()

        # 源脚本只设置 arm_control_mode(2)，不设置 MPC 模式、不做躯干复位
        cls.hardware.set_arm_control_mode(2)
        time.sleep(0.5)
    
    @classmethod
    def tearDownClass(cls):
        """测试类清理：关闭硬件连接"""
        print("\n" + "="*70)
        print("测试套件清理: 关闭硬件连接")
        print("="*70)
        
        if hasattr(cls, 'hardware'):
            # 源脚本无 teardown，这里仅做安全关闭
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
        try:
            result = self.hardware.send_arm_ee_joint_space([0.0]*7, [0.0]*7)
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

        left_joints  = [-30.0, 20.0, 15.0, -45.0, 25.0, 10.0, -35.0]
        right_joints = [-30.0, -20.0, -15.0, -45.0, -25.0, -10.0, -35.0]

        print(f"  左臂关节: {left_joints}")
        print(f"  右臂关节: {right_joints}")

        result = self.hardware.send_arm_ee_joint_space(left_joints, right_joints)

        self.assertTrue(result.success, f"发送关节角度指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        time.sleep(3.5)
        print("  ✅ 展开双臂完成")

    def test_02_bend_arms(self):
        """测试2: 弯曲收回双臂"""
        print("  测试目标: 双臂弯曲收回姿势")

        left_joints  = [-20.0, 30.0, -25.0, -20.0, 40.0, -15.0, 25.0]
        right_joints = [-20.0, -30.0, 25.0, -20.0, -40.0, 15.0, 25.0]

        print(f"  左臂关节: {left_joints}")
        print(f"  右臂关节: {right_joints}")

        result = self.hardware.send_arm_ee_joint_space(left_joints, right_joints)

        self.assertTrue(result.success, f"发送关节角度指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        time.sleep(3.5)
        print("  ✅ 弯曲收回完成")

    def test_03_zero_position(self):
        """测试3: 回到零位"""
        print("  测试目标: 所有关节回到零位")

        result = self.hardware.send_arm_ee_joint_space([0.0]*7, [0.0]*7)

        self.assertTrue(result.success, f"发送零位指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        time.sleep(2.5)
        print("  ✅ 回到零位完成")


def run_tests():
    """运行测试套件"""
    if not rospy.core.is_initialized():
        rospy.init_node('test_arm_joint_space', anonymous=True)
    
    suite = unittest.TestLoader().loadTestsFromTestCase(TestArmJointSpace)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - 手臂关节空间控制")
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
