#!/usr/bin/env python3
"""
测试手臂末端位姿控制（世界坐标系）

对应底层测试: test_kuavo_5w/03_arm_control/test_arm_ee_world.py
适配器接口: LejuWheeledArmHardware.send_ee_pose(frame=FrameType.WORLD)
ROS 话题: /mm/two_arm_hand_pose_cmd (kuavo_msgs/twoArmHandPoseCmd)
反馈话题: /lb_arm_ee_reach_time (std_msgs/Float32)

功能说明:
- 测试世界坐标系下的双臂末端位姿控制
- 验证渐进式位姿变化（从近到远、从低到高）
- 使用适配器层的 send_ee_pose() 方法
- 自动等待到达时间反馈

注意事项:
1. 源脚本(cmd_arm_ee_world_test.py)不设置 MPC 模式，控制器内部自动管理
2. **必须设置手臂控制模式**：先 Mode 1（重置），再 Mode 2（外部控制）
3. pitch=-90° 是手臂向前伸展的标准姿态，避免奇异点
4. 必须设置末端跟踪焦点(focus_ee=True)
"""

import sys
import os
import time
import unittest
import numpy as np

# 添加项目根目录到 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.insert(0, project_root)

import rospy
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from core.domain.pose import Pose6D
from core.domain.enums import FrameType, ArmSide
from core.domain.result import Result


class TestArmEEWorld(unittest.TestCase):
    """手臂末端位姿控制测试类（世界坐标系）"""
    
    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接，设置 MPC 和手臂控制模式"""
        print("\n" + "="*70)
        print("测试套件初始化: 手臂末端位姿控制（世界坐标系）")
        print("="*70)
        
        cls.hardware = LejuWheeledArmHardware(config={
            'skip_sdk_managers': True,# 手臂末端位姿控制不需要 SDK
            'skip_end_effector': True,# 手臂末端位姿控制不需要末端执行器
            'skip_camera': True,# 手臂末端位姿控制不需要相机
            'skip_state_manager': True,# 手臂末端位姿控制不需要状态管理器
            'skip_force_publishers': True,# 手臂末端位姿控制不需要力控发布器
        })
        result = cls.hardware.initialize()

        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")

        # 环境检测: 检查必要的 ROS 服务是否可用
        from apps.test_kuavo_5w_adapter._scaffold import check_services_available
        ok, missing = check_services_available([
            '/mobile_manipulator_reset_torso',
            '/mobile_manipulator_mpc_control',
            '/wheel_arm_change_arm_ctrl_mode',
        ])
        if not ok:
            raise unittest.SkipTest(f"ROS 服务不可用: {missing}，请启动控制器进程")

        print("✅ 硬件初始化成功")

        # === 前置设置（与源脚本 cmd_arm_ee_world_test.py 一致）===
        # 1. 躯干复位
        reset_result = cls.hardware.reset_torso_to_initial()
        if reset_result.success:
            print(f"  ✓ 躯干已复位: {reset_result.message}")
            time.sleep(2.0)

        # 2. 手臂控制模式: 重置(1) → 外部控制(2)
        cls.hardware.set_arm_control_mode(1)
        time.sleep(1.0)
        cls.hardware.set_arm_control_mode(2)

        # 3. 设置末端跟踪焦点（源脚本默认 focus_ee=True）
        cls.hardware.set_focus_ee(focus_ee=True)

        # 注意: 源脚本不设置 MPC 模式，控制器内部自动管理

        print("\n💡 提示: 手臂控制需要完整的初始化流程（手臂控制模式 + 焦点）")
    
    @classmethod
    def tearDownClass(cls):
        """测试类清理：关闭硬件连接"""
        print("\n" + "="*70)
        print("测试套件清理: 关闭硬件连接")
        print("="*70)
        
        if hasattr(cls, 'hardware'):
            # === 后置复位（与源脚本 teardown 一致）===
            # 1. 手臂复位到初始位置
            cls.hardware.set_arm_control_mode(1)
            time.sleep(2.0)
            # 2. 躯干复位
            cls.hardware.reset_torso_to_initial()
            time.sleep(2.0)

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
    
    def test_01_arms_spread(self):
        """测试1: 左右展开（初始姿态，pitch=0°）"""
        print("  测试目标: 双臂向两侧展开")
        print("  左手位姿: x=0.1, y=0.4, z=0.7, pitch=0°")
        print("  右手位姿: x=0.1, y=-0.4, z=0.7, pitch=0°")
        
        # 双臂位姿（注意：Pose6D 参数顺序为 x, y, z, yaw, pitch, roll）
        left_pose = Pose6D(x=0.1, y=0.4, z=0.7, yaw=0.0, pitch=0.0, roll=0.0)
        right_pose = Pose6D(x=0.1, y=-0.4, z=0.7, yaw=0.0, pitch=0.0, roll=0.0)
        
        result = self.hardware.send_both_ee_poses(left_pose, right_pose, frame=FrameType.WORLD)
        
        # 断言
        self.assertTrue(result.success, f"发送双臂位姿失败: {result.message}")
        print(f"  ✅ 双臂成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 左右展开完成")
    
    def test_02_arms_forward_swing(self):
        """测试2: 前摆臂（x=0.3, pitch=-90°）"""
        print("  测试目标: 双臂向前摆动")
        print("  左手位姿: x=0.3, y=0.4, z=0.7, pitch=-90°")
        print("  右手位姿: x=0.3, y=-0.4, z=0.7, pitch=-90°")
        
        left_pose = Pose6D(x=0.3, y=0.4, z=0.7, yaw=0.0, pitch=-1.5708, roll=0.0)
        right_pose = Pose6D(x=0.3, y=-0.4, z=0.7, yaw=0.0, pitch=-1.5708, roll=0.0)
        
        result = self.hardware.send_both_ee_poses(left_pose, right_pose, frame=FrameType.WORLD)
        self.assertTrue(result.success, f"发送双臂位姿失败: {result.message}")
        print(f"  ✅ 双臂成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 前摆臂完成")
    
    def test_03_arms_forward_close(self):
        """测试3: 前摆臂收拢（x=0.3, y=±0.2）"""
        print("  测试目标: 双臂向前收拢")
        print("  左手位姿: x=0.3, y=0.2, z=0.7, pitch=-90°")
        print("  右手位姿: x=0.3, y=-0.2, z=0.7, pitch=-90°")
        
        left_pose = Pose6D(x=0.3, y=0.2, z=0.7, yaw=0.0, pitch=-1.5708, roll=0.0)
        right_pose = Pose6D(x=0.3, y=-0.2, z=0.7, yaw=0.0, pitch=-1.5708, roll=0.0)
        
        result = self.hardware.send_both_ee_poses(left_pose, right_pose, frame=FrameType.WORLD)
        self.assertTrue(result.success, f"发送双臂位姿失败: {result.message}")
        print(f"  ✅ 双臂成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 前摆臂收拢完成")
    
    def test_04_arms_extend(self):
        """测试4: 前伸（x=0.5）"""
        print("  测试目标: 双臂向前伸展")
        print("  左手位姿: x=0.5, y=0.2, z=0.7, pitch=-90°")
        print("  右手位姿: x=0.5, y=-0.2, z=0.7, pitch=-90°")
        
        left_pose = Pose6D(x=0.5, y=0.2, z=0.7, yaw=0.0, pitch=-1.5708, roll=0.0)
        right_pose = Pose6D(x=0.5, y=-0.2, z=0.7, yaw=0.0, pitch=-1.5708, roll=0.0)
        
        result = self.hardware.send_both_ee_poses(left_pose, right_pose, frame=FrameType.WORLD)
        self.assertTrue(result.success, f"发送双臂位姿失败: {result.message}")
        print(f"  ✅ 双臂成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 前伸完成")
    
    def test_05_arms_lift(self):
        """测试5: 抬高（z=0.85）"""
        print("  测试目标: 双臂抬高")
        print("  左手位姿: x=0.5, y=0.2, z=0.85, pitch=-90°")
        print("  右手位姿: x=0.5, y=-0.2, z=0.85, pitch=-90°")
        
        left_pose = Pose6D(x=0.5, y=0.2, z=0.85, yaw=0.0, pitch=-1.5708, roll=0.0)
        right_pose = Pose6D(x=0.5, y=-0.2, z=0.85, yaw=0.0, pitch=-1.5708, roll=0.0)
        
        result = self.hardware.send_both_ee_poses(left_pose, right_pose, frame=FrameType.WORLD)
        self.assertTrue(result.success, f"发送双臂位姿失败: {result.message}")
        print(f"  ✅ 双臂成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 抬高完成")
    
    def test_06_arms_max_extend(self):
        """测试6: 极限前伸（x=1.2）"""
        print("  测试目标: 双臂极限前伸")
        print("  左手位姿: x=1.2, y=0.2, z=0.85, pitch=-90°")
        print("  右手位姿: x=1.2, y=-0.2, z=0.85, pitch=-90°")
        
        left_pose = Pose6D(x=1.2, y=0.2, z=0.85, yaw=0.0, pitch=-1.5708, roll=0.0)
        right_pose = Pose6D(x=1.2, y=-0.2, z=0.85, yaw=0.0, pitch=-1.5708, roll=0.0)
        
        result = self.hardware.send_both_ee_poses(left_pose, right_pose, frame=FrameType.WORLD)
        self.assertTrue(result.success, f"发送双臂位姿失败: {result.message}")
        print(f"  ✅ 双臂成功: {result.message}")
        
        time.sleep(4.0)  # 极限位置需要更多时间
        print("  ✅ 极限前伸完成")
    
    def test_07_arms_return_safe(self):
        """测试7: 回到安全位置（x=0.5）"""
        print("  测试目标: 从极限位置回到安全位置")
        print("  左手位姿: x=0.5, y=0.2, z=0.85, pitch=-90°")
        print("  右手位姿: x=0.5, y=-0.2, z=0.85, pitch=-90°")
        
        left_pose = Pose6D(x=0.5, y=0.2, z=0.85, yaw=0.0, pitch=-1.5708, roll=0.0)
        right_pose = Pose6D(x=0.5, y=-0.2, z=0.85, yaw=0.0, pitch=-1.5708, roll=0.0)
        
        result = self.hardware.send_both_ee_poses(left_pose, right_pose, frame=FrameType.WORLD)
        self.assertTrue(result.success, f"发送双臂位姿失败: {result.message}")
        print(f"  ✅ 双臂成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 回到安全位置完成")
    
    def test_08_arms_return_initial(self):
        """测试8: 回到初始位置（pitch=0°）"""
        print("  测试目标: 回到初始姿态")
        print("  左手位姿: x=0.2, y=0.3, z=0.8, pitch=0°")
        print("  右手位姿: x=0.2, y=-0.3, z=0.8, pitch=0°")
        
        left_pose = Pose6D(x=0.2, y=0.3, z=0.8, yaw=0.0, pitch=0.0, roll=0.0)
        right_pose = Pose6D(x=0.2, y=-0.3, z=0.8, yaw=0.0, pitch=0.0, roll=0.0)
        
        result = self.hardware.send_both_ee_poses(left_pose, right_pose, frame=FrameType.WORLD)
        self.assertTrue(result.success, f"发送双臂位姿失败: {result.message}")
        print(f"  ✅ 双臂成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 回到初始位置完成")


def run_tests():
    """运行测试套件"""
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('test_arm_ee_world', anonymous=True)
    
    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestArmEEWorld)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - 手臂末端位姿控制（世界坐标系）")
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
