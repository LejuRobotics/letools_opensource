#!/usr/bin/env python3
"""
测试底盘位置控制（本体坐标系）

对应底层测试: test_kuavo_5w/01_base_control/test_cmd_pose_base.py
适配器接口: LejuWheeledArmHardware.send_base_pose(frame=FrameType.LOCAL)
ROS 话题: /cmd_pose (geometry_msgs/Twist)
反馈话题: /lb_cmd_pose_reach_time (std_msgs/Float32)

功能说明:
- 测试本体坐标系下的底盘相对位置控制
- 验证前进、后退、旋转等相对移动
- 使用适配器层的 send_base_pose() 方法
- 自动等待到达时间反馈

注意事项:
1. 位置控制是相对移动（基于当前位置）
2. yaw可以发送多圈（如360表示旋转一圈）
3. 适配器会自动订阅 /lb_cmd_pose_reach_time 并等待
4. 无需设置 MPC 模式
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


class TestBasePoseLocal(unittest.TestCase):
    """底盘位置控制测试类（本体坐标系）"""
    
    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接"""
        print("\n" + "="*70)
        print("测试套件初始化: 底盘位置控制（本体坐标系）")
        print("="*70)
        
        cls.hardware = LejuWheeledArmHardware(config={
            'robot_type': 'leju_wheeled',
            'skip_sdk_managers': True,  # 底盘控制不需要 SDK 管理器，跳过 KuavoSDK.Init(~7s)
            'skip_end_effector': True,
            'skip_camera': True,
            'skip_state_manager': True,
            'skip_force_publishers': True,
        })
        result = cls.hardware.initialize()
        
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")
        
        print("✅ 硬件初始化成功")
        print("\n💡 提示: /cmd_pose 话题优先级高，无需设置 MPC 模式")
    
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
    
    def test_01_forward_relative(self):
        """测试1: 本体坐标系相对前进"""
        print("  测试目标: 相对当前位置向前移动 0.5 米")
        
        result = self.hardware.send_base_pose(
            x=0.5, y=0.0, yaw=0.0, frame=FrameType.LOCAL
        )
        
        # 断言：检查返回结果
        self.assertTrue(result.success, f"发送位置指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        print("  ✅ 相对前进完成（适配器已自动等待到达时间）")
    
    def test_02_backward_relative(self):
        """测试2: 本体坐标系相对后退"""
        print("  测试目标: 相对当前位置向后移动 0.3 米")
        
        result = self.hardware.send_base_pose(
            x=-0.3, y=0.0, yaw=0.0, frame=FrameType.LOCAL
        )
        
        self.assertTrue(result.success, f"发送位置指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        print("  ✅ 相对后退完成")
    
    def test_03_lateral_left_relative(self):
        """测试3: 本体坐标系相对向左平移"""
        print("  测试目标: 相对当前位置向左移动 0.3 米")
        
        result = self.hardware.send_base_pose(
            x=0.0, y=0.3, yaw=0.0, frame=FrameType.LOCAL
        )
        
        self.assertTrue(result.success, f"发送位置指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        print("  ✅ 相对左平移完成")
    
    def test_04_lateral_right_relative(self):
        """测试4: 本体坐标系相对向右平移"""
        print("  测试目标: 相对当前位置向右移动 0.3 米")
        
        result = self.hardware.send_base_pose(
            x=0.0, y=-0.3, yaw=0.0, frame=FrameType.LOCAL
        )
        
        self.assertTrue(result.success, f"发送位置指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        print("  ✅ 相对右平移完成")
    
    def test_05_rotation_ccw_relative(self):
        """测试5: 本体坐标系相对逆时针旋转"""
        print("  测试目标: 相对当前位置逆时针旋转 90 度")
        
        result = self.hardware.send_base_pose(
            x=0.0, y=0.0, yaw=90, frame=FrameType.LOCAL
        )
        
        self.assertTrue(result.success, f"发送位置指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        print("  ✅ 相对逆时针旋转完成")
    
    def test_06_rotation_cw_relative(self):
        """测试6: 本体坐标系相对顺时针旋转"""
        print("  测试目标: 相对当前位置顺时针旋转 90 度")
        
        result = self.hardware.send_base_pose(
            x=0.0, y=0.0, yaw=-90, frame=FrameType.LOCAL
        )
        
        self.assertTrue(result.success, f"发送位置指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        print("  ✅ 相对顺时针旋转完成")
    
    def test_07_full_rotation(self):
        """测试7: 本体坐标系完整旋转一圈"""
        print("  测试目标: 相对当前位置旋转一圈 (360度)")
        
        result = self.hardware.send_base_pose(
            x=0.0, y=0.0, yaw=360, frame=FrameType.LOCAL
        )
        
        self.assertTrue(result.success, f"发送位置指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        print("  ✅ 完整旋转一圈完成")
    
    def test_08_combined_motion(self):
        """测试8: 本体坐标系复合运动（前进+旋转）"""
        print("  测试目标: 相对当前位置同时前进和旋转")
        
        result = self.hardware.send_base_pose(
            x=0.3, y=0.0, yaw=45, frame=FrameType.LOCAL  # 45度
        )
        
        self.assertTrue(result.success, f"发送位置指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        print("  ✅ 复合运动完成")


def run_tests():
    """运行测试套件"""
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('test_base_pose_local', anonymous=True)
    
    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestBasePoseLocal)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - 底盘位置控制（本体坐标系）")
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
