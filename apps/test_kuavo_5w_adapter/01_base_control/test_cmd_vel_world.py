#!/usr/bin/env python3
"""
测试底盘速度控制（世界坐标系）

对应底层测试: test_kuavo_5w/01_base_control/test_cmd_vel_world.py
适配器接口: LejuWheeledArmHardware.send_base_velocity(frame=FrameType.WORLD)
ROS 话题: /cmd_vel_world (geometry_msgs/Twist)

功能说明:
- 测试世界坐标系下的底盘速度控制
- 验证前进、后退、旋转等基本运动
- 使用适配器层的 send_base_velocity() 方法

注意事项:
1. 速度命令需要持续发布，否则1秒后会自动停止
2. 适配器内部默认持续发布5秒
3. 世界坐标系：x方向指向机器人启动时的正前方
4. 无需设置 MPC 模式（/cmd_vel_world 优先级最高）
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


class TestBaseVelocityWorld(unittest.TestCase):
    """底盘速度控制测试类（世界坐标系）"""
    
    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接"""
        print("\n" + "="*70)
        print("测试套件初始化: 底盘速度控制（世界坐标系）")
        print("="*70)
        
        cls.hardware = LejuWheeledArmHardware(config={
            'robot_type': 'leju_wheeled',
            'skip_sdk_managers': True,  # 底盘速度控制不需要 SDK 管理器
            'skip_end_effector': True, # 底盘速度控制不需要末端执行器
            'skip_camera': True,# 底盘速度控制不需要相机
            'skip_state_manager': True,# 底盘速度控制不需要状态管理器
            'skip_force_publishers': True,# 底盘速度控制不需要力控发布器
        })
        result = cls.hardware.initialize()
        
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")
        
        print("✅ 硬件初始化成功")
        print("\n💡 提示: /cmd_vel_world 话题优先级最高，无需设置 MPC 模式")
    
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
        # 每次测试后停止机器人
        try:
            self.hardware.send_base_velocity(vx=0.0, vy=0.0, vyaw=0.0, frame=FrameType.WORLD)
            time.sleep(0.5)
        except Exception as e:
            print(f"⚠️  停止机器人时出错: {e}")
    
    def test_01_forward_motion(self):
        """测试1: 世界坐标系前进"""
        print("  测试目标: 以 0.3 m/s 的速度沿世界系x轴前进")
        
        result = self.hardware.send_base_velocity(
            vx=0.3, vy=0.0, vyaw=0.0, frame=FrameType.WORLD
        )
        
        # 断言：检查返回结果
        self.assertTrue(result.success, f"发送速度指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        # 等待运动完成
        time.sleep(3.0)
        print("  ✅ 前进运动完成")
    
    def test_02_backward_motion(self):
        """测试2: 世界坐标系后退"""
        print("  测试目标: 以 0.2 m/s 的速度沿世界系x轴后退")
        
        result = self.hardware.send_base_velocity(
            vx=-0.2, vy=0.0, vyaw=0.0, frame=FrameType.WORLD
        )
        
        self.assertTrue(result.success, f"发送速度指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 后退运动完成")
    
    def test_03_lateral_motion_left(self):
        """测试3: 世界坐标系向左平移"""
        print("  测试目标: 以 0.1 m/s 的速度沿世界系y轴向左平移")
        
        result = self.hardware.send_base_velocity(
            vx=0.0, vy=0.1, vyaw=0.0, frame=FrameType.WORLD
        )
        
        self.assertTrue(result.success, f"发送速度指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 左平移运动完成")
    
    def test_04_lateral_motion_right(self):
        """测试4: 世界坐标系向右平移"""
        print("  测试目标: 以 0.1 m/s 的速度沿世界系y轴向右平移")
        
        result = self.hardware.send_base_velocity(
            vx=0.0, vy=-0.1, vyaw=0.0, frame=FrameType.WORLD
        )
        
        self.assertTrue(result.success, f"发送速度指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 右平移运动完成")
    
    def test_05_rotation_ccw(self):
        """测试5: 世界坐标系逆时针旋转"""
        print("  测试目标: 以 28.6 deg/s 的速度逆时针旋转")

        result = self.hardware.send_base_velocity(
            vx=0.0, vy=0.0, vyaw=28.6, frame=FrameType.WORLD
        )
        
        self.assertTrue(result.success, f"发送速度指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 逆时针旋转完成")
    
    def test_06_rotation_cw(self):
        """测试6: 世界坐标系顺时针旋转"""
        print("  测试目标: 以 28.6 deg/s 的速度顺时针旋转")

        result = self.hardware.send_base_velocity(
            vx=0.0, vy=0.0, vyaw=-28.6, frame=FrameType.WORLD
        )
        
        self.assertTrue(result.success, f"发送速度指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 顺时针旋转完成")
    
    def test_07_combined_motion(self):
        """测试7: 世界坐标系复合运动（前进+旋转）"""
        print("  测试目标: 同时沿世界系x轴前进和旋转")
        
        result = self.hardware.send_base_velocity(
            vx=0.2, vy=0.0, vyaw=17.2, frame=FrameType.WORLD
        )
        
        self.assertTrue(result.success, f"发送速度指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        time.sleep(3.0)
        print("  ✅ 复合运动完成")
    
    def test_08_stop_command(self):
        """测试8: 停止命令"""
        print("  测试目标: 发送停止命令")
        
        result = self.hardware.send_base_velocity(
            vx=0.0, vy=0.0, vyaw=0.0, frame=FrameType.WORLD
        )
        
        self.assertTrue(result.success, f"发送停止指令失败: {result.message}")
        print(f"  ✅ 成功: {result.message}")
        
        time.sleep(1.0)
        print("  ✅ 机器人已停止")


def run_tests():
    """运行测试套件"""
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('test_base_velocity_world', anonymous=True)
    
    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestBaseVelocityWorld)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("Kuavo 5-W 应用层测试 - 底盘速度控制（世界坐标系）")
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
