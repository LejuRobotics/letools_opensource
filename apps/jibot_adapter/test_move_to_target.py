#!/usr/bin/env python3
"""
测试 JiBot 底盘 map 绝对目标点移动（适配器层）

对应 T1 脚本: jibot_upper_machine_python_tests/move_to_target.py
适配器接口: LejuWheeledArmHardware.base_move_to_target_jibot()
ROS 服务:  /move_base/move_to_target (leju_mobile_base_msgs/MoveToTarget)

功能说明:
- 测试底盘移动到 map 坐标系下的绝对目标点
- 自动读取 /jarvis/pose 获取当前 map 坐标并计算附近目标
- 验证返回的 task_id 格式 (goto_task_xxx)

注意事项:
1. 这是 map 坐标系绝对目标点，不是相对位移
2. 目标点需使用 /jarvis/pose 或 /move_base/amcl_pose 的 map 坐标
3. 不要使用 /odometry/filtered 的坐标
"""

import sys
import os
import math
import time
import unittest

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, project_root)

import rospy
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from apps.jibot._scaffold import jibot_setup
from core.domain.chassis_options import MoveToTargetOptions


def read_current_map_pose(timeout=5.0):
    """从 /jarvis/pose 读取当前 map 坐标系位姿。

    Returns:
        (x, y, theta): 当前位置和偏航角，失败则返回 None
    """
    try:
        from nav_msgs.msg import Odometry
        msg = rospy.wait_for_message("/jarvis/pose", Odometry, timeout=timeout)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(z, w)
        return x, y, theta
    except Exception as e:
        print(f"  ⚠️  无法读取 /jarvis/pose: {e}")
        return None


class TestMoveToTarget(unittest.TestCase):
    """JiBot 底盘绝对目标点移动测试"""

    @classmethod
    def setUpClass(cls):
        print(f"\n{'='*70}")
        print("JiBot T2 测试套件: 底盘绝对目标点移动 (base_move_to_target_jibot)")
        print(f"{'='*70}")

        cls.hardware = LejuWheeledArmHardware(config={'skip_sdk_managers': True})
        result = cls.hardware.initialize()
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")
        print("✅ 硬件初始化成功")

        jibot_setup()

        # 读取当前 map 坐标
        cls.current_pose = read_current_map_pose()
        if cls.current_pose:
            x, y, theta = cls.current_pose
            print(f"  ✓ 当前 map 坐标: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")

    @classmethod
    def tearDownClass(cls):
        print(f"\n{'='*70}")
        print("测试套件清理")
        print(f"{'='*70}")

        if hasattr(cls, 'hardware'):
            cls.hardware.shutdown()
            print("✅ 硬件已关闭")

    def setUp(self):
        print(f"\n--- 开始测试: {self._testMethodName} ---")

    def tearDown(self):
        print(f"--- 结束测试: {self._testMethodName} ---")
        time.sleep(1.0)

    def test_01_nearby_target_from_jarvis_pose(self):
        """从 /jarvis/pose 读取位置，向 x+0.2m 移动"""
        if self.current_pose is None:
            self.skipTest("/jarvis/pose 不可用，跳过绝对目标点测试")

        cur_x, cur_y, cur_theta = self.current_pose
        target_x = cur_x + 0.2
        target_y = cur_y
        target_theta = cur_theta

        print(f"  当前位置: x={cur_x:.3f}, y={cur_y:.3f}, theta={cur_theta:.3f}")
        print(f"  目标位置: x={target_x:.3f}, y={target_y:.3f}, theta={target_theta:.3f}")

        result = self.hardware.base_move_to_target_jibot(
            x=target_x, y=target_y, theta=target_theta,
        )

        self.assertTrue(result.success, f"服务调用失败: {result.message}")
        self.assertIn("task_id", result.data, "返回数据中缺少 task_id")
        task_id = result.data["task_id"]
        self.assertTrue(
            task_id.startswith("goto_task_"),
            f"task_id 格式异常，期望 goto_task_xxx: {task_id}",
        )
        print(f"  ✅ move_to_target 成功: task_id={task_id}")

    def test_02_with_custom_options(self):
        """使用自定义导航参数移动"""
        if self.current_pose is None:
            self.skipTest("/jarvis/pose 不可用，跳过绝对目标点测试")

        cur_x, cur_y, cur_theta = self.current_pose
        target_x = cur_x + 0.15
        target_y = cur_y

        options = MoveToTargetOptions(
            linear_velocity=0.2,
            angular_velocity=0.3,
            position_threshold=0.05,
            angle_threshold=0.1,
            avoid_enabled=False,
        )

        print(f"  目标位置: x={target_x:.3f}, y={target_y:.3f}, theta={cur_theta:.3f}")
        print(f"  自定义参数: linear_v={options.linear_velocity}, pos_th={options.position_threshold}")

        result = self.hardware.base_move_to_target_jibot(
            x=target_x, y=target_y, theta=cur_theta, options=options,
        )

        self.assertTrue(result.success, f"服务调用失败: {result.message}")
        self.assertIn("task_id", result.data)
        print(f"  ✅ move_to_target 成功: task_id={result.data['task_id']}")


def run_tests():
    if not rospy.core.is_initialized():
        rospy.init_node('jibot_test_move_to_target', anonymous=True)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestMoveToTarget)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == '__main__':
    print(f"\n{'='*70}")
    print("JiBot T2 测试 - 底盘绝对目标点移动 (base_move_to_target_jibot)")
    print(f"{'='*70}")

    try:
        success = run_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⚠️  测试被用户中断")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ 测试执行出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
