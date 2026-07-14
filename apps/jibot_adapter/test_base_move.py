#!/usr/bin/env python3
"""
测试 JiBot 底盘相对移动（适配器层）

对应 T1 脚本: jibot_upper_machine_python_tests/base_move.py
适配器接口: LejuWheeledArmHardware.base_move_relative_jibot()
ROS 服务:  /move_base/base_move (leju_mobile_base_msgs/BaseMove)

功能说明:
- 测试底盘从当前位置做相对移动
- 验证返回的 task_id 格式 (move_task_xxx)
- 默认使用小位移 (0.2m 前进)

注意事项:
1. 这是相对移动，不是地图绝对目标点
2. 首次实机测试建议使用小位移
3. 调用成功返回 task_id，后续用 check_arrived 检查到达
"""

import sys
import os
import time
import unittest

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, project_root)

import rospy
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from apps.jibot._scaffold import jibot_setup
from core.domain.chassis_options import MoveToTargetOptions


class TestBaseMove(unittest.TestCase):
    """JiBot 底盘相对移动测试"""

    @classmethod
    def setUpClass(cls):
        print(f"\n{'='*70}")
        print("JiBot T2 测试套件: 底盘相对移动 (base_move_relative_jibot)")
        print(f"{'='*70}")

        cls.hardware = LejuWheeledArmHardware(config={'skip_sdk_managers': True})
        result = cls.hardware.initialize()
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")
        print("✅ 硬件初始化成功")

        jibot_setup()

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

    def test_01_forward_small(self):
        """小位移前进 0.2m"""
        print("  测试目标: 前进 0.2m")

        result = self.hardware.base_move_relative_jibot(x=0.2, y=0.0, theta=0.0)

        self.assertTrue(result.success, f"服务调用失败: {result.message}")
        self.assertIn("task_id", result.data, "返回数据中缺少 task_id")
        task_id = result.data["task_id"]
        self.assertTrue(
            task_id.startswith("move_task_"),
            f"task_id 格式异常，期望 move_task_xxx: {task_id}",
        )
        print(f"  ✅ base_move 成功: task_id={task_id}")

    def test_02_forward_with_options(self):
        """前进 0.1m，自定义导航参数"""
        print("  测试目标: 前进 0.1m，低速度高精度")

        options = MoveToTargetOptions(
            linear_velocity=0.1,
            angular_velocity=0.2,
            position_threshold=0.05,
            angle_threshold=0.05,
        )

        result = self.hardware.base_move_relative_jibot(
            x=0.1, y=0.0, theta=0.0, options=options,
        )

        self.assertTrue(result.success, f"服务调用失败: {result.message}")
        self.assertIn("task_id", result.data)
        print(f"  ✅ base_move 成功: task_id={result.data['task_id']}")

    def test_03_rotate_in_place(self):
        """原地旋转 0.3 rad"""
        print("  测试目标: 原地旋转 0.3 rad")

        result = self.hardware.base_move_relative_jibot(x=0.0, y=0.0, theta=0.3)

        self.assertTrue(result.success, f"服务调用失败: {result.message}")
        self.assertIn("task_id", result.data)
        print(f"  ✅ base_move 成功: task_id={result.data['task_id']}")

    def test_04_backward(self):
        """后退 0.1m"""
        print("  测试目标: 后退 0.1m")

        result = self.hardware.base_move_relative_jibot(x=-0.1, y=0.0, theta=0.0)

        self.assertTrue(result.success, f"服务调用失败: {result.message}")
        self.assertIn("task_id", result.data)
        print(f"  ✅ base_move 成功: task_id={result.data['task_id']}")


def run_tests():
    if not rospy.core.is_initialized():
        rospy.init_node('jibot_test_base_move', anonymous=True)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestBaseMove)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == '__main__':
    print(f"\n{'='*70}")
    print("JiBot T2 测试 - 底盘相对移动 (base_move_relative_jibot)")
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
