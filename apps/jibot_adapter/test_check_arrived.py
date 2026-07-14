#!/usr/bin/env python3
"""
测试 JiBot 底盘任务到达检查（适配器层）

对应 T1 脚本: jibot_upper_machine_python_tests/check_arrived.py
适配器接口: LejuWheeledArmHardware.check_arrived_jibot()
ROS 服务:  /move_base/check_arrived (leju_mobile_base_msgs/CheckArrived)

功能说明:
- 测试查询指定底盘任务的到达状态
- 在 setUpClass 中先调用 base_move_relative_jibot(x=0.2) 获取真实 task_id
- 验证 check_arrived 返回的 arrived/status/message 字段

注意事项:
1. task_id 必须使用 base_move 或 move_to_target 返回的真实值
2. blocking=True 时调用会阻塞到任务完成或超时
3. status=2 表示到达，status=0 message="timeout" 表示未到达
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


class TestCheckArrived(unittest.TestCase):
    """JiBot 底盘任务到达检查测试"""

    @classmethod
    def setUpClass(cls):
        print(f"\n{'='*70}")
        print("JiBot T2 测试套件: 任务到达检查 (check_arrived_jibot)")
        print(f"{'='*70}")

        cls.hardware = LejuWheeledArmHardware(config={'skip_sdk_managers': True})
        result = cls.hardware.initialize()
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")
        print("✅ 硬件初始化成功")

        jibot_setup()

        # 预先发送一个小的 base_move 获取真实 task_id
        print("\n📌 前置: 发送 base_move(x=0.2) 获取 task_id ...")
        result = cls.hardware.base_move_relative_jibot(x=0.2, y=0.0, theta=0.0)
        if not result.success:
            raise RuntimeError(f"获取 task_id 失败: {result.message}")

        cls.test_task_id = result.data["task_id"]
        print(f"  ✓ 获得 task_id: {cls.test_task_id}")

        # 等待一小段时间让底盘开始执行，避免首测时任务尚未启动
        time.sleep(0.5)

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
        time.sleep(0.5)

    def test_01_nonblocking_check(self):
        """非阻塞检查任务状态"""
        print(f"  测试目标: 非阻塞查询 task_id={self.test_task_id}")

        result = self.hardware.check_arrived_jibot(
            task_id=self.test_task_id, blocking=False,
        )

        self.assertTrue(result.success, f"check_arrived 调用失败: {result.message}")
        self.assertIn("arrived", result.data)
        self.assertIn("status", result.data)
        self.assertIn("message", result.data)

        arrived = result.data["arrived"]
        status_msg = result.data["message"]
        print(f"  ✅ check_arrived 成功: arrived={arrived}, status={status_msg}")

    def test_02_blocking_check(self):
        """阻塞等待任务到达"""
        print(f"  测试目标: 阻塞等待 task_id={self.test_task_id}")

        result = self.hardware.check_arrived_jibot(
            task_id=self.test_task_id, blocking=True, timeout=30.0,
        )

        self.assertTrue(result.success, f"check_arrived 调用失败: {result.message}")
        arrived = result.data["arrived"]
        status_msg = result.data["message"]
        status_code = result.data["status"]

        print(f"  check_arrived 结果: arrived={arrived}, status={status_code}, "
              f"message={status_msg}")

        # 常见响应说明
        if arrived and status_code == 2:
            print("  ✅ 机器人已到达目标点")
        elif status_msg == "timeout":
            print("  ⚠️  超时未到达（可能移动较慢或目标较远）")
        elif status_msg == "interrupted":
            print("  ⚠️  任务被中断（检查目标点/定位/costmap）")


def run_tests():
    if not rospy.core.is_initialized():
        rospy.init_node('jibot_test_check_arrived', anonymous=True)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestCheckArrived)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == '__main__':
    print(f"\n{'='*70}")
    print("JiBot T2 测试 - 任务到达检查 (check_arrived_jibot)")
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
