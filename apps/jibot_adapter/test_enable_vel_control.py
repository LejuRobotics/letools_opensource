#!/usr/bin/env python3
"""
测试 JiBot 底盘速度控制权限切换（适配器层）

对应 T1 脚本: jibot_upper_machine_python_tests/enable_vel_control.py
适配器接口: LejuWheeledArmHardware.enable_vel_control_jibot()
            LejuWheeledArmHardware.get_vel_control_state_jibot()
ROS 服务:  /enable_vel_control (std_srvs/SetBool)
ROS 话题:  /enable_vel_control_state (std_msgs/Bool)

功能说明:
- 测试底盘速度控制权限切换
- enable=False：导航模块接管底盘
- enable=True：手动/外部速度控制

注意事项:
1. 该接口来自 AAEON 下位机 kuavo-ros-control，非 Jarvis 底盘服务
2. 可能并非所有环境都有，测试前用 rosservice list | grep enable_vel_control 检查
3. 测试会切换状态，建议测试后恢复到原始值
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


def is_vel_control_available():
    """检查 /enable_vel_control 服务是否存在。"""
    try:
        rospy.wait_for_service("/enable_vel_control", timeout=2.0)
        return True
    except (rospy.ROSInterruptException, rospy.ROSException):
        return False


class TestEnableVelControl(unittest.TestCase):
    """JiBot 底盘速度控制权限切换测试"""

    @classmethod
    def setUpClass(cls):
        print(f"\n{'='*70}")
        print("JiBot T2 测试套件: 速度控制权限切换 (enable_vel_control_jibot)")
        print(f"{'='*70}")

        cls.hardware = LejuWheeledArmHardware(config={'skip_sdk_managers': True})
        result = cls.hardware.initialize()
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")
        print("✅ 硬件初始化成功")

        jibot_setup()

        # enable_vel_control 是可选的，来自 AAEON 下位机
        cls.service_available = is_vel_control_available()
        if cls.service_available:
            # 记录初始状态，测试后恢复
            state_result = cls.hardware.get_vel_control_state_jibot(timeout=3.0)
            cls.initial_state = state_result.data["state"] if state_result.success else None
            print(f"  ✓ /enable_vel_control 可用，初始状态: {cls.initial_state}")
        else:
            cls.initial_state = None
            print("  ⚠ /enable_vel_control 不可用（AAEON 下位机未启动），将跳过测试")

    @classmethod
    def tearDownClass(cls):
        print(f"\n{'='*70}")
        print("测试套件清理")
        print(f"{'='*70}")

        # 恢复到初始状态
        if cls.service_available and cls.initial_state is not None:
            state_result = cls.hardware.get_vel_control_state_jibot(timeout=3.0)
            current = state_result.data["state"] if state_result.success else None
            if current != cls.initial_state:
                print(f"  恢复速度控制权限: {cls.initial_state}")
                cls.hardware.enable_vel_control_jibot(cls.initial_state)

        if hasattr(cls, 'hardware'):
            cls.hardware.shutdown()
            print("✅ 硬件已关闭")

    def setUp(self):
        if not self.service_available:
            self.skipTest("/enable_vel_control 服务不可用")
        print(f"\n--- 开始测试: {self._testMethodName} ---")

    def tearDown(self):
        print(f"--- 结束测试: {self._testMethodName} ---")
        time.sleep(0.5)

    def test_01_get_current_state(self):
        """获取当前速度控制权限状态"""
        print("  测试目标: 读取 /enable_vel_control_state")

        result = self.hardware.get_vel_control_state_jibot(timeout=3.0)

        self.assertTrue(result.success, f"读取状态失败: {result.message}")
        self.assertIn("state", result.data, "返回数据中缺少 state")
        print(f"  ✅ 当前状态: state={result.data['state']}")

    def test_02_switch_to_false(self):
        """切换为 False（导航接管）"""
        print("  测试目标: enable_vel_control(False)")

        result = self.hardware.enable_vel_control_jibot(enable=False)

        self.assertTrue(result.success, f"切换失败: {result.message}")
        self.assertIn("state_after", result.data)
        print(f"  ✅ enable_vel_control(False): state_after={result.data['state_after']}")

    def test_03_switch_to_true(self):
        """切换为 True（速度控制开启）"""
        print("  测试目标: enable_vel_control(True)")

        result = self.hardware.enable_vel_control_jibot(enable=True)

        self.assertTrue(result.success, f"切换失败: {result.message}")
        self.assertIn("state_after", result.data)
        print(f"  ✅ enable_vel_control(True): state_after={result.data['state_after']}")

    def test_04_roundtrip(self):
        """往返切换：False → True，验证状态跟随"""
        print("  测试目标: False → True 往返切换")

        # 先设 False
        r1 = self.hardware.enable_vel_control_jibot(enable=False)
        self.assertTrue(r1.success, f"False 切换失败: {r1.message}")
        print(f"    False → state={r1.data['state_after']}")

        # 再设 True
        r2 = self.hardware.enable_vel_control_jibot(enable=True)
        self.assertTrue(r2.success, f"True 切换失败: {r2.message}")
        print(f"    True  → state={r2.data['state_after']}")

        # 验证 True 时状态确实为 True
        self.assertTrue(
            r2.data.get("state_after") is True,
            f"True 切换后状态应为 True: {r2.data}"
        )
        print("  ✅ 往返切换成功，状态正确跟随")


def run_tests():
    if not rospy.core.is_initialized():
        rospy.init_node('jibot_test_enable_vel_control', anonymous=True)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestEnableVelControl)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == '__main__':
    print(f"\n{'='*70}")
    print("JiBot T2 测试 - 速度控制权限切换 (enable_vel_control_jibot)")
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
