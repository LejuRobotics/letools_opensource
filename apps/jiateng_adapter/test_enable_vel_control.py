#!/usr/bin/env python3
"""
测试嘉腾底盘外部速度通道开关。

适配器接口:
- LejuWheeledArmHardware.enable_vel_control_jiateng()
- LejuWheeledArmHardware.get_vel_control_state_jiateng()

ROS 服务: /enable_vel_control
ROS 话题: /enable_vel_control_state

注意事项:
1. enable=False：关闭外部 /cmd_vel 转发
2. enable=True：开启外部 /cmd_vel 转发
3. 该开关不会禁用嘉腾 /move_base 导航服务
4. 若成功读取初始状态，切换测试结束后会自动恢复
5. 只在 main 中保留一个 suite.addTest
"""

import os
import sys
import unittest

PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)
sys.path.insert(0, PROJECT_ROOT)

from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from apps.jiateng_adapter._scaffold import (
    assert_no_active_navigation_task,
    jiateng_setup,
)


class TestEnableVelControl(unittest.TestCase):
    """嘉腾底盘外部速度通道测试。"""

    @classmethod
    def setUpClass(cls):
        jiateng_setup(services=("/enable_vel_control",))
        cls.hardware = LejuWheeledArmHardware(
            config={"skip_sdk_managers": True}
        )
        result = cls.hardware.get_vel_control_state_jiateng(timeout=3.0)
        cls.initial_state = (
            result.data["state"]
            if result.success
            else None
        )
        print(f"初始外部速度通道状态: {cls.initial_state}")

    @classmethod
    def tearDownClass(cls):
        if cls.initial_state is None:
            return

        result = cls.hardware.get_vel_control_state_jiateng(timeout=3.0)
        current_state = (
            result.data["state"]
            if result.success
            else None
        )
        if current_state != cls.initial_state:
            print(f"恢复外部速度通道状态: {cls.initial_state}")
            cls.hardware.enable_vel_control_jiateng(cls.initial_state)

    def setUp(self):
        assert_no_active_navigation_task()

    def test_01_get_current_state(self):
        """读取当前外部速度通道状态。"""
        result = self.hardware.get_vel_control_state_jiateng(timeout=3.0)
        self.assertTrue(result.success, result.message)
        self.assertIsInstance(result.data.get("state"), bool)
        print(f"当前外部速度通道状态: {result.data['state']}")

    def test_02_set_false(self):
        """设置 enable=False，关闭外部 /cmd_vel 转发。"""
        result = self.hardware.enable_vel_control_jiateng(False)
        self.assertTrue(result.success, result.message)
        self.assertIs(result.data.get("state_after"), False)
        print("外部速度通道已设置为 False")

    def test_03_set_true(self):
        """设置 enable=True，开启外部 /cmd_vel 转发。"""
        result = self.hardware.enable_vel_control_jiateng(True)
        self.assertTrue(result.success, result.message)
        self.assertIs(result.data.get("state_after"), True)
        print("外部速度通道已设置为 True")


if __name__ == "__main__":
    suite = unittest.TestSuite()

    # suite.addTest(TestEnableVelControl("test_01_get_current_state"))
    suite.addTest(TestEnableVelControl("test_02_set_false"))
    # suite.addTest(TestEnableVelControl("test_03_set_true"))

    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
