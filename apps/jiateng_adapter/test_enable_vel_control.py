#!/usr/bin/env python3
"""
测试嘉腾底盘外部速度通道开关。

适配器接口:
- LejuWheeledArmHardware.enable_vel_control_jiateng()
- LejuWheeledArmHardware.get_vel_control_state_jiateng()

ROS 服务: /enable_vel_control
ROS 话题: /enable_vel_control_state

注意事项:
1. enable=True：外部 /cmd_vel、/cmd_pose、/cmd_pose_world 控制模式
2. enable=False：嘉腾 /move_base 导航模式
3. 两种模式不能与另一类运动命令并发使用
4. 本脚本保留设置结果，不会自动恢复原状态
"""

import os
import sys
import unittest
import rospy

PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)
sys.path.insert(0, PROJECT_ROOT)

from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from apps.jiateng_adapter._scaffold import jiateng_setup


class TestEnableVelControl(unittest.TestCase):
    """嘉腾底盘外部速度通道测试。"""

    @classmethod
    def setUpClass(cls):
        jiateng_setup(services=("/enable_vel_control",))
        cls.hardware = LejuWheeledArmHardware(
            config={"skip_sdk_managers": True}
        )
        result = cls.hardware.get_vel_control_state_jiateng(timeout=3.0)
        cls.initial_state = result.data["state"] if result.success else None
        print(f"初始控制模式: {cls.initial_state}")

    def test_01_get_current_state(self):
        """读取当前外部速度通道状态。"""
        result = self.hardware.get_vel_control_state_jiateng(timeout=3.0)
        self.assertTrue(result.success, result.message)
        self.assertIsInstance(result.data.get("state"), bool)
        print(f"当前外部速度通道状态: {result.data['state']}")

    def test_02_set_true(self):
        """切换到外部 /cmd_vel、/cmd_pose 控制模式。"""
        result = self.hardware.enable_vel_control_jiateng(True)
        self.assertTrue(result.success, result.message)
        self.assertIs(result.data.get("state_after"), True)
        print("已切换到外部 /cmd_vel、/cmd_pose 控制模式")

    def test_03_set_false(self):
        """切换到嘉腾 /move_base 导航模式。"""
        result = self.hardware.enable_vel_control_jiateng(False)
        self.assertTrue(result.success, result.message)
        self.assertIs(result.data.get("state_after"), False)
        print("已切换到嘉腾 /move_base 导航模式")


if __name__ == "__main__":
    suite = unittest.TestSuite()

    suite.addTest(TestEnableVelControl("test_01_get_current_state"))
    rospy.sleep(0.5)
    suite.addTest(TestEnableVelControl("test_02_set_true"))
    rospy.sleep(0.5)
    suite.addTest(TestEnableVelControl("test_01_get_current_state"))
    rospy.sleep(0.5)
    suite.addTest(TestEnableVelControl("test_03_set_false"))
    rospy.sleep(0.5)
    suite.addTest(TestEnableVelControl("test_01_get_current_state"))

    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
