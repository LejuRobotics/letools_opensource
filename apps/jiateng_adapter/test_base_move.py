#!/usr/bin/env python3
"""
验证嘉腾底盘的本体坐标系相对移动。

适配器接口：``LejuWheeledArmHardware.base_move_relative_jiateng()``
ROS 服务：``/move_base/base_move``（``leju_mobile_base_msgs/BaseMove``）

覆盖场景：
- ``allow_rotation=False``：x、y 平动；
- ``allow_rotation=True``：原地旋转；
- ``allow_rotation=True``：移动并旋转。

每个用例都会等待服务返回的 ``task_id`` 到达。测试前若检测到外部控制模式，
会自动切换到嘉腾导航模式（``/enable_vel_control=false``）；该状态在测试结束后保持不变。

``x``、``y`` 的单位为米，``theta`` 的单位为弧度；它们均相对于机器人当前
本体坐标系，不是 ``map`` 坐标。
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
from apps.jiateng_adapter._scaffold import (
    assert_no_active_navigation_task,
    assert_vel_control_state,
    jiateng_setup,
    read_vel_control_state,
)
from core.domain.chassis_options import MoveToTargetOptions


class TestBaseMove(unittest.TestCase):
    """嘉腾底盘相对移动测试。"""

    @classmethod
    def setUpClass(cls):
        jiateng_setup(
            services=(
                "/move_base/base_move",
                "/move_base/check_arrived",
            )
        )
        cls.hardware = LejuWheeledArmHardware(
            config={"skip_sdk_managers": True}
        )

    def setUp(self):
        # /move_base 导航需要独占嘉腾控制权。
        if read_vel_control_state():
            print(
                "WARNING: 当前为外部控制模式，"
                "已切换到嘉腾 /move_base 导航模式后继续测试。"
            )
            result = self.hardware.enable_vel_control_jiateng(False)
            self.assertTrue(result.success, result.message)

        assert_vel_control_state(False)
        assert_no_active_navigation_task()

    def _run_relative_move(
        self,
        *,
        x,
        y,
        theta,
        allow_rotation,
        timeout=60.0,
    ):
        """提交一个相对导航任务，并等待该任务到达。"""
        options = MoveToTargetOptions(
            avoid_enabled=True,
            avoid_distance=0.5,
            linear_velocity=0.08,
            angular_velocity=0.15,
            position_threshold=0.03,
            angle_threshold=0.08,
            allow_rotation=allow_rotation,
        )

        print(
            f"测试参数: x={x}, y={y}, theta={theta}, "
            f"allow_rotation={allow_rotation}"
        )
        result = self.hardware.base_move_relative_jiateng(
            x=x,
            y=y,
            theta=theta,
            options=options,
        )
        print("base_move response:", result.message)
        self.assertTrue(result.success, result.message)

        self.assertIsNotNone(result.data, "服务成功但未返回数据")
        task_id = result.data.get("task_id")
        self.assertTrue(task_id, "返回结果中缺少 task_id")

        arrived = self.hardware.check_arrived_jiateng(
            task_id=task_id,
            blocking=True,
            timeout=timeout,
        )
        self.assertTrue(arrived.success, arrived.message)
        self.assertIsNotNone(arrived.data, "到达查询未返回数据")
        self.assertTrue(arrived.data.get("arrived"), arrived.data)

        print(f"相对移动完成: task_id={task_id}")
        return task_id

    def test_01_translation_without_rotation(self):
        """不改变朝向，执行本体坐标系 x、y 平动。"""
        self._run_relative_move(
            x=0.10,
            y=0.10,
            theta=0.0,
            allow_rotation=False,
        )

    def test_02_rotate_in_place(self):
        """允许旋转，原地顺时针旋转 1.57rad。"""
        self._run_relative_move(
            x=0.0,
            y=0.0,
            theta=-1.57,
            allow_rotation=True,
        )

    def test_03_move_and_rotate(self):
        """允许旋转，同时执行平动和朝向变化。"""
        self._run_relative_move(
            x=0.10,
            y=0.08,
            theta=0.30,
            allow_rotation=True,
        )


if __name__ == "__main__":
    suite = unittest.TestSuite()

    # 按以下顺序执行三个真实运动任务；可按现场需要注释掉任意用例。
    suite.addTest(TestBaseMove("test_01_translation_without_rotation"))
    rospy.sleep(0.5)
    suite.addTest(TestBaseMove("test_02_rotate_in_place"))
    rospy.sleep(0.5)
    suite.addTest(TestBaseMove("test_03_move_and_rotate"))

    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
