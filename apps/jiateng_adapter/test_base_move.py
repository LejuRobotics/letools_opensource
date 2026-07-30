#!/usr/bin/env python3
"""
测试嘉腾底盘相对移动适配器。

适配器接口: LejuWheeledArmHardware.base_move_relative_jiateng()
ROS 服务:  /move_base/base_move (leju_mobile_base_msgs/BaseMove)

功能说明:
- 测试 allow_rotation=False 时的 x、y 平动
- 测试 allow_rotation=True 时的原地旋转
- 测试 allow_rotation=True 时的移动加旋转
- 验证服务返回 task_id，并等待任务到达

注意事项:
1. 这是相对移动，不是地图绝对目标点
2. x、y 单位为米，theta 单位为弧度
3. 只在 main 中保留一个 suite.addTest，避免连续执行多个运动
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
        self.assertNotIn(
            "accepted_zero_displacement",
            result.message,
            f"非零运动被判定为零位移: {result.message}",
        )

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
        """allow_rotation=False，执行 x、y 平动。"""
        self._run_relative_move(
            x=0.10,
            y=0.08,
            theta=0.0,
            allow_rotation=False,
        )

    def test_02_rotate_in_place(self):
        """allow_rotation=True，原地顺时针旋转 1.57rad。"""
        self._run_relative_move(
            x=0.0,
            y=0.0,
            theta=-1.57,
            allow_rotation=True,
        )

    def test_03_move_and_rotate(self):
        """allow_rotation=True，同时移动和旋转。"""
        self._run_relative_move(
            x=0.10,
            y=0.08,
            theta=0.30,
            allow_rotation=True,
        )


if __name__ == "__main__":
    suite = unittest.TestSuite()

    # suite.addTest(TestBaseMove("test_01_translation_without_rotation"))
    # suite.addTest(TestBaseMove("test_02_rotate_in_place"))
    suite.addTest(TestBaseMove("test_03_move_and_rotate"))

    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
