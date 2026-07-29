#!/usr/bin/env python3
"""
测试嘉腾底盘 map 绝对目标点移动适配器。

适配器接口: LejuWheeledArmHardware.base_move_to_target_jiateng()
ROS 服务: /move_base/move_to_target

注意事项:
1. 目标使用 map 坐标，不是机器人坐标系相对位移
2. 当前位姿读取自 /move_base/amcl_pose
3. map x 增加不一定等于机器人正前方
4. 只在 main 中保留一个 suite.addTest
"""

import math
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
    read_current_map_pose,
)
from core.domain.chassis_options import MoveToTargetOptions


class TestMoveToTarget(unittest.TestCase):
    """嘉腾底盘绝对位置移动测试。"""

    @classmethod
    def setUpClass(cls):
        jiateng_setup(
            services=(
                "/move_base/move_to_target",
                "/move_base/check_arrived",
            )
        )
        cls.hardware = LejuWheeledArmHardware(
            config={"skip_sdk_managers": True}
        )

    def setUp(self):
        assert_no_active_navigation_task()

    def _run_absolute_target(
        self,
        *,
        target_x,
        target_y,
        target_theta,
        options,
        timeout=60.0,
    ):
        print(
            f"绝对目标: x={target_x:.3f}, y={target_y:.3f}, "
            f"theta={target_theta:.3f}"
        )
        result = self.hardware.base_move_to_target_jiateng(
            x=target_x,
            y=target_y,
            theta=target_theta,
            options=options,
        )
        print("move_to_target response:", result.message)
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
        print(f"绝对位置移动完成: task_id={task_id}")

    def test_01_nearby_target_from_amcl_pose(self):
        """从当前 map 位置向 map x 正方向移动 0.10m。"""
        x, y, theta = read_current_map_pose()
        options = MoveToTargetOptions(
            avoid_enabled=True,
            avoid_distance=0.5,
            linear_velocity=0.08,
            angular_velocity=0.15,
            position_threshold=0.03,
            angle_threshold=0.08,
            allow_rotation=True,
        )
        self._run_absolute_target(
            target_x=x + 0.10,
            target_y=y,
            target_theta=theta,
            options=options,
        )

    def test_02_move_and_change_heading(self):
        """移动到附近 map 目标，同时改变朝向 0.30rad。"""
        x, y, theta = read_current_map_pose()
        target_theta = math.atan2(
            math.sin(theta + 0.30),
            math.cos(theta + 0.30),
        )
        options = MoveToTargetOptions(
            avoid_enabled=True,
            avoid_distance=0.5,
            linear_velocity=0.08,
            angular_velocity=0.15,
            position_threshold=0.03,
            angle_threshold=0.08,
            allow_rotation=True,
        )
        self._run_absolute_target(
            target_x=x + 0.08,
            target_y=y + 0.08,
            target_theta=target_theta,
            options=options,
        )


if __name__ == "__main__":
    suite = unittest.TestSuite()

    # suite.addTest(TestMoveToTarget("test_01_nearby_target_from_amcl_pose"))
    suite.addTest(TestMoveToTarget("test_02_move_and_change_heading"))

    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
