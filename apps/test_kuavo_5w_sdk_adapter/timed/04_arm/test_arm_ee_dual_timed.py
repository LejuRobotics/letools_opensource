#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂末端单次位姿控制 - TimedCmd 版本（躯干不回零）

轮臂末端独立控制 - 双臂单次末端位姿（planner 4+5/6+7），躯干/底盘保持不动。

对齐 test_arm_ee_single_timed.py：
  set_control_mode(3) → focus_ee=False → 手臂不复位 → send_arm_ee_*_timed → set_arm_control_mode(0) 释放末端保持

【前置条件】
    source /opt/ros/noetic/setup.bash
    source infrastructure/ros_packages/devel/setup.bash   # kuavo_msgs 等消息包

【用法】
    # 世界系（默认），分别指定左右臂位姿，躯干不回零
    python3 test_arm_ee_dual_timed.py --left-pose 0.5 0.25 0.7 0 -90 0 --right-pose 0.5 -0.25 0.7 0 -90 0

    # 局部系，跳过手臂复位，结束后查询静差
    python3 test_arm_ee_dual_timed.py --frame local --left-pose 0.5 0.2 0.7 0 -90 0 --right-pose 0.5 -0.2 0.7 0 -90 0 --no-reset-arm --get-error

    # 查看全部参数
    python3 test_arm_ee_dual_timed.py -h

【默认参数】 --frame world --time 2.0 --focus-ee False，固定躯干不回零（--left-pose/--right-pose 必填）

【参数详解】
    --frame {world,local}       坐标系。world=世界系（原点足底中心, x前/y左/z上）；
                                local=局部系（原点肩关节, y轴指向对侧肩）。
    --time FLOAT                期望执行时间（秒），默认 2.0。Ruckig 规划会尽量在此时间内完成。
    --left-pose X Y Z YAW PITCH ROLL
                                左臂末端 6D 位姿（必填）。位置单位米，角度单位度。
    --right-pose X Y Z YAW PITCH ROLL
                                右臂末端 6D 位姿（必填）。位置单位米，角度单位度。
                                顺序固定为 [x, y, z, yaw, pitch, roll]（ZYX 欧拉角）。
    --focus-ee                  末端优先模式（默认 False=躯干优先）。
                                False 时末端指令不会扭曲躯干；True 时允许末端带动躯干。
    --no-reset-arm              跳过手臂物理复位（不拉回初始位置），但仍切外部控制模式。
                                注意：若上次运动后手臂处于奇异位形，可能导致规划失败。
    --get-error                 运动结束后查询双臂末端跟踪静差（位置米/姿态弧度）。
"""
import argparse
import sys
import time
import unittest
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
from adapters.hardware.factory import HardwareFactory
from core.interfaces.i_hardware import IHardware
from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup

init_logging()
logger = get_logger(__name__)


def _parse_args():
    parser = argparse.ArgumentParser(description='双臂末端单次位姿（TimedCmd），躯干不回零/底盘不动')
    parser.add_argument('--frame', choices=['world', 'local'], default='world',
                        help='world=世界系(基于odom), local=相对浮动基座')
    parser.add_argument('--time', type=float, default=2.0, help='期望执行时间(秒)')
    parser.add_argument('--left-pose', nargs=6, type=float, required=True,
                        metavar=('x', 'y', 'z', 'yaw', 'pitch', 'roll'),
                        help='左臂末端6D位姿, 位置米/角度度, 顺序 [x,y,z,yaw,pitch,roll]')
    parser.add_argument('--right-pose', nargs=6, type=float, required=True,
                        metavar=('x', 'y', 'z', 'yaw', 'pitch', 'roll'),
                        help='右臂末端6D位姿, 位置米/角度度, 顺序 [x,y,z,yaw,pitch,roll]')
    parser.add_argument('--focus-ee', action='store_true', default=False,
                        help='保持末端优先(默认False=躯干优先,禁止末端扭曲躯干)')
    parser.add_argument('--no-reset-arm', action='store_true', help='兼容参数：当前脚本固定不复位手臂')
    parser.add_argument('--get-error', action='store_true', help='运动结束后查询双臂末端跟踪静差')
    return parser.parse_args()


class TestArmEEDualTimed(unittest.TestCase):
    """双臂末端单次位姿 - TimedCmd 版本测试（躯干不回零）

    【测试步骤】
    1. 设置 focus_ee（默认 False=躯干优先，末端不可扭曲躯干）
    2. 不执行躯干复位，仅按参数决定是否复位手臂
    3. 发送带时间的双臂末端位姿命令（planner 4+5/6+7）
    4. 预期: 双臂在期望时间内平滑移动到目标位姿，躯干/底盘不动
    """

    hardware: IHardware = None
    frame: str = 'local'
    desire_time: float = 2.0
    left_pose: list = None
    right_pose: list = None
    focus_ee: bool = False
    no_reset_arm: bool = False
    get_error: bool = False

    @classmethod
    def setUpClass(cls):
        cls.hardware = HardwareFactory.create_hardware(
            config={
                'robot_type': 'leju_wheeled',
                'sdk_managers_whitelist': ['timed'],
                'skip_end_effector': True,
                'skip_camera': True,
                'skip_chassis': True,
                'skip_token_manager': True
            }
        )
        cls.hardware.initialize()

    @classmethod
    def tearDownClass(cls):
        cls.hardware.shutdown()

    def setUp(self):
        """初始化机器人状态：躯干固定不回零，双臂末端独立控制；不复位手臂"""
        factory_setup(self.hardware,
                      need_arm_reset=False,
                      need_torso_reset=False,
                      focus_ee=self.focus_ee,
                      focus_z=False)

    def tearDown(self):
        """保持运动后状态：释放末端保持，不复位手臂，不复位躯干"""
        logger.info("--- 后置保持：释放末端保持，不复位手臂，躯干保持不动 ---")
        result = self.hardware.set_arm_control_mode(0)
        if result.success:
            logger.info("已释放外部末端保持: set_arm_control_mode(0)")
        else:
            logger.warning(f"释放外部末端保持警告: {result.message}")

    def test_ee_dual(self):
        """测试双臂末端单次位姿"""
        fn = getattr(self.hardware, f"send_arm_ee_{self.frame}_timed")
        logger.info(f"--- 双臂/{self.frame}系 目标位姿 left={self.left_pose}, right={self.right_pose} ---")
        result = fn(left_pose=self.left_pose, right_pose=self.right_pose, desire_time=self.desire_time)
        self.assertTrue(result.success, f"发送指令失败: {result.message}")
        logger.info(f"指令下发成功: {result.message}")
        time.sleep(self.desire_time + 0.5)

        if self.get_error:
            for arm_name, is_left in [('左臂', True), ('右臂', False)]:
                res = self.hardware.get_ee_pose_reach_error(is_left=is_left)
                if res.success:
                    err = res.data.get('err_vector', [])
                    logger.info(f"{arm_name}末端跟踪静差 [x,y,z,yaw,pitch,roll]={[round(v, 4) for v in err]} "
                                f"(位置m/姿态rad)")
                else:
                    logger.warning(f"{arm_name}静差查询: {res.message}")


if __name__ == '__main__':
    args = _parse_args()
    TestArmEEDualTimed.frame = args.frame
    TestArmEEDualTimed.desire_time = args.time
    TestArmEEDualTimed.left_pose = args.left_pose
    TestArmEEDualTimed.right_pose = args.right_pose
    TestArmEEDualTimed.focus_ee = args.focus_ee
    TestArmEEDualTimed.no_reset_arm = args.no_reset_arm
    TestArmEEDualTimed.get_error = args.get_error
    unittest.main(argv=[sys.argv[0]])
