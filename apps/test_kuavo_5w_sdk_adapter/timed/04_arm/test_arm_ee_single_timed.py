#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单臂末端单次位姿控制 - TimedCmd 版本

轮臂末端独立控制 - 单次末端位姿（planner 4/5/6/7），躯干/底盘保持不动。

对齐源脚本 cmd_arm_ee_only_test.py：
  set_control_mode(3) → focus_ee=False → 复位 → send_timed_single_command

【前置条件】
    source /opt/ros/noetic/setup.bash
    source infrastructure/ros_packages/devel/setup.bash   # kuavo_msgs 等消息包

【用法】
    # 左臂/世界系（默认），指定位姿
    python3 test_arm_ee_single_timed.py --pose 0.5 0.25 1.0 0 -90 0

    # 右臂/局部系，跳过复位，结束后查静差
    python3 test_arm_ee_single_timed.py --arm right --frame local --pose 0.5 -0.25 1.0 0 -90 0 --no-reset-torso --no-reset-arm --get-error
        

    # 查看全部参数
    python3 test_arm_ee_single_timed.py -h

【默认参数】 --arm left --frame world --time 2.0 --focus-ee False（--pose 必填）

【参数详解】
    --arm {left,right}      左臂/右臂。左臂世界系 y 正值向左，右臂局部系 y 负值向右。
    --frame {world,local}   坐标系。world=世界系（原点足底中心, x前/y左/z上）；
                            local=局部系（原点肩关节, y轴指向对侧肩）。
    --time FLOAT            期望执行时间（秒），默认 2.0。Ruckig 规划会尽量在此时间内完成。
    --pose X Y Z YAW PITCH ROLL
                            末端 6D 位姿（必填）。位置单位米，角度单位度。
                            顺序固定为 [x, y, z, yaw, pitch, roll]（ZYX 欧拉角）。
                            安全范围（world）：x∈[0.2,0.6], |y|∈[0.1,0.4], z∈[0.3,0.8]。
    --focus-ee              末端优先模式（默认 False=躯干优先）。
                            False 时末端指令不会扭曲躯干；True 时允许末端带动躯干。
    --no-reset-torso        跳过躯干复位。默认先将躯干回正到初始位姿。
    --no-reset-arm          跳过手臂物理复位（不拉回初始位置），但仍切外部控制模式。
                            注意：若上次运动后手臂处于奇异位形，可能导致规划失败。
    --get-error             运动结束后查询末端跟踪静差（位置米/姿态弧度）。
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
from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown

init_logging()
logger = get_logger(__name__)


def _parse_args():
    parser = argparse.ArgumentParser(description='单臂末端单次位姿（TimedCmd），躯干/底盘不动')
    parser.add_argument('--arm', choices=['left', 'right'], default='left', help='左臂/右臂')
    parser.add_argument('--frame', choices=['world', 'local'], default='world',
                        help='world=世界系(基于odom), local=相对浮动基座')
    parser.add_argument('--time', type=float, default=2.0, help='期望执行时间(秒)')
    parser.add_argument('--pose', nargs=6, type=float, required=True,
                        metavar=('x', 'y', 'z', 'yaw', 'pitch', 'roll'),
                        help='末端6D位姿, 位置米/角度度, 顺序 [x,y,z,yaw,pitch,roll]')
    parser.add_argument('--focus-ee', action='store_true', default=False,
                        help='保持末端优先(默认False=躯干优先,禁止末端扭曲躯干)')
    parser.add_argument('--no-reset-torso', action='store_true', help='跳过躯干复位')
    parser.add_argument('--no-reset-arm', action='store_true', help='跳过手臂复位')
    parser.add_argument('--get-error', action='store_true', help='运动结束后查询末端跟踪静差')
    return parser.parse_args()


class TestArmEESingleTimed(unittest.TestCase):
    """单臂末端单次位姿 - TimedCmd 版本测试

    【测试步骤】
    1. 设置 focus_ee（默认 False=躯干优先，末端不可扭曲躯干）
    2. 发送带时间的单臂末端位姿命令（planner 4/5/6/7）
    3. 预期: 手臂在期望时间内平滑移动到目标位姿，躯干/底盘不动
    """

    hardware: IHardware = None
    arm: str = 'left'
    frame: str = 'world'
    desire_time: float = 2.0
    pose: list = None
    focus_ee: bool = False
    no_reset_torso: bool = False
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
        """初始化机器人状态（末端独立控制）"""
        factory_setup(self.hardware,
                      need_arm=not self.no_reset_arm,
                      need_torso_reset=not self.no_reset_torso,
                      focus_ee=self.focus_ee,
                      focus_z=False)

    def tearDown(self):
        """恢复机器人状态"""
        factory_teardown(self.hardware, need_arm=not self.no_reset_arm)

    def test_ee_single(self):
        """测试单臂末端单次位姿"""
        fn = getattr(self.hardware, f"send_timed_{self.arm}_arm_ee_{self.frame}")
        logger.info(f"--- {self.arm}臂/{self.frame}系 目标位姿 {self.pose} ---")
        result = fn(pose=self.pose, desire_time=self.desire_time)
        self.assertTrue(result.success, f"发送指令失败: {result.message}")
        logger.info(f"指令下发成功: {result.message}")
        time.sleep(self.desire_time + 0.5)

        if self.get_error:
            res = self.hardware.get_ee_pose_reach_error(is_left=(self.arm == 'left'))
            if res.success:
                err = res.data.get('err_vector', [])
                logger.info(f"末端跟踪静差 [x,y,z,yaw,pitch,roll]={[round(v, 4) for v in err]} "
                            f"(位置m/姿态rad)")
            else:
                logger.warning(f"静差查询: {res.message}")


if __name__ == '__main__':
    args = _parse_args()
    TestArmEESingleTimed.arm = args.arm
    TestArmEESingleTimed.frame = args.frame
    TestArmEESingleTimed.desire_time = args.time
    TestArmEESingleTimed.pose = args.pose
    TestArmEESingleTimed.focus_ee = args.focus_ee
    TestArmEESingleTimed.no_reset_torso = args.no_reset_torso
    TestArmEESingleTimed.no_reset_arm = args.no_reset_arm
    TestArmEESingleTimed.get_error = args.get_error
    unittest.main(argv=[sys.argv[0]])
