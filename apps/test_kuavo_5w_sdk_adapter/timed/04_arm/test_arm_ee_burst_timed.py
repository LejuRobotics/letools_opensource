#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单臂末端在线连发航点 - TimedCmd 版本

轮臂末端独立控制 - 在线连发多个末端航点（循环 send_timed_single_command），
躯干/底盘保持不动。航点间由 MPC 在线平滑插值（二阶连续、三阶可导，有失真）。

对齐源脚本 cmd_arm_ee_traj_stream_test.py。

【前置条件】
    source /opt/ros/noetic/setup.bash
    source infrastructure/ros_packages/devel/setup.bash   # kuavo_msgs 等消息包

【用法】
    # 左臂世界系，3个航点，每个2秒
    python3 test_arm_ee_burst_timed.py --arm left --frame world --time 2.0 \
        --traj "0.1,0.4,0.7,0,0,0" "0.3,0.4,0.7,0,-90,0" "0.3,0.2,0.85,0,-90,0"

    # 从文件读航点（每行 x,y,z,yaw,pitch,roll，角度用度）
    python3 test_arm_ee_burst_timed.py --arm left --frame world --time 2.0 \
        --traj-file examples/burst_waypoints_left.csv

    # 右臂局部系，从文件读航点
    python3 test_arm_ee_burst_timed.py --arm right --frame local --time 1.5 \
        --traj-file examples/burst_waypoints_right.csv

    # 起点设为当前位姿，并在结束时查静差
    python3 test_arm_ee_burst_timed.py --arm left --frame world --time 2.0 \
        --from-current --traj "0.3,0.4,0.7,0,-90,0" --get-error

    # 不带 --traj/--traj-file 时用内置默认 3 航点
    python3 test_arm_ee_burst_timed.py

    # 查看全部参数
    python3 test_arm_ee_burst_timed.py -h

【默认参数】 --arm left --frame world --time 2.0 --settle 1.0 --focus-ee False

【参数详解】
    --arm {left,right}      左臂/右臂。左臂世界系 y 正值向左，右臂局部系 y 负值向右。
    --frame {world,local}   坐标系。world=世界系（原点足底中心, x前/y左/z上）；
                            local=局部系（原点肩关节, y轴指向对侧肩）。
    --time FLOAT            每个航点的期望执行时间（秒），默认 2.0。
    --traj "x,y,z,yaw,pitch,roll" [...]
                            航点列表，可多个，空格分隔。位置米，角度度。
                            示例: --traj "0.3,0.25,0.5,0,0,0" "0.5,0.25,0.5,0,-90,0"
    --traj-file PATH        从 CSV 文件读航点，每行一个航点（6 值，# 开头为注释）。
                            示例文件: examples/burst_waypoints_left.csv
    --from-current          以当前末端位姿为起点插入第 0 航点，实现连续轨迹。
    --settle FLOAT          每个航点后的额外等待时间（秒），默认 1.0。
    --focus-ee              末端优先模式（默认 False=躯干优先）。
    --no-reset-torso        跳过躯干复位。
    --no-reset-arm          跳过手臂物理复位（仍切外部控制模式）。
    --get-error             每个航点到达后查询静差。
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

# 默认航点 [[x, y, z, yaw, pitch, roll], ...]（位置米，姿态度）
DEFAULT_WAYPOINTS = [
    [0.3, 0.25, 0.5, 0, 0, 0],
    [0.5, 0.25, 0.5, 0, 0, 0],
    [0.3, 0.25, 0.7, 0, 0, 0],
]


def parse_waypoint(s):
    """把 '0.3,0.4,0.7,0,-90,0' 解析成 6 元组 float（位置米/角度度）。"""
    parts = [float(v.strip()) for v in s.split(',')]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            f"航点需 6 个值 x,y,z,yaw,pitch,roll，收到 {len(parts)} 个: {s}")
    return list(parts)


def load_waypoints_file(path):
    """从文件加载航点，每行 'x,y,z,yaw,pitch,roll'，# 开头为注释。"""
    pts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            pts.append(parse_waypoint(line))
    if not pts:
        raise ValueError(f"文件 {path} 中未解析到任何航点")
    return pts


def _parse_args():
    parser = argparse.ArgumentParser(description='单臂末端在线连发航点（TimedCmd），躯干/底盘不动')
    parser.add_argument('--arm', choices=['left', 'right'], default='left', help='左臂/右臂')
    parser.add_argument('--frame', choices=['world', 'local'], default='world',
                        help='world=世界系(基于odom), local=相对浮动基座')
    parser.add_argument('--time', type=float, default=2.0, help='每个航点期望执行时间(秒)')
    parser.add_argument('--traj', nargs='+', type=parse_waypoint, default=None,
                        metavar='x,y,z,yaw,pitch,roll',
                        help='航点列表(角度度)，可多个，空格分隔')
    parser.add_argument('--traj-file', type=str, default=None,
                        help='从文件读航点(每行 x,y,z,yaw,pitch,roll)')
    parser.add_argument('--from-current', action='store_true', default=False,
                        help='以当前末端位姿为起点插入第0航点，连续连发')
    parser.add_argument('--focus-ee', action='store_true', default=False,
                        help='末端优先(默认False=躯干优先,躯干不动)')
    parser.add_argument('--no-reset-torso', action='store_true', help='跳过躯干复位')
    parser.add_argument('--no-reset-arm', action='store_true', help='跳过手臂复位')
    parser.add_argument('--get-error', action='store_true', help='结束后查静差')
    parser.add_argument('--settle', type=float, default=1.0, help='每个航点后额外等待(秒)')
    return parser.parse_args()


class TestArmEEBurstTimed(unittest.TestCase):
    """单臂末端在线连发航点 - TimedCmd 版本测试

    【测试步骤】
    1. 设置 focus_ee（默认 False=躯干优先，末端不可扭曲躯干）
    2. 逐点连发 send_timed_{arm}_arm_ee_{frame}（planner 4/5/6/7）
    3. 每段 desire_time，settle 额外等待
    4. 预期: 手臂依次平滑经过各航点，躯干/底盘不动
    """

    hardware: IHardware = None
    arm: str = 'left'
    frame: str = 'world'
    desire_time: float = 2.0
    waypoints: list = None
    focus_ee: bool = False
    no_reset_torso: bool = False
    no_reset_arm: bool = False
    get_error: bool = False
    settle: float = 1.0

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

        # from-current: 读当前末端位姿插入第0航点
        if getattr(cls, '_from_current', False):
            poses = cls.hardware.get_ee_poses()
            if poses and len(poses) >= 2:
                entry = poses[0] if cls.arm == 'left' else poses[1]
                pos = entry.get('position', {})
                ori = entry.get('orientation_euler', {})
                current = [pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0),
                           ori.get('yaw', 0.0), ori.get('pitch', 0.0), ori.get('roll', 0.0)]
                cls.waypoints = [current] + cls.waypoints
                logger.info(f"--from-current: 当前位姿 {[round(v, 3) for v in current]} 已插入为第0航点")
            else:
                logger.warning("--from-current: 无法读取当前末端位姿，按原航点执行")

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

    def test_burst(self):
        """测试单臂末端连发航点"""
        waypoints = list(self.waypoints)
        if not waypoints:
            self.fail("至少需要 --traj 或 --traj-file 提供航点（或使用内置默认）")

        fn = getattr(self.hardware, f"send_timed_{self.arm}_arm_ee_{self.frame}")
        logger.info(f"--- {self.arm}臂/{self.frame}系 连发 {len(waypoints)} 航点 ---")

        total_actual = 0.0
        for i, wp in enumerate(waypoints, 1):
            result = fn(pose=wp, desire_time=self.desire_time)
            self.assertTrue(result.success, f"第 {i} 航点下发失败: {result.message}")
            logger.info(f"[{i}/{len(waypoints)}] 已下发: {[round(v, 3) for v in wp]}")
            time.sleep(self.desire_time + self.settle)
            total_actual += self.desire_time
            logger.info(f"[{i}/{len(waypoints)}] 完成")

        logger.info(f"连发完成, {len(waypoints)} 点, 累计约 {total_actual:.2f}s")

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

    # 合并航点来源
    waypoints = []
    if args.traj:
        waypoints.extend(args.traj)
    if args.traj_file:
        waypoints.extend(load_waypoints_file(args.traj_file))
    if not waypoints:
        waypoints = DEFAULT_WAYPOINTS
        logger.info(f"未提供 --traj/--traj-file，使用内置默认 {len(waypoints)} 航点")

    TestArmEEBurstTimed.arm = args.arm
    TestArmEEBurstTimed.frame = args.frame
    TestArmEEBurstTimed.desire_time = args.time
    TestArmEEBurstTimed.focus_ee = args.focus_ee
    TestArmEEBurstTimed.no_reset_torso = args.no_reset_torso
    TestArmEEBurstTimed.no_reset_arm = args.no_reset_arm
    TestArmEEBurstTimed.get_error = args.get_error
    TestArmEEBurstTimed.settle = args.settle

    TestArmEEBurstTimed.waypoints = waypoints
    TestArmEEBurstTimed._from_current = args.from_current

    unittest.main(argv=[sys.argv[0]])
