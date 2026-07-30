#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单臂末端离线整体时间最优轨迹 - TimedCmd 版本

轮臂末端独立控制 - 离线一次性提交整条带时间戳轨迹（整体时间最优），
躯干/底盘保持不动。底层 Ruckig 预规划，二阶连续三阶可导。

对齐源脚本 cmd_arm_ee_offline_traj_test.py：
  enable(True) → set_offline_trajectory → sleep(total_time) → enable(False)

离线专用约定（与在线 TimedCmd 不同！）：
  - planner_index: 0=左臂, 1=右臂
  - frame: 0=世界系, 1=局部系
  - times 为绝对时间（秒），第一帧必须 0，严格递增
  - cmd_vec 姿态字段直接收弧度（本脚本 CLI 输入度，内部转弧度）

【前置条件】
    source /opt/ros/noetic/setup.bash
    source infrastructure/ros_packages/devel/setup.bash   # kuavo_msgs 等消息包

【用法】
    # 左臂世界系，一行多个航点（t,x,y,z,yaw,pitch,roll，角度用度）
    python3 test_arm_ee_offline_traj.py --arm left --frame world \
        --traj "0,0.3,0.25,0.5,0,0,0" "1,0.5,0.25,0.5,0,0,0" "2,0.5,0.15,0.6,0,0,0"

    # 左臂世界系，从文件读航点（每行 t,x,y,z,yaw,pitch,roll）
    python3 test_arm_ee_offline_traj.py --arm left --frame world \
        --traj-file examples/offline_traj_left.csv

    # 右臂局部系，从文件读航点
    python3 test_arm_ee_offline_traj.py --arm right --frame local \
        --traj-file examples/offline_traj_right.csv

    # 结束后查静差
    python3 test_arm_ee_offline_traj.py --traj "0,0.3,0.25,0.5,0,0,0" "1,0.5,0.25,0.5,0,0,0" --get-error

    # 不带 --traj/--traj-file 时用内置默认轨迹
    python3 test_arm_ee_offline_traj.py

    # 查看全部参数
    python3 test_arm_ee_offline_traj.py -h

【默认参数】 --arm left --frame world --focus-ee False

【参数详解】
    --arm {left,right}      左臂/右臂。左臂世界系 y 正值向左，右臂局部系 y 负值向右。
    --frame {world,local}   坐标系。world=世界系（原点足底中心, x前/y左/z上）；
                            local=局部系（原点肩关节, y轴指向对侧肩）。
    --traj "t,x,y,z,yaw,pitch,roll" [...]
                            航点列表，可多个，空格分隔。时间秒，位置米，角度度。
                            第一帧时间必须为 0，时间严格递增。
                            示例: --traj "0,0.3,0.25,0.5,0,0,0" "1,0.5,0.25,0.5,0,0,0"
    --traj-file PATH        从 CSV 文件读航点，每行一个航点（7 值，# 开头为注释）。
                            示例文件: examples/offline_traj_left.csv
    --focus-ee              末端优先模式（默认 False=躯干优先）。
    --no-reset-torso        跳过躯干复位。
    --no-reset-arm          跳过手臂物理复位（仍切外部控制模式）。
    --get-error             轨迹执行完后查询静差。

【离线 vs 在线区别】
    离线（本脚本）: 一次性提交整条带时间戳轨迹，Ruckig 预规划整体时间最优，
                    二阶连续三阶可导，适合密集航点/示教回放/涂胶。
    在线（burst）:  逐点发在线服务，每点独立规划，MPC 在线平滑（有失真）。
    姿态单位: CLI 输入度，离线服务内部要求弧度（脚本自动转换）。
"""
import argparse
import math
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

# 默认轨迹 (t, [x, y, z, yaw, pitch, roll])（位置米，姿态弧度！）
DEFAULT_WAYPOINTS = [
    (0.0, [0.3, 0.25, 0.5, 0.0, 0.0, 0.0]),
    (1.0, [0.5, 0.25, 0.5, 0.0, 0.0, 0.0]),
    (2.0, [0.5, 0.15, 0.6, 0.0, 0.0, 0.0]),
    (3.0, [0.3, 0.15, 0.6, 0.0, 0.0, 0.0]),
]
POST_SETTLE = 0.5


def parse_waypoint(s):
    """把 't,x,y,z,yaw,pitch,roll' 解析成 (time, [x,y,z,yaw_deg,pitch_deg,roll_deg])。"""
    parts = [float(v.strip()) for v in s.split(',')]
    if len(parts) != 7:
        raise argparse.ArgumentTypeError(
            f"离线航点需 7 个值 t,x,y,z,yaw,pitch,roll，收到 {len(parts)} 个: {s}")
    t = parts[0]
    pose_deg = parts[1:]
    return t, pose_deg


def degrees_to_radians(pose_deg):
    """位置(米)不变，姿态(度)转弧度。顺序 [x,y,z,yaw,pitch,roll]。"""
    pose_rad = list(pose_deg)
    for i in [3, 4, 5]:
        pose_rad[i] = math.radians(pose_deg[i])
    return pose_rad


def load_waypoints_file(path):
    """从文件加载航点，每行 't,x,y,z,yaw,pitch,roll'，# 开头为注释。"""
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


def validate_times(waypoints):
    """校验：第一帧时间为 0、时间严格递增。"""
    if abs(waypoints[0][0]) > 1e-6:
        raise ValueError(f"第一帧时间必须为 0，收到 {waypoints[0][0]}")
    for j in range(1, len(waypoints)):
        if waypoints[j][0] <= waypoints[j - 1][0]:
            raise ValueError(
                f"时间未严格递增：第{j-1}点 t={waypoints[j-1][0]}，第{j}点 t={waypoints[j][0]}")


def _parse_args():
    parser = argparse.ArgumentParser(description='单臂末端离线整体时间最优轨迹，躯干/底盘不动')
    parser.add_argument('--arm', choices=['left', 'right'], default='left', help='左臂/右臂')
    parser.add_argument('--frame', choices=['world', 'local'], default='world',
                        help='world=世界系(基于odom), local=相对浮动基座')
    parser.add_argument('--traj', nargs='+', type=parse_waypoint, default=None,
                        metavar='t,x,y,z,yaw,pitch,roll',
                        help='航点列表(绝对时间秒/角度度)，可多个，空格分隔')
    parser.add_argument('--traj-file', type=str, default=None,
                        help='从文件读航点(每行 t,x,y,z,yaw,pitch,roll)')
    parser.add_argument('--focus-ee', action='store_true', default=False,
                        help='末端优先(默认False=躯干优先,躯干不动)')
    parser.add_argument('--no-reset-torso', action='store_true', help='跳过躯干复位')
    parser.add_argument('--no-reset-arm', action='store_true', help='跳过手臂复位')
    parser.add_argument('--get-error', action='store_true', help='结束后查静差')
    return parser.parse_args()


class TestArmEEOfflineTraj(unittest.TestCase):
    """单臂末端离线整体时间最优轨迹 - TimedCmd 版本测试

    【测试步骤】
    1. 设置 focus_ee（默认 False=躯干优先，末端不可扭曲躯干）
    2. enable_offline_trajectory(True) → set_offline_trajectory → sleep → enable(False)
    3. 预期: 手臂按整体时间最优轨迹平滑运动，躯干/底盘不动
    """

    hardware: IHardware = None
    arm: str = 'left'
    frame: str = 'world'
    waypoints: list = None   # [(t, [x,y,z,yaw,pitch,roll]), ...] 姿态弧度
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

    def test_offline_traj(self):
        """测试单臂末端离线整体时间最优轨迹"""
        waypoints = list(self.waypoints)
        if not waypoints:
            self.fail("至少需要 --traj 或 --traj-file 提供航点（或使用内置默认）")

        total_time = waypoints[-1][0]
        planner_index = 0 if self.arm == 'left' else 1
        frame_int = 0 if self.frame == 'world' else 1
        logger.info(f"--- {self.arm}臂/{self.frame}系 planner={planner_index} frame={frame_int} "
                    f"{len(waypoints)} 点, 总时长 {total_time:.2f}s ---")

        trajectories = [{
            'planner_index': planner_index,
            'frame': frame_int,
            'timed_traj': [
                {'desire_time': t, 'cmd_vec': list(pose)}
                for t, pose in waypoints
            ],
        }]

        # 1. 先使能
        r1 = self.hardware.enable_offline_trajectory(True)
        self.assertTrue(r1.success, f"离线轨迹使能失败: {r1.message}")

        # 2. 一次性提交
        r2 = self.hardware.set_offline_trajectory(trajectories)
        if not r2.success:
            self.hardware.enable_offline_trajectory(False)
        self.assertTrue(r2.success, f"离线轨迹提交失败: {r2.message}")

        # 3. 等待执行
        time.sleep(total_time + POST_SETTLE)

        # 4. 关闭使能
        r4 = self.hardware.enable_offline_trajectory(False)
        self.assertTrue(r4.success, f"离线轨迹关闭使能失败: {r4.message}")
        logger.info(f"离线轨迹执行完成, 总时长 {total_time:.2f}s")

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

    # 合并航点来源（度 → 弧度）
    waypoints = []
    if args.traj:
        waypoints.extend(args.traj)
    if args.traj_file:
        waypoints.extend(load_waypoints_file(args.traj_file))
    if not waypoints:
        waypoints = DEFAULT_WAYPOINTS
        logger.info(f"未提供 --traj/--traj-file，使用内置默认 {len(waypoints)} 航点")
    else:
        # CLI 输入姿态是度，离线 cmd_vec 需要弧度
        waypoints = [(t, degrees_to_radians(pose)) for t, pose in waypoints]

    validate_times(waypoints)

    TestArmEEOfflineTraj.arm = args.arm
    TestArmEEOfflineTraj.frame = args.frame
    TestArmEEOfflineTraj.waypoints = waypoints
    TestArmEEOfflineTraj.focus_ee = args.focus_ee
    TestArmEEOfflineTraj.no_reset_torso = args.no_reset_torso
    TestArmEEOfflineTraj.no_reset_arm = args.no_reset_arm
    TestArmEEOfflineTraj.get_error = args.get_error

    unittest.main(argv=[sys.argv[0]])
