#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""basket_vision + 双臂箱体抓取搬运集成脚本。

默认是安全调试模式：读取 /tag_detections、转换到 base_link、生成并打印轨迹，
但不驱动机器人。只有显式传入 --execute 才执行真机动作。

视觉消息：
    apriltag_ros/AprilTagDetectionArray
    pose = [x, y, z, qx, qy, qz, qw]
    frame_id 通常为 camera_color_optical_frame

轨迹格式：
    [x, y, z, roll, pitch, yaw]，位置单位 m，角度单位 deg。
"""

import argparse
import logging
import math
import statistics
import sys
import time
from dataclasses import dataclass, fields
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rospy
import tf2_geometry_msgs  # noqa: F401 - 注册 PoseStamped 的 TF2 转换
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from core.common.math_utils import quaternion_to_euler
from core.domain.enums import MPCControlMode


LOG = logging.getLogger("boxcarry")

LEFT_PLANNER_LOCAL = 6
RIGHT_PLANNER_LOCAL = 7


@dataclass(frozen=True)
class DetectedPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    detection_id: int
    source_frame: str
    target_frame: str


@dataclass
class CarryPlan:
    left_out: List[float]
    left_out2: List[float]
    left_grasp: List[float]
    left_up: List[float]
    left_pull: List[float]
    right_out: List[float]
    right_out2: List[float]
    right_grasp: List[float]
    right_up: List[float]
    right_pull: List[float]
    chest_up_left: List[float]
    chest_up_right: List[float]
    chest_left: List[float]
    chest_right: List[float]
    table_left: List[float]
    table_right: List[float]
    expand_left: List[float]
    expand_right: List[float]
    reset_left: List[float]
    reset_right: List[float]


def decode_basket_id(combined_id: int) -> Tuple[int, int]:
    """basket_vision ID = data_id(3位) * 100 + basket_class(2位)。"""
    return divmod(int(combined_id), 100)


def id_matches(combined_id: int, requested_id: int) -> bool:
    """-1 匹配首个目标；三位 ID 匹配 data_id；更长 ID 精确匹配。"""
    if requested_id < 0:
        return True
    if requested_id <= 999:
        data_id, _ = decode_basket_id(combined_id)
        return data_id == requested_id
    return combined_id == requested_id


def pose_distance(a: DetectedPose, b: DetectedPose) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def circular_mean(values: Sequence[float]) -> float:
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


class StableBasketDetector:
    """收集一帧或多帧位姿，并统一转换到机械臂目标坐标系。"""

    def __init__(
        self,
        topic: str,
        requested_id: int,
        samples: int,
        timeout: float,
        max_spread: float,
        target_frame: str,
        tf_timeout: float,
    ):
        self.topic = topic
        self.requested_id = requested_id
        self.samples = max(1, samples)
        self.timeout = timeout
        self.max_spread = max_spread
        self.target_frame = target_frame
        self.tf_timeout = tf_timeout
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _select_detection(self, msg: AprilTagDetectionArray):
        candidates = [
            det for det in msg.detections
            if det.id and id_matches(int(det.id[0]), self.requested_id)
        ]
        if not candidates:
            return None
        # 默认选择离相机最近的目标，避免检测顺序变化导致目标跳变。
        return min(
            candidates,
            key=lambda det: (
                det.pose.pose.pose.position.x ** 2
                + det.pose.pose.pose.position.y ** 2
                + det.pose.pose.pose.position.z ** 2
            ),
        )

    def _to_target_frame(
        self, msg: AprilTagDetectionArray, det
    ) -> DetectedPose:
        source = det.pose.header.frame_id or msg.header.frame_id
        if not source:
            raise RuntimeError("检测消息缺少 frame_id，禁止将未知坐标系用于机械臂")

        stamped = PoseStamped()
        stamped.header = det.pose.header
        if stamped.header.stamp == rospy.Time():
            stamped.header.stamp = msg.header.stamp
        stamped.header.frame_id = source
        stamped.pose = det.pose.pose.pose

        if source != self.target_frame:
            try:
                stamped = self.tf_buffer.transform(
                    stamped,
                    self.target_frame,
                    rospy.Duration(self.tf_timeout),
                )
            except Exception as exc:
                raise RuntimeError(
                    f"无法把视觉位姿从 {source} 转换到 {self.target_frame}: {exc}"
                )

        q = stamped.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        p = stamped.pose.position
        return DetectedPose(
            x=p.x,
            y=p.y,
            z=p.z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            detection_id=int(det.id[0]),
            source_frame=source,
            target_frame=self.target_frame,
        )

    def detect(self) -> DetectedPose:
        deadline = time.monotonic() + self.timeout
        observations: List[DetectedPose] = []
        selected_id: Optional[int] = None
        last_stamp = None

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            remaining = max(0.05, deadline - time.monotonic())
            try:
                msg = rospy.wait_for_message(
                    self.topic,
                    AprilTagDetectionArray,
                    timeout=min(1.0, remaining),
                )
            except rospy.ROSException:
                continue

            stamp = msg.header.stamp.to_nsec()
            if stamp and stamp == last_stamp:
                continue
            last_stamp = stamp

            det = self._select_detection(msg)
            if det is None:
                continue
            detection_id = int(det.id[0])
            if selected_id is None:
                selected_id = detection_id
            if detection_id != selected_id:
                continue

            try:
                observations.append(self._to_target_frame(msg, det))
            except RuntimeError as exc:
                LOG.warning("%s", exc)
                continue
            if len(observations) >= self.samples:
                break

        if len(observations) < self.samples:
            raise RuntimeError(
                f"{self.timeout:.1f}s 内仅取得 "
                f"{len(observations)}/{self.samples} 个有效视觉样本"
            )

        xs = [p.x for p in observations]
        ys = [p.y for p in observations]
        zs = [p.z for p in observations]
        center = DetectedPose(
            x=statistics.median(xs),
            y=statistics.median(ys),
            z=statistics.median(zs),
            roll=circular_mean([p.roll for p in observations]),
            pitch=circular_mean([p.pitch for p in observations]),
            yaw=circular_mean([p.yaw for p in observations]),
            detection_id=observations[0].detection_id,
            source_frame=observations[0].source_frame,
            target_frame=observations[0].target_frame,
        )
        spread = max(pose_distance(p, center) for p in observations)
        if spread > self.max_spread:
            raise RuntimeError(
                f"视觉位姿不稳定：最大偏差 {spread:.4f}m，"
                f"阈值 {self.max_spread:.4f}m"
            )
        LOG.info("视觉稳定性：%d 帧，最大偏差 %.4fm", len(observations), spread)
        return center


def rotate_xy(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def offset_xyz(
    pose: DetectedPose, local_x: float, local_y: float, local_z: float
) -> Tuple[float, float, float]:
    dx, dy = rotate_xy(local_x, local_y, pose.yaw)
    return pose.x + local_x, pose.y + local_y, pose.z + local_z


def build_plan(box: DetectedPose, args) -> CarryPlan:
    """只用视觉替换抓取/提起/拉出点；搬运后半段保留已验证固定轨迹。"""
    side = args.box_width / 2.0 + args.side_clearance + 0.12
    grasp_x_offset_r = 0.05
    grasp_z_offset_r = 0.15
    lx, ly, lz = offset_xyz(
        box, args.grasp_x_offset, side, args.grasp_z_offset
    )
    rx, ry, rz = offset_xyz(
            box, grasp_x_offset_r, -side, grasp_z_offset_r
    )
    left_grasp = [lx+0.05, ly-0.06, lz-0.14, 0, -90, -10 ]
    right_grasp = [rx-0.05, ry+0.40, rz+0.00, 0, -90, 10]

    left_out_xyz = offset_xyz(
        box,
        args.grasp_x_offset-0.03,
        side + args.approach_side_distance-0.06,
        args.grasp_z_offset + args.approach_height-0.13,
    )
    left_out2_xyz = offset_xyz(
            box,
            args.grasp_x_offset-0.03,
            side + args.approach_side_distance-0.06,
            args.grasp_z_offset + args.approach_height-0.13-0.1,
    )
    right_out_xyz = offset_xyz(
        box,
        grasp_x_offset_r-0.1,
        -side - args.approach_side_distance+0.49,
        args.grasp_z_offset+0.13 ,
    )
    right_out2_xyz = offset_xyz(
            box,
            grasp_x_offset_r-0.1,
            -side - args.approach_side_distance+0.49,
            args.grasp_z_offset+0.13-0.26 ,
    )
    left_up_xyz = offset_xyz(
        box,
        args.grasp_x_offset,
        side-0.07,
        args.grasp_z_offset + args.lift_height+0.1-0.2,
    )
    left_pull_xyz = offset_xyz(
        box,
        args.grasp_x_offset+0.05,
        side + args.pull_distance+0.00,
        args.grasp_z_offset + args.lift_height-0.1,
    )
    right_up_xyz = offset_xyz(
        box,
        grasp_x_offset_r-0.05,
        -side,
        args.grasp_z_offset + args.lift_height-0.1,
    )
    right_pull_xyz = offset_xyz(
        box,
        grasp_x_offset_r,
        -side,
        args.grasp_z_offset + args.lift_height-0.1,
    )


    _s = args.box_width / 0.60
        # 依据箱子宽度选择已调好的胸前保持动作
    if abs(args.box_width - 0.40) < 0.01:
        # 40 cm 箱子：双臂同步抱住
        chest_up_left = [0.7, 0.45 , 1.20, 0, -90, -5]
        chest_up_right = [0.70, 0.05  , 1.22, 0, -90, 5]

        chest_left = [0.7, 0.45 , 1.20, 0, -90, -5]
        chest_right = [0.70, 0.05 , 1.22, 0, -90, 5]

    elif abs(args.box_width - 0.50) < 0.01:
        # 50 cm 小箱子
        chest_up_left = [0.65, 0.35 , 1.10, 0, -90, -5]
        chest_up_right = [0.60, -0.22 , 1.12, 0, -90, 5]
        chest_left = [0.67, 0.27 , 1.10, 0, -90, -5]
        chest_right = [0.60, -0.30 , 1.12, 0, -90, 5]

    elif abs(args.box_width - 0.70) < 0.01:
        # 70 cm 箱子
        chest_up_left = [0.7, 0.64 , 1.20,0, -90, -5]
        chest_up_right = [0.7, -0.15 , 1.20,0, -90, 5]
        chest_left = [0.7, 0.64 , 1.20, 0, -90, -5]
        chest_right = [0.7, -0.15 , 1.20, 0, -90, 5]

    else:
        raise ValueError(
            "当前只支持 0.40m、0.50m 和 0.70m 箱子，实际 box_width=%.3f"
            % args.box_width
        )
    return CarryPlan(
        left_out=[*left_out_xyz, *args.left_rpy],
        left_out2=[*left_out2_xyz, *args.left_rpy],
        left_grasp=left_grasp,
        left_up=[*left_up_xyz,0, -90, -5 ],
        left_pull=[*left_pull_xyz,0, -90, -5],
        right_out=[*right_out_xyz, *args.right_rpy],
        right_out2=[*right_out2_xyz, *args.right_rpy],
        right_grasp=right_grasp,
        right_up=[*right_up_xyz, 0, -90, 5],
        right_pull=[*right_pull_xyz, 0, -90, 5],
        chest_up_left=chest_up_left,
        chest_up_right=chest_up_right,
        chest_left=chest_left,
        chest_right=chest_right,
        table_left=[0.85, 0.37 * _s, getattr(args, 'table_left_z', 0.63), 0, -90, 0],
        table_right=[0.80, -0.40 * _s, getattr(args, 'table_right_z', 0.50), 0, -90, 0],
        expand_left=[0.95, 0.50 * _s, getattr(args, 'expand_left_z', 0.60), 0, -90, 0],
        expand_right=[0.90, -0.50 * _s, getattr(args, 'expand_right_z', 0.60), 0, -90, 0],
        reset_left=[0.85, 0.45, 0.50, 0, 0, 0],
        reset_right=[0.80, -0.45, 0.50, 0, 0, 0],
    )


def validate_plan(plan: CarryPlan, args) -> None:
    for field in fields(plan):
        pose = getattr(plan, field.name)
        if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
            raise ValueError(f"{field.name}: 非法轨迹点 {pose}")
        x, y, z = pose[:3]
        if not args.x_min <= x <= args.x_max:
            raise ValueError(f"{field.name}: x={x:.3f} 超出安全范围")
        if not args.y_min <= y <= args.y_max:
            raise ValueError(f"{field.name}: y={y:.3f} 超出安全范围")
        if not args.z_min <= z <= args.z_max:
            raise ValueError(f"{field.name}: z={z:.3f} 超出安全范围")

    grasp_distance = math.dist(plan.left_grasp[:3], plan.right_grasp[:3])
    # if grasp_distance < args.box_width:
    #     raise ValueError(
    #         f"双手抓取间距 {grasp_distance:.3f}m 小于箱宽 "
    #         f"{args.box_width:.3f}m"
    #     )


def print_plan(box: DetectedPose, plan: CarryPlan) -> None:
    data_id, basket_class = decode_basket_id(box.detection_id)
    LOG.info(
        "目标 ID=%d (data_id=%d class=%02d), %s -> %s",
        box.detection_id, data_id, basket_class,
        box.source_frame, box.target_frame,
    )
    LOG.info(
        "箱体位姿 XYZ=(%.4f, %.4f, %.4f), RPY=(%.2f°, %.2f°, %.2f°)",
        box.x, box.y, box.z,
        math.degrees(box.roll), math.degrees(box.pitch), math.degrees(box.yaw),
    )
    LOG.info("====== 最终轨迹 [x,y,z,roll,pitch,yaw] ======")
    for field in fields(plan):
        LOG.info("%-18s %s", field.name, [round(v, 4) for v in getattr(plan, field.name)])


def require_success(result, operation: str) -> None:
    if result is None or not result.success:
        raise RuntimeError(
            f"{operation}失败: {getattr(result, 'message', '无返回值')}"
        )


class MotionController:
    def __init__(self, hardware, interactive: bool):
        self.hardware = hardware
        self.interactive = interactive
        self.stopped = False

    def confirm(self, description: str, poses: Sequence[List[float]]) -> bool:
        if not self.interactive:
            return True
        print(f"\n即将执行：{description}")
        for pose in poses:
            print("  ", [round(v, 4) for v in pose])
        while True:
            answer = input("执行？[y]继续 / [n]跳过 / [q]退出: ").strip().lower()
            if answer in ("", "y", "yes"):
                return True
            if answer in ("n", "no"):
                return False
            if answer in ("q", "quit", "exit"):
                self.stopped = True
                return False

    def single(self, planner: int, pose: List[float], duration: float, description: str):
        if self.stopped or not self.confirm(description, [pose]):
            return
        require_success(
            self.hardware.send_timed_multi_commands(
                [{"planner_index": planner, "desire_time": duration, "cmd_vec": pose}],
                is_sync=True,
            ),
            description,
        )

    def both(
        self, left: List[float], right: List[float], duration: float, description: str
    ):
        if self.stopped or not self.confirm(description, [left, right]):
            return
        require_success(
            self.hardware.send_timed_multi_commands(
                [
                    {"planner_index": LEFT_PLANNER_LOCAL, "desire_time": duration, "cmd_vec": left},
                    {"planner_index": RIGHT_PLANNER_LOCAL, "desire_time": duration, "cmd_vec": right},
                ],
                is_sync=True,
            ),
            description,
        )


def parse_triplet(value: str) -> List[float]:
    try:
        result = [float(item.strip()) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc))
    if len(result) != 3:
        raise argparse.ArgumentTypeError("必须提供三个逗号分隔数值")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="允许驱动真机；默认仅调试规划")
    parser.add_argument("--interactive", "-i", action="store_true", help="每一步运动前人工确认")
    parser.add_argument("--mock", action="store_true", help="使用 --mock-pose，不读取视觉")
    parser.add_argument("--mock-pose", type=parse_triplet, default=[0.70, 0.0, 0.91],
                        help="base_link 中箱体中心 x,y,z")
    parser.add_argument("--mock-yaw", type=float, default=0.0, help="模拟箱体 yaw（度）")

    parser.add_argument("--tag-topic", default="/tag_detections")
    parser.add_argument("--tag-id", type=int, default=-1,
                        help="-1=最近目标；601=匹配data_id；60102=精确匹配")
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--samples", type=int, default=1,
                        help="定位所需消息帧数；默认1帧，不循环触发视觉模型")
    parser.add_argument("--detection-timeout", type=float, default=20.0)
    parser.add_argument("--tf-timeout", type=float, default=1.0)
    parser.add_argument("--max-position-spread", type=float, default=0.015)

    parser.add_argument("--box-width", type=float, default=0.60)
    parser.add_argument("--grasp-x-offset", type=float, default=0.0)
    parser.add_argument("--grasp-z-offset", type=float, default=-0.05,
                        help="相对视觉箱体原点的抓取高度偏移")
    parser.add_argument("--side-clearance", type=float, default=0.025)
    parser.add_argument("--approach-side-distance", type=float, default=0.15)
    parser.add_argument("--approach-height", type=float, default=0.12)
    parser.add_argument("--lift-height", type=float, default=0.15)
    parser.add_argument("--pull-distance", type=float, default=0.20)
    parser.add_argument("--left-rpy", type=parse_triplet, default=[0, -85, 0])
    parser.add_argument("--right-rpy", type=parse_triplet, default=[0, -95, 0])

    parser.add_argument("--x-min", type=float, default=0.25)
    parser.add_argument("--x-max", type=float, default=1.00)
    parser.add_argument("--y-min", type=float, default=-0.70)
    parser.add_argument("--y-max", type=float, default=0.70)
    parser.add_argument("--z-min", type=float, default=0.30)
    parser.add_argument("--z-max", type=float, default=1.30)
    return parser


def execute_plan(plan: CarryPlan, interactive: bool) -> int:
    from apps.test_kuavo_5w_adapter._scaffold import (
        adapter_setup,
        adapter_teardown,
        check_services_available,
    )

    hardware = LejuWheeledArmHardware(
        config={
            "skip_camera": True,
            "skip_end_effector": True,
            "skip_state_manager": True,
            "skip_force_publishers": True,
            "sdk_managers_whitelist": ["timed"],
            "angle_unit": "deg",
        }
    )
    initialized = False
    setup_complete = False
    try:
        require_success(hardware.initialize(), "硬件初始化")
        initialized = True
        check_services_available(["/mobile_manipulator_mpc_control"])
        adapter_setup(hardware, need_arm=True, mpc_mode=MPCControlMode.BASE_ARM)
        setup_complete = True
        time.sleep(2.0)
        require_success(hardware.arm_reset(), "机械臂复位")

        for planner in (LEFT_PLANNER_LOCAL, RIGHT_PLANNER_LOCAL):
            require_success(
                hardware.set_ruckig_params_timed(
                    planner_index=planner,
                    is_sync=True,
                    velocity_max=[0.35] * 6,
                    acceleration_max=[1.2] * 6,
                    jerk_max=[6.0] * 6,
                ),
                f"设置 planner {planner} 参数",
            )

        controller = MotionController(hardware, interactive)
        steps = [
            (controller.single, (LEFT_PLANNER_LOCAL, plan.left_out, 7.0, "左臂绕行")),
            (controller.single, (LEFT_PLANNER_LOCAL, plan.left_grasp, 7.0, "左臂抓取位")),
            (controller.single, (LEFT_PLANNER_LOCAL, plan.left_up, 7.0, "左臂提起")),
            (controller.single, (LEFT_PLANNER_LOCAL, plan.left_pull, 7.0, "左臂拉出")),
            (controller.single, (RIGHT_PLANNER_LOCAL, plan.right_out, 8.0, "右臂绕行")),
            (controller.single, (RIGHT_PLANNER_LOCAL, plan.right_out2, 8.0, "右臂绕行")),
            (controller.single, (RIGHT_PLANNER_LOCAL, plan.right_grasp, 7.0, "右臂抓取位")),
            (controller.both, (plan.chest_up_left, plan.chest_up_right, 5.0, "双臂抬至胸前")),
            (controller.both, (plan.chest_left, plan.chest_right, 8.0, "双臂搬到胸前")),
            (controller.both, (plan.table_left, plan.table_right, 5.0, "放到桌面")),
            (controller.both, (plan.expand_left, plan.expand_right, 4.0, "双臂外扩")),
            (controller.both, (plan.reset_left, plan.reset_right, 4.0, "双臂复位")),
        ]
        for action, parameters in steps:
            if controller.stopped or rospy.is_shutdown():
                break
            action(*parameters)

        return 130 if controller.stopped else 0
    except KeyboardInterrupt:
        LOG.warning("收到 Ctrl+C，停止后续动作")
        return 130
    except Exception:
        LOG.exception("搬运任务失败，停止后续动作")
        return 1
    finally:
        if setup_complete:
            try:
                adapter_teardown(hardware, need_arm=True, restore_mpc=True)
            except Exception:
                LOG.exception("控制环境清理失败")
        if initialized:
            try:
                hardware.shutdown()
            except Exception:
                LOG.exception("硬件关闭失败")


def main() -> int:
    args = build_parser().parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    rospy.init_node("basket_vision_boxcarry", anonymous=False)

    try:
        if args.mock:
            box = DetectedPose(
                x=args.mock_pose[0],
                y=args.mock_pose[1],
                z=args.mock_pose[2],
                roll=0.0,
                pitch=0.0,
                yaw=math.radians(args.mock_yaw),
                detection_id=60100,
                source_frame=args.target_frame,
                target_frame=args.target_frame,
            )
        else:
            LOG.info("等待 basket_vision: %s", args.tag_topic)
            box = StableBasketDetector(
                topic=args.tag_topic,
                requested_id=args.tag_id,
                samples=args.samples,
                timeout=args.detection_timeout,
                max_spread=args.max_position_spread,
                target_frame=args.target_frame,
                tf_timeout=args.tf_timeout,
            ).detect()

        plan = build_plan(box, args)
        validate_plan(plan, args)
        print_plan(box, plan)
    except Exception:
        LOG.exception("视觉定位或轨迹规划失败")
        return 1

    if not args.execute:
        LOG.warning(
            "调试模式：没有发送运动指令。确认坐标和轨迹后使用 --execute；"
            "首次真机建议同时使用 --interactive。"
        )
        return 0
    return execute_plan(plan, interactive=args.interactive)


if __name__ == "__main__":
    sys.exit(main())
