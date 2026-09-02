#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取当前双臂末端 local/base_link 位姿

【前置条件】
    source /opt/ros/noetic/setup.bash
    source infrastructure/ros_packages/devel/setup.bash

【用法】
    直接修改下方【调参区】，然后运行：
        python3 test_get_arm_ee_current_pose.py

说明：
    - /humanoid_wheel/eePoses 原始数据已验证是 odom 系；
    - 本脚本将 eePoses 通过 TF: odom -> base_link 转换为 local/base_link 位姿；
    - 输出格式为 [x, y, z, yaw_deg, pitch_deg, roll_deg]；
    - 该脚本只读反馈，不下发任何动作；
    - 位置可作为 local 当前末端位置参考；姿态是 TF 转换后的实际姿态，是否可直接作为 TimedCmd 姿态下发需另行验证。
"""
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

project_root = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
from adapters.hardware.factory import HardwareFactory

init_logging()
logger = get_logger(__name__)

Pose = List[float]
_raw_ee_poses: Optional[List[float]] = None

# ========================= 调参区 =========================
# 初始化后等待 ROS 反馈缓存的时间。
WAIT_SECONDS = 2.0

# 连续采样次数。想边调整位姿边观察，可以改大，例如 20。
SAMPLE_COUNT = 1

# 每次采样间隔，单位秒。
SAMPLE_INTERVAL = 1.0

# eePoses 原始数据源坐标系。根据当前诊断结果应为 odom。
EE_POSES_SOURCE_FRAME = 'odom'
TARGET_LOCAL_FRAME = 'base_link'

# 是否同时打印 odom 原始位姿。
PRINT_RAW_ODOM = False
# ==========================================================


def _normalize_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _log_pose(name: str, pose: Pose):
    logger.info(
        "%s [x,y,z,yaw_deg,pitch_deg,roll_deg]=[%s]",
        name,
        ", ".join(f"{v:.4f}" for v in pose),
    )


def _raw_callback(msg):
    global _raw_ee_poses
    _raw_ee_poses = list(msg.data)


def _subscribe_raw_ee_poses():
    import rospy
    from std_msgs.msg import Float64MultiArray

    rospy.Subscriber('/humanoid_wheel/eePoses', Float64MultiArray, _raw_callback)
    logger.info('已订阅 /humanoid_wheel/eePoses')


def _raw_to_deg_pose(raw_pose: List[float]) -> Pose:
    """raw 格式: [x, y, z, yaw_rad, pitch_rad, roll_rad]。"""
    return [
        float(raw_pose[0]),
        float(raw_pose[1]),
        float(raw_pose[2]),
        _normalize_deg(math.degrees(float(raw_pose[3]))),
        _normalize_deg(math.degrees(float(raw_pose[4]))),
        _normalize_deg(math.degrees(float(raw_pose[5]))),
    ]


def _raw_pose_to_local(listener, raw_pose: List[float]) -> Pose:
    """将 eePoses 的 odom 位姿转换成 base_link/local 位姿。"""
    import rospy
    import tf
    from geometry_msgs.msg import PoseStamped

    yaw = float(raw_pose[3])
    pitch = float(raw_pose[4])
    roll = float(raw_pose[5])

    msg = PoseStamped()
    msg.header.frame_id = EE_POSES_SOURCE_FRAME
    msg.header.stamp = rospy.Time(0)
    msg.pose.position.x = float(raw_pose[0])
    msg.pose.position.y = float(raw_pose[1])
    msg.pose.position.z = float(raw_pose[2])

    qx, qy, qz, qw = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    transformed = listener.transformPose(TARGET_LOCAL_FRAME, msg)
    quat = transformed.pose.orientation
    local_roll, local_pitch, local_yaw = tf.transformations.euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w]
    )

    return [
        float(transformed.pose.position.x),
        float(transformed.pose.position.y),
        float(transformed.pose.position.z),
        _normalize_deg(math.degrees(local_yaw)),
        _normalize_deg(math.degrees(local_pitch)),
        _normalize_deg(math.degrees(local_roll)),
    ]


def read_current_local_ee_poses(listener) -> Tuple[Pose, Pose]:
    """读取当前双臂末端 local/base_link 位姿，返回顺序: left, right。"""
    import rospy

    if not _raw_ee_poses:
        raise RuntimeError('未读取到 /humanoid_wheel/eePoses 原始数据')
    if len(_raw_ee_poses) < 12:
        raise RuntimeError(f'/humanoid_wheel/eePoses 数据长度不足: {len(_raw_ee_poses)}，期望至少 12')

    listener.waitForTransform(
        TARGET_LOCAL_FRAME,
        EE_POSES_SOURCE_FRAME,
        rospy.Time(0),
        rospy.Duration(0.5),
    )

    left_raw = _raw_ee_poses[0:6]
    right_raw = _raw_ee_poses[6:12]
    return _raw_pose_to_local(listener, left_raw), _raw_pose_to_local(listener, right_raw)


def _make_tf_listener():
    import tf

    listener = tf.TransformListener()
    time.sleep(0.5)
    return listener


def main():
    hardware = HardwareFactory.create_hardware(
        config={
            'robot_type': 'leju_wheeled',
            'skip_sdk_managers': True,
            'skip_end_effector': True,
            'skip_camera': True,
            'skip_chassis': True,
            'skip_force_publishers': True,
            'skip_token_manager': True,
        }
    )

    try:
        hardware.initialize()
        _subscribe_raw_ee_poses()
        time.sleep(WAIT_SECONDS)
        listener = _make_tf_listener()

        for index in range(1, SAMPLE_COUNT + 1):
            logger.info('========== 当前末端 local 位姿采样 %d/%d ==========', index, SAMPLE_COUNT)

            if PRINT_RAW_ODOM and _raw_ee_poses and len(_raw_ee_poses) >= 12:
                _log_pose('左臂 raw odom 位姿', _raw_to_deg_pose(_raw_ee_poses[0:6]))
                _log_pose('右臂 raw odom 位姿', _raw_to_deg_pose(_raw_ee_poses[6:12]))

            left_local, right_local = read_current_local_ee_poses(listener)
            _log_pose('左臂 当前 local/base_link 位姿', left_local)
            _log_pose('右臂 当前 local/base_link 位姿', right_local)

            if index < SAMPLE_COUNT:
                time.sleep(SAMPLE_INTERVAL)

        return 0
    except Exception as e:
        logger.error('读取当前 local/base_link 末端位姿失败: %s', e, exc_info=True)
        return 1
    finally:
        hardware.shutdown()


if __name__ == '__main__':
    sys.exit(main())
