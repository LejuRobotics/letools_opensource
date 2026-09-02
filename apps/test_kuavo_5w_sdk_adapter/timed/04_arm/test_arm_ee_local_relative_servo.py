#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂末端 local/base_link 相对补偿 - TimedCmd 版本

用途：
    读取当前末端 local/base_link 位置，在 x/y/z 上叠加 offset，姿态使用固定欧拉角，
    然后通过 send_arm_ee_local_timed 一次性发送目标位姿。

控制逻辑：
    1. 订阅 /humanoid_wheel/eePoses；
    2. 将 eePoses 原始 odom 位姿通过 TF: odom -> base_link 转成当前 local 位姿；
    3. 只取当前 x/y/z；
    4. 目标位置 = 当前 x/y/z + OFFSET_X/Y/Z；
    5. 目标姿态 = FIXED_YAW/FIXED_PITCH/FIXED_ROLL；
    6. 调用 send_arm_ee_local_timed(left_pose, right_pose, desire_time)。
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
from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup

init_logging()
logger = get_logger(__name__)

Pose = List[float]
_raw_ee_poses: Optional[List[float]] = None

# ========================= 调参区 =========================
# 1) 位置 offset：单位 m。只补偿 x/y/z。
OFFSET_X = 0.05         # local x，正值向前
OFFSET_Y = 0.0          # local y，正值向左
OFFSET_Z = -0.05        # local z，正值向上

# 2) 固定末端姿态：单位 deg。不会使用反馈读到的欧拉角。
FIXED_YAW = 0.0
FIXED_PITCH = -90.0
FIXED_ROLL = 0.0
# ==========================================================

# 安全/运行参数：一般不用改。
DESIRE_TIME = 0.6
WAIT_FEEDBACK = 2.0
FOCUS_EE = False
RELEASE_AFTER = True
RELEASE_ONLY = False
SAFE_EXECUTE = False  # 是否安全执行，日志验证是否符合预期
EE_POSES_SOURCE_FRAME = 'odom'
TARGET_LOCAL_FRAME = 'base_link'


def normalize_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _raw_callback(msg):
    global _raw_ee_poses
    _raw_ee_poses = list(msg.data)


def _subscribe_raw_ee_poses():
    import rospy
    from std_msgs.msg import Float64MultiArray

    rospy.Subscriber('/humanoid_wheel/eePoses', Float64MultiArray, _raw_callback)
    logger.info('已订阅 /humanoid_wheel/eePoses')


def _make_tf_listener():
    import tf

    listener = tf.TransformListener()
    time.sleep(0.5)
    return listener


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
        normalize_deg(math.degrees(local_yaw)),
        normalize_deg(math.degrees(local_pitch)),
        normalize_deg(math.degrees(local_roll)),
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


def log_pose(prefix: str, left_pose: Pose, right_pose: Pose):
    logger.info('%s left =[%.4f, %.4f, %.4f, %.2f, %.2f, %.2f]', prefix, *left_pose)
    logger.info('%s right=[%.4f, %.4f, %.4f, %.2f, %.2f, %.2f]', prefix, *right_pose)


def build_targets(left_current: Pose, right_current: Pose) -> Tuple[Pose, Pose]:
    """目标 = 当前位置 + offset + 固定欧拉角。"""
    fixed_orientation = [FIXED_YAW, FIXED_PITCH, FIXED_ROLL]
    left_target = [
        left_current[0] + OFFSET_X,
        left_current[1] + OFFSET_Y,
        left_current[2] + OFFSET_Z,
        *fixed_orientation,
    ]
    right_target = [
        right_current[0] + OFFSET_X,
        right_current[1] + OFFSET_Y,
        right_current[2] + OFFSET_Z,
        *fixed_orientation,
    ]
    return left_target, right_target


def release_external_hold(hardware) -> bool:
    result = hardware.set_arm_control_mode(0)
    if result.success:
        logger.info('已释放外部末端保持: set_arm_control_mode(0)')
        return True
    logger.warning(f'释放外部末端保持失败: {result.message}')
    return False


def send_relative_offset_once(hardware, listener, execute: bool = False) -> bool:
    logger.info('位置 offset: x=%.4f, y=%.4f, z=%.4f', OFFSET_X, OFFSET_Y, OFFSET_Z)
    logger.info('固定姿态: yaw=%.2f, pitch=%.2f, roll=%.2f', FIXED_YAW, FIXED_PITCH, FIXED_ROLL)

    left_current, right_current = read_current_local_ee_poses(listener)
    log_pose('当前 local/base_link 位姿', left_current, right_current)

    left_target, right_target = build_targets(left_current, right_current)
    log_pose('目标 local 位姿', left_target, right_target)

    if not execute:
        logger.warning('SAFE_EXECUTE=False: 只打印目标，不下发末端指令')
        return True

    result = hardware.send_arm_ee_local_timed(
        left_pose=left_target,
        right_pose=right_target,
        desire_time=DESIRE_TIME,
    )
    if not result.success:
        logger.warning(f'末端相对补偿指令发送失败: {result.message}')
        return False

    logger.info(f'末端相对补偿指令已发送: {result.message}')
    time.sleep(DESIRE_TIME + 0.2)
    return True


def main():
    hardware = HardwareFactory.create_hardware(
        config={
            'robot_type': 'leju_wheeled',
            'sdk_managers_whitelist': ['timed'],
            'skip_end_effector': True,
            'skip_camera': True,
            'skip_chassis': True,
            'skip_token_manager': True,
        }
    )

    try:
        hardware.initialize()
        _subscribe_raw_ee_poses()

        if RELEASE_ONLY:
            return 0 if release_external_hold(hardware) else 1

        factory_setup(
            hardware,
            need_arm_reset=False,
            need_torso_reset=False,
            focus_ee=FOCUS_EE,
            focus_z=False,
        )
        time.sleep(WAIT_FEEDBACK)
        listener = _make_tf_listener()

        if not SAFE_EXECUTE:
            logger.warning('当前为安全诊断模式，不会下发动作。确认目标位姿后，将 SAFE_EXECUTE 改为 True 再执行。')

        ok = send_relative_offset_once(hardware=hardware, listener=listener, execute=SAFE_EXECUTE)

        if ok and RELEASE_AFTER and SAFE_EXECUTE:
            release_external_hold(hardware)

        return 0 if ok else 1
    finally:
        hardware.shutdown()


if __name__ == '__main__':
    sys.exit(main())
