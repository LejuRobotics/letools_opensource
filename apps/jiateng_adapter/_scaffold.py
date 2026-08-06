#!/usr/bin/env python3
"""嘉腾底盘适配器测试共用的 ROS 初始化与状态读取。"""

import math
import rospy

REQUIRED_SERVICES = (
    "/move_base/base_move",
    "/move_base/check_arrived",
    "/move_base/move_to_target",
)


def jiateng_setup(timeout=5.0, services=None):
    """初始化测试节点，并等待当前测试所需的嘉腾服务。"""
    if not rospy.core.is_initialized():
        rospy.init_node(
            "jiateng_adapter_test",
            anonymous=True,
            disable_signals=True,
        )

    selected_services = REQUIRED_SERVICES if services is None else services
    for service in selected_services:
        rospy.wait_for_service(service, timeout=timeout)


def read_vel_control_state(timeout=3.0):
    """读取外部控制通道状态。"""
    from std_msgs.msg import Bool

    message = rospy.wait_for_message(
        "/enable_vel_control_state",
        Bool,
        timeout=timeout,
    )
    return message.data


def assert_vel_control_state(expected, timeout=3.0):
    """确认当前控制权符合测试类型，不自动切换控制权。"""
    actual = read_vel_control_state(timeout=timeout)
    if actual != expected:
        required = "true（外部控制）" if expected else "false（嘉腾导航）"
        actual_text = "true（外部控制）" if actual else "false（嘉腾导航）"
        raise RuntimeError(
            "/enable_vel_control_state 状态不符合当前测试: "
            f"当前为 {actual_text}，需要 {required}。"
        )
    return actual


def assert_no_active_navigation_task(timeout=3.0):
    """确认嘉腾底盘处于 AUTO 且没有正在执行的导航任务。"""
    from leju_mobile_base_msgs.msg import RobotStatus

    status = rospy.wait_for_message(
        "/move_base/robot_status",
        RobotStatus,
        timeout=timeout,
    )
    if status.mode != status.MODE_AUTO:
        raise RuntimeError(f"底盘不在 AUTO 模式: mode={status.mode}")
    if status.task_status not in (status.TASK_NONE, status.TASK_COMPLETED):
        raise RuntimeError(
            "底盘存在活动任务: "
            f"task_id={status.task_id}, status={status.task_status}"
        )
    if status.error_code:
        raise RuntimeError(
            f"底盘错误: {status.error_code} {status.error_msg}"
        )
    return status


def read_current_map_pose(timeout=5.0):
    """读取嘉腾 AMCL 位姿，返回 map 坐标 ``(x, y, yaw)``。"""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    message = rospy.wait_for_message(
        "/move_base/amcl_pose",
        PoseWithCovarianceStamped,
        timeout=timeout,
    )
    if message.header.frame_id != "map":
        raise RuntimeError(
            "/move_base/amcl_pose 不是 map 坐标系: "
            f"frame_id={message.header.frame_id!r}"
        )

    pose = message.pose.pose
    q = pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return pose.position.x, pose.position.y, yaw
