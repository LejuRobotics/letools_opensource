#!/usr/bin/env python3
"""直接调用 ROS MultiTimedCmd 服务，同步控制躯干 4D 与双臂 14D。

运行时输入桌面高度和伺服高度，躯干目标计算为：
    [x, z, yaw, pitch] = [0.22, 桌面高度 + 伺服高度 + 0.17, 0.0, 0.25]
随后控制双臂局部系末端到：
    left  = [0.7,  0.4, 桌面高度 + 伺服高度, -pi/2, 0.0,  pi/18]
    right = [0.7, -0.4, 桌面高度 + 伺服高度,  pi/2, 0.0, -pi/18]
最后再次同步控制躯干与双臂关节，双臂目标与第一个动作相同；躯干在第一个
动作基础上将 z 增加 0.1 米，并将 pitch 改为 0.0。
其中位置单位为米，角度单位为弧度。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, List

import rospy
from kuavo_msgs.msg import timedSingleCmd
from kuavo_msgs.srv import (
    changeArmCtrlMode,
    changeArmCtrlModeRequest,
    changeTorsoCtrlMode,
    changeTorsoCtrlModeRequest,
    lbMultiTimedPosCmd,
    lbMultiTimedPosCmdRequest,
)
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


MULTI_CMD_SERVICE = "/mobile_manipulator_timed_multi_cmd"
MPC_MODE_SERVICE = "/mobile_manipulator_mpc_control"
ARM_MODE_SERVICE = "/wheel_arm_change_arm_ctrl_mode"
TORSO_RESET_SERVICE = "/mobile_manipulator_reset_torso"
FOCUS_EE_TOPIC = "/mobile_manipulator_focus_ee"
FOCUS_Z_TOPIC = "/mobile_manipulator_focus_z"
DESIRE_TIME_S = 3.0
TABLE_HEIGHT_RANGE_M = (0.65, 0.90)
SERVO_HEIGHT_RANGE_M = (0.15, 0.30)

# 躯干格式：[x, z, yaw, pitch]；x/z 为米，yaw/pitch 为弧度。
TORSO_X_M = 0.22
TORSO_Z_OFFSET_M = 0.17
TORSO_YAW_RAD = 0.0
TORSO_PITCH_RAD = 0.25

# 双臂格式：[左臂 7D, 右臂 7D]；单位为弧度。
ARM_JOINTS_RAD = [
    0.9948376736,
    0.7243116396,
    -0.1972222055,
    -2.5045474766,
    0.3717551307,
    0.6684611035,
    0.0401425728,
    0.9948376736,
    -0.7243116396,
    0.1972222055,
    -2.5045474766,
    -0.3717551307,
    -0.6684611035,
    0.0401425728,
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=10.0,
        help="等待 ROS 服务的超时时间，默认 10 秒",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="仅打印发给 ROS 服务的弧度指令，不控制机器人",
    )
    args = parser.parse_args(rospy.myargv()[1:])
    if args.service_timeout <= 0.0:
        parser.error("--service-timeout 必须大于 0")
    return args


def prompt_height(prompt: str, minimum_m: float, maximum_m: float) -> float:
    """循环读取指定闭区间内的有限米制高度。"""
    while True:
        raw = input(prompt).strip()
        try:
            value = float(raw)
        except ValueError:
            print("输入无效，请输入数字，例如 0.75")
            continue
        if not math.isfinite(value):
            print("输入无效，高度必须是有限数值")
            continue
        if not minimum_m <= value <= maximum_m:
            print(
                "输入超出范围，请输入 "
                f"[{minimum_m:.2f}, {maximum_m:.2f}] 米之间的数值"
            )
            continue
        return value


def calculate_torso_pose(
    table_height_m: float,
    servo_height_m: float,
) -> List[float]:
    """根据桌面高度和伺服高度生成躯干 4D 绝对目标。"""
    torso_z_m = table_height_m + servo_height_m + TORSO_Z_OFFSET_M
    return [TORSO_X_M, torso_z_m, TORSO_YAW_RAD, TORSO_PITCH_RAD]


def calculate_arm_ee_local_poses(
    table_height_m: float,
    servo_height_m: float,
) -> tuple[List[float], List[float]]:
    """生成左右臂局部坐标系 6D 末端目标，姿态角直接使用弧度。"""
    ee_z_m = table_height_m + servo_height_m
    left_pose = [0.7, 0.4, ee_z_m, -math.pi / 2.0, 0.0, math.pi / 18.0]
    right_pose = [0.7, -0.4, ee_z_m, math.pi / 2.0, 0.0, -math.pi / 18.0]
    return left_pose, right_pose


def build_torso_arm_joint_commands(
    desire_time: float,
    torso_pose: List[float],
) -> List[Dict[str, object]]:
    """生成第一个动作：躯干和双臂关节同步命令。"""
    if len(torso_pose) != 4:
        raise ValueError("torso_pose 必须包含 4 个元素")
    if len(ARM_JOINTS_RAD) != 14:
        raise ValueError("ARM_JOINTS_RAD 必须包含 14 个元素")

    return [
        {
            "planner_index": 2,
            "desire_time": float(desire_time),
            "cmd_vec": [float(value) for value in torso_pose],
        },
        {
            "planner_index": 8,
            "desire_time": float(desire_time),
            "cmd_vec": list(ARM_JOINTS_RAD[:7]),
        },
        {
            "planner_index": 9,
            "desire_time": float(desire_time),
            "cmd_vec": list(ARM_JOINTS_RAD[7:]),
        },
    ]


def build_arm_ee_local_commands(
    desire_time: float,
    left_pose: List[float],
    right_pose: List[float],
) -> List[Dict[str, object]]:
    """生成第二个动作：左右臂局部系末端同步命令。"""
    if len(left_pose) != 6 or len(right_pose) != 6:
        raise ValueError("左右臂末端位姿必须各包含 6 个元素")
    return [
        {
            "planner_index": 6,
            "desire_time": float(desire_time),
            "cmd_vec": [float(value) for value in left_pose],
        },
        {
            "planner_index": 7,
            "desire_time": float(desire_time),
            "cmd_vec": [float(value) for value in right_pose],
        },
    ]


def wait_for_service(service_name: str, timeout: float) -> None:
    rospy.loginfo("等待服务 %s", service_name)
    rospy.wait_for_service(service_name, timeout=timeout)


def set_focus_false() -> tuple[rospy.Publisher, rospy.Publisher]:
    """将末端和 Z 轴控制焦点都设为 false，并保持锁存发布器存活。"""
    focus_ee_publisher = rospy.Publisher(
        FOCUS_EE_TOPIC,
        Bool,
        queue_size=1,
        latch=True,
    )
    focus_z_publisher = rospy.Publisher(
        FOCUS_Z_TOPIC,
        Bool,
        queue_size=1,
        latch=True,
    )
    # 给 Publisher 留出与 ROS master 建立连接的时间，再锁存目标值。
    rospy.sleep(0.2)
    focus_ee_publisher.publish(Bool(data=False))
    focus_z_publisher.publish(Bool(data=False))
    rospy.sleep(0.2)
    rospy.loginfo("控制焦点已设置: focus_ee=false, focus_z=false")
    return focus_ee_publisher, focus_z_publisher


def set_mpc_mode(mode: int, timeout: float) -> None:
    """设置 MPC 模式：1=ArmOnly，0=NoControl。"""
    wait_for_service(MPC_MODE_SERVICE, timeout)
    proxy = rospy.ServiceProxy(MPC_MODE_SERVICE, changeTorsoCtrlMode)
    request = changeTorsoCtrlModeRequest()
    request.control_mode = int(mode)
    response = proxy(request)
    if not response.result:
        raise RuntimeError(
            getattr(response, "message", "") or f"MPC 模式 {mode} 设置失败"
        )
    rospy.loginfo("MPC 模式已设置为 %d", mode)


def set_arm_control_mode(mode: int, timeout: float) -> None:
    """设置手臂模式：2=外部控制，0=当前位置保持。"""
    wait_for_service(ARM_MODE_SERVICE, timeout)
    proxy = rospy.ServiceProxy(ARM_MODE_SERVICE, changeArmCtrlMode)
    request = changeArmCtrlModeRequest()
    request.control_mode = int(mode)
    response = proxy(request)
    if not response.result:
        raise RuntimeError(
            getattr(response, "message", "") or f"手臂模式 {mode} 设置失败"
        )
    rospy.loginfo("手臂控制模式已设置为 %d", mode)


def reset_arm_and_torso(timeout: float) -> None:
    """复位手臂和躯干；手臂模式 1 本身会触发回初始姿态运动。"""
    rospy.logwarn("开始动作 4：复位手臂和躯干")
    set_arm_control_mode(1, timeout)
    rospy.loginfo("手臂复位指令已下发，等待复位完成")
    rospy.sleep(2.0)

    wait_for_service(TORSO_RESET_SERVICE, timeout)
    proxy = rospy.ServiceProxy(TORSO_RESET_SERVICE, SetBool)
    response = proxy(True)
    if not response.success:
        raise RuntimeError(response.message or "躯干复位失败")
    rospy.loginfo("躯干复位指令已下发，等待复位完成")
    rospy.sleep(2.5)
    rospy.loginfo("手臂和躯干复位完成")


def wait_for_action_enter(action_name: str) -> None:
    """仅在用户明确按下 Enter 后继续；输入 q 可取消剩余动作。"""
    try:
        response = input(
            f"\n[{action_name}] 确认环境安全后按 Enter 执行，输入 q 取消："
        ).strip().lower()
    except (EOFError, KeyboardInterrupt) as exc:
        raise RuntimeError(f"{action_name} 已取消") from exc
    if response == "q":
        raise RuntimeError(f"用户取消 {action_name}")
    if response:
        raise RuntimeError(f"{action_name} 未执行：必须直接按 Enter 确认")


def send_multi_timed_cmd(
    commands: List[Dict[str, object]],
    timeout: float,
) -> float:
    """直接调用 /mobile_manipulator_timed_multi_cmd。"""
    wait_for_service(MULTI_CMD_SERVICE, timeout)
    proxy = rospy.ServiceProxy(MULTI_CMD_SERVICE, lbMultiTimedPosCmd)
    request = lbMultiTimedPosCmdRequest()
    request.isSync = True

    for command in commands:
        timed_command = timedSingleCmd()
        timed_command.planner_index = int(command["planner_index"])
        timed_command.desireTime = float(command["desire_time"])
        timed_command.cmdVec = [float(value) for value in command["cmd_vec"]]
        request.timedCmdVec.append(timed_command)

    response = proxy(request)
    if not response.isSuccess:
        raise RuntimeError(response.message or "MultiTimedCmd 执行失败")
    return float(response.actualTime)


def release_control(timeout: float) -> None:
    """结束时保持当前手臂位置，并释放 MPC 控制。"""
    try:
        set_arm_control_mode(0, timeout)
    except Exception as exc:
        rospy.logwarn("手臂切换到当前位置保持失败: %s", exc)
    try:
        set_mpc_mode(0, timeout)
    except Exception as exc:
        rospy.logwarn("释放 MPC 控制失败: %s", exc)


def main() -> int:
    args = parse_args()
    try:
        table_height_m = prompt_height(
            "请输入桌面高度（0.65~0.90 米）：",
            *TABLE_HEIGHT_RANGE_M,
        )
        servo_height_m = prompt_height(
            "请输入伺服高度（0.15~0.30 米）：",
            *SERVO_HEIGHT_RANGE_M,
        )
    except (EOFError, KeyboardInterrupt):
        print("\n输入已取消")
        return 1

    torso_pose = calculate_torso_pose(table_height_m, servo_height_m)
    left_ee_pose, right_ee_pose = calculate_arm_ee_local_poses(
        table_height_m,
        servo_height_m,
    )
    first_commands = build_torso_arm_joint_commands(
        DESIRE_TIME_S,
        torso_pose,
    )
    second_commands = build_arm_ee_local_commands(
        DESIRE_TIME_S,
        left_ee_pose,
        right_ee_pose,
    )
    final_torso_pose = list(torso_pose)
    final_torso_pose[1] += 0.1
    final_torso_pose[3] = 0.0
    third_commands = build_torso_arm_joint_commands(
        DESIRE_TIME_S,
        final_torso_pose,
    )
    print(
        "躯干目标 [x,z,yaw,pitch] = "
        f"[{torso_pose[0]:.3f}, {torso_pose[1]:.3f}, "
        f"{torso_pose[2]:.3f}, {torso_pose[3]:.3f}] "
        "（m, m, rad, rad）"
    )
    print(f"左臂局部系末端目标 = {left_ee_pose}（m, m, m, rad, rad, rad）")
    print(f"右臂局部系末端目标 = {right_ee_pose}（m, m, m, rad, rad, rad）")
    if args.dry_run:
        print("动作 1 MultiTimedCmd：躯干 + 双臂关节")
        for command in first_commands:
            print(command)
        print("动作 2 MultiTimedCmd：双臂局部系末端")
        for command in second_commands:
            print(command)
        print("动作 3 MultiTimedCmd：躯干 z+0.1m、pitch=0 + 双臂关节")
        for command in third_commands:
            print(command)
        print("动作 4：手臂模式 1 复位 + /mobile_manipulator_reset_torso")
        return 0

    rospy.init_node("torso_arm_joint_multi_timed_cmd", anonymous=False)
    control_prepared = False
    focus_publishers = None
    try:
        rospy.logwarn("即将执行实机运动：躯干 + 双臂关节同步控制")
        rospy.loginfo("躯干目标 [x,z,yaw,pitch] = %s", torso_pose)
        rospy.loginfo("双臂关节目标（弧度） = %s", ARM_JOINTS_RAD)

        wait_for_action_enter("动作 1：躯干 + 双臂关节")
        focus_publishers = set_focus_false()
        set_mpc_mode(1, args.service_timeout)
        control_prepared = True
        set_arm_control_mode(2, args.service_timeout)

        actual_time = send_multi_timed_cmd(first_commands, args.service_timeout)
        rospy.loginfo("动作 1 成功，规划执行时间 %.3fs", actual_time)
        time.sleep(max(actual_time, DESIRE_TIME_S) + 0.5)

        rospy.logwarn("开始动作 2：双臂局部系末端同步控制")
        rospy.loginfo("左臂局部系末端目标 = %s", left_ee_pose)
        rospy.loginfo("右臂局部系末端目标 = %s", right_ee_pose)
        wait_for_action_enter("动作 2：双臂局部系末端")
        actual_time = send_multi_timed_cmd(second_commands, args.service_timeout)
        rospy.loginfo("动作 2 成功，规划执行时间 %.3fs", actual_time)
        time.sleep(max(actual_time, DESIRE_TIME_S) + 0.5)

        rospy.logwarn("开始动作 3：恢复双臂关节，躯干 z+0.1m、pitch=0")
        rospy.loginfo("动作 3 躯干目标 [x,z,yaw,pitch] = %s", final_torso_pose)
        wait_for_action_enter("动作 3：躯干 z+0.1m、pitch=0 + 双臂关节")
        actual_time = send_multi_timed_cmd(third_commands, args.service_timeout)
        rospy.loginfo("动作 3 成功，规划执行时间 %.3fs", actual_time)
        time.sleep(max(actual_time, DESIRE_TIME_S) + 0.5)
        rospy.loginfo("三个 MultiTimedCmd 动作均已完成")

        wait_for_action_enter("动作 4：手臂和躯干复位")
        reset_arm_and_torso(args.service_timeout)
        return 0
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as exc:
        rospy.logerr("控制失败: %s", exc)
        return 1
    finally:
        if control_prepared:
            release_control(args.service_timeout)


if __name__ == "__main__":
    sys.exit(main())
