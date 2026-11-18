#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LeTools ROS 2 底盘最底层客户端（米 / 弧度）。

覆盖 8 条接口：
  /control/locomotion/cmd_vel_base                 Twist   本体系速度
  /control/locomotion/cmd_vel_odom                 Twist   世界系速度（⚠️ 里程计参考系，见下）
  /control/locomotion/cmd_pose_base                Pose2D  本体系增量
  /control/locomotion/cmd_pose_odom                Pose2D  世界系绝对（⚠️ 里程计参考系，见下）
  /control/locomotion/set_base_pose_timed_target   WheelSetTimedTarget
  /control/locomotion/state_base_reach_time        Float32
  /control/locomotion/get_velocity_limits          GetVelocityLimits
  /control/wheel_mpc/reset_vel_ruckig              SetBool

QoS 按接口文档：命令类 topic = best_effort / volatile / keep_last(5)。
单位：米、米/秒、弧度；度→弧度的转换在 adapter 层。
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import Twist, Vector3
from leju.msg import Pose2D
from leju.srv import GetVelocityLimits, SetBool, WheelSetTimedTarget
from std_msgs.msg import Float32

CMD_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
REACH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

VEL_BASE_TOPIC = "/control/locomotion/cmd_vel_base"
VEL_ODOM_TOPIC = "/control/locomotion/cmd_vel_odom"
POSE_BASE_TOPIC = "/control/locomotion/cmd_pose_base"
POSE_ODOM_TOPIC = "/control/locomotion/cmd_pose_odom"
BASE_REACH_TOPIC = "/control/locomotion/state_base_reach_time"
BASE_TIMED_SRV = "/control/locomotion/set_base_pose_timed_target"
VEL_LIMITS_SRV = "/control/locomotion/get_velocity_limits"
RESET_VEL_SRV = "/control/wheel_mpc/reset_vel_ruckig"

FRAME_WORLD = 0
FRAME_LOCAL = 1


def _numbers(values: Sequence[float], size: int, name: str) -> list:
    result = [float(v) for v in values]
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


@dataclass(frozen=True)
class VelocityLimits:
    linear_x_min: float
    linear_x_max: float
    linear_y_min: float
    linear_y_max: float
    angular_z_min: float
    angular_z_max: float


@dataclass(frozen=True)
class ServiceReply:
    success: bool
    message: str
    actual_time: Optional[float] = None
    limits: Optional[VelocityLimits] = None


class LocomotionClient:
    """底盘 ROS 2 端点。"""

    def __init__(self, node_name: str = "letools_locomotion_client", node=None) -> None:
        self._owns_node = node is None
        self._owns_context = False
        if node is None:
            self._owns_context = not rclpy.ok()
            if self._owns_context:
                rclpy.init()
            self.node = Node(node_name)
        else:
            self.node = node

        self._lock = threading.Lock()
        self._reach_frames = 0
        self._reach_value: Optional[float] = None

        self.vel_base_pub = self.node.create_publisher(Twist, VEL_BASE_TOPIC, CMD_QOS)
        self.vel_odom_pub = self.node.create_publisher(Twist, VEL_ODOM_TOPIC, CMD_QOS)
        self.pose_base_pub = self.node.create_publisher(Pose2D, POSE_BASE_TOPIC, CMD_QOS)
        self.pose_odom_pub = self.node.create_publisher(Pose2D, POSE_ODOM_TOPIC, CMD_QOS)
        self._reach_sub = self.node.create_subscription(
            Float32, BASE_REACH_TOPIC, self._on_reach, REACH_QOS
        )
        self._timed_cli = self.node.create_client(WheelSetTimedTarget, BASE_TIMED_SRV)
        self._limits_cli = self.node.create_client(GetVelocityLimits, VEL_LIMITS_SRV)
        self._reset_cli = self.node.create_client(SetBool, RESET_VEL_SRV)

    def _on_reach(self, msg: Float32) -> None:
        with self._lock:
            self._reach_frames += 1
            self._reach_value = float(msg.data)

    def _call(self, client, request, timeout: float):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            return None
        try:
            return future.result()
        except Exception:  # noqa: BLE001
            return None

    def spin_once(self, timeout: float = 0.05) -> None:
        rclpy.spin_once(self.node, timeout_sec=timeout)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    # ---- 速度 ----
    def publish_velocity_base(self, vx: float, vy: float, wz_rad: float) -> bool:
        """本体系速度（m/s, rad/s）；无订阅者返回 False。"""
        if self.node.count_subscribers(VEL_BASE_TOPIC) == 0:
            return False
        self.vel_base_pub.publish(
            Twist(linear=Vector3(x=float(vx), y=float(vy), z=0.0),
                  angular=Vector3(x=0.0, y=0.0, z=float(wz_rad)))
        )
        return True

    def publish_velocity_odom(self, vx: float, vy: float, wz_rad: float) -> bool:
        """世界系速度（m/s, rad/s）；无订阅者返回 False。

        ⚠️ 世界系 = 运控启动时建立的**里程计参考系**（启动瞬间位姿即原点 0,0,0），
        **不是** SLAM 地图/amcl 系，也不是底盘 ROS 1 的 /odom 系；
        运控重启后原点重置，同一组数值对应的物理位置随之改变。
        """
        if self.node.count_subscribers(VEL_ODOM_TOPIC) == 0:
            return False
        self.vel_odom_pub.publish(
            Twist(linear=Vector3(x=float(vx), y=float(vy), z=0.0),
                  angular=Vector3(x=0.0, y=0.0, z=float(wz_rad)))
        )
        return True

    # ---- 位姿 ----
    def publish_pose_base(self, x: float, y: float, theta_rad: float) -> bool:
        """本体系增量位姿（m, rad）。"""
        if self.node.count_subscribers(POSE_BASE_TOPIC) == 0:
            return False
        self.pose_base_pub.publish(
            Pose2D(x=float(x), y=float(y), theta=float(theta_rad))
        )
        return True

    def publish_pose_odom(self, x: float, y: float, theta_rad: float) -> bool:
        """世界系绝对位姿（m, rad）。

        ⚠️ 世界系 = 运控启动时建立的**里程计参考系**（启动瞬间位姿即原点 0,0,0），
        **不是** SLAM 地图/amcl 系，也不是底盘 ROS 1 的 /odom 系；
        运控重启后原点重置，同一组数值对应的物理位置随之改变。
        """
        if self.node.count_subscribers(POSE_ODOM_TOPIC) == 0:
            return False
        self.pose_odom_pub.publish(
            Pose2D(x=float(x), y=float(y), theta=float(theta_rad))
        )
        return True

    # ---- 定时位姿 ----
    def send_timed_target(self, frame: int, desire_time: float,
                          cmd_vec_rad: Sequence[float], timeout: float = 15.0) -> ServiceReply:
        if int(frame) not in (FRAME_WORLD, FRAME_LOCAL):
            raise ValueError("frame must be 0 (world) or 1 (local)")
        values = _numbers(cmd_vec_rad, 3, "base cmd_vec [x, y, yaw]")
        request = WheelSetTimedTarget.Request()
        request.frame = int(frame)
        request.desire_time = float(desire_time)
        request.cmd_vec = values
        response = self._call(self._timed_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "set_base_pose_timed_target unavailable or timed out")
        remote_ex = int(getattr(response, "remote_ex", 0))
        message = str(getattr(response, "message", ""))
        if remote_ex:
            message = f"remote_ex={remote_ex}: {message}"
        return ServiceReply(
            bool(response.is_success) and remote_ex == 0,
            message,
            actual_time=float(response.actual_time),
        )

    # ---- 限位 / 清零 ----
    def get_velocity_limits(self, timeout: float = 5.0) -> ServiceReply:
        response = self._call(self._limits_cli, GetVelocityLimits.Request(), timeout)
        if response is None:
            return ServiceReply(False, "get_velocity_limits unavailable or timed out")
        remote_ex = int(getattr(response, "remote_ex", 0))
        message = str(getattr(response, "message", ""))
        if remote_ex:
            message = f"remote_ex={remote_ex}: {message}"
        if not response.success or remote_ex:
            return ServiceReply(False, message)
        limits = VelocityLimits(
            float(response.linear_x_min), float(response.linear_x_max),
            float(response.linear_y_min), float(response.linear_y_max),
            float(response.angular_z_min), float(response.angular_z_max),
        )
        return ServiceReply(True, message or "velocity limits", limits=limits)

    def reset_vel_ruckig(self, timeout: float = 5.0) -> ServiceReply:
        response = self._call(
            self._reset_cli, SetBool.Request(data=True), timeout
        )
        if response is None:
            return ServiceReply(False, "reset_vel_ruckig unavailable or timed out")
        remote_ex = int(getattr(response, "remote_ex", 0))
        message = str(getattr(response, "message", ""))
        if remote_ex:
            message = f"remote_ex={remote_ex}: {message}"
        return ServiceReply(bool(response.success) and remote_ex == 0, message)

    # ---- reach_time ----
    @property
    def reach_frames(self) -> int:
        with self._lock:
            return self._reach_frames

    @property
    def reach_value(self) -> Optional[float]:
        with self._lock:
            return self._reach_value

    def reach_topic_available(self) -> bool:
        return self.node.count_publishers(BASE_REACH_TOPIC) > 0

    def wait_for_reach_time(self, previous_frames: int, timeout: float,
                            fallback_actual_time: Optional[float] = None) -> ServiceReply:
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            with self._lock:
                frames = self._reach_frames
                value = self._reach_value
            if frames > previous_frames and value is not None and value > 0:
                return ServiceReply(True, f"fresh planner duration {value:.2f}s",
                                    actual_time=value)
            self.spin_once(0.02)
        if fallback_actual_time is not None:
            return ServiceReply(True,
                                f"fallback actual_time {fallback_actual_time:.2f}s",
                                actual_time=float(fallback_actual_time))
        return ServiceReply(False, "no fresh base reach_time frame")

    def close(self) -> None:
        if not self._owns_node:
            return
        try:
            self.node.destroy_node()
        finally:
            if self._owns_context:
                rclpy.shutdown()

    def __enter__(self) -> "LocomotionClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
