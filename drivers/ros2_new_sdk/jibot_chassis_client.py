# LeTools/drivers/ros2_new_sdk/jibot_chassis_client.py
"""JiBot 底盘原生接口的 ROS 2 客户端。

只在这一层出现 ROS 2 端点名、消息类型与 QoS；单位保持底盘约定：
位置 m、角度 rad、速度 m/s 与 rad/s。

对应 ROS 1 实现：adapters/hardware/leju_wheeled/mixins/jibot/chassis_mixin.py

端点：
  - 服务 /move_base/base_move            leju/srv/BaseMove      本体系相对目标
  - 服务 /move_base/move_to_target       leju/srv/MoveToTarget  世界系绝对目标
  - 服务 /move_base/check_arrived        leju/srv/CheckArrived  按 task_id 查到达
  - 话题 /move_base/base_cmd_vel         Twist（发布）          底盘原始速度通道
  - 话题 /move_base/base_cmd_vel_status  BaseCmdVelStatus（订阅）速度命令有效性
  - 服务 /control/wheel_mpc/enable_vel_control_mode  SetBool    底盘控制权开关
  - 话题 /control/wheel_mpc/enable_vel_control       Bool（订阅）控制权当前标志
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool

from leju.msg import BaseCmdVelStatus
from leju.srv import BaseMove, CheckArrived, MoveToTarget, SetBool

BASE_MOVE_SRV = "/move_base/base_move"
MOVE_TO_TARGET_SRV = "/move_base/move_to_target"
CHECK_ARRIVED_SRV = "/move_base/check_arrived"
BASE_CMD_VEL_TOPIC = "/move_base/base_cmd_vel"
BASE_CMD_VEL_STATUS_TOPIC = "/move_base/base_cmd_vel_status"
VEL_CONTROL_SRV = "/control/wheel_mpc/enable_vel_control_mode"
VEL_CONTROL_TOPIC = "/control/wheel_mpc/enable_vel_control"
AMCL_POSE_TOPIC = "/move_base/amcl_pose"

# 文档推荐 QoS：reliability=best_effort, durability=volatile, keep_last 5
CMD_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass
class ServiceReply:
    """服务应答的统一形状（与 arm_client.ServiceReply 同风格）。"""

    success: bool
    message: str = ""
    task_id: str = ""
    arrived: bool = False
    status: int = 0
    value: Optional[int] = None


class JibotChassisClient:
    """jiBot 底盘原生端点；可复用外部 Node，也可自建。"""

    def __init__(self, node_name: str = "letools_jibot_chassis",
                 node: Optional[Node] = None) -> None:
        self._owns_node = node is None
        self.node = node if node is not None else Node(node_name)
        self._closed = False
        self._lock = threading.Lock()
        self._pubs: Dict[str, Any] = {}
        self._cmd_vel_status: Optional[BaseCmdVelStatus] = None
        self._cmd_vel_status_time: Optional[float] = None
        self._vel_control_flag: Optional[bool] = None
        self._vel_control_time: Optional[float] = None
        self._map_pose: Optional[PoseWithCovarianceStamped] = None
        self._map_pose_time: Optional[float] = None

        self._base_move_cli = self.node.create_client(BaseMove, BASE_MOVE_SRV)
        self._move_to_target_cli = self.node.create_client(MoveToTarget, MOVE_TO_TARGET_SRV)
        self._check_arrived_cli = self.node.create_client(CheckArrived, CHECK_ARRIVED_SRV)
        self._vel_control_cli = self.node.create_client(SetBool, VEL_CONTROL_SRV)

        self._status_sub = self.node.create_subscription(
            BaseCmdVelStatus, BASE_CMD_VEL_STATUS_TOPIC, self._on_status, CMD_QOS)
        self._vel_control_sub = self.node.create_subscription(
            Bool, VEL_CONTROL_TOPIC, self._on_vel_control, CMD_QOS)
        self._map_pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self._on_map_pose, CMD_QOS)

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------
    def _pub(self, topic: str):
        with self._lock:
            pub = self._pubs.get(topic)
            if pub is None:
                pub = self.node.create_publisher(Twist, topic, CMD_QOS)
                self._pubs[topic] = pub
            return pub

    def _on_status(self, msg: BaseCmdVelStatus) -> None:
        with self._lock:
            self._cmd_vel_status = msg
            self._cmd_vel_status_time = time.monotonic()

    def _on_vel_control(self, msg: Bool) -> None:
        with self._lock:
            self._vel_control_flag = bool(msg.data)
            self._vel_control_time = time.monotonic()

    def _on_map_pose(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._map_pose = msg
            self._map_pose_time = time.monotonic()

    def _call(self, cli, request, timeout: float):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        future = cli.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if time.monotonic() > deadline:
                return None
        return future.result()

    @staticmethod
    def _fill_move_request(req, x, y, theta, avoid_enable, avoid_distance,
                           linear_velocity, angular_velocity,
                           position_threshold, angle_threshold, allow_rotation):
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        req.avoid_enable = bool(avoid_enable)
        req.avoid_distance = float(avoid_distance)
        req.linear_velocity = float(linear_velocity)
        req.angular_velocity = float(angular_velocity)
        req.position_threshold = float(position_threshold)
        req.angle_threshold = float(angle_threshold)
        req.allow_rotation = bool(allow_rotation)
        return req

    # ------------------------------------------------------------------
    # 服务
    # ------------------------------------------------------------------
    def base_move(self, x, y, theta, avoid_enable, avoid_distance,
                  linear_velocity, angular_velocity, position_threshold,
                  angle_threshold, allow_rotation, timeout: float = 15.0) -> ServiceReply:
        request = self._fill_move_request(
            BaseMove.Request(), x, y, theta, avoid_enable, avoid_distance,
            linear_velocity, angular_velocity, position_threshold,
            angle_threshold, allow_rotation)
        response = self._call(self._base_move_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "base_move unavailable or timed out")
        return ServiceReply(bool(response.success), str(response.message),
                            task_id=str(response.task_id))

    def move_to_target(self, x, y, theta, avoid_enable, avoid_distance,
                       linear_velocity, angular_velocity, position_threshold,
                       angle_threshold, allow_rotation, timeout: float = 15.0) -> ServiceReply:
        request = self._fill_move_request(
            MoveToTarget.Request(), x, y, theta, avoid_enable, avoid_distance,
            linear_velocity, angular_velocity, position_threshold,
            angle_threshold, allow_rotation)
        response = self._call(self._move_to_target_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "move_to_target unavailable or timed out")
        return ServiceReply(bool(response.success), str(response.message),
                            task_id=str(response.task_id))

    def check_arrived(self, task_id: str, blocking: bool = False,
                      business_timeout: float = -1.0,
                      timeout: float = 20.0) -> ServiceReply:
        request = CheckArrived.Request()
        request.task_id = str(task_id)
        request.blocking = bool(blocking)
        request.timeout = float(business_timeout)
        response = self._call(self._check_arrived_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "check_arrived unavailable or timed out")
        return ServiceReply(bool(response.success), str(response.message),
                            arrived=bool(response.arrived),
                            status=int(response.status))

    def set_vel_control(self, enable: bool, timeout: float = 5.0) -> ServiceReply:
        request = SetBool.Request()
        request.data = bool(enable)
        response = self._call(self._vel_control_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "enable_vel_control_mode unavailable or timed out")
        return ServiceReply(bool(response.success), str(response.message))

    # ------------------------------------------------------------------
    # 话题
    # ------------------------------------------------------------------
    def publish_base_cmd_vel(self, vx: float, vy: float, wz: float,
                             topic: str = BASE_CMD_VEL_TOPIC) -> bool:
        pub = self._pub(topic)
        if pub.get_subscription_count() == 0:
            return False
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        pub.publish(msg)
        return True

    def read_vel_control_flag(self) -> Optional[bool]:
        with self._lock:
            return self._vel_control_flag

    def wait_vel_control_flag(self, timeout: float = 3.0) -> Optional[bool]:
        """等待一帧速度控制标志；已有缓存直接返回，超时返回 None。"""
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            with self._lock:
                flag = self._vel_control_flag
                received = self._vel_control_time
            if received is not None:
                return flag
            if time.monotonic() >= deadline:
                return None
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def read_map_pose(self, max_age_s: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """最新 /move_base/amcl_pose；超过 max_age_s 视为无数据。"""
        with self._lock:
            msg = self._map_pose
            received = self._map_pose_time
        if msg is None:
            return None
        age = None if received is None else max(0.0, time.monotonic() - received)
        if max_age_s is not None and (age is None or age > float(max_age_s)):
            return None
        pose = msg.pose.pose
        return {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "qx": float(pose.orientation.x),
            "qy": float(pose.orientation.y),
            "qz": float(pose.orientation.z),
            "qw": float(pose.orientation.w),
            "frame_id": str(msg.header.frame_id),
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "age_s": age,
        }

    def last_cmd_vel_status(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            msg = self._cmd_vel_status
            received = self._cmd_vel_status_time
        if msg is None:
            return None
        return {
            "cmd_vel_effective": bool(msg.cmd_vel_effective),
            "reason": int(msg.reason),
            "description": str(msg.description),
            "age_s": None if received is None else max(0.0, time.monotonic() - received),
        }

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def spin_once(self, timeout: float = 0.05) -> None:
        rclpy.spin_once(self.node, timeout_sec=timeout)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + float(seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        with self._lock:
            pubs = list(self._pubs.values())
            self._pubs.clear()
        for pub in pubs:
            try:
                self.node.destroy_publisher(pub)
            except Exception:
                pass
        for sub in (self._status_sub, self._vel_control_sub, self._map_pose_sub):
            try:
                self.node.destroy_subscription(sub)
            except Exception:
                pass
        if self._owns_node:
            try:
                self.node.destroy_node()
            except Exception:
                pass

    def __enter__(self) -> "JibotChassisClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
