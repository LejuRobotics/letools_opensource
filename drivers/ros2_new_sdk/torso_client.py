#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LeTools ROS 2 躯干 / 下肢最底层客户端（弧度 / 米）。

覆盖：
  /control/torso/cmd_torso_pose                 Float64Array   [x, z, yaw, pitch]
  /control/torso/cmd_leg_joints                 JointState     4 关节
  /control/torso/state_torso_pose_reach_time    Float32
  /control/torso/state_leg_reach_time           Float32
  /control/torso/set_torso_pose_timed_target    WheelSetTimedTarget
  /control/torso/set_leg_joints_timed_target    WheelSetTimedTarget
  /control/torso/get_torso_init_pose            GetTorsoInitPose
  /control/torso/reset_torso                    SetBool
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from leju.msg import Float64Array
from leju.srv import GetTorsoInitPose, SetBool, WheelSetTimedTarget


CMD_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
REACH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

TORSO_CMD_TOPIC = "/control/torso/cmd_torso_pose"
LEG_CMD_TOPIC = "/control/torso/cmd_leg_joints"
TORSO_REACH_TOPIC = "/control/torso/state_torso_pose_reach_time"
LEG_REACH_TOPIC = "/control/torso/state_leg_reach_time"
TORSO_INIT_SRV = "/control/torso/get_torso_init_pose"
TORSO_RESET_SRV = "/control/torso/reset_torso"
TORSO_TIMED_SRV = "/control/torso/set_torso_pose_timed_target"
LEG_TIMED_SRV = "/control/torso/set_leg_joints_timed_target"

TORSO_DOF = 4
LEG_DOF = 4


def _numbers(values: Sequence[float], size: int, name: str) -> List[float]:
    result = [float(v) for v in values]
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


@dataclass(frozen=True)
class ServiceReply:
    success: bool
    message: str
    actual_time: Optional[float] = None
    vector: Optional[List[float]] = None


class TorsoClient:
    """躯干 / 下肢 ROS 2 端点。"""

    def __init__(self, node_name: str = "letools_torso_client", node=None) -> None:
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
        self._reach: Dict[str, Tuple[int, Optional[float]]] = {
            "torso": (0, None),
            "leg": (0, None),
        }

        self.torso_pub = self.node.create_publisher(Float64Array, TORSO_CMD_TOPIC, CMD_QOS)
        self.leg_pub = self.node.create_publisher(JointState, LEG_CMD_TOPIC, CMD_QOS)
        self._torso_reach_sub = self.node.create_subscription(
            Float32, TORSO_REACH_TOPIC, self._reach_cb("torso"), REACH_QOS
        )
        self._leg_reach_sub = self.node.create_subscription(
            Float32, LEG_REACH_TOPIC, self._reach_cb("leg"), REACH_QOS
        )
        self._init_cli = self.node.create_client(GetTorsoInitPose, TORSO_INIT_SRV)
        self._reset_cli = self.node.create_client(SetBool, TORSO_RESET_SRV)
        self._torso_timed_cli = self.node.create_client(WheelSetTimedTarget, TORSO_TIMED_SRV)
        self._leg_timed_cli = self.node.create_client(WheelSetTimedTarget, LEG_TIMED_SRV)

    def _reach_cb(self, kind: str):
        def callback(msg: Float32) -> None:
            with self._lock:
                frames, _ = self._reach[kind]
                self._reach[kind] = (frames + 1, float(msg.data))

        return callback

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

    @staticmethod
    def _reply(response, success_field: str = "result") -> ServiceReply:
        if response is None:
            return ServiceReply(False, "service unavailable or timed out")
        remote_ex = int(getattr(response, "remote_ex", 0))
        message = str(getattr(response, "message", ""))
        if remote_ex:
            message = f"remote_ex={remote_ex}: {message}"
        return ServiceReply(
            bool(getattr(response, success_field)) and remote_ex == 0, message
        )

    def spin_once(self, timeout: float = 0.05) -> None:
        rclpy.spin_once(self.node, timeout_sec=timeout)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            self.spin_once(min(0.05, deadline - time.monotonic()))

    # ---- 流式话题 ----
    def send_torso_pose(self, pose_rad: Sequence[float]) -> bool:
        values = _numbers(pose_rad, TORSO_DOF, "torso pose")
        if self.torso_pub.get_subscription_count() == 0:
            return False
        msg = Float64Array()
        msg.data = values
        self.torso_pub.publish(msg)
        return True

    def send_leg_joints(self, joints_rad: Sequence[float]) -> bool:
        values = _numbers(joints_rad, LEG_DOF, "leg joints")
        if self.leg_pub.get_subscription_count() == 0:
            return False
        msg = JointState()
        msg.position = values
        self.leg_pub.publish(msg)
        return True

    # ---- reach_time ----
    def reach_frames(self, kind: str) -> int:
        if kind not in self._reach:
            raise ValueError("kind must be torso or leg")
        with self._lock:
            return self._reach[kind][0]

    def reach_value(self, kind: str) -> Optional[float]:
        if kind not in self._reach:
            raise ValueError("kind must be torso or leg")
        with self._lock:
            value = self._reach[kind][1]
        return value if value is not None and math.isfinite(value) and value > 0 else None

    def reach_topic_available(self, kind: str) -> bool:
        topic = TORSO_REACH_TOPIC if kind == "torso" else LEG_REACH_TOPIC
        return self.node.count_publishers(topic) > 0

    def wait_for_reach_time(
        self, kind: str, previous_frames: int, timeout: float
    ) -> Optional[float]:
        if kind not in self._reach:
            raise ValueError("kind must be torso or leg")
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            with self._lock:
                frames, value = self._reach[kind]
            if frames > previous_frames and value is not None and math.isfinite(value) and value > 0:
                return value
        return None

    # ---- 定时目标 ----
    def _send_timed(self, client, values: Sequence[float], size: int,
                    desire_time: float, timeout: float) -> ServiceReply:
        values = _numbers(values, size, "timed target")
        if not math.isfinite(desire_time) or desire_time <= 0:
            raise ValueError("desire_time must be positive and finite")
        request = WheelSetTimedTarget.Request()
        request.frame = 0
        request.desire_time = float(desire_time)
        request.cmd_vec = values
        response = self._call(client, request, timeout)
        if response is None:
            return ServiceReply(False, "timed target service unavailable or timed out")
        reply = self._reply(response, success_field="is_success")
        return ServiceReply(reply.success, reply.message,
                            actual_time=float(response.actual_time))

    def send_timed_torso_target(self, pose_rad: Sequence[float],
                                desire_time: float = 2.0,
                                timeout: float = 5.0) -> ServiceReply:
        return self._send_timed(self._torso_timed_cli, pose_rad, TORSO_DOF,
                                desire_time, timeout)

    def send_timed_leg_target(self, joints_rad: Sequence[float],
                              desire_time: float = 2.0,
                              timeout: float = 5.0) -> ServiceReply:
        return self._send_timed(self._leg_timed_cli, joints_rad, LEG_DOF,
                                desire_time, timeout)

    # ---- 初始位姿 / 复位 ----
    def get_torso_init_pose(self, timeout: float = 5.0) -> ServiceReply:
        response = self._call(self._init_cli, GetTorsoInitPose.Request(), timeout)
        if response is None:
            return ServiceReply(False, "get_torso_init_pose unavailable or timed out")
        reply = self._reply(response, success_field="result")
        return ServiceReply(
            reply.success, reply.message,
            vector=[float(response.linear_x), float(response.linear_z),
                    float(response.yaw), float(response.pitch)],
        )

    def reset_torso(self, enable: bool = True, timeout: float = 5.0) -> ServiceReply:
        request = SetBool.Request()
        request.data = bool(enable)
        response = self._call(self._reset_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "reset_torso unavailable or timed out")
        return self._reply(response, success_field="success")

    def close(self) -> None:
        if not self._owns_node:
            return
        try:
            self.node.destroy_node()
        finally:
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()

    def __enter__(self) -> "TorsoClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
