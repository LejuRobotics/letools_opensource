#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LeTools ROS 2 多体定时 / 离线轨迹客户端（弧度）。

覆盖：
  /control/wheel_mpc/set_multi_body_timed_target      LbSetMultiBodyTimedTarget
  /control/wheel_mpc/set_multi_body_offline_traj      WheelMultiTimedOfflineTraj
  /control/wheel_mpc/enable_multi_body_offline_traj   SetBool
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from leju.msg import TimedPoint, TimedSingleCmd, WheelTimedOfflineTraj
from leju.srv import LbSetMultiBodyTimedTarget, SetBool, WheelMultiTimedOfflineTraj


TIMED_SRV = "/control/wheel_mpc/set_multi_body_timed_target"
OFFLINE_SRV = "/control/wheel_mpc/set_multi_body_offline_traj"
OFFLINE_ENABLE_SRV = "/control/wheel_mpc/enable_multi_body_offline_traj"
FOCUS_Z_TOPIC = "/control/wheel_mpc/constraint/focus_z"
FOCUS_EE_TOPIC = "/control/wheel_mpc/constraint/focus_ee"
FOCUS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)


@dataclass(frozen=True)
class ServiceReply:
    success: bool
    message: str
    actual_time: Optional[float] = None


class MultibodyClient:
    def __init__(self, node_name: str = "letools_multibody_client", node=None) -> None:
        self._owns_node = node is None
        self._owns_context = False
        if node is None:
            self._owns_context = not rclpy.ok()
            if self._owns_context:
                rclpy.init()
            self.node = Node(node_name)
        else:
            self.node = node

        self._timed_cli = self.node.create_client(LbSetMultiBodyTimedTarget, TIMED_SRV)
        self._offline_cli = self.node.create_client(WheelMultiTimedOfflineTraj, OFFLINE_SRV)
        self._enable_cli = self.node.create_client(SetBool, OFFLINE_ENABLE_SRV)
        self.focus_z_pub = self.node.create_publisher(Bool, FOCUS_Z_TOPIC, FOCUS_QOS)
        self.focus_ee_pub = self.node.create_publisher(Bool, FOCUS_EE_TOPIC, FOCUS_QOS)

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
    def _reply(response, success_field: str = "is_success") -> ServiceReply:
        if response is None:
            return ServiceReply(False, "service unavailable or timed out")
        remote_ex = int(getattr(response, "remote_ex", 0))
        message = str(getattr(response, "message", ""))
        if remote_ex:
            message = f"remote_ex={remote_ex}: {message}"
        return ServiceReply(bool(getattr(response, success_field)) and remote_ex == 0, message)

    def send_timed_target(self, commands: Sequence[dict], is_sync: bool = True,
                          timeout: float = 15.0) -> ServiceReply:
        if not commands:
            raise ValueError("commands must not be empty")
        request = LbSetMultiBodyTimedTarget.Request()
        request.is_sync = bool(is_sync)
        request.timed_cmd_vec = []
        for cmd in commands:
            item = TimedSingleCmd()
            item.planner_index = int(cmd["planner_index"])
            item.desire_time = float(cmd["desire_time"])
            values = [float(v) for v in cmd["cmd_vec"]]
            if not values or not all(math.isfinite(v) for v in values):
                raise ValueError("cmd_vec must be non-empty finite values")
            item.cmd_vec = values
            request.timed_cmd_vec.append(item)
        response = self._call(self._timed_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "set_multi_body_timed_target unavailable or timed out")
        reply = self._reply(response)
        return ServiceReply(reply.success, reply.message,
                            actual_time=float(response.actual_time))

    def set_offline_traj(self, trajectories: Sequence[dict], timeout: float = 15.0) -> ServiceReply:
        if not trajectories:
            raise ValueError("trajectories must not be empty")
        request = WheelMultiTimedOfflineTraj.Request()
        request.offline_traj = []
        for traj in trajectories:
            item = WheelTimedOfflineTraj()
            item.planner_index = int(traj["planner_index"])
            item.frame = int(traj.get("frame", 0))
            item.timed_traj = []
            for point in traj["points"]:
                tp = TimedPoint()
                tp.desire_time = float(point["desire_time"])
                values = [float(v) for v in point["cmd_vec"]]
                if not values or not all(math.isfinite(v) for v in values):
                    raise ValueError("cmd_vec must be non-empty finite values")
                tp.cmd_vec = values
                item.timed_traj.append(tp)
            request.offline_traj.append(item)
        response = self._call(self._offline_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "set_multi_body_offline_traj unavailable or timed out")
        return self._reply(response)

    def enable_offline_traj(self, enable: bool, timeout: float = 5.0) -> ServiceReply:
        request = SetBool.Request()
        request.data = bool(enable)
        response = self._call(self._enable_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "enable_multi_body_offline_traj unavailable or timed out")
        return self._reply(response, success_field="success")

    def _publish_bool(self, pub, topic: str, value: bool, repeats: int = 5) -> bool:
        if self.node.count_subscribers(topic) == 0:
            return False
        msg = Bool(data=bool(value))
        for _ in range(max(1, int(repeats))):
            pub.publish(msg)
            time.sleep(0.02)
        return True

    def set_focus_z(self, enable: bool, repeats: int = 5) -> bool:
        return self._publish_bool(self.focus_z_pub, FOCUS_Z_TOPIC, enable, repeats)

    def set_focus_ee(self, enable: bool, repeats: int = 5) -> bool:
        return self._publish_bool(self.focus_ee_pub, FOCUS_EE_TOPIC, enable, repeats)

    def close(self) -> None:
        if not self._owns_node:
            return
        try:
            self.node.destroy_node()
        finally:
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()

    def __enter__(self) -> "MultibodyClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
