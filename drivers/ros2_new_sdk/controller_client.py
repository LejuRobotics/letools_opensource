#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LeTools ROS 2 控制器管理 / 启动阶段客户端。

覆盖：
  /control/controller/get_status          ControlStatus
  /control/controller/list_controllers    ControlGetList
  /control/controller/switch_controller   ControlSwitch    ⚠ 会切控制权
  /control/controller/load_controller     ControlLoad      ⚠ 会加载控制器
  /control/controller/event_switch        StringStamped    (TRANSIENT_LOCAL)
  /control/controller/event_ready         StringStamped    (TRANSIENT_LOCAL)
  /control/startup/state                  StringStamped    (TRANSIENT_LOCAL)
  /control/startup/confirm                Trigger          ⚠ 推进启动阶段

状态变更类方法只提供显式调用，本客户端不会自动调用。
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from leju.msg import StringStamped
from leju.srv import ControlGetList, ControlLoad, ControlStatus, ControlSwitch, Trigger

STATUS_SRV = "/control/controller/get_status"
LIST_SRV = "/control/controller/list_controllers"
SWITCH_SRV = "/control/controller/switch_controller"
LOAD_SRV = "/control/controller/load_controller"
CONFIRM_SRV = "/control/startup/confirm"
EVENT_SWITCH_TOPIC = "/control/controller/event_switch"
EVENT_READY_TOPIC = "/control/controller/event_ready"
STARTUP_STATE_TOPIC = "/control/startup/state"

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

STRING_TOPICS = {
    "switch": EVENT_SWITCH_TOPIC,
    "ready": EVENT_READY_TOPIC,
    "startup": STARTUP_STATE_TOPIC,
}


@dataclass(frozen=True)
class ServiceReply:
    success: bool
    message: str
    data: Optional[object] = None


class ControllerClient:
    def __init__(self, node_name: str = "letools_controller_client", node=None) -> None:
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
        self._state: Dict[str, Tuple[int, Optional[str]]] = {
            key: (0, None) for key in STRING_TOPICS
        }

        for key, topic in STRING_TOPICS.items():
            self.node.create_subscription(
                StringStamped, topic, self._on_string(key), LATCHED_QOS)

        self._status_cli = self.node.create_client(ControlStatus, STATUS_SRV)
        self._list_cli = self.node.create_client(ControlGetList, LIST_SRV)
        self._switch_cli = self.node.create_client(ControlSwitch, SWITCH_SRV)
        self._load_cli = self.node.create_client(ControlLoad, LOAD_SRV)
        self._confirm_cli = self.node.create_client(Trigger, CONFIRM_SRV)

    def _on_string(self, key: str):
        def callback(msg: StringStamped) -> None:
            with self._lock:
                frames, _ = self._state[key]
                self._state[key] = (frames + 1, str(msg.data))
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
    def _reply(response, success_field: str = "success") -> ServiceReply:
        if response is None:
            return ServiceReply(False, "service unavailable or timed out")
        remote_ex = int(getattr(response, "remote_ex", 0))
        message = str(getattr(response, "message", ""))
        if remote_ex:
            message = f"remote_ex={remote_ex}: {message}"
        return ServiceReply(bool(getattr(response, success_field)) and remote_ex == 0, message)

    def spin_once(self, timeout: float = 0.05) -> None:
        rclpy.spin_once(self.node, timeout_sec=timeout)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    # ---- 查询 ----
    def get_status(self, timeout: float = 5.0) -> ServiceReply:
        response = self._call(self._status_cli, ControlStatus.Request(), timeout)
        if response is None:
            return ServiceReply(False, "get_status unavailable or timed out")
        data = {
            "current": str(response.current_controller),
            "switching": bool(response.switching),
            "pending": str(getattr(response, "pending_controller", "")),
        }
        return ServiceReply(True, "controller status", data)

    def list_controllers(self, timeout: float = 5.0) -> ServiceReply:
        response = self._call(self._list_cli, ControlGetList.Request(), timeout)
        reply = self._reply(response)
        if not reply.success:
            return reply
        items = [{"name": str(c.name), "type": str(c.type)} for c in response.controllers]
        return ServiceReply(True, reply.message or "controller list", items)

    # ---- 状态变更（显式调用） ----
    def switch_controller(self, controller_name: str, timeout: float = 10.0) -> ServiceReply:
        request = ControlSwitch.Request()
        request.controller_name = str(controller_name)
        return self._reply(self._call(self._switch_cli, request, timeout))

    def load_controller(self, name: str, config_path: str = "",
                        timeout: float = 30.0) -> ServiceReply:
        request = ControlLoad.Request()
        request.name = str(name)
        request.config_path = str(config_path)
        return self._reply(self._call(self._load_cli, request, timeout))

    def confirm_startup(self, timeout: float = 5.0) -> ServiceReply:
        return self._reply(self._call(self._confirm_cli, Trigger.Request(), timeout))

    # ---- 事件 / 启动状态 ----
    def _state_value(self, key: str) -> Tuple[int, Optional[str]]:
        with self._lock:
            return self._state[key]

    def event_switch(self) -> Optional[str]:
        return self._state_value("switch")[1]

    def event_switch_frames(self) -> int:
        return self._state_value("switch")[0]

    def event_ready(self) -> Optional[str]:
        return self._state_value("ready")[1]

    def event_ready_frames(self) -> int:
        return self._state_value("ready")[0]

    def startup_state(self) -> Optional[str]:
        return self._state_value("startup")[1]

    def startup_state_frames(self) -> int:
        return self._state_value("startup")[0]

    def wait_for_state(self, kind: str, previous_frames: int,
                       timeout: float = 5.0) -> ServiceReply:
        if kind not in STRING_TOPICS:
            raise ValueError("kind must be switch/ready/startup")
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            frames, value = self._state_value(kind)
            if frames > previous_frames and value is not None:
                return ServiceReply(True, f"{kind} state", {"value": value, "frames": frames})
            self.spin_once(0.02)
        return ServiceReply(False, f"no fresh {kind} state frame")

    def close(self) -> None:
        if not self._owns_node:
            return
        try:
            self.node.destroy_node()
        finally:
            if self._owns_context:
                rclpy.shutdown()

    def __enter__(self) -> "ControllerClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
