#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LeTools ROS 2 头部最底层客户端。

文件位置：
    drivers/ros2_new_sdk/head_client.py

职责：
    - 只和 leju_sdk_ros2 / ROS 2 / DDS 通信
    - 管理 /control/head/cmd_head
    - 管理 /control/head/current_head_cmd
    - 管理 /control/head/get_head_limits

约定：
    - 本文件所有角度参数和返回值都是【弧度】
    - 度/弧度转换放在 adapters/hardware_ros2/mixins/head_mixin.py
    - current_head_cmd 是“目标回读”，不是实测关节角

运行前：
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
"""
from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from leju.msg import Float64Array, LowState
from leju.srv import GetHeadLimits


# ---------------------------------------------------------------------------
# 接口常量：只有这个文件知道 ROS 2 端点名字和类型
# ---------------------------------------------------------------------------
LOWSTATE_TOPIC = "/lowstate"
HEAD_SLOT = 29            # zhead_1_joint
HEAD_DOF = 2              # zhead_1 / zhead_2
CMD_TOPIC = "/control/head/cmd_head"
CURRENT_TOPIC = "/control/head/current_head_cmd"
LIMITS_SRV = "/control/head/get_head_limits"

# /lowstate 实测关节角（slots 29/30）
MEASURED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# /control/head/cmd_head
# 命令流：与官方 ROS2 案例一致，reliable + volatile + keep_last(1)
CMD_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# /control/head/current_head_cmd
# latched 状态：reliable + transient_local + keep_last(1)
CURRENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class HeadClient:
    """头部最底层 ROS 2 客户端。"""

    def __init__(self, node_name: str = "letools_head_client", node=None) -> None:
        self._owns_node = node is None
        self._owns_context = False
        if node is None:
            self._owns_context = not rclpy.ok()
            if self._owns_context:
                rclpy.init()
            self.node = Node(node_name)
        else:
            self.node = node

        self._latest: Optional[List[float]] = None
        self._frames = 0
        self._measured: Optional[List[float]] = None
        self._measured_frames = 0

        # /control/head/cmd_head：发布头部目标
        self.cmd_pub = self.node.create_publisher(
            Float64Array,
            CMD_TOPIC,
            CMD_QOS,
        )

        # /control/head/current_head_cmd：latched，读当前头部目标
        self._current_sub = self.node.create_subscription(
            Float64Array,
            CURRENT_TOPIC,
            self._on_current,
            CURRENT_QOS,
        )

        # /control/head/get_head_limits：查询 yaw/pitch 限位
        self.limits_cli = self.node.create_client(
            GetHeadLimits,
            LIMITS_SRV,
        )

        # 实测头部角度来自 /lowstate 的 slot 29/30（current_head_cmd 只是目标回读）
        self._measured_sub = self.node.create_subscription(
            LowState,
            LOWSTATE_TOPIC,
            self._on_lowstate,
            MEASURED_QOS,
        )

        self.node.get_logger().info(
            f"HeadClient ready: {CMD_TOPIC} / {CURRENT_TOPIC} / {LIMITS_SRV}"
        )

    @property
    def frames(self) -> int:
        """current_head_cmd 已收到的帧数。"""
        return self._frames

    def _on_current(self, msg: Float64Array) -> None:
        self._latest = [float(v) for v in msg.data]
        self._frames += 1

    def _on_lowstate(self, msg: LowState) -> None:
        if len(msg.motor_state) < HEAD_SLOT + HEAD_DOF:
            return
        values = [
            float(msg.motor_state[index].q)
            for index in range(HEAD_SLOT, HEAD_SLOT + HEAD_DOF)
        ]
        if all(math.isfinite(v) for v in values):
            self._measured = values
            self._measured_frames += 1

    @property
    def measured_frames(self) -> int:
        return self._measured_frames

    def read_measured(self) -> Optional[List[float]]:
        """返回头部实测角 [zhead_1, zhead_2]（弧度），不是目标回读。"""
        return list(self._measured) if self._measured is not None else None

    # ------------------------------------------------------------------
    # 基础自检
    # ------------------------------------------------------------------
    def wait_ready(self, timeout: float = 10.0) -> bool:
        """等 cmd_head 有订阅者，并且 current_head_cmd 收到至少一帧。"""
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.cmd_pub.get_subscription_count() > 0 and self._frames > 0:
                return True

        self.node.get_logger().warn(
            "wait_ready timeout: "
            f"subscribers={self.cmd_pub.get_subscription_count()}, "
            f"frames={self._frames}"
        )
        return False

    def wait_measured(self, timeout: float = 2.0) -> bool:
        """等 /lowstate 给出第一帧头部实测角（订阅发现可能晚于 header 服务）。"""
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if self._measured is not None:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return self._measured is not None

    def wait_for_new_frame(
        self,
        previous_frames: int,
        timeout: float = 2.0,
    ) -> Optional[List[float]]:
        """等 current_head_cmd 出现比 previous_frames 更新的一帧。"""
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._frames > previous_frames:
                return self.read_current()
        return None

    def spin_for(self, seconds: float) -> None:
        """阻塞式 spin 一小段时间，给 DDS 发现和回调留时间。"""
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ------------------------------------------------------------------
    # 头部控制
    # ------------------------------------------------------------------
    def send(self, yaw_rad: float, pitch_rad: float) -> bool:
        """发布头部目标，单位弧度。

        返回 True 表示已 publish 且有订阅者；
        返回 False 表示没有订阅者，指令没有真正送达。
        """
        if not math.isfinite(yaw_rad) or not math.isfinite(pitch_rad):
            raise ValueError("yaw_rad/pitch_rad must be finite")

        if self.cmd_pub.get_subscription_count() == 0:
            self.node.get_logger().warn(
                f"{CMD_TOPIC} has no subscriber; command not delivered"
            )
            return False

        msg = Float64Array()
        msg.data = [float(yaw_rad), float(pitch_rad)]
        self.cmd_pub.publish(msg)
        self.spin_for(0.05)
        return True

    def read_current(self) -> Optional[List[float]]:
        """返回最近一帧 current_head_cmd，单位弧度。

        注意：这是控制器收到的“目标”，不是 lowstate 里的实测角度。
        """
        return list(self._latest) if self._latest is not None else None

    def get_limits(
        self,
        timeout: float = 5.0,
    ) -> Optional[Tuple[List[float], List[float]]]:
        """查询头部限位，返回 (pos_min, pos_max)，单位弧度。

        该服务必须通过 leju_sdk_ros2_bridge；不可用时返回 None。
        """
        if not self.limits_cli.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().warn(f"{LIMITS_SRV} not available")
            return None

        try:
            future = self.limits_cli.call_async(GetHeadLimits.Request())
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=timeout,
            )
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"get_head_limits failed: {exc}")
            return None

        if response is None or not response.success:
            message = getattr(response, "message", "no response")
            self.node.get_logger().warn(
                f"get_head_limits rejected: {message}"
            )
            return None

        return (
            [float(v) for v in response.pos_min],
            [float(v) for v in response.pos_max],
        )

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def close(self) -> None:
        if not self._owns_node:
            return
        try:
            self.node.destroy_node()
        finally:
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()

    def __enter__(self) -> "HeadClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def main() -> int:
    """只做连接自检，不动机器人。"""
    with HeadClient() as client:
        ready = client.wait_ready(timeout=10.0)
        print(
            f"ready={ready} "
            f"subscribers={client.cmd_pub.get_subscription_count()} "
            f"frames={client.frames}"
        )

        limits = client.get_limits()
        print(f"limits_rad={limits}")
        print(f"current_rad={client.read_current()}")

        return 0 if ready else 2


if __name__ == "__main__":
    raise SystemExit(main())