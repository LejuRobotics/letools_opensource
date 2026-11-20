#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LeTools ROS 2 上肢最底层客户端。

本文件只负责 leju/ROS 2/DDS 通信，所有角度均为弧度。
Topic 直连；service 依赖 leju_sdk_ros2_bridge。

覆盖的接口（driver 层，唯一持有端点/字段/QoS 的地方）：
  Topic   /control/arm/cmd_joints
          /control/arm/cmd_arm_joints_traj
          /control/arm/cmd_ee_poses
          /control/arm/cmd_ee_poses_traj
          /control/arm/state_{left,right}_arm_{pose,joints}_reach_time
          /control/wheel_mpc/state_arm_control_mode
          /planning/kinematics/ik_result_arm_joints（只读求解结果）
          /lowstate（槽位 0..3 + 15..28 -> 18 维模型角）
  Service /control/controller/get_status
          /control/{arm,wheel_mpc}/{get,set}_control_mode
          /control/wheel_mpc/{get,set}_arm_quick_mode
          /control/arm/set_{left,right}_{joints,ee_pose}_timed_target
          /control/arm/get_ee_pose_reach_error
          /control/wheel_mpc/check_ee_accessibility
          /control/wheel_mpc/enable_single_timed_target_async_exec
          /control/arm/execute_arm_action
          /control/arm/interrupt_arm_action（旧名 interrupt_arm_traj 保留兼容）
          /planning/kinematics/solve_arm_fk        （内部拼 18 维）
          /planning/kinematics/solve_ee_poses_ik   （只解臂，q_torso 原样返回）
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
from std_msgs.msg import Float32

from leju.msg import (
    CmdArmJoints,
    CmdArmJointsTraj,
    CmdEePoses,
    CmdEePosesIK,
    CmdEePosesTraj,
    Float64Array,
    LowState,
)
from leju.srv import (
    AccessIkSolve,
    ChangeArmCtrlMode,
    ChangeLbQuickMode,
    ChangeTorsoCtrlMode,
    ControlStatus,
    EePoseReachError,
    ExecuteArmAction,
    SetArmQuickMode,
    SetBool,
    SolveEePoses,
    SolveFk,
    SolveFkReferFrame,
    Trigger,
    WheelSetTimedTarget,
)


LATEST_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
JOINTS_TOPIC = "/control/arm/cmd_joints"
JOINT_TRAJ_TOPIC = "/control/arm/cmd_arm_joints_traj"
EE_TOPIC = "/control/arm/cmd_ee_poses"
EE_TRAJ_TOPIC = "/control/arm/cmd_ee_poses_traj"
IK_INPUT_TOPIC = "/control/kinematics/cmd_ee_poses_ik"
IK_ADVANCE_TOPIC = "/control/kinematics/cmd_ee_poses_advance_ik"
IK_RESULT_TOPIC = "/planning/kinematics/ik_result_arm_joints"
LOWSTATE_TOPIC = "/lowstate"
BEST_EFFORT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
# IK 输入话题的下位机订阅端是 RELIABLE/VOLATILE
IK_PUB_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# 保留 S7 结果观测的 RELIABLE/VOLATILE、KEEP_LAST(10)；不发布控制。
IK_RESULT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
LEFT_ARM_SLOT = 15
RIGHT_ARM_SLOT = 22
ARM_DOF = 7

# 模型角 18 维 = [躯干 4, 左臂 7, 右臂 7]；躯干槽位与 URDF 关节序一致
TORSO_SLOTS = (0, 1, 2, 3)
IK_DOF = len(TORSO_SLOTS) + 2 * ARM_DOF


@dataclass(frozen=True)
class ControllerState:
    current: str
    switching: bool
    pending: str


@dataclass(frozen=True)
class ServiceReply:
    success: bool
    message: str
    value: Optional[int] = None
    actual_time: Optional[float] = None
    vector: Optional[List[float]] = None      # err_vector / q_best / q_arm
    linear_error: Optional[float] = None      # accessibility 的 best_linear_error
    torso: Optional[List[float]] = None       # IK 返回的 q_torso
    poses: Optional[dict] = None              # FK 返回的双臂末端位姿


def _numbers(values: Sequence[float], size: int, name: str) -> List[float]:
    result = [float(v) for v in values]
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def _model_positions_from(msg: LowState) -> Optional[List[float]]:
    """从一帧 LowState 取 [躯干 4, 左臂 7, 右臂 7] 共 18 维模型角（同一帧）。"""
    if len(msg.motor_state) < RIGHT_ARM_SLOT + ARM_DOF:
        return None
    q = [float(state.q) for state in msg.motor_state]
    values = [q[i] for i in TORSO_SLOTS] + q[LEFT_ARM_SLOT:RIGHT_ARM_SLOT + ARM_DOF]
    if not all(math.isfinite(v) for v in values):
        return None
    return values


def _zyx_to_quat(yaw: float, pitch: float, roll: float) -> Tuple[float, float, float, float]:
    """ZYX(yaw,pitch,roll) 弧度 -> 四元数 (x,y,z,w)。"""
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _fill_hand_pose(target, zyx6: Sequence[float]) -> None:
    qx, qy, qz, qw = _zyx_to_quat(zyx6[3], zyx6[4], zyx6[5])
    target.pos_xyz = [float(v) for v in zyx6[:3]]
    target.quat_xyzw = [qx, qy, qz, qw]


def _hand_poses_to_dict(hand_poses) -> dict:
    return {
        "left": {
            "pos": [float(v) for v in hand_poses.left_pose.pos_xyz],
            "quat": [float(v) for v in hand_poses.left_pose.quat_xyzw],
        },
        "right": {
            "pos": [float(v) for v in hand_poses.right_pose.pos_xyz],
            "quat": [float(v) for v in hand_poses.right_pose.quat_xyzw],
        },
    }


class ArmClient:
    """上肢 ROS 2 端点与实测关节状态。"""

    def __init__(self, node_name: str = "letools_arm_client", node=None) -> None:
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
        self._arm_positions: Optional[List[float]] = None
        self._lowstate_tick: Optional[int] = None
        self._reach_frames: Dict[str, int] = {
            "left_joint": 0,
            "right_joint": 0,
            "left_ee": 0,
            "right_ee": 0,
        }
        self._reach_values: Dict[str, Optional[float]] = {
            key: None for key in self._reach_frames
        }
        self._reach_topics: Dict[str, str] = {}
        self._model_positions: Optional[List[float]] = None
        self._torques: Optional[List[float]] = None
        self._arm_ctrl_mode: Optional[List[float]] = None
        self._arm_ctrl_mode_frames = 0
        self._ik_result_positions: Optional[List[float]] = None
        self._ik_result_frames = 0

        self.joint_pub = self.node.create_publisher(
            CmdArmJoints, JOINTS_TOPIC, LATEST_QOS
        )
        self.joint_traj_pub = self.node.create_publisher(
            CmdArmJointsTraj, JOINT_TRAJ_TOPIC, LATEST_QOS
        )
        self.ee_pub = self.node.create_publisher(CmdEePoses, EE_TOPIC, LATEST_QOS)
        self.ee_traj_pub = self.node.create_publisher(
            CmdEePosesTraj, EE_TRAJ_TOPIC, LATEST_QOS
        )
        self.ee_ik_pub = self.node.create_publisher(
            CmdEePoses, IK_INPUT_TOPIC, IK_PUB_QOS
        )
        self.ee_adv_ik_pub = self.node.create_publisher(
            CmdEePosesIK, IK_ADVANCE_TOPIC, IK_PUB_QOS
        )
        self._lowstate_sub = self.node.create_subscription(
            LowState, LOWSTATE_TOPIC, self._on_lowstate, LATEST_QOS
        )
        self._ik_result_sub = self.node.create_subscription(
            CmdArmJoints, IK_RESULT_TOPIC, self._on_ik_result, IK_RESULT_QOS
        )
        self._reach_subs = []
        for side in ("left", "right"):
            for kind, endpoint in (
                ("joint", f"state_{side}_arm_joints_reach_time"),
                ("ee", f"state_{side}_arm_pose_reach_time"),
            ):
                key = f"{side}_{kind}"
                topic = f"/control/arm/{endpoint}"
                self._reach_topics[key] = topic
                self._reach_subs.append(
                    self.node.create_subscription(
                        Float32,
                        topic,
                        self._reach_callback(key),
                        BEST_EFFORT_QOS,
                    )
                )

        self._controller_cli = self.node.create_client(
            ControlStatus, "/control/controller/get_status"
        )
        self._arm_mode_get_cli = self.node.create_client(
            ChangeArmCtrlMode, "/control/arm/get_control_mode"
        )
        self._arm_mode_set_cli = self.node.create_client(
            ChangeArmCtrlMode, "/control/arm/set_control_mode"
        )
        self._wheel_mode_get_cli = self.node.create_client(
            ChangeTorsoCtrlMode, "/control/wheel_mpc/get_control_mode"
        )
        self._wheel_mode_set_cli = self.node.create_client(
            ChangeTorsoCtrlMode, "/control/wheel_mpc/set_control_mode"
        )
        self._wheel_quick_get_cli = self.node.create_client(
            ChangeLbQuickMode, "/control/wheel_mpc/get_arm_quick_mode"
        )
        self._wheel_quick_set_cli = self.node.create_client(
            ChangeLbQuickMode, "/control/wheel_mpc/set_arm_quick_mode"
        )
        self._humanoid_quick_get_cli = self.node.create_client(
            SetArmQuickMode, "/control/mpc/get_quick_mode"
        )
        self._humanoid_quick_set_cli = self.node.create_client(
            SetArmQuickMode, "/control/mpc/set_quick_mode"
        )
        self._timed_clis = {
            side: self.node.create_client(
                WheelSetTimedTarget,
                f"/control/arm/set_{side}_joints_timed_target",
            )
            for side in ("left", "right")
        }
        self._timed_ee_clis = {
            side: self.node.create_client(
                WheelSetTimedTarget,
                f"/control/arm/set_{side}_ee_pose_timed_target",
            )
            for side in ("left", "right")
        }
        self._reach_error_cli = self.node.create_client(
            EePoseReachError, "/control/arm/get_ee_pose_reach_error"
        )
        self._access_cli = self.node.create_client(
            AccessIkSolve, "/control/wheel_mpc/check_ee_accessibility"
        )
        self._async_cli = self.node.create_client(
            SetBool, "/control/wheel_mpc/enable_single_timed_target_async_exec"
        )
        self._arm_action_cli = self.node.create_client(
            ExecuteArmAction, "/control/arm/execute_arm_action"
        )
        self._interrupt_cli = self.node.create_client(
            Trigger, "/control/arm/interrupt_arm_action"
        )
        # 旧版运控/桥只提供 interrupt_arm_traj，保留作为回退。
        self._interrupt_legacy_cli = self.node.create_client(
            Trigger, "/control/arm/interrupt_arm_traj"
        )
        self._fk_cli = self.node.create_client(
            SolveFk, "/planning/kinematics/solve_arm_fk"
        )
        self._ik_cli = self.node.create_client(
            SolveEePoses, "/planning/kinematics/solve_ee_poses_ik"
        )
        self._ik_multi_cli = self.node.create_client(
            SolveEePoses, "/planning/kinematics/solve_ee_poses_ik_multi_refer"
        )
        self._fk_frame_cli = self.node.create_client(
            SolveFkReferFrame, "/planning/kinematics/solve_arm_fk_with_frame"
        )
        self._arm_ctrl_mode_sub = self.node.create_subscription(
            Float64Array,
            "/control/wheel_mpc/state_arm_control_mode",
            self._on_arm_ctrl_mode,
            BEST_EFFORT_QOS,
        )
        self._known_clients = {
            "/control/controller/get_status": self._controller_cli,
            "/control/arm/get_control_mode": self._arm_mode_get_cli,
            "/control/arm/set_control_mode": self._arm_mode_set_cli,
            "/control/wheel_mpc/get_control_mode": self._wheel_mode_get_cli,
            "/control/wheel_mpc/set_control_mode": self._wheel_mode_set_cli,
            "/control/wheel_mpc/get_arm_quick_mode": self._wheel_quick_get_cli,
            "/control/wheel_mpc/set_arm_quick_mode": self._wheel_quick_set_cli,
            "/control/arm/get_ee_pose_reach_error": self._reach_error_cli,
            "/control/wheel_mpc/check_ee_accessibility": self._access_cli,
            "/control/wheel_mpc/enable_single_timed_target_async_exec": self._async_cli,
            "/control/arm/execute_arm_action": self._arm_action_cli,
            "/control/arm/interrupt_arm_action": self._interrupt_cli,
            "/control/arm/interrupt_arm_traj": self._interrupt_legacy_cli,
            "/planning/kinematics/solve_arm_fk": self._fk_cli,
            "/planning/kinematics/solve_ee_poses_ik": self._ik_cli,
            "/planning/kinematics/solve_ee_poses_ik_multi_refer": self._ik_multi_cli,
            "/planning/kinematics/solve_arm_fk_with_frame": self._fk_frame_cli,
        }

    def _on_lowstate(self, msg: LowState) -> None:
        model = _model_positions_from(msg)
        if model is None:
            return
        with self._lock:
            self._model_positions = model
            self._arm_positions = model[len(TORSO_SLOTS):]
            self._torques = [float(state.tau_est) for state in msg.motor_state]
            self._lowstate_tick = int(msg.tick)

    def _on_ik_result(self, msg: CmdArmJoints) -> None:
        values = ([float(v) for v in msg.left_joints.joint_angles]
                  + [float(v) for v in msg.right_joints.joint_angles])
        with self._lock:
            self._ik_result_positions = values
            self._ik_result_frames += 1

    @property
    def ik_result_frames(self) -> int:
        with self._lock:
            return self._ik_result_frames

    def read_ik_result(self) -> Optional[List[float]]:
        """最近求解结果 [左臂 7, 右臂 7]（弧度），不是实测或控制请求。"""
        with self._lock:
            return (list(self._ik_result_positions)
                    if self._ik_result_positions is not None else None)

    def _reach_callback(self, key: str):
        def callback(msg: Float32) -> None:
            with self._lock:
                self._reach_values[key] = float(msg.data)
                self._reach_frames[key] += 1

        return callback

    @property
    def lowstate_tick(self) -> Optional[int]:
        with self._lock:
            return self._lowstate_tick

    def read_arm_positions(self) -> Optional[List[float]]:
        """返回 [左臂 7, 右臂 7] 实测关节角，不是目标回读。"""
        with self._lock:
            return list(self._arm_positions) if self._arm_positions is not None else None

    def read_joint_torques(self) -> Optional[List[float]]:
        """返回 /lowstate 全 35 个电机的 tau_est（Nm）。"""
        with self._lock:
            return list(self._torques) if self._torques is not None else None

    def reach_value(self, side: str, kind: str) -> Optional[float]:
        key = f"{side}_{kind}"
        if key not in self._reach_values:
            raise ValueError("side must be left/right; kind must be joint/ee")
        with self._lock:
            value = self._reach_values[key]
        return value if value is not None and math.isfinite(value) and value > 0 else None

    def reach_frames(self, side: str, kind: str) -> int:
        key = f"{side}_{kind}"
        if key not in self._reach_frames:
            raise ValueError("side must be left/right; kind must be joint/ee")
        with self._lock:
            return self._reach_frames[key]

    def wait_for_reach_time(
        self, side: str, kind: str, previous_frames: int, timeout: float
    ) -> Optional[float]:
        key = f"{side}_{kind}"
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            with self._lock:
                if self._reach_frames[key] > previous_frames:
                    value = self._reach_values[key]
                    if value is not None and math.isfinite(value) and value > 0:
                        return value
        return None

    def spin_once(self, timeout: float = 0.05) -> None:
        rclpy.spin_once(self.node, timeout_sec=timeout)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            self.spin_once(min(0.05, deadline - time.monotonic()))

    def wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            if self.read_arm_positions() is not None:
                return True
        return False

    def wait_for_consumer(self, topic: str, timeout: float = 5.0) -> bool:
        publishers = {
            JOINTS_TOPIC: self.joint_pub,
            JOINT_TRAJ_TOPIC: self.joint_traj_pub,
            EE_TOPIC: self.ee_pub,
            EE_TRAJ_TOPIC: self.ee_traj_pub,
        }
        if topic not in publishers:
            raise ValueError(f"unknown command topic: {topic}")
        publisher = publishers[topic]
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            if publisher.get_subscription_count() > 0:
                return True
        return False

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
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"service call failed: {exc}")
            return None

    @staticmethod
    def _reply(response, value=None, success_field: str = "result") -> ServiceReply:
        if response is None:
            return ServiceReply(False, "service unavailable or timed out")
        remote_ex = int(getattr(response, "remote_ex", 0))
        success = bool(getattr(response, success_field)) and remote_ex == 0
        message = str(getattr(response, "message", ""))
        if remote_ex:
            message = f"remote_ex={remote_ex}: {message}"
        return ServiceReply(success, message, value)

    def get_controller(self, timeout: float = 5.0) -> Optional[ControllerState]:
        response = self._call(
            self._controller_cli, ControlStatus.Request(), timeout
        )
        if response is None or int(response.remote_ex) != 0:
            return None
        return ControllerState(
            str(response.current_controller),
            bool(response.switching),
            str(response.pending_controller),
        )

    def get_arm_mode(self, timeout: float = 5.0) -> ServiceReply:
        response = self._call(
            self._arm_mode_get_cli, ChangeArmCtrlMode.Request(), timeout
        )
        if response is None:
            return ServiceReply(False, "get arm control mode unavailable")
        reply = self._reply(response, int(response.mode))
        return reply

    def set_arm_mode(self, mode: int, timeout: float = 5.0) -> ServiceReply:
        request = ChangeArmCtrlMode.Request()
        request.control_mode = int(mode)
        response = self._call(self._arm_mode_set_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "set arm control mode unavailable")
        return self._reply(response, int(response.mode))

    def get_wheel_mode(self, timeout: float = 5.0) -> ServiceReply:
        response = self._call(
            self._wheel_mode_get_cli, ChangeTorsoCtrlMode.Request(), timeout
        )
        if response is None:
            return ServiceReply(False, "get wheel_mpc control mode unavailable")
        return self._reply(response, int(response.mode))

    def set_wheel_mode(self, mode: int, timeout: float = 5.0) -> ServiceReply:
        request = ChangeTorsoCtrlMode.Request()
        request.control_mode = int(mode)
        response = self._call(self._wheel_mode_set_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "set wheel_mpc control mode unavailable")
        return self._reply(response, int(response.mode))

    def get_quick_mode(self, stack: str, timeout: float = 5.0) -> ServiceReply:
        if stack == "wheeled":
            response = self._call(
                self._wheel_quick_get_cli, ChangeLbQuickMode.Request(), timeout
            )
            field = "success"
        elif stack == "humanoid":
            response = self._call(
                self._humanoid_quick_get_cli, SetArmQuickMode.Request(), timeout
            )
            field = "result"
        else:
            raise ValueError("stack must be wheeled or humanoid")
        if response is None:
            return ServiceReply(False, "get arm quick mode unavailable")
        value = int(response.quick_mode if stack == "wheeled" else response.source)
        return self._reply(response, value, success_field=field)

    def set_quick_mode(
        self, stack: str, mode: int, timeout: float = 5.0
    ) -> ServiceReply:
        if stack == "wheeled":
            request = ChangeLbQuickMode.Request()
            request.quick_mode = int(mode)
            response = self._call(self._wheel_quick_set_cli, request, timeout)
            field = "success"
        elif stack == "humanoid":
            request = SetArmQuickMode.Request()
            request.source = int(mode)
            response = self._call(self._humanoid_quick_set_cli, request, timeout)
            field = "result"
        else:
            raise ValueError("stack must be wheeled or humanoid")
        if response is None:
            return ServiceReply(False, "set arm quick mode unavailable")
        value = int(response.quick_mode if stack == "wheeled" else response.source)
        return self._reply(response, value, success_field=field)

    def send_joint_positions(self, positions_rad: Sequence[float]) -> bool:
        values = _numbers(positions_rad, 14, "positions_rad")
        if self.joint_pub.get_subscription_count() == 0:
            return False
        msg = CmdArmJoints()
        msg.left_joints.joint_angles = values[:7]
        msg.right_joints.joint_angles = values[7:]
        self.joint_pub.publish(msg)
        return True

    def send_joint_trajectory(
        self, times: Sequence[float], positions_rad: Sequence[Sequence[float]]
    ) -> bool:
        times = _numbers(times, len(times), "times")
        if len(times) < 2 or times[0] < 0 or any(
            b <= a for a, b in zip(times, times[1:])
        ):
            raise ValueError("trajectory times must be non-negative and strictly increasing")
        if len(positions_rad) != len(times):
            raise ValueError("trajectory positions must match times")
        flat = []
        for point in positions_rad:
            flat.extend(_numbers(point, 14, "trajectory point"))
        if self.joint_traj_pub.get_subscription_count() == 0:
            return False
        msg = CmdArmJointsTraj()
        msg.times = times
        msg.values = flat
        self.joint_traj_pub.publish(msg)
        return True

    def cancel_joint_trajectory(self) -> bool:
        if self.joint_traj_pub.get_subscription_count() == 0:
            return False
        self.joint_traj_pub.publish(CmdArmJointsTraj())
        return True

    @staticmethod
    def _fill_ee(msg: CmdEePoses, left: Sequence[float], right: Sequence[float], frame: int) -> None:
        if int(frame) not in (0, 1):
            raise ValueError("frame must be 0 (world) or 1 (local)")
        for target, values in (
            (msg.hand_poses.left_pose, _numbers(left, 7, "left pose")),
            (msg.hand_poses.right_pose, _numbers(right, 7, "right pose")),
        ):
            norm = math.sqrt(sum(value * value for value in values[3:]))
            if abs(norm - 1.0) > 1e-3:
                raise ValueError("pose quaternion xyzw must have unit norm")
            target.pos_xyz = values[:3]
            target.quat_xyzw = values[3:]
        msg.frame = int(frame)

    def send_ee_poses(
        self, left: Sequence[float], right: Sequence[float], frame: int = 1
    ) -> bool:
        msg = CmdEePoses()
        self._fill_ee(msg, left, right, frame)
        if self.ee_pub.get_subscription_count() == 0:
            return False
        self.ee_pub.publish(msg)
        return True

    def send_ee_trajectory(
        self,
        times: Sequence[float],
        left: Sequence[Sequence[float]],
        right: Sequence[Sequence[float]],
        frame: int = 1,
    ) -> bool:
        times = _numbers(times, len(times), "times")
        if len(times) < 2 or times[0] < 0 or any(
            b <= a for a, b in zip(times, times[1:])
        ):
            raise ValueError("trajectory times must be non-negative and strictly increasing")
        if len(left) != len(times) or len(right) != len(times):
            raise ValueError("EE trajectory poses must match times")
        if self.ee_traj_pub.get_subscription_count() == 0:
            return False
        msg = CmdEePosesTraj()
        msg.times = times
        msg.values = []
        for left_pose, right_pose in zip(left, right):
            point = CmdEePoses()
            self._fill_ee(point, left_pose, right_pose, frame)
            msg.values.append(point)
        self.ee_traj_pub.publish(msg)
        return True

    def cancel_ee_trajectory(self) -> bool:
        if self.ee_traj_pub.get_subscription_count() == 0:
            return False
        self.ee_traj_pub.publish(CmdEePosesTraj())
        return True

    def send_timed_joint_target(
        self, side: str, joints_rad: Sequence[float], desire_time: float,
        timeout: float = 5.0,
    ) -> ServiceReply:
        return self._send_timed(
            self._timed_clis.get(side), joints_rad, 7, 0, desire_time, timeout
        )

    def send_timed_ee_target(
        self, side: str, pose_zyx_rad: Sequence[float], desire_time: float,
        frame: int = 1, timeout: float = 5.0,
    ) -> ServiceReply:
        return self._send_timed(
            self._timed_ee_clis.get(side), pose_zyx_rad, 6, frame,
            desire_time, timeout,
        )

    def _send_timed(
        self, client, values: Sequence[float], size: int, frame: int,
        desire_time: float, timeout: float,
    ) -> ServiceReply:
        if client is None:
            raise ValueError("side must be left or right")
        values = _numbers(values, size, "timed target")
        if int(frame) not in (0, 1):
            raise ValueError("frame must be 0 or 1")
        if not math.isfinite(desire_time) or desire_time <= 0:
            raise ValueError("desire_time must be positive and finite")
        request = WheelSetTimedTarget.Request()
        request.frame = int(frame)
        request.desire_time = float(desire_time)
        request.cmd_vec = values
        response = self._call(client, request, timeout)
        if response is None:
            return ServiceReply(False, "timed target service unavailable or timed out")
        reply = self._reply(response, success_field="is_success")
        return ServiceReply(
            reply.success, reply.message, actual_time=float(response.actual_time)
        )

    # ---------------- 模型角（18 维）/ 端点可用性 ----------------

    def read_torso_positions(self) -> Optional[List[float]]:
        """返回 [knee, leg, waist_pitch, waist_yaw] 实测角（弧度）。"""
        with self._lock:
            if self._model_positions is None:
                return None
            return list(self._model_positions[:len(TORSO_SLOTS)])

    def read_model_positions(self) -> Optional[List[float]]:
        """返回 [躯干 4, 左臂 7, 右臂 7] 共 18 维实测角（同一帧）。"""
        with self._lock:
            return list(self._model_positions) if self._model_positions is not None else None

    def service_available(self, name: str, timeout: float = 0.0) -> bool:
        client = self._known_clients.get(name)
        if client is None:
            return False
        return bool(client.wait_for_service(timeout_sec=max(0.0, timeout)))

    def reach_topic_available(self, side: str, kind: str) -> bool:
        key = f"{side}_{kind}"
        if key not in self._reach_topics:
            raise ValueError("side must be left/right; kind must be joint/ee")
        return self.node.count_publishers(self._reach_topics[key]) > 0

    def _on_arm_ctrl_mode(self, msg: Float64Array) -> None:
        with self._lock:
            self._arm_ctrl_mode = [float(v) for v in msg.data]
            self._arm_ctrl_mode_frames += 1

    def read_arm_control_mode(self) -> Optional[List[float]]:
        with self._lock:
            return list(self._arm_ctrl_mode) if self._arm_ctrl_mode is not None else None

    def arm_control_mode_frames(self) -> int:
        with self._lock:
            return self._arm_ctrl_mode_frames

    # ---------------- 新增服务 ----------------

    def get_reach_error(self, side: str, timeout: float = 5.0) -> ServiceReply:
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        request = EePoseReachError.Request()
        request.is_left = side == "left"
        response = self._call(self._reach_error_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "get_ee_pose_reach_error unavailable or timed out")
        return ServiceReply(
            bool(response.success),
            "" if response.success else f"reach error query failed (remote_ex={int(response.remote_ex)})",
            vector=[float(v) for v in response.err_vector],
        )

    def check_accessibility(
        self, side: str, pose_zyx_rad: Sequence[float], is_local: bool = True,
        is_whole_body: bool = False, total_time: float = 2.0,
        max_attempts: int = 20, linear_error_max: float = 0.01,
        angular_error_max: float = 0.05, timeout: float = 10.0,
    ) -> ServiceReply:
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        pose = _numbers(pose_zyx_rad, 6, "pose_zyx_rad")
        request = AccessIkSolve.Request()
        request.total_time_desired = float(total_time)
        request.max_attempts = int(max_attempts)
        request.linear_error_max = float(linear_error_max)
        request.angular_error_max = float(angular_error_max)
        request.pose_desired = pose
        request.is_local = bool(is_local)
        request.is_left = side == "left"
        request.is_whole_body = bool(is_whole_body)
        response = self._call(self._access_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "check_ee_accessibility unavailable or timed out")
        return ServiceReply(
            bool(response.success),
            "" if response.success else f"target not reachable (remote_ex={int(response.remote_ex)})",
            vector=[float(v) for v in response.q_best],
            linear_error=float(response.best_linear_error),
        )

    def set_async_exec(self, enabled: bool, timeout: float = 5.0) -> ServiceReply:
        """定时目标并发开关。没有查询接口，默认串行(false)。"""
        request = SetBool.Request()
        request.data = bool(enabled)
        response = self._call(self._async_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "enable_single_timed_target_async_exec unavailable")
        return self._reply(response, 1 if enabled else 0, success_field="success")

    def execute_arm_action(self, action_name: str, timeout: float = 15.0) -> ServiceReply:
        request = ExecuteArmAction.Request()
        request.action_name = str(action_name)
        response = self._call(self._arm_action_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "execute_arm_action unavailable")
        return self._reply(response, success_field="success")

    def interrupt_arm_traj(self, timeout: float = 5.0) -> ServiceReply:
        """中断手臂动作；优先 interrupt_arm_action，旧服务名作为回退。"""
        client = self._interrupt_cli
        if not client.wait_for_service(timeout_sec=min(1.0, max(0.0, timeout))):
            client = self._interrupt_legacy_cli
        response = self._call(client, Trigger.Request(), timeout)
        if response is None:
            return ServiceReply(
                False, "interrupt_arm_action unavailable (also tried interrupt_arm_traj)")
        return self._reply(response, success_field="success")

    # ---------------- FK / IK ----------------

    def solve_fk(self, arm_positions_rad: Optional[Sequence[float]] = None,
                 timeout: float = 10.0) -> ServiceReply:
        """正解：14 维臂角（缺省用实测）-> 双臂末端位姿；内部拼 18 维。"""
        if arm_positions_rad is None:
            model = self.read_model_positions()
            if model is None:
                return ServiceReply(False, "no measured model positions")
        else:
            arm = _numbers(arm_positions_rad, 2 * ARM_DOF, "arm positions")
            torso = self.read_torso_positions()
            if torso is None:
                return ServiceReply(False, "no measured torso positions")
            model = torso + arm
        request = SolveFk.Request()
        request.q = [float(v) for v in model]
        response = self._call(self._fk_cli, request, timeout)
        if response is None:
            return ServiceReply(False, "solve_arm_fk unavailable or timed out")
        return ServiceReply(
            bool(response.success),
            str(response.message),
            poses=_hand_poses_to_dict(response.hand_poses),
        )

    def solve_ik(
        self, side: str, pose_zyx_rad: Sequence[float], is_local: bool = True,
        other_pose_zyx_rad: Optional[Sequence[float]] = None,
        arm_positions_rad: Optional[Sequence[float]] = None,
        timeout: float = 10.0,
        multi: bool = False,
        constraint_mode: Optional[int] = None,
    ) -> ServiceReply:
        """逆解：单侧 6 维 ZYX -> 14 维臂角；q_torso 原样返回但不由本层下发。

        ``multi`` 走多参考系服务；``constraint_mode`` 非空时启用自定义 IK 参数
        （容差/迭代次数与厂商 aria_ik_fk 用例一致）。
        """
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        target = _numbers(pose_zyx_rad, 6, "pose_zyx_rad")
        other = target if other_pose_zyx_rad is None else _numbers(
            other_pose_zyx_rad, 6, "other_pose_zyx_rad"
        )
        left_zyx, right_zyx = (target, other) if side == "left" else (other, target)
        if arm_positions_rad is None:
            q14 = self.read_arm_positions()
            if q14 is None:
                return ServiceReply(False, "no measured arm positions")
        else:
            q14 = _numbers(arm_positions_rad, 2 * ARM_DOF, "arm positions")

        inner = CmdEePosesIK()
        inner.ee_poses.frame = 1 if is_local else 0
        _fill_hand_pose(inner.ee_poses.hand_poses.left_pose, left_zyx)
        _fill_hand_pose(inner.ee_poses.hand_poses.right_pose, right_zyx)
        inner.current_joints.left_joints.joint_angles = [float(v) for v in q14[:ARM_DOF]]
        inner.current_joints.right_joints.joint_angles = [float(v) for v in q14[ARM_DOF:]]
        inner.use_custom_ik_param = False
        if constraint_mode is not None:
            inner.use_custom_ik_param = True
            param = inner.ik_param
            param.major_optimality_tol = 0.004
            param.major_feasibility_tol = 0.004
            param.minor_feasibility_tol = 0.004
            param.major_iterations_limit = 100.0
            param.orientation_constraint_tol = 0.004
            param.pos_constraint_tol = 0.004
            param.pos_cost_weight = 10.0
            param.constraint_mode = int(constraint_mode)
        request = SolveEePoses.Request()
        request.solve_ee_poses_request = inner
        client = self._ik_multi_cli if multi else self._ik_cli
        service = "solve_ee_poses_ik_multi_refer" if multi else "solve_ee_poses_ik"
        response = self._call(client, request, timeout)
        if response is None:
            return ServiceReply(False, f"{service} unavailable or timed out")
        return ServiceReply(
            bool(response.success),
            str(response.message),
            vector=[float(v) for v in response.q_arm],
            torso=[float(v) for v in response.q_torso],
            poses=_hand_poses_to_dict(response.hand_poses) if response.q_arm else None,
        )

    def solve_ik_multi_refer(
        self, side: str, pose_zyx_rad: Sequence[float], is_local: bool = True,
        other_pose_zyx_rad: Optional[Sequence[float]] = None,
        arm_positions_rad: Optional[Sequence[float]] = None,
        timeout: float = 10.0,
        constraint_mode: Optional[int] = None,
    ) -> ServiceReply:
        """多参考系逆解：请求体与单参考一致，只是服务端不同。"""
        return self.solve_ik(
            side, pose_zyx_rad, is_local=is_local,
            other_pose_zyx_rad=other_pose_zyx_rad,
            arm_positions_rad=arm_positions_rad, timeout=timeout,
            multi=True, constraint_mode=constraint_mode,
        )

    def solve_fk_with_frame(
        self, q_rad: Optional[Sequence[float]] = None,
        base_frame: str = "", end_effector_frame: str = "",
        hand_side: int = 0, timeout: float = 10.0,
    ) -> ServiceReply:
        """带参考系正解：q 为 14 或 18 维（缺省用实测 18 维）。"""
        if q_rad is None:
            q = self.read_model_positions()
            if q is None:
                return ServiceReply(False, "no measured model positions")
        else:
            q = [float(v) for v in q_rad]
            if len(q) not in (2 * ARM_DOF, IK_DOF) or not all(
                    math.isfinite(v) for v in q):
                return ServiceReply(False, "q must be 14 or 18 finite values")
        request = SolveFkReferFrame.Request()
        request.q = q
        request.base_frame = str(base_frame)
        request.end_effector_frame = str(end_effector_frame)
        request.hand_side = int(hand_side)
        response = self._call(self._fk_frame_cli, request, timeout)
        if response is None:
            return ServiceReply(
                False, "solve_arm_fk_with_frame unavailable or timed out")
        remote_ex = int(getattr(response, "remote_ex", 0))
        message = str(response.message)
        if remote_ex:
            message = f"remote_ex={remote_ex}: {message}"
        return ServiceReply(
            bool(response.success) and remote_ex == 0,
            message,
            poses=_hand_poses_to_dict(response.hand_poses),
        )

    def publish_ee_poses_ik(
        self, left_zyx_rad: Sequence[float], right_zyx_rad: Sequence[float],
        frame: int = 0, repeats: int = 1, interval: float = 0.05,
    ) -> ServiceReply:
        """发布 IK 输入（CmdEePoses）；只触发求解，结果看 ik_result_arm_joints。"""
        message = CmdEePoses()
        message.frame = int(frame)
        _fill_hand_pose(message.hand_poses.left_pose,
                        _numbers(left_zyx_rad, 6, "left pose"))
        _fill_hand_pose(message.hand_poses.right_pose,
                        _numbers(right_zyx_rad, 6, "right pose"))
        count = max(1, int(repeats))
        for index in range(count):
            self.ee_ik_pub.publish(message)
            if index + 1 < count:
                time.sleep(max(0.0, interval))
        return ServiceReply(True, f"published {count}x on {IK_INPUT_TOPIC}")

    def publish_ee_poses_advance_ik(
        self, left_zyx_rad: Sequence[float], right_zyx_rad: Sequence[float],
        frame: int = 0, q14_rad: Optional[Sequence[float]] = None,
        repeats: int = 1, interval: float = 0.05,
    ) -> ServiceReply:
        """发布进阶 IK 输入（CmdEePosesIK）；参数与厂商用例一致。"""
        if q14_rad is None:
            q14 = self.read_arm_positions()
            if q14 is None:
                return ServiceReply(False, "no measured arm positions")
        else:
            q14 = _numbers(q14_rad, 2 * ARM_DOF, "arm positions")
        message = CmdEePosesIK()
        message.ee_poses.frame = int(frame)
        _fill_hand_pose(message.ee_poses.hand_poses.left_pose,
                        _numbers(left_zyx_rad, 6, "left pose"))
        _fill_hand_pose(message.ee_poses.hand_poses.right_pose,
                        _numbers(right_zyx_rad, 6, "right pose"))
        message.current_joints.left_joints.joint_angles = [
            float(v) for v in q14[:ARM_DOF]]
        message.current_joints.right_joints.joint_angles = [
            float(v) for v in q14[ARM_DOF:]]
        message.use_custom_ik_param = True
        param = message.ik_param
        param.major_optimality_tol = 0.004
        param.major_feasibility_tol = 0.004
        param.minor_feasibility_tol = 0.004
        param.major_iterations_limit = 100.0
        param.orientation_constraint_tol = 0.004
        param.pos_constraint_tol = 0.004
        param.pos_cost_weight = 10.0
        count = max(1, int(repeats))
        for index in range(count):
            self.ee_adv_ik_pub.publish(message)
            if index + 1 < count:
                time.sleep(max(0.0, interval))
        return ServiceReply(True, f"published {count}x on {IK_ADVANCE_TOPIC}")

    def close(self) -> None:
        if not self._owns_node:
            return
        try:
            self.node.destroy_node()
        finally:
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()

    def __enter__(self) -> "ArmClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
