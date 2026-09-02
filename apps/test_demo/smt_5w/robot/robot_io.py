#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""商超机器人原生 ROS 运行时。

本模块集中管理 ROS 节点、消息类型、话题、服务与 TF。模块导入阶段只依赖
Python 标准库，因此在没有安装 ROS 的开发机上也可以安全导入；真正的 ROS
依赖仅在 :class:`RobotIO` 实例化时加载。

设计约束：

* 业务层不直接创建 ``Publisher``、``Subscriber`` 或 ``ServiceProxy``；
* 所有 readiness 等待都有明确超时，禁止无限阻塞；
* 人形与轮臂复用同一运行时契约，仅在确有差异的能力上覆写；
* 双臂 IK 统一使用 ``two_arm_hand_pose_cmd_srv_muli_refer`` 服务。
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple

from .geometry import (
    normalize_angle,
    quat_xyzw,
    quaternion_multiply,
    rotate_vector_by_quat,
    yaw_from_quat,
)


DEFAULT_IK_SERVICE = "/ik/two_arm_hand_pose_cmd_srv_muli_refer"
DEFAULT_HAND_TOPIC = "control_robot_hand_position"
DEFAULT_LEJUCLAW_SERVICE = "/control_robot_leju_claw"


@dataclass(frozen=True)
class _LejuClawInFlight:
    """一次夹爪调用的 generation；完成事件必须与当次 client 成对保存。"""

    completed: threading.Event
    timed_out: threading.Event
    client: Any


class RobotControlError(RuntimeError):
    """机器人控制层统一异常基类。"""


class RosDependencyError(RobotControlError):
    """ROS 或机器人消息依赖缺失。"""


class RosReadinessError(RobotControlError):
    """ROS 话题、服务或 TF 在限定时间内未就绪。"""


class RosOperationError(RobotControlError):
    """ROS 调用已发起，但执行失败。"""


@dataclass(frozen=True)
class RosTopics:
    """运行时使用的话题名称，支持通过配置覆盖。"""

    head: str = "/robot_head_motion_data"
    cmd_vel: str = "/cmd_vel"
    lb_torso_pose: str = "/cmd_lb_torso_pose"
    torso_open_loop_state: str = "/torso_open_loop_state"
    lb_torso_pose_reach_time: str = "/lb_torso_pose_reach_time"
    arm_trajectory: str = "/kuavo_arm_traj"
    sensors: str = "/sensors_data_raw"
    qr_detection: str = "/robot_tag_info"
    gait: str = "/humanoid_switch_gait_by_name"

    @classmethod
    def from_params(cls, params: Mapping[str, Any]) -> "RosTopics":
        return cls(
            head=str(
                _setting(
                    params,
                    ("topics", "head"),
                    ("qr", "head_topic"),
                    default=cls.head,
                )
            ),
            cmd_vel=str(_setting(params, ("topics", "cmd_vel"), default=cls.cmd_vel)),
            lb_torso_pose=str(
                _setting(
                    params,
                    ("topics", "lb_torso_pose"),
                    default=cls.lb_torso_pose,
                )
            ),
            torso_open_loop_state=str(
                _setting(
                    params,
                    ("topics", "torso_open_loop_state"),
                    default=cls.torso_open_loop_state,
                )
            ),
            lb_torso_pose_reach_time=str(
                _setting(
                    params,
                    ("topics", "lb_torso_pose_reach_time"),
                    default=cls.lb_torso_pose_reach_time,
                )
            ),
            arm_trajectory=str(
                _setting(params, ("topics", "arm_trajectory"), default=cls.arm_trajectory)
            ),
            sensors=str(_setting(params, ("topics", "sensors"), default=cls.sensors)),
            qr_detection=str(
                _setting(
                    params,
                    ("topics", "qr_detection"),
                    ("qr", "detection_topic"),
                    default=cls.qr_detection,
                )
            ),
            gait=str(_setting(params, ("topics", "gait"), default=cls.gait)),
        )


@dataclass(frozen=True)
class RosServices:
    """运行时使用的服务名称。"""

    arm_ik: str = DEFAULT_IK_SERVICE
    arm_mode: str = "/arm_traj_change_mode"
    base_pitch_limit: str = "/humanoid/mpc/enable_base_pitch_limit"

    @classmethod
    def from_params(cls, params: Mapping[str, Any]) -> "RosServices":
        return cls(
            arm_ik=str(_setting(params, ("services", "arm_ik"), default=cls.arm_ik)),
            arm_mode=str(_setting(params, ("services", "arm_mode"), default=cls.arm_mode)),
            base_pitch_limit=str(
                _setting(
                    params,
                    ("services", "base_pitch_limit"),
                    default=cls.base_pitch_limit,
                )
            ),
        )


@dataclass(frozen=True)
class RosTimeouts:
    """所有 ROS readiness 超时，单位均为秒。"""

    readiness: float = 5.0
    service: float = 5.0
    publisher: float = 3.0
    topic: float = 2.0
    transform: float = 0.5

    @classmethod
    def from_params(cls, params: Mapping[str, Any]) -> "RosTimeouts":
        readiness = _positive_timeout(
            _setting(
                params,
                ("timeouts", "readiness"),
                ("robot", "readiness_timeout"),
                ("robot_io", "readiness_timeout"),
                ("runtime", "readiness_timeout"),
                ("readiness_timeout",),
                default=cls.readiness,
            ),
            "readiness",
        )
        return cls(
            readiness=readiness,
            service=_positive_timeout(
                _setting(
                    params,
                    ("timeouts", "service"),
                    ("robot_io", "service_timeout"),
                    ("runtime", "service_timeout"),
                    default=readiness,
                ),
                "service",
            ),
            publisher=_positive_timeout(
                _setting(
                    params,
                    ("timeouts", "publisher"),
                    ("robot_io", "publisher_timeout"),
                    ("runtime", "publisher_timeout"),
                    default=cls.publisher,
                ),
                "publisher",
            ),
            topic=_positive_timeout(
                _setting(
                    params,
                    ("timeouts", "topic"),
                    ("robot_io", "topic_timeout"),
                    ("runtime", "topic_timeout"),
                    default=cls.topic,
                ),
                "topic",
            ),
            transform=_positive_timeout(
                _setting(
                    params,
                    ("timeouts", "transform"),
                    ("robot_io", "transform_timeout"),
                    ("runtime", "transform_timeout"),
                    default=cls.transform,
                ),
                "transform",
            ),
        )


@dataclass
class RosBindings:
    """ROS 模块与消息类型集合。

    该对象允许测试注入轻量 fake，避免单元测试依赖 ROS master。
    """

    rospy: Any
    tf2_ros: Any
    Twist: Any
    JointState: Any
    AprilTagDetectionArray: Any
    robotHeadMotionData: Any
    sensorsData: Any
    twoArmHandPoseCmd: Any
    twoArmHandPoseCmdSrv: Any
    twoArmHandPoseCmdSrvRequest: Any
    switchGaitByName: Any = None
    changeArmCtrlMode: Any = None
    changeArmCtrlModeRequest: Any = None
    SetBool: Any = None
    SetBoolRequest: Any = None
    ikSolveParam: Any = None
    Detection2DArray: Any = None
    robotHandPosition: Any = None
    controlLejuClaw: Any = None
    controlLejuClawRequest: Any = None
    Float32: Any = None


def load_ros_bindings(robot_type: str) -> RosBindings:
    """按机器人类型延迟导入 ROS 依赖，并给出可操作的错误信息。"""

    normalized = _normalize_robot_type(robot_type)
    try:
        import rospy
        import tf2_ros
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import JointState

        try:
            from apriltag_ros.msg import AprilTagDetectionArray
        except ImportError:
            from kuavo_msgs.msg import AprilTagDetectionArray

        from kuavo_msgs.msg import robotHeadMotionData, sensorsData, twoArmHandPoseCmd
        from kuavo_msgs.srv import twoArmHandPoseCmdSrv, twoArmHandPoseCmdSrvRequest

        bindings = RosBindings(
            rospy=rospy,
            tf2_ros=tf2_ros,
            Twist=Twist,
            JointState=JointState,
            AprilTagDetectionArray=AprilTagDetectionArray,
            robotHeadMotionData=robotHeadMotionData,
            sensorsData=sensorsData,
            twoArmHandPoseCmd=twoArmHandPoseCmd,
            twoArmHandPoseCmdSrv=twoArmHandPoseCmdSrv,
            twoArmHandPoseCmdSrvRequest=twoArmHandPoseCmdSrvRequest,
        )

        if normalized == "humanoid":
            from kuavo_msgs.msg import switchGaitByName
            from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
            from std_srvs.srv import SetBool, SetBoolRequest

            bindings.switchGaitByName = switchGaitByName
            bindings.changeArmCtrlMode = changeArmCtrlMode
            bindings.changeArmCtrlModeRequest = changeArmCtrlModeRequest
            bindings.SetBool = SetBool
            bindings.SetBoolRequest = SetBoolRequest
        elif normalized == "wheel":
            from kuavo_msgs.msg import ikSolveParam
            from std_msgs.msg import Float32

            bindings.ikSolveParam = ikSolveParam
            bindings.Float32 = Float32
        return bindings
    except ImportError as exc:
        raise RosDependencyError(
            "无法加载 %s 机器人所需的 ROS 依赖: %s。"
            "请确认已 source 对应工作空间的 setup.bash。" % (normalized, exc)
        ) from exc


class RobotIO:
    """ROS 能力的统一入口。

    子类只负责声明机器人差异；通用的话题、IK、TF 和 readiness 逻辑均在此
    实现。实例化会创建通信端点，但不会无限等待任何外部节点。
    """

    robot_type = "base"
    qr_detection_frame = "base_link"
    walk_qr_frame = "base_link"

    def __init__(
        self,
        params: Optional[Mapping[str, Any]] = None,
        node_name: str = "supermarket_robot",
        init_node: bool = True,
        bindings: Optional[RosBindings] = None,
        auto_connect_ik: bool = False,
    ) -> None:
        self.params: Dict[str, Any] = dict(params or {})
        self.topics = RosTopics.from_params(self.params)
        self.services = RosServices.from_params(self.params)
        self.timeouts = RosTimeouts.from_params(self.params)
        self.bindings = bindings or load_ros_bindings(self.robot_type)
        self.rospy = self.bindings.rospy
        self.tf2_ros = self.bindings.tf2_ros
        self._service_lock = threading.RLock()
        self._ik_lock = threading.RLock()
        self._end_effector_lock = threading.RLock()

        if init_node:
            self._initialize_node(node_name)

        # 对外保留常用消息类型，方便各原生控制器通过鸭子类型接入。
        self.Twist = self.bindings.Twist
        self.JointState = self.bindings.JointState
        self.AprilTagDetectionArray = self.bindings.AprilTagDetectionArray
        self.robotHeadMotionData = self.bindings.robotHeadMotionData
        self.sensorsData = self.bindings.sensorsData
        self.twoArmHandPoseCmd = self.bindings.twoArmHandPoseCmd
        self.twoArmHandPoseCmdSrv = self.bindings.twoArmHandPoseCmdSrv
        self.twoArmHandPoseCmdSrvRequest = self.bindings.twoArmHandPoseCmdSrvRequest

        self.head_pub = self.publisher(
            self.topics.head, self.robotHeadMotionData, queue_size=10
        )
        self.cmd_vel_pub = self.publisher(
            self.topics.cmd_vel, self.Twist, queue_size=10
        )
        self.arm_traj_pub = self.publisher(
            self.topics.arm_trajectory, self.JointState, queue_size=10
        )
        self.tf_buffer = self.tf2_ros.Buffer()
        self.tf_listener = self.tf2_ros.TransformListener(self.tf_buffer)

        self.ik_service_name = self.services.arm_ik
        self.ik_srv = None
        self._hand_pub = None
        self._lejuclaw_client = None
        self._lejuclaw_service_name = None
        self._lejuclaw_inflight = None
        if auto_connect_ik:
            self.start_ik_service(timeout=self.timeouts.service)

    def _initialize_node(self, node_name: str) -> None:
        """确保 ROS 节点只初始化一次。"""

        try:
            initialized = bool(self.rospy.core.is_initialized())
        except (AttributeError, TypeError):
            initialized = False
        if not initialized:
            try:
                self.rospy.init_node(str(node_name), anonymous=False)
            except Exception as exc:
                raise RosOperationError("ROS 节点初始化失败[%s]: %s" % (node_name, exc)) from exc

    # ------------------------------------------------------------------
    # 通用 ROS 原语：其他 robot/* 模块仅通过这些入口访问 ROS。
    # ------------------------------------------------------------------
    def publisher(self, topic: str, message_type: Any, **kwargs: Any) -> Any:
        try:
            return self.rospy.Publisher(str(topic), message_type, **kwargs)
        except Exception as exc:
            raise RosOperationError("创建 Publisher 失败[%s]: %s" % (topic, exc)) from exc

    def subscribe(
        self,
        topic: str,
        message_type: Any,
        callback: Callable[[Any], None],
        **kwargs: Any
    ) -> Any:
        try:
            return self.rospy.Subscriber(str(topic), message_type, callback, **kwargs)
        except Exception as exc:
            raise RosOperationError("创建 Subscriber 失败[%s]: %s" % (topic, exc)) from exc

    def service_proxy(self, name: str, service_type: Any, **kwargs: Any) -> Any:
        try:
            return self.rospy.ServiceProxy(str(name), service_type, **kwargs)
        except Exception as exc:
            raise RosOperationError("创建 ServiceProxy 失败[%s]: %s" % (name, exc)) from exc

    def wait_for_service(self, name: str, timeout: Optional[float] = None) -> bool:
        wait_timeout = self._timeout(timeout, self.timeouts.service, "服务")
        try:
            self.rospy.wait_for_service(str(name), timeout=wait_timeout)
            return True
        except Exception as exc:
            raise RosReadinessError(
                "ROS 服务未在 %.2fs 内就绪[%s]: %s" % (wait_timeout, name, exc)
            ) from exc

    def wait_for_message(
        self,
        topic: str,
        message_type: Any,
        timeout: Optional[float] = None,
    ) -> Any:
        wait_timeout = self._timeout(timeout, self.timeouts.topic, "话题")
        try:
            return self.rospy.wait_for_message(
                str(topic), message_type, timeout=wait_timeout
            )
        except Exception as exc:
            raise RosReadinessError(
                "ROS 话题未在 %.2fs 内收到消息[%s]: %s" % (wait_timeout, topic, exc)
            ) from exc

    def wait_for_publisher(
        self,
        publisher: Any,
        label: str,
        timeout: Optional[float] = None,
    ) -> bool:
        """等待 Publisher 至少出现一个订阅者，使用单调时钟避免仿真时钟卡死。"""

        wait_timeout = self._timeout(timeout, self.timeouts.publisher, "Publisher")
        deadline = time.monotonic() + wait_timeout
        while not self.is_shutdown() and time.monotonic() < deadline:
            try:
                if int(publisher.get_num_connections()) > 0:
                    return True
            except Exception as exc:
                raise RosOperationError("读取 Publisher 连接数失败[%s]: %s" % (label, exc)) from exc
            # readiness 使用墙钟休眠；若 /use_sim_time 已开启但 /clock 停止，
            # rospy.sleep 可能永远不返回，从而破坏上面的 monotonic 超时。
            time.sleep(0.05)
        if self.is_shutdown():
            raise RosReadinessError("ROS 已关闭，Publisher 等待中断[%s]" % label)
        raise RosReadinessError(
            "Publisher 未在 %.2fs 内连接订阅者[%s]" % (wait_timeout, label)
        )

    def is_shutdown(self) -> bool:
        return bool(self.rospy.is_shutdown())

    def sleep(self, seconds: float) -> None:
        self.rospy.sleep(max(0.0, float(seconds)))

    def rate(self, hz: float) -> Any:
        if float(hz) <= 0.0:
            raise ValueError("Rate 必须大于 0，当前=%s" % hz)
        return self.rospy.Rate(float(hz))

    def now(self) -> Any:
        return self.rospy.Time.now()

    def logdebug(self, message: str, *args: Any) -> None:
        self.rospy.logdebug(message, *args)

    def loginfo(self, message: str, *args: Any) -> None:
        self.rospy.loginfo(message, *args)

    def logwarn(self, message: str, *args: Any) -> None:
        self.rospy.logwarn(message, *args)

    def logerr(self, message: str, *args: Any) -> None:
        self.rospy.logerr(message, *args)

    # ------------------------------------------------------------------
    # 消息、订阅与 IK 服务工厂。
    # ------------------------------------------------------------------
    def subscribe_tags(
        self, callback: Callable[[Any], None], topic: Optional[str] = None
    ) -> Any:
        return self.subscribe(
            topic or self.topics.qr_detection,
            self.AprilTagDetectionArray,
            callback,
            queue_size=10,
        )

    def subscribe_sensors(self, callback: Callable[[Any], None]) -> Any:
        return self.subscribe(
            self.topics.sensors, self.sensorsData, callback, queue_size=10
        )

    def subscribe_detections(
        self,
        callback: Callable[[Any], None],
        topic: str,
    ) -> Any:
        """订阅 YOLO 检测；消息类型在首次使用时加载并缓存。"""

        return self.subscribe(
            str(topic),
            self._detection_message_type(),
            callback,
            queue_size=10,
        )

    def start_ik_service(
        self,
        service_name: Optional[str] = None,
        timeout: Optional[float] = None,
    ) -> Any:
        """连接唯一的 multi-reference 双臂 IK 服务。"""

        name = str(service_name or self.services.arm_ik)
        with self._service_lock:
            if self.ik_srv is not None and self.ik_service_name == name:
                return self.ik_srv
            self.loginfo("等待双臂 IK 服务 %s", name)
            self.wait_for_service(name, timeout=timeout)
            self.ik_srv = self.service_proxy(name, self.twoArmHandPoseCmdSrv)
            self.ik_service_name = name
            return self.ik_srv

    def call_ik_request(self, request: Any, timeout: Optional[float] = None) -> Any:
        """发送 IK 请求；首次调用时在带超时的 readiness 检查后建立连接。"""

        client = self.start_ik_service(timeout=timeout)
        with self._ik_lock:
            if self.is_shutdown():
                raise RosOperationError("ROS 已关闭，无法调用双臂 IK")
            try:
                return client(request)
            except Exception as exc:
                raise RosOperationError(
                    "双臂 IK 服务调用失败[%s]: %s" % (self.ik_service_name, exc)
                ) from exc

    def make_ik_request(self) -> Any:
        return self.twoArmHandPoseCmdSrvRequest()

    def make_ik_command(self) -> Any:
        return self.twoArmHandPoseCmd()

    def make_joint_state(self) -> Any:
        return self.JointState()

    def make_wheel_ik_param(self) -> Any:
        raise RobotControlError("当前运行时不支持轮臂 IK 参数")

    # ------------------------------------------------------------------
    # 常用机器人动作。
    # ------------------------------------------------------------------
    def publish_head(self, yaw_deg: float, pitch_deg: float) -> None:
        msg = self.robotHeadMotionData()
        yaw = max(-30.0, min(30.0, float(yaw_deg)))
        pitch = max(-25.0, min(25.0, float(pitch_deg)))
        msg.joint_data = [yaw, pitch]
        try:
            connections = int(self.head_pub.get_num_connections())
        except Exception:
            connections = 0
        if connections <= 0:
            # Publisher 刚创建时与头部控制节点的 TCP 连接尚未建立，
            # 此时立即发布会被静默丢弃（历史 head_move.py 靠等待连接规避）。
            try:
                self.wait_for_publisher(self.head_pub, self.topics.head, timeout=2.0)
            except Exception as exc:
                self.logwarn(
                    "头部话题 %s 无订阅者，头部指令可能未送达: %s",
                    self.topics.head,
                    exc,
                )
        self.head_pub.publish(msg)

    def publish_hand_target_pos(
        self,
        left_hand: Sequence[float],
        right_hand: Sequence[float],
        timeout: Optional[float] = None,
    ) -> None:
        """发布双侧灵巧手位置，首次发布前进行有界 Publisher readiness 等待。"""

        with self._end_effector_lock:
            if self._hand_pub is None:
                publisher = self.publisher(
                    DEFAULT_HAND_TOPIC,
                    self._hand_message_type(),
                    queue_size=10,
                )
                self.wait_for_publisher(
                    publisher,
                    DEFAULT_HAND_TOPIC,
                    timeout=timeout,
                )
                # 只有 readiness 成功的 Publisher 才能复用；失败时保持 None，
                # 下一次调用必须重新创建并重新等待。
                self._hand_pub = publisher
            try:
                message = self._hand_message_type()()
                message.left_hand_position = list(left_hand)
                message.right_hand_position = list(right_hand)
                self._hand_pub.publish(message)
            except Exception as exc:
                raise RosOperationError("发布灵巧手位置失败: %s" % exc) from exc

    def call_lejuclaw(
        self,
        positions: Sequence[float],
        velocity: Sequence[float],
        effort: Sequence[float],
        timeout: Optional[float] = None,
        service_name: str = DEFAULT_LEJUCLAW_SERVICE,
    ) -> Any:
        """通过带 readiness 和响应超时的服务调用控制双侧乐聚夹爪。"""

        wait_timeout = self._timeout(timeout, self.timeouts.service, "乐聚夹爪服务")
        service_type, request_type = self._lejuclaw_service_types()
        name = str(service_name)
        with self._end_effector_lock:
            inflight = self._lejuclaw_inflight
            if inflight is not None and not inflight.completed.is_set():
                raise RosOperationError(
                    "上一次乐聚夹爪服务调用仍在执行，不能并发重试；"
                    "服务超时不会取消已发出的 ROS 请求"
                )
            if inflight is not None:
                if (
                    inflight.timed_out.is_set()
                    and self._lejuclaw_client is inflight.client
                ):
                    self._lejuclaw_client = None
                    self._lejuclaw_service_name = None
                self._lejuclaw_inflight = None
            if (
                self._lejuclaw_client is None
                or self._lejuclaw_service_name != name
            ):
                self.wait_for_service(name, timeout=wait_timeout)
                self._lejuclaw_client = self.service_proxy(name, service_type)
                self._lejuclaw_service_name = name
            request = request_type()
            request.data.name = ["left_claw", "right_claw"]
            request.data.position = list(positions)
            request.data.velocity = list(velocity)
            request.data.effort = list(effort)
            client = self._lejuclaw_client
            completed = threading.Event()
            timed_out = threading.Event()
            result: Dict[str, Any] = {}
            call = _LejuClawInFlight(
                completed=completed,
                timed_out=timed_out,
                client=client,
            )
            self._lejuclaw_inflight = call

        def invoke() -> None:
            try:
                result["value"] = client(request)
            except BaseException as exc:
                result["error"] = exc
            finally:
                completed.set()

        worker = threading.Thread(
            target=invoke,
            name="robot-io-lejuclaw-service",
            daemon=True,
        )
        worker.start()
        if not completed.wait(wait_timeout):
            # 先标记 generation，确保在超时清理等待锁期间抢先进入的新调用
            # 也不会复用这一个已经超时的 ServiceProxy。
            timed_out.set()
            self._invalidate_timed_out_lejuclaw_call(call)
            raise RosReadinessError(
                "乐聚夹爪服务调用[%s]响应超时（%.3f 秒）；"
                "已发出的 ROS 请求无法取消" % (name, wait_timeout)
            )

        with self._end_effector_lock:
            if self._lejuclaw_inflight is call:
                self._lejuclaw_inflight = None
        if "error" in result:
            raise RosOperationError(
                "乐聚夹爪服务调用[%s]失败: %s" % (name, result["error"])
            ) from result["error"]
        return result.get("value")

    def _invalidate_timed_out_lejuclaw_call(
        self,
        call: _LejuClawInFlight,
    ) -> bool:
        """只失效仍属于同一 generation 的超时 client。

        worker 可能在 ``Event.wait`` 返回超时后立即完成，随后另一线程创建新的
        generation。identity 守卫确保迟到的超时清理绝不覆盖新 event/client。
        """

        with self._end_effector_lock:
            if self._lejuclaw_inflight is not call:
                return False
            if self._lejuclaw_client is call.client:
                self._lejuclaw_client = None
                self._lejuclaw_service_name = None
            # 保留当前 generation，直到旧 worker 完成前拒绝重试。
            self._lejuclaw_inflight = call
            return True

    def publish_cmd_vel(self, vx: float, vy: float, vz: float) -> None:
        msg = self.Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0
        msg.angular.z = float(vz)
        self.cmd_vel_pub.publish(msg)

    def get_robot_pose(
        self, timeout: Optional[float] = None
    ) -> Tuple[float, float, float]:
        wait_timeout = self._timeout(timeout, self.timeouts.transform, "TF")
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                self.rospy.Time(0),
                self.rospy.Duration(wait_timeout),
            )
        except Exception as exc:
            raise RosReadinessError(
                "无法在 %.2fs 内获取 odom->base_link: %s" % (wait_timeout, exc)
            ) from exc
        translation = transform.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            yaw_from_quat(transform.transform.rotation),
        )

    def transform_qr_for_walk(self, qr: Mapping[str, Any]) -> Dict[str, Any]:
        return dict(qr)

    def publish_stance(self) -> None:
        self.loginfo("当前机器人类型不需要 stance 消息")

    def set_arm_external_control(self, timeout: Optional[float] = None) -> bool:
        return True

    def set_arm_default_control(self, timeout: Optional[float] = None) -> bool:
        return True

    def enable_base_pitch_limit(
        self, enable: bool = True, timeout: Optional[float] = None
    ) -> Tuple[bool, str]:
        return True, "no-op"

    @staticmethod
    def _timeout(value: Optional[float], default: float, label: str) -> float:
        return _positive_timeout(default if value is None else value, label)

    def _detection_message_type(self) -> Any:
        message_type = self.bindings.Detection2DArray
        if message_type is None:
            try:
                from vision_msgs.msg import Detection2DArray
            except ImportError as exc:
                raise RosDependencyError(
                    "无法加载 vision_msgs.msg.Detection2DArray"
                ) from exc
            self.bindings.Detection2DArray = Detection2DArray
            message_type = Detection2DArray
        return message_type

    def _hand_message_type(self) -> Any:
        message_type = self.bindings.robotHandPosition
        if message_type is None:
            try:
                from kuavo_msgs.msg import robotHandPosition
            except ImportError as exc:
                raise RosDependencyError(
                    "无法加载 kuavo_msgs.msg.robotHandPosition"
                ) from exc
            self.bindings.robotHandPosition = robotHandPosition
            message_type = robotHandPosition
        return message_type

    def _lejuclaw_service_types(self) -> Tuple[Any, Any]:
        service_type = self.bindings.controlLejuClaw
        request_type = self.bindings.controlLejuClawRequest
        if service_type is None or request_type is None:
            try:
                from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest
            except ImportError as exc:
                raise RosDependencyError(
                    "无法加载 kuavo_msgs 乐聚夹爪服务类型"
                ) from exc
            self.bindings.controlLejuClaw = controlLejuClaw
            self.bindings.controlLejuClawRequest = controlLejuClawRequest
            service_type = controlLejuClaw
            request_type = controlLejuClawRequest
        return service_type, request_type

class HumanoidRobotIO(RobotIO):
    """人形机器人 ROS 实现，包含步态和手臂模式服务。"""

    robot_type = "humanoid"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        bindings = self.bindings
        required = {
            "switchGaitByName": bindings.switchGaitByName,
            "changeArmCtrlMode": bindings.changeArmCtrlMode,
            "changeArmCtrlModeRequest": bindings.changeArmCtrlModeRequest,
            "SetBool": bindings.SetBool,
            "SetBoolRequest": bindings.SetBoolRequest,
        }
        missing = [name for name, value in required.items() if value is None]
        if missing:
            raise RosDependencyError("人形 ROS bindings 不完整: %s" % ", ".join(missing))

        self.gait_pub = self.publisher(
            self.topics.gait, bindings.switchGaitByName, queue_size=10
        )
        self._arm_mode_client = None

    def publish_stance(self) -> None:
        msg = self.bindings.switchGaitByName()
        msg.header.stamp = self.now()
        msg.gait_name = "stance"
        self.gait_pub.publish(msg)

    def set_arm_control_mode(
        self,
        mode: int,
        label: str,
        timeout: Optional[float] = None,
    ) -> bool:
        with self._service_lock:
            if self._arm_mode_client is None:
                self.wait_for_service(self.services.arm_mode, timeout=timeout)
                self._arm_mode_client = self.service_proxy(
                    self.services.arm_mode, self.bindings.changeArmCtrlMode
                )
            request = self.bindings.changeArmCtrlModeRequest()
            request.control_mode = int(mode)
            try:
                response = self._arm_mode_client(request)
            except Exception as exc:
                raise RosOperationError("%s调用失败: %s" % (label, exc)) from exc
        if not bool(getattr(response, "result", False)):
            raise RosOperationError(
                "%s失败(mode=%d): %s"
                % (label, int(mode), getattr(response, "message", "无返回说明"))
            )
        self.loginfo(
            "%s成功(mode=%d): %s",
            label,
            int(mode),
            getattr(response, "message", ""),
        )
        return True

    def set_arm_external_control(self, timeout: Optional[float] = None) -> bool:
        return self.set_arm_control_mode(2, "切换手臂外部控制模式", timeout)

    def set_arm_default_control(self, timeout: Optional[float] = None) -> bool:
        return self.set_arm_control_mode(1, "恢复手臂默认控制模式", timeout)

    def enable_base_pitch_limit(
        self, enable: bool = True, timeout: Optional[float] = None
    ) -> Tuple[bool, str]:
        self.wait_for_service(self.services.base_pitch_limit, timeout=timeout)
        client = self.service_proxy(self.services.base_pitch_limit, self.bindings.SetBool)
        request = self.bindings.SetBoolRequest()
        request.data = bool(enable)
        try:
            response = client(request)
        except Exception as exc:
            raise RosOperationError("设置躯干俯仰限制失败: %s" % exc) from exc
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", ""))
        if not success:
            raise RosOperationError("设置躯干俯仰限制失败: %s" % (message or "服务拒绝"))
        return True, message or "success"


class WheelRobotIO(RobotIO):
    """轮臂机器人 ROS 实现，提供轮臂专用 IK 参数和二维码坐标变换。"""

    robot_type = "wheel"
    qr_detection_frame = "waist_yaw_link"
    walk_qr_frame = "base_link"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        if self.bindings.ikSolveParam is None:
            raise RosDependencyError("轮臂 ROS bindings 缺少 ikSolveParam")
        if self.bindings.Float32 is None:
            raise RosDependencyError("轮臂 ROS bindings 缺少 std_msgs/Float32")
        self.Float32 = self.bindings.Float32
        self.lb_torso_pose_pub = self.publisher(
            self.topics.lb_torso_pose,
            self.Twist,
            queue_size=10,
        )

    def get_torso_open_loop_pose(
        self,
        timeout: Optional[float] = None,
    ) -> Tuple[float, float, float, float]:
        """读取轮臂躯干开环位姿 ``(x, z, yaw, pitch)``。"""

        msg = self.wait_for_message(
            self.topics.torso_open_loop_state,
            self.Twist,
            timeout=timeout,
        )
        pose = (
            float(msg.linear.x),
            float(msg.linear.z),
            float(msg.angular.z),
            float(msg.angular.y),
        )
        if not all(math.isfinite(value) for value in pose):
            raise RosOperationError("躯干开环状态包含非有限数: %s" % (pose,))
        return pose

    def publish_torso_pose(
        self,
        x: float,
        z: float,
        yaw: float = 0.0,
        pitch: float = 0.0,
    ) -> None:
        """发布一帧轮臂躯干绝对位姿。

        轮臂没有躯干 y 和 roll 自由度，因此对应字段固定为零。本接口只负责
        构造并发布 ROS 消息；插值时序由 ``WheelMotionController`` 管理。
        """

        msg = self.Twist()
        msg.linear.x = float(x)
        msg.linear.y = 0.0
        msg.linear.z = float(z)
        msg.angular.x = 0.0
        msg.angular.y = float(pitch)
        msg.angular.z = float(yaw)
        self.lb_torso_pose_pub.publish(msg)

    def publish_torso_delta(self, dx: float, dz: float) -> None:
        """兼容旧调用：将单次相对偏移转换成绝对位姿后发布。"""

        x, z, yaw, pitch = self.get_torso_open_loop_pose()
        self.publish_torso_pose(x + float(dx), z + float(dz), yaw, pitch)

    def wait_for_torso_reach_time(self, timeout: float = 1.0) -> float:
        """等待驱动返回预计到达时间，单位为秒。"""

        msg = self.wait_for_message(
            self.topics.lb_torso_pose_reach_time,
            self.Float32,
            timeout=timeout,
        )
        reach_time = float(msg.data)
        if not math.isfinite(reach_time):
            raise RosOperationError("躯干到达时间不是有限数: %s" % reach_time)
        return max(0.0, reach_time)

    def set_arm_external_control(self, timeout: Optional[float] = None) -> bool:
        self.loginfo("轮臂无需切换手臂外部控制模式")
        return True

    def set_arm_default_control(self, timeout: Optional[float] = None) -> bool:
        self.loginfo("轮臂无需恢复手臂默认控制模式")
        return True

    def enable_base_pitch_limit(
        self, enable: bool = True, timeout: Optional[float] = None
    ) -> Tuple[bool, str]:
        self.loginfo("轮臂无躯干俯仰限制接口，跳过 enable=%s", bool(enable))
        return True, "wheel no-op"

    def make_wheel_ik_param(self) -> Any:
        config = _mapping(_mapping(self.params, "wheel"), "ik")
        param = self.bindings.ikSolveParam()
        param.major_optimality_tol = float(config.get("major_optimality_tol", 4e-3))
        param.major_feasibility_tol = float(config.get("major_feasibility_tol", 4e-3))
        param.minor_feasibility_tol = float(config.get("minor_feasibility_tol", 4e-3))
        param.major_iterations_limit = int(config.get("major_iterations_limit", 100))
        param.oritation_constraint_tol = float(
            config.get("oritation_constraint_tol", 4e-3)
        )
        param.pos_constraint_tol = float(config.get("pos_constraint_tol", 4e-3))
        param.pos_cost_weight = float(config.get("pos_cost_weight", 1.0))
        param.constraint_mode = int(config.get("constraint_mode", 1))
        return param

    def transform_qr_for_walk(self, qr: Mapping[str, Any]) -> Dict[str, Any]:
        timeout = self.timeouts.transform
        try:
            transform = self.tf_buffer.lookup_transform(
                self.walk_qr_frame,
                self.qr_detection_frame,
                self.rospy.Time(0),
                self.rospy.Duration(timeout),
            )
        except Exception as exc:
            raise RosReadinessError(
                "无法在 %.2fs 内将二维码从 %s 转到 %s: %s"
                % (timeout, self.qr_detection_frame, self.walk_qr_frame, exc)
            ) from exc

        trans = transform.transform.translation
        rot = transform.transform.rotation
        tf_quat = quat_xyzw(rot)
        point = [float(qr["x"]), float(qr["y"]), float(qr["z"])]
        rotated = rotate_vector_by_quat(point, tf_quat)
        result = dict(qr)
        result["x"] = rotated[0] + float(trans.x)
        result["y"] = rotated[1] + float(trans.y)
        result["z"] = rotated[2] + float(trans.z)
        result["yaw"] = normalize_angle(
            float(qr.get("yaw", 0.0)) + yaw_from_quat(rot)
        )
        if "quat" in qr:
            result["quat"] = quaternion_multiply(tf_quat, qr["quat"])
        return result


def build_robot_io(
    robot_type: str,
    params: Optional[Mapping[str, Any]] = None,
    **kwargs: Any
) -> RobotIO:
    """根据配置构建具体运行时，避免业务层出现类型分支。"""

    normalized = _normalize_robot_type(robot_type)
    if normalized == "humanoid":
        return HumanoidRobotIO(params=params, **kwargs)
    if normalized == "wheel":
        return WheelRobotIO(params=params, **kwargs)
    raise ValueError("不支持的机器人类型: %s" % robot_type)


def _normalize_robot_type(value: Any) -> str:
    normalized = str(value or "humanoid").strip().lower()
    if normalized in ("humanoid", "biped", "kuavo"):
        return "humanoid"
    if normalized in ("wheel", "wheeled", "mobile"):
        return "wheel"
    raise ValueError("不支持的机器人类型: %s" % value)


def _mapping(value: Any, key: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        return {}
    child = value.get(key, {})
    return child if isinstance(child, Mapping) else {}


def _setting(
    params: Mapping[str, Any],
    *paths: Sequence[str],
    **kwargs: Any
) -> Any:
    """同时兼容新 ``robot.*`` 配置与旧扁平配置。"""

    default = kwargs.get("default")
    roots = [params]
    robot = _mapping(params, "robot")
    if robot:
        roots.insert(0, robot)
    for path in paths:
        for root in roots:
            current: Any = root
            found = True
            for key in path:
                if not isinstance(current, Mapping) or key not in current:
                    found = False
                    break
                current = current[key]
            if found and current is not None:
                return current
    return default


def _positive_timeout(value: Any, label: str) -> float:
    try:
        timeout = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("%s 超时必须是数字，当前=%r" % (label, value)) from exc
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("%s 超时必须为有限正数，当前=%r" % (label, value))
    return timeout


__all__ = [
    "DEFAULT_IK_SERVICE",
    "HumanoidRobotIO",
    "RobotControlError",
    "RosBindings",
    "RosDependencyError",
    "RosOperationError",
    "RosReadinessError",
    "RobotIO",
    "RosServices",
    "RosTimeouts",
    "RosTopics",
    "WheelRobotIO",
    "build_robot_io",
    "load_ros_bindings",
]
