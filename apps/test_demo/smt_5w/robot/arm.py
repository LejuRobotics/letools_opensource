#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双臂 IK、关节状态与轨迹执行控制器。

该模块不直接导入 ROS，而是通过 :class:`~supermarket.robot.robot_io.RobotIO`
访问消息、话题与服务。因此它可以在无 ROS 环境中导入，也便于用 fake RobotIO 做
单元测试。抓取、放置和收臂应共享同一个 :class:`ArmController` 实例，避免多套
软件关节状态相互覆盖。
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .robot_io import RobotControlError, RobotIO, RosReadinessError


class ArmControlError(RobotControlError):
    """手臂控制统一异常。"""


class ArmReadinessError(ArmControlError):
    """手臂轨迹订阅者、关节反馈或 IK 服务未就绪。"""


class IkSolveError(ArmControlError):
    """IK 请求非法、求解失败或返回格式异常。"""


class TrajectoryExecutionError(ArmControlError):
    """关节轨迹无法安全执行。"""


@dataclass(frozen=True)
class IkSolution:
    """一次 IK 的标准化结果。

    ``trajectory_joints_deg`` 用于 ``/kuavo_arm_traj``，``seed_joints_rad``
    用于下一段链式 IK 的 q0。两者单位不同，使用独立字段可避免隐式混用。
    """

    trajectory_joints_deg: Tuple[float, ...]
    seed_joints_rad: Tuple[float, ...]
    time_cost: float
    label: str
    raw_response: Any = None


class ArmController:
    """原生 ROS 双臂控制器。

    默认安全策略：

    * 插值起点优先使用新鲜的 ``/sensors_data_raw`` 真实关节反馈；
    * 无反馈时可回退到上一次已发布轨迹，但会记录警告；
    * 默认不允许凭空假设复位姿态，确需兼容旧行为时可在配置中设置
      ``arm.allow_default_start: true``；
    * 两段及多段 IK 自动将上一段结果作为下一段 q0。
    """

    JOINT_COUNT = 14

    def __init__(
        self,
        robot_io: RobotIO,
        params: Optional[Mapping[str, Any]] = None,
        auto_subscribe: bool = True,
    ) -> None:
        self.robot_io = robot_io
        self.params: Dict[str, Any] = dict(params or robot_io.params or {})
        self.arm_params = _mapping(self.params, "arm")

        self.last_arm_joints: Optional[List[float]] = None
        self.current_arm_joints_deg: Optional[List[float]] = None
        self.current_arm_joints_stamp: Optional[float] = None
        self.sensors_sub = None

        self.sensor_topic = str(
            self.arm_params.get("sensor_topic", robot_io.topics.sensors)
        )
        self.sensor_timeout = _positive_float(
            self.arm_params.get("sensor_timeout", robot_io.timeouts.topic),
            "arm.sensor_timeout",
        )
        self.sensor_max_age = _positive_float(
            self.arm_params.get("sensor_max_age", 0.5), "arm.sensor_max_age"
        )
        self.publisher_timeout = _positive_float(
            self.arm_params.get("publisher_timeout", robot_io.timeouts.publisher),
            "arm.publisher_timeout",
        )
        self.allow_cached_start = _as_bool(
            self.arm_params.get("allow_cached_start", True)
        )
        self.allow_default_start = _as_bool(
            self.arm_params.get("allow_default_start", False)
        )
        self.use_sensor_q0 = _as_bool(
            self.arm_params.get("use_sensor_q0", True)
        )
        self.sensor_q0_timeout = _positive_float(
            self.arm_params.get("sensor_q0_timeout", self.sensor_timeout),
            "arm.sensor_q0_timeout",
        )
        configured_delta = self.arm_params.get("max_joint_step_deg")
        self.max_joint_step_deg = (
            None
            if configured_delta is None
            else _positive_float(configured_delta, "arm.max_joint_step_deg")
        )

        # /sensors_data_raw 的 joint_data.joint_q 布局：
        # 人形(约28+)：
        #   [0:12] 双腿，[12] 腰 yaw，[13:20] 左臂7，[20:27] 右臂7，[27:29] 头
        # 轮臂(20，kuavo_v63 / LUNBI)：
        #   [0:4] 下肢/腰，[4:11] 左臂7，[11:18] 右臂7，[18:20] 头
        # 因此轮臂双臂默认从下标 4 开始，人形从 13 开始。
        indices = self.arm_params.get("sensor_joint_indices")
        if indices is None:
            default_start = 4 if self.robot_io.robot_type == "wheel" else 13
            start = int(self.arm_params.get("sensor_joint_start", default_start))
            self.sensor_joint_indices = tuple(range(start, start + self.JOINT_COUNT))
        else:
            parsed = tuple(int(value) for value in list(indices))
            if len(parsed) != self.JOINT_COUNT or min(parsed) < 0:
                raise ValueError("arm.sensor_joint_indices 必须包含14个非负索引")
            self.sensor_joint_indices = parsed
        self.robot_io.loginfo(
            "手臂传感器关节索引[%s]: %s (需要 joint_q 长度>=%d)",
            self.robot_io.robot_type,
            list(self.sensor_joint_indices),
            max(self.sensor_joint_indices) + 1,
        )

        self._state_lock = threading.RLock()
        self._trajectory_lock = threading.RLock()
        self._arm_publisher_ready = False
        if auto_subscribe:
            self.sensors_sub = self.robot_io.subscribe_sensors(self.sensors_callback)

    # ------------------------------------------------------------------
    # 真实关节状态。
    # ------------------------------------------------------------------
    def sensors_callback(self, message: Any) -> None:
        """解析传感器关节弧度，并缓存为轨迹接口使用的角度值。

        ``/sensors_data_raw`` 的消息类型是 ``kuavo_msgs/sensorsData``，关节角在
        ``joint_data.joint_q``（``float64[]``，弧度）中，不是 ``JointState.position``。
        """

        positions = _extract_sensor_joint_positions(message)
        maximum = max(self.sensor_joint_indices)
        if len(positions) <= maximum:
            self.robot_io.logwarn(
                "忽略长度不足的关节反馈: 需要索引%d，实际长度=%d",
                maximum,
                len(positions),
            )
            return
        try:
            joints = [
                math.degrees(float(positions[index]))
                for index in self.sensor_joint_indices
            ]
            _validate_joints(joints, "传感器手臂关节")
        except (TypeError, ValueError) as exc:
            self.robot_io.logwarn("忽略非法关节反馈: %s", exc)
            return
        with self._state_lock:
            self.current_arm_joints_deg = joints
            self.current_arm_joints_stamp = time.monotonic()

    def get_current_arm_joints_deg(
        self,
        timeout: Optional[float] = None,
        max_age: Optional[float] = None,
    ) -> Optional[List[float]]:
        """返回新鲜的真实关节角；超时仅返回 ``None`` 并记录原因。

        需要强制成功的调用方应使用 :meth:`require_current_arm_joints_deg`。
        """

        allowed_age = self.sensor_max_age if max_age is None else _positive_float(
            max_age, "关节反馈最大年龄"
        )
        cached = self._fresh_sensor_joints(allowed_age)
        if cached is not None:
            return cached

        wait_timeout = self.sensor_timeout if timeout is None else _positive_float(
            timeout, "关节反馈等待"
        )
        try:
            message = self.robot_io.wait_for_message(
                self.sensor_topic,
                self.robot_io.sensorsData,
                timeout=wait_timeout,
            )
            self.sensors_callback(message)
        except RosReadinessError as exc:
            self.robot_io.logwarn("获取当前手臂关节失败: %s", exc)
            return None
        return self._fresh_sensor_joints(allowed_age)

    def require_current_arm_joints_deg(
        self, timeout: Optional[float] = None
    ) -> List[float]:
        joints = self.get_current_arm_joints_deg(timeout=timeout)
        if joints is None:
            raise ArmReadinessError(
                "未在限定时间内获得14维真实手臂关节反馈[%s]" % self.sensor_topic
            )
        return joints

    def _fresh_sensor_joints(self, max_age: float) -> Optional[List[float]]:
        with self._state_lock:
            if (
                self.current_arm_joints_deg is None
                or self.current_arm_joints_stamp is None
            ):
                return None
            if time.monotonic() - self.current_arm_joints_stamp > max_age:
                return None
            return list(self.current_arm_joints_deg)

    # ------------------------------------------------------------------
    # IK：所有路径统一走 multi-reference 服务。
    # ------------------------------------------------------------------
    def solve_pose(
        self,
        left_xyz: Sequence[float],
        right_xyz: Sequence[float],
        left_quat: Sequence[float],
        right_quat: Sequence[float],
        label: str,
        q0_joints: Optional[Sequence[float]] = None,
    ) -> IkSolution:
        """统一构造完整双手位姿 IK，并应用当前机器人类型的求解参数。

        位置单位为米，四元数顺序为 ``xyzw``，``q0_joints`` 为14维弧度。
        轮臂会自动从 ``wheel.ik`` 读取 ``frame`` 和自定义 ``ikSolveParam``；
        调用方无需接触 ROS 消息或 RobotIO 的 IK 工厂。
        """

        left = _validate_xyz(left_xyz, "左手IK目标")
        right = _validate_xyz(right_xyz, "右手IK目标")
        left_orientation = _validate_quaternion(left_quat, "左手IK姿态")
        right_orientation = _validate_quaternion(right_quat, "右手IK姿态")
        seed = self._resolve_ik_seed(q0_joints)

        is_wheel = self.robot_io.robot_type == "wheel"
        if is_wheel:
            wheel_ik = _mapping(_mapping(self.params, "wheel"), "ik")
            frame = int(
                wheel_ik.get("frame", self.arm_params.get("wheel_ik_frame", 0))
            )
        else:
            frame = int(self.arm_params.get("humanoid_ik_frame", 2))

        command = self._build_ik_command(
            left,
            right,
            left_orientation,
            right_orientation,
            frame,
            seed,
        )
        command.use_custom_ik_param = is_wheel
        if is_wheel:
            command.ik_param = self.robot_io.make_wheel_ik_param()

        left_pose = command.hand_poses.left_pose
        right_pose = command.hand_poses.right_pose
        ik_param = _ik_param_summary(command.ik_param) if is_wheel else None
        self.robot_io.loginfo(
            "IK输入[%s]: frame=%d custom=%s ik_param=%s",
            label,
            command.frame,
            command.use_custom_ik_param,
            ik_param,
        )
        self.robot_io.loginfo(
            "IK输入[%s]: left(xyz=%s quat=%s elbow=%s q0=%s) "
            "right(xyz=%s quat=%s elbow=%s q0=%s)",
            label,
            _rounded(left_pose.pos_xyz),
            _rounded(left_pose.quat_xyzw),
            _rounded(left_pose.elbow_pos_xyz),
            _rounded(getattr(left_pose, "joint_angles", [])),
            _rounded(right_pose.pos_xyz),
            _rounded(right_pose.quat_xyzw),
            _rounded(right_pose.elbow_pos_xyz),
            _rounded(getattr(right_pose, "joint_angles", [])),
        )

        request = self.robot_io.make_ik_request()
        request.twoArmHandPoseCmdRequest = command
        response = self.robot_io.call_ik_request(
            request, timeout=self.robot_io.timeouts.service
        )
        if not bool(getattr(response, "success", False)):
            reason = getattr(response, "error_reason", "") or "服务未说明原因"
            self.robot_io.logerr("IK输出[%s]: success=False reason=%s", label, reason)
            raise IkSolveError("IK求解失败[%s]: %s" % (label, reason))
        solution = self._solution_from_response(response, label)
        self.robot_io.loginfo(
            "IK输出[%s]: success=True time_cost=%.6fs seed_rad=%s trajectory_deg=%s",
            label,
            solution.time_cost,
            _rounded(solution.seed_joints_rad),
            _rounded(solution.trajectory_joints_deg),
        )
        return solution

    def _resolve_ik_seed(
        self, q0_joints: Optional[Sequence[float]]
    ) -> Optional[List[float]]:
        if q0_joints is not None:
            return _validate_joints(q0_joints, "IK q0")
        if not self.use_sensor_q0:
            return None

        # 外部轨迹发布可能改变真实关节而不更新软件缓存，因此优先读取实际姿态
        # 作为首段 q0。
        sensed = self.get_current_arm_joints_deg(timeout=self.sensor_q0_timeout)
        if sensed is None:
            return None
        return self.convert_traj_joints_for_ik_seed(sensed)

    def _build_ik_command(
        self,
        left_xyz: Sequence[float],
        right_xyz: Sequence[float],
        left_quat: Sequence[float],
        right_quat: Sequence[float],
        frame: int,
        q0_joints: Optional[Sequence[float]],
    ) -> Any:
        command = self.robot_io.make_ik_command()
        command.frame = int(frame)
        command.joint_angles_as_q0 = False
        left_elbow, right_elbow = self.ik_elbow_points()

        command.hand_poses.left_pose.pos_xyz = list(left_xyz)
        command.hand_poses.left_pose.quat_xyzw = list(left_quat)
        command.hand_poses.left_pose.elbow_pos_xyz = left_elbow
        command.hand_poses.right_pose.pos_xyz = list(right_xyz)
        command.hand_poses.right_pose.quat_xyzw = list(right_quat)
        command.hand_poses.right_pose.elbow_pos_xyz = right_elbow

        if q0_joints is not None:
            seed = _validate_joints(q0_joints, "IK q0")
            command.joint_angles_as_q0 = True
            command.hand_poses.left_pose.joint_angles = seed[:7]
            command.hand_poses.right_pose.joint_angles = seed[7:]
        return command

    def _solution_from_response(self, response: Any, label: str) -> IkSolution:
        raw: Optional[List[float]] = None
        trajectory: Optional[List[float]] = None
        # 轮臂服务常把最终解写在 hand_poses；人形通常返回 q_arm。
        if self.robot_io.robot_type == "wheel":
            trajectory = self.response_hand_pose_joints(response)
            if trajectory is not None:
                raw = self.convert_traj_joints_for_ik_seed(trajectory)
        if trajectory is None:
            candidate = list(getattr(response, "q_arm", []) or [])
            if len(candidate) == self.JOINT_COUNT:
                raw = _validate_joints(candidate, "IK q_arm")
                trajectory = self.convert_ik_q_arm_for_traj(raw)
        if trajectory is None or raw is None:
            raise IkSolveError("IK返回关节数异常[%s]，需要14维" % label)

        solution = IkSolution(
            trajectory_joints_deg=tuple(trajectory),
            seed_joints_rad=tuple(raw),
            time_cost=float(getattr(response, "time_cost", 0.0)),
            label=str(label),
            raw_response=response,
        )
        return solution

    def ik_elbow_points(self) -> Tuple[List[float], List[float]]:
        left = _configured_vector(
            self.arm_params, "ik_left_elbow_pos_xyz", [0.0, 0.0, 0.0], 3
        )
        right = _configured_vector(
            self.arm_params, "ik_right_elbow_pos_xyz", [0.0, 0.0, 0.0], 3
        )
        return left, right

    # ------------------------------------------------------------------
    # 关节轨迹执行。
    # ------------------------------------------------------------------
    def move_joints_interpolated(
        self,
        target_joints: Sequence[float],
        duration: float = 4.0,
        steps: int = 20,
        start_joints: Optional[Sequence[float]] = None,
    ) -> List[float]:
        """线性插值执行双臂轨迹，默认从真实关节反馈开始。"""

        target = _validate_joints(target_joints, "手臂目标关节")
        seconds = _positive_float(duration, "手臂轨迹时长")
        count = max(1, int(steps))
        start = self._resolve_start_joints(start_joints)
        self.validate_joint_delta(start, target, self.max_joint_step_deg)
        self._ensure_arm_publisher_ready(self.publisher_timeout)

        hz = max(1.0, float(count) / max(seconds, 0.1))
        rate = self.robot_io.rate(hz)
        names = ["arm_joint_%d" % index for index in range(1, 15)]
        with self._trajectory_lock:
            for step in range(1, count + 1):
                if self.robot_io.is_shutdown():
                    raise TrajectoryExecutionError("ROS 已关闭，手臂插值中断")
                alpha = float(step) / float(count)
                joints = [
                    source + (destination - source) * alpha
                    for source, destination in zip(start, target)
                ]
                message = self.robot_io.make_joint_state()
                message.header.stamp = self.robot_io.now()
                message.name = names
                message.position = joints
                try:
                    self.robot_io.arm_traj_pub.publish(message)
                except Exception as exc:
                    raise TrajectoryExecutionError("发布手臂轨迹失败: %s" % exc) from exc
                self.last_arm_joints = list(joints)
                rate.sleep()
        settle = max(0.0, float(self.arm_params.get("settle_time", 0.5)))
        if settle:
            self.robot_io.sleep(settle)
        return list(target)

    def _resolve_start_joints(
        self, explicit: Optional[Sequence[float]]
    ) -> List[float]:
        if explicit is not None:
            return _validate_joints(explicit, "插值显式起始关节")

        # 即使存在软件缓存，也先主动读取真实关节反馈。
        sensed = self.get_current_arm_joints_deg(timeout=self.sensor_timeout)
        if sensed is not None:
            return sensed
        if self.allow_cached_start and self.last_arm_joints is not None:
            self.robot_io.logwarn("真实关节反馈不可用，回退到上一次已发布轨迹终点")
            return _validate_joints(self.last_arm_joints, "缓存起始关节")
        if self.allow_default_start:
            self.robot_io.logwarn("真实及缓存关节均不可用，按配置回退到默认复位姿态")
            return self.default_reset_arm_deg()
        raise ArmReadinessError(
            "无法确定手臂插值起点：真实关节反馈不可用，且未允许默认姿态回退"
        )

    def _ensure_arm_publisher_ready(self, timeout: float) -> None:
        if self._arm_publisher_ready:
            return
        self.robot_io.wait_for_publisher(
            self.robot_io.arm_traj_pub,
            self.robot_io.topics.arm_trajectory,
            timeout=timeout,
        )
        self._arm_publisher_ready = True

    @staticmethod
    def validate_joint_delta(
        start_joints: Sequence[float],
        target_joints: Sequence[float],
        max_delta_deg: Optional[float],
    ) -> float:
        start = _validate_joints(start_joints, "跳变检查起点")
        target = _validate_joints(target_joints, "跳变检查目标")
        maximum = max(abs(end - begin) for begin, end in zip(start, target))
        if max_delta_deg is not None and maximum > float(max_delta_deg):
            raise TrajectoryExecutionError(
                "关节目标跳变过大: 最大=%.2f°，限制=%.2f°"
                % (maximum, float(max_delta_deg))
            )
        return maximum

    # ------------------------------------------------------------------
    # 常用安全动作及兼容工具。
    # ------------------------------------------------------------------
    def enter_external_mode(self, timeout: Optional[float] = None) -> bool:
        return self.robot_io.set_arm_external_control(timeout=timeout)

    def restore_default_mode(self, timeout: Optional[float] = None) -> bool:
        return self.robot_io.set_arm_default_control(timeout=timeout)

    def reset(self) -> bool:
        self.robot_io.publish_head(0.0, 0.0)
        self.robot_io.sleep(0.5)
        start = self.get_current_arm_joints_deg(timeout=self.sensor_timeout)
        if start is None and self.allow_cached_start and self.last_arm_joints is not None:
            start = list(self.last_arm_joints)
        if start is None:
            raise ArmReadinessError("复位失败：无法获得当前手臂关节角")
        self.move_joints_interpolated(
            self.default_finish_reset_arm_deg(), 3.0, 60, start
        )
        return True

    @staticmethod
    def default_reset_arm_deg() -> List[float]:
        return [
            20, 0, 0, -30, 0, 0, 0,
            20, 0, 0, -30, 0, 0, 0,
        ]

    @staticmethod
    def default_finish_reset_arm_deg() -> List[float]:
        return [
            20, 10, 0, -30, 0, 0, 0,
            20, -10, 0, -30, 0, 0, 0,
        ]

    @staticmethod
    def convert_ik_q_arm_for_traj(q_arm: Sequence[float]) -> List[float]:
        values = _validate_joints(q_arm, "IK弧度关节")
        return [math.degrees(value) for value in values]

    @staticmethod
    def convert_traj_joints_for_ik_seed(joints: Sequence[float]) -> List[float]:
        values = _validate_joints(joints, "轨迹角度关节")
        return [math.radians(value) for value in values]

    @staticmethod
    def response_hand_pose_joints(response: Any) -> Optional[List[float]]:
        hand_poses = getattr(response, "hand_poses", None)
        if hand_poses is None:
            return None
        left = list(
            getattr(getattr(hand_poses, "left_pose", None), "joint_angles", []) or []
        )
        right = list(
            getattr(getattr(hand_poses, "right_pose", None), "joint_angles", []) or []
        )
        values = left + right
        if len(values) != ArmController.JOINT_COUNT:
            return None
        try:
            radians = _validate_joints(values, "IK hand_poses关节")
        except ValueError:
            return None
        return [math.degrees(value) for value in radians]


def _mapping(value: Any, key: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        return {}
    child = value.get(key, {})
    return child if isinstance(child, Mapping) else {}


def _rounded(values: Sequence[float], digits: int = 5) -> List[float]:
    return [round(float(value), digits) for value in list(values)]


def _ik_param_summary(param: Any) -> Dict[str, Any]:
    fields = (
        "major_optimality_tol",
        "major_feasibility_tol",
        "minor_feasibility_tol",
        "major_iterations_limit",
        "oritation_constraint_tol",
        "pos_constraint_tol",
        "pos_cost_weight",
        "constraint_mode",
    )
    return {name: getattr(param, name, None) for name in fields}


def _extract_sensor_joint_positions(message: Any) -> List[float]:
    """从 ``sensorsData`` 提取 ``joint_data.joint_q`` 列表。"""

    joint_data = getattr(message, "joint_data", None)
    joint_q = getattr(joint_data, "joint_q", None)
    if joint_q is None:
        return []
    try:
        return list(joint_q)
    except TypeError:
        return []


def _as_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _positive_float(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("%s 必须为数字，当前=%r" % (label, value)) from exc
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError("%s 必须为有限正数，当前=%r" % (label, value))
    return number


def _validate_joints(values: Sequence[float], label: str) -> List[float]:
    try:
        joints = [float(value) for value in list(values)]
    except (TypeError, ValueError) as exc:
        raise ValueError("%s必须是数值序列" % label) from exc
    if len(joints) != ArmController.JOINT_COUNT:
        raise ValueError("%s必须为14维，当前=%d" % (label, len(joints)))
    if not all(math.isfinite(value) for value in joints):
        raise ValueError("%s包含非有限值" % label)
    return joints


def _validate_xyz(values: Sequence[float], label: str) -> List[float]:
    try:
        point = [float(value) for value in list(values)]
    except (TypeError, ValueError) as exc:
        raise IkSolveError("%s必须是数值序列" % label) from exc
    if len(point) != 3 or not all(math.isfinite(value) for value in point):
        raise IkSolveError("%s必须为3维有限坐标" % label)
    return point


def _validate_quaternion(values: Sequence[float], label: str) -> List[float]:
    try:
        quaternion = [float(value) for value in list(values)]
    except (TypeError, ValueError) as exc:
        raise IkSolveError("%s必须是数值序列" % label) from exc
    if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
        raise IkSolveError("%s必须为4维有限数值" % label)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 0.0:
        raise IkSolveError("%s不能为零四元数" % label)
    return [value / norm for value in quaternion]


def _configured_vector(
    config: Mapping[str, Any], key: str, default: Sequence[float], length: int
) -> List[float]:
    try:
        values = [float(value) for value in list(config.get(key, default))]
    except (TypeError, ValueError) as exc:
        raise ValueError("arm.%s 必须是数值序列" % key) from exc
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise ValueError("arm.%s 必须为%d维有限数值" % (key, length))
    return values


__all__ = [
    "ArmControlError",
    "ArmController",
    "ArmReadinessError",
    "IkSolution",
    "IkSolveError",
    "TrajectoryExecutionError",
]
