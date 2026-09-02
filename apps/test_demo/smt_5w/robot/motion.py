#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形与轮式机器人的闭环底盘运动控制。

模块采用 RobotIO 注入，不在顶层导入 ``rospy`` 或任何 ROS 消息。运动模块
只调用 RobotIO 的正式控制、TF、休眠、关闭状态和日志接口。

所有面向状态机的动作均明确返回 ``True`` 或 ``False``。闭环超时、停止请求
或下发失败后会尽力连续发送零速度，避免保留上一条非零速度指令。
"""

import math
import time

from .geometry import normalize_angle, rotate_vector_by_quat
from .robot_io import RosReadinessError

WHEEL_TORSO_INITIAL_POSE = (0.196123, 0.0005, 0.789919, 0.0)


class MotionError(RuntimeError):
    """运动请求缺少运行时能力或闭环动作无法完成。"""


class BaseMotionController(object):
    """人形与轮式控制器共享的运行时适配和闭环工具。"""

    TURN_180_YAW_TOLERANCE = math.radians(5.0)

    def __init__(self, robot_io, params=None):
        self.robot_io = robot_io
        robot_io_params = robot_io.params
        self.params = params if params is not None else robot_io_params
        if self.params is None:
            self.params = {}

    def _is_shutdown(self):
        """安全读取停止状态；接口异常时按停止处理。"""

        try:
            return bool(self.robot_io.is_shutdown())
        except Exception:
            return True

    def _sleep(self, seconds):
        """可中断休眠；ROS 停止时返回 False。"""

        if self._is_shutdown():
            return False
        try:
            self.robot_io.sleep(max(0.0, float(seconds)))
        except Exception as exc:
            if self._is_shutdown():
                return False
            raise MotionError("运动控制休眠失败: %s" % exc) from exc
        return not self._is_shutdown()

    def _publish_cmd_vel(self, vx, vy, wz):
        result = self.robot_io.publish_cmd_vel(float(vx), float(vy), float(wz))
        return result is not False

    def _get_robot_pose(self):
        try:
            pose = self.robot_io.get_robot_pose()
        except RosReadinessError as exc:
            # TF 在运动刚启动时可能短暂不可用。闭环调用者会在总超时内重试，
            # 因而这里将 readiness 失败转换为“本周期无位姿”。
            self.robot_io.logwarn("本周期未获取到机器人位姿: %s", exc)
            return None, None, None
        if pose is None or len(pose) < 3:
            return None, None, None
        return pose[0], pose[1], pose[2]

    def _best_effort_stop(self):
        """异常路径使用的停止动作，不覆盖原始异常。"""

        try:
            return bool(self.stop())
        except Exception:
            return False

    def stop(self, duration=0.6, control_dt=0.05):
        """在一小段时间内重复发布零速度。

        成功完成零速度下发返回 True；ROS 已停止、发布失败或休眠被中断时
        返回 False。速度参数与旧实现保持不变。
        """

        control_dt = float(control_dt)
        if control_dt <= 0.0:
            raise ValueError("control_dt 必须大于 0")
        duration = max(float(duration), control_dt)
        deadline = time.monotonic() + duration
        published = False

        while time.monotonic() < deadline:
            if self._is_shutdown():
                return False
            if not self._publish_cmd_vel(0.0, 0.0, 0.0):
                return False
            published = True
            remaining = deadline - time.monotonic()
            if remaining > 0.0 and not self._sleep(min(control_dt, remaining)):
                return False
        return published and not self._is_shutdown()

    def lateral_adjust(
        self,
        target_y,
        lateral_speed,
        timeout,
        pos_tolerance=0.01,
        min_lateral_speed=None,
        log_label="二维码y",
    ):
        """以启动时机体坐标系为基准，闭环横移指定距离。"""

        del log_label  # 保留兼容参数，日志由上层 RobotIO 统一处理。
        start_x, start_y, start_yaw = self._get_robot_pose()
        if start_x is None or start_y is None or start_yaw is None:
            self._best_effort_stop()
            return False

        target_y = float(target_y)
        lateral_speed = abs(float(lateral_speed))
        timeout = max(float(timeout) + 1.0, 2.0)
        control_dt = 0.1
        pos_tolerance = abs(float(pos_tolerance))
        if min_lateral_speed is None:
            min_lateral_speed = min(0.04, max(lateral_speed * 0.5, 0.02))
        else:
            min_lateral_speed = abs(float(min_lateral_speed))

        deadline = time.monotonic() + timeout
        try:
            while not self._is_shutdown() and time.monotonic() < deadline:
                current_x, current_y, current_yaw = self._get_robot_pose()
                if current_x is None or current_y is None or current_yaw is None:
                    if not self._sleep(control_dt):
                        break
                    continue

                dx = float(current_x) - float(start_x)
                dy = float(current_y) - float(start_y)
                moved_y = -dx * math.sin(float(start_yaw)) + dy * math.cos(float(start_yaw))
                remaining_y = target_y - moved_y
                if abs(remaining_y) <= pos_tolerance:
                    return self.stop()

                vy = self.planned_axis_speed(
                    remaining_y,
                    lateral_speed,
                    min_lateral_speed,
                    pos_tolerance,
                )
                if not self._publish_cmd_vel(0.0, vy, 0.0):
                    break
                if not self._sleep(control_dt):
                    break
        except Exception:
            self._best_effort_stop()
            raise

        self._best_effort_stop()
        return False

    @staticmethod
    def approach_target_from_qr_pose(qr, approach_distance):
        """根据二维码法向计算相对接近位移和最终朝向。"""

        normal = rotate_vector_by_quat([0.0, 0.0, 1.0], qr["quat"])
        normal_xy_norm = math.sqrt(normal[0] * normal[0] + normal[1] * normal[1])
        if normal_xy_norm < 1e-6:
            return (
                max(float(qr["x"]) - float(approach_distance), 0.0),
                float(qr["y"]),
                0.0,
            )
        move_x = float(qr["x"]) + float(approach_distance) * normal[0]
        move_y = float(qr["y"]) + float(approach_distance) * normal[1]
        yaw = normalize_angle(math.atan2(normal[1], normal[0]) + math.pi)
        return move_x, move_y, yaw

    @staticmethod
    def planned_turn_speed(angle_diff, angular_speed):
        """按角度误差规划转速，保留原分段速度参数。"""

        angle_diff = float(angle_diff)
        abs_diff = abs(angle_diff)
        if abs_diff <= 1e-12:
            return 0.0
        max_speed = abs(float(angular_speed))
        min_speed = min(0.06, max_speed * 0.3)
        if abs_diff > 0.35:
            turn_speed = max_speed
        elif abs_diff > 0.12:
            turn_speed = min_speed + (abs_diff - 0.12) / (0.35 - 0.12) * (max_speed - min_speed)
        else:
            turn_speed = max(min_speed, abs_diff * 0.5)
        return turn_speed if angle_diff > 0.0 else -turn_speed

    @staticmethod
    def planned_axis_speed(error, max_abs, min_abs, tolerance):
        """按轴向误差生成带最小启动速度的有符号速度。"""

        error = float(error)
        max_abs = abs(float(max_abs))
        min_abs = abs(float(min_abs))
        tolerance = abs(float(tolerance))
        if abs(error) <= tolerance:
            return 0.0
        speed = max(-max_abs, min(max_abs, 0.8 * error))
        if abs(speed) < min_abs:
            return min_abs if error > 0.0 else -min_abs
        return speed

    def turn_to_yaw(self, target_yaw, yaw_tolerance, angular_speed, timeout, control_dt):
        """闭环旋转到 odom 坐标系中的目标 yaw。"""

        deadline = time.monotonic() + max(0.0, float(timeout))
        while not self._is_shutdown() and time.monotonic() < deadline:
            current_x, current_y, current_yaw = self._get_robot_pose()
            if current_x is None or current_y is None or current_yaw is None:
                if not self._sleep(control_dt):
                    return False
                continue
            angle_diff = normalize_angle(float(target_yaw) - float(current_yaw))
            if abs(angle_diff) < abs(float(yaw_tolerance)):
                return True
            speed = self.planned_turn_speed(angle_diff, angular_speed)
            if not self._publish_cmd_vel(0.0, 0.0, speed):
                return False
            if not self._sleep(control_dt):
                return False
        return False

    def _relative_target_to_odom(self, move_x, move_y, yaw, unavailable_message):
        current_x, current_y, current_yaw = self._get_robot_pose()
        if current_x is None or current_y is None or current_yaw is None:
            raise MotionError(unavailable_message)
        cos_yaw = math.cos(float(current_yaw))
        sin_yaw = math.sin(float(current_yaw))
        return (
            float(current_x) + float(move_x) * cos_yaw - float(move_y) * sin_yaw,
            float(current_y) + float(move_x) * sin_yaw + float(move_y) * cos_yaw,
            normalize_angle(float(current_yaw) + float(yaw)),
        )

    def relative_target_to_odom(self, move_x, move_y, yaw):
        return self._relative_target_to_odom(
            move_x,
            move_y,
            yaw,
            "无法获取 odom->base_link，不能执行闭环接近",
        )

    def turn_180(self):
        current_x, current_y, current_yaw = self._get_robot_pose()
        if current_x is None or current_y is None or current_yaw is None:
            self._best_effort_stop()
            return False
        try:
            ok = self.turn_to_yaw(
                normalize_angle(float(current_yaw) + math.pi),
                self.TURN_180_YAW_TOLERANCE,
                0.25,
                20.0,
                0.1,
            )
        except Exception:
            self._best_effort_stop()
            raise
        stopped = self.stop()
        return bool(ok and stopped)


class HumanoidMotionController(BaseMotionController):
    """人形机器人先转向、直行、再校正朝向的闭环控制器。"""

    def stance(self):
        if not self.stop():
            return False
        if self._is_shutdown():
            return False
        if self.robot_io.publish_stance() is False:
            self._best_effort_stop()
            return False
        return self._sleep(2.0)

    def walk_to_qr(self, qr, approach_distance):
        move_x, move_y, yaw = self.approach_target_from_qr_pose(qr, approach_distance)
        self.robot_io.loginfo(
            "二维码接近目标 ID=%s: qr_xyz=(%.3f, %.3f, %.3f) "
            "approach=%.3fm 相对到达位 move_xyyaw=(%.3f, %.3f, %.3f rad)",
            qr.get("id", qr.get("tag_id", "?")),
            float(qr["x"]),
            float(qr["y"]),
            float(qr.get("z", 0.0)),
            float(approach_distance),
            float(move_x),
            float(move_y),
            float(yaw),
        )
        if move_x <= 0.0 and abs(move_y) < 1e-3 and abs(yaw) < 1e-3:
            return self.stop()
        target_x, target_y, target_yaw = self.relative_target_to_odom(move_x, move_y, yaw)
        if not self.velocity_walk_to_target(target_x, target_y, target_yaw):
            raise MotionError("闭环接近失败")
        return True

    def velocity_walk_to_target(self, target_x, target_y, target_yaw):
        linear_speed = float(self.params.get("walk", {}).get("linear_speed", 0.15))
        angular_speed = 0.25
        pos_tolerance = 0.07
        yaw_tolerance = math.radians(5.0)
        try:
            turned = self.turn_to_target_direction(
                target_x,
                target_y,
                yaw_tolerance,
                angular_speed,
                180.0,
                0.1,
            )
            if not turned:
                self._best_effort_stop()
                return False
            if not self.keep_creeping():
                self._best_effort_stop()
                return False
            walked = self.walk_straight_to_target(
                target_x,
                target_y,
                pos_tolerance,
                linear_speed,
                30.0,
                0.1,
            )
            if not self.stop() or not walked:
                return False
            final_ok = self.turn_to_yaw(
                target_yaw,
                yaw_tolerance,
                angular_speed,
                60.0,
                0.1,
            )
            stopped = self.stop()
            return bool(final_ok and stopped)
        except Exception:
            self._best_effort_stop()
            raise

    def turn_to_target_direction(
        self,
        target_x,
        target_y,
        yaw_tolerance,
        angular_speed,
        timeout,
        control_dt,
    ):
        deadline = time.monotonic() + max(0.0, float(timeout))
        while not self._is_shutdown() and time.monotonic() < deadline:
            current_x, current_y, current_yaw = self._get_robot_pose()
            if current_x is None or current_y is None or current_yaw is None:
                if not self._sleep(control_dt):
                    return False
                continue
            angle_diff = normalize_angle(
                math.atan2(float(target_y) - float(current_y), float(target_x) - float(current_x))
                - float(current_yaw)
            )
            if abs(angle_diff) < abs(float(yaw_tolerance)):
                return True
            speed = self.planned_turn_speed(angle_diff, angular_speed)
            if not self._publish_cmd_vel(0.0, 0.0, speed):
                return False
            if not self._sleep(control_dt):
                return False
        return False

    def walk_straight_to_target(
        self,
        target_x,
        target_y,
        pos_tolerance,
        linear_speed,
        timeout,
        control_dt,
    ):
        deadline = time.monotonic() + max(0.0, float(timeout))
        while not self._is_shutdown() and time.monotonic() < deadline:
            current_x, current_y, current_yaw = self._get_robot_pose()
            if current_x is None or current_y is None or current_yaw is None:
                if not self._sleep(control_dt):
                    return False
                continue
            dx = float(target_x) - float(current_x)
            dy = float(target_y) - float(current_y)
            distance = math.sqrt(dx * dx + dy * dy)
            if distance < abs(float(pos_tolerance)):
                return True
            angle_diff = normalize_angle(math.atan2(dy, dx) - float(current_yaw))
            vx = float(linear_speed) if distance > 0.5 else float(linear_speed) * 0.7
            if abs(angle_diff) < 0.05:
                wz = 0.0
            elif abs(angle_diff) < 0.15:
                wz = 0.1 if angle_diff > 0.0 else -0.1
            else:
                wz = 0.15 if angle_diff > 0.0 else -0.15
            if not self._publish_cmd_vel(vx, 0.0, wz):
                return False
            if not self._sleep(control_dt):
                return False
        return False

    def keep_creeping(self):
        """保留旧策略：转向后以 0.05 m/s 前爬 1 秒。"""

        for _ in range(10):
            if self._is_shutdown():
                return False
            if not self._publish_cmd_vel(0.05, 0.0, 0.0):
                return False
            if not self._sleep(0.1):
                return False
        return True


class WheelMotionController(BaseMotionController):
    """轮式机器人同时控制 x/y/yaw 的闭环控制器。"""

    TURN_180_YAW_TOLERANCE = math.radians(2.0)

    def stance(self):
        if not self.stop():
            return False
        return self._sleep(0.5)

    def move_torso_relative_xyz(
        self,
        dx=0.0,
        dy=0.0,
        dz=0.0,
        duration=2.0,
        steps=30,
        initial_pose=None,
        yaw=None,
        pitch=None,
        wait_reach_time=True,
    ):
        """相对固定参考位姿偏移，从当前开环位姿线性插值到绝对目标。

        不传 ``initial_pose`` 时，默认以调用瞬间的当前位姿为参考；
        ``yaw``/``pitch`` 不传时默认为 0。路径起点始终为实时当前位姿。
        """

        # 1. 参数校验
        try:
            dx, dy, dz, duration = (float(v) for v in (dx, dy, dz, duration))
        except (TypeError, ValueError) as exc:
            raise ValueError("dx、dy、dz 和 duration 必须是数字") from exc
        if not all(math.isfinite(v) for v in (dx, dy, dz, duration)):
            raise ValueError("dx、dy、dz 和 duration 必须是有限数")
        if duration <= 0.0 or isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
            raise ValueError("duration 和 steps 必须是正数/正整数")
        if abs(dy) > 1e-12:
            self.robot_io.logwarn("轮臂没有躯干 y 自由度，dy=%.6f m 已忽略", dy)

        # 2. 读取参考位姿与当前起点
        def _read_torso_pose(label):
            try:
                pose = tuple(self.robot_io.get_torso_open_loop_pose())
            except RosReadinessError as exc:
                raise MotionError("无法读取躯干%s: %s" % (label, exc)) from exc
            if len(pose) != 4:
                raise MotionError("躯干%s必须为 (x, z, yaw, pitch)" % label)
            return pose

        if initial_pose is None:
            reference = _read_torso_pose("开环参考位姿")
        else:
            try:
                reference = tuple(float(v) for v in initial_pose)
            except (TypeError, ValueError) as exc:
                raise ValueError("initial_pose 必须为四维数值") from exc
            if len(reference) != 4 or not all(math.isfinite(v) for v in reference):
                raise ValueError("initial_pose 必须为四维有限数")

        start = _read_torso_pose("开环初始位姿")

        # 3. 计算绝对目标与插值增量
        target_yaw = 0.0 if yaw is None else float(yaw)
        target_pitch = 0.0 if pitch is None else float(pitch)
        goal = (reference[0] + dx, reference[1] + dz, target_yaw, target_pitch)
        if not all(math.isfinite(v) for v in goal):
            raise ValueError("躯干绝对目标必须是有限数")

        delta = tuple(g - s for g, s in zip(goal, start))
        step_period = duration / float(steps)

        self.robot_io.loginfo(
            "躯干相对偏移[dx=%.3f, dz=%.3f] -> 绝对目标[x=%.3f, z=%.3f, yaw=%.3f, pitch=%.3f]",
            dx, dz, *goal,
        )

        # 4. 分步插值发布
        for index in range(1, steps + 1):
            if self._is_shutdown():
                return False
            alpha = float(index) / float(steps)
            target = tuple(s + alpha * d for s, d in zip(start, delta))
            if self.robot_io.publish_torso_pose(*target) is False:
                return False
            if not self._sleep(step_period):
                return False

        # 5. 等待躯干到位
        if wait_reach_time:
            try:
                reach_time = self.robot_io.wait_for_torso_reach_time(timeout=1.0)
                if not self._sleep(reach_time + 0.5):
                    return False
            except RosReadinessError:
                self.robot_io.logwarn("未收到躯干到达时间，使用 0.5 秒后备等待")
                if not self._sleep(0.5):
                    return False
        return True

    def walk_to_qr(
        self,
        qr,
        approach_distance,
        use_qr_y_offset=True,
        use_qr_yaw=True,
    ):
        walk_qr = self.robot_io.transform_qr_for_walk(qr)
        move_x, move_y, relative_yaw = self.approach_target_from_qr_pose(
            walk_qr,
            approach_distance,
        )
        if not use_qr_y_offset:
            move_y = 0.0
        if not use_qr_yaw:
            relative_yaw = 0.0
        self.robot_io.loginfo(
            "二维码接近目标 ID=%s: qr_xyz=(%.3f, %.3f, %.3f) walk_qr_xyz=(%.3f, %.3f, %.3f) "
            "approach=%.3fm use_y=%s use_yaw=%s "
            "相对到达位 move_xyyaw=(%.3f, %.3f, %.3f rad)",
            walk_qr.get("id", walk_qr.get("tag_id", qr.get("id", qr.get("tag_id", "?")))),
            float(qr["x"]),
            float(qr["y"]),
            float(qr.get("z", 0.0)),
            float(walk_qr["x"]),
            float(walk_qr["y"]),
            float(walk_qr.get("z", 0.0)),
            float(approach_distance),
            bool(use_qr_y_offset),
            bool(use_qr_yaw),
            float(move_x),
            float(move_y),
            float(relative_yaw),
        )
        target_x, target_y, target_yaw = self._relative_target_to_odom(
            move_x,
            move_y,
            relative_yaw,
            "无法获取 odom->base_link，不能执行轮臂闭环接近",
        )
        if not self.xy_velocity_walk_to_target(target_x, target_y, target_yaw):
            raise MotionError("轮臂接近二维码失败")
        return True

    def xy_velocity_walk_to_target(
        self,
        target_x,
        target_y,
        target_yaw,
        linear_speed=0.15,
    ):
        """闭环移动到 odom 目标位姿。

        ``linear_speed`` 为单次调用的线速度上限，默认 0.15 m/s。
        """

        linear_speed = abs(float(linear_speed))
        min_axis_speed = 0.06
        angular_speed = 0.25
        pos_tolerance = 0.07
        axis_tolerance = 0.02
        yaw_tolerance = math.radians(2.0)
        timeout = 30.0
        control_dt = 0.1
        deadline = time.monotonic() + timeout
        try:
            while not self._is_shutdown() and time.monotonic() < deadline:
                current_x, current_y, current_yaw = self._get_robot_pose()
                if current_x is None or current_y is None or current_yaw is None:
                    if not self._sleep(control_dt):
                        break
                    continue
                dx_odom = float(target_x) - float(current_x)
                dy_odom = float(target_y) - float(current_y)
                cos_yaw = math.cos(float(current_yaw))
                sin_yaw = math.sin(float(current_yaw))
                err_x = cos_yaw * dx_odom + sin_yaw * dy_odom
                err_y = -sin_yaw * dx_odom + cos_yaw * dy_odom
                distance = math.sqrt(dx_odom * dx_odom + dy_odom * dy_odom)
                yaw_error = normalize_angle(float(target_yaw) - float(current_yaw))
                if distance < pos_tolerance and abs(yaw_error) < yaw_tolerance:
                    return self.stop()
                vx = self.planned_axis_speed(err_x, linear_speed, min_axis_speed, axis_tolerance)
                vy = self.planned_axis_speed(err_y, linear_speed, min_axis_speed, axis_tolerance)
                wz = (
                    self.planned_turn_speed(yaw_error, angular_speed)
                    if abs(yaw_error) >= yaw_tolerance
                    else 0.0
                )
                if not self._publish_cmd_vel(vx, vy, wz):
                    break
                if not self._sleep(control_dt):
                    break
        except Exception:
            self._best_effort_stop()
            raise

        self._best_effort_stop()
        return False

def build_motion_controller(robot_io, params=None):
    """根据 RobotIO 的机器人类型创建运动控制器。

    机器人型号判断集中在工厂中，业务入口只依赖统一的运动接口。后续新增底盘
    类型时，只需在这里注册控制器，不必修改状态机。
    """

    robot_type = str(robot_io.robot_type).strip().lower()
    if robot_type == "humanoid":
        return HumanoidMotionController(robot_io, params=params)
    if robot_type == "wheel":
        return WheelMotionController(robot_io, params=params)
    raise ValueError("不支持的运动控制器类型: %s" % (robot_type or "<empty>"))


__all__ = [
    "BaseMotionController",
    "HumanoidMotionController",
    "MotionError",
    "WheelMotionController",
    "build_motion_controller",
]
