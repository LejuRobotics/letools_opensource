# -*- coding: utf-8 -*-
"""GraspMtbfCalcLegMove：根据 TargetTag 计算并安全控制躯干/腿姿态（蹲、抬、恢复）。

- leg_mode="target"：stand_in_tag 位姿 + offset → tag 系 → odom → base_link；
  当 control_base=false 时只使用计算结果的高度/姿态，躯干 x 强制使用 fixed_torso_x
- leg_mode 其他值（搬箱树里用 "leg"）：直接用 offset_x/y/z + offset_yaw 作为 base 系躯干目标
- SDK 1.4.5 固定 BaseArm；下发前检查底盘速度指令及电机反馈，执行期间持续监控
- 指令零速与控制器目标反馈不等同于物理锁止或实测到位
"""

import ast
import math
import os
import time

import py_trees
from py_trees.common import Status

from core.common.transform import pose6d_to_matrix, transform_pose
from core.domain.pose import Pose6D
from orchestration.nodes.base_node import BaseAction
from .scene_io import get_scene_io
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _parse_list(raw, default, name):
    if raw is None:
        return list(default)
    try:
        parsed = raw if isinstance(raw, (list, tuple)) else ast.literal_eval(str(raw))
        values = [float(x) for x in parsed]
    except (TypeError, ValueError, SyntaxError) as exc:
        raise ValueError(f"{name}必须是{len(default)}个有限数") from exc
    if len(values) != len(default) or not all(math.isfinite(x) for x in values):
        raise ValueError(f"{name}必须是{len(default)}个有限数")
    return values


def _as_bool(value):
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
        raise ValueError(f"无效布尔值: {value}")
    return bool(value)


def _base_from_odom(pose_odom: Pose6D):
    """将 odom 系位姿换算到 base_link 系（经 tf2）。失败返回 None。"""
    try:
        import rospy
        import tf2_ros
        import tf2_geometry_msgs  # noqa: F401
        from geometry_msgs.msg import PoseStamped
        from core.common.math_utils import quaternion_to_euler

        if not hasattr(_base_from_odom, "_buffer"):
            _base_from_odom._buffer = tf2_ros.Buffer()
            _base_from_odom._listener = tf2_ros.TransformListener(_base_from_odom._buffer)

        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.header.stamp = rospy.Time(0)
        ps.pose.position.x = pose_odom.x
        ps.pose.position.y = pose_odom.y
        ps.pose.position.z = pose_odom.z
        qx, qy, qz, qw = pose_odom.to_quaternion()
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        transform = _base_from_odom._buffer.lookup_transform(
            "base_link", "odom", rospy.Time(0), rospy.Duration(0.5))
        out = tf2_geometry_msgs.do_transform_pose(ps, transform)
        roll, pitch, yaw = quaternion_to_euler(
            out.pose.orientation.x, out.pose.orientation.y,
            out.pose.orientation.z, out.pose.orientation.w)
        return Pose6D(x=out.pose.position.x, y=out.pose.position.y, z=out.pose.position.z,
                      roll=roll, pitch=pitch, yaw=yaw)
    except Exception:
        return None


@define_manifest(
    label="计算并控制躯干/腿",
    category=["motion", "torso"],
    tree_type="grasp_mtbf_v1",
    description="根据 TargetTag 计算躯干目标（或直接用 offset），下发躯干位姿控制（蹲/抬/恢复）",
    params=[
        {"name": "stand_in_tag_pos", "type": "string", "default": "[-0.04, 0.15, 0.37]", "description": "站立位置在Tag坐标系下的位置 [x,y,z]（米）"},
        {"name": "stand_in_tag_euler", "type": "string", "default": "[-1.57, 1.57, 0.0]", "description": "站立姿态在Tag坐标系下的欧拉角 [r,p,y]（弧度）"},
        {"name": "leg_mode", "type": "string", "default": "target", "description": "腿部控制模式", "options": ["target", "leg"]},
        {"name": "tag_id", "type": "int", "default": "0", "description": "leg_mode=target 时读 latest_tag_<tag_id>（NodePercep 输出）"},
        {"name": "offset_x", "type": "float", "default": "0.0", "description": "X方向偏移（米）"},
        {"name": "offset_y", "type": "float", "default": "0.0", "description": "Y方向偏移（米）"},
        {"name": "offset_z", "type": "float", "default": "0.0", "description": "Z方向偏移（米）"},
        {"name": "offset_yaw", "type": "float", "default": "0.0", "description": "偏航角偏移（度）"},
        {"name": "offset_pitch", "type": "float", "default": "0.0", "description": "俯仰角偏移（度），正值前倾"},
        {"name": "control_base", "type": "bool", "default": "false", "description": "该节点禁止控制底盘；必须为false"},
        {"name": "fixed_torso_x", "type": "float", "default": "0.15", "description": "control_base=false且target模式时锁定的base系躯干x（米）"},
        {"name": "min_torso_x", "type": "float", "default": "-0.05", "description": "躯干x安全下限（米）"},
        {"name": "max_torso_x", "type": "float", "default": "0.30", "description": "躯干x安全上限（米）"},
        {"name": "min_torso_z", "type": "float", "default": "0.60", "description": "躯干z安全下限（米）"},
        {"name": "max_torso_z", "type": "float", "default": "1.50", "description": "躯干z安全上限（米）"},
        {"name": "max_torso_x_step", "type": "float", "default": "0.20", "description": "相对当前目标允许的最大x跳变（米）"},
        {"name": "ready_timeout", "type": "float", "default": "2.0", "description": "等待电机反馈和底盘指令零速的超时（秒）；兼容旧mode_timeout参数"},
        {"name": "motion_timeout", "type": "float", "default": "60.0", "description": "允许的最大躯干规划时长（秒）"},
        {"name": "settle_time", "type": "float", "default": "0.30", "description": "规划结束后的稳定等待（秒）"},
        {"name": "base_velocity_threshold", "type": "float", "default": "0.02", "description": "允许的底盘速度指令绝对值上限"},
        {"name": "total_time", "type": "float", "default": "2.0", "description": "躯干执行时长(秒)，调大降速"},
    ],
    inputs=[
        {"name": "target_tag", "type": "object", "required": False, "default_key": "TargetTag", "description": "目标 Tag（leg_mode=target 时需要）"},
    ],
    outputs=[],
)
class GraspMtbfCalcLegMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._hardware = None
        self._fault_latched = None
        self._phase = "idle"
        self._target = None
        self._total_time = 0.0
        self._deadline = 0.0
        self._motion_finish_at = 0.0
        self._target_confirm_at = 0.0
        self._target_confirmed = False
        self._settle_time = 0.3
        self._base_velocity_threshold = 0.02

    @staticmethod
    def _target_values(target_state):
        if target_state is None:
            return None
        try:
            if not isinstance(target_state, dict):
                raise ValueError("反馈格式不是字典")
            values = target_state.get("values")
            if isinstance(values, (list, tuple)) and len(values) >= 6:
                result = [float(value) for value in values[:6]]
            else:
                position = target_state.get("position")
                orientation = target_state.get("orientation_euler")
                if not isinstance(position, dict) or not isinstance(orientation, dict):
                    raise ValueError("反馈缺少values或position/orientation_euler")
                result = [
                    float(position["x"]), float(position["y"]),
                    float(position["z"]), float(orientation["yaw"]),
                    float(orientation["pitch"]), float(orientation["roll"]),
                ]
            if not all(math.isfinite(value) for value in result):
                raise ValueError("反馈包含NaN或Inf")
            return result
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"躯干目标反馈无效: {exc}") from exc

    def _motor_faults(self):
        getter = getattr(self._hardware, "get_motor_error_codes", None)
        codes = getter() if callable(getter) else None
        if codes is None:
            return None
        if not isinstance(codes, (list, tuple)) or not codes:
            return [("feedback", "格式错误或为空")]
        faults = []
        for index, code in enumerate(codes):
            try:
                value = float(code)
            except (TypeError, ValueError):
                faults.append((index, code))
                continue
            if not math.isfinite(value) or abs(value) > 1e-9:
                faults.append((index, code))
        return faults

    def _base_command_is_zero(self):
        getter = getattr(self._hardware, "get_base_cmd_vel", None)
        velocity = getter() if callable(getter) else None
        if velocity is None:
            return False, "未收到/base_cmd_vel反馈"
        if not isinstance(velocity, dict):
            return False, "底盘速度指令反馈格式错误"
        try:
            values = [float(velocity[key]) for key in ("vx", "vy", "wz")]
        except (KeyError, TypeError, ValueError):
            return False, "底盘速度指令反馈缺少vx/vy/wz或数值无效"
        if not all(math.isfinite(value) for value in values):
            return False, "底盘速度指令反馈包含非有限值"
        stopped = all(abs(value) <= self._base_velocity_threshold for value in values)
        return stopped, f"vx={values[0]:.3f}, vy={values[1]:.3f}, wz={values[2]:.3f}"

    def _safety_stop(self, reason):
        self.feedback_message = reason
        self._phase = "failed"
        if self._hardware is None:
            return
        self._fault_latched = reason
        freeze = getattr(self._hardware, "set_motion_control_enabled", None)
        if callable(freeze):
            try:
                result = freeze(False)
                if result is None or not result.success:
                    self.feedback_message += f"；停止请求失败: {getattr(result, 'message', '缺少响应')}"
            except Exception as exc:
                self.feedback_message += f"；停止请求异常: {exc}"
        else:
            self.feedback_message += "；停止接口不可用"

    def _validate_target(self, target):
        values = [target.x, target.z, target.yaw, target.pitch]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("躯干目标包含NaN或Inf")

        min_x = float(self.params.get("min_torso_x", -0.05))
        max_x = float(self.params.get("max_torso_x", 0.30))
        min_z = float(self.params.get("min_torso_z", 0.60))
        max_z = float(self.params.get("max_torso_z", 1.50))
        bounds = [min_x, max_x, min_z, max_z]
        if not all(math.isfinite(value) for value in bounds):
            raise ValueError("躯干安全范围必须是有限数")
        if min_x >= max_x or min_z >= max_z:
            raise ValueError("躯干安全范围下限必须小于上限")
        if not min_x <= target.x <= max_x:
            raise ValueError(f"躯干x={target.x:.3f}超出安全范围[{min_x:.3f}, {max_x:.3f}]")
        if not min_z <= target.z <= max_z:
            raise ValueError(f"躯干z={target.z:.3f}超出安全范围[{min_z:.3f}, {max_z:.3f}]")
        if abs(math.degrees(target.pitch)) > 30.0 or abs(math.degrees(target.yaw)) > 30.0:
            raise ValueError("躯干pitch/yaw超过30度安全范围")

        getter = getattr(self._hardware, "get_torso_target_6d", None)
        current = self._target_values(getter()) if callable(getter) else None
        if current is not None:
            max_step = float(self.params.get("max_torso_x_step", 0.20))
            if not math.isfinite(max_step) or max_step <= 0.0:
                raise ValueError("max_torso_x_step必须是大于0的有限数")
            if abs(target.x - current[0]) > max_step:
                raise ValueError(
                    f"躯干x跳变过大: current={current[0]:.3f}, "
                    f"target={target.x:.3f}, limit={max_step:.3f}")
        return current is not None

    def initialise(self):
        if self._fault_latched:
            self.feedback_message = self._fault_latched + "；需确认控制器状态后重启流程"
            self._phase = "failed"
            return
        self._hardware = None
        self._phase = "idle"
        self._target = None
        self._target_confirmed = False

        if _DRY_RUN:
            self._phase = "success"
            return

        try:
            self._hardware = get_scene_io()
            if _as_bool(self.params.get("control_base", False)):
                raise ValueError("GraspMtbfCalcLegMove不允许控制底盘，请使用独立导航节点")

            offset_x = float(self.params.get("offset_x", 0.0))
            offset_y = float(self.params.get("offset_y", 0.0))
            offset_z = float(self.params.get("offset_z", 0.0))
            offset_yaw_deg = float(self.params.get("offset_yaw", 0.0))
            offset_pitch_deg = float(self.params.get("offset_pitch", 0.0))
            leg_mode = str(self.params.get("leg_mode", "target")).strip().lower()
            if leg_mode not in ("target", "leg"):
                raise ValueError(f"不支持的leg_mode: {leg_mode}")

            self._total_time = float(self.params.get("total_time", 3.0))
            if not math.isfinite(self._total_time) or self._total_time < 1.0:
                raise ValueError("total_time必须是大于等于1秒的有限数")
            self._motion_timeout = float(self.params.get("motion_timeout", 60.0))
            if not math.isfinite(self._motion_timeout) or not 0 < self._total_time <= self._motion_timeout:
                raise ValueError("total_time不能超过有效的motion_timeout")
            self._settle_time = float(self.params.get("settle_time", 0.30))
            self._base_velocity_threshold = float(
                self.params.get("base_velocity_threshold", 0.02))
            if not math.isfinite(self._settle_time) or self._settle_time < 0.0:
                raise ValueError("settle_time必须是大于等于0的有限数")
            if (not math.isfinite(self._base_velocity_threshold) or
                    self._base_velocity_threshold < 0.0):
                raise ValueError("base_velocity_threshold必须是大于等于0的有限数")

            self._tag_id = int(self.params.get("tag_id", 0) or 0)
            if leg_mode == "target" and self._tag_id > -1:
                try:
                    self.global_blackboard.register_key(
                        key=f"latest_tag_{self._tag_id}",
                        access=py_trees.common.Access.READ)
                except Exception:
                    pass

            if leg_mode == "target":
                tag = getattr(
                    self.global_blackboard, f"latest_tag_{self._tag_id}", None)
                if tag is None or getattr(tag, "pose_in_world", None) is None:
                    raise ValueError(f"latest_tag_{self._tag_id} 未就绪")

                stand_pos = _parse_list(
                    self.params.get("stand_in_tag_pos"), [-0.04, 0.15, 0.37],
                    "stand_in_tag_pos")
                stand_euler = _parse_list(
                    self.params.get("stand_in_tag_euler"), [-1.57, 1.57, 0.0],
                    "stand_in_tag_euler")
                stand_pos = [stand_pos[0] + offset_x,
                             stand_pos[1] + offset_y,
                             stand_pos[2] + offset_z]

                t = tag.pose_in_world
                tag_fixed = Pose6D(
                    x=t.x, y=t.y, z=t.z,
                    roll=math.pi / 2, pitch=0.0, yaw=t.yaw)
                stand_in_tag = Pose6D(
                    x=stand_pos[0], y=stand_pos[1], z=stand_pos[2],
                    roll=stand_euler[0], pitch=stand_euler[1],
                    yaw=stand_euler[2])
                stand_in_odom = transform_pose(
                    stand_in_tag, pose6d_to_matrix(tag_fixed))
                transformed = _base_from_odom(stand_in_odom)
                if transformed is None:
                    raise ValueError("odom→base_link 变换失败")

                # Tag坐标轴与base坐标轴并不一致。该节点不允许Tag偏移改变
                # 躯干前后位置，否则offset_z会被旋转成危险的torso x。
                self._target = Pose6D(
                    x=float(self.params.get("fixed_torso_x", 0.15)),
                    y=0.0,
                    z=transformed.z,
                    roll=0.0,
                    pitch=transformed.pitch + math.radians(offset_pitch_deg),
                    yaw=transformed.yaw + math.radians(offset_yaw_deg),
                )
            else:
                self._target = Pose6D(
                    x=offset_x, y=offset_y, z=offset_z, roll=0.0,
                    pitch=math.radians(offset_pitch_deg),
                    yaw=math.radians(offset_yaw_deg))

            self._validate_target(self._target)
            faults = self._motor_faults()
            if faults:
                raise RuntimeError(f"电机存在错误码: {faults[:4]}")

            ready_timeout = float(self.params.get("ready_timeout", self.params.get("mode_timeout", 2.0)))
            if not math.isfinite(ready_timeout) or ready_timeout <= 0.0:
                raise ValueError("ready_timeout必须是大于0的有限数")
            self._deadline = time.monotonic() + ready_timeout
            self._phase = "wait_ready"
            self.feedback_message = "等待电机反馈和底盘指令零速（SDK固定BaseArm）"
        except (TypeError, ValueError) as exc:
            self.feedback_message = f"GraspMtbfCalcLegMove参数/目标错误: {exc}"
            self._phase = "failed"
        except Exception as exc:
            self._safety_stop(f"GraspMtbfCalcLegMove启动失败: {exc}")

    def update(self):
        try:
            return self._update()
        except Exception as exc:
            self._safety_stop(f"GraspMtbfCalcLegMove执行异常: {exc}")
            return Status.FAILURE

    def _update(self):
        if self._phase == "success":
            return Status.SUCCESS
        if self._phase == "failed":
            return Status.FAILURE
        if self._phase == "idle":
            return Status.RUNNING

        now = time.monotonic()
        faults = self._motor_faults()
        if faults is None:
            if self._phase == "wait_motion" or now >= self._deadline:
                self._safety_stop("等待电机错误码反馈超时或执行中反馈丢失")
                return Status.FAILURE
            self.feedback_message = "等待电机错误码反馈"
            return Status.RUNNING
        if faults:
            self._safety_stop(f"躯干运动中检测到电机错误码: {faults[:4]}")
            return Status.FAILURE

        if self._phase == "wait_ready":
            stopped, velocity_text = self._base_command_is_zero()
            if now >= self._deadline:
                self._safety_stop(f"躯干就绪确认超时（底盘指令/当前躯干目标）: {velocity_text}")
                return Status.FAILURE
            if stopped:
                # A scene-local subscription may not have received its first
                # sample during initialise. Never skip the x-step limit then.
                if not self._validate_target(self._target):
                    self.feedback_message = "等待控制器躯干当前目标反馈以检查x跳变"
                    return Status.RUNNING
                result = self._hardware.send_torso_pose_timed(
                    x=self._target.x,
                    z=self._target.z,
                    yaw=math.degrees(self._target.yaw),
                    pitch=math.degrees(self._target.pitch),
                    desire_time=self._total_time,
                )
                if not result.success:
                    self._safety_stop(f"躯干控制指令被拒绝: {result.message}")
                    return Status.FAILURE
                if not isinstance(result.data, dict) or "actual_time" not in result.data:
                    raise ValueError("躯干控制返回缺少actual_time")
                actual_time = float(result.data["actual_time"])
                if not math.isfinite(actual_time) or not 0 <= actual_time <= self._motion_timeout:
                    self._safety_stop(f"躯干控制返回非法执行时长: {actual_time}")
                    return Status.FAILURE
                actual_time = max(actual_time, self._total_time)
                self._motion_finish_at = time.monotonic() + actual_time
                # torso_target_6D 是轨迹当前点，只能在规划结束后与终点比较。
                self._target_confirm_at = self._motion_finish_at
                self._target_confirmed = False
                self._deadline = self._motion_finish_at + self._settle_time + 1.0
                self._phase = "wait_motion"
                self.feedback_message = (
                    f"躯干运动执行中: x={self._target.x:.3f}, "
                    f"z={self._target.z:.3f}, duration={actual_time:.2f}s")
                return Status.RUNNING

            self.feedback_message = f"等待底盘指令零速: {velocity_text}"
            return Status.RUNNING

        if self._phase == "wait_motion":
            stopped, velocity_text = self._base_command_is_zero()
            if not stopped:
                self._safety_stop(f"躯干动作期间检测到底盘非零或无效速度指令: {velocity_text}")
                return Status.FAILURE

            if now >= self._target_confirm_at:
                getter = getattr(self._hardware, "get_torso_target_6d", None)
                try:
                    current = self._target_values(getter()) if callable(getter) else None
                except ValueError as exc:
                    self._safety_stop(str(exc))
                    return Status.FAILURE
                if current is None:
                    self.feedback_message = "等待控制器躯干终点反馈"
                elif (
                        abs(current[0] - self._target.x) > 0.02 or
                        abs(current[2] - self._target.z) > 0.02):
                    self._safety_stop(
                        "控制器反馈的躯干目标与下发值不一致: "
                        f"feedback=({current[0]:.3f},{current[2]:.3f}), "
                        f"command=({self._target.x:.3f},{self._target.z:.3f})")
                    return Status.FAILURE
                else:
                    self._target_confirmed = True
                    self._target_confirm_at = float("inf")

            if now >= self._deadline:
                reason = ("未收到控制器有效的躯干终点反馈"
                          if not self._target_confirmed else "躯干运动执行超时")
                self._safety_stop(reason)
                return Status.FAILURE
            if (self._target_confirmed and
                    now >= self._motion_finish_at + self._settle_time):
                self.feedback_message = (
                    f"躯干规划结束且目标反馈已确认: x={self._target.x:.3f}, z={self._target.z:.3f}")
                self._phase = "success"
                return Status.SUCCESS
            return Status.RUNNING

        self._safety_stop(f"未知执行阶段: {self._phase}")
        return Status.FAILURE

    def terminate(self, new_status):
        if new_status != Status.SUCCESS and self._phase in ("wait_ready", "wait_motion"):
            self._safety_stop(self.feedback_message or "GraspMtbfCalcLegMove被中断")
