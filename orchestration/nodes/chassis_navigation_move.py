# -*- coding: utf-8 -*-
"""通用底盘导航移动节点。

节点只依赖 ``IHardware`` 的通用导航能力，不直接访问厂商 ROS 服务：

1. 确认底盘处于导航模式；
2. 按本体相对坐标或 map 绝对坐标提交移动任务；
3. 非阻塞轮询任务状态，直到到达、失败或超时。

紧凑命令格式为 ``[x, y, theta]``，平移单位为米，角度单位为弧度。
"""

import json
import math
import os
import time

from py_trees.common import Status

from core.domain.chassis_options import MoveToTargetOptions
from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest


def _is_dry_run():
    """行为树 ``--dry-run`` 模式下不连接 ROS，也不控制真实底盘。"""
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in (
        "1",
        "true",
        "yes",
    )


@define_manifest(
    label="底盘导航移动",
    category=["motion", "chassis", "navigation"],
    tree_type="studio_smoke",
    description=(
        "通过通用 Hardware 接口执行本体相对移动或 map 绝对目标移动，"
        "具体底盘由硬件配置选择"
    ),
    params=[
        {
            "name": "navigation_config",
            "type": "json",
            "default": {},
            "description": "可选导航配置，包含 enable、目标点、速度和到达阈值",
        },
        {
            "name": "target_key",
            "type": "string",
            "default": "pick_point",
            "description": "navigation_config 中的目标点字段，例如 pick_point",
        },
        {
            "name": "enabled",
            "type": "bool",
            "default": True,
            "description": "False 时跳过导航并直接返回 SUCCESS",
        },
        {
            "name": "mode",
            "type": "string",
            "default": "relative",
            "options": ["relative", "map"],
            "description": "relative 为本体相对坐标，map 为地图绝对坐标",
        },
        {
            "name": "command",
            "type": "json",
            "default": [0.0, 0.0, 0.0],
            "description": "[x(m), y(m), theta(rad)]",
        },
        {
            "name": "avoid_enabled",
            "type": "bool",
            "default": True,
            "description": "是否启用导航避障",
        },
        {
            "name": "avoid_distance",
            "type": "float",
            "default": 0.5,
            "description": "避障距离(m)",
        },
        {
            "name": "linear_velocity",
            "type": "float",
            "default": 0.08,
            "description": "线速度(m/s)",
        },
        {
            "name": "angular_velocity",
            "type": "float",
            "default": 0.15,
            "description": "角速度(rad/s)",
        },
        {
            "name": "position_threshold",
            "type": "float",
            "default": 0.03,
            "description": "位置到达阈值(m)",
        },
        {
            "name": "angle_threshold",
            "type": "float",
            "default": 0.08,
            "description": "角度到达阈值(rad)",
        },
        {
            "name": "allow_rotation",
            "type": "bool",
            "default": True,
            "description": "是否允许底盘旋转",
        },
        {
            "name": "timeout",
            "type": "float",
            "default": 60.0,
            "description": "等待到达的总超时时间(s)",
        },
        {
            "name": "poll_interval",
            "type": "float",
            "default": 0.2,
            "description": "到达状态轮询间隔(s)",
        },
        {
            "name": "auto_switch_navigation",
            "type": "bool",
            "default": True,
            "description": "外部控制模式下是否自动切换到导航模式",
        },
        {
            "name": "stop_on_terminate",
            "type": "bool",
            "default": True,
            "description": "任务失败、超时或行为树中止时停止导航",
        },
    ],
    inputs=[],
    outputs=[],
)
class ChassisNavigationMove(BaseAction):
    """提交一个通用底盘导航任务，并映射为行为树状态。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._hardware = None
        self._task_id = ""
        self._started_at = 0.0
        self._next_poll_at = 0.0
        self._timeout = 60.0
        self._poll_interval = 0.2
        self._initialise_error = ""
        self._task_finished = False
        self._stop_on_terminate = True
        self._disabled = False

    def initialise(self):
        """校验参数、确认控制模式并提交一次移动任务。"""
        self._hardware = None
        self._task_id = ""
        self._started_at = 0.0
        self._next_poll_at = 0.0
        self._initialise_error = ""
        self._task_finished = False
        self._stop_on_terminate = True
        self._disabled = False

        try:
            params = self._navigation_params()
            enabled = self._as_bool(
                params.get("enabled", True), "enabled"
            )
            if not enabled:
                self._disabled = True
                self.feedback_message = "底盘导航已禁用，跳过当前节点"
                return

            mode = self._parse_mode(params.get("mode", "relative"))
            x, y, theta = self._parse_command(
                params.get("command", [0.0, 0.0, 0.0])
            )
            options = self._parse_options(params)
            self._timeout = self._positive_number(
                self.params.get("timeout", 60.0), "timeout"
            )
            self._poll_interval = self._positive_number(
                self.params.get("poll_interval", 0.2), "poll_interval"
            )
            self._stop_on_terminate = self._as_bool(
                self.params.get("stop_on_terminate", True),
                "stop_on_terminate",
            )

            if _is_dry_run():
                self.feedback_message = (
                    f"dry-run chassis navigation mode={mode}, "
                    f"command={[x, y, theta]}"
                )
                return

            self._hardware = get_shared_hardware()
            mode_result = self._ensure_navigation_mode(
                self._as_bool(
                    self.params.get("auto_switch_navigation", True),
                    "auto_switch_navigation",
                )
            )
            if not mode_result.success:
                raise RuntimeError(mode_result.message or "切换底盘导航模式失败")

            if mode == "relative":
                result = self._hardware.move_chassis_relative(
                    x=x,
                    y=y,
                    theta=theta,
                    options=options,
                )
            else:
                result = self._hardware.move_chassis_to_target(
                    x=x,
                    y=y,
                    theta=theta,
                    options=options,
                )

            if not result.success:
                raise RuntimeError(result.message or "底盘拒绝移动任务")
            data = result.data if isinstance(result.data, dict) else {}
            self._task_id = str(data.get("task_id", "")).strip()
            if not self._task_id:
                raise RuntimeError("底盘已接受任务，但未返回 task_id")

            self._started_at = time.monotonic()
            self._next_poll_at = self._started_at
            self.feedback_message = f"底盘导航任务已提交: {self._task_id}"
        except (TypeError, ValueError, RuntimeError) as exc:
            self._initialise_error = str(exc)
            self.feedback_message = f"底盘导航初始化失败: {exc}"
        except Exception as exc:
            self._initialise_error = str(exc)
            self.feedback_message = f"底盘导航异常: {exc}"

    def update(self):
        """按固定间隔非阻塞查询导航任务是否到达。"""
        if self._initialise_error:
            return Status.FAILURE

        if self._disabled:
            return Status.SUCCESS

        if _is_dry_run():
            return Status.SUCCESS

        if self._hardware is None or not self._task_id:
            self.feedback_message = "底盘导航任务尚未成功提交"
            return Status.FAILURE

        now = time.monotonic()
        elapsed = now - self._started_at
        if elapsed > self._timeout:
            self.feedback_message = (
                f"底盘导航任务等待超时: task_id={self._task_id}, "
                f"timeout={self._timeout:.1f}s"
            )
            return Status.FAILURE

        if now < self._next_poll_at:
            return Status.RUNNING
        self._next_poll_at = now + self._poll_interval

        result = self._hardware.check_chassis_arrived(
            task_id=self._task_id,
            blocking=False,
            timeout=0.0,
        )
        if not result.success:
            self.feedback_message = result.message or "底盘导航到达查询失败"
            return Status.FAILURE

        data = result.data if isinstance(result.data, dict) else {}
        if bool(data.get("arrived", False)):
            self._task_finished = True
            self.feedback_message = f"底盘已到达: {self._task_id}"
            return Status.SUCCESS

        status = data.get("status", "unknown")
        self.feedback_message = (
            f"底盘导航中: task_id={self._task_id}, status={status}"
        )
        return Status.RUNNING

    def terminate(self, new_status):
        """非正常结束时停止尚未完成的导航，避免底盘脱离行为树继续运动。"""
        should_stop = (
            not _is_dry_run()
            and self._hardware is not None
            and bool(self._task_id)
            and not self._task_finished
            and new_status != Status.SUCCESS
            and self._stop_on_terminate
        )
        if not should_stop:
            return

        try:
            result = self._hardware.set_chassis_external_control(True)
            if result.success:
                self.feedback_message += "；已停止未完成的底盘导航"
            else:
                self.feedback_message += (
                    "；停止底盘导航失败: "
                    f"{result.message or 'unknown error'}"
                )
        except Exception as exc:
            self.feedback_message += f"；停止底盘导航异常: {exc}"
        finally:
            # 已发起停止，不再对同一个任务重复操作。
            self._task_finished = True

    def _ensure_navigation_mode(self, auto_switch):
        """保证导航后端拥有控制权；外部控制状态 false 表示导航模式。"""
        state_result = self._hardware.get_chassis_external_control_state()
        if not state_result.success:
            return state_result

        data = state_result.data if isinstance(state_result.data, dict) else {}
        external_control_enabled = data.get("state")
        if external_control_enabled is False:
            return state_result
        if external_control_enabled is not True:
            return Result.fail("底盘返回了未知的控制模式")
        if not auto_switch:
            return Result.fail(
                "底盘当前为外部控制模式；请切换到导航模式，"
                "或启用 auto_switch_navigation"
            )
        return self._hardware.set_chassis_external_control(False)

    @staticmethod
    def _parse_mode(raw):
        mode = str(raw).strip().lower()
        if mode not in ("relative", "map"):
            raise ValueError("mode 必须为 relative 或 map")
        return mode

    def _navigation_params(self):
        """兼容普通参数和工厂展平后的导航配置；显式参数优先。"""
        config = self.params.get("navigation_config", {})
        if not isinstance(config, dict):
            raise ValueError("navigation_config 必须是对象")
        fields = {
            "enabled": "enable",
            "command": self.params.get("target_key", "pick_point"),
            "linear_velocity": "linear_velocity",
            "angular_velocity": "angular_velocity",
            "position_threshold": "arrived_position_threshold",
            "angle_threshold": "arrived_angular_threshold",
            "allow_rotation": "allow_rotation",
            "mode": "mode",
            "avoid_enabled": "avoid_enabled",
            "avoid_distance": "avoid_distance",
        }
        resolved = {}
        missing = object()
        for key, field in fields.items():
            value = self.params.get(
                key,
                self.params.get(f"navigation_config.{field}", config.get(field, missing)),
            )
            if value is not missing:
                resolved[key] = value
        if self.params.get("navigation_config__board_key") and "command" not in resolved:
            raise ValueError("navigation_config 缺少目标点配置")
        return resolved

    @staticmethod
    def _parse_command(raw):
        """解析 ``[x, y, theta]``，并拒绝 NaN/Inf。"""
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise ValueError(f"command 不是合法 JSON: {exc}")
        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            raise ValueError("command 必须是 [x, y, theta]")
        values = tuple(float(value) for value in raw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("command 中的 x、y、theta 必须是有限数")
        return values

    @classmethod
    def _parse_options(cls, params):
        """从节点参数构造与 ROS 解耦的导航选项。"""
        avoid_distance = cls._nonnegative_number(
            params.get("avoid_distance", 0.5), "avoid_distance"
        )
        linear_velocity = cls._positive_number(
            params.get("linear_velocity", 0.08), "linear_velocity"
        )
        angular_velocity = cls._positive_number(
            params.get("angular_velocity", 0.15), "angular_velocity"
        )
        position_threshold = cls._positive_number(
            params.get("position_threshold", 0.03), "position_threshold"
        )
        angle_threshold = cls._positive_number(
            params.get("angle_threshold", 0.08), "angle_threshold"
        )
        return MoveToTargetOptions(
            avoid_enabled=cls._as_bool(
                params.get("avoid_enabled", True), "avoid_enabled"
            ),
            avoid_distance=avoid_distance,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            position_threshold=position_threshold,
            angle_threshold=angle_threshold,
            allow_rotation=cls._as_bool(
                params.get("allow_rotation", True), "allow_rotation"
            ),
        )

    @staticmethod
    def _as_bool(raw, name):
        """兼容 JSON 布尔值和行为树编辑器产生的布尔字符串。"""
        if isinstance(raw, bool):
            return raw
        if isinstance(raw, str):
            value = raw.strip().lower()
            if value in ("true", "1", "yes"):
                return True
            if value in ("false", "0", "no"):
                return False
        if isinstance(raw, (int, float)) and raw in (0, 1):
            return bool(raw)
        raise ValueError(f"{name} 必须是布尔值")

    @staticmethod
    def _positive_number(raw, name):
        value = float(raw)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} 必须是大于 0 的有限数")
        return value

    @staticmethod
    def _nonnegative_number(raw, name):
        value = float(raw)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} 必须是大于等于 0 的有限数")
        return value
