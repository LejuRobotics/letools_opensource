"""Scene-only, cancellable JiBot navigation submission."""
import math
import os
import threading
import time

import py_trees
from py_trees.common import Status

from core.domain.chassis_options import MoveToTargetOptions
from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest
from .scene_io import get_scene_io

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _as_bool(value):
    if isinstance(value, dict) and "value" in value:
        value = value["value"]
    if isinstance(value, str):
        value = value.strip().lower()
        if value in ("true", "1", "yes", "on"):
            return True
        if value in ("false", "0", "no", "off", ""):
            return False
        raise ValueError("invalid boolean value: " + value)
    return bool(value)


def _positive(value, label):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(label + " must be finite and positive")
    return value


class _JibotServiceAction(BaseAction):
    """Nonblocking tree-side waits, with cancellation passed into the RPC gate."""
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._hardware = None
        self._flight = None
        self._failed = ""
        self._restore_attempted = False
        self._task_submitted = False

    def _reset_call(self):
        if self._flight is not None:
            self._flight["cancel"].set()
        self._flight = None
        self._task_submitted = False
        # Failure is deliberately latched across initialise(). A tree retry is
        # not evidence that a previous navigation RPC did not execute.
        if not self._failed:
            self._restore_attempted = False

    def _start_call(self, call, timeout):
        flight = {"cancel": threading.Event(), "lock": threading.Lock(),
                  "result": None, "finished": None,
                  "deadline": time.monotonic() + timeout}
        self._flight = flight

        def worker():
            try:
                remaining = flight["deadline"] - time.monotonic()
                if flight["cancel"].is_set() or remaining <= 0:
                    result = Result.fail("call cancelled before service dispatch")
                else:
                    result = call(flight["cancel"], remaining)
            except Exception as exc:
                result = Result.fail(f"JiBot service error: {exc}")
            with flight["lock"]:
                flight["result"] = result
                flight["finished"] = time.monotonic()

        try:
            threading.Thread(target=worker, name="grasp_jibot_call", daemon=True).start()
        except Exception as exc:
            flight["result"] = Result.fail(f"cannot start JiBot service worker: {exc}")
            flight["finished"] = time.monotonic()

    def _poll_call(self):
        flight = self._flight
        with flight["lock"]:
            if flight["result"] is not None:
                if flight["finished"] <= flight["deadline"]:
                    return flight["result"]
                flight["cancel"].set()
                return Result.fail("JiBot 服务响应超过截止时间，执行结果未确认")
        if time.monotonic() >= flight["deadline"]:
            flight["cancel"].set()
            return Result.fail("JiBot 服务提交超时，执行结果未确认")
        return None

    def _restore_control(self, reason):
        if self._restore_attempted or self._hardware is None:
            return
        self._restore_attempted = True
        self.feedback_message = reason + "；正在请求恢复 /enable_vel_control=True"

        def restore():
            try:
                result = self._hardware.enable_vel_control_jibot(True, timeout=5.0)
                detail = ("控制权恢复请求已确认" if result.success else
                          "控制权恢复请求失败: " + result.message)
            except Exception as exc:
                detail = f"控制权恢复请求异常: {exc}"
            self.feedback_message = reason + "；" + detail

        threading.Thread(target=restore, name="grasp_jibot_restore", daemon=True).start()

    def _fail(self, reason):
        self._failed = reason or "JiBot action failed"
        if self._flight is not None:
            self._flight["cancel"].set()
        self.feedback_message = self._failed
        self._restore_control(self._failed)
        return Status.FAILURE

    def terminate(self, new_status):
        if not _DRY_RUN and new_status != Status.SUCCESS and not self._task_submitted:
            self._fail(self._failed or "JiBot action interrupted")


@define_manifest(
    label="MTBF JiBot绝对目标点", category=["motion", "grasp_mtbf_v1"],
    tree_type="grasp_mtbf_v1",
    description="仅本场景使用；有界提交导航任务，到达由后续检查节点确认",
    params=[
        {"name": "x", "type": "float", "default": "0.0", "description": "map坐标系下目标x(m)"},
        {"name": "y", "type": "float", "default": "0.0", "description": "map坐标系下目标y(m)"},
        {"name": "theta", "type": "float", "default": "0.0", "description": "map坐标系下目标yaw(deg)"},
        {"name": "theta_unit", "type": "string", "default": "deg", "description": "theta的单位: deg或rad"},
        {"name": "avoid_enabled", "type": "bool", "default": "False", "description": "是否启用避障"},
        {"name": "avoid_distance", "type": "float", "default": "0.5", "description": "避障距离(m)"},
        {"name": "linear_velocity", "type": "float", "default": "0.30", "description": "线速度(m/s)"},
        {"name": "angular_velocity", "type": "float", "default": "0.50", "description": "角速度(rad/s)"},
        {"name": "position_threshold", "type": "float", "default": "0.08", "description": "位置到达阈值(m)"},
        {"name": "angle_threshold", "type": "float", "default": "0.1", "description": "角度到达阈值(rad)"},
        {"name": "allow_rotation", "type": "bool", "default": "True", "description": "是否允许旋转"},
        {"name": "service_call_timeout", "type": "float", "default": "8.0", "description": "提交导航任务的总超时(s)"},
        {"name": "task_id_key", "type": "string", "default": "current_task_id", "description": "保存task_id到黑板的键名"},
    ],
    inputs=[], outputs=[],
)
class GraspMtbfMoveToTarget(_JibotServiceAction):
    _relative = False

    def initialise(self):
        self._reset_call()
        if _DRY_RUN or self._failed:
            return
        self._hardware = get_scene_io()
        try:
            self._service_call_timeout = _positive(
                self.params.get("service_call_timeout", 8.0), "service_call_timeout")
            self._x = float(self.params.get("x", 0.2 if self._relative else 0.0))
            self._y = float(self.params.get("y", 0.0))
            theta = float(self.params.get("theta", 0.0))
            unit = str(self.params.get("theta_unit", "rad" if self._relative else "deg")).lower()
            if unit not in ("rad", "deg"):
                raise ValueError("theta_unit must be 'deg' or 'rad'")
            self._theta = math.radians(theta) if unit == "deg" else theta
            if not all(math.isfinite(x) for x in (self._x, self._y, self._theta)):
                raise ValueError("navigation target must contain finite values")
            self._options = MoveToTargetOptions(
                avoid_enabled=_as_bool(self.params.get("avoid_enabled", False)),
                avoid_distance=float(self.params.get("avoid_distance", 0.5)),
                linear_velocity=float(self.params.get("linear_velocity", 0.15 if self._relative else 1.0)),
                angular_velocity=float(self.params.get("angular_velocity", 0.25 if self._relative else 1.0)),
                position_threshold=float(self.params.get("position_threshold", 0.08)),
                angle_threshold=float(self.params.get("angle_threshold", 0.1)),
                allow_rotation=_as_bool(self.params.get("allow_rotation", True)),
            )
            for key in ("avoid_distance", "linear_velocity", "angular_velocity",
                        "position_threshold", "angle_threshold"):
                _positive(getattr(self._options, key), key)
        except (TypeError, ValueError) as exc:
            self._fail(f"navigation parameter error: {exc}")

    def _submit(self, cancel, remaining):
        fn = (self._hardware.base_move_relative_jibot if self._relative else
              self._hardware.base_move_to_target_jibot)
        return fn(x=self._x, y=self._y, theta=self._theta, options=self._options,
                  cancel_event=cancel, timeout=remaining)

    def update(self):
        if _DRY_RUN:
            self.feedback_message = "dry-run JiBot submission"
            return Status.SUCCESS
        if self._failed:
            return Status.FAILURE
        if self._task_submitted:
            return Status.SUCCESS
        if self._flight is None:
            self._start_call(self._submit, self._service_call_timeout)
            self.feedback_message = "正在提交导航任务"
            return Status.RUNNING
        result = self._poll_call()
        if result is None:
            return Status.RUNNING
        if not result.success:
            return self._fail(result.message)
        task_id = result.data.get("task_id") if isinstance(result.data, dict) else None
        if not task_id:
            return self._fail("navigation acknowledgement missing task_id")
        task_key = str(self.params.get("task_id_key", "current_task_id"))
        try:
            self.global_blackboard.register_key(key=task_key, access=py_trees.common.Access.WRITE)
            self.global_blackboard.set(task_key, task_id)
        except Exception as exc:
            return self._fail(f"cannot save navigation task_id: {exc}")
        self._task_id = task_id
        self._task_submitted = True
        self.feedback_message = f"导航任务已提交: {task_id}，尚未确认到达"
        return Status.SUCCESS
