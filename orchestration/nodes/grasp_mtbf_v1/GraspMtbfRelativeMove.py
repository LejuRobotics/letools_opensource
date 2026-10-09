"""Relative retreat waits for JiBot arrival before the next control handover."""
import time

from py_trees.common import Status
from orchestration.utils.manifest_decorators import define_manifest
from .GraspMtbfMoveToTarget import GraspMtbfMoveToTarget, _DRY_RUN, _positive


@define_manifest(
    label="MTBF JiBot相对移动", category=["motion", "grasp_mtbf_v1"],
    tree_type="grasp_mtbf_v1", description="相对移动并非阻塞轮询到达；theta默认rad",
    params=[
        {"name": "x", "type": "float", "default": "0.2", "description": "相对当前位置的x方向位移(m)"},
        {"name": "y", "type": "float", "default": "0.0", "description": "相对当前位置的y方向位移(m)"},
        {"name": "theta", "type": "float", "default": "0.0", "description": "相对当前位置的yaw角度变化(rad)"},
        {"name": "avoid_enabled", "type": "bool", "default": "False", "description": "是否启用避障"},
        {"name": "avoid_distance", "type": "float", "default": "0.5", "description": "避障距离(m)"},
        {"name": "linear_velocity", "type": "float", "default": "0.30", "description": "线速度(m/s)"},
        {"name": "angular_velocity", "type": "float", "default": "0.50", "description": "角速度(rad/s)"},
        {"name": "position_threshold", "type": "float", "default": "0.08", "description": "位置到达阈值(m)"},
        {"name": "angle_threshold", "type": "float", "default": "0.1", "description": "角度到达阈值(rad)"},
        {"name": "allow_rotation", "type": "bool", "default": "True", "description": "是否允许旋转"},
        {"name": "service_call_timeout", "type": "float", "default": "8.0", "description": "单次服务总时限(s)"},
        {"name": "timeout", "type": "float", "default": "60.0", "description": "提交后等待到达的总时限(s)"},
        {"name": "poll_interval", "type": "float", "default": "0.2", "description": "到达轮询间隔(s)"},
    ],
    inputs=[], outputs=[],
)
class GraspMtbfRelativeMove(GraspMtbfMoveToTarget):
    _relative = True

    def initialise(self):
        super().initialise()
        self._arrival_deadline = None
        self._next_poll = 0.0
        self._arrived = False
        if not _DRY_RUN and not self._failed:
            try:
                self._movement_timeout = _positive(self.params.get("timeout", 60.0), "timeout")
                self._poll_interval = _positive(self.params.get("poll_interval", 0.2), "poll_interval")
            except (TypeError, ValueError) as exc:
                self._fail(f"navigation parameter error: {exc}")

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS
        if self._failed:
            return Status.FAILURE
        if self._arrived:
            return Status.SUCCESS
        if not self._task_submitted:
            status = super().update()
            if status != Status.SUCCESS:
                return status
            self._arrival_deadline = time.monotonic() + self._movement_timeout
            self._flight = None
            return Status.RUNNING
        now = time.monotonic()
        if now >= self._arrival_deadline:
            return self._fail("JiBot relative movement arrival timed out")
        if self._flight is None:
            if now < self._next_poll:
                return Status.RUNNING
            budget = min(self._service_call_timeout, self._arrival_deadline - now)
            self._start_call(lambda cancel, remaining: self._hardware.check_arrived_jibot(
                self._task_id, blocking=False, timeout=0.0,
                cancel_event=cancel, service_timeout=remaining), budget)
            return Status.RUNNING
        result = self._poll_call()
        if result is None:
            return Status.RUNNING
        if not result.success:
            return self._fail(result.message)
        data = result.data if isinstance(result.data, dict) else {}
        status = data.get("status")
        if data.get("arrived") is True and data.get("success") is True and status == 2:
            self._arrived = True
            self.feedback_message = f"JiBot relative task arrived: {self._task_id}"
            return Status.SUCCESS
        message = str(data.get("message", "")).lower()
        if status not in (0, 1) or any(x in message for x in ("interrupt", "cancel", "failed", "error")):
            return self._fail(f"JiBot relative task failed: {data}")
        self._flight = None
        self._next_poll = time.monotonic() + self._poll_interval
        return Status.RUNNING

    def terminate(self, new_status):
        if not _DRY_RUN and new_status != Status.SUCCESS:
            self._fail(self._failed or "JiBot relative movement interrupted")
