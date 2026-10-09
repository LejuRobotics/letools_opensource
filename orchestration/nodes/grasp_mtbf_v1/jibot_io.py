"""JiBot ROS transport private to the grasp_mtbf_v1 scene.

The gates belong to the scene IO instance. A timeout bounds the caller; it
cannot retract a request already accepted by the ROS server.
"""
import math
import time

from core.common.logger import get_logger
from core.domain.chassis_options import MoveToTargetOptions
from core.domain.result import Result
from .ros_service_call import RosServiceGate

logger = get_logger(__name__)


class JiBotIO:
    def _init_jibot_io(self):
        self._jibot_navigation_calls = RosServiceGate()
        self._jibot_arrival_calls = RosServiceGate()
        self._jibot_costmap_calls = RosServiceGate()
        self._jibot_release_calls = RosServiceGate()
        # A failed handover must not prevent an independent recovery attempt.
        self._jibot_restore_calls = RosServiceGate()

    @staticmethod
    def _deadline(timeout):
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("service timeout must be finite and positive")
        return time.monotonic() + timeout

    @staticmethod
    def _remaining(deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("JiBot service deadline expired before sending")
        return remaining

    @staticmethod
    def _to_ros_options(options):
        from leju_mobile_base_msgs.msg import MoveToTargetOptions as RosOptions
        return RosOptions(
            avoid_enabled=options.avoid_enabled,
            avoid_distance=options.avoid_distance,
            linear_velocity=options.linear_velocity,
            angular_velocity=options.angular_velocity,
            position_threshold=options.position_threshold,
            angle_threshold=options.angle_threshold,
            allow_rotation=options.allow_rotation,
        )

    def _enable_navigation_costmaps(self, deadline, cancel_event):
        from std_srvs.srv import SetBool, SetBoolRequest
        try:
            response = self._jibot_costmap_calls.call(
                "/move_base/enable_costmaps", SetBool, SetBoolRequest(True),
                timeout=min(1.0, self._remaining(deadline)),
                cancel_event=cancel_event,
            )
            if not response.success:
                logger.warning("[grasp_mtbf_v1] costmaps rejected: %s", response.message)
        except Exception as exc:
            # Legacy deployments do not provide costmaps. Cancellation is still
            # checked at the navigation RPC boundary after this best-effort step.
            logger.warning("[grasp_mtbf_v1] costmaps unavailable: %s", exc)

    def _submit_jibot(self, relative, x, y, theta, options, cancel_event, timeout):
        label = "base_move" if relative else "move_to_target"
        try:
            from leju_mobile_base_msgs.srv import BaseMove, MoveToTarget
            deadline = self._deadline(timeout)
            if not all(math.isfinite(float(value)) for value in (x, y, theta)):
                raise ValueError("navigation target must contain finite values")
            options = options if options is not None else MoveToTargetOptions()
            ros_options = self._to_ros_options(options)
            service_type = BaseMove if relative else MoveToTarget
            request = service_type._request_class(x, y, theta, ros_options)
            if not relative:
                self._enable_navigation_costmaps(deadline, cancel_event)
            response = self._jibot_navigation_calls.call(
                "/move_base/" + label, service_type, request,
                timeout=self._remaining(deadline), cancel_event=cancel_event,
            )
            data = {"task_id": response.task_id} if response.task_id else None
            if response.success:
                return Result.ok(f"{label} task accepted: {response.message}", data=data)
            return Result.fail(f"{label} failed: {response.message}",
                               error_code="JIBOT_NAVIGATION_FAILED", data=data)
        except Exception as exc:
            return Result.fail(f"{label} error: {exc}", error_code="JIBOT_SERVICE_ERROR")

    def base_move_to_target_jibot(self, x, y, theta, options=None, *,
                                 cancel_event=None, timeout=8.0):
        return self._submit_jibot(False, x, y, theta, options, cancel_event, timeout)

    def base_move_relative_jibot(self, x, y, theta, options=None, *,
                                cancel_event=None, timeout=8.0):
        return self._submit_jibot(True, x, y, theta, options, cancel_event, timeout)

    def check_arrived_jibot(self, task_id, blocking=True, timeout=20.0, *,
                           cancel_event=None, service_timeout=5.0):
        try:
            from leju_mobile_base_msgs.srv import CheckArrived
            request = CheckArrived._request_class(task_id, blocking, timeout)
            response = self._jibot_arrival_calls.call(
                "/move_base/check_arrived", CheckArrived, request,
                timeout=service_timeout, cancel_event=cancel_event,
            )
            # RPC success is separate from the server's navigation task status.
            return Result.ok(f"check_arrived: {response.message}", data={
                "arrived": response.arrived, "success": response.success,
                "status": response.status, "message": response.message,
            })
        except Exception as exc:
            return Result.fail(f"check_arrived error: {exc}", error_code="JIBOT_SERVICE_ERROR")

    def enable_vel_control_jibot(self, enable, *, cancel_event=None, timeout=5.0):
        """Switch ownership; acknowledgement is not proof of physical stopping."""
        try:
            from std_srvs.srv import SetBool, SetBoolRequest
            gate = self._jibot_restore_calls if enable else self._jibot_release_calls
            response = gate.call("/enable_vel_control", SetBool,
                                 SetBoolRequest(bool(enable)), timeout=timeout,
                                 cancel_event=cancel_event)
            # No blocking topic read is appended to the bounded service call.
            # The old optional state read could be absent; do not infer state
            # from an acknowledgement or claim that the base has stopped.
            data = {"state_after": None}
            if response.success:
                return Result.ok(f"enable_vel_control: {response.message}", data=data)
            return Result.fail(f"enable_vel_control failed: {response.message}",
                               error_code="JIBOT_VEL_CONTROL_FAILED", data=data)
        except Exception as exc:
            return Result.fail(f"enable_vel_control error: {exc}",
                               error_code="JIBOT_SERVICE_ERROR")
