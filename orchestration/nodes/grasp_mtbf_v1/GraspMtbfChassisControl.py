"""Scene-only bounded JiBot control handover (not measured base stopping)."""
from py_trees.common import Status
from orchestration.utils.manifest_decorators import define_manifest
from .GraspMtbfMoveToTarget import _JibotServiceAction, _DRY_RUN, _as_bool, _positive
from .scene_io import get_scene_io


@define_manifest(
    label="MTBF底盘控制权切换", category=["motion", "grasp_mtbf_v1"],
    tree_type="grasp_mtbf_v1", description="有界调用enable_vel_control，不据此推断实际停止",
    params=[{"name": "enable", "type": "bool", "default": True}], inputs=[], outputs=[],
)
class GraspMtbfChassisControl(_JibotServiceAction):
    def initialise(self):
        self._reset_call()
        self._done = False
        if _DRY_RUN or self._failed:
            return
        self._hardware = get_scene_io()
        try:
            self._enable = _as_bool(self.params.get("enable", True))
            self._service_call_timeout = _positive(
                self.params.get("service_call_timeout", 5.0), "service_call_timeout")
        except (TypeError, ValueError) as exc:
            self._fail(f"control handover parameter error: {exc}")

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS
        if self._failed:
            return Status.FAILURE
        if self._done:
            return Status.SUCCESS
        if self._flight is None:
            self._start_call(lambda cancel, remaining: self._hardware.enable_vel_control_jibot(
                self._enable, cancel_event=cancel, timeout=remaining), self._service_call_timeout)
            return Status.RUNNING
        result = self._poll_call()
        if result is None:
            return Status.RUNNING
        if not result.success:
            # Repeating a timed-out restore through the same gate is unsafe and
            # would be rejected. Only a failed release needs the separate gate.
            if self._enable:
                self._restore_attempted = True
            return self._fail(result.message)
        self._done = True
        self.feedback_message = f"enable_vel_control={self._enable} request acknowledged"
        return Status.SUCCESS

    def terminate(self, new_status):
        if not _DRY_RUN and new_status != Status.SUCCESS:
            if getattr(self, "_enable", False):
                self._restore_attempted = True
            self._fail(self._failed or "JiBot control handover interrupted")
