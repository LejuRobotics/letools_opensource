# -*- coding: utf-8 -*-
"""Execute one-side bin-planner sequences with chassis navigation only.

The legacy BinPlannerNavTorsoHead flow plans one observed side at a time. One
vision call may return several pick candidates in ``sequence``. This node keeps
that behavior in LeTools: observe one side, execute every planned work pose with
a place trip after each fake grasp, then switch directly to the opposite
observation pose. Each side is therefore observed once per front/rear cycle.
The loop finishes only after both sides report slice heights below the
configured threshold.
"""

import json
import math
import os
import time
from typing import Any, Dict, Tuple

import py_trees
from py_trees.common import Status

from module_internal.bin_planner import BinPlannerClient
from module_internal.bin_planner.config import DECISION_KEY, DEFAULT_SERVICE_NAME, NAV_GOAL_KEY
from module_internal.bin_planner.domain_types import BinPlannerDecision, NavigationPose
from module_internal.bin_planner.parser import resolve_nav_pose
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.base_move_to_target_jibot import (
    BaseMoveToTargetJibotParams,
    BaseMoveToTargetJibotSkill,
)
from skills.atomic.refactored_sdk.check_arrived_jibot import (
    CheckArrivedJibotParams,
    CheckArrivedJibotSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes", "on")
_SKIP_CHASSIS_NAV = os.environ.get("DEPALLETIZE_SKIP_CHASSIS_NAV", "").lower() in (
    "1",
    "true",
    "yes",
    "on",
)
_SKIP_TF_WAIT = os.environ.get("DEPALLETIZE_SKIP_TF_WAIT", "").lower() in ("1", "true", "yes", "on")


def _as_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _as_float(value: Any, default: float = 0.0) -> float:
    if value in (None, ""):
        return float(default)
    return float(value)


def _deg_to_rad(deg: float) -> float:
    return float(deg) * math.pi / 180.0


def _load_pose_table(value: Any) -> Dict[str, Any]:
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return {}
        return json.loads(stripped)
    return {}


def _extract_face_zone(step: Dict[str, Any]) -> Tuple[int, int]:
    payload = step.get("result") or step.get("decision") or step
    if isinstance(payload, str):
        payload = json.loads(payload)
    if not isinstance(payload, dict):
        raise TypeError(f"sequence item payload must be a mapping, got {type(payload)!r}")
    face = payload.get("face", payload.get("face_id"))
    zone = payload.get("zone", payload.get("zone_id"))
    if face in (None, "") or zone in (None, ""):
        raise ValueError(f"sequence item missing face/zone: {step}")
    return int(face), int(zone)


@define_manifest(
    label="BinPlanner chassis sequence loop",
    category=["motion", "chassis", "perception", "depalletize_bin_v1_internal"],
    tree_type="studio_smoke",
    description="Observe front/rear, run bin planner once per side, and navigate every sequence item with fake grasp/place.",
    params=[
        {"name": "obs_front_x", "type": "float", "default": "0.0", "description": "front observe x"},
        {"name": "obs_front_y", "type": "float", "default": "0.0", "description": "front observe y"},
        {"name": "obs_front_theta_deg", "type": "float", "default": "0.0", "description": "front observe yaw deg"},
        {"name": "obs_rear_x", "type": "float", "default": "0.0", "description": "rear observe x"},
        {"name": "obs_rear_y", "type": "float", "default": "0.0", "description": "rear observe y"},
        {"name": "obs_rear_theta_deg", "type": "float", "default": "0.0", "description": "rear observe yaw deg"},
        {"name": "observe_side", "type": "string", "default": "front", "description": "first observe side"},
        {"name": "service_name", "type": "string", "default": DEFAULT_SERVICE_NAME, "description": "bin planner service"},
        {"name": "service_timeout_sec", "type": "float", "default": "120.0", "description": "bin planner wait timeout"},
        {"name": "decide_front_x", "type": "float", "default": "1.05", "description": "front stack pose x for vision service"},
        {"name": "decide_front_y", "type": "float", "default": "0.0", "description": "front stack pose y for vision service"},
        {"name": "decide_yaw_deg", "type": "float", "default": "0.0", "description": "front stack yaw for vision service"},
        {"name": "decide_rear_front_x", "type": "float", "default": "", "description": "optional rear stack pose x"},
        {"name": "decide_rear_front_y", "type": "float", "default": "", "description": "optional rear stack pose y"},
        {"name": "decide_rear_yaw_deg", "type": "float", "default": "", "description": "optional rear stack yaw"},
        {"name": "camera_tf_target_frame", "type": "string", "default": "base_link", "description": "robot frame"},
        {"name": "camera_tf_source_frame", "type": "string", "default": "camera_color_optical_frame", "description": "camera optical frame"},
        {"name": "camera_tf_timeout_sec", "type": "float", "default": "15.0", "description": "tf timeout"},
        {"name": "observe_nav_settle_s", "type": "float", "default": "0.6", "description": "wait after arriving at an observation pose"},
        {"name": "max_consecutive_rear_only", "type": "int", "default": "2", "description": "stop after repeated contradictory rear_only results"},
        {"name": "nav_pose_table", "type": "json", "default": "{}", "description": "face/zone to chassis pose table"},
        {"name": "fake_grasp_enabled", "type": "bool", "default": "True", "description": "skip real grasp and write grasp_done"},
        {"name": "place_x", "type": "float", "default": "0.0", "description": "place x"},
        {"name": "place_y", "type": "float", "default": "0.0", "description": "place y"},
        {"name": "place_theta_deg", "type": "float", "default": "0.0", "description": "place yaw deg"},
        {"name": "place_goal_label", "type": "string", "default": "place_pose", "description": "place goal label"},
        {"name": "enable_slice_height_finish", "type": "bool", "default": "True", "description": "finish when slice heights are below threshold"},
        {"name": "finish_slice_height_threshold", "type": "float", "default": "0.5", "description": "finish slice height threshold"},
        {"name": "nav_avoid_enabled", "type": "bool", "default": "False", "description": "avoid enabled"},
        {"name": "nav_avoid_distance", "type": "float", "default": "0.5", "description": "avoid distance"},
        {"name": "nav_linear_velocity", "type": "float", "default": "1.0", "description": "linear velocity"},
        {"name": "nav_angular_velocity", "type": "float", "default": "1.0", "description": "angular velocity"},
        {"name": "nav_position_threshold", "type": "float", "default": "0.03", "description": "position threshold"},
        {"name": "nav_angle_threshold", "type": "float", "default": "0.03", "description": "angle threshold"},
        {"name": "nav_allow_rotation", "type": "bool", "default": "True", "description": "allow rotation"},
        {"name": "nav_arrival_timeout_sec", "type": "float", "default": "120.0", "description": "arrival timeout"},
    ],
    inputs=[],
    outputs=[
        {"name": DECISION_KEY, "type": "dict", "description": "current selected sequence item"},
        {"name": NAV_GOAL_KEY, "type": "dict", "description": "current chassis goal"},
        {"name": "grasp_done", "type": "bool", "description": "fake grasp completion flag"},
        {"name": "binplanner_sequence_state", "type": "dict", "description": "current side and sequence progress"},
    ],
)
class BinPlannerChassisSequenceLoopMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._phase = "idle"
        self._sides = []
        self._side_i = 0
        self._sequence = []
        self._sequence_raw = {}
        self._seq_i = 0
        self._side = "front"
        self._observe_count = 0
        self._tf_buffer = None
        self._tf_listener = None
        self._tf_started_at = 0.0
        self._settle_until = 0.0
        self._consecutive_rear_only = 0
        self._completed_sides = set()
        self._rospy = None
        self._finished = False

    def initialise(self):
        first_side = str(self.params.get("observe_side", "front") or "front").strip().lower()
        if first_side not in ("front", "rear"):
            first_side = "front"
        self._sides = [first_side]
        for side in ("front", "rear"):
            if side not in self._sides:
                self._sides.append(side)
        self._side_i = 0
        self._side = first_side
        self._observe_count = 0
        self._sequence = []
        self._sequence_raw = {}
        self._seq_i = 0
        self._phase = "nav_observe"
        self._tf_buffer = None
        self._tf_listener = None
        self._tf_started_at = 0.0
        self._settle_until = 0.0
        self._consecutive_rear_only = 0
        self._completed_sides = set()
        self._finished = False
        self._write_state("start")

    def update(self):
        if self._finished:
            return Status.SUCCESS
        try:
            while True:
                if self._phase == "nav_observe":
                    self._nav_to_observe_pose()
                    settle_s = max(
                        0.0,
                        _as_float(self.params.get("observe_nav_settle_s", 0.6), 0.6),
                    )
                    self._settle_until = time.monotonic() + settle_s
                    self._phase = "wait_observe_settle"
                    self._write_state("observe_arrived")
                    return Status.RUNNING

                if self._phase == "wait_observe_settle":
                    remaining = self._settle_until - time.monotonic()
                    if remaining > 0.0:
                        self.feedback_message = (
                            f"waiting for observation pose to settle ({remaining:.2f}s)"
                        )
                        return Status.RUNNING
                    self._phase = "wait_tf"
                    self._start_tf_wait()
                    self._write_state("observe_settled")
                    continue

                if self._phase == "wait_tf":
                    if not self._tf_ready():
                        return Status.RUNNING
                    self._phase = "decide"
                    self._write_state("tf_ready")
                    continue

                if self._phase == "decide":
                    decision = self._call_planner_for_side(self._current_side())
                    self._observe_count += 1
                    self._sequence = list(decision.sequence or [])
                    self._sequence_raw = dict(decision.raw or {})
                    self._seq_i = 0
                    action = str(decision.action or "")
                    self._log_decision_summary(decision)
                    if self._should_finish_by_slice_threshold(decision.raw):
                        completed_side = self._current_side()
                        self._completed_sides.add(completed_side)
                        if len(self._completed_sides) < 2:
                            self._log_info(
                                f"[BinPlannerChassisSequenceLoop] side={completed_side} "
                                "slice heights below threshold; switch observation side"
                            )
                            self._advance_side()
                            self._phase = "nav_observe"
                            self._write_state("side_completed_switch_side")
                            continue
                        self._log_info(
                            "[BinPlannerChassisSequenceLoop] finish: front and rear "
                            "slice heights are both below threshold"
                        )
                        self._finished = True
                        self._write_state("completed")
                        return Status.SUCCESS
                    self._completed_sides.discard(self._current_side())
                    if action == "rear_only":
                        previous_side = self._current_side()
                        self._consecutive_rear_only += 1
                        max_rear_only = max(
                            1,
                            int(
                                _as_float(
                                    self.params.get("max_consecutive_rear_only", 2),
                                    2,
                                )
                            ),
                        )
                        if self._consecutive_rear_only >= max_rear_only:
                            raise RuntimeError(
                                "planner returned rear_only on consecutive opposite-side "
                                f"observations (last_side={previous_side}, "
                                f"count={self._consecutive_rear_only}); stop to avoid "
                                "front/rear oscillation. Check camera framing and rear "
                                "stack-pose calibration."
                            )
                        self._log_info(
                            f"[BinPlannerChassisSequenceLoop] side={previous_side} "
                            "action=rear_only, switch observation side"
                        )
                        self._advance_side()
                        self._phase = "nav_observe"
                        self._write_state("rear_only_switch_side")
                        continue
                    if not self._sequence:
                        self._consecutive_rear_only = 0
                        previous_side = self._current_side()
                        self._log_info(
                            f"[BinPlannerChassisSequenceLoop] side={previous_side} "
                            f"action={action or '<empty>'} sequence empty; "
                            "switch observation side"
                        )
                        self._advance_side()
                        self._phase = "nav_observe"
                        self._write_state("empty_sequence_switch_side")
                        continue
                    self._consecutive_rear_only = 0
                    self._log_info(
                        f"[BinPlannerChassisSequenceLoop] side={self._current_side()} planner returned {len(self._sequence)} steps"
                    )
                    self._phase = "exec_sequence"
                    self._write_state("sequence_ready")
                    continue

                if self._phase == "exec_sequence":
                    if self._seq_i >= len(self._sequence):
                        previous_side = self._current_side()
                        self._advance_side()
                        self._log_info(
                            f"[BinPlannerChassisSequenceLoop] side={previous_side} "
                            f"sequence completed; switch to {self._current_side()} observation"
                        )
                        self._phase = "nav_observe"
                        self._write_state("sequence_completed_switch_side")
                        continue
                    self._run_one_sequence_item(self._seq_i)
                    self._seq_i += 1
                    self._write_state("sequence_step_completed")
                    return Status.RUNNING

                raise RuntimeError(f"unknown phase: {self._phase}")
        except Exception as exc:
            self.feedback_message = str(exc)
            self._log_error(f"[BinPlannerChassisSequenceLoop] failed: {exc}")
            return Status.FAILURE

    def _current_side(self) -> str:
        return self._side

    def _advance_side(self):
        self._side = "rear" if self._side == "front" else "front"
        self._side_i = 0 if self._side == "front" else 1
        self._sequence = []
        self._sequence_raw = {}
        self._seq_i = 0

    def _log_decision_summary(self, decision: BinPlannerDecision):
        raw = decision.raw if isinstance(decision.raw, dict) else {}
        summary = {
            "side": self._current_side(),
            "action": str(decision.action or ""),
            "sequence_len": len(decision.sequence or []),
            "result": raw.get("result"),
            "slice1_abcd_heights": raw.get("slice1_abcd_heights"),
            "slice2_abcd_heights": raw.get("slice2_abcd_heights"),
            "reason": (raw.get("intermediate") or {}).get("reason")
            if isinstance(raw.get("intermediate"), dict)
            else raw.get("reason"),
        }
        self._log_info(
            "[BinPlannerChassisSequenceLoop] decision: "
            + json.dumps(summary, ensure_ascii=False, sort_keys=True)
        )

    def _extract_slice_peak_values(self, data: Dict[str, Any]):
        values = []

        def collect(value: Any, path):
            if isinstance(value, dict):
                for key, child in value.items():
                    collect(child, path + [str(key).lower()])
                return
            if isinstance(value, list):
                for index, child in enumerate(value):
                    collect(child, path + [str(index)])
                return
            if value in (None, ""):
                return
            try:
                number = float(value)
            except (TypeError, ValueError):
                return
            joined = "_".join(path)
            has_slice = "slice" in joined
            has_cell = any(cell in path for cell in ("a", "b", "c", "d"))
            has_height = "height" in joined
            if has_slice and has_cell and has_height:
                values.append(number)

        collect(data or {}, [])
        return values

    def _should_finish_by_slice_threshold(self, data: Dict[str, Any]) -> bool:
        if not _as_bool(self.params.get("enable_slice_height_finish", True)):
            return False
        values = self._extract_slice_peak_values(data)
        if not values:
            self._log_info("[BinPlannerChassisSequenceLoop] finish check skipped: no slice heights")
            return False
        threshold = _as_float(self.params.get("finish_slice_height_threshold", 0.5), 0.5)
        max_height = max(values)
        self._log_info(
            f"[BinPlannerChassisSequenceLoop] finish check: points={len(values)}, max={max_height:.4f}, threshold={threshold:.4f}"
        )
        return max_height < threshold

    def _observe_pose(self, side: str) -> NavigationPose:
        prefix = "obs_rear" if side == "rear" else "obs_front"
        return NavigationPose(
            x=_as_float(self.params.get(f"{prefix}_x", 0.0)),
            y=_as_float(self.params.get(f"{prefix}_y", 0.0)),
            theta_deg=_as_float(self.params.get(f"{prefix}_theta_deg", 0.0)),
        )

    def _nav_to_observe_pose(self):
        side = self._current_side()
        pose = self._observe_pose(side)
        self._log_info(
            f"[BinPlannerChassisSequenceLoop] nav to {side} observe pose: x={pose.x:.3f}, y={pose.y:.3f}, theta={pose.theta_deg:.1f}deg"
        )
        self._run_nav_pose(pose, f"observe_{side}")

    def _call_planner_for_side(self, side: str) -> BinPlannerDecision:
        front_x = _as_float(self.params.get("decide_front_x", 1.05), 1.05)
        front_y = _as_float(self.params.get("decide_front_y", 0.0), 0.0)
        yaw_deg = _as_float(self.params.get("decide_yaw_deg", 0.0), 0.0)
        if side == "rear":
            front_x = _as_float(self.params.get("decide_rear_front_x", front_x), front_x)
            front_y = _as_float(self.params.get("decide_rear_front_y", front_y), front_y)
            yaw_deg = _as_float(self.params.get("decide_rear_yaw_deg", yaw_deg), yaw_deg)

        if _DRY_RUN:
            sequence = [
                {"result": {"face": 2, "zone": 1}, "sentence": f"dry-run {side} step 1"},
                {"result": {"face": 1, "zone": 2}, "sentence": f"dry-run {side} step 2"},
            ] if side == "front" else [
                {"result": {"face": 0, "zone": 1}, "sentence": "dry-run rear step 1"},
            ]
            return BinPlannerDecision(
                sequence=sequence,
                selected_index=None,
                face=None,
                zone=None,
                side=side,
                action="execute_sequence",
                raw={"sequence": sequence, "side": side, "action": "execute_sequence", "dry_run": True},
            )

        self._log_info(
            f"[BinPlannerChassisSequenceLoop] call bin planner side={side}: front_x={front_x:.3f}, front_y={front_y:.3f}, yaw_deg={yaw_deg:.1f}"
        )
        client = BinPlannerClient(
            service_name=str(self.params.get("service_name", DEFAULT_SERVICE_NAME)),
            timeout_sec=_as_float(self.params.get("service_timeout_sec", 120.0), 120.0),
        )
        decision = client.run_decide_with_stack_pose(front_x, front_y, yaw_deg, step_index=0)
        return BinPlannerDecision(
            sequence=list(decision.sequence or []),
            selected_index=None,
            face=decision.face,
            zone=decision.zone,
            side=side,
            action=str(decision.action or ""),
            raw=dict(decision.raw or {}),
        )

    def _run_one_sequence_item(self, index: int):
        side = self._current_side()
        step = self._sequence[index]
        face, zone = _extract_face_zone(step)
        selected = BinPlannerDecision(
            sequence=list(self._sequence),
            selected_index=index,
            face=face,
            zone=zone,
            side=side,
            action="execute_sequence",
            raw=dict(self._sequence_raw),
        )
        table = self._get_pose_table()
        work_pose = resolve_nav_pose(selected, table)
        self._write_blackboard(DECISION_KEY, self._decision_to_dict(selected))
        self._write_nav_goal(work_pose, f"{side}.face_{face}.zone_{zone}")
        self._log_info(
            f"[BinPlannerChassisSequenceLoop] step {index + 1}/{len(self._sequence)} side={side} face={face} zone={zone} -> work x={work_pose.x:.3f}, y={work_pose.y:.3f}, theta={work_pose.theta_deg:.1f}deg"
        )
        self._run_nav_pose(work_pose, f"work_{side}_face_{face}_zone_{zone}")
        self._fake_grasp(selected)
        place_pose = NavigationPose(
            x=_as_float(self.params.get("place_x", 0.0)),
            y=_as_float(self.params.get("place_y", 0.0)),
            theta_deg=_as_float(self.params.get("place_theta_deg", 0.0)),
        )
        self._write_nav_goal(place_pose, str(self.params.get("place_goal_label", "place_pose")))
        self._run_nav_pose(place_pose, str(self.params.get("place_goal_label", "place_pose")))

    def _fake_grasp(self, decision: BinPlannerDecision):
        if not _as_bool(self.params.get("fake_grasp_enabled", True)):
            raise RuntimeError("fake_grasp_enabled is false; real grasp is not implemented in this chassis-only scenario")
        payload = {
            "skipped_real_grasp": True,
            "selected_index": decision.selected_index,
            "side": decision.side,
            "face": decision.face,
            "zone": decision.zone,
            "message": "fake grasp completed after reaching work pose",
        }
        self._write_blackboard("grasp_done", True)
        self._write_blackboard("grasp_completion", payload)
        self._log_info(
            f"[BinPlannerChassisSequenceLoop] fake grasp done: side={decision.side} face={decision.face} zone={decision.zone}"
        )

    def _run_nav_pose(self, pose: NavigationPose, label: str):
        if _DRY_RUN or _SKIP_CHASSIS_NAV:
            mode = "dry-run" if _DRY_RUN else "skip-chassis"
            self._log_info(
                f"[BinPlannerChassisSequenceLoop][{mode}] nav {label}: x={pose.x:.3f}, y={pose.y:.3f}, theta={pose.theta_deg:.1f}deg"
            )
            self._write_blackboard("current_task_id", f"{mode}:{label}")
            return

        hardware = get_shared_hardware()
        enable_result = hardware.enable_vel_control_jibot(False)
        if not enable_result.success:
            raise RuntimeError(enable_result.message or "enable_vel_control_jibot(False) failed")

        move_skill = BaseMoveToTargetJibotSkill(hardware=hardware)
        init_result = move_skill.initialize(
            BaseMoveToTargetJibotParams(
                x=float(pose.x),
                y=float(pose.y),
                theta=_deg_to_rad(pose.theta_deg),
                avoid_enabled=_as_bool(self.params.get("nav_avoid_enabled", False)),
                avoid_distance=_as_float(self.params.get("nav_avoid_distance", 0.5), 0.5),
                linear_velocity=_as_float(self.params.get("nav_linear_velocity", 1.0), 1.0),
                angular_velocity=_as_float(self.params.get("nav_angular_velocity", 1.0), 1.0),
                position_threshold=_as_float(self.params.get("nav_position_threshold", 0.03), 0.03),
                angle_threshold=_as_float(self.params.get("nav_angle_threshold", 0.03), 0.03),
                allow_rotation=_as_bool(self.params.get("nav_allow_rotation", True)),
            )
        )
        if not init_result.success:
            raise RuntimeError(init_result.message or f"move init failed for {label}")
        move_result = move_skill.execute()
        if not move_result.success:
            raise RuntimeError(move_result.message or f"move failed for {label}")
        data = move_result.data if isinstance(move_result.data, dict) else {}
        task_id = str(data.get("task_id", ""))
        self._write_blackboard("current_task_id", task_id)

        check_skill = CheckArrivedJibotSkill(hardware=hardware)
        init_result = check_skill.initialize(
            CheckArrivedJibotParams(
                task_id=task_id,
                blocking=True,
                timeout=_as_float(self.params.get("nav_arrival_timeout_sec", 120.0), 120.0),
            )
        )
        if not init_result.success:
            raise RuntimeError(init_result.message or f"arrival check init failed for {label}")
        check_result = check_skill.execute()
        if not check_result.success:
            raise RuntimeError(check_result.message or f"arrival check failed for {label}")
        check_data = check_result.data if isinstance(check_result.data, dict) else {}
        if not bool(check_data.get("arrived", False)):
            raise RuntimeError(f"navigation did not arrive for {label}: {check_data}")
        self._log_info(f"[BinPlannerChassisSequenceLoop] arrived {label}: {check_data.get('message', 'arrived')}")

    def _get_pose_table(self) -> Dict[str, Any]:
        table = _load_pose_table(self.params.get("nav_pose_table", {}))
        if table:
            return table
        try:
            self.global_blackboard.register_key(key="nav_pose_table", access=py_trees.common.Access.READ)
        except Exception:
            pass
        try:
            return _load_pose_table(self.global_blackboard.get("nav_pose_table"))
        except Exception:
            return {}

    def _start_tf_wait(self):
        self._tf_started_at = time.monotonic()
        self._tf_buffer = None
        self._tf_listener = None
        if _DRY_RUN or _SKIP_TF_WAIT:
            return
        import rospy
        import tf2_ros
        self._rospy = rospy
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def _tf_ready(self) -> bool:
        target = str(self.params.get("camera_tf_target_frame", "base_link"))
        source = str(self.params.get("camera_tf_source_frame", "camera_color_optical_frame"))
        timeout = _as_float(self.params.get("camera_tf_timeout_sec", 15.0), 15.0)
        if _DRY_RUN or _SKIP_TF_WAIT:
            mode = "dry-run" if _DRY_RUN else "skip-tf"
            self._log_info(f"[BinPlannerChassisSequenceLoop][{mode}] TF ready: {target} <- {source}")
            return True
        if self._tf_buffer and self._tf_buffer.can_transform(target, source, self._rospy.Time(0), self._rospy.Duration(0.1)):
            self._log_info(f"[BinPlannerChassisSequenceLoop] TF ready: {target} <- {source}")
            return True
        elapsed = time.monotonic() - self._tf_started_at
        if elapsed >= timeout:
            raise RuntimeError(f"TF unavailable after {elapsed:.1f}s: {target} <- {source}")
        self.feedback_message = f"waiting for TF: {target} <- {source} ({elapsed:.1f}/{timeout:.1f}s)"
        return False

    def _write_nav_goal(self, pose: NavigationPose, label: str):
        self._write_blackboard(
            NAV_GOAL_KEY,
            {"x": pose.x, "y": pose.y, "theta": pose.theta_deg, "theta_unit": "deg", "label": label},
        )

    def _write_state(self, event: str):
        self._write_blackboard(
            "binplanner_sequence_state",
            {
                "event": event,
                "phase": self._phase,
                "side": self._current_side() if self._side_i < len(self._sides) else "done",
                "side_index": self._side_i,
                "observe_count": self._observe_count,
                "sequence_index": self._seq_i,
                "sequence_len": len(self._sequence),
                "consecutive_rear_only": self._consecutive_rear_only,
            },
        )

    def _write_blackboard(self, key: str, value: Any):
        try:
            self.global_blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
        except Exception:
            pass
        try:
            self.global_blackboard.set(key, value)
        except Exception:
            try:
                setattr(self.global_blackboard, key, value)
            except Exception:
                pass

    def _decision_to_dict(self, decision: BinPlannerDecision) -> Dict[str, Any]:
        return {
            "sequence": decision.sequence,
            "selected_index": decision.selected_index,
            "face": decision.face,
            "zone": decision.zone,
            "side": decision.side,
            "action": decision.action,
            "raw": decision.raw,
        }

    def _log_info(self, message: str):
        try:
            import rospy
            rospy.loginfo(message)
        except Exception:
            print(message)

    def _log_error(self, message: str):
        try:
            import rospy
            rospy.logerr(message)
        except Exception:
            print(message)
