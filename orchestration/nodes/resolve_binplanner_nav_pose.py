# -*- coding: utf-8 -*-
"""Resolve bin planner face/zone output into a chassis navigation pose."""

import json
import os

import py_trees
from py_trees.common import Status

from module_internal.bin_planner.config import DECISION_KEY, NAV_GOAL_KEY
from module_internal.bin_planner.parser import parse_decision, resolve_nav_pose
from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _load_table(value):
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        value = value.strip()
        if not value:
            return {}
        return json.loads(value)
    return {}


def _load_flat_table(params, prefix="pose_table_json"):
    flat = getattr(params, "params", {})
    marker = f"{prefix}."
    table = {}
    for key, value in flat.items():
        if not key.startswith(marker):
            continue
        remainder = key[len(marker):]
        if "." not in remainder:
            continue
        pose_key, field = remainder.rsplit(".", 1)
        table.setdefault(pose_key, {})[field] = value
    return table


@define_manifest(
    label="BinPlanner导航点查表",
    category=["logic", "depalletize_bin_v1_internal"],
    tree_type="studio_smoke",
    description="Read sequence/face/zone and write a map-frame chassis pose.",
    params=[
        {"name": "decision_key", "type": "string", "default": DECISION_KEY, "description": "blackboard input key"},
        {"name": "nav_goal_key", "type": "string", "default": NAV_GOAL_KEY, "description": "blackboard output key"},
        {"name": "pose_table_json", "type": "string", "default": "{\"face_0.zone_1\":{\"x\":0.226,\"y\":-0.452,\"theta_deg\":90.0}}", "description": "face/zone to map pose table"},
    ],
    inputs=[],
    outputs=[],
)
class ResolveBinplannerNavPose(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False

    def initialise(self):
        self._done = False
        self.global_blackboard.register_key(
            key=str(self.params.get("decision_key", DECISION_KEY)),
            access=py_trees.common.Access.READ,
        )
        self.global_blackboard.register_key(
            key=str(self.params.get("nav_goal_key", NAV_GOAL_KEY)),
            access=py_trees.common.Access.WRITE,
        )

    def update(self):
        if self._done:
            return Status.SUCCESS

        decision_key = str(self.params.get("decision_key", DECISION_KEY))
        nav_goal_key = str(self.params.get("nav_goal_key", NAV_GOAL_KEY))
        try:
            raw_decision = getattr(self.global_blackboard, decision_key)
            selected_index = int(raw_decision.get("selected_index", 0)) if isinstance(raw_decision, dict) else 0
            decision = parse_decision(raw_decision, step_index=selected_index)
            table = _load_table(self.params.get("pose_table_json", ""))
            if not table:
                table = _load_flat_table(self.params)
            pose = resolve_nav_pose(decision, table)
            payload = {
                "x": pose.x,
                "y": pose.y,
                "theta": pose.theta_deg,
                "theta_unit": "deg",
                "source_sequence": decision.sequence,
                "source_face": decision.face,
                "source_zone": decision.zone,
            }
            setattr(self.global_blackboard, nav_goal_key, payload)
            self.feedback_message = "nav_goal=" + json.dumps(payload, ensure_ascii=False)
            self._done = True
            return Status.SUCCESS
        except Exception as exc:  # noqa: BLE001
            self.feedback_message = f"resolve nav pose failed: {exc}"
            return Status.FAILURE
