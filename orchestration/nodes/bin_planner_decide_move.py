# -*- coding: utf-8 -*-
"""Call the bin planner vision service and write its decision to blackboard."""

import json
import os

import py_trees
from py_trees.common import Status

from module_internal.bin_planner import BinPlannerClient
from module_internal.bin_planner.config import DECISION_KEY, DEFAULT_SERVICE_NAME
from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="BinPlanner视觉决策",
    category=["perception", "depalletize_bin_v1_internal"],
    tree_type="studio_smoke",
    description="Call /lingbot/run_decide_with_stack_pose and save sequence/face/zone.",
    params=[
        {"name": "service_name", "type": "string", "default": DEFAULT_SERVICE_NAME, "description": "ROS service name"},
        {"name": "front_x", "type": "float", "default": "1.05", "description": "stack pose x used by vision service"},
        {"name": "front_y", "type": "float", "default": "0.0", "description": "stack pose y used by vision service"},
        {"name": "yaw_deg", "type": "float", "default": "0.0", "description": "stack yaw used by vision service"},
        {"name": "observe_side", "type": "string", "default": "front", "description": "observation side used for navigation lookup"},
        {"name": "step_index", "type": "int", "default": "0", "description": "sequence step selected for this navigation"},
        {"name": "timeout_sec", "type": "float", "default": "20.0", "description": "service wait timeout"},
        {"name": "decision_key", "type": "string", "default": DECISION_KEY, "description": "blackboard output key"},
    ],
    inputs=[],
    outputs=[],
)
class BinPlannerDecideMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False

    def initialise(self):
        self._done = False
        key = str(self.params.get("decision_key", DECISION_KEY))
        self.global_blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def update(self):
        if self._done:
            return Status.SUCCESS

        key = str(self.params.get("decision_key", DECISION_KEY))
        try:
            client = BinPlannerClient(
                service_name=str(self.params.get("service_name", DEFAULT_SERVICE_NAME)),
                timeout_sec=float(self.params.get("timeout_sec", 20.0)),
            )
            decision = client.run_decide_with_stack_pose(
                front_x=float(self.params.get("front_x", 1.05)),
                front_y=float(self.params.get("front_y", 0.0)),
                yaw_deg=float(self.params.get("yaw_deg", 0.0)),
                step_index=int(self.params.get("step_index", 0)),
            )
            payload = {
                "sequence": decision.sequence,
                "sequence_length": len(decision.sequence),
                "selected_index": decision.selected_index,
                "face": decision.face,
                "zone": decision.zone,
                "side": str(self.params.get("observe_side", decision.side)),
                "action": decision.action,
                "raw": decision.raw,
            }
            setattr(self.global_blackboard, key, payload)
            self.feedback_message = "bin planner decision=" + json.dumps(payload, ensure_ascii=False)
            self._done = True
            return Status.SUCCESS
        except Exception as exc:  # noqa: BLE001 - behavior tree should surface the exact boundary error
            self.feedback_message = f"bin planner decision failed: {exc}"
            return Status.FAILURE
