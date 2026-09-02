# -*- coding: utf-8 -*-
"""wheel_arm_single_tag_pick_v1 的源 SDK 语义 Tag→joint 节点（基于 SingleTagJointPickPlanSkill）。"""

from __future__ import annotations

import math
import os
from typing import Any, Optional

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from skills.atomic.refactored_sdk.single_tag_joint_pick_plan import (
    SingleTagJointPickPlanSkill,
    SingleTagJointPickPlanParams,
)


class NodeSourceTagToArmGoalSingleTag(BaseAction):
    """通过 SingleTagJointPickPlanSkill 进行抓取轨迹规划，并在成功后原子写入双臂多帧轨迹。"""

    def __init__(self, name: str, label: str, namespace: str, params: dict[str, Any]):
        super().__init__(name, label, namespace, params)
        self._tag_id = self._parse_tag_id(params.get("tag_id", 1))
        self._tag_version_seen: Optional[int] = None
        self._skill: Optional[SingleTagJointPickPlanSkill] = None
        for key in (
            f"latest_tag_{self._tag_id}",
            f"latest_tag_{self._tag_id}_version",
        ):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.READ
            )
        for key in ("left_arm_joint_traj", "right_arm_joint_traj"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE
            )

    @staticmethod
    def _parse_tag_id(value: Any) -> int:
        if isinstance(value, str) and value.startswith("${"):
            raise ValueError(f"unresolved tag_id macro: {value}")
        return int(value)

    def initialise(self) -> None:
        self._skill = SingleTagJointPickPlanSkill(get_shared_hardware())
        self._tag_version_seen = None

    def update(self) -> Status:
        if os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes"):
            return Status.SUCCESS
        tag = self._read_blackboard(f"latest_tag_{self._tag_id}")
        version = self._read_blackboard(f"latest_tag_{self._tag_id}_version")
        if tag is None or version is None:
            self.feedback_message = "waiting for a new tag"
            return Status.RUNNING
        parsed_version = self._parse_version(version)
        if parsed_version is None:
            self.feedback_message = f"invalid tag version: {version!r}"
            return Status.FAILURE
        if self._tag_version_seen is not None and parsed_version <= self._tag_version_seen:
            return Status.RUNNING
        if not self._valid_tag(tag):
            self.feedback_message = "invalid tag data"
            return Status.FAILURE

        if self._skill is None:
            self._skill = SingleTagJointPickPlanSkill(get_shared_hardware())

        plan_params = SingleTagJointPickPlanParams(
            tag_pose=tag.pose_in_world,
            box_width=float(self.params.get("box_width", 0.35)),
            box_behind_tag=float(self.params.get("box_behind_tag", 0.0)),
            box_beneath_tag=float(self.params.get("box_beneath_tag", 0.0)),
            box_left_tag=float(self.params.get("box_left_tag", 0.0)),
            hand_pitch_degree=float(self.params.get("hand_pitch_degree", 0.0)),
            traj_point_num=int(self.params.get("traj_point_num", 100)),
            ik_retry_count=5,
            enable_joint_mirroring=True,
            enable_high_position_accuracy=False,
        )

        init_res = self._skill.initialize(plan_params)
        if not init_res.success:
            self.feedback_message = f"skill init failed: {init_res.message}"
            return Status.FAILURE

        plan_res = self._skill.execute()
        if not plan_res.success or not plan_res.data:
            self.feedback_message = (
                "source joint planning failed: "
                f"{plan_res.message}; no partial trajectory written"
            )
            return Status.FAILURE

        left = plan_res.data.get("left_arm_joint_traj")
        right = plan_res.data.get("right_arm_joint_traj")
        if left is None or right is None:
            self.feedback_message = "planning skill returned incomplete trajectory data"
            return Status.FAILURE

        self.global_blackboard.left_arm_joint_traj = left
        self.global_blackboard.right_arm_joint_traj = right
        self._tag_version_seen = parsed_version
        self.feedback_message = f"planned {len(left)} bimanual trajectory frames"
        return Status.SUCCESS

    @staticmethod
    def _parse_version(value: Any) -> Optional[int]:
        """将黑板中的 tag version 安全转为有限整数；非法时返回 None。"""
        try:
            parsed = int(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(float(parsed)):
            return None
        return parsed

    def _read_blackboard(self, key: str) -> Any:
        try:
            return getattr(self.global_blackboard, key)
        except (AttributeError, KeyError):
            return None

    @staticmethod
    def _valid_tag(tag: Any) -> bool:
        pose = getattr(tag, "pose_in_world", None)
        if pose is None:
            return False
        values = (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)
        return all(isinstance(value, (int, float)) and math.isfinite(value) for value in values)

