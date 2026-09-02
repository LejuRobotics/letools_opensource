# -*- coding: utf-8 -*-
"""为单 Tag 场景写入不可变的虚拟 Tag 和版本。"""

from __future__ import annotations

import math
import time

import py_trees
from py_trees.common import Status

from core.domain.perception import TagDetection
from core.domain.pose import Pose6D
from orchestration.nodes.base_node import BaseAction


class NodeInjectSingleTag(BaseAction):
    """写入一次虚拟 Tag；不修改已有 Tag 对象。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        raw_tag_id = self.params.get("tag_id", 1)
        self._tag_id = self._parse_tag_id(raw_tag_id)
        self._use_virtual_tag = self._parse_bool(
            self.params.get("use_virtual_tag", True)
        )

        self.global_blackboard.register_key(
            key=f"latest_tag_{self._tag_id}",
            access=py_trees.common.Access.WRITE,
        )
        self.global_blackboard.register_key(
            key=f"latest_tag_{self._tag_id}_version",
            access=py_trees.common.Access.WRITE,
        )

    @staticmethod
    def _parse_tag_id(value):
        if isinstance(value, str) and value.startswith("${") and value.endswith("}"):
            raise ValueError(f"unresolved tag_id macro: {value}")
        return int(value)

    @staticmethod
    def _parse_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in ("1", "true", "yes"):
                return True
            if normalized in ("0", "false", "no"):
                return False
        raise ValueError(f"invalid use_virtual_tag value: {value!r}")

    def update(self):
        if not self._use_virtual_tag:
            return Status.SUCCESS
        pose = self.params.get("pose_in_odom", [0.5, 0.0, 0.75, 0.0, 0.0, 0.0])
        try:
            values = [float(value) for value in pose]
        except (TypeError, ValueError):
            self.feedback_message = "pose_in_odom must be numeric"
            return Status.FAILURE
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            self.feedback_message = "pose_in_odom must be 6 finite values"
            return Status.FAILURE
        tag = TagDetection(
            tag_id=self._tag_id,
            pose_in_world=Pose6D(
                x=values[0], y=values[1], z=values[2],
                yaw=values[3], pitch=values[4], roll=values[5],
            ),
            size=float(self.params.get("size", 0.1)),
            confidence=1.0,
            timestamp=time.time(),
            frame_id="odom",
        )
        setattr(self.global_blackboard, f"latest_tag_{self._tag_id}", tag)
        setattr(self.global_blackboard, f"latest_tag_{self._tag_id}_version", 1)
        return Status.SUCCESS
