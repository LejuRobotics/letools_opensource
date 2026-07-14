# -*- coding: utf-8 -*-
"""NodeComputePickGoal:算底盘 ODOM 系目标位姿(spec § 3.1)。

输入:黑板 latest_tag_{id}(ODOM 系 TAG→world 变换,来自 PerceptionAdapter 订阅 _odom)
      + stand_in_tag_pos/euler(TAG 系相对偏移)
输出:黑板 walk_goal(ODOM 系底盘目标)+ is_walk_goal_new=True

公式:walk_goal = tag.pose_in_world.(x,y,z) + stand_in_tag_pos
"""
import os
import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class NodeComputePickGoal(BaseAction):
    """读 tag,算 walk_goal,写黑板。"""

    def __init__(self, name, label, namespace, params):
        super(NodeComputePickGoal, self).__init__(name, label, namespace, params)
        self._tag_id = int(self.params.get("tag_id", 0))
        # 注册需要读取的 tag key，避免 py_trees 2.x 中 KeyError
        self.global_blackboard.register_key(
            key=f"latest_tag_{self._tag_id}", access=py_trees.common.Access.READ
        )
        self.global_blackboard.register_key(
            key="walk_goal", access=py_trees.common.Access.WRITE
        )
        self.global_blackboard.register_key(
            key="is_walk_goal_new", access=py_trees.common.Access.WRITE
        )

    def initialise(self):
        pass

    def update(self):
        if _is_dry_run():
            self._write_walk_goal((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            return Status.SUCCESS

        tag = getattr(self.global_blackboard, f"latest_tag_{self._tag_id}", None)
        if tag is None:
            self.feedback_message = f"tag {self._tag_id} not on blackboard"
            return Status.RUNNING

        # tag.pose_in_world 是 Pose6D dataclass，字段 .x/.y/.z
        stand_pos = self.params.get("stand_in_tag_pos", [0.0, 0.0, 0.0])
        new_pos = (
            tag.pose_in_world.x + stand_pos[0],
            tag.pose_in_world.y + stand_pos[1],
            tag.pose_in_world.z + stand_pos[2],
        )
        self._write_walk_goal(new_pos, (0.0, 0.0, 0.0))
        return Status.SUCCESS

    def _write_walk_goal(self, pos, euler):
        from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose
        setattr(
            self.global_blackboard, "walk_goal",
            Pose(pos=pos, quat=(0.0, 0.0, 0.0, 1.0), frame="odom"),
        )
        setattr(self.global_blackboard, "is_walk_goal_new", True)
        self.feedback_message = f"walk_goal={pos}"
