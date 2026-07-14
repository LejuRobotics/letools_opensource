# -*- coding: utf-8 -*-
"""SetBackWalkGoal:固定写 walk_goal=(0,0,0) 触发回原点(spec § 2.3 步骤 7)。"""
import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction


class SetBackWalkGoal(BaseAction):
    """固定写 walk_goal=zero pose + is_walk_goal_new=True。"""

    def __init__(self, name, label, namespace, params):
        super(SetBackWalkGoal, self).__init__(name, label, namespace, params)
        self.global_blackboard.register_key(key="walk_goal", access=py_trees.common.Access.WRITE)
        self.global_blackboard.register_key(
            key="is_walk_goal_new", access=py_trees.common.Access.WRITE
        )

    def initialise(self):
        pass

    def update(self):
        from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose
        setattr(
            self.global_blackboard, "walk_goal",
            Pose(pos=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0), frame="odom"),
        )
        setattr(self.global_blackboard, "is_walk_goal_new", True)
        self.feedback_message = "set walk_goal=(0,0,0)"
        return Status.SUCCESS
