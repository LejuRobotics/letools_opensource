# -*- coding: utf-8 -*-
"""SetBackWalkGoal 单元测试。"""
import pytest
from py_trees.common import Status

from orchestration.nodes.set_back_walk_goal import SetBackWalkGoal

pytestmark = pytest.mark.unit


def test_writes_zero_pose_to_blackboard():
    """固定写 walk_goal=(0,0,0) + is_walk_goal_new=True。"""
    node = SetBackWalkGoal("s", "set_back", "ns", {})
    node.initialise()
    assert node.update() == Status.SUCCESS
    goal = getattr(node.global_blackboard, "walk_goal", None)
    assert goal is not None
    # pos 是 numpy array;首 3 个元素应全为 0
    assert goal.pos[0] == 0.0
    assert goal.pos[1] == 0.0
    assert goal.pos[2] == 0.0
    assert getattr(node.global_blackboard, "is_walk_goal_new") is True
