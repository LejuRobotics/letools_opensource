# -*- coding: utf-8 -*-
"""NodeComputePickGoal 单元测试。"""
import os
import pytest
from unittest.mock import patch, MagicMock
from py_trees.common import Status

from orchestration.nodes.node_compute_pick_goal import NodeComputePickGoal

pytestmark = pytest.mark.unit


def test_dry_run_writes_zero_walk_goal():
    """干跑 → 写 walk_goal(0,0,0) + is_walk_goal_new=True。"""
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        node = NodeComputePickGoal("c", "compute", "ns", {
            "tag_id": 1,
            "stand_in_tag_pos": [0.0, 0.0, 0.55],
            "stand_in_tag_euler": [0.0, 0.0, 0.0],
        })
        node.initialise()
        result = node.update()
        assert result == Status.SUCCESS
        walk = getattr(node.global_blackboard, "walk_goal", None)
        assert walk is not None
        # walk_goal = (0,0,0) in dry-run
        assert walk.pos[0] == pytest.approx(0.0, abs=1e-6)
        assert walk.pos[2] == pytest.approx(0.0, abs=1e-6)
        assert getattr(node.global_blackboard, "is_walk_goal_new") is True
    finally:
        del os.environ["STUDIO_DRY_RUN"]
