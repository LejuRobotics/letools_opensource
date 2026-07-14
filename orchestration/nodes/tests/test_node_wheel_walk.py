# -*- coding: utf-8 -*-
"""NodeWheelWalk 单元测试。"""
import os
import pytest
from unittest.mock import patch, MagicMock
from py_trees.common import Status
import py_trees

from orchestration.nodes.node_wheel_walk import NodeWheelWalk


@pytest.mark.unit
def test_dry_run_returns_success():
    """干跑 → SUCCESS。"""
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        node = NodeWheelWalk("w", "walk", "ns", {"walk_mode": "cmd_pos_world"})
        node.initialise()
        assert node.update() == Status.SUCCESS
    finally:
        del os.environ["STUDIO_DRY_RUN"]


def test_real_run_cmd_pos_world_calls_chassis_api():
    """非干跑 + walk_mode=cmd_pos_world + is_walk_goal_new=True → 调 chassis_cmd_pos_world + SUCCESS。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    mock_hw = MagicMock()
    fake_goal = MagicMock(pos=(1.0, 0, 0), quat=(0, 0, 0, 1))
    with patch("orchestration.nodes.node_wheel_walk.get_shared_hardware", return_value=mock_hw):
        # 用 Blackboard.set 写(写后 storage 才会带 '/' 前缀,Node 读得见)
        bb = py_trees.blackboard.Blackboard()
        bb.set("is_walk_goal_new", True)
        bb.set("walk_goal", fake_goal)
        node = NodeWheelWalk("w", "walk", "ns", {"walk_mode": "cmd_pos_world"})
        result = node.update()
        # cmd_pos_world 是 1-shot 命令,SUCCESS 即"已发起"
        assert result == Status.SUCCESS
        mock_hw.control.chassis_cmd_pos_world.assert_called_once()


@pytest.mark.unit
def test_real_run_no_new_goal_returns_running():
    """非干跑 + is_walk_goal_new=False → RUNNING(等新目标)。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    mock_hw = MagicMock()
    with patch("orchestration.nodes.node_wheel_walk.get_shared_hardware", return_value=mock_hw):
        py_trees.blackboard.Blackboard().set("is_walk_goal_new", False)
        node = NodeWheelWalk("w", "walk", "ns", {"walk_mode": "cmd_pos_world"})
        result = node.update()
        assert result == Status.RUNNING
        mock_hw.control.chassis_cmd_pos_world.assert_not_called()
