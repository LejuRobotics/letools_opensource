# -*- coding: utf-8 -*-
"""GraspMtbfWheelWalk 单元测试。"""
import os
import pytest

pytestmark = pytest.mark.unit
from unittest.mock import patch, MagicMock
from py_trees.common import Status
import py_trees

from orchestration.nodes.grasp_mtbf_v1.GraspMtbfWheelWalk import GraspMtbfWheelWalk
from core.domain.result import Result


@pytest.mark.unit
def test_dry_run_returns_success():
    """干跑 → SUCCESS。"""
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        node = GraspMtbfWheelWalk("w", "walk", "ns", {"walk_mode": "cmd_pos_world"})
        node.initialise()
        assert node.update() == Status.SUCCESS
    finally:
        del os.environ["STUDIO_DRY_RUN"]


def test_real_run_cmd_pos_world_calls_chassis_api():
    """非干跑 + walk_mode=cmd_pos_world + is_walk_goal_new=True → 调 send_world_position + SUCCESS。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    mock_hw = MagicMock()
    fake_goal = MagicMock(pos=(1.0, 0, 0), quat=(0, 0, 0, 1))
    fake_goal.get_euler.return_value = (0.0, 0.0, 0.0)
    mock_hw.set_ruckig_planner_params.return_value = Result.ok()
    mock_hw.send_world_position.return_value = Result.ok()
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfWheelWalk.get_scene_io", return_value=mock_hw):
        # 用 Blackboard.set 写(写后 storage 才会带 '/' 前缀,Node 读得见)
        bb = py_trees.blackboard.Blackboard()
        bb.set("is_walk_goal_new", True)
        bb.set("walk_goal", fake_goal)
        node = GraspMtbfWheelWalk("w", "walk", "ns", {"walk_mode": "cmd_pos_world"})
        result = node.update()
        # cmd_pos_world 是 1-shot 命令,SUCCESS 即"已发起"
        assert result == Status.SUCCESS
        mock_hw.send_world_position.assert_called_once_with(1.0, 0.0, 0.0)


@pytest.mark.unit
def test_real_run_no_new_goal_returns_running():
    """非干跑 + is_walk_goal_new=False → RUNNING(等新目标)。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    mock_hw = MagicMock()
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfWheelWalk.get_scene_io", return_value=mock_hw):
        py_trees.blackboard.Blackboard().set("is_walk_goal_new", False)
        node = GraspMtbfWheelWalk("w", "walk", "ns", {"walk_mode": "cmd_pos_world"})
        result = node.update()
        assert result == Status.RUNNING
        mock_hw.send_world_position.assert_not_called()


@pytest.mark.parametrize("failed_step", ["limits", "command"])
def test_failed_control_result_does_not_consume_goal(monkeypatch, failed_step):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hw = MagicMock()
    hw.set_ruckig_planner_params.return_value = Result.fail("limits") if failed_step == "limits" else Result.ok()
    hw.send_world_position.return_value = Result.fail("command")
    goal = MagicMock(pos=(1.0, 0.0, 0.0))
    goal.get_euler.return_value = (0.0, 0.0, 0.0)
    bb = py_trees.blackboard.Blackboard()
    bb.set("is_walk_goal_new", True)
    bb.set("walk_goal", goal)
    with patch("orchestration.nodes.grasp_mtbf_v1.GraspMtbfWheelWalk.get_scene_io", return_value=hw):
        node = GraspMtbfWheelWalk("w", "walk", "ns", {"walk_mode": "cmd_pos_world"})
        node.initialise()
        assert node.update() == Status.FAILURE
    assert bb.get("is_walk_goal_new") is True
    if failed_step == "limits":
        hw.send_world_position.assert_not_called()
