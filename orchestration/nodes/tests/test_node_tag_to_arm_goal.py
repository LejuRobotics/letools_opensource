# -*- coding: utf-8 -*-
"""NodeTagToArmGoal 单元测试。"""
import os
import pytest
from unittest.mock import patch, MagicMock
from py_trees.common import Status

from orchestration.nodes.node_tag_to_arm_goal import NodeTagToArmGoal


@pytest.mark.unit
def test_dry_run_returns_success():
    """干跑 → SUCCESS。"""
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        node = NodeTagToArmGoal("a", "arm_goal", "ns", {
            "tag_id": 1,
            "control_type": "joint",
            "keypoints_source": "pick",
            "box_width": 0.35, "box_behind_tag": 0.0,
            "box_beneath_tag": 0.0, "box_left_tag": 0.0,
            "hand_pitch_degree": 0.0,
        })
        node.initialise()
        assert node.update() == Status.SUCCESS
    finally:
        del os.environ["STUDIO_DRY_RUN"]


@pytest.mark.unit
def test_initialise_calls_pick_keypoints():
    """initialise() 应按 keypoints_source='pick' 调 generate_pick_keypoints。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    with patch(
        "orchestration.nodes.node_tag_to_arm_goal.generate_pick_keypoints"
    ) as mock_pick:
        mock_pick.return_value = ([], [])
        node = NodeTagToArmGoal("a", "arm_goal", "ns", {
            "tag_id": 1, "control_type": "joint", "keypoints_source": "pick",
            "box_width": 0.35, "box_behind_tag": 0.0, "box_beneath_tag": 0.0,
            "box_left_tag": 0.0, "hand_pitch_degree": 0.0,
        })
        node.initialise()
        mock_pick.assert_called_once()


@pytest.mark.unit
def test_initialise_calls_lift_keypoints_when_source_lift():
    """initialise() 应按 keypoints_source='lift' 调 generate_lift_keypoints。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    with patch(
        "orchestration.nodes.node_tag_to_arm_goal.generate_lift_keypoints"
    ) as mock_lift:
        mock_lift.return_value = ([], [])
        node = NodeTagToArmGoal("a", "arm_goal", "ns", {
            "tag_id": 1, "control_type": "joint", "keypoints_source": "lift",
            "box_width": 0.35, "box_behind_tag": 0.0, "box_beneath_tag": 0.0,
            "box_left_tag": 0.0, "z_lift": 0.2,
        })
        node.initialise()
        mock_lift.assert_called_once()


def test_run_ik_returns_multi_frame_traj():
    """_run_ik() 遍历全部 keypoints，返回多帧轨迹（WORLD 系）。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    from orchestration.nodes.node_tag_to_arm_goal import NodeTagToArmGoal

    node = NodeTagToArmGoal("a", "arm_goal", "ns", {
        "tag_id": 1, "control_type": "joint", "keypoints_source": "pick",
        "box_width": 0.35, "box_behind_tag": 0.0, "box_beneath_tag": 0.0,
        "box_left_tag": 0.0, "hand_pitch_degree": 0.0,
    })
    node.initialise()
    assert len(node._left_keypoints) == 4
    assert len(node._right_keypoints) == 4

    with patch.object(node, "_solve_single_ik") as mock_ik:
        mock_ik.side_effect = [
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7],
            [0.11, 0.21, 0.31, 0.41, 0.51, 0.61, 0.71],
            [-0.11, -0.21, -0.31, -0.41, -0.51, -0.61, -0.71],
            [0.12, 0.22, 0.32, 0.42, 0.52, 0.62, 0.72],
            [-0.12, -0.22, -0.32, -0.42, -0.52, -0.62, -0.72],
            [0.13, 0.23, 0.33, 0.43, 0.53, 0.63, 0.73],
            [-0.13, -0.23, -0.33, -0.43, -0.53, -0.63, -0.73],
        ]

        from core.domain.perception import TagDetection
        from core.domain.pose import Pose6D
        tag = TagDetection(tag_id=1, pose_in_world=Pose6D(x=1.0, y=1.0, z=1.0))

        left_traj, right_traj = node._run_ik(tag)

        assert len(left_traj) == 4
        assert len(right_traj) == 4
        assert len(left_traj[0]) == 7
        assert left_traj[-1] == [0.13, 0.23, 0.33, 0.43, 0.53, 0.63, 0.73]


def test_run_ik_partial_failure_skips_frame():
    """_run_ik() 某帧 IK 失败时跳过该帧，其余帧仍返回。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    from orchestration.nodes.node_tag_to_arm_goal import NodeTagToArmGoal

    node = NodeTagToArmGoal("a", "arm_goal", "ns", {
        "tag_id": 1, "control_type": "joint", "keypoints_source": "pick",
        "box_width": 0.35, "box_behind_tag": 0.0, "box_beneath_tag": 0.0,
        "box_left_tag": 0.0, "hand_pitch_degree": 0.0,
    })
    node.initialise()

    with patch.object(node, "_solve_single_ik") as mock_ik:
        mock_ik.side_effect = [
            [0.1] * 7, [-0.1] * 7,     # 帧1 OK
            None, [-0.2] * 7,          # 帧2 左臂失败
            [0.3] * 7, [-0.3] * 7,     # 帧3 OK
            [0.4] * 7, [-0.4] * 7,     # 帧4 OK
        ]

        from core.domain.perception import TagDetection
        from core.domain.pose import Pose6D
        tag = TagDetection(tag_id=1, pose_in_world=Pose6D(x=1.0, y=1.0, z=1.0))

        left_traj, right_traj = node._run_ik(tag)
        assert len(left_traj) == 3
        assert len(right_traj) == 3
