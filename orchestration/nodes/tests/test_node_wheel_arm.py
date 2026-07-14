# -*- coding: utf-8 -*-
"""NodeWheelArm 单元测试。"""
import os
import pytest
from unittest.mock import patch, MagicMock
from py_trees.common import Status
import py_trees

from orchestration.nodes.node_wheel_arm import NodeWheelArm


@pytest.mark.unit
def test_dry_run_returns_success():
    """干跑 → SUCCESS。"""
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        node = NodeWheelArm("a", "arm", "ns", {"control_type": "joint"})
        node.initialise()
        assert node.update() == Status.SUCCESS
    finally:
        del os.environ["STUDIO_DRY_RUN"]


def test_joint_control_reads_traj_from_blackboard():
    """非干跑 + control_type=joint + 多帧 traj 已写黑板 → 调 send_arm_joint_traj_sdk + SUCCESS。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    mock_hw = MagicMock()
    # mock get_arm_joint_positions → 返回当前关节角（可选）
    mock_hw.get_arm_joint_positions.return_value = MagicMock(success=True, data=[0.0] * 14)
    with patch("orchestration.nodes.node_wheel_arm.get_shared_hardware", return_value=mock_hw):
        # 写多帧轨迹（每帧 7 关节 × 左右臂）
        bb = py_trees.blackboard.Blackboard()
        bb.set("left_arm_joint_traj", [[0.1] * 7, [0.2] * 7, [0.3] * 7, [0.4] * 7])
        bb.set("right_arm_joint_traj", [[-0.1] * 7, [-0.2] * 7, [-0.3] * 7, [-0.4] * 7])
        node = NodeWheelArm("a", "arm", "ns", {"control_type": "joint"})
        result = node.update()
        assert result == Status.SUCCESS
        mock_hw.send_arm_joint_traj_sdk.assert_called_once()
        # 验证轨迹帧数（含起点帧 = 1 current + 4 keypoints）
        sent_traj = mock_hw.send_arm_joint_traj_sdk.call_args[0][0]
        assert len(sent_traj) == 5  # current + 4 frames


@pytest.mark.unit
def test_joint_control_no_traj_returns_running():
    """非干跑 + traj 未写 → RUNNING(等上游)。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    # 清黑板避免受前面测试影响
    bb = py_trees.blackboard.Blackboard()
    for k in list(bb.storage.keys()):
        if any(x in k for x in ("left_arm", "right_arm")):
            del bb.storage[k]
    mock_hw = MagicMock()
    with patch("orchestration.nodes.node_wheel_arm.get_shared_hardware", return_value=mock_hw):
        node = NodeWheelArm("a", "arm", "ns", {"control_type": "joint"})
        result = node.update()
        assert result == Status.RUNNING
        mock_hw.control.arm_joint_traj.assert_not_called()
