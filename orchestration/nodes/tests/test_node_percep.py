# -*- coding: utf-8 -*-
"""NodePercep 单元测试。"""
import os
import pytest
from unittest.mock import patch, MagicMock
from py_trees.common import Status

from orchestration.nodes.node_percep import NodePercep

pytestmark = pytest.mark.unit


def test_dry_run_returns_success_without_writing():
    """干跑模式 → 直接 SUCCESS,不调 hardware。"""
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        node = NodePercep("p", "percep", "ns", {"tag_ids": [1]})
        node.initialise()
        assert node.update() == Status.SUCCESS
    finally:
        del os.environ["STUDIO_DRY_RUN"]


def test_real_run_writes_blackboard_when_tag_detected():
    """非干跑 + mock SDK 返回 tag → 写黑板 latest_tag_<id> + version。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    fake_tag = MagicMock(tag_id=1, pose_in_world=MagicMock(), size=0.1, confidence=1.0)
    mock_hw = MagicMock()
    mock_hw.perception.get_tag_detections.return_value = [fake_tag]
    with patch("orchestration.nodes.node_percep.get_shared_hardware", return_value=mock_hw):
        node = NodePercep("p", "percep", "ns", {"tag_ids": [1]})
        node.initialise()
        result = node.update()
        assert result == Status.RUNNING
        assert getattr(node.global_blackboard, "latest_tag_1", None) is fake_tag
        assert getattr(node.global_blackboard, "latest_tag_1_version", 0) == 1
