# -*- coding: utf-8 -*-
"""NodePercep 单元测试。"""
import os
import pytest
from unittest.mock import patch, MagicMock
from py_trees.common import Status

from orchestration.nodes.node_percep import NodePercep
from core.domain.result import Result

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
    mock_hw.perception.start.return_value = Result.ok()
    mock_hw.perception.stop.return_value = Result.ok()
    mock_hw.perception.get_tag_detections.return_value = [fake_tag]
    with patch("orchestration.nodes.node_percep.get_shared_hardware", return_value=mock_hw):
        node = NodePercep("p", "percep", "ns", {"tag_ids": [1]})
        node.initialise()
        result = node.update()
        assert result == Status.RUNNING
        assert getattr(node.global_blackboard, "latest_tag_1", None) is fake_tag
        assert getattr(node.global_blackboard, "latest_tag_1_version", 0) == 1
        mock_hw.perception.start.assert_called_once()
        node.terminate(Status.INVALID)
        mock_hw.perception.stop.assert_called_once()
        # 下游可能在检测阶段结束后使用已观测到的 Tag，保留黑板快照。
        assert getattr(node.global_blackboard, "latest_tag_1", None) is fake_tag


def test_start_failure_returns_failure_without_reading(monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hardware = MagicMock()
    hardware.perception.start.return_value = Result.fail("detector unavailable")
    with patch("orchestration.nodes.node_percep.get_shared_hardware", return_value=hardware):
        node = NodePercep("p", "percep", "ns", {"tag_ids": [1]})
        node.initialise()
        assert node.update() == Status.FAILURE
        assert "detector unavailable" in node.feedback_message
        hardware.perception.get_tag_detections.assert_not_called()
        node.terminate(Status.FAILURE)
        hardware.perception.stop.assert_called_once()


def test_missing_adapter_returns_failure(monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    with patch("orchestration.nodes.node_percep.get_shared_hardware", return_value=MagicMock(perception=None)):
        node = NodePercep("p", "percep", "ns", {"tag_ids": [1]})
        node.initialise()
        assert node.update() == Status.FAILURE
        node.terminate(Status.FAILURE)


def test_virtual_mode_never_starts_or_stops_hardware(monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    with patch("orchestration.nodes.node_percep.get_shared_hardware") as hardware:
        node = NodePercep("p", "percep", "ns", {"tag_ids": [1], "use_virtual_tag": True})
        node.initialise()
        assert node.update() == Status.RUNNING
        node.terminate(Status.INVALID)
        hardware.assert_not_called()
