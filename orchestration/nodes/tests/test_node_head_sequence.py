# -*- coding: utf-8 -*-
"""NodeHeadSequence 单元测试:扫头 + 循环重扫 + 超时。"""
import os
import time
import pytest
from unittest.mock import patch, MagicMock
from py_trees.common import Status

from orchestration.nodes.node_head_sequence import NodeHeadSequence

pytestmark = pytest.mark.unit


def test_dry_run_returns_success():
    """干跑模式下,NodeHeadSequence 应直接 SUCCESS,带 'dry-run' 诊断。"""
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        node = NodeHeadSequence("h", "head", "ns", {
            "head_search_yaws": [0, 30],
            "head_search_pitchs": [0],
            "scan_timeout": 5.0,
        })
        node.initialise()
        assert node.update() == Status.SUCCESS
        assert "dry-run" in node.feedback_message
    finally:
        del os.environ["STUDIO_DRY_RUN"]


def test_scan_loop_returns_running_when_no_timeout():
    """非干跑 + mock SDK + 未超时 → RUNNING(模拟循环重扫)。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    mock_hw = MagicMock()
    with patch("orchestration.nodes.node_head_sequence.get_shared_hardware", return_value=mock_hw):
        node = NodeHeadSequence("h", "head", "ns", {
            "head_search_yaws": [0, 30],
            "head_search_pitchs": [0],
            "scan_timeout": 60.0,
        })
        node.initialise()
        result = node.update()
        assert result == Status.RUNNING


def test_scan_timeout_returns_failure():
    """非干跑 + scan_timeout=0.05 + 短暂 sleep → FAILURE,带 'timeout' 诊断。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    mock_hw = MagicMock()
    with patch("orchestration.nodes.node_head_sequence.get_shared_hardware", return_value=mock_hw):
        node = NodeHeadSequence("h", "head", "ns", {
            "head_search_yaws": [0, 30],
            "head_search_pitchs": [0],
            "scan_timeout": 0.05,
        })
        node.initialise()
        time.sleep(0.1)
        result = node.update()
        assert result == Status.FAILURE
        assert "timeout" in node.feedback_message.lower()
