# -*- coding: utf-8 -*-
"""EndEffectorClose 单元测试。"""
import os
import pytest
from unittest.mock import patch, MagicMock
from py_trees.common import Status

from orchestration.nodes.end_effector_close import EndEffectorClose

pytestmark = pytest.mark.unit


def test_dry_run_returns_success():
    """干跑 → SUCCESS,带 'dry-run' 诊断。"""
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        node = EndEffectorClose("g", "gripper", "ns", {})
        node.initialise()
        assert node.update() == Status.SUCCESS
        assert "dry-run" in node.feedback_message
    finally:
        del os.environ["STUDIO_DRY_RUN"]


def test_default_params_full_close_both_arms():
    """默认参数:side=BOTH, position=100, effort=1.0 → 调 control_end_effector。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    mock_hw = MagicMock()
    with patch("orchestration.nodes.end_effector_close.get_shared_hardware", return_value=mock_hw):
        node = EndEffectorClose("g", "gripper", "ns", {})
        node.initialise()
        result = node.update()
        assert result == Status.SUCCESS
        # control_end_effector 应被调用 2 次(LEFT + RIGHT)
        assert mock_hw.control_end_effector.call_count == 2
