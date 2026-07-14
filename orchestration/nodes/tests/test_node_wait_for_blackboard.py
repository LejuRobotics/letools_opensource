# -*- coding: utf-8 -*-
"""NodeWaitForBlackboard 单元测试:三链行为(spec § 6)。

- 干跑 / use_virtual_tag=true:立即注入 fake tag + SUCCESS
- 真机 use_virtual_tag=false:等真 tag → SUCCESS;超时降级注入 fake tag + SUCCESS
"""
import os
import time
import pytest
from unittest.mock import patch, MagicMock
from py_trees.common import Status

from orchestration.nodes.node_wait_for_blackboard import NodeWaitForBlackboard

pytestmark = pytest.mark.unit


def test_dry_run_injects_fake_tag_immediately():
    """干跑 → 立即注入 fake tag(pose = virtual - stand_in) + SUCCESS。"""
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        node = NodeWaitForBlackboard("w", "wait", "ns", {
            "tag_id": 1,
            "stand_in_tag_pos": [0.0, 0.0, 0.55],
            "stand_in_tag_euler": [0.0, 0.0, 0.0],
            "use_virtual_tag": False,
            "real_tag_timeout_sec": 30.0,
            "virtual_tag_pose_in_odom": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        })
        node.initialise()
        result = node.update()
        assert result == Status.SUCCESS
        fake = getattr(node.global_blackboard, "latest_tag_1", None)
        assert fake is not None
        # 公式: pose_in_world = virtual - stand_in
        # virtual=(1,0,0,0,0,0), stand_in=(0,0,0.55) → tag.pose.z=0-0.55=-0.55
        assert fake.pose_in_world.z == pytest.approx(-0.55, abs=1e-6)
    finally:
        del os.environ["STUDIO_DRY_RUN"]


def test_use_virtual_tag_true_injects_fake_tag_in_real_run():
    """非干跑 + use_virtual_tag=true → 也立即注入 fake tag(spec § 6.1 链 2)。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1,
        "stand_in_tag_pos": [0.0, 0.0, 0.55],
        "stand_in_tag_euler": [0.0, 0.0, 0.0],
        "use_virtual_tag": True,
        "real_tag_timeout_sec": 30.0,
        "virtual_tag_pose_in_odom": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    })
    node.initialise()
    result = node.update()
    assert result == Status.SUCCESS
    assert getattr(node.global_blackboard, "latest_tag_1", None) is not None


def test_real_mode_waits_for_real_tag_then_success():
    """非干跑 + use_virtual_tag=false + 真 tag 已写黑板 → SUCCESS。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1,
        "stand_in_tag_pos": [0.0, 0.0, 0.55],
        "stand_in_tag_euler": [0.0, 0.0, 0.0],
        "use_virtual_tag": False,
        "real_tag_timeout_sec": 30.0,
        "virtual_tag_pose_in_odom": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    })
    node.initialise()
    # 写一个真 tag
    fake_real = MagicMock(tag_id=1, pose_in_world=MagicMock(z=0.0))
    setattr(node.global_blackboard, "latest_tag_1", fake_real)
    result = node.update()
    assert result == Status.SUCCESS


def test_real_mode_timeout_falls_back_to_virtual():
    """非干跑 + use_virtual_tag=false + 无真 tag + 超时 → 降级注入 fake tag + SUCCESS。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1,
        "stand_in_tag_pos": [0.0, 0.0, 0.55],
        "stand_in_tag_euler": [0.0, 0.0, 0.0],
        "use_virtual_tag": False,
        "real_tag_timeout_sec": 0.05,  # 50ms 超时
        "virtual_tag_pose_in_odom": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    })
    node.initialise()
    time.sleep(0.1)  # 等超时
    result = node.update()
    assert result == Status.SUCCESS
    fake = getattr(node.global_blackboard, "latest_tag_1", None)
    assert fake is not None  # 降级注入
