# -*- coding: utf-8 -*-
"""NodeWaitForBlackboard 单元测试:三链行为(spec § 6)。

- 干跑 / use_virtual_tag=true:立即注入 fake tag + SUCCESS
- 真机 use_virtual_tag=false:等真 tag → SUCCESS;超时默认 FAILURE,显式 fallback_to_virtual=true 才降级注入 fake tag
"""
import os
import time
import pytest
from unittest.mock import patch, MagicMock
from py_trees.blackboard import Blackboard
from py_trees.common import Access, Status

from core.domain.perception import TagDetection
from core.domain.pose import Pose6D
from orchestration.nodes.node_percep import NodePercep

from orchestration.nodes.node_wait_for_blackboard import NodeWaitForBlackboard

pytestmark = pytest.mark.unit


@pytest.fixture(autouse=True)
def isolated_blackboard(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)


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


def test_real_mode_timeout_without_fallback_param_fails_by_default():
    """未配置 fallback_to_virtual 时,超时不注入 fake tag,返回 FAILURE。"""
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1,
        "stand_in_tag_pos": [0.0, 0.0, 0.55],
        "stand_in_tag_euler": [0.0, 0.0, 0.0],
        "use_virtual_tag": False,
        "real_tag_timeout_sec": 0.05,
        "virtual_tag_pose_in_odom": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    })
    node.initialise()
    time.sleep(0.1)
    assert node.update() == Status.FAILURE
    assert not node.global_blackboard.exists("latest_tag_1")


def test_real_mode_timeout_falls_back_to_virtual():
    """非干跑 + use_virtual_tag=false + 无真 tag + 超时 + fallback_to_virtual=true → 降级注入 fake tag + SUCCESS。"""
    if "STUDIO_DRY_RUN" in os.environ:
        del os.environ["STUDIO_DRY_RUN"]
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1,
        "stand_in_tag_pos": [0.0, 0.0, 0.55],
        "stand_in_tag_euler": [0.0, 0.0, 0.0],
        "use_virtual_tag": False,
        "fallback_to_virtual": True,
        "real_tag_timeout_sec": 0.05,  # 50ms 超时
        "virtual_tag_pose_in_odom": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    })
    node.initialise()
    time.sleep(0.1)  # 等超时
    result = node.update()
    assert result == Status.SUCCESS
    fake = getattr(node.global_blackboard, "latest_tag_1", None)
    assert fake is not None  # 降级注入


@pytest.mark.parametrize("fresh", [False, True])
def test_timeout_without_fallback_does_not_inject_target(fresh):
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1, "fallback_to_virtual": False,
        "require_fresh_tag": fresh, "real_tag_timeout_sec": 5.0,
    })
    with patch("orchestration.nodes.node_wait_for_blackboard.time.monotonic", return_value=100.0):
        node.initialise()
    board = node.global_blackboard
    board.set("latest_tag_1_xyz", [9.0, 9.0, 9.0])
    with patch("orchestration.nodes.node_wait_for_blackboard.time.monotonic", return_value=105.0), \
         patch.object(node, "_inject_virtual_tag") as inject:
        assert node.update() == Status.FAILURE
        inject.assert_not_called()
    assert "virtual fallback disabled" in node.feedback_message
    assert not board.exists("latest_tag_1_xyz")
    assert not board.exists("latest_tag_1")


@pytest.mark.parametrize("dry_run", [False, True])
def test_explicit_virtual_and_dry_run_override_disabled_fallback(monkeypatch, dry_run):
    if dry_run:
        monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1, "use_virtual_tag": not dry_run,
        "fallback_to_virtual": False, "require_fresh_tag": True,
        "virtual_tag_pose_in_odom": [0.7, 0.0, 1.1, 0.0, 0.0, 0.0],
    })
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert node.global_blackboard.get("latest_tag_1_xyz") == [0.7, 0.0, 1.1]


@pytest.mark.parametrize("timestamp", [99.0, 0.0, float("nan"), 102.0])
def test_old_invalid_or_future_frame_is_rejected_even_when_version_increases(timestamp):
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1, "fallback_to_virtual": False, "require_fresh_tag": True,
    })
    board = node.global_blackboard
    board.register_key(key="latest_tag_1_version", access=Access.WRITE)
    board.set("latest_tag_1_version", 4)
    board.set("latest_tag_1_xyz", [9.0, 9.0, 9.0])
    with patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=100.0):
        node.initialise()
    assert not board.exists("latest_tag_1_xyz")
    tag = TagDetection(1, Pose6D(x=0.7, y=0.0, z=1.1), timestamp=timestamp)
    board.set("latest_tag_1", tag)
    board.set("latest_tag_1_version", 5)
    with patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=101.0):
        assert node.update() == Status.RUNNING
    assert board.get("latest_tag_1") is tag
    assert not board.exists("latest_tag_1_xyz")


def test_fresh_tag_requires_version_update_and_is_accepted_without_clearing_detection():
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1, "fallback_to_virtual": False, "require_fresh_tag": True,
    })
    board = node.global_blackboard
    board.register_key(key="latest_tag_1_version", access=Access.WRITE)
    board.set("latest_tag_1_version", 4)
    with patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=100.0):
        node.initialise()
    tag = TagDetection(1, Pose6D(x=0.7, y=0.2, z=1.1), timestamp=100.5)
    board.set("latest_tag_1", tag)
    with patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=101.0):
        assert node.update() == Status.RUNNING
        board.set("latest_tag_1_version", 5)
        assert node.update() == Status.SUCCESS
    assert board.get("latest_tag_1") is tag
    assert board.get("latest_tag_1_xyz") == [0.7, 0.2, 1.1]


def test_reentering_wait_rejects_previously_accepted_tag():
    node = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1, "fallback_to_virtual": False, "require_fresh_tag": True,
    })
    board = node.global_blackboard
    board.register_key(key="latest_tag_1_version", access=Access.WRITE)
    with patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=100.0):
        node.initialise()
    board.set("latest_tag_1_version", 1)
    board.set("latest_tag_1", TagDetection(1, Pose6D(x=0.7, y=0.0, z=1.1), timestamp=100.5))
    with patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=101.0):
        assert node.update() == Status.SUCCESS
    with patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=102.0):
        node.initialise()
        board.set("latest_tag_1_version", 2)
        assert node.update() == Status.RUNNING
    assert not board.exists("latest_tag_1_xyz")


def test_parallel_wait_then_perception_accepts_new_frame_on_next_tick():
    import py_trees
    from py_trees.common import ParallelPolicy

    wait = NodeWaitForBlackboard("w", "wait", "ns", {
        "tag_id": 1, "fallback_to_virtual": False, "require_fresh_tag": True,
    })
    percep = NodePercep("p", "percep", "ns", {"tag_ids": [1]})
    root = py_trees.composites.Parallel("detect", policy=ParallelPolicy.SuccessOnOne(), children=[wait, percep])
    tree = py_trees.trees.BehaviourTree(root)
    tag = TagDetection(1, Pose6D(x=0.7, y=0.0, z=1.1), timestamp=100.5)
    hardware = MagicMock()
    hardware.perception.get_tag_detections.return_value = [tag]
    with patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=100.0), \
         patch("orchestration.nodes.node_percep.get_shared_hardware", return_value=hardware):
        tree.tick()
        assert root.status == Status.RUNNING
        with patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=101.0):
            tree.tick()
    assert root.status == Status.SUCCESS
    assert wait.global_blackboard.get("latest_tag_1_xyz") == [0.7, 0.0, 1.1]


def test_freshness_uses_ros_clock_and_timeout_uses_monotonic_clock():
    from orchestration.nodes.node_wait_for_blackboard import _tag_time
    fake_ros = MagicMock()
    fake_ros.core.is_initialized.return_value = True
    fake_ros.get_time.return_value = 10.0
    with patch.dict("sys.modules", {"rospy": fake_ros}):
        assert _tag_time() == 10.0
        node = NodeWaitForBlackboard("w", "wait", "ns", {
            "tag_id": 1, "fallback_to_virtual": False, "require_fresh_tag": True,
            "real_tag_timeout_sec": 5.0,
        })
        with patch("orchestration.nodes.node_wait_for_blackboard.time.monotonic", return_value=100.0):
            node.initialise()
        assert node._tag_start_time == 10.0
        with patch("orchestration.nodes.node_wait_for_blackboard.time.monotonic", return_value=105.0):
            assert node.update() == Status.FAILURE
