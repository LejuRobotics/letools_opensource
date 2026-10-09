# -*- coding: utf-8 -*-
"""NodePalletPose 单元测试。

本节点不访问硬件，所以这个测试也不需要：tag 位姿手工构造、标定结果写在临时
YAML 里、托盘位姿是纯计算。没有 mock 硬件、没有 ROS、没有相机。

黑板权限是刻意分开的：节点只注册 `latest_tag_<id>` 的 **READ**，写 tag 的是
另一个 client（`make_writer`），模拟真实运行里持有 WRITE 的上游 `NodePercep`。
第一版测试用节点自己的 client 去写，被 py_trees 挡了下来 —— 那个报错是对的。

运行：
    pytest orchestration/nodes/tests/test_node_pallet_pose.py -m unit -v

CI 的 verify:opensource 跑的正是 `pytest orchestration/nodes/tests/ -m unit`，
所以这个文件会被自动带上。
"""
import logging
from unittest.mock import MagicMock

import numpy as np
import py_trees
import pytest
import yaml
from py_trees.common import Access, Status

from core.common.transform import matrix_to_pose6d
from orchestration.nodes.node_pallet_pose import NodePalletPose

pytestmark = pytest.mark.unit


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def write_calibration(path, by_tag):
    """写一个最小可用的 pallet_tag.yaml，返回路径字符串。"""
    path.write_text(
        yaml.safe_dump(
            {"T_pallet_tag_by_marker": {str(k): v for k, v in by_tag.items()}},
            allow_unicode=True),
        encoding="utf-8")
    return str(path)


def fake_detection(pose):
    """冒充 NodePercep 写在黑板上的 TagDetection（本节点只用到 pose_in_world）。"""
    return MagicMock(pose_in_world=pose, tag_id=0)


def make_node(config_path, tag_ids, **extra):
    params = {"tag_ids": tag_ids, "config_path": str(config_path)}
    params.update(extra)
    node = NodePalletPose("pallet", "pallet", "ns", params)
    node.initialise()
    return node


def make_writer(tag_ids):
    """模拟 NodePercep：独立 client，持有 latest_tag_<id> 的写权限。"""
    writer = py_trees.blackboard.Client(name="test_node_percep")
    for tid in tag_ids:
        writer.register_key(key=f"latest_tag_{tid}", access=Access.WRITE)
        writer.register_key(key=f"latest_tag_{tid}_version", access=Access.WRITE)
    return writer


def write_tag(writer, tag_id, pose, version=1):
    setattr(writer, f"latest_tag_{tag_id}", fake_detection(pose))
    setattr(writer, f"latest_tag_{tag_id}_version", version)


def clear_tag(writer, tag_id):
    """显式清空。py_trees 的黑板是进程级的，不清理会让用例之间互相污染。"""
    setattr(writer, f"latest_tag_{tag_id}", None)
    setattr(writer, f"latest_tag_{tag_id}_version", 0)


@pytest.fixture(autouse=True)
def _no_dry_run(monkeypatch):
    """默认按真跑测；要测干跑的用例自己 setenv 覆盖。"""
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)


# --------------------------------------------------------------------------- #
# tests
# --------------------------------------------------------------------------- #
def test_dry_run_returns_success_and_skips_everything(tmp_path, monkeypatch):
    """干跑 → SUCCESS，既不读标定也不碰黑板。"""
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    node = make_node(tmp_path / "does_not_exist.yaml", [0, 1])
    assert node.update() == Status.SUCCESS


def test_missing_calibration_fails_with_actionable_message(tmp_path):
    """标定文件不存在 → FAILURE，且 feedback 要告诉人下一步做什么。"""
    node = make_node(tmp_path / "nope.yaml", [0, 1])
    assert node.update() == Status.FAILURE
    assert "pallet_calibrate" in node.feedback_message


def test_waits_while_no_tag_on_blackboard(tmp_path):
    """有标定但黑板上还没有 tag → 保持 RUNNING，不写托盘位姿。"""
    writer = make_writer([0, 1])
    clear_tag(writer, 0)
    clear_tag(writer, 1)
    calib = write_calibration(tmp_path / "c.yaml", {0: np.eye(4).tolist()})
    node = make_node(calib, [0, 1])
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet", "unset") is None


def test_single_tag_produces_pose_and_bumps_version(tmp_path):
    """一个 tag + 单位标定 → 托盘位姿应当等于该 tag 的位姿。"""
    calib = write_calibration(tmp_path / "c.yaml", {0: np.eye(4).tolist()})
    node = make_node(calib, [0])

    writer = make_writer([0])
    pose = matrix_to_pose6d(np.eye(4))
    pose.x, pose.y, pose.z = 1.0, 2.0, 3.0
    write_tag(writer, 0, pose, version=1)

    assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_pallet", None)
    assert out is not None, "托盘位姿没写进黑板"
    assert np.allclose([out.x, out.y, out.z], [1.0, 2.0, 3.0], atol=1e-9)
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1


def test_two_tags_are_fused(tmp_path):
    """两个 tag 各自推一个托盘位姿，融合结果应落在两者之间。

    构造：让 tag0 推出托盘在 x=0、tag1 推出在 x=-2，融合结果应当在 (-2, 0) 内。
    """
    calib = write_calibration(tmp_path / "c.yaml",
                              {0: np.eye(4).tolist(), 1: np.eye(4).tolist()})
    node = make_node(calib, [0, 1])

    T1 = np.eye(4)
    T1[:3, 3] = [-2.0, 0.0, 0.0]
    writer = make_writer([0, 1])
    write_tag(writer, 0, matrix_to_pose6d(np.eye(4)), version=1)
    write_tag(writer, 1, matrix_to_pose6d(T1), version=1)

    assert node.update() == Status.RUNNING
    out = getattr(node.global_blackboard, "latest_pallet", None)
    assert out is not None
    assert -2.0 < out.x < 0.0, f"融合结果 {out.x} 不在两个 tag 的答案之间"


def test_uncalibrated_tag_is_skipped(tmp_path):
    """黑板上有个没标定过的 tag id → 忽略它，不报错也不出结果。"""
    calib = write_calibration(tmp_path / "c.yaml", {0: np.eye(4).tolist()})
    node = make_node(calib, [0, 7])

    writer = make_writer([0, 7])
    clear_tag(writer, 0)
    write_tag(writer, 7, matrix_to_pose6d(np.eye(4)), version=1)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet", None) is None

    write_tag(writer, 0, matrix_to_pose6d(np.eye(4)), version=1)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet", None) is not None


def test_version_gate_skips_recompute(tmp_path):
    """tag 版本号没变 → 不重算（否则 50 Hz 每 tick 都白算一遍）。"""
    calib = write_calibration(tmp_path / "c.yaml", {0: np.eye(4).tolist()})
    node = make_node(calib, [0])

    writer = make_writer([0])
    write_tag(writer, 0, matrix_to_pose6d(np.eye(4)), version=1)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1

    # 同一版本再 tick 若干次，版本号不应继续涨
    for _ in range(3):
        assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1


# --------------------------------------------------------------------------- #
# 「键从未被写过」——2026-09-21 修掉的那条会把整棵树弄死的路
# --------------------------------------------------------------------------- #
# py_trees 的黑板是**进程级**的，本文件前面的用例已经把 `latest_tag_0` / `_1` /
# `_7` 写出来了。要复现"从未被写过"，得挑一个全文件没人碰过的 id。
_NEVER_WRITTEN = 97
_NEVER_WRITTEN_ALONE = 96


def test_a_never_written_tag_key_raises_KeyError_not_AttributeError(tmp_path):
    """★ **墓碑**：先把陷阱本身钉住 —— 这不是"防御性编程"，是真会抛的。

    `getattr(client, key, None)` 看上去在兜底，但 py_trees 的
    `Client.__getattr__` 在键**从未被写过**时抛的是 **`KeyError`**，那个 `None`
    默认值**接不住它**。而 py_trees 的 `Behaviour.tick()` **不接** `update()`
    抛出的异常 —— 于是 `KeyError` 一路抛出去，**整棵树连每帧日志一起没**。

    这条用例断言的是"旧写法必然抛"：谁把 `_read_blackboard` 改回裸 `getattr`，
    下一条用例会红，而这条会一直绿着提醒他为什么。
    """
    calib = write_calibration(tmp_path / "c.yaml",
                              {_NEVER_WRITTEN: np.eye(4).tolist()})
    node = make_node(calib, [_NEVER_WRITTEN])
    with pytest.raises(KeyError):
        getattr(node.global_blackboard, f"latest_tag_{_NEVER_WRITTEN}")


def test_a_never_written_tag_does_not_kill_the_tree(tmp_path, caplog):
    """★ 核心回归：上游没覆盖到的那个 id，**不许把整棵树弄死**。

    真实触发条件有两个，都会发生：① 本节点的 `tag_ids` 里有上游 `NodePercep`
    没有的 id（两个列表配岔了）；② 树上压根没有写那些键的节点。修之前这两条
    都是**当场整棵树没**，而且报错在 py_trees 内部、离现场很远。
    """
    calib = write_calibration(
        tmp_path / "c.yaml",
        {0: np.eye(4).tolist(), _NEVER_WRITTEN: np.eye(4).tolist()})
    writer = make_writer([0])                      # 上游只产出 tag 0
    write_tag(writer, 0, matrix_to_pose6d(np.eye(4)))
    node = make_node(calib, [0, _NEVER_WRITTEN])   # 本节点却还要一个 97

    with caplog.at_level(logging.WARNING):
        assert node.update() == Status.RUNNING, "从未被写过的 tag 键把节点弄死了"

    # 有的那个照常用：缺一个 id 不该让整个节点停摆
    assert getattr(node.global_blackboard, "latest_pallet", None) is not None, (
        "缺一个 tag 不该妨碍用另一个算")
    assert f"latest_tag_{_NEVER_WRITTEN}" in caplog.text, (
        f"配岔了却一声不响：{caplog.text!r}")
    print("    从未被写过的 tag 键 → 不炸，照用有的那个，并 WARNING 点名")


def test_a_never_written_tag_warns_once_and_says_so_in_feedback(tmp_path, caplog):
    """一个能用的 tag 都没有时：feedback 要区分"还没检测到"与"永远不会来"。

    都只说一句"等 tag"的话，界面上看不出是相机还没看到标签、还是 `tag_ids`
    配岔了 —— 而后者**永远不会好**。警告按 id 去重，不是每 tick 一条
    （50 Hz 下现场只剩它）。
    """
    calib = write_calibration(tmp_path / "c.yaml",
                              {_NEVER_WRITTEN_ALONE: np.eye(4).tolist()})
    node = make_node(calib, [_NEVER_WRITTEN_ALONE])

    with caplog.at_level(logging.WARNING):
        for _ in range(5):
            assert node.update() == Status.RUNNING

    assert "从未被写过" in node.feedback_message, node.feedback_message
    assert str(_NEVER_WRITTEN_ALONE) in node.feedback_message, (
        f"feedback 该点出是哪个 id，否则界面上还是看不出配岔了："
        f"{node.feedback_message}")
    assert caplog.text.count(f"latest_tag_{_NEVER_WRITTEN_ALONE}") == 1, (
        f"警告该只喊一次，实际喊了 "
        f"{caplog.text.count(f'latest_tag_{_NEVER_WRITTEN_ALONE}')} 次")
    print("    永远等不到时：feedback 点出是哪个 id，警告只喊一次")


def test_a_non_numeric_version_does_not_raise(tmp_path):
    """版本号不是数字时按 0 处理 —— 旧代码这里 `int()` 直接抛穿 `update()`。

    与 `node_inject_servo_input._read_version` 同一口径。上游把版本号写成字符串
    不是天方夜谭（场景 JSON 与黑板的 READ_BOARD 分支都会递字符串进来），
    而这里的 `int()` 在 `update()` 里 —— 抛出去同样是整棵树没。
    """
    calib = write_calibration(tmp_path / "c.yaml", {0: np.eye(4).tolist()})
    writer = make_writer([0])
    write_tag(writer, 0, matrix_to_pose6d(np.eye(4)))
    setattr(writer, "latest_tag_0_version", "七")
    node = make_node(calib, [0])
    assert node.update() == Status.RUNNING
    print("    版本号是字符串 → 按 0 处理，不抛")
