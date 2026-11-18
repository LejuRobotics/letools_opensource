# -*- coding: utf-8 -*-
"""Offline coverage of the case01 -> case04 test-department suite."""

import copy
import json
from pathlib import Path
from unittest.mock import MagicMock, call

import pytest
from py_trees.blackboard import Blackboard, Client
from py_trees.common import Status

from apps.test_upper_init.run_behavior_tree_json import _load_board_into_blackboard
from core.domain.result import Result
from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
import orchestration.nodes.arm_joint_timed_move as arm
import orchestration.nodes.arm_joint_traj_sdk_move as trajectory
import orchestration.nodes.base_move_relative_jibot_move as chassis
import orchestration.nodes.head_control_sdk_move as head
import orchestration.nodes.multi_timed_command_move as multi
import orchestration.nodes.node_enable_vel_control as guard
import orchestration.nodes.wait_seconds as wait

pytestmark = pytest.mark.unit
ROOT = Path(__file__).resolve().parents[3]
SCENARIOS = ROOT / "orchestration/scenarios"
SUITE = SCENARIOS / "instructional_case_05"
CASE_NAMES = tuple("instructional_case_%02d" % number for number in range(1, 5))
MOTION_MODULES = (arm, trajectory, chassis, head, multi, guard)


def _load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _walk(node):
    yield node
    for child in node.get("childs", []):
        yield from _walk(child)


def _tree():
    board = Client(name="case05_test", namespace="/")
    _load_board_into_blackboard(board, str(SUITE / "board.json"))
    factory = BehaviorTreeFactory(
        board, enable_parallel_loading=False,
        subtree_json_path=str(SUITE / "py_tree_child.json"),
    )
    return factory.load_tree_from_json(str(SUITE / "py_tree.json")), board


def _finish(tree):
    for _ in range(100):
        tree.tick()
        if tree.root.status in (Status.SUCCESS, Status.FAILURE):
            return tree.root.status
    pytest.fail("case05 did not terminate within 100 offline ticks")


@pytest.fixture
def hardware(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.setattr("socket.socket.connect", lambda *args, **kwargs: pytest.fail("network forbidden"))
    monkeypatch.setattr("orchestration.shared_hardware.get_shared_hardware",
                        lambda: pytest.fail("real hardware forbidden"))
    hw = MagicMock()
    for name in ("control_head_sdk", "send_arm_joint_timed", "send_arm_joint_traj_sdk",
                 "send_timed_multi_commands", "set_arm_control_mode", "set_focus_ee",
                 "set_focus_z", "enable_vel_control_jibot", "base_move_relative_jibot"):
        getattr(hw, name).return_value = Result.ok(data={"actual_time": 3.0})
    hw.get_vel_control_state_jibot.return_value = Result.ok(data={"state": True})
    for module in MOTION_MODULES:
        monkeypatch.setattr(module, "_DRY_RUN", False)
        monkeypatch.setattr(module, "get_shared_hardware", lambda: hw)
    monkeypatch.setattr(wait, "_DRY_RUN", True)  # Timing is exercised separately with a fake clock.
    monkeypatch.setattr("time.sleep", lambda _: None)
    return hw


def test_scenario_contains_only_three_json_files_and_readme():
    assert {path.name for path in SUITE.iterdir()} == {
        "py_tree.json", "py_tree_child.json", "board.json", "README.md",
    }


def test_snapshot_only_prefixes_labels_and_blackboard_keys():
    board = _load(SUITE / "board.json")
    subtrees = _load(SUITE / "py_tree_child.json")
    root = _load(SUITE / "py_tree.json")["tree"]
    assert root["name"] == "Sequence"
    assert root["params"]["memory"]["value"] is True
    assert len(root["childs"]) == 7
    assert [child["name"] for child in root["childs"][::2]] == [name + ".json" for name in CASE_NAMES]
    assert set(subtrees) == {name + ".json" for name in CASE_NAMES}
    for number, child in enumerate(root["childs"][1::2], start=1):
        assert child["name"] == "WaitSeconds"
        assert child["label"] == "wait_between_case_%02d_and_%02d" % (number, number + 1)
        assert child["params"]["duration_sec"] == {
            "source": "CUSTOM", "value": 2.0, "data_type": "float",
        }
        assert child["childs"] == []
    total_keys = 0
    for number, case_name in enumerate(CASE_NAMES, start=1):
        prefix = "case_%02d__" % number
        original_board = _load(SCENARIOS / case_name / "board.json")
        total_keys += len(original_board)
        assert {key[len(prefix):]: value for key, value in board.items() if key.startswith(prefix)} == original_board
        unwrapped = copy.deepcopy(subtrees[case_name + ".json"]["tree"])
        for node in _walk(unwrapped):
            assert node["label"].startswith(prefix)
            node["label"] = node["label"][len(prefix):]
            for param in node.get("params", {}).values():
                if isinstance(param, dict) and param.get("source") == "READ_BOARD":
                    assert param["board_key"].startswith(prefix)
                    param["board_key"] = param["board_key"][len(prefix):]
        assert unwrapped == _load(SCENARIOS / case_name / "py_tree.json")["tree"]
    assert len(board) == total_keys


def test_executes_each_case_once_in_order_and_restores_chassis_guard(hardware):
    tree, _ = _tree()
    assert len(list(tree.root.iterate())) == 66
    for index, case_root in enumerate(tree.root.children[::2], start=1):
        assert case_root.subtree_file == "instructional_case_%02d.json" % index
        assert case_root.name.startswith("case_%02d__" % index)
    assert _finish(tree) == Status.SUCCESS
    motion_calls = [c[0] for c in hardware.mock_calls if c[0] in {
        "control_head_sdk", "send_arm_joint_timed", "send_arm_joint_traj_sdk",
        "send_timed_multi_commands", "base_move_relative_jibot",
    }]
    expected = ["control_head_sdk"] * 6
    expected += ["control_head_sdk", "send_arm_joint_timed"]
    expected += ["control_head_sdk", "send_arm_joint_traj_sdk"] * 4
    expected += ["control_head_sdk", "send_timed_multi_commands", "send_arm_joint_timed"] * 4
    expected += ["control_head_sdk", "send_timed_multi_commands"]
    expected += ["control_head_sdk", "base_move_relative_jibot", "send_timed_multi_commands"] * 3
    assert motion_calls == expected
    assert [c.kwargs["theta"] for c in hardware.base_move_relative_jibot.call_args_list] == [
        0.349066, -0.698132, 0.349066,
    ]
    assert hardware.enable_vel_control_jibot.call_args_list == [call(False), call(True)]
    assert [
        [command["planner_index"] for command in c.kwargs["commands"]]
        for c in hardware.send_timed_multi_commands.call_args_list
    ] == [[2]] * 4 + [[3, 8, 9]] * 4
    hardware.enable_quick_mode.assert_not_called()
    hardware.set_arm_quick_mode.assert_not_called()
    hardware.send_arm_joint_trajectory.assert_not_called()
    hardware.send_leg_joint_command.assert_not_called()


def test_each_case_boundary_waits_two_seconds_before_next_case(hardware, monkeypatch):
    clock = [100.0]
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    monkeypatch.setattr(wait, "_DRY_RUN", False)
    monkeypatch.setattr("time.time", lambda: clock[0])
    tree, _ = _tree()
    # Keep the real public WaitSeconds node/skill at all three case boundaries;
    # skip only case01's four internal waits to isolate the inter-case timing.
    for node in tree.root.children[0].iterate():
        if isinstance(node, wait.WaitSeconds):
            monkeypatch.setattr(node, "initialise", lambda: None)
            monkeypatch.setattr(node, "update", lambda: Status.SUCCESS)

    for child_index in (1, 3, 5):
        gap = tree.root.children[child_index]
        # Motion nodes may need another tick to report completion after a send.
        for _ in range(100):
            tree.tick()
            if gap.status == Status.RUNNING:
                break
        else:
            pytest.fail("case05 did not reach inter-case wait %s" % child_index)
        assert isinstance(gap, wait.WaitSeconds)
        assert gap._skill.params.duration_sec == 2.0
        assert gap.status == tree.root.status == Status.RUNNING
        assert all(child.status == Status.SUCCESS for child in tree.root.children[:child_index])
        assert all(child.status == Status.INVALID for child in tree.root.children[child_index + 1:])
        calls_before = list(hardware.mock_calls)
        started = clock[0]
        clock[0] = started + 1.999
        tree.tick()
        assert gap.status == Status.RUNNING
        assert tree.root.children[child_index + 1].status == Status.INVALID
        assert hardware.mock_calls == calls_before  # No repeated case or new motion during the wait.
        clock[0] = started + 2.0
        tree.tick()
        assert gap.status == Status.SUCCESS
        assert tree.root.children[child_index + 1].status != Status.INVALID
    assert _finish(tree) == Status.SUCCESS
    assert clock[0] == 106.0  # Three intervals, no leading/trailing wait.


@pytest.mark.parametrize("gap_index", range(3))
def test_wait_failure_prevents_next_case(hardware, monkeypatch, gap_index):
    tree, _ = _tree()
    child_index = gap_index * 2 + 1
    gap = tree.root.children[child_index]
    monkeypatch.setattr(gap, "initialise", lambda: None)
    monkeypatch.setattr(gap, "update", lambda: Status.FAILURE)
    assert _finish(tree) == Status.FAILURE
    assert gap.status == Status.FAILURE
    assert all(child.status == Status.SUCCESS for child in tree.root.children[:child_index])
    assert all(child.status == Status.INVALID for child in tree.root.children[child_index + 1:])


@pytest.mark.parametrize("case_index", range(4), ids=CASE_NAMES)
def test_case_failure_prevents_later_cases(hardware, monkeypatch, case_index):
    tree, _ = _tree()
    child_index = case_index * 2
    case_root = tree.root.children[child_index]
    # Inject a returned failure at the first action, not a fake controller fault.
    first_action = next(node for node in case_root.iterate() if not node.children)
    monkeypatch.setattr(first_action, "initialise", lambda: None)
    monkeypatch.setattr(first_action, "update", lambda: Status.FAILURE)
    assert _finish(tree) == Status.FAILURE
    assert case_root.status == Status.FAILURE
    assert all(child.status == Status.SUCCESS for child in tree.root.children[:child_index])
    assert all(child.status == Status.INVALID for child in tree.root.children[child_index + 1:])


def test_same_named_parameters_are_isolated(hardware):
    tree, board = _tree()
    board.set("case_03__motion_duration_sec", 7.0)
    board.set("case_03__torso_left_commands", [{"planner_index": 2, "cmd_vec": [0.14, 1.05, 15.0, 0.0]}])
    # Static duration is resolved when the factory builds nodes; build again.
    factory = BehaviorTreeFactory(board, enable_parallel_loading=False,
                                  subtree_json_path=str(SUITE / "py_tree_child.json"))
    tree = factory.load_tree_from_json(str(SUITE / "py_tree.json"))
    assert _finish(tree) == Status.SUCCESS
    groups = [c.kwargs["commands"] for c in hardware.send_timed_multi_commands.call_args_list]
    assert all(group[0]["desire_time"] == 7.0 for group in groups[:4])
    assert groups[1][0]["cmd_vec"] == [0.14, 1.05, 15.0, 0.0]
    assert all(command["desire_time"] == 3.0 for group in groups[4:] for command in group)
    assert groups[5][0]["cmd_vec"] == [29.0, -41.11, 32.21, 20.0]


def test_empty_case03_torso_stops_before_its_arm_and_case04(hardware):
    tree, board = _tree()
    board.set("case_03__torso_home_commands", [{"planner_index": 2, "cmd_vec": []}])
    assert _finish(tree) == Status.FAILURE
    assert tree.root.children[5].status == Status.INVALID  # No wait after a failed case03.
    assert tree.root.children[6].status == Status.INVALID
    hardware.send_timed_multi_commands.assert_not_called()
    # The only TimedCmd arm call is case02's first phase.
    assert hardware.send_arm_joint_timed.call_count == 1
    hardware.enable_vel_control_jibot.assert_not_called()


def test_dry_run_never_gets_hardware(hardware, monkeypatch):
    for module in MOTION_MODULES:
        monkeypatch.setattr(module, "_DRY_RUN", True)
        monkeypatch.setattr(module, "get_shared_hardware", lambda: pytest.fail("dry-run accessed hardware"))
    tree, _ = _tree()
    assert _finish(tree) == Status.SUCCESS
    assert hardware.mock_calls == []
