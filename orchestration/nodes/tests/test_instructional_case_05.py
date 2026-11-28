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
import orchestration.nodes.timed_arm_joint as arm
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
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hw = MagicMock()
    for name in ("control_head_sdk", "timed_arm_joint", "send_arm_joint_traj_sdk",
                 "send_timed_multi_commands", "set_arm_control_mode", "set_focus_ee",
                 "set_focus_z", "enable_vel_control_jibot", "base_move_relative_jibot"):
        getattr(hw, name).return_value = Result.ok(data={"actual_time": 3.0})
    hw.get_vel_control_state_jibot.return_value = Result.ok(data={"state": True})
    for module in MOTION_MODULES:
        if module is not arm:
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
            output = node.get("params", {}).get("actual_time_output_key")
            if output:
                assert output["value"].startswith(prefix)
                output["value"] = output["value"][len(prefix):]
        assert unwrapped == _load(SCENARIOS / case_name / "py_tree.json")["tree"]
    assert len(board) == total_keys


def test_executes_each_case_once_in_order_and_restores_chassis_guard(hardware):
    tree, _ = _tree()
    assert len(list(tree.root.iterate())) == 76
    for index, case_root in enumerate(tree.root.children[::2], start=1):
        assert case_root.subtree_file == "instructional_case_%02d.json" % index
        assert case_root.name.startswith("case_%02d__" % index)
    assert _finish(tree) == Status.SUCCESS
    motion_calls = [c[0] for c in hardware.mock_calls if c[0] in {
        "control_head_sdk", "timed_arm_joint", "send_arm_joint_traj_sdk",
        "send_timed_multi_commands", "base_move_relative_jibot",
    }]
    expected = ["control_head_sdk"] * 6
    expected += ["control_head_sdk", "timed_arm_joint"]
    expected += ["control_head_sdk", "send_arm_joint_traj_sdk"] * 4
    expected += ["control_head_sdk", "send_timed_multi_commands", "timed_arm_joint"] * 4
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
    # Keep real WaitSeconds at case boundaries, bypass internal waits here;
    # arm actual_time + settle_time is covered independently below.
    for case_root in tree.root.children[::2]:
        for node in case_root.iterate():
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
    assert hardware.timed_arm_joint.call_count == 1
    hardware.enable_vel_control_jibot.assert_not_called()


def test_dry_run_never_gets_hardware(hardware, monkeypatch):
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    for module in MOTION_MODULES:
        if module is not arm:
            monkeypatch.setattr(module, "_DRY_RUN", True)
        monkeypatch.setattr(module, "get_shared_hardware", lambda: pytest.fail("dry-run accessed hardware"))
    tree, _ = _tree()
    assert _finish(tree) == Status.SUCCESS
    assert hardware.mock_calls == []


ARM_LABELS = (
    "case_02__arm_zero", "case_03__arms_home", "case_03__arms_left",
    "case_03__arms_right", "case_03__arms_return_home",
)


def _arm_sequence(tree, label):
    return next(node for node in tree.root.iterate() if node.name == label)


@pytest.mark.parametrize("label", ARM_LABELS)
@pytest.mark.parametrize("result_data", [{"actual_time": 3.0}, {}, None])
def test_arm_wait_uses_actual_time_or_runtime_desire_time_without_resending(
        hardware, monkeypatch, label, result_data):
    clock = [100.0]
    monkeypatch.setattr("time.time", lambda: clock[0])
    monkeypatch.setattr(wait, "_DRY_RUN", False)
    hardware.timed_arm_joint.return_value = Result.ok(data=result_data)
    tree, board = _tree()
    sequence = _arm_sequence(tree, label)
    send, pause = sequence.children
    # Updates after construction must be read at initialise, not from snapshots.
    duration_key = send.params["desire_time__board_key"]
    board.set(duration_key, 7.0)
    angles = _load(SCENARIOS / "instructional_case_03/board.json")
    target = angles["arm_home_joint_angles"][:7] + angles["arm_left_joint_angles"][7:]
    board.set(send.params["joint_angles__board_key"], target)
    settle = 0.0
    if "settle_time__board_key" in pause.params:
        settle = 0.5
        board.set(pause.params["settle_time__board_key"], settle)
    sequence.tick_once()
    assert send.status == Status.SUCCESS  # Send node itself never waits.
    assert pause.status == sequence.status == Status.RUNNING
    hardware.timed_arm_joint.assert_called_once_with(
        left_arm_joint=target[:7], right_arm_joint=target[7:], desire_time=7.0, is_sync=True)
    hardware.set_arm_control_mode.assert_not_called()  # Preserve original scenario mode semantics.
    planned = 3.0 if result_data and result_data.get("actual_time") is not None else 7.0
    assert pause._skill.params.duration_sec == planned + settle
    # Once the send has succeeded, changing the desired duration cannot change
    # the independent wait's fallback: it must use the sent command's snapshot.
    board.set(duration_key, 3.0)
    output_key = send.params["actual_time_output_key"]
    assert board.get(output_key) == planned
    calls_before = list(hardware.mock_calls)
    for offset in (0.0, 0.1, planned + settle - 0.001):
        clock[0] = 100.0 + offset
        sequence.tick_once()
        assert pause.status == sequence.status == Status.RUNNING
        assert hardware.mock_calls == calls_before
    clock[0] = 100.0 + planned + settle
    sequence.tick_once()
    assert sequence.status == Status.SUCCESS
    assert hardware.mock_calls == calls_before


@pytest.mark.parametrize("label", ARM_LABELS)
def test_failed_arm_send_clears_old_duration_and_never_starts_wait(hardware, monkeypatch, label):
    monkeypatch.setattr(wait, "_DRY_RUN", False)
    hardware.timed_arm_joint.return_value = Result.fail("send rejected")
    tree, board = _tree()
    sequence = _arm_sequence(tree, label)
    send, pause = sequence.children
    board.set(send.params["actual_time_output_key"], 3.0)
    sequence.tick_once()
    assert sequence.status == send.status == Status.FAILURE
    assert pause.status == Status.INVALID
    assert pause._skill is None
    assert board.get(send.params["actual_time_output_key"]) is None
    assert send.update() == Status.FAILURE
    assert send.update() == Status.FAILURE
    hardware.timed_arm_joint.assert_called_once()


@pytest.mark.parametrize("joint_angles", [[], [0.0] * 7, [0.0] * 15])
def test_invalid_legacy_arm_target_fails_before_send_or_wait(hardware, joint_angles):
    tree, board = _tree()
    sequence = _arm_sequence(tree, ARM_LABELS[0])
    send, pause = sequence.children
    board.set(send.params["joint_angles__board_key"], joint_angles)
    sequence.tick_once()
    assert sequence.status == Status.FAILURE
    assert pause.status == Status.INVALID
    hardware.timed_arm_joint.assert_not_called()


def test_native_timed_arm_joint_defaults_remain_send_only_with_mode_preparation(hardware):
    node = arm.TimedArmJoint("native", "native", "/", {
        "left_arm_joint": [0.0] * 7, "desire_time": 3.0,
    })
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert node.update() == Status.SUCCESS
    hardware.set_arm_control_mode.assert_called_once_with(2)
    hardware.timed_arm_joint.assert_called_once_with(
        left_arm_joint=[0.0] * 7, right_arm_joint=None, desire_time=3.0, is_sync=True)


@pytest.mark.parametrize("actual_time", [None, True, False, -1.0, float("nan"), float("inf"), "bad", 10 ** 400],
                         ids=["null", "true", "false", "negative", "nan", "infinity", "text", "overflow"])
def test_invalid_planned_duration_clears_output_and_never_waits_or_resends(hardware, actual_time):
    hardware.timed_arm_joint.return_value = Result.ok(data={"actual_time": actual_time})
    tree, board = _tree()
    sequence = _arm_sequence(tree, ARM_LABELS[0])
    send, pause = sequence.children
    board.set(send.params["actual_time_output_key"], 3.0)
    sequence.tick_once()
    assert sequence.status == send.status == Status.FAILURE
    assert pause.status == Status.INVALID
    assert board.get(send.params["actual_time_output_key"]) is None
    assert send.update() == Status.FAILURE
    hardware.timed_arm_joint.assert_called_once()


@pytest.mark.parametrize("key", ["duration_sec", "settle_time"])
@pytest.mark.parametrize("value", [None, True, False, -1.0, float("nan"), float("inf"), "bad", 10 ** 400],
                         ids=["null", "true", "false", "negative", "nan", "infinity", "text", "overflow"])
def test_invalid_wait_duration_fails_without_waiting(hardware, monkeypatch, key, value):
    monkeypatch.setattr(wait, "_DRY_RUN", False)
    node = wait.WaitSeconds("invalid_wait", "invalid_wait", "/", {key: value})
    node.initialise()
    assert node.update() == Status.FAILURE
    assert node._skill is None
    assert "init failed" in node.feedback_message
    assert hardware.mock_calls == []


def test_legacy_and_native_arm_targets_are_mutually_exclusive(hardware):
    node = arm.TimedArmJoint("mixed", "mixed", "/", {
        "joint_angles": [0.0] * 14, "left_arm_joint": [0.0] * 7, "desire_time": 3.0,
    })
    node.initialise()
    assert node.update() == Status.FAILURE
    hardware.timed_arm_joint.assert_not_called()
