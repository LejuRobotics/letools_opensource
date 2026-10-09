"""使用场景 JSON 验证逐手观测和首手导航；动作全部替换为无硬件桩。"""

import json
from pathlib import Path
from unittest.mock import patch

import py_trees
import pytest
from py_trees.common import Access, Status

from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
from orchestration.nodes.run_check import RunCheck


SCENARIO = Path(__file__).resolve().parents[2] / "scenarios/zhaofeng_feeding_internal"


@pytest.fixture
def board():
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="place-navigation-test")
    for key in ("pick_batch", "place_item_index", "place_back_required"):
        writer.register_key(key=key, access=Access.WRITE)
    writer.set("place_back_required", False)
    yield writer
    py_trees.blackboard.Blackboard.clear()


class _Action(py_trees.behaviour.Behaviour):
    def __init__(self, label, board, events, fail_navigation):
        super().__init__(label)
        self.board = board
        self.events = events
        self.fail_navigation = fail_navigation
        self.count = 0

    def initialise(self):
        self.count = 0
        self.events.append((self.name, self.board.get("place_item_index"), "start"))

    def update(self):
        self.count += 1
        if self.name == "nav_to_place_observe":
            if self.fail_navigation:
                return Status.FAILURE
            if self.count < 3:
                return Status.RUNNING
        if self.name == "move_to_place_observe" and self.count < 2:
            return Status.RUNNING
        if self.name == "apriltag_perception_current_hand":
            return Status.RUNNING
        self.events.append((self.name, self.board.get("place_item_index"), "done"))
        return Status.SUCCESS


def _place_loop(board, events, fail_navigation=False):
    main = json.loads((SCENARIO / "py_tree.json").read_text())

    def find(node):
        if node["name"] == "ForEach" and node.get("params", {}).get("index_key", {}).get("value") == "place_item_index":
            return node
        for child in node.get("childs", []):
            found = find(child)
            if found is not None:
                return found
        return None

    factory = BehaviorTreeFactory(
        board, enable_parallel_loading=False,
        subtree_json_path=str(SCENARIO / "py_tree_child.json"),
    )
    with patch.object(factory, "_create_node_instance", side_effect=lambda name, label, namespace, params: _Action(label, board, events, fail_navigation)):
        return factory._build_tree_recursive(find(main["tree"]), parent_namespace=None)


@pytest.mark.parametrize("item_count", [1, 2])
def test_first_hand_navigates_each_batch_and_every_hand_prepares(board, item_count):
    events = []
    node = _place_loop(board, events)
    for batch in range(2):
        board.set("pick_batch", [{"active_arm": hand} for hand in ("right", "left")[:item_count]])
        events.clear()
        node.tick_once()
        assert node.status == Status.RUNNING
        assert ("nav_to_place_observe", 0, "start") in events
        assert ("move_to_place_observe", 0, "start") in events
        assert not any(name == "plan_current_hand_clear_place" for name, _, _ in events)
        for _ in range(10):
            node.tick_once()
            if node.status != Status.RUNNING:
                break
        assert node.status == Status.SUCCESS
        assert [index for name, index, phase in events if name == "nav_to_place_observe" and phase == "start"] == [0]
        for index in range(item_count):
            observe = events.index(("move_to_place_observe", index, "done"))
            perceive = events.index(("plan_current_hand_clear_place", index, "start"))
            compute = events.index(("compute_place_key_points_by_hand", index, "start"))
            control = events.index(("move_to_place_ready", index, "start"))
            assert observe < perceive < compute < control
            assert events.index(("nav_to_place_observe", 0, "done")) < perceive
        node.stop(Status.INVALID)


def test_navigation_failure_prevents_perception_and_place(board):
    board.set("pick_batch", [{"active_arm": "left"}])
    events = []
    node = _place_loop(board, events, fail_navigation=True)
    node.tick_once()
    assert node.status == Status.FAILURE
    assert not any(name in ("plan_current_hand_clear_place", "move_to_place_ready") for name, _, _ in events)


def test_missing_gate_condition_fails_without_ticking_child(board):
    events = []
    child = _Action("nav_to_place_observe", board, events, False)
    gate = RunCheck("gate", child, "place_item_index", 0)
    gate.tick_once()
    assert gate.status == Status.FAILURE
    assert events == []
