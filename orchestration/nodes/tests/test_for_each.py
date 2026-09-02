"""ForEach 黑板批次迭代测试。"""

import py_trees
from py_trees.common import Access, Status

from orchestration.nodes.for_each import ForEach


class RecordCurrent(py_trees.behaviour.Behaviour):
    def __init__(self, seen):
        super().__init__(name="record_current")
        self.seen = seen
        self.blackboard = self.attach_blackboard_client(name="record_current")
        self.blackboard.register_key(key="obj_xyz", access=Access.READ)

    def update(self):
        self.seen.append(dict(self.blackboard.get("obj_xyz")))
        return Status.SUCCESS


def test_for_each_runs_child_once_per_ordered_item():
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="for_each_input")
    writer.register_key(key="pick_batch", access=Access.WRITE)
    writer.set(
        "pick_batch",
        [
            {"x": 0.8, "y": -0.2, "z": 0.7},
            {"x": 0.9, "y": 0.3, "z": 0.75},
        ],
    )
    seen = []
    node = ForEach(
        name="for_each",
        child=RecordCurrent(seen),
        source_key="pick_batch",
        target_key="obj_xyz",
    )
    tree = py_trees.trees.BehaviourTree(node)

    tree.tick()
    assert node.status == Status.RUNNING
    tree.tick()
    assert node.status == Status.SUCCESS
    assert [item["y"] for item in seen] == [-0.2, 0.3]
    assert writer is not None


def test_for_each_rejects_empty_batch():
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="for_each_empty_input")
    writer.register_key(key="pick_batch", access=Access.WRITE)
    writer.set("pick_batch", [])
    node = ForEach(
        name="for_each",
        child=RecordCurrent([]),
        source_key="pick_batch",
        target_key="obj_xyz",
    )
    py_trees.trees.BehaviourTree(node).tick()
    assert node.status == Status.FAILURE
    assert writer is not None
