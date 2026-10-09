"""RepeatUntil 装饰器测试。"""

import py_trees
from py_trees.common import Access, Status

from orchestration.nodes.repeat_until import RepeatUntil


class _FinishOnSecondTick(py_trees.behaviour.Behaviour):
    def __init__(self, name, status_writer):
        super().__init__(name=name)
        self.status_writer = status_writer
        self.tick_count = 0

    def update(self):
        self.tick_count += 1
        self.status_writer.set("loop_status", {"is_finished": True})
        return Status.SUCCESS if self.tick_count == 2 else Status.RUNNING


def test_waits_for_current_iteration_before_exiting(check_before_iteration=False):
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="repeat-until-test-writer")
    writer.register_key(key="loop_status", access=Access.WRITE)
    child = _FinishOnSecondTick("child", writer)
    repeat = RepeatUntil(
        name="repeat",
        child=child,
        condition_key="loop_status",
        condition_path="is_finished",
        wait_for_child_completion=True,
        check_before_iteration=check_before_iteration,
    )

    repeat.tick_once()
    assert repeat.status == Status.RUNNING
    assert child.status == Status.RUNNING

    repeat.tick_once()
    assert repeat.status == Status.SUCCESS
    assert child.tick_count == 2


def test_precheck_does_not_interrupt_running_iteration():
    test_waits_for_current_iteration_before_exiting(check_before_iteration=True)


if __name__ == "__main__":
    test_waits_for_current_iteration_before_exiting()
    print("PASS test_waits_for_current_iteration_before_exiting")
