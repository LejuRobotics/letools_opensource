"""RepeatUntil 装饰器测试。"""

import py_trees
from py_trees.common import Access, Status
import pytest

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


class _CountIterations(py_trees.behaviour.Behaviour):
    def __init__(self, name="child", fail_on=0):
        super().__init__(name=name)
        self.started = 0
        self.tick_count = 0
        self.fail_on = fail_on

    def initialise(self):
        self.started += 1
        self.tick_count = 0

    def update(self):
        self.tick_count += 1
        if self.tick_count == 1:
            return Status.RUNNING
        return Status.FAILURE if self.started == self.fail_on else Status.SUCCESS


def _repeat(child, **params):
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="loop-writer")
    writer.register_key(key="stop_requested", access=Access.WRITE)
    writer.set("stop_requested", False)
    repeat = RepeatUntil("repeat", child, "stop_requested", **params)
    return repeat, writer


@pytest.mark.parametrize("count", [1, 3])
def test_counts_only_completed_successful_iterations(count):
    child = _CountIterations()
    repeat, _ = _repeat(child, max_iterations=count)
    for tick in range(count * 2):
        repeat.tick_once()
        assert repeat.iterations == (tick + 1) // 2
        expected = Status.SUCCESS if tick == count * 2 - 1 else Status.RUNNING
        assert repeat.status == expected
    assert child.started == count


def test_zero_limit_and_default_keep_repeating():
    for params in ({}, {"max_iterations": 0}):
        child = _CountIterations()
        repeat, _ = _repeat(child, **params)
        for _ in range(8):
            repeat.tick_once()
            assert repeat.status == Status.RUNNING
        assert repeat.iterations == child.started == 4


def test_failure_does_not_count_or_retry_iteration():
    child = _CountIterations(fail_on=2)
    repeat, _ = _repeat(child, max_iterations=3)
    for _ in range(4):
        repeat.tick_once()
    assert repeat.status == Status.FAILURE
    assert repeat.iterations == 1
    assert child.started == 2


def test_stop_request_waits_for_complete_iteration():
    child = _CountIterations()
    repeat, writer = _repeat(child, max_iterations=3, wait_for_child_completion=True,
                             check_before_iteration=True)
    repeat.tick_once()
    writer.set("stop_requested", True)
    repeat.tick_once()
    assert repeat.status == Status.SUCCESS
    assert repeat.iterations == child.started == 1


def test_precheck_skips_first_or_next_iteration_and_resets_count_on_reentry():
    child = _CountIterations()
    repeat, writer = _repeat(child, max_iterations=3, check_before_iteration=True)
    writer.set("stop_requested", True)
    repeat.tick_once()
    assert repeat.status == Status.SUCCESS
    assert repeat.iterations == child.started == 0
    writer.set("stop_requested", False)
    repeat.tick_once()
    repeat.tick_once()
    assert repeat.status == Status.RUNNING
    assert repeat.iterations == child.started == 1
    writer.set("stop_requested", True)
    repeat.tick_once()
    assert repeat.status == Status.SUCCESS
    assert repeat.iterations == child.started == 1
    repeat.stop(Status.INVALID)
    repeat.tick_once()
    assert repeat.iterations == 0
    assert child.started == 1


def test_reentry_resets_iteration_limit_after_success():
    child = _CountIterations()
    repeat, _ = _repeat(child, max_iterations=1)
    for expected_starts in (1, 2):
        repeat.tick_once()
        assert repeat.status == Status.RUNNING
        assert repeat.iterations == 0
        repeat.tick_once()
        assert repeat.status == Status.SUCCESS
        assert repeat.iterations == 1
        assert child.started == expected_starts


def test_default_condition_can_interrupt_without_counting_incomplete_child():
    child = _CountIterations()
    repeat, writer = _repeat(child, max_iterations=3)
    writer.set("stop_requested", True)
    repeat.tick_once()
    assert repeat.status == Status.SUCCESS
    assert repeat.iterations == 0
    assert child.status == Status.INVALID


@pytest.mark.parametrize("value", [True, False, 1.5, "3", None, -1])
def test_rejects_invalid_iteration_limit(value):
    with pytest.raises((TypeError, ValueError), match="max_iterations"):
        _repeat(_CountIterations(), max_iterations=value)


if __name__ == "__main__":
    test_waits_for_current_iteration_before_exiting()
    print("PASS test_waits_for_current_iteration_before_exiting")
