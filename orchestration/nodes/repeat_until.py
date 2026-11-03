"""通用黑板条件循环装饰器。"""

from typing import Any

import py_trees
from py_trees.common import Access, Status


class RepeatUntil(py_trees.decorators.Decorator):
    """重复成功的子树，直到黑板条件等于期望值。

    条件在每个 tick 检查；当条件满足时，即使子树仍在运行也会被中止并返回
    ``SUCCESS``。启用 ``wait_for_child_completion`` 后，则等待当前一轮结束，
    再根据子树结果退出。子树失败则立即向上传递 ``FAILURE``。
    启用 ``check_before_iteration`` 后，每轮开始前也检查条件，满足时跳过该轮。
    ``max_iterations`` 限制成功完成的轮数，0 表示不限制；失败不计数。
    """

    def __init__(
        self,
        name,
        child,
        condition_key,
        condition_path="",
        expected_value=True,
        wait_for_child_completion=False,
        check_before_iteration=False,
        max_iterations=0,
    ):
        if isinstance(max_iterations, bool) or not isinstance(max_iterations, int):
            raise TypeError("max_iterations 必须是 int")
        if max_iterations < 0:
            raise ValueError("max_iterations 必须大于等于 0")
        super().__init__(name=name, child=child)
        self.max_iterations = max_iterations
        self.iterations = 0
        self.condition_key = condition_key
        self.condition_path = condition_path
        self.expected_value = expected_value
        self.wait_for_child_completion = wait_for_child_completion
        self.check_before_iteration = check_before_iteration
        self.blackboard = self.attach_blackboard_client(name=f"{name}_condition")
        self.blackboard.register_key(key=condition_key, access=Access.READ)

    def tick(self):
        if self.status != Status.RUNNING:
            self.iterations = 0
        starting_iteration = (
            self.status != Status.RUNNING
            or self.decorated.status == Status.INVALID
        )
        if self.check_before_iteration and starting_iteration:
            try:
                condition_met = self._condition_value() == self.expected_value
            except KeyError:
                condition_met = False
            except Exception as exc:
                self.feedback_message = f"读取循环条件失败: {exc}"
                self.stop(Status.FAILURE)
                yield self
                return
            if condition_met:
                self.feedback_message = (
                    f"本轮开始前循环条件已满足: {self.condition_key}{self._path_label()}"
                )
                self.stop(Status.SUCCESS)
                yield self
                return
        yield from super().tick()

    def update(self):
        if self.decorated.status == Status.SUCCESS:
            self.iterations += 1

        try:
            condition_value = self._condition_value()
        except KeyError:
            condition_value = _MISSING
        except Exception as exc:
            self.feedback_message = f"读取循环条件失败: {exc}"
            return Status.FAILURE

        condition_met = (
            condition_value is not _MISSING
            and condition_value == self.expected_value
        )
        if condition_met and self.wait_for_child_completion:
            if self.decorated.status == Status.FAILURE:
                self.feedback_message = "子流程失败"
                return Status.FAILURE
            if self.decorated.status == Status.SUCCESS:
                self.feedback_message = (
                    f"本轮完成且循环条件满足: {self.condition_key}"
                    f"{self._path_label()} == {self.expected_value!r}"
                )
                return Status.SUCCESS
            self.feedback_message = "循环条件满足，等待本轮完成"
            return Status.RUNNING
        if condition_met:
            if self.decorated.status == Status.RUNNING:
                self.decorated.stop(Status.INVALID)
            self.feedback_message = f"循环条件满足: {self.condition_key}{self._path_label()} == {self.expected_value!r}"
            return Status.SUCCESS
        if self.decorated.status == Status.FAILURE:
            self.feedback_message = "子流程失败"
            return Status.FAILURE
        if self.decorated.status == Status.SUCCESS:
            if self.max_iterations and self.iterations >= self.max_iterations:
                self.feedback_message = f"已完成 {self.iterations}/{self.max_iterations} 轮"
                return Status.SUCCESS
            self.decorated.stop(Status.INVALID)
            self.feedback_message = f"已完成 {self.iterations} 轮，继续下一轮"
            return Status.RUNNING
        return Status.RUNNING

    def _condition_value(self) -> Any:
        value = self.blackboard.get(self.condition_key)
        if not self.condition_path:
            return value
        for segment in self.condition_path.split("."):
            if isinstance(value, dict):
                value = value[segment]
            else:
                value = getattr(value, segment)
        return value

    def _path_label(self):
        return f".{self.condition_path}" if self.condition_path else ""


_MISSING = object()
