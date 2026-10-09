"""黑板条件相等时执行子树，否则跳过并返回 SUCCESS。"""

import py_trees
from py_trees.common import Access, Status


class RunIfBlackboard(py_trees.decorators.Decorator):
    """开始执行前判断一次；条件不满足时不 tick 子树。"""

    def __init__(self, name, child, condition_key, expected_value=True):
        super().__init__(name=name, child=child)
        self.condition_key = str(condition_key).strip()
        if not self.condition_key:
            raise ValueError("RunIfBlackboard requires condition_key")
        self.expected_value = expected_value
        self.blackboard = self.attach_blackboard_client(name=f"{name}_condition")
        self.blackboard.register_key(key=self.condition_key, access=Access.READ)

    def tick(self):
        if self.status != Status.RUNNING:
            try:
                should_run = self.blackboard.get(self.condition_key) == self.expected_value
            except Exception as exc:
                self.feedback_message = f"读取执行条件失败: {exc}"
                self.stop(Status.FAILURE)
                yield self
                return
            if not should_run:
                self.feedback_message = f"跳过：{self.condition_key} != {self.expected_value!r}"
                self.stop(Status.SUCCESS)
                yield self
                return
        yield from super().tick()

    def update(self):
        return self.decorated.status
