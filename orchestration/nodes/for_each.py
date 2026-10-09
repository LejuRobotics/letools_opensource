"""依次把黑板列表元素写入目标键，并为每个元素执行一次子树。"""

from copy import deepcopy

import py_trees
from py_trees.common import Access, Status


class ForEach(py_trees.decorators.Decorator):
    """对 ``source_key`` 中的列表逐项执行同一个子树。"""

    def __init__(self, name, child, source_key, target_key, index_key=""):
        super().__init__(name=name, child=child)
        self.source_key = str(source_key).strip()
        self.target_key = str(target_key).strip()
        self.index_key = str(index_key).strip()
        if not self.source_key or not self.target_key:
            raise ValueError("ForEach requires source_key and target_key")
        if self.index_key and self.index_key in (self.source_key, self.target_key):
            raise ValueError("ForEach index_key 不能覆盖 source_key 或 target_key")
        self.blackboard = self.attach_blackboard_client(name=f"{name}_items")
        self.blackboard.register_key(key=self.source_key, access=Access.READ)
        self.blackboard.register_key(key=self.target_key, access=Access.WRITE)
        if self.index_key:
            self.blackboard.register_key(key=self.index_key, access=Access.WRITE)
        self._items = []
        self._index = 0
        self._error = ""

    def initialise(self):
        self._index = 0
        self._error = ""
        try:
            source = self.blackboard.get(self.source_key)
            if not isinstance(source, list) or not source:
                raise ValueError(f"黑板键 {self.source_key} 必须是非空列表")
            self._items = deepcopy(source)
            self._write_current_item()
        except Exception as exc:
            self._items = []
            self._error = str(exc)

    def tick(self):
        """源列表无效时不要 tick 子树，直接返回失败。"""
        if self.status != Status.RUNNING:
            self.initialise()
        if self._error:
            self.feedback_message = f"遍历失败: {self._error}"
            self.stop(Status.FAILURE)
            self.status = Status.FAILURE
            yield self
            return
        for node in self.decorated.tick():
            yield node
        new_status = self.update()
        if new_status != Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def update(self):
        if self._error:
            self.feedback_message = f"遍历失败: {self._error}"
            return Status.FAILURE
        if self.decorated.status == Status.FAILURE:
            self.feedback_message = f"第 {self._index + 1} 项执行失败"
            return Status.FAILURE
        if self.decorated.status != Status.SUCCESS:
            return Status.RUNNING

        self._index += 1
        if self._index >= len(self._items):
            self.feedback_message = f"已完成 {len(self._items)} 项"
            return Status.SUCCESS

        self.decorated.stop(Status.INVALID)
        try:
            self._write_current_item()
        except Exception as exc:
            self._error = str(exc)
            self.feedback_message = f"遍历失败: {self._error}"
            return Status.FAILURE
        self.feedback_message = f"开始第 {self._index + 1}/{len(self._items)} 项"
        return Status.RUNNING

    def _write_current_item(self):
        self.blackboard.set(self.target_key, deepcopy(self._items[self._index]))
        if self.index_key:
            # 每次进入 ForEach 都从 0 开始，供子树判断本批的第一项。
            self.blackboard.set(self.index_key, self._index)
