# -*- coding: utf-8 -*-
"""RunIfIndex：按索引门控装饰器 — 仅当黑板 nav_point_index 匹配时才执行子节点。

继承 py_trees.decorators.Decorator，由 BehaviorTreeFactory 作为装饰器节点创建。

黑板键 nav_point_index 支持三种格式:
  - 0 或未设置 → 执行所有子节点（默认，兼容原流程）
  - 3           → 只执行 index=3 的子节点
  - "1,3,5"     → 只执行 index 在列表中的子节点（逗号分隔）
"""

import py_trees
from py_trees.common import Status
from py_trees.decorators import Decorator


class RunIfIndex(Decorator):
    """按索引门控装饰器。

    构造函数参数（由 BehaviorTreeFactory 注入）:
        name: 节点名称
        child: 被装饰的子节点
        index: 当前门控的索引号 (1-based)
        nav_point_index_key: 黑板中存储目标索引的键名
    """

    def __init__(self, name, child, index=1, nav_point_index_key="nav_point_index"):
        super().__init__(name=name, child=child)
        self._index = int(index)
        self._nav_point_index_key = str(nav_point_index_key)
        self._should_run = True
        self._resolved = False

    def initialise(self):
        self._resolved = False
        self._should_run = True

    def update(self):
        if not self._resolved:
            self._resolved = True
            self._should_run = self._check_should_run()

            if not self._should_run:
                self.feedback_message = f"SKIP gate[{self._index}]"

        if not self._should_run:
            return Status.SUCCESS

        return self.decorated.status

    def _check_should_run(self):
        raw = self._read_blackboard()
        indices = self._parse_indices(raw)
        # 空或包含 0 → 全部执行；列表包含自己的 index → 执行
        if not indices or 0 in indices:
            return True
        return self._index in indices

    def _read_blackboard(self):
        try:
            bb = self.attach_blackboard_client()
            bb.register_key(key=self._nav_point_index_key, access=py_trees.common.Access.READ)
            if bb.exists(self._nav_point_index_key):
                return bb.get(self._nav_point_index_key)
        except Exception:
            pass
        return 0

    @staticmethod
    def _parse_indices(value):
        """解析黑板值为整数集合。

        "0"       → {0}        全部执行
        3         → {3}        只跑第 3 个
        "1,3,5"   → {1, 3, 5}  只跑 1/3/5
        "1, 3, 5" → {1, 3, 5}  空格兼容
        "all"     → {0}        全部执行
        """
        if value is None:
            return set()
        if isinstance(value, (int, float)):
            return {int(value)}
        s = str(value).strip().lower()
        if not s or s in ("all", "none"):
            return {0}
        # 逗号分隔
        result = set()
        for part in s.split(","):
            part = part.strip()
            if not part:
                continue
            try:
                result.add(int(part))
            except ValueError:
                pass
        return result if result else {0}
