"""从可跑通的测试脚本抽出的 Adapter 原子技能集合。

注意：这里**不要**做聚合导入（from .xxx import ...）。
行为树构建阶段可能并行 import 多个叶子节点模块，聚合导入会放大 import 锁竞争，
在某些 Python 版本/线程时序下触发 `_DeadlockError`。
"""

__all__ = []

