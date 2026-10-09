"""黑板条件相等时执行子树，否则跳过并返回 SUCCESS。"""

from orchestration.nodes.run_if_blackboard import RunIfBlackboard


class RunCheck(RunIfBlackboard):
    """开始执行前检查黑板条件，条件不满足时不 tick 子树。"""

    def __init__(self, name, child, condition_key, expected_value=True):
        if not str(condition_key).strip():
            raise ValueError("RunCheck requires condition_key")
        super().__init__(
            name=name,
            child=child,
            condition_key=condition_key,
            expected_value=expected_value,
        )


__all__ = ["RunCheck"]
