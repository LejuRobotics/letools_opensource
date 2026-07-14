from .behavior_tree_factory import BehaviorTreeFactory

__all__ = ["BehaviorTreeFactory", "BehaviorTreeEngine"]


def __getattr__(name):
    if name == "BehaviorTreeEngine":
        from .behavior_tree_engine import BehaviorTreeEngine

        return BehaviorTreeEngine
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
