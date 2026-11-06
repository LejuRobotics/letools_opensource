"""公共工具按需导入，轻量配置模块不应触发日志初始化及其可选依赖。"""

from importlib import import_module

__all__ = ["get_logger", "set_trace_id", "ConfigLoader"]


def __getattr__(name):
    if name in ("get_logger", "set_trace_id"):
        value = getattr(import_module(".logger", __name__), name)
    elif name == "ConfigLoader":
        value = import_module(".config_loader", __name__).ConfigLoader
    else:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(__all__))
