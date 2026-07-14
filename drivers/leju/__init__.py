# kuavo_application_framework/drivers/leju/__init__.py
"""
驱动层：乐聚机器人原始 SDK 入口。
根据架构定义，此处不进行业务逻辑封装，仅提供底层通信能力。
"""

try:
    # 尝试导入乐聚官方 SDK
    from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    class KuavoRobot:
        def __init__(self, *args, **kwargs): pass
    class KuavoRobotState:
        def __init__(self, *args, **kwargs): pass

__all__ = ['KuavoRobot', 'KuavoRobotState', 'SDK_AVAILABLE']