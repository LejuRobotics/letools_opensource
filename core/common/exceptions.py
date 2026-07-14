# kuavo_application_framework/core/common/exceptions.py

class KuavoFrameworkError(Exception):
    """框架基础异常类"""
    pass

class HardwareConnectionError(KuavoFrameworkError):
    """硬件连接失败或断开时抛出"""
    pass

class SkillExecutionError(KuavoFrameworkError):
    """技能执行过程中发生逻辑错误或超时"""
    pass

class ConfigurationError(KuavoFrameworkError):
    """配置文件缺失或格式错误时抛出"""
    pass

class KinematicsError(KuavoFrameworkError):
    """运动学解算（如逆解）失败时抛出"""
    pass