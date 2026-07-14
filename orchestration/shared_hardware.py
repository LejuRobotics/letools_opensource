# -*- coding: utf-8 -*-
"""共享 IHardware 单例，供编排节点与 Skill 注入。

注意：硬件导入链（openai/numpy/kuavo_humanoid_sdk）通过懒加载延迟到首次调用，
避免在模块导入阶段触发约 0.5s 的初始化开销。
"""

import threading

_shared_hardware = None
_lock = threading.Lock()
_config_override = None


def set_hardware_config(config: dict):
    """在首次 get_shared_hardware() 前设置配置覆盖（如 skip 选项）。

    必须在硬件初始化前调用，否则不生效。
    """
    global _config_override
    with _lock:
        if _shared_hardware is not None:
            raise RuntimeError("硬件已初始化，无法更改配置")
        _config_override = config


def get_shared_hardware():
    """获取共享 IHardware 实例（单例，线程安全）。"""
    global _shared_hardware
    with _lock:
        if _shared_hardware is None:
            from adapters.hardware.factory import HardwareFactory  # 懒加载
            from core.common.app_config import get_hardware_factory_config

            config = get_hardware_factory_config()
            if _config_override:
                config.update(_config_override)

            _shared_hardware = HardwareFactory.create_hardware(config=config)
            if hasattr(_shared_hardware, "initialize"):
                _shared_hardware.initialize()
        return _shared_hardware


def reset_shared_hardware():
    """重置单例（测试或重新初始化）。"""
    global _shared_hardware
    with _lock:
        if _shared_hardware is not None and hasattr(_shared_hardware, "shutdown"):
            try:
                _shared_hardware.shutdown()
            except Exception:
                pass
        _shared_hardware = None
