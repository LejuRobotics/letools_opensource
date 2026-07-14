# -*- coding: utf-8 -*-
"""加载 LeTools/config/app_config.yaml。"""

from pathlib import Path
from typing import Any, Dict

import yaml

_APP_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "app_config.yaml"
_cached: Dict[str, Any] = None


def get_app_config_path() -> Path:
    return _APP_CONFIG_PATH


def load_app_config(reload: bool = False) -> Dict[str, Any]:
    global _cached
    if _cached is not None and not reload:
        return _cached
    if not _APP_CONFIG_PATH.exists():
        raise FileNotFoundError(f"app_config not found: {_APP_CONFIG_PATH}")
    with open(_APP_CONFIG_PATH, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    _cached = data
    return data


def get_hardware_factory_config() -> Dict[str, Any]:
    """供 HardwareFactory.create_hardware 使用的配置 dict。"""
    cfg = load_app_config()
    robot_type = cfg.get("robot_type")
    if not robot_type:
        raise ValueError("app_config.yaml missing robot_type")
    return dict(cfg)
