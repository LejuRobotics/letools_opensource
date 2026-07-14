# kuavo_application_framework/core/common/config_loader.py
import yaml
from pathlib import Path
from .exceptions import ConfigurationError

class ConfigLoader:
    _instance = None
    _config = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ConfigLoader, cls).__new__(cls)
        return cls._instance

    def load(self, config_path: str):
        path = Path(config_path)
        if not path.exists():
            raise ConfigurationError(f"Config file not found: {config_path}")
        
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        
        self._config = data
        return self._config

    @property
    def config(self):
        if self._config is None:
            raise ConfigurationError("Config not loaded yet. Call load() first.")
        return self._config

def load_config(config_path: str):
    """加载配置文件"""
    loader = ConfigLoader()
    return loader.load(config_path)