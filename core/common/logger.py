# kuavo_application_framework/core/common/logger.py
import logging
import logging.handlers
import os
import sys
import uuid
from pathlib import Path
from contextvars import ContextVar
from datetime import datetime
from typing import Optional

# 用于在异步或多线程环境中存储当前的 Trace ID
trace_id_var: ContextVar[str] = ContextVar('trace_id', default='N/A')

# 全局日志目录
_log_dir: Optional[Path] = None
_initialized: bool = False


class RelativePathFilter(logging.Filter):
    """
    自定义过滤器，将绝对路径转换为相对于项目根目录的相对路径
    
    例如：
    /home/user/project/adapters/hardware/utils.py -> adapters/hardware/utils.py
    """
    def __init__(self, base_path: str = None):
        super().__init__()
        # 如果没有指定基础路径，尝试找到项目根目录
        if base_path:
            self.base_path = Path(base_path).resolve()
        else:
            # 尝试从当前文件位置向上查找项目根目录
            current_file = Path(__file__).resolve()
            # 假设项目根目录是包含 'core' 目录的父目录
            project_root = current_file.parent.parent.parent
            self.base_path = project_root.resolve()
    
    def filter(self, record):
        # 获取文件的绝对路径
        pathname = getattr(record, 'pathname', '')
        if pathname:
            try:
                # 首先将路径转换为绝对路径（处理相对路径的情况）
                abs_path = Path(pathname).resolve()
                # 然后转换为相对路径
                rel_path = abs_path.relative_to(self.base_path)
                # 规范化路径（移除 ../ 等）
                record.rel_path = str(rel_path)
            except ValueError:
                # 如果文件不在基础路径下，使用文件名
                record.rel_path = Path(pathname).name
        else:
            record.rel_path = 'unknown'
        return True


def _load_log_config(config_path: str = None) -> dict:
    """
    从 YAML 配置文件加载日志配置
    
    :param config_path: 配置文件路径，如果为 None 则使用默认路径
    :return: 配置字典
    """
    try:
        import yaml
        
        # 默认配置文件路径
        if config_path is None:
            # 尝试多个可能的位置
            possible_paths = [
                Path(__file__).parent.parent.parent / 'config' / 'log_config.yaml',
                Path('config') / 'log_config.yaml',
                Path('../config') / 'log_config.yaml',
            ]
            
            for path in possible_paths:
                if path.exists():
                    config_path = str(path)
                    break
        
        if config_path and Path(config_path).exists():
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                if config and 'logging' in config:
                    return config['logging']
        
        # 如果没有找到配置文件，返回空字典（将使用默认值）
        return {}
        
    except Exception as e:
        # 如果加载失败，返回空字典，后续将使用默认值
        print(f"Warning: Failed to load log config: {e}")
        return {}


def init_logging(
    log_dir: str = None,
    level: str = None,
    log_format: str = None,
    max_bytes: int = None,
    backup_count: int = None,
    console_output: bool = None,
    file_output: bool = None,
    config_path: str = None
) -> None:
    """
    初始化统一日志系统
    
    :param log_dir: 日志目录路径（如果为 None，则从配置文件读取）
    :param level: 日志级别 (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    :param log_format: 日志格式 (simple, detailed, json)
    :param max_bytes: 单个日志文件最大字节数
    :param backup_count: 保留的备份文件数量
    :param console_output: 是否输出到控制台
    :param file_output: 是否输出到文件
    :param config_path: 配置文件路径（如果为 None，使用默认路径）
    """
    global _log_dir, _initialized
    
    if _initialized:
        return
    
    # 从配置文件加载默认值
    config = _load_log_config(config_path)
    
    # 使用传入参数或配置文件中的值
    log_dir = log_dir or config.get('log_dir', 'log')
    level = level or config.get('level', 'INFO')
    log_format = log_format or config.get('format', 'detailed')
    max_bytes = max_bytes or config.get('max_bytes', 10 * 1024 * 1024)
    backup_count = backup_count if backup_count is not None else config.get('backup_count', 5)
    console_output = console_output if console_output is not None else config.get('console_output', True)
    file_output = file_output if file_output is not None else config.get('file_output', True)
    
    # 设置日志目录
    _log_dir = Path(log_dir)
    _log_dir.mkdir(parents=True, exist_ok=True)
    
    # 获取根日志器
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    
    # 清除现有处理器
    root_logger.handlers.clear()
    
    # 创建格式化器
    if log_format == "simple":
        formatter = logging.Formatter(
            '[%(asctime)s] [%(levelname)s] %(name)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
    elif log_format == "json":
        # 简单的 JSON 格式（实际项目可使用 python-json-logger）
        formatter = logging.Formatter(
            '{"time": "%(asctime)s", "level": "%(levelname)s", "logger": "%(name)s", "message": "%(message)s"}',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
    else:  # detailed
        # 控制台使用简洁格式（不含文件信息）
        console_formatter = logging.Formatter(
            f'[%(asctime)s] [TRACE:{trace_id_var.get()}] [%(levelname)s] %(name)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        # 文件日志使用详细格式（含相对路径和行号，解决同名文件问题）
        # 注意：已有文件路径，因此移除 logger 名称避免重复
        file_formatter = logging.Formatter(
            f'[%(asctime)s] [TRACE:{trace_id_var.get()}] [%(levelname)s] [%(rel_path)s:%(lineno)d] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
    
    # 控制台处理器
    if console_output:
        console_handler = logging.StreamHandler(sys.stdout)
        # 控制台使用简洁格式（不含文件信息，便于实时查看）
        console_fmt = console_formatter if log_format == 'detailed' else formatter
        console_handler.setFormatter(console_fmt)
        console_handler.setLevel(getattr(logging, level.upper(), logging.INFO))
        root_logger.addHandler(console_handler)
    
    # 文件处理器（按大小轮转）
    if file_output:
        # 创建相对路径过滤器（基于当前工作目录）
        rel_path_filter = RelativePathFilter(Path.cwd())
        
        # 主日志文件（精确到秒，避免覆盖）
        log_file = _log_dir / f"kuavo_studio_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        file_handler = logging.handlers.RotatingFileHandler(
            log_file,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding='utf-8'
        )
        # 文件日志使用详细格式（含文件和行号，便于问题定位）
        file_fmt = file_formatter if log_format == 'detailed' else formatter
        file_handler.setFormatter(file_fmt)
        file_handler.addFilter(rel_path_filter)  # 添加相对路径过滤器
        file_handler.setLevel(getattr(logging, level.upper(), logging.INFO))
        root_logger.addHandler(file_handler)
        
        # 错误日志文件（精确到秒，避免覆盖）
        error_log_file = _log_dir / f"kuavo_studio_error_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        error_handler = logging.handlers.RotatingFileHandler(
            error_log_file,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding='utf-8'
        )
        error_formatter = logging.Formatter(
            f'[%(asctime)s] [TRACE:{trace_id_var.get()}] [%(levelname)s] [%(rel_path)s:%(lineno)d] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        error_handler.setFormatter(error_formatter)
        error_handler.addFilter(rel_path_filter)  # 添加相对路径过滤器
        error_handler.setLevel(logging.ERROR)
        root_logger.addHandler(error_handler)
    
    _initialized = True
    
    # 如果没有设置 Trace ID，自动生成一个（基于时间戳和随机数）
    if trace_id_var.get() == 'N/A':
        # 生成简短的 Trace ID（8位十六进制）
        auto_trace_id = uuid.uuid4().hex[:8]
        trace_id_var.set(auto_trace_id)
    
    # 记录初始化信息
    root_logger.info(f"Logging system initialized. Log directory: {_log_dir}")
    root_logger.info(f"Log level: {level}, Format: {log_format}, Trace ID: {trace_id_var.get()}")


def get_logger(name: str) -> logging.Logger:
    """
    获取日志器实例
    
    :param name: 日志器名称（通常是 __name__）
    :return: Logger 实例
    """
    # 如果未初始化，使用默认配置
    if not _initialized:
        init_logging()
    
    logger = logging.getLogger(name)
    return logger


def set_trace_id(trace_id: str = None):
    """设置当前上下文的 Trace ID"""
    trace_id_var.set(trace_id or str(uuid.uuid4())[:8])


def get_log_dir() -> Path:
    """获取日志目录"""
    if _log_dir is None:
        init_logging()
    return _log_dir