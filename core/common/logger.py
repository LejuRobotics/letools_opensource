# kuavo_application_framework/core/common/logger.py
import logging
import logging.handlers
import os
import sys
import uuid
import threading
from pathlib import Path
from datetime import datetime
from typing import Optional

import colorlog

# 当前 Trace ID：进程级全局，供所有线程共享。
# 曾用 ContextVar 存储，但 threading.Thread 不会自动复制调用方的 context，
# 导致子线程（SDK 管理器、异步装饰器等）读到默认值 N/A，日志丢失追踪能力。
_trace_id: str = 'N/A'
_trace_id_lock = threading.Lock()

# 全局日志目录
_log_dir: Optional[Path] = None
_initialized: bool = False

# 已打开的日志文件路径：重新配置时复用，避免每次重配都新建一份文件
_active_log_file: Optional[Path] = None
_active_error_log_file: Optional[Path] = None


class TraceIdFilter(logging.Filter):
    """
    为每条日志记录注入当前进程的 Trace ID。

    日志格式必须使用 %(trace_id)s 占位符，不能写成 f-string——f-string 会在
    Formatter 构造时就把当时的 Trace ID 固化进格式串，导致此后 set_trace_id()
    完全失效（表现为整场日志恒为 N/A）。
    """
    def filter(self, record):
        record.trace_id = _trace_id
        return True


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


# 自挂终端 handler 又未关 propagate 的外部 logger：同一条日志会被打印两次。
#
# rosout             — rosgraph 给 logging.getLogger('rosout') 挂的 RosStreamHandler；
#                      rospy 的 loginfo/logwarn/logerr 全部经由该 logger。
# kuavo-humanoid-sdk — 上游 SDK 的 common/logger.py 在包被 import 时就挂了 StreamHandler。
_EXTERNAL_LOGGERS_WITH_OWN_CONSOLE = ('rosout', 'kuavo-humanoid-sdk')


def _writes_to_terminal(handler: logging.Handler) -> bool:
    """
    判断 handler 是否往终端写。

    写文件的 handler 不算终端——它们写的是各自独立的文件，不产生重复，且是排查
    SDK 内部问题的依据，必须保留。注意 FileHandler 继承自 StreamHandler，必须先
    排除，否则 SDK 的 RotatingFileHandler 会被一起摘掉。

    RosStreamHandler 直接继承 logging.Handler（不是 StreamHandler），isinstance
    判断不到，只能按类名识别。其余认不出的 handler 一律按「不往终端写」处理，
    这样 /rosout 话题上报之类的 handler 不会被误伤。
    """
    if isinstance(handler, logging.FileHandler):
        return False
    return isinstance(handler, logging.StreamHandler) or type(handler).__name__ == 'RosStreamHandler'


def suppress_duplicate_console_output() -> None:
    """
    摘掉外部 logger 自挂的终端 handler，避免同一条日志被打印两次。

    上面两个 logger 都同时满足「自己挂了写终端的 handler」与「propagate 为真」，
    因此同一条记录会先被它们自己的 handler 打一遍，再传播到 root 被本模块的
    console handler 打一遍——内容相同、只有格式不同。

    调用时机决定这一趟是否有效：外部 logger 是在其所在包被 import 时才建立的，
    早于本函数调用点就什么也摘不到。因此除了 init_logging() 里的调用之外，
    import 完全部上游 SDK 的调用方（adapters 侧）需要再调一次。摘除是幂等的，
    多调几次无副作用，而且记录仍会经 propagate 统一由 root 打印，不会丢日志。
    """
    for name in _EXTERNAL_LOGGERS_WITH_OWN_CONSOLE:
        external_logger = logging.getLogger(name)
        for handler in list(external_logger.handlers):
            if _writes_to_terminal(handler):
                external_logger.removeHandler(handler)


# 流水账 logger：INFO 级的内容对本项目没有价值。
#
# rospy.internal   — 每次 topic 连接/断开都打一行 "adding connection to ... count N"，
#                    单场启动近 30 行。
# transitions.core — 状态机内部回调流水账，每次状态迁移 3 行，内容还高度重复。
_NOISY_LOGGERS = ('rospy.internal', 'transitions.core')


def set_noisy_loggers_suppressed(suppressed: bool = True) -> None:
    """
    压制／恢复流水账 logger。

    压制是把这两个 logger 的级别提到 WARNING，只保留告警与错误。注意方向：把级别
    设成 DEBUG 是降低门槛、放行更多内容，起不到压制作用。

    恢复同样重要：若压制一旦生效就再也调不出这些流水账，排查 ROS 拓扑问题时反而
    少了线索，因此调试级别下要设回 NOTSET 跟随 root。
    """
    for name in _NOISY_LOGGERS:
        logging.getLogger(name).setLevel(logging.WARNING if suppressed else logging.NOTSET)


# 只在终端上压制的 logger：它们的记录仍会全量写进各自的文件日志。
#
# kuavo-humanoid-sdk — 上游 SDK 在控制指令路径上打 INFO：core.py 的 _is_mpc_mode()
#                      被 11 个控制方法调用，其中 control_robot_end_effector_pose()
#                      位于 50Hz 的轨迹发布循环里，单条 3 秒轨迹能刷出上百行
#                      "[Core] Current controller: mpc"。
#
# 与 _NOISY_LOGGERS 的区别在压制手段：这里不动 logger 级别，只过滤本项目的 console
# handler。SDK 自己挂了 RotatingFileHandler 在同一个 logger 上（见上面
# _EXTERNAL_LOGGERS_WITH_OWN_CONSOLE 的说明），改 logger 级别会让记录在任何 handler
# 之前就被 isEnabledFor() 拦掉，连它的文件日志一起废掉。
_CONSOLE_ONLY_SUPPRESSED_LOGGERS = ('kuavo-humanoid-sdk',)


class _ConsoleNoiseFilter(logging.Filter):
    """把 _CONSOLE_ONLY_SUPPRESSED_LOGGERS 中 INFO 及以下的记录挡在终端之外。

    只挂在 console handler 上——root 的文件 handler 与外部 logger 自挂的文件 handler
    都不受影响，日志落盘依旧全量。
    """

    def __init__(self, suppressed: bool = True):
        super().__init__()
        self.suppressed = suppressed

    def filter(self, record: logging.LogRecord) -> bool:
        if not self.suppressed:
            return True
        if record.name in _CONSOLE_ONLY_SUPPRESSED_LOGGERS:
            return record.levelno >= logging.WARNING
        return True


def init_logging(
    log_dir: str = None,
    level: str = None,
    log_format: str = None,
    max_bytes: int = None,
    backup_count: int = None,
    console_output: bool = None,
    file_output: bool = None,
    config_path: str = None,
    force: bool = False
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
    :param force: 已初始化时是否强制重新配置。rospy.init_node() 会重装 root 上的
                  handler，调用方需在其后用 force=True 重新收敛日志配置。
    """
    global _log_dir, _initialized, _active_log_file, _active_error_log_file
    
    if _initialized and not force:
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

    # 在创建 Formatter 之前确定 Trace ID（Formatter 通过 %(trace_id)s 读取它）
    if _trace_id == 'N/A':
        set_trace_id()

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
        # 控制台使用简洁格式（不含文件信息），并按等级着色以便快速定位 ERROR
        # 传 stream 让 colorlog 在输出被重定向到文件时自动降级为无色
        console_formatter = colorlog.ColoredFormatter(
            '[%(asctime)s] [TRACE:%(trace_id)s] [%(log_color)s%(levelname)s%(reset)s] %(name)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
            stream=sys.stdout
        )
        # 文件日志使用详细格式（含相对路径和行号，解决同名文件问题）
        # 注意：已有文件路径，因此移除 logger 名称避免重复
        file_formatter = logging.Formatter(
            '[%(asctime)s] [TRACE:%(trace_id)s] [%(levelname)s] [%(rel_path)s:%(lineno)d] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
    
    # 控制台处理器
    if console_output:
        console_handler = logging.StreamHandler(sys.stdout)
        # 控制台使用简洁格式（不含文件信息，便于实时查看）
        console_fmt = console_formatter if log_format == 'detailed' else formatter
        console_handler.setFormatter(console_fmt)
        console_handler.setLevel(getattr(logging, level.upper(), logging.INFO))
        console_handler.addFilter(TraceIdFilter())  # 注入 Trace ID
        # 挡掉外部 logger 的终端噪音；DEBUG 级别下放行，与 set_noisy_loggers_suppressed 同约定
        console_handler.addFilter(_ConsoleNoiseFilter(root_logger.level > logging.DEBUG))
        root_logger.addHandler(console_handler)
    
    # 文件处理器（按大小轮转）
    if file_output:
        # 创建相对路径过滤器（基于当前工作目录）
        rel_path_filter = RelativePathFilter(Path.cwd())
        
        # 主日志文件（精确到秒，避免覆盖；重新配置时复用已打开的文件）
        if _active_log_file is None:
            _active_log_file = _log_dir / f"LeTools_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        log_file = _active_log_file
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
        file_handler.addFilter(TraceIdFilter())  # 注入 Trace ID
        file_handler.setLevel(getattr(logging, level.upper(), logging.INFO))
        root_logger.addHandler(file_handler)
        
        # 错误日志文件（精确到秒，避免覆盖；重新配置时复用已打开的文件）
        if _active_error_log_file is None:
            _active_error_log_file = _log_dir / f"LeTools_error_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        error_log_file = _active_error_log_file
        error_handler = logging.handlers.RotatingFileHandler(
            error_log_file,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding='utf-8'
        )
        error_formatter = logging.Formatter(
            '[%(asctime)s] [TRACE:%(trace_id)s] [%(levelname)s] [%(rel_path)s:%(lineno)d] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        error_handler.setFormatter(error_formatter)
        error_handler.addFilter(rel_path_filter)  # 添加相对路径过滤器
        error_handler.addFilter(TraceIdFilter())  # 注入 Trace ID
        error_handler.setLevel(logging.ERROR)
        root_logger.addHandler(error_handler)
    
    _initialized = True
    
    # 收敛外部 logger（rospy 的 rosout 等）自挂的终端 handler，避免日志被打印两次
    suppress_duplicate_console_output()

    # 流水账 logger 只在调试级别下放行——排查 ROS 拓扑时才需要它们
    set_noisy_loggers_suppressed(root_logger.level > logging.DEBUG)
    
    # 记录初始化信息
    root_logger.info(f"Logging system initialized. Log directory: {_log_dir}")
    root_logger.info(f"Log level: {level}, Format: {log_format}, Trace ID: {_trace_id}")


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
    """设置进程级 Trace ID，对之后所有线程的日志生效"""
    global _trace_id
    with _trace_id_lock:
        _trace_id = trace_id or str(uuid.uuid4())[:8]


def get_log_dir() -> Path:
    """获取日志目录"""
    if _log_dir is None:
        init_logging()
    return _log_dir


def get_active_log_file() -> Path:
    """获取当前主日志文件路径"""
    if _active_log_file is None:
        init_logging()
    return _active_log_file
