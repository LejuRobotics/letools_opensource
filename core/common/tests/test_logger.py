# -*- coding: utf-8 -*-
"""core/common/logger 统一日志系统单元测试。

覆盖四处已修复的缺陷：
1. Trace ID 曾恒为 N/A —— Formatter 用 f-string 提前求值，set_trace_id() 失效
2. 日志文件名沿用 kuavo_studio_* —— 现统一为 LeTools_*
3. rospy 挂在 'rosout' logger 上的终端 handler 导致同一条日志打印两次
4. Trace ID 曾用 ContextVar 存储 —— threading.Thread 不复制 context，
   子线程（SDK 管理器、异步装饰器等）读到默认值 N/A，现改为进程级全局变量
5. 控制台各等级曾统一无色 —— ERROR 与 INFO 视觉上无法区分，现按等级着色
6. 上游 SDK 自挂终端 handler 又未关 propagate —— 同一条日志被打印两次
"""

import io
import logging
import logging.handlers
import sys
import threading

import pytest

from core.common import logger as logger_module


@pytest.fixture
def configured_logging(tmp_path, monkeypatch):
    """在临时目录重新初始化日志系统，用例结束后复原 root 上的 handler。"""
    root_logger = logging.getLogger()
    saved_handlers = list(root_logger.handlers)
    root_logger.handlers.clear()

    monkeypatch.setattr(logger_module, "_initialized", False)
    monkeypatch.setattr(logger_module, "_active_log_file", None)
    monkeypatch.setattr(logger_module, "_active_error_log_file", None)
    monkeypatch.setattr(logger_module, "_trace_id", "N/A")

    logger_module.init_logging(
        log_dir=str(tmp_path),
        level="DEBUG",
        log_format="detailed",
        console_output=False,
        file_output=True,
    )

    yield tmp_path

    root_logger.handlers.clear()
    root_logger.handlers.extend(saved_handlers)


def _render_through_handlers(record: logging.LogRecord) -> list:
    """让 record 依次经过本项目挂载的 handler，返回实际写出的文本。

    只取带 TraceIdFilter 的 handler——pytest 在 log_cli 开启时也会往 root 上挂
    捕获用的 handler，其 formatter 不含本项目的字段，不应参与断言。
    """
    rendered = []
    for handler in logging.getLogger().handlers:
        if not any(isinstance(f, logger_module.TraceIdFilter) for f in handler.filters):
            continue
        if handler.level > record.levelno:
            continue
        for log_filter in handler.filters:
            log_filter.filter(record)
        rendered.append(handler.formatter.format(record))
    return rendered


def _make_record(level=logging.INFO, msg="hello") -> logging.LogRecord:
    return logging.LogRecord(
        name="test", level=level, pathname=__file__, lineno=1,
        msg=msg, args=(), exc_info=None,
    )


def test_trace_id_follows_set_trace_id(configured_logging):
    """Formatter 构造之后再设置 Trace ID，输出必须反映新值（回归：曾恒为 N/A）。"""
    logger_module.set_trace_id("deadbeef")

    rendered = _render_through_handlers(_make_record())

    assert rendered, "至少应有一个 handler 输出"
    assert all("[TRACE:deadbeef]" in text for text in rendered), rendered


def test_trace_id_is_generated_on_init(configured_logging):
    """未显式设置时，初始化阶段应自动产生一个非占位的 Trace ID。"""
    rendered = _render_through_handlers(_make_record())

    assert rendered
    assert all("[TRACE:N/A]" not in text for text in rendered)


def test_trace_id_visible_in_child_thread(configured_logging):
    """子线程必须沿用主线程设置的 Trace ID（回归：ContextVar 不跨 threading.Thread）。"""
    logger_module.set_trace_id("cafebabe")
    collected = []

    def worker():
        collected.extend(_render_through_handlers(_make_record(msg="from-thread")))

    thread = threading.Thread(target=worker)
    thread.start()
    thread.join()

    assert collected, "子线程应有日志输出"
    assert all("[TRACE:cafebabe]" in text for text in collected), collected


def test_set_trace_id_in_child_thread_is_visible_in_main(configured_logging):
    """子线程里设置的 Trace ID 主线程也能看到——Trace ID 是进程级的。"""
    logger_module.set_trace_id("main0001")

    def worker():
        logger_module.set_trace_id("worker01")

    thread = threading.Thread(target=worker)
    thread.start()
    thread.join()

    rendered = _render_through_handlers(_make_record())
    assert all("[TRACE:worker01]" in text for text in rendered), rendered


def test_log_files_are_named_letools(configured_logging):
    names = sorted(p.name for p in configured_logging.iterdir())

    assert any(name.startswith("LeTools_") for name in names), names
    assert any(name.startswith("LeTools_error_") for name in names), names
    assert not any("kuavo_studio" in name for name in names), names


def test_rosout_console_handler_removed(configured_logging):
    """RosStreamHandler 与 root handler 会造成日志重复，必须被移除。"""

    class RosStreamHandler(logging.Handler):
        def emit(self, record):
            pass

    rosout_logger = logging.getLogger("rosout")
    saved_handlers = list(rosout_logger.handlers)
    rosout_logger.addHandler(RosStreamHandler())
    try:
        logger_module.init_logging(log_dir=str(configured_logging), force=True)

        remaining = [type(h).__name__ for h in rosout_logger.handlers]
        assert "RosStreamHandler" not in remaining, remaining
    finally:
        rosout_logger.handlers.clear()
        rosout_logger.handlers.extend(saved_handlers)


def test_force_reconfigure_reuses_same_log_file(configured_logging):
    """force=True 重配时复用已打开的日志文件，不因时间戳变化另建新文件。"""
    before = sorted(p.name for p in configured_logging.iterdir())

    logger_module.init_logging(log_dir=str(configured_logging), force=True)

    after = sorted(p.name for p in configured_logging.iterdir())
    assert before == after


def test_init_logging_is_idempotent_without_force(configured_logging):
    """重复调用（不带 force）不应重复挂载 handler。"""
    root_logger = logging.getLogger()
    count_before = len(root_logger.handlers)

    logger_module.init_logging(log_dir=str(configured_logging))

    assert len(root_logger.handlers) == count_before


@pytest.fixture
def console_logging(tmp_path, monkeypatch):
    """重配日志系统并把控制台流换成可控的非终端流。

    sys.stdout 被替换为 StringIO（isatty() 为 False），使 colorlog
    「非终端自动降级无色」这一分支可复现，不受 pytest 捕获方式影响。
    """
    root_logger = logging.getLogger()
    saved_handlers = list(root_logger.handlers)
    root_logger.handlers.clear()

    monkeypatch.setattr(logger_module, "_initialized", False)
    monkeypatch.setattr(logger_module, "_active_log_file", None)
    monkeypatch.setattr(logger_module, "_active_error_log_file", None)
    monkeypatch.setattr(logger_module, "_trace_id", "N/A")
    monkeypatch.setattr(sys, "stdout", io.StringIO())
    monkeypatch.delenv("FORCE_COLOR", raising=False)
    monkeypatch.delenv("NO_COLOR", raising=False)

    logger_module.init_logging(
        log_dir=str(tmp_path),
        level="DEBUG",
        log_format="detailed",
        console_output=True,
        file_output=False,
    )

    yield sys.stdout

    root_logger.handlers.clear()
    root_logger.handlers.extend(saved_handlers)


def test_sdk_console_handler_removed_but_file_handler_kept(configured_logging):
    """SDK 自挂的终端 handler 会造成日志重复，必须移除；写文件的不受影响。

    上游 kuavo_humanoid_sdk/common/logger.py 给 'kuavo-humanoid-sdk' 同时挂了
    RotatingFileHandler 与 StreamHandler，且没有关掉 propagate。终端 handler 与
    root 上的 console handler 会各打印一遍同一条日志；文件 handler 写的是自己
    独立的文件，不产生重复，是排查 SDK 内部问题的依据，必须保留。
    """
    sdk_logger = logging.getLogger("kuavo-humanoid-sdk")
    saved_handlers = list(sdk_logger.handlers)
    sdk_logger.handlers.clear()

    console_handler = logging.StreamHandler()
    file_handler = logging.handlers.RotatingFileHandler(
        configured_logging / "sdk.log", maxBytes=1024, backupCount=1, encoding="utf-8"
    )
    sdk_logger.addHandler(console_handler)
    sdk_logger.addHandler(file_handler)
    try:
        logger_module.init_logging(log_dir=str(configured_logging), force=True)

        remaining = list(sdk_logger.handlers)
        assert console_handler not in remaining, "SDK 的终端 handler 应被移除"
        assert file_handler in remaining, "SDK 的文件 handler 应保留"
    finally:
        sdk_logger.handlers.clear()
        sdk_logger.handlers.extend(saved_handlers)
        file_handler.close()


def test_late_created_sdk_logger_console_handler_removed(configured_logging):
    """日志系统初始化之后才建好的外部 logger，仍须能被单独收敛。

    真机路径即如此：init_logging() 的两次 force=True 调用都早于 SDK 的运行时
    import（lifecycle_mixin 先重配日志，之后才 import SDK 管理器模块），因此
    adapters 侧在 import 完 SDK 之后还要再调用一次本函数。
    """
    sdk_logger = logging.getLogger("kuavo-humanoid-sdk")

    # 模拟「init_logging 已跑完，此后 SDK 才建立自己的 logger 与 handler」
    logger_module.init_logging(log_dir=str(configured_logging), force=True)
    late_handler = logging.StreamHandler()
    sdk_logger.addHandler(late_handler)
    try:
        logger_module.suppress_duplicate_console_output()

        assert late_handler not in list(sdk_logger.handlers)
    finally:
        sdk_logger.removeHandler(late_handler)


def _init_console_capture(tmp_path, monkeypatch, level):
    """把日志系统重配成只输出到控制台，返回 (缓冲区, 原 root handler 列表)。

    必须在测试体内调用：pytest 的 fd 捕获会在测试体执行时替换 sys.stdout，
    放在 fixture 里绑定会绑到错的流上，断言就读不到 console handler 的实际输出。
    """
    root_logger = logging.getLogger()
    saved_handlers = list(root_logger.handlers)
    root_logger.handlers.clear()

    console_buffer = io.StringIO()
    monkeypatch.setattr(logger_module, "_initialized", False)
    monkeypatch.setattr(logger_module, "_active_log_file", None)
    monkeypatch.setattr(logger_module, "_active_error_log_file", None)
    monkeypatch.setattr(logger_module, "_trace_id", "N/A")
    monkeypatch.setattr(sys, "stdout", console_buffer)

    logger_module.init_logging(
        log_dir=str(tmp_path),
        level=level,
        log_format="detailed",
        console_output=True,
        file_output=False,
    )
    return console_buffer, saved_handlers


def _restore_root_handlers(saved_handlers):
    root_logger = logging.getLogger()
    root_logger.handlers.clear()
    root_logger.handlers.extend(saved_handlers)


def test_noisy_loggers_suppressed_at_info_level(tmp_path, monkeypatch):
    """默认级别下，rospy.internal 与 transitions.core 的流水账不再输出。

    回归：单场启动有近 30 行 "adding connection to ... count N"（每次 topic
    连接/断开一行）与状态机回调流水账（每次状态迁移 3 行），淹没了真正有价值
    的启动日志。
    """
    console_buffer, saved_handlers = _init_console_capture(tmp_path, monkeypatch, "INFO")
    try:
        logging.getLogger("rospy.internal").info("adding connection to [/foo], count 0")
        logging.getLogger("transitions.core").info("Finished processing state stance exit callbacks.")

        output = console_buffer.getvalue()
        assert "adding connection" not in output, output
        assert "Finished processing" not in output, output
    finally:
        _restore_root_handlers(saved_handlers)


def test_noisy_loggers_keep_warnings(tmp_path, monkeypatch):
    """压制只针对 INFO —— 这两个 logger 的 WARNING 及以上必须保留。"""
    console_buffer, saved_handlers = _init_console_capture(tmp_path, monkeypatch, "INFO")
    try:
        logging.getLogger("rospy.internal").warning("ROS 节点异常退出")

        assert "ROS 节点异常退出" in console_buffer.getvalue()
    finally:
        _restore_root_handlers(saved_handlers)


def test_noisy_loggers_restored_at_debug_level(tmp_path, monkeypatch):
    """调成 DEBUG 排查 ROS 拓扑问题时，流水账应重新可见。

    压制必须可逆：上一个用例可能已经把这两个 logger 设为 WARNING，本次重配
    成 DEBUG 时若不显式恢复，它们会一直沉默下去。
    """
    console_buffer, saved_handlers = _init_console_capture(tmp_path, monkeypatch, "DEBUG")
    try:
        logging.getLogger("rospy.internal").info("adding connection to [/foo], count 0")

        assert "adding connection" in console_buffer.getvalue()
    finally:
        _restore_root_handlers(saved_handlers)


def test_sdk_log_written_to_console_once(tmp_path, monkeypatch):
    """SDK 日志在控制台只出现一次。

    摘掉 SDK 自己的终端 handler 之后，记录仍应经 propagate 到达 root，被 LeTools
    的 console handler 按统一格式（带 TRACE ID 与等级颜色）打印——不是把 SDK 日志
    整个丢掉。

    这里不复用 console_logging fixture：它替换 sys.stdout 的时机早于测试体，而
    pytest 的 fd 捕获会在测试体执行时再把 sys.stdout 换掉，导致断言读到的不是
    console handler 实际绑定的那个流。
    """
    sdk_logger = logging.getLogger("kuavo-humanoid-sdk")
    saved_sdk_handlers = list(sdk_logger.handlers)
    saved_sdk_level = sdk_logger.level
    sdk_logger.handlers.clear()
    sdk_logger.setLevel(logging.DEBUG)

    sdk_terminal = io.StringIO()
    sdk_logger.addHandler(logging.StreamHandler(sdk_terminal))

    console_buffer, saved_handlers = _init_console_capture(tmp_path, monkeypatch, "DEBUG")
    try:
        sdk_logger.info("sdk-hello")

        assert sdk_terminal.getvalue() == "", "SDK 自己的终端 handler 应已被移除"
        assert console_buffer.getvalue().count("sdk-hello") == 1, console_buffer.getvalue()
    finally:
        _restore_root_handlers(saved_handlers)
        sdk_logger.handlers.clear()
        sdk_logger.handlers.extend(saved_sdk_handlers)
        sdk_logger.setLevel(saved_sdk_level)


def test_sdk_info_kept_out_of_console(tmp_path, monkeypatch):
    """SDK 的 INFO 不再进终端。

    回归：上游 core.py 的 _is_mpc_mode() 在控制指令路径上打 "[Core] Current
    controller: ..."，而它被 11 个控制方法调用，其中 control_robot_end_effector_pose()
    位于 50Hz 的轨迹发布循环里——单条 3 秒轨迹能刷出上百行。
    """
    sdk_logger = logging.getLogger("kuavo-humanoid-sdk")
    saved_level = sdk_logger.level
    sdk_logger.setLevel(logging.DEBUG)

    console_buffer, saved_handlers = _init_console_capture(tmp_path, monkeypatch, "INFO")
    try:
        sdk_logger.info("[Core] Current controller: mpc")

        assert "Current controller" not in console_buffer.getvalue(), console_buffer.getvalue()
    finally:
        _restore_root_handlers(saved_handlers)
        sdk_logger.setLevel(saved_level)


def test_sdk_warning_still_reaches_console(tmp_path, monkeypatch):
    """压制只针对 INFO —— SDK 的 WARNING 及以上必须照常出现在终端。

    不能一刀切：MPC 服务不可用的告警正是 WARNING，它是排查静默降级的唯一线索。
    """
    sdk_logger = logging.getLogger("kuavo-humanoid-sdk")
    saved_level = sdk_logger.level
    sdk_logger.setLevel(logging.DEBUG)

    console_buffer, saved_handlers = _init_console_capture(tmp_path, monkeypatch, "INFO")
    try:
        sdk_logger.warning("[Core] Manipulation MPC control mode service not available")

        assert "service not available" in console_buffer.getvalue(), console_buffer.getvalue()
    finally:
        _restore_root_handlers(saved_handlers)
        sdk_logger.setLevel(saved_level)


def test_sdk_info_still_written_to_sdk_own_file(tmp_path, monkeypatch):
    """SDK 自己的文件日志必须拿到全量记录（含 INFO）。

    这是本压制方式的立身之本：它只在 LeTools 的 console handler 上过滤，不动 logger
    级别。若改用 setLevel(WARNING) 压制，记录会在任何 handler 之前就被 isEnabledFor()
    拦掉，连 SDK 自挂的 RotatingFileHandler 一起废掉——而那是排查 SDK 内部问题的依据。
    """
    sdk_logger = logging.getLogger("kuavo-humanoid-sdk")
    saved_handlers = list(sdk_logger.handlers)
    saved_level = sdk_logger.level
    sdk_logger.handlers.clear()
    sdk_logger.setLevel(logging.DEBUG)

    sdk_file = tmp_path / "sdk.log"
    sdk_file_handler = logging.FileHandler(sdk_file, encoding="utf-8")
    sdk_logger.addHandler(sdk_file_handler)

    _, saved_root_handlers = _init_console_capture(tmp_path, monkeypatch, "INFO")
    try:
        sdk_logger.info("[Core] Current controller: mpc")
        sdk_file_handler.flush()

        assert "Current controller" in sdk_file.read_text(encoding="utf-8")
    finally:
        _restore_root_handlers(saved_root_handlers)
        sdk_logger.handlers.clear()
        sdk_logger.handlers.extend(saved_handlers)
        sdk_logger.setLevel(saved_level)
        sdk_file_handler.close()


def test_sdk_info_restored_at_debug_level(tmp_path, monkeypatch):
    """调成 DEBUG 排查问题时，SDK 的 INFO 应重新可见。

    与 _NOISY_LOGGERS 同一约定：压制一旦生效就再也调不出来，排障时反而少了线索。
    """
    sdk_logger = logging.getLogger("kuavo-humanoid-sdk")
    saved_level = sdk_logger.level
    sdk_logger.setLevel(logging.DEBUG)

    console_buffer, saved_handlers = _init_console_capture(tmp_path, monkeypatch, "DEBUG")
    try:
        sdk_logger.info("[Core] Current controller: mpc")

        assert "Current controller" in console_buffer.getvalue(), console_buffer.getvalue()
    finally:
        _restore_root_handlers(saved_handlers)
        sdk_logger.setLevel(saved_level)


def test_unrelated_logger_info_unaffected(tmp_path, monkeypatch):
    """过滤只认名单内的 logger，其它 logger 的 INFO 照常输出。"""
    console_buffer, saved_handlers = _init_console_capture(tmp_path, monkeypatch, "INFO")
    try:
        logging.getLogger("core.common.logger").info("hello-from-letools")

        assert "hello-from-letools" in console_buffer.getvalue(), console_buffer.getvalue()
    finally:
        _restore_root_handlers(saved_handlers)


def test_console_colors_error_level(console_logging, monkeypatch):
    """ERROR 在控制台带红色 ANSI 码，与 INFO 视觉上可区分。"""
    monkeypatch.setenv("FORCE_COLOR", "1")

    rendered = _render_through_handlers(_make_record(level=logging.ERROR))

    assert rendered, "控制台 handler 应输出"
    assert all("\x1b[31m" in text for text in rendered), rendered


def test_console_colors_info_level(console_logging, monkeypatch):
    """INFO 在控制台带绿色 ANSI 码。"""
    monkeypatch.setenv("FORCE_COLOR", "1")

    rendered = _render_through_handlers(_make_record(level=logging.INFO))

    assert rendered, "控制台 handler 应输出"
    assert all("\x1b[32m" in text for text in rendered), rendered


def test_console_colors_disabled_when_not_a_tty(console_logging):
    """控制台不是终端时不着色——重定向到文件后不应残留 ANSI 码。"""
    rendered = _render_through_handlers(_make_record(level=logging.ERROR))

    assert rendered, "控制台 handler 应输出"
    assert all("\x1b[" not in text for text in rendered), rendered


def test_file_handlers_stay_plain_text(configured_logging):
    """文件日志保持纯文本，ANSI 码会污染 grep 与日志分析。"""
    rendered = _render_through_handlers(_make_record(level=logging.ERROR))

    assert rendered, "文件 handler 应输出"
    assert all("\x1b[" not in text for text in rendered), rendered
