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
import os
import sys
import threading
import time

import pytest

from core.common import logger as logger_module


@pytest.fixture(autouse=True)
def _no_letools_log_env(monkeypatch):
    """把 `LETOOLS_LOG_*` 环境变量从这个测试模块里清掉。

    这几个变量是**故意**影响 `init_logging` 的输入（一键启动脚本靠它们把三个进程
    指到同一个文件、打开按天清理）。于是本机一旦 export 过，用例里那些"默认值"
    的预期就不成立了 —— **不是代码坏了，是测试跟着环境漂了**。
    清掉之后每个用例只依赖它自己显式给的东西，跑在哪台机器上都一样。
    """
    for key in ('LETOOLS_LOG_DIR', 'LETOOLS_LOG_FILE', 'LETOOLS_LOG_TAG',
                'LETOOLS_LOG_MAX_BYTES', 'LETOOLS_LOG_PURGE_DAYS',
                'LETOOLS_LOG_ROLLOVER', 'LETOOLS_LOG_CONSOLE'):
        monkeypatch.delenv(key, raising=False)


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


# --------------------------------------------------------------------------- #
# 满了"另开一个新文件" + 按天清理 + 环境变量（2026-10-15，托盘/纸箱检测日志用）
#
# 需求原文："单个 log 最大 50m，旧 log 3 天自动清理"，且满了要**另开一个新文件**
# —— 不是 RotatingFileHandler 那种改名成 .1/.2 再删最旧的语义。
# --------------------------------------------------------------------------- #
def test_rolled_name_does_not_collide_with_the_timestamp(tmp_path):
    """★ 下一个文件名**不能**把时间戳的最后一段当成序号。

    基础名就以时间戳结尾（`..._143012`），拿 `_\\d+$` 去认序号会得到
    `..._143013.log` —— 看起来像个新时间戳，实际是撞车。所以分隔符是 `_p`。
    """
    nxt = logger_module._next_rolled_path(tmp_path / 'LeTools_20261015_143012.log')
    assert nxt.name == 'LeTools_20261015_143012_p02.log', nxt.name

    nxt2 = logger_module._next_rolled_path(nxt)
    assert nxt2.name == 'LeTools_20261015_143012_p03.log', nxt2.name

    # 带角色 tag 的名字同样成立
    tagged = logger_module._next_rolled_path(
        tmp_path / 'LeTools_pallet_20261015_1430.log')
    assert tagged.name == 'LeTools_pallet_20261015_1430_p02.log', tagged.name


def test_size_capped_handler_opens_new_files_and_keeps_the_old_ones(tmp_path):
    """★ 写满就**另开新文件**：旧的原样留着、**不改名**、**没有 .1/.2**。

    这一条与 `RotatingFileHandler` 的差别正是需求要的：后者会把当前文件改名成
    `.1` 并在 `backupCount` 到点时**删掉最旧的**。
    """
    first = tmp_path / 'LeTools_x_20261015_120000.log'
    handler = logger_module.SizeCappedNewFileHandler(first, max_bytes=300)
    handler.setFormatter(logging.Formatter('%(message)s'))

    probe = logging.getLogger('size_capped_probe')
    probe.handlers = [handler]
    probe.propagate = False
    probe.setLevel(logging.DEBUG)
    try:
        # 每 100 条查一次大小；每条约 41 字节，100 条 ~4KB >> 300B -> 必然切换
        for _ in range(600):
            probe.info('y' * 40)
    finally:
        handler.close()

    files = sorted(p.name for p in tmp_path.glob('*.log'))
    assert len(files) >= 2, f'应至少开出第二个文件，实际 {files}'
    assert first.exists(), '第一个文件必须原样留着'
    assert first.stat().st_size >= 300, '第一个文件应已经写超过阈值（才谈得上"满了"）'
    assert handler.switches >= 1, handler.switches
    # 改名轮转的指纹：同名前缀下出现 .1/.2
    assert not list(tmp_path.glob('*.1')), '不该出现 RotatingFileHandler 的 .1'
    assert all('_p' in n for n in files[1:]), f'新文件应带 _pNN 序号：{files}'


def test_purge_old_logs_uses_mtime_and_skips_subdirs(tmp_path):
    """★ 按 **mtime** 清理（不解析文件名），且**不递归**进子目录。"""
    old = tmp_path / 'LeTools_old.log'
    fresh = tmp_path / 'LeTools_fresh.log'
    old.write_text('x')
    fresh.write_text('x')
    # 4 天前 / 1 天前
    os.utime(old, (time.time() - 4 * 86400, time.time() - 4 * 86400))
    os.utime(fresh, (time.time() - 1 * 86400, time.time() - 1 * 86400))

    # 结构性目录（帧 dump 的 JSON）绝不能被顺手删掉
    dump = tmp_path / 'pallet_servo_dump'
    dump.mkdir()
    stale_json = dump / '20261001-000000-000001.json'
    stale_json.write_text('{}')
    os.utime(stale_json, (time.time() - 30 * 86400,) * 2)

    removed = logger_module.purge_old_logs(tmp_path, days=3)

    assert removed == 1, removed
    assert not old.exists(), '4 天前的应被删'
    assert fresh.exists(), '1 天前的必须留着'
    assert stale_json.exists(), '子目录不是 *.log，不该被删'


def test_init_logging_reads_env_and_explicit_args_win(tmp_path, monkeypatch):
    """环境变量驱动目录/上限/清理/换文件；**显式参数优先于环境变量**。"""
    root_logger = logging.getLogger()
    saved = list(root_logger.handlers)
    root_logger.handlers.clear()
    monkeypatch.setattr(logger_module, '_initialized', False)
    monkeypatch.setattr(logger_module, '_active_log_file', None)
    monkeypatch.setattr(logger_module, '_active_error_log_file', None)
    monkeypatch.setattr(logger_module, '_trace_id', 'N/A')

    env_dir = tmp_path / 'from_env'
    monkeypatch.setenv('LETOOLS_LOG_DIR', str(env_dir))
    monkeypatch.setenv('LETOOLS_LOG_TAG', '_pallet')
    monkeypatch.setenv('LETOOLS_LOG_MAX_BYTES', '52428800')
    monkeypatch.setenv('LETOOLS_LOG_ROLLOVER', 'new_file')
    try:
        # 不传 log_dir -> 用环境变量
        logger_module.init_logging(level='INFO', console_output=False)
        got = logger_module.get_active_log_file()
        assert got.parent == env_dir, got
        assert got.name.startswith('LeTools_pallet_'), got.name
        assert env_dir.exists()
        # 用的是"满了另开一个"那款 handler，不是改名轮转
        file_handlers = [h for h in root_logger.handlers
                         if isinstance(h, logging.handlers.RotatingFileHandler)]
        assert not any(type(h) is logging.handlers.RotatingFileHandler
                       for h in file_handlers), 'new_file 模式不该用 RotatingFileHandler'

        # 显式参数优先于环境变量（单元测试靠这条隔离，不被外部 env 顶掉）
        root_logger.handlers.clear()
        monkeypatch.setattr(logger_module, '_initialized', False)
        monkeypatch.setattr(logger_module, '_active_log_file', None)
        monkeypatch.setattr(logger_module, '_active_error_log_file', None)
        explicit = tmp_path / 'explicit'
        logger_module.init_logging(log_dir=str(explicit), level='INFO',
                                   console_output=False)
        assert logger_module.get_active_log_file().parent == explicit
    finally:
        root_logger.handlers.clear()
        root_logger.handlers.extend(saved)


def test_shared_log_file_env_makes_every_process_write_one_file(tmp_path, monkeypatch):
    """`LETOOLS_LOG_FILE` = 一键启动脚本让三个进程**共用同一个文件**的机制。

    托盘检测 / 纸箱检测 / 行为树是三个独立进程；脚本分配一个带时间戳的路径、
    通过环境变量传下去，各自以 append 打开 —— 排查一次故障只翻一个文件。
    """
    root_logger = logging.getLogger()
    saved = list(root_logger.handlers)
    root_logger.handlers.clear()
    monkeypatch.setattr(logger_module, '_initialized', False)
    monkeypatch.setattr(logger_module, '_active_log_file', None)
    monkeypatch.setattr(logger_module, '_active_error_log_file', None)
    monkeypatch.setattr(logger_module, '_trace_id', 'N/A')

    shared = tmp_path / 'pallet_log' / 'pallet_20261015_143012.log'
    monkeypatch.setenv('LETOOLS_LOG_FILE', str(shared))
    try:
        logger_module.init_logging(level='INFO', console_output=False)
        got = logger_module.get_active_log_file()
        assert got == shared, f'{got} != {shared}'
        assert shared.parent.exists(), '目录应由 init_logging 建出来'
    finally:
        root_logger.handlers.clear()
        root_logger.handlers.extend(saved)


def test_console_output_can_be_turned_off(tmp_path, monkeypatch):
    """★ `console_output=False` 时 root 上**不留任何写终端的 handler**。

    这是一键启动脚本"三个进程共用一个日志文件"能成立的前提：脚本把检测器的
    stdout **也**重定向进那个文件，而 logger 的控制台 handler 同样写 stdout ——
    控制台不关的话，**每条日志会在同一个文件里出现两次**（一次详细格式、
    一次控制台格式），文件难读、50MB 额度也翻倍地用掉。
    """
    root_logger = logging.getLogger()
    saved = list(root_logger.handlers)
    root_logger.handlers.clear()
    monkeypatch.setattr(logger_module, '_initialized', False)
    monkeypatch.setattr(logger_module, '_active_log_file', None)
    monkeypatch.setattr(logger_module, '_active_error_log_file', None)
    monkeypatch.setattr(logger_module, '_trace_id', 'N/A')
    try:
        logger_module.init_logging(log_dir=str(tmp_path), level='INFO',
                                   console_output=False, file_output=True)
        to_terminal = [h for h in root_logger.handlers
                       if isinstance(h, logging.StreamHandler)
                       and not isinstance(h, logging.FileHandler)]
        assert to_terminal == [], to_terminal
        # 但文件那一路必须在（关掉的只是终端那一份）
        files = [h for h in root_logger.handlers
                 if isinstance(h, logging.FileHandler)]
        assert files, '关控制台不该把落盘也关掉'
    finally:
        root_logger.handlers.clear()
        root_logger.handlers.extend(saved)
