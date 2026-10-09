#!/usr/bin/env python3
# -*- coding: utf-8 -*- python3 /media/data/LeTools/apps/test_upper_init/run_carton_behavior_tree_json.py
"""Carton-specific thin wrapper around the generic JSON behavior tree runner."""

import atexit
import logging
import os
import sys
import threading
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from core.common.logger import get_active_log_file, init_logging
from apps.test_upper_init import run_behavior_tree_json as generic_runner


CARTON_SCENARIO_DIR = (
    PROJECT_ROOT / "orchestration" / "scenarios" / "haicheng_box_internal"
)
logger = logging.getLogger(__name__)

_ORIGINAL_STDOUT = sys.stdout
_ORIGINAL_STDERR = sys.stderr
_STREAM_TEE = None


class _StreamTee:
    """将终端原始输出同时追加到当前主日志文件。"""

    def __init__(self, original_stream, copy_path: Path):
        self._original_stream = original_stream
        self._copy_file = open(copy_path, "a", encoding="utf-8", buffering=1)
        self._lock = threading.Lock()
        self.encoding = getattr(original_stream, "encoding", "utf-8")
        self.errors = getattr(original_stream, "errors", "replace")

    def write(self, data):
        if not data:
            return 0
        with self._lock:
            self._original_stream.write(data)
            self._copy_file.write(data)
        return len(data)

    def flush(self):
        with self._lock:
            self._original_stream.flush()
            self._copy_file.flush()

    def isatty(self):
        return bool(getattr(self._original_stream, "isatty", lambda: False)())

    def fileno(self):
        return self._original_stream.fileno()

    def close(self):
        with self._lock:
            try:
                self._copy_file.flush()
            finally:
                self._copy_file.close()


def _install_terminal_tee():
    global _STREAM_TEE
    if _STREAM_TEE is not None:
        return

    active_log_file = get_active_log_file()
    tee_stdout = _StreamTee(_ORIGINAL_STDOUT, active_log_file)
    tee_stderr = _StreamTee(_ORIGINAL_STDERR, active_log_file)
    sys.stdout = tee_stdout
    sys.stderr = tee_stderr
    _STREAM_TEE = (tee_stdout, tee_stderr)


def _remove_terminal_tee():
    global _STREAM_TEE
    if _STREAM_TEE is None:
        return

    sys.stdout = _ORIGINAL_STDOUT
    sys.stderr = _ORIGINAL_STDERR
    for tee in _STREAM_TEE:
        tee.close()
    _STREAM_TEE = None


def _is_writable_directory(path: Path) -> bool:
    try:
        path.mkdir(parents=True, exist_ok=True)
        probe = path / ".write_probe"
        with open(probe, "w", encoding="utf-8") as f:
            f.write("ok")
        probe.unlink()
        return True
    except Exception:
        return False


def _get_option_value(argv, option_name, default=None):
    try:
        idx = argv.index(option_name)
    except ValueError:
        return default
    if idx + 1 >= len(argv):
        return default
    return argv[idx + 1]


def _prepare_ros_log_dir(ros_node: str):
    env_log_dir = os.environ.get("ROS_LOG_DIR", "").strip()
    candidates = []
    if env_log_dir:
        candidates.append(Path(env_log_dir).expanduser())
    candidates.append(Path.home() / ".ros" / "log")
    candidates.append(Path("/tmp") / "LeTools_ros_logs" / ros_node)
    candidates.append(PROJECT_ROOT / ".runtime" / "ros_logs" / ros_node)

    chosen = None
    for candidate in candidates:
        if _is_writable_directory(candidate):
            chosen = candidate
            break

    if chosen is None:
        raise RuntimeError(
            "未找到可写的 ROS 日志目录，请检查 ROS_LOG_DIR、~/.ros/log 或 /tmp 权限"
        )

    resolved = str(chosen.resolve())
    original = env_log_dir if env_log_dir else "~/.ros/log"
    os.environ["ROS_LOG_DIR"] = resolved
    logger.info("[carton] ROS_LOG_DIR: %s -> %s", original, resolved)


atexit.register(_remove_terminal_tee)


def _has_option(argv, *options):
    return any(arg in options for arg in argv)


def main():
    argv = sys.argv[1:]
    forwarded = list(argv)

    if not _has_option(forwarded, "--scenario", "-s"):
        forwarded = ["--scenario", str(CARTON_SCENARIO_DIR)] + forwarded

    init_logging()
    _install_terminal_tee()

    if "--dry-run" not in forwarded:
        ros_node = _get_option_value(forwarded, "--ros-node", "behavior_tree_main")
        _prepare_ros_log_dir(ros_node)

    sys.argv = [sys.argv[0]] + forwarded
    generic_runner.main()


if __name__ == "__main__":
    main()
