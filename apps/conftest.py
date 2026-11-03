# -*- coding: utf-8 -*-
"""apps 级 pytest 配置。

py_trees 的 ``Blackboard`` 是进程级全局单例，同一 pytest 进程内多个测试文件
共享它。前一个文件留下的键（``servo_exec_idle`` / ``servo_phase`` 等）会让
后一个文件的用例失败。autouse fixture 在每个用例开始前清空，保证与文件
顺序无关；各测试文件 ``main()`` 入口复用同一个重置函数。
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from _blackboard_isolation import reset_blackboard  # noqa: E402


@pytest.fixture(autouse=True)
def _isolate_py_trees_blackboard():
    """每个用例开始前重置全局黑板。"""
    reset_blackboard()
