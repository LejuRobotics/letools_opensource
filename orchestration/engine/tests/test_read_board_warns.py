# -*- coding: utf-8 -*-
"""`READ_BOARD` 缺键时必须**说出来**，不能静默回退到代码默认值。

静默回退是这个仓库反复踩的坑：板子上少了一行，节点照样跑，只是用的是代码里
那个值 —— 而现场以为自己在用板子上的值。
"""
import contextlib
import logging

import py_trees
import pytest
from py_trees.common import Access

from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory

pytestmark = pytest.mark.unit


@contextlib.contextmanager
def _capture_logs():
    records = []
    handler = logging.Handler()
    handler.emit = records.append
    root = logging.getLogger()
    root.addHandler(handler)
    old_level = root.level
    root.setLevel(logging.DEBUG)
    try:
        yield records
    finally:
        root.removeHandler(handler)
        root.setLevel(old_level)


@pytest.fixture(autouse=True)
def _isolated_blackboard():
    """py_trees 的 `Blackboard.clients`/`storage` 是**类属性**，跨测试残留：
    同名 Client 会抛「already been registered」，上一条用例的键也会被读到。"""
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _factory():
    client = py_trees.blackboard.Client(name="test_board", namespace="/")
    return BehaviorTreeFactory(client), client


def test_a_present_key_is_read_from_the_board():
    """黑板上有值就取黑板的 —— 这条**以前没有测试**。"""
    factory, client = _factory()
    client.register_key(key="max_dt_s", access=Access.WRITE)
    client.set("max_dt_s", 0.08)

    params = factory._parse_params(
        {"max_dt_s": {"source": "READ_BOARD", "board_key": "max_dt_s"}},
        namespace="")

    assert params.get("max_dt_s") == 0.08


def test_a_missing_key_warns_and_is_absent():
    """缺键：params 里**没有这个键**（节点用代码默认值），外加一条 WARNING
    说清键名 + 回退到哪个默认值。"""
    factory, client = _factory()
    client.register_key(key="present", access=Access.WRITE)
    client.set("present", 1)

    with _capture_logs() as records:
        params = factory._parse_params(
            {"max_dt_s": {"source": "READ_BOARD", "board_key": "max_dt_s"}},
            namespace="")

    assert "max_dt_s" not in params, "缺键不该进 params —— 节点要能用自己的默认值"
    warnings = [r for r in records if r.levelno >= logging.WARNING]
    assert len(warnings) == 1, [r.getMessage() for r in records]
    message = warnings[0].getMessage()
    assert "max_dt_s" in message
    assert "READ_BOARD" in message
