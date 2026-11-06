# -*- coding: utf-8 -*-
"""TF Buffer 并发惰性初始化测试（不依赖 ROS）。"""

import importlib.util
import sys
import threading
import time
from pathlib import Path
from types import ModuleType

_MODULE_PATH = (Path(__file__).resolve().parents[1] / "adapters" / "hardware" /
                "leju_wheeled" / "mixins" / "ik_service_mixin.py")
_SPEC = importlib.util.spec_from_file_location("ik_service_mixin_under_test", _MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
IkServiceMixin = _MODULE.IkServiceMixin


def test_concurrent_first_access_creates_only_one_buffer_and_listener(monkeypatch):
    buffers = []
    listeners = []
    fake_tf2_ros = ModuleType("tf2_ros")

    class FakeBuffer:
        def __init__(self):
            buffers.append(self)
            # 拉长创建窗口，确保无锁实现会让另一个线程也进入 Buffer 构造。
            time.sleep(0.05)

    class FakeListener:
        def __init__(self, buffer):
            self.buffer = buffer
            listeners.append(self)

    fake_tf2_ros.Buffer = FakeBuffer
    fake_tf2_ros.TransformListener = FakeListener
    monkeypatch.setitem(sys.modules, "tf2_ros", fake_tf2_ros)

    hardware = IkServiceMixin()
    start = threading.Barrier(3)
    results = []
    errors = []

    def get_buffer():
        try:
            start.wait()
            results.append(hardware._get_tf_buffer())
        except Exception as exc:
            errors.append(exc)

    threads = [threading.Thread(target=get_buffer) for _ in range(2)]
    for thread in threads:
        thread.start()
    start.wait()
    for thread in threads:
        thread.join(timeout=1.0)

    assert not errors
    assert all(not thread.is_alive() for thread in threads)
    assert len(buffers) == 1
    assert len(listeners) == 1
    assert results == [buffers[0], buffers[0]]
    assert hardware._tf_buffer is buffers[0]
    assert hardware._tf_listener is listeners[0]
    assert listeners[0].buffer is buffers[0]
