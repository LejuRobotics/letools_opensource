# -*- coding: utf-8 -*-
"""Async：异步执行装饰器（用于并行不阻塞）。

将任意子节点放入后台线程 tick，从而：
- 主线程 tick 时不会被子节点的阻塞型 execute/sleep 卡住
- 在 Parallel 复合节点下可“同时启动”多个动作（从编排角度并行）

注意：
- 这不保证底层硬件真的能同时执行两条控制链路（取决于控制器/SDK）。
- 黑板与 ROS/SDK 调用在多线程下需自行评估线程安全。
"""

import os
import threading
import time
from typing import Optional

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="异步执行(Async)",
    category=["utility", "concurrency"],
    tree_type="studio_smoke",
    description="将子节点放到后台线程 tick，使其不阻塞主线程（用于 Parallel 真并行启动）",
    params=[
        {"name": "tick_hz", "type": "float", "default": "50.0", "description": "后台 tick 频率（Hz）"},
    ],
    inputs=[],
    outputs=[],
)
class Async(Behaviour):
    def __init__(self, name: str, child: Behaviour, tick_hz: float = 50.0):
        super().__init__(name=name)
        self.child = child
        self.tick_hz = float(tick_hz) if tick_hz else 50.0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._done = False
        self._final_status: Optional[Status] = None
        self._error: Optional[BaseException] = None

    def initialise(self):
        self._stop.clear()
        self._done = False
        self._final_status = None
        self._error = None

        if _DRY_RUN:
            self._done = True
            self._final_status = Status.SUCCESS
            return

        # best-effort setup for child (tree may not traverse into it)
        try:
            self.child.setup(timeout=0)
        except Exception:
            pass

        def _run():
            period = 1.0 / max(self.tick_hz, 1.0)
            try:
                while not self._stop.is_set():
                    # Tick child exactly once
                    for _ in self.child.tick():
                        pass
                    st = getattr(self.child, "status", None)
                    if st in (Status.SUCCESS, Status.FAILURE):
                        self._final_status = st
                        self._done = True
                        return
                    time.sleep(period)
            except BaseException as exc:
                self._error = exc
                self._final_status = Status.FAILURE
                self._done = True

        self._thread = threading.Thread(target=_run, name=f"Async[{self.name}]", daemon=True)
        self._thread.start()

    def update(self):
        if self._done:
            if self._error is not None:
                self.feedback_message = str(self._error)
                return Status.FAILURE
            return self._final_status or Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status):
        # Stop background ticking
        self._stop.set()
        try:
            if self.child is not None:
                self.child.stop(Status.INVALID)
        except Exception:
            pass

