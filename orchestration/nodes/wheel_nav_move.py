# -*- coding: utf-8 -*-
"""WheelNavMove: 仿真底盘导航节点。

直接调用 hardware.send_world_position(x, y, yaw) 控制底盘移动，
不依赖 /move_base/move_to_target 服务（MuJoCo 仿真中没有该服务）。
在后台线程中执行，支持与躯干调整并行。
"""

import os
import math
import threading

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware

logger = get_logger(__name__)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class WheelNavMove(BaseAction):
    """仿真底盘导航：后台线程调用 send_world_position，不阻塞行为树。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False
        self._thread = None
        self._result = None

    def initialise(self):
        self._done = False
        self._result = None

        if _DRY_RUN:
            self.feedback_message = "dry-run wheel_nav_move"
            return

        x = float(self.params.get("x", 0.0))
        y = float(self.params.get("y", 0.0))
        yaw_deg = float(self.params.get("yaw_deg", 0.0))
        yaw_rad = math.radians(yaw_deg)

        hw = get_shared_hardware()

        logger.info("[WheelNavMove] 导航到 (%.3f, %.3f, yaw=%.1f°)", x, y, yaw_deg)
        self._thread = threading.Thread(
            target=self._nav_worker,
            args=(hw, x, y, yaw_rad),
            daemon=True,
        )
        self._thread.start()

    def _nav_worker(self, hw, x, y, yaw_rad):
        try:
            hw.send_world_position(x, y, yaw_rad)
            self._result = True
        except Exception as e:
            logger.error("[WheelNavMove] 导航异常: %s", e, exc_info=True)
            self._result = False

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS

        if self._done:
            return Status.SUCCESS

        if self._thread is None:
            return Status.FAILURE

        if self._thread.is_alive():
            return Status.RUNNING

        self._done = True
        if self._result:
            logger.info("[WheelNavMove] 导航完成")
            return Status.SUCCESS
        else:
            return Status.FAILURE
