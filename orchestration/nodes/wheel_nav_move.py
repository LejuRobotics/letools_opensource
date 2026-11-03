# -*- coding: utf-8 -*-
"""WheelNavMove: 底盘移动节点。

通过 ``mode`` 参数选择坐标语义：
- ``world``（默认）：调用 hardware.send_world_position，保持旧案例原有行为。
- ``base``：调用 hardware.send_base_position，x/y 为底盘局部坐标系位移。

两种模式都在后台线程中执行，不阻塞行为树。``base`` 模式会检查接口结果；
``world`` 模式保留旧行为，不改变已有案例的成功/失败语义。
"""

import os
import math
import threading

from py_trees.common import Status

from core.common.logger import get_logger
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware

logger = get_logger(__name__)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class WheelNavMove(BaseAction):
    """后台线程执行底盘局部相对移动。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False
        self._thread = None
        self._result = None
        self._mode = "world"

    def initialise(self):
        self._done = False
        self._result = None
        self._thread = None
        self._mode = str(self.params.get("mode", "world")).strip().lower()

        if self._mode not in ("world", "base"):
            self._result = False
            self._done = True
            self.feedback_message = "mode 必须是 world 或 base"
            logger.error("[WheelNavMove] %s", self.feedback_message)
            return

        if _DRY_RUN:
            self.feedback_message = "dry-run wheel_nav_move"
            return

        try:
            x = float(self.params.get("x", 0.0))
            y = float(self.params.get("y", 0.0))
            yaw_deg = float(self.params.get("yaw_deg", 0.0))
            if self._mode == "base":
                if not all(math.isfinite(value) for value in (x, y, yaw_deg)):
                    raise ValueError("局部位移和转角必须为有限数值")
                if x == 0.0 and y == 0.0 and yaw_deg == 0.0:
                    raise ValueError("局部目标尚未配置：请填写 nav_to_place 的 x、y、yaw_deg")
        except (TypeError, ValueError) as e:
            self._result = False
            self._done = True
            self.feedback_message = str(e)
            logger.error("[WheelNavMove] %s", e)
            return
        yaw_rad = math.radians(yaw_deg)

        hw = get_shared_hardware()

        if self._mode == "base":
            logger.info("[WheelNavMove] 局部相对移动 dx=%.3fm, dy=%.3fm, dyaw=%.1f°", x, y, yaw_deg)
        else:
            logger.info("[WheelNavMove] 导航到 (%.3f, %.3f, yaw=%.1f°)", x, y, yaw_deg)
        self._thread = threading.Thread(
            target=self._nav_worker,
            args=(hw, x, y, yaw_rad, self._mode),
            daemon=True,
        )
        self._thread.start()

    def _nav_worker(self, hw, x, y, yaw_rad, mode):
        try:
            if mode == "base":
                result = hw.send_base_position(x, y, yaw_rad)
                self._result = bool(result.success)
                if not self._result:
                    logger.error("[WheelNavMove] 底盘相对移动失败: %s", result.message)
            else:
                # 保留旧 WheelNavMove 的行为：接口只负责发送，不改变原有结果语义。
                hw.send_world_position(x, y, yaw_rad)
                self._result = True
        except Exception as e:
            logger.error("[WheelNavMove] 底盘移动异常: %s", e, exc_info=True)
            self._result = False

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS

        if self._done:
            if self._mode == "world":
                return Status.SUCCESS
            return Status.SUCCESS if self._result else Status.FAILURE

        if self._thread is None:
            return Status.FAILURE

        if self._thread.is_alive():
            return Status.RUNNING

        self._done = True
        if self._mode == "world":
            logger.info("[WheelNavMove] 导航完成")
            return Status.SUCCESS
        if self._result:
            logger.info("[WheelNavMove] 底盘相对移动接口执行完成")
            return Status.SUCCESS
        else:
            return Status.FAILURE
