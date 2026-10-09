# -*- coding: utf-8 -*-
"""GraspMtbfCheckArrived：JiBot底盘任务到达检查薄节点 → check_arrived_jibot 原子技能。"""

import os
import math
import time

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from .scene_io import get_scene_io
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _as_bool(value):
    """兼容行为树编辑器产生的布尔字符串。"""
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
        raise ValueError(f"invalid boolean value: {value}")
    return bool(value)


@define_manifest(
    label="JiBot底盘任务到达检查",
    category=["motion", "chassis", "jibot"],
    tree_type="grasp_mtbf_v1",
    description="对齐 test_check_arrived.py：调用 hardware.check_arrived_jibot()",
    params=[
        {"name": "task_id", "type": "string", "default": "", "description": "由base_move或move_to_target返回的任务ID"},
        {"name": "task_id_key", "type": "string", "default": "current_task_id", "description": "从黑板读取task_id的键名(优先级高于task_id参数)"},
        {"name": "blocking", "type": "bool", "default": "True", "description": "是否阻塞等待任务完成"},
        {"name": "timeout", "type": "float", "default": "20.0", "description": "超时时间(s)，blocking=True时有效"},
        {"name": "poll_interval", "type": "float", "default": "0.2", "description": "非阻塞轮询间隔(s)"},
    ],
    inputs=[],
    outputs=[],
)
class GraspMtbfCheckArrived(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._hardware = None
        self._task_id = ""
        self._blocking = True
        self._timeout = 20.0
        self._poll_interval = 0.2
        self._started_at = 0.0
        self._next_poll_at = 0.0
        self._initialise_error = ""
        self._restore_attempted = False
        self._last_result = None
        self._dry_done = False

    def _restore_control(self, reason):
        """到达检查失败或被中断时停止导航并恢复下位机速度控制。"""
        if self._restore_attempted or self._hardware is None:
            return
        self._restore_attempted = True
        try:
            result = self._hardware.enable_vel_control_jibot(True)
            if result.success:
                self.feedback_message = f"{reason}；已恢复 /enable_vel_control=True"
            else:
                self.feedback_message = (
                    f"{reason}；恢复 /enable_vel_control 失败: "
                    f"{result.message or 'unknown error'}"
                )
        except Exception as exc:
            self.feedback_message = f"{reason}；恢复 /enable_vel_control 异常: {exc}"

    def initialise(self):
        self._dry_done = False
        self._hardware = None
        self._task_id = ""
        self._started_at = 0.0
        self._next_poll_at = 0.0
        self._initialise_error = ""
        self._restore_attempted = False
        self._last_result = None
        if _DRY_RUN:
            return

        self._hardware = get_scene_io()
        try:
            task_id = str(self.params.get("task_id", ""))
            task_id_key = str(self.params.get("task_id_key", "current_task_id"))

            if task_id_key:
                try:
                    self.global_blackboard.register_key(
                        key=task_id_key, access=py_trees.common.Access.READ
                    )
                except AttributeError:
                    pass
                try:
                    task_id = self.global_blackboard.get(task_id_key)
                except (KeyError, AttributeError):
                    task_id = ""

            if not task_id:
                task_id = str(self.params.get("task_id", ""))
            if not task_id:
                raise ValueError("缺少导航 task_id")

            self._task_id = str(task_id)
            self._blocking = _as_bool(self.params.get("blocking", True))
            self._timeout = float(self.params.get("timeout", 20.0))
            self._poll_interval = float(self.params.get("poll_interval", 0.2))
            if not math.isfinite(self._timeout) or self._timeout <= 0.0:
                raise ValueError("timeout 必须大于 0")
            if not math.isfinite(self._poll_interval) or self._poll_interval <= 0.0:
                raise ValueError("poll_interval 必须大于 0")
            self._started_at = time.monotonic()
            self._next_poll_at = self._started_at
        except (TypeError, ValueError) as exc:
            self._initialise_error = str(exc)
            self.feedback_message = f"到达检查参数错误: {exc}"

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run check_arrived_jibot"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._initialise_error:
            self._restore_control(self.feedback_message)
            return Status.FAILURE
        if self._hardware is None or not self._task_id:
            self._restore_control("到达检查未初始化")
            return Status.FAILURE

        now = time.monotonic()
        if self._blocking and now - self._started_at > self._timeout:
            reason = (
                f"等待导航到达超时: task_id={self._task_id}, "
                f"timeout={self._timeout:.1f}s"
            )
            self.feedback_message = reason
            self._restore_control(reason)
            return Status.FAILURE
        if now < self._next_poll_at:
            return Status.RUNNING
        self._next_poll_at = now + self._poll_interval

        # 行为树主循环永不调用服务端的 blocking 模式，避免远端卡住整棵树。
        result = self._hardware.check_arrived_jibot(
            task_id=self._task_id,
            blocking=False,
            timeout=0.0,
        )
        self._last_result = result
        if not result.success:
            reason = result.message or "check_arrived_jibot failed"
            self.feedback_message = reason
            self._restore_control(reason)
            return Status.FAILURE

        data = result.data if isinstance(result.data, dict) else {}
        if bool(data.get("arrived", False)):
            self.feedback_message = data.get("message") or "arrived"
            return Status.SUCCESS

        message = str(data.get("message") or result.message or "not arrived")
        terminal_errors = ("interrupt", "error", "fail", "abort", "cancel")
        if any(marker in message.lower() for marker in terminal_errors):
            reason = f"导航任务异常结束: {message}"
            self.feedback_message = reason
            self._restore_control(reason)
            return Status.FAILURE

        if not self._blocking:
            reason = f"JiBot did not arrive: {message}"
            self.feedback_message = reason
            self._restore_control(reason)
            return Status.FAILURE

        self.feedback_message = f"等待导航到达: task_id={self._task_id}, {message}"
        return Status.RUNNING

    def terminate(self, new_status):
        """行为树暂停、停止或失败时恢复控制，防止导航脱离行为树继续运行。"""
        if new_status != Status.SUCCESS:
            self._restore_control(self.feedback_message or "到达检查被中断")
