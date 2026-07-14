# -*- coding: utf-8 -*-
"""NodeHeadSequence:头部按 yaws/pitchs 序列扫。
注:`pitchs` 拼写保留与源版 case_wheel_pick_and_place.py 一致。
"""
import os
import time
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware


def _is_dry_run() -> bool:
    """每次 update 调用都重新读环境变量,便于测试动态切换。"""
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class NodeHeadSequence(BaseAction):
    """按 yaws/pitchs 网格循环扫头,扫到目标/超时返回。

    每步驻留 dwell_sec 秒(默认 1.5s),给头部物理运动留足时间。
    """

    def __init__(self, name, label, namespace, params):
        super(NodeHeadSequence, self).__init__(name, label, namespace, params)
        self._head_traj = []
        self._current_index = 0
        self._start_time = 0.0
        self._cmd_sent_time = 0.0    # 当前指令发出时间

    def initialise(self):
        yaws = self.params.get("head_search_yaws", [0.0])
        pitchs = self.params.get("head_search_pitchs", [0.0])
        self._head_traj = []
        for pitch in pitchs:
            for yaw in yaws:
                self._head_traj.append((float(yaw), float(pitch)))
        self._current_index = 0
        self._start_time = time.time()
        self._cmd_sent_time = 0.0

    def update(self):
        if _is_dry_run():
            self.feedback_message = f"dry-run head_seq: {len(self._head_traj)} frames"
            return Status.SUCCESS

        timeout = float(self.params.get("scan_timeout", 30.0))
        dwell = float(self.params.get("head_dwell_sec", 1.5))

        if time.time() - self._start_time > timeout:
            self.feedback_message = f"head scan timeout ({timeout}s)"
            return Status.FAILURE

        # 驻留等待: 当前位置未驻留够时间则等待
        if self._cmd_sent_time > 0 and time.time() - self._cmd_sent_time < dwell:
            return Status.RUNNING

        if self._current_index >= len(self._head_traj):
            # 循环重扫
            self._current_index = 0
            self.feedback_message = "head rescan loop (index reset)"

        yaw, pitch = self._head_traj[self._current_index]
        try:
            hw = get_shared_hardware()
            hw.control_head(yaw, pitch)
            self.feedback_message = f"head [{self._current_index}/{len(self._head_traj)}] yaw={yaw:.0f}° pitch={pitch:.0f}°"
            self._cmd_sent_time = time.time()
            self._current_index += 1
        except Exception as e:
            self.feedback_message = f"control_head failed: {e}"
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        # 父 Parallel 终止时调(BaseAction 默认空)
        pass
