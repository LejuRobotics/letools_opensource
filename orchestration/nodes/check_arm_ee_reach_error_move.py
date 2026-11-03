# -*- coding: utf-8 -*-
"""CheckArmEeReachErrorMove：手臂末端到位检查与误差反馈。

轮询 ``hardware.get_ee_pose_reach_error()``（服务
``/mobile_manipulator_ee_pose_reach_error``）：该服务的 ``success`` 即控制器的
"末端运动已完成"判定，因此本节点**只以服务 success 为准**，不另设阈值，
避免出现"控制器说到了、节点说没到"的矛盾；误差数值只作输出上报。

到位后把误差向量写入黑板供后续节点使用，终端 feedback 里也报一份。
"""

import os
import time

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

_SIDES = ("left", "right")
_ERR_VECTOR_LEN = 6


@define_manifest(
    label="手臂末端到位检查",
    category=["motion", "arm", "check"],
    tree_type="studio_smoke",
    description=(
        "轮询 hardware.get_ee_pose_reach_error()，到位返回 SUCCESS 并报出执行误差；"
        "超时返回 FAILURE"
    ),
    params=[
        {"name": "side", "type": "string", "default": "left", "description": "检查哪一侧：left / right / both"},
        {"name": "timeout", "type": "float", "default": "10.0", "description": "等待到位的最长秒数，超时判 FAILURE"},
        {"name": "poll_interval", "type": "float", "default": "0.1", "description": "轮询间隔（秒）"},
        {
            "name": "result_key",
            "type": "string",
            "default": "arm_ee_reach_error",
            "description": "误差结果写入黑板的键名，留空则不写",
        },
    ],
    inputs=[],
    outputs=[],
)
class CheckArmEeReachErrorMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._hardware = None
        self._sides = ()
        self._pending = ()
        self._errors = {}
        self._deadline = 0.0
        self._poll_interval = 0.1
        self._result_key = ""
        self._setup_error = None

    def initialise(self):
        self._hardware = None
        self._sides = ()
        self._pending = ()
        self._errors = {}
        self._deadline = 0.0
        self._result_key = ""
        self._setup_error = None

        try:
            sides = self._parse_sides(self.params.get("side", "left"))
            timeout = self._positive_float(self.params.get("timeout", 10.0), "timeout")
            self._poll_interval = self._positive_float(
                self.params.get("poll_interval", 0.1), "poll_interval"
            )
        except ValueError as exc:
            self._setup_error = str(exc)
            self.feedback_message = f"check_arm_ee_reach_error: {exc}"
            return

        self._sides = sides
        self._pending = list(sides)
        # 默认键名与 manifest 的 default 保持一致：params 里缺该字段时也要写黑板
        self._result_key = str(
            self.params.get("result_key", "arm_ee_reach_error") or ""
        ).strip()
        self._deadline = time.monotonic() + timeout

        if _DRY_RUN:
            self.feedback_message = f"dry-run check_arm_ee_reach_error side={','.join(sides)}"
            return

        self._hardware = get_shared_hardware()

    def update(self):
        if self._setup_error is not None:
            return Status.FAILURE

        if _DRY_RUN:
            return Status.SUCCESS

        if self._hardware is None:
            self.feedback_message = "check_arm_ee_reach_error: hardware unavailable"
            return Status.FAILURE

        for side in list(self._pending):
            result = self._hardware.get_ee_pose_reach_error(is_left=(side == "left"))
            if not result.success:
                continue
            err_vector = self._extract_error(result, side)
            if err_vector is None:
                self.feedback_message = f"check_arm_ee_reach_error: {side} 静差向量维度异常"
                return Status.FAILURE
            self._errors[side] = err_vector
            self._pending.remove(side)

        if not self._pending:
            self._publish_result()
            return Status.SUCCESS

        if time.monotonic() >= self._deadline:
            self.feedback_message = (
                f"check_arm_ee_reach_error: 等待到位超时，未到位侧={'/'.join(self._pending)}"
            )
            return Status.FAILURE

        time.sleep(self._poll_interval)
        self.feedback_message = f"check_arm_ee_reach_error: 等待到位 {','.join(self._pending)}"
        return Status.RUNNING

    @staticmethod
    def _parse_sides(raw):
        """归一化 side 参数，返回按固定顺序排列的侧别元组。"""
        value = str(raw).strip().lower()
        if value == "both":
            return _SIDES
        if value not in _SIDES:
            raise ValueError(f"side 必须为 left / right / both，实际为 {raw!r}")
        return (value,)

    @staticmethod
    def _positive_float(raw, name):
        try:
            value = float(raw)
        except (TypeError, ValueError):
            raise ValueError(f"{name} 必须为正数，实际为 {raw!r}")
        if value <= 0.0:
            raise ValueError(f"{name} 必须为正数，实际为 {raw!r}")
        return value

    @staticmethod
    def _extract_error(result, side):
        """取出 6 维静差向量；维度异常返回 None（由调用方判失败）。"""
        data = result.data if isinstance(result.data, dict) else {}
        err_vector = data.get("err_vector")
        if not isinstance(err_vector, (list, tuple)) or len(err_vector) != _ERR_VECTOR_LEN:
            return None
        return [float(v) for v in err_vector]

    def _publish_result(self):
        """写黑板并组织 feedback 文案。"""
        if self._result_key:
            try:
                self.global_blackboard.register_key(
                    key=self._result_key, access=py_trees.common.Access.WRITE
                )
            except Exception:
                pass
            try:
                self.global_blackboard.set(self._result_key, dict(self._errors))
            except Exception as exc:
                # 黑板写不进去不影响"是否到位"这一结论，只记录不判失败
                self.feedback_message = f"check_arm_ee_reach_error: 写黑板失败 {exc}"
                return

        details = " ".join(
            f"{side}=[{', '.join(f'{v:.6f}' for v in self._errors[side])}]"
            for side in self._sides
            if side in self._errors
        )
        self.feedback_message = f"check_arm_ee_reach_error: 已到位 ({details})"
