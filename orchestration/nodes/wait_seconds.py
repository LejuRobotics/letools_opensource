# -*- coding: utf-8 -*-
"""WaitSeconds：等待指定秒数薄节点 → wait_seconds 原子技能。"""

import os

from py_trees.common import Access, Status

from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.wait_seconds import (
    WaitSecondsParams,
    WaitSecondsSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="等待(秒)",
    category=["utility", "timing"],
    tree_type="studio_smoke",
    description="等待指定秒数后返回 SUCCESS（RUNNING 可重入/幂等）",
    params=[
        {"name": "duration_sec", "type": "float", "default": "1.0", "description": "等待秒数"},
    ],
    inputs=[],
    outputs=[],
)
class WaitSeconds(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None

        if _DRY_RUN:
            self._dry_done = True
            return

        # ⚠️ **`timeout` 要跟着 `duration_sec` 一起给**（2026-09-30 修）。
        # `WaitSecondsParams.timeout` 的默认是 **120s**，而 `SkillBase.execute()`
        # 每次都查 `now - start > timeout` -> `Result.fail("Timeout")` **并打
        # ERROR 日志**。这个技能本来就是"等着不做事"，所以 duration 一超过 120s
        # （场景里是 `86400`）就**必然**超时退化成 FAILURE ——
        # `Parallel(success_on_one)` 于是把整棵树判 FAILURE，`start_all.sh`
        # 跟着收摊。现场表现就是"跑着跑着自己退出了"。
        # 实测（`skills/atomic/refactored_sdk/wait_seconds.py`）：
        #   duration=86400 timeout=120   -> execute 返回 fail("Timeout") + ERROR 日志
        #   duration=86400 timeout=86400 -> ok
        _dur = float(self._param("duration_sec", 1.0))
        skill_params = WaitSecondsParams(
            duration_sec=_dur,
            timeout=max(_dur + 60.0, 120.0),
        )
        self._skill = WaitSecondsSkill()
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "wait_seconds init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "wait_seconds failed"
            return Status.FAILURE
        return Status.RUNNING

    def _param(self, key, default=None):
        board_key = str(
            self.params.get(f"{key}__board_key", "")
        ).strip()
        if board_key:
            self.global_blackboard.register_key(key=board_key, access=Access.READ)
            if not self.global_blackboard.exists(board_key):
                raise ValueError(f"黑板缺少运行时键 {board_key}")
            return self.global_blackboard.get(board_key)
        return self.params.get(key, default)
