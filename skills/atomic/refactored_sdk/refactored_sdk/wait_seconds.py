# -*- coding: utf-8 -*-
"""Atomic skill: wait_seconds.

Aligns with WaitSeconds node — 纯编排工具技能，无硬件依赖，阻塞等待指定秒数。
"""

import os
import time
from dataclasses import dataclass

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class WaitSecondsParams(SkillParams):
    """等待指定秒数。"""

    skill_name: str = "wait_seconds"
    duration_sec: float = 1.0
    timeout: float = 120.0


@define_manifest(
    label="等待(秒)",
    category=["utility", "timing"],
    tree_type="studio_smoke",
    description="等待指定秒数后返回 SUCCESS",
    params=[
        {"name": "duration_sec", "type": "float", "default": "1.0", "description": "等待秒数"},
    ],
    inputs=[],
    outputs=[],
)
class WaitSecondsSkill(SkillBase):
    """等待指定秒数（纯编排工具，无硬件依赖）。"""

    def __init__(self):
        super().__init__(name="wait_seconds")
        self.params: WaitSecondsParams = None
        self._start = 0.0

    def on_initialize(self, params: WaitSecondsParams) -> Result:
        if not isinstance(params, WaitSecondsParams):
            return Result.fail("Invalid parameters for WaitSecondsSkill")
        self.params = params
        self._start = time.time()
        return Result.ok()

    def on_execute(self) -> Result:
        _DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        duration = float(self.params.duration_sec) if self.params else 1.0

        if _DRY_RUN:
            return Result.ok("dry-run wait_seconds")

        if time.time() - self._start < duration:
            return Result.ok()  # 还在等待中，不算失败
        return Result.ok()

    def on_is_finished(self) -> bool:
        _DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        if _DRY_RUN:
            return True
        duration = float(self.params.duration_sec) if self.params else 1.0
        return time.time() - self._start >= duration
