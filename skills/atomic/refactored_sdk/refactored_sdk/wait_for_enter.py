# -*- coding: utf-8 -*-
"""Atomic skill: wait_for_enter.

Aligns with WaitForEnter node — 纯编排工具技能，无硬件依赖，等待用户按 Enter 后继续。
"""

import os
import sys
from dataclasses import dataclass

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class WaitForEnterParams(SkillParams):
    """等待用户按 Enter。"""

    skill_name: str = "wait_for_enter"
    message: str = "按 Enter 继续..."
    timeout: float = 3600.0  # 1小时，基本不会超时


@define_manifest(
    label="按Enter继续",
    category=["utility", "test"],
    tree_type="studio_smoke",
    description="等待用户按下 Enter 后继续，用于调试时暂停以便查看日志",
    params=[
        {
            "name": "message",
            "type": "string",
            "default": "按 Enter 继续...",
            "description": "暂停时的提示信息",
        },
    ],
    inputs=[],
    outputs=[],
)
class WaitForEnterSkill(SkillBase):
    """等待用户按 Enter（纯编排工具，无硬件依赖）。"""

    def __init__(self):
        super().__init__(name="wait_for_enter")
        self.params: WaitForEnterParams = None
        self._done = False

    def on_initialize(self, params: WaitForEnterParams) -> Result:
        if not isinstance(params, WaitForEnterParams):
            return Result.fail("Invalid parameters for WaitForEnterSkill")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("WaitForEnterSkill already finished")

        _DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        if _DRY_RUN:
            self._done = True
            return Result.ok("dry-run wait_for_enter")

        msg = self.params.message if self.params else "按 Enter 继续..."
        try:
            input(msg)
        except EOFError:
            pass
        self._done = True
        return Result.ok()

    def on_is_finished(self) -> bool:
        return self._done
