# -*- coding: utf-8 -*-
"""Atomic skill: check_arrived_jibot_sdk.

Aligns with `test_check_arrived.py` by calling `hardware.check_arrived_jibot()`.
"""

from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class CheckArrivedJibotParams(SkillParams):
    """对齐 test_check_arrived.py：hardware.check_arrived_jibot()。"""

    skill_name: str = "check_arrived_jibot_sdk"
    task_id: str = ""
    blocking: bool = True
    timeout: float = 20.0


@define_manifest(
    label="JiBot底盘任务到达检查",
    category=["motion", "chassis", "jibot"],
    tree_type="studio_smoke",
    description="对齐 test_check_arrived.py：调用 hardware.check_arrived_jibot()",
    params=[
        {"name": "task_id", "type": "string", "default": "", "description": "由base_move或move_to_target返回的任务ID"},
        {"name": "blocking", "type": "bool", "default": "True", "description": "是否阻塞等待任务完成"},
        {"name": "timeout", "type": "float", "default": "20.0", "description": "超时时间(s)，blocking=True时有效"},
    ],
    inputs=[],
    outputs=[],
)
class CheckArrivedJibotSkill(SkillBase):
    """JiBot底盘任务到达检查（Adapter）：hardware.check_arrived_jibot()。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="check_arrived_jibot_sdk")
        self.hardware = hardware
        self.params: Optional[CheckArrivedJibotParams] = None
        self._done = False
        self._result = None

    def on_initialize(self, params: CheckArrivedJibotParams) -> Result:
        if not isinstance(params, CheckArrivedJibotParams):
            return Result.fail("Invalid parameters for CheckArrivedJibotSkill")
        self.params = params
        self._done = False
        self._result = None
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return self._result if self._result else Result.ok("Already finished")

        result = self.hardware.check_arrived_jibot(
            task_id=str(self.params.task_id),
            blocking=bool(self.params.blocking),
            timeout=float(self.params.timeout),
        )
        self._done = True
        self._result = result
        if result.success:
            logger.info(
                "check_arrived_jibot_sdk: task_id=%s arrived=%s status=%d message=%s",
                str(self.params.task_id),
                result.data.get("arrived") if result.data else "N/A",
                result.data.get("status") if result.data else -1,
                result.data.get("message") if result.data else "N/A",
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done
