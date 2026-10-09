"""Copyable Skill template. Replace CustomSkill and the hardware method."""

from dataclasses import dataclass
from typing import Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class CustomSkillParams(SkillParams):
    skill_name: str = "custom_skill"
    value: float = 0.0
    timeout: float = 30.0


class CustomSkill(SkillBase):
    def __init__(self, hardware: IHardware):
        super().__init__(name="custom_skill")
        self.hardware = hardware
        self.params: Optional[CustomSkillParams] = None
        self._done = False

    def on_initialize(self, params: CustomSkillParams) -> Result:
        if not isinstance(params, CustomSkillParams):
            return Result.fail("invalid CustomSkillParams", error_code="INVALID_PARAMS")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("already finished")
        # Replace this line with one IHardware call. Do not import vendor SDK here.
        self._done = True
        return Result.ok("custom skill placeholder executed", data={"value": self.params.value})

    def on_cancel(self) -> Result:
        self._done = True
        return Result.ok("cancelled")

    def on_is_finished(self) -> bool:
        return self._done
