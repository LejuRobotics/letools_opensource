"""单次查询SDK手臂控制模式，不修改模式。"""

from dataclasses import dataclass
from typing import Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class GetArmControlModeParams(SkillParams):
    skill_name: str = "get_arm_control_mode"


class GetArmControlModeSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="get_arm_control_mode")
        self.hardware = hardware
        self._ready = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: GetArmControlModeParams) -> Result:
        self._ready = isinstance(params, GetArmControlModeParams)
        self._result = None
        return Result.ok() if self._ready else Result.fail("Invalid GetArmControlModeParams")

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if not self._ready:
            return Result.fail("GetArmControlModeSkill 未成功初始化")
        self._result = self.hardware.get_arm_control_mode()
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
