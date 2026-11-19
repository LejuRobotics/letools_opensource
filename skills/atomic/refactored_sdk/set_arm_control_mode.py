"""单次设置手臂模式，复用现有Hardware入口，不自动恢复或等待到位。"""

from dataclasses import dataclass
from typing import Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetArmControlModeParams(SkillParams):
    skill_name: str = "set_arm_control_mode"
    control_mode: Optional[int] = None


class SetArmControlModeSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_arm_control_mode")
        self.hardware = hardware
        self._mode: Optional[int] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetArmControlModeParams) -> Result:
        self._mode = None
        self._result = None
        if not isinstance(params, SetArmControlModeParams):
            return Result.fail("Invalid SetArmControlModeParams")
        if type(params.control_mode) is not int or params.control_mode not in (0, 1, 2):
            return Result.fail("control_mode 必须为整数0、1或2")
        self._mode = params.control_mode
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._mode is None:
            return Result.fail("SetArmControlModeSkill 未成功初始化")
        self._result = self.hardware.set_arm_control_mode(control_mode=self._mode)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
