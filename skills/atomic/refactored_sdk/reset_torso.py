"""躯干复位请求，单次调用并缓存结果，不等待到位。"""

from dataclasses import dataclass
from typing import Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class ResetTorsoParams(SkillParams):
    skill_name: str = "reset_torso"


class ResetTorsoSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="reset_torso")
        self.hardware = hardware
        self._initialized = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: ResetTorsoParams) -> Result:
        self._result = None
        self._initialized = isinstance(params, ResetTorsoParams)
        return Result.ok() if self._initialized else Result.fail("Invalid ResetTorsoParams")

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if not self._initialized:
            return Result.fail("ResetTorsoSkill 未成功初始化")
        try:
            self._result = self.hardware.reset_torso()
        except Exception as exc:
            self._result = Result.fail(f"reset_torso 执行失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
