"""读取双臂关节反馈（deg），单次执行缓存结果。"""

from dataclasses import dataclass
from typing import Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class GetArmJointParams(SkillParams):
    skill_name: str = "get_arm_joint"


class GetArmJointSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="get_arm_joint")
        self.hardware = hardware
        self._ready = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: GetArmJointParams) -> Result:
        self._ready = isinstance(params, GetArmJointParams)
        self._result = None
        return Result.ok() if self._ready else Result.fail("Invalid GetArmJointParams")

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if not self._ready:
            return Result.fail("GetArmJointSkill 未成功初始化")
        self._result = self.hardware.get_arm_joint()
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
