"""双臂14维关节单目标发送技能（deg），不等待到位。"""

from dataclasses import dataclass, field
from typing import List, Optional

from core.domain.joint_state import validate_arm_joint
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetArmJointParams(SkillParams):
    skill_name: str = "set_arm_joint"
    joint: List[float] = field(default_factory=list)


class SetArmJointSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_arm_joint")
        self.hardware = hardware
        self._joint: Optional[List[float]] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetArmJointParams) -> Result:
        self._joint = None
        self._result = None
        if not isinstance(params, SetArmJointParams):
            return Result.fail("Invalid parameters for SetArmJointSkill")
        try:
            self._joint = validate_arm_joint(params.joint)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._joint is None:
            return Result.fail("SetArmJointSkill 未成功初始化")
        self._result = self.hardware.set_arm_joint(joint=self._joint)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
