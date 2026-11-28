"""轮式躯干4关节单帧发送；固定deg，不插值、不等待、不切模式。"""

from dataclasses import dataclass, field
from typing import List, Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.torso import validate_torso_joint
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetTorsoJointParams(SkillParams):
    skill_name: str = "set_torso_joint"
    joint: List[float] = field(default_factory=list)


class SetTorsoJointSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_torso_joint")
        self.hardware = hardware
        self._joint: Optional[List[float]] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetTorsoJointParams) -> Result:
        self._joint = None
        self._result = None
        if not isinstance(params, SetTorsoJointParams):
            return Result.fail("Invalid SetTorsoJointParams")
        try:
            self._joint = validate_torso_joint(params.joint)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._joint is None:
            return Result.fail("SetTorsoJointSkill 未成功初始化")
        try:
            self._result = self.hardware.set_torso_joint(joint=self._joint)
        except Exception as exc:
            self._result = Result.fail(f"set_torso_joint 执行失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
