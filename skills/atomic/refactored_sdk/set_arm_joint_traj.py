"""原生ArmAPI关节轨迹同步执行技能（deg），不做实测到位判断。"""

from dataclasses import dataclass, field
from typing import List, Optional
from core.domain.arm_trajectory import validate_set_arm_joint_traj
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetArmJointTrajParams(SkillParams):
    skill_name: str = "set_arm_joint_traj"
    joint_traj: List[List[float]] = field(default_factory=list)
    total_time: Optional[float] = None


class SetArmJointTrajSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_arm_joint_traj")
        self.hardware = hardware
        self._query: Optional[dict] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetArmJointTrajParams) -> Result:
        self._query = None
        self._result = None
        if not isinstance(params, SetArmJointTrajParams):
            return Result.fail("Invalid SetArmJointTrajParams")
        try:
            frames, duration = validate_set_arm_joint_traj(params.joint_traj, params.total_time)
            self._query = dict(joint_traj=frames, total_time=duration)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("SetArmJointTrajSkill 未成功初始化")
        self._result = self.hardware.set_arm_joint_traj(**self._query)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
