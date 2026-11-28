"""SDK TorsoAPI同步4关节轨迹；SDK管理插值与MPC模式，不做到位验收。"""

from dataclasses import dataclass, field
from typing import List, Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.torso import validate_torso_joint_traj
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetTorsoJointTrajParams(SkillParams):
    skill_name: str = "set_torso_joint_traj"
    joint_traj: List[List[float]] = field(default_factory=list)
    total_time: Optional[float] = None


class SetTorsoJointTrajSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_torso_joint_traj")
        self.hardware = hardware
        self._query: Optional[dict] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetTorsoJointTrajParams) -> Result:
        self._query = None
        self._result = None
        if not isinstance(params, SetTorsoJointTrajParams):
            return Result.fail("Invalid SetTorsoJointTrajParams")
        try:
            frames, duration = validate_torso_joint_traj(params.joint_traj, params.total_time)
            self._query = dict(joint_traj=frames, total_time=duration)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("SetTorsoJointTrajSkill 未成功初始化")
        try:
            self._result = self.hardware.set_torso_joint_traj(**self._query)
        except Exception as exc:
            self._result = Result.fail(f"set_torso_joint_traj 执行失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
