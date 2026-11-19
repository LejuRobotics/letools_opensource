"""双臂原生关节轨迹发送技能（deg），保留每帧时间，不重采样或等待。"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from core.domain.joint_state import validate_arm_joint_trajectory
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetArmJointTrajectoryParams(SkillParams):
    skill_name: str = "set_arm_joint_trajectory"
    times: List[float] = field(default_factory=list)
    q_frames: List[List[float]] = field(default_factory=list)


class SetArmJointTrajectorySkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_arm_joint_trajectory")
        self.hardware = hardware
        self._trajectory: Optional[Tuple[List[float], List[List[float]]]] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetArmJointTrajectoryParams) -> Result:
        self._trajectory = None
        self._result = None
        if not isinstance(params, SetArmJointTrajectoryParams):
            return Result.fail("Invalid parameters for SetArmJointTrajectorySkill")
        try:
            self._trajectory = validate_arm_joint_trajectory(params.times, params.q_frames)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._trajectory is None:
            return Result.fail("SetArmJointTrajectorySkill 未成功初始化")
        times, frames = self._trajectory
        self._result = self.hardware.set_arm_joint_trajectory(times=times, q_frames=frames)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
