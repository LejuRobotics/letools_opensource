"""轮式双臂末端轨迹ArmAPI同步执行技能（m/deg）。"""

from dataclasses import dataclass, field
from typing import List, Optional
from core.domain.arm_trajectory import validate_set_arm_pose_traj
from core.domain.pose import validate_arm_frame
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetArmPoseTrajParams(SkillParams):
    skill_name: str = "set_arm_pose_traj"
    left_traj: List[List[float]] = field(default_factory=list)
    right_traj: List[List[float]] = field(default_factory=list)
    total_time: Optional[float] = None
    frame: str = "local"
    direct_to_wbc: bool = True
    back_default: bool = False


class SetArmPoseTrajSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_arm_pose_traj")
        self.hardware = hardware
        self._query: Optional[dict] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetArmPoseTrajParams) -> Result:
        self._query = None
        self._result = None
        if not isinstance(params, SetArmPoseTrajParams):
            return Result.fail("Invalid SetArmPoseTrajParams")
        try:
            left, right, duration = validate_set_arm_pose_traj(params.left_traj, params.right_traj, params.total_time)
            frame = validate_arm_frame(params.frame)
            if type(params.direct_to_wbc) is not bool or type(params.back_default) is not bool:
                return Result.fail("direct_to_wbc/back_default必须为bool")
            self._query = dict(left_traj=left, right_traj=right, total_time=duration, frame=frame,
                               direct_to_wbc=params.direct_to_wbc, back_default=params.back_default)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("SetArmPoseTrajSkill 未成功初始化")
        self._result = self.hardware.set_arm_pose_traj(**self._query)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
