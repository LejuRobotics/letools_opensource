"""单/双臂multi关节目标技能：固定deg，planner8/9，不等待到位。"""

from dataclasses import dataclass
from typing import List, Optional

from core.domain.joint_state import validate_timed_arm_joint_targets
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class TimedArmJointParams(SkillParams):
    skill_name: str = "timed_arm_joint"
    left_arm_joint: Optional[List[float]] = None
    right_arm_joint: Optional[List[float]] = None
    desire_time: Optional[float] = None
    is_sync: bool = True


class TimedArmJointSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="timed_arm_joint")
        self.hardware = hardware
        self._query: Optional[dict] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: TimedArmJointParams) -> Result:
        self._query = None
        self._result = None
        if not isinstance(params, TimedArmJointParams):
            return Result.fail("Invalid TimedArmJointParams")
        try:
            left, right, duration = validate_timed_arm_joint_targets(
                params.left_arm_joint, params.right_arm_joint, params.desire_time, params.is_sync)
            self._query = dict(left_arm_joint=left, right_arm_joint=right,
                               desire_time=duration, is_sync=params.is_sync)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("TimedArmJointSkill 未成功初始化")
        self._result = self.hardware.timed_arm_joint(**self._query)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
