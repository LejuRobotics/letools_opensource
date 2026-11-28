"""单次设置躯干Ruckig持久参数，不发送运动目标或自动恢复。"""

from dataclasses import dataclass, field
from typing import List, Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.ruckig_params import validate_torso_ruckig_params
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class TimedTorsoRuckigParams(SkillParams):
    skill_name: str = "timed_torso_ruckig_params"
    mode: str = ""
    velocity_max: List[float] = field(default_factory=list)
    acceleration_max: List[float] = field(default_factory=list)
    jerk_max: List[float] = field(default_factory=list)
    velocity_min: Optional[List[float]] = None
    acceleration_min: Optional[List[float]] = None
    is_sync: bool = True


class TimedTorsoRuckigParamsSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="timed_torso_ruckig_params")
        self.hardware = hardware
        self._query: Optional[dict] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: TimedTorsoRuckigParams) -> Result:
        self._query = None
        self._result = None
        if not isinstance(params, TimedTorsoRuckigParams):
            return Result.fail("Invalid TimedTorsoRuckigParams")
        try:
            self._query = validate_torso_ruckig_params(
                params.mode, params.velocity_max, params.acceleration_max, params.jerk_max,
                params.velocity_min, params.acceleration_min, params.is_sync)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("TimedTorsoRuckigParamsSkill 未成功初始化")
        try:
            self._result = self.hardware.timed_torso_ruckig_params(**self._query)
        except Exception as exc:
            self._result = Result.fail(f"timed_torso_ruckig_params 执行失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
