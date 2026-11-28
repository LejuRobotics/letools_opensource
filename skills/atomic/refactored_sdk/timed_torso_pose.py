"""躯干Timed目标单次提交；可选准备模式，不插值、不等待到位。"""

from dataclasses import dataclass, field
from typing import List, Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.torso import validate_timed_torso_pose
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class TimedTorsoPoseParams(SkillParams):
    skill_name: str = "timed_torso_pose"
    pose: List[float] = field(default_factory=list)
    desire_time: Optional[float] = None
    is_sync: bool = True
    prepare_arm_control: bool = True


class TimedTorsoPoseSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="timed_torso_pose")
        self.hardware = hardware
        self._query: Optional[dict] = None
        self._result: Optional[Result] = None
        self._prepare_arm_control = True

    def on_initialize(self, params: TimedTorsoPoseParams) -> Result:
        self._query = None
        self._result = None
        self._prepare_arm_control = True
        if not isinstance(params, TimedTorsoPoseParams):
            return Result.fail("Invalid TimedTorsoPoseParams")
        try:
            values, duration = validate_timed_torso_pose(params.pose, params.desire_time, params.is_sync)
            if type(params.prepare_arm_control) is not bool:
                raise ValueError("prepare_arm_control 必须为bool")
            self._prepare_arm_control = params.prepare_arm_control
            self._query = dict(pose=values, desire_time=duration, is_sync=params.is_sync)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("TimedTorsoPoseSkill 未成功初始化")
        try:
            # 沿用Timed组合控制的模式准备；切2可能激活残留手臂目标。
            # 不清空目标、不设置focus、不切MPC或自动恢复模式。
            if self._prepare_arm_control:
                result = self.hardware.set_arm_control_mode(2)
                if not result.success:
                    self._result = result
                    return self._result
            self._result = self.hardware.timed_torso_pose(**self._query)
        except Exception as exc:
            self._result = Result.fail(f"timed_torso_pose 执行失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
