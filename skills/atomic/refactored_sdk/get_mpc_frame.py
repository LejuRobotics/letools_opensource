"""独立MPC frameget，缓存一次调用的完整结果。"""

from dataclasses import dataclass
from typing import Optional
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.mpc import validate_mpc_value
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class GetMpcFrameParams(SkillParams):
    skill_name: str = "get_mpc_frame"


class GetMpcFrameSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="get_mpc_frame")
        self.hardware = hardware
        self._ready = False
        self._value = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: GetMpcFrameParams) -> Result:
        self._result = None
        self._ready = False
        self._value = None
        if not isinstance(params, GetMpcFrameParams):
            return Result.fail("Invalid GetMpcFrameParams")
        self._ready = True
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if not self._ready:
            return Result.fail("GetMpcFrameSkill 未成功初始化")
        try:
            self._result = self.hardware.get_mpc_frame()
        except Exception as exc:
            self._result = Result.fail(f"get_mpc_frame 失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
