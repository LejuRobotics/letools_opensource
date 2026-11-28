"""独立MPC flowset，缓存一次调用的完整结果。"""

from dataclasses import dataclass
from typing import Optional
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.mpc import validate_mpc_value
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetMpcControlFlowParams(SkillParams):
    skill_name: str = "set_mpc_control_flow"
    flow: Optional[int] = None


class SetMpcControlFlowSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_mpc_control_flow")
        self.hardware = hardware
        self._ready = False
        self._value = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetMpcControlFlowParams) -> Result:
        self._result = None
        self._ready = False
        self._value = None
        if not isinstance(params, SetMpcControlFlowParams):
            return Result.fail("Invalid SetMpcControlFlowParams")
        try:
            self._value = validate_mpc_value("flow", params.flow)
        except ValueError as exc:
            return Result.fail(str(exc))
        self._ready = True
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if not self._ready:
            return Result.fail("SetMpcControlFlowSkill 未成功初始化")
        try:
            self._result = self.hardware.set_mpc_control_flow(flow=self._value)
        except Exception as exc:
            self._result = Result.fail(f"set_mpc_control_flow 失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
