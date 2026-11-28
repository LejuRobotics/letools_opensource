"""躯干位姿查询，单次调用并缓存结果，不等待到位。"""

from dataclasses import dataclass
from typing import Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class GetTorsoPoseParams(SkillParams):
    skill_name: str = "get_torso_pose"


class GetTorsoPoseSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="get_torso_pose")
        self.hardware = hardware
        self._initialized = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: GetTorsoPoseParams) -> Result:
        self._result = None
        self._initialized = isinstance(params, GetTorsoPoseParams)
        return Result.ok() if self._initialized else Result.fail("Invalid GetTorsoPoseParams")

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if not self._initialized:
            return Result.fail("GetTorsoPoseSkill 未成功初始化")
        try:
            self._result = self.hardware.get_torso_pose()
        except Exception as exc:
            self._result = Result.fail(f"get_torso_pose 执行失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
