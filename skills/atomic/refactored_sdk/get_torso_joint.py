"""单次读取4关节deg反馈缓存，不检查到位或新鲜度。"""

from dataclasses import dataclass
from typing import Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.torso import validate_torso_joint
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class GetTorsoJointParams(SkillParams):
    skill_name: str = "get_torso_joint"


class GetTorsoJointSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="get_torso_joint")
        self.hardware = hardware
        self._initialized = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: GetTorsoJointParams) -> Result:
        self._result = None
        self._initialized = isinstance(params, GetTorsoJointParams)
        return Result.ok() if self._initialized else Result.fail("Invalid GetTorsoJointParams")

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if not self._initialized:
            return Result.fail("GetTorsoJointSkill 未成功初始化")
        try:
            result = self.hardware.get_torso_joint()
            if result.success:
                validate_torso_joint(result.data)
            self._result = result
        except Exception as exc:
            self._result = Result.fail(f"get_torso_joint 执行失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
