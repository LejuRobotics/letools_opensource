"""躯干4分量单帧发送；固定m/deg，不插值、不等待、不切模式。"""

from dataclasses import dataclass, field
from typing import List, Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.torso import validate_torso_pose
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetTorsoPoseParams(SkillParams):
    skill_name: str = "set_torso_pose"
    pose: List[float] = field(default_factory=list)


class SetTorsoPoseSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_torso_pose")
        self.hardware = hardware
        self._pose: Optional[List[float]] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetTorsoPoseParams) -> Result:
        self._pose = None
        self._result = None
        if not isinstance(params, SetTorsoPoseParams):
            return Result.fail("Invalid SetTorsoPoseParams")
        try:
            self._pose = validate_torso_pose(params.pose)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._pose is None:
            return Result.fail("SetTorsoPoseSkill 未成功初始化")
        try:
            self._result = self.hardware.set_torso_pose(pose=self._pose)
        except Exception as exc:
            self._result = Result.fail(f"set_torso_pose 执行失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
