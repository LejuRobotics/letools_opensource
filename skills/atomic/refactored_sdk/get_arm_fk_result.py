"""指定双臂deg关节求m/deg位姿，不读取当前关节、不发送运动。"""

from dataclasses import dataclass, field
from typing import List, Optional
from core.domain.pose import validate_arm_frame
from core.domain.joint_state import validate_arm_joint
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class GetArmFkResultParams(SkillParams):
    skill_name: str = "get_arm_fk_result"
    joint: List[float] = field(default_factory=list)
    frame: str = "local"
    fk_frame: Optional[str] = None


class GetArmFkResultSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="get_arm_fk_result")
        self.hardware = hardware
        self._query: Optional[dict] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: GetArmFkResultParams) -> Result:
        self._query = None
        self._result = None
        if not isinstance(params, GetArmFkResultParams):
            return Result.fail("Invalid GetArmFkResultParams")
        try:
            joint = validate_arm_joint(params.joint)
            frame = validate_arm_frame(params.frame)
            if not isinstance(params.fk_frame, str) or not params.fk_frame.strip():
                return Result.fail("必须提供已确认的 fk_frame")
            self._query = dict(joint=joint, frame=frame, fk_frame=params.fk_frame)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("GetArmFkResultSkill 未成功初始化")
        self._result = self.hardware.get_arm_fk_result(**self._query)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
