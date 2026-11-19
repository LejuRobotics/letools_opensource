"""读取双臂末端（m/deg），显式TF/FK来源，不做来源回退。"""

from dataclasses import dataclass
from typing import Optional, Tuple

from core.domain.pose import validate_arm_frame
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class GetArmPoseParams(SkillParams):
    skill_name: str = "get_arm_pose"
    frame: str = "local"
    source: str = "tf"
    fk_frame: Optional[str] = None


class GetArmPoseSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="get_arm_pose")
        self.hardware = hardware
        self._query: Optional[Tuple[str, str, Optional[str]]] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: GetArmPoseParams) -> Result:
        self._query = None
        self._result = None
        if not isinstance(params, GetArmPoseParams):
            return Result.fail("Invalid GetArmPoseParams")
        try:
            frame = validate_arm_frame(params.frame)
            if params.source not in ("tf", "fk"):
                return Result.fail("source 必须为 tf 或 fk")
            if params.source == "fk" and (not isinstance(params.fk_frame, str) or not params.fk_frame.strip()):
                return Result.fail("source=fk 时必须提供已确认的 fk_frame")
            self._query = (frame, params.source, params.fk_frame)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("GetArmPoseSkill 未成功初始化")
        frame, source, fk_frame = self._query
        self._result = self.hardware.get_arm_pose(frame=frame, source=source, fk_frame=fk_frame)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
