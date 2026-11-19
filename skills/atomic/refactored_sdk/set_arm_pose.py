"""单次发送双臂末端位姿（xyz米、yaw/pitch/roll度），默认local。"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from core.domain.pose import validate_arm_pose, validate_arm_frame
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class SetArmPoseParams(SkillParams):
    skill_name: str = "set_arm_pose"
    left_pose: List[float] = field(default_factory=list)
    right_pose: List[float] = field(default_factory=list)
    frame: str = "local"


class SetArmPoseSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_arm_pose")
        self.hardware = hardware
        self._poses: Optional[Tuple[List[float], List[float], str]] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetArmPoseParams) -> Result:
        self._poses = None
        self._result = None
        if not isinstance(params, SetArmPoseParams):
            return Result.fail("Invalid SetArmPoseParams")
        try:
            left = validate_arm_pose(params.left_pose)
            right = validate_arm_pose(params.right_pose)
            frame = validate_arm_frame(params.frame)
            self._poses = (left, right, frame)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._poses is None:
            return Result.fail("SetArmPoseSkill 未成功初始化")
        left, right, frame = self._poses
        self._result = self.hardware.set_arm_pose(left_pose=left, right_pose=right, frame=frame)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
