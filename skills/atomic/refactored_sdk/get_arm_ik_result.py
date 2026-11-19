"""双臂IK纯计算技能：输入m/deg，输出14维deg，不发送运动。"""

from dataclasses import dataclass, field
from typing import List, Optional
from core.domain.pose import validate_arm_pose, validate_arm_frame
from core.domain.joint_state import validate_arm_joint
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class GetArmIkResultParams(SkillParams):
    skill_name: str = "get_arm_ik_result"
    left_pose: List[float] = field(default_factory=list)
    right_pose: List[float] = field(default_factory=list)
    frame: str = "local"
    ik_frame: Optional[str] = None
    joint: Optional[List[float]] = None


class GetArmIkResultSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="get_arm_ik_result")
        self.hardware = hardware
        self._query: Optional[dict] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: GetArmIkResultParams) -> Result:
        self._query = None
        self._result = None
        if not isinstance(params, GetArmIkResultParams):
            return Result.fail("Invalid GetArmIkResultParams")
        try:
            left, right = validate_arm_pose(params.left_pose), validate_arm_pose(params.right_pose)
            frame = validate_arm_frame(params.frame)
            if not isinstance(params.ik_frame, str) or not params.ik_frame.strip():
                return Result.fail("必须提供已确认的 ik_frame")
            joint = None if params.joint is None else validate_arm_joint(params.joint)
            self._query = dict(left_pose=left, right_pose=right, frame=frame, ik_frame=params.ik_frame, joint=joint)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("GetArmIkResultSkill 未成功初始化")
        self._result = self.hardware.get_arm_ik_result(**self._query)
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
