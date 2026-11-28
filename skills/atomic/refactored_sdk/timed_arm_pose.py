"""单/双臂MultiCmd末端目标技能：固定m/deg，服务返回不等于到位。"""

from dataclasses import dataclass
from typing import List, Optional

from core.domain.pose import validate_timed_arm_pose_targets
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase


@dataclass
class TimedArmPoseParams(SkillParams):
    skill_name: str = "timed_arm_pose"
    left_pose: Optional[List[float]] = None
    right_pose: Optional[List[float]] = None
    desire_time: Optional[float] = None
    frame: str = "local"
    is_sync: bool = True
    focus_ee: Optional[bool] = None
    focus_z: Optional[bool] = None
    prepare_arm_control: bool = True


class TimedArmPoseSkill(SkillBase):
    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="timed_arm_pose")
        self.hardware = hardware
        self._query: Optional[dict] = None
        self._result: Optional[Result] = None
        self._focus: tuple = ()
        self._prepare_arm_control = True

    def on_initialize(self, params: TimedArmPoseParams) -> Result:
        self._query = None
        self._result = None
        self._focus = ()
        self._prepare_arm_control = True
        if not isinstance(params, TimedArmPoseParams):
            return Result.fail("Invalid TimedArmPoseParams")
        try:
            left, right, duration, frame = validate_timed_arm_pose_targets(
                params.left_pose, params.right_pose, params.desire_time, params.frame, params.is_sync)
            for label, value in (("focus_ee", params.focus_ee), ("focus_z", params.focus_z)):
                if value is not None and type(value) is not bool:
                    raise ValueError(f"{label} 必须为bool或None，None表示不修改")
            if type(params.prepare_arm_control) is not bool:
                raise ValueError("prepare_arm_control 必须为bool")
            self._prepare_arm_control = params.prepare_arm_control
            self._focus = (("set_focus_ee", params.focus_ee), ("set_focus_z", params.focus_z))
            self._query = dict(left_pose=left, right_pose=right,
                               desire_time=duration, frame=frame, is_sync=params.is_sync)
        except ValueError as exc:
            return Result.fail(str(exc))
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self._query is None:
            return Result.fail("TimedArmPoseSkill 未成功初始化")
        try:
            # 切2本身可能激活残留目标；调用前须确认控制器无危险旧目标。
            # 这里只检查接口返回，不保证模式已实时确认或目标已清空。
            if self._prepare_arm_control:
                result = self.hardware.set_arm_control_mode(2)
                if not result.success:
                    self._result = result
                    return self._result
            # focus是独立的全局话题配置，由Skill组合，不透传给运动Adapter。
            # 设置与运动提交非原子；失败不发目标，不回滚或自动恢复已生效的设置。
            for name, value in self._focus:
                if value is not None:
                    result = getattr(self.hardware, name)(value)
                    if not result.success:
                        self._result = result
                        return self._result
            self._result = self.hardware.timed_arm_pose(**self._query)
        except Exception as exc:
            # 缓存异常失败，避免下一tick重复设置focus或重复发令。
            self._result = Result.fail(f"timed_arm_pose 执行失败: {exc}")
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None
