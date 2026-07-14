from core.common.logger import get_logger
from core.domain.result import Result
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase

from .params import HeadControlParams

logger = get_logger(__name__)


class HeadControlSkill(SkillBase):
    """头部单步：control_head(yaw, pitch) 角度。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="head_control")
        self.hardware = hardware
        self.params: HeadControlParams = None
        self._done = False

    def on_initialize(self, params: HeadControlParams) -> Result:
        if not isinstance(params, HeadControlParams):
            return Result.fail("Invalid parameters for HeadControlSkill")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("Head control already finished")

        result = self.hardware.control_head(
            self.params.yaw_deg,
            self.params.pitch_deg,
        )
        self._done = True
        if result.success:
            logger.info(
                "head_control: yaw=%.1f pitch=%.1f deg",
                self.params.yaw_deg,
                self.params.pitch_deg,
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done
