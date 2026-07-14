from core.common.logger import get_logger
from core.domain.result import Result
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase

from .params import LegControlParams

logger = get_logger(__name__)


class LegControlSkill(SkillBase):
    """腿部关节单步控制：send_leg_joint_command。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="leg_control")
        self.hardware = hardware
        self.params: LegControlParams = None
        self._done = False

    def on_initialize(self, params: LegControlParams) -> Result:
        if not isinstance(params, LegControlParams):
            return Result.fail("Invalid parameters for LegControlSkill")
        if len(params.joint_angles_deg) != 4:
            return Result.fail(
                f"leg_control expects 4 joint angles, got {len(params.joint_angles_deg)}"
            )
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("Leg control already finished")

        result = self.hardware.send_leg_joint_command(self.params.joint_angles_deg)
        if not result.success:
            self._done = True
            return result

        logger.info(
            "leg_control: sent joint_angles_deg=%s", self.params.joint_angles_deg
        )
        self._done = True
        return Result.ok()

    def on_is_finished(self) -> bool:
        return self._done
