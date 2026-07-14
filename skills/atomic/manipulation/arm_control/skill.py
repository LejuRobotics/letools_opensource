from core.common.logger import get_logger
from core.domain.result import Result
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase

from .params import ArmControlParams

logger = get_logger(__name__)


class ArmControlSkill(SkillBase):
    """手臂关节轨迹：enable_quick_mode → send_arm_joint_trajectory（对齐 test_arm_joint）。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_control")
        self.hardware = hardware
        self.params: ArmControlParams = None
        self._done = False
        self._quick_enabled = False

    def on_initialize(self, params: ArmControlParams) -> Result:
        if not isinstance(params, ArmControlParams):
            return Result.fail("Invalid parameters for ArmControlSkill")
        if len(params.joint_angles_deg) != 14:
            return Result.fail(
                f"arm_control expects 14 joint angles, got {len(params.joint_angles_deg)}"
            )
        self.params = params
        self._done = False
        self._quick_enabled = False

        if params.enable_quick_mode:
            quick_result = self.hardware.enable_quick_mode(True)
            if not quick_result.success:
                return quick_result
            self._quick_enabled = True
            logger.info("arm_control: enable_quick_mode(True)")

        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("Arm control already finished")

        traj_result = self.hardware.send_arm_joint_trajectory(
            self.params.joint_angles_deg,
            self.params.time_sec,
        )
        self._done = True
        if traj_result.success:
            logger.info(
                "arm_control: send_arm_joint_trajectory, time_sec=%.1f",
                self.params.time_sec,
            )
        return traj_result

    def on_cancel(self) -> Result:
        if self._quick_enabled:
            self.hardware.enable_quick_mode(False)
            self._quick_enabled = False
        self._done = True
        return Result.ok("ArmControlSkill cancelled")

    def on_is_finished(self) -> bool:
        return self._done
