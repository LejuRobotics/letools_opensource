import time

from core.common.logger import get_logger
from core.domain.result import Result
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase

from .params import ChassisVelocityParams

logger = get_logger(__name__)


class ChassisVelocitySkill(SkillBase):
    """底盘短动：send_base_velocity → 等待 duration → 零速。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="chassis_velocity")
        self.hardware = hardware
        self.params: ChassisVelocityParams = None
        self._phase = "idle"
        self._move_start = 0.0

    def on_initialize(self, params: ChassisVelocityParams) -> Result:
        if not isinstance(params, ChassisVelocityParams):
            return Result.fail("Invalid parameters for ChassisVelocitySkill")
        self.params = params
        self._phase = "moving"
        self._move_start = time.time()
        result = self.hardware.send_base_velocity(
            self.params.vx,
            self.params.vy,
            self.params.vyaw,
            self.params.frame,
        )
        if not result.success:
            self._phase = "failed"
            return result
        logger.info(
            "chassis_velocity: start vx=%.3f vy=%.3f vyaw=%.3f duration=%.1fs",
            self.params.vx,
            self.params.vy,
            self.params.vyaw,
            self.params.duration_sec,
        )
        return Result.ok()

    def on_execute(self) -> Result:
        if self._phase == "failed":
            return Result.fail("Chassis velocity start failed")

        if self._phase == "moving":
            elapsed = time.time() - self._move_start
            if elapsed < self.params.duration_sec:
                self.hardware.send_base_velocity(
                    self.params.vx,
                    self.params.vy,
                    self.params.vyaw,
                    self.params.frame,
                )
                return Result.ok()
            self._phase = "stopping"

        if self._phase == "stopping":
            result = self.hardware.send_base_velocity(
                0.0, 0.0, 0.0, self.params.frame
            )
            if not result.success:
                self._phase = "failed"
                return result
            logger.info("chassis_velocity: zero velocity sent")
            self._phase = "done"
            return Result.ok()

        return Result.ok()

    def on_cancel(self) -> Result:
        if self.params is not None:
            self.hardware.send_base_velocity(0.0, 0.0, 0.0, self.params.frame)
        self._phase = "done"
        return Result.ok("ChassisVelocitySkill cancelled")

    def on_is_finished(self) -> bool:
        return self._phase in ("done", "failed")
