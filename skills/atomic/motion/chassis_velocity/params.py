from dataclasses import dataclass

from core.domain.enums import FrameType
from core.domain.skill_params import SkillParams


@dataclass
class ChassisVelocityParams(SkillParams):
    """底盘短动参数（锁定值见 PHASE1 §2.5）。"""

    skill_name: str = "chassis_velocity"
    vx: float = 0.3
    vy: float = 0.0
    vyaw: float = 0.0
    duration_sec: float = 3.0
    frame: FrameType = FrameType.LOCAL
    timeout: float = 30.0
