from dataclasses import dataclass

from core.domain.skill_params import SkillParams


@dataclass
class HeadControlParams(SkillParams):
    """头部控制（锁定值见 PHASE1 §2.5）。"""

    skill_name: str = "head_control"
    yaw_deg: float = 11.5
    pitch_deg: float = 5.7
    timeout: float = 30.0
