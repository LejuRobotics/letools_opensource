from dataclasses import dataclass, field
from typing import List

from core.domain.skill_params import SkillParams


@dataclass
class LegControlParams(SkillParams):
    """腿部关节控制（4 自由度，度）。默认与 test_leg_joint 初始姿态一致。"""

    skill_name: str = "leg_control"
    joint_angles_deg: List[float] = field(
        default_factory=lambda: [14.90, -32.01, 18.03, 30.0]
    )
    timeout: float = 30.0
