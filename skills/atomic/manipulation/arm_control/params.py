from dataclasses import dataclass, field
from typing import List

from core.domain.skill_params import SkillParams


@dataclass
class ArmControlParams(SkillParams):
    """手臂关节轨迹（度），对齐 test_arm_joint.py test_01_spread_arms。"""

    skill_name: str = "arm_control"
    joint_angles_deg: List[float] = field(
        default_factory=lambda: [
            -30.0,
            20.0,
            15.0,
            -45.0,
            25.0,
            10.0,
            -35.0,
            -30.0,
            -20.0,
            -15.0,
            -45.0,
            -25.0,
            -10.0,
            -35.0,
        ]
    )
    time_sec: float = 3.0
    enable_quick_mode: bool = True
    timeout: float = 30.0
