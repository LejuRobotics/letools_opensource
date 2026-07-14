from typing import List
from dataclasses import dataclass, field
from core.domain.skill_params import SkillParams


@dataclass
class IKSolveParam:
    major_optimality_tol: float = 1e-6
    major_feasibility_tol: float = 1e-6
    minor_feasibility_tol: float = 1e-6
    major_iterations_limit: int = 1000
    oritation_constraint_tol: float = 0.01
    pos_constraint_tol: float = 0.01
    pos_cost_weight: float = 1.0


@dataclass
class TwoArmHandPoseControlParams(SkillParams):
    left_target: List[float] = field(default_factory=lambda: [0.4, 0.150, 0.65, 0.0, -90.0, 0.0])
    right_target: List[float] = field(default_factory=lambda: [0.4, -0.150, 0.65, 0.0, -90.0, 0.0])
    frame: int = 1
    use_custom_ik_param: bool = True
    ik_param: IKSolveParam = field(default_factory=IKSolveParam)
    timeout: float = 30.0