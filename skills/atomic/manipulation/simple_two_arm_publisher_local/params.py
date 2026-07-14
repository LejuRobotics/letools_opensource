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
class SimpleTwoArmPublisherLocalParams(SkillParams):
    left_start: List[float] = field(default_factory=lambda: [0.0, 0.4, 0.0, 0.0])
    left_target: List[float] = field(default_factory=lambda: [0.4, 0.4, 0.0, 0.0])
    right_start: List[float] = field(default_factory=lambda: [0.0, -0.4, 0.0, 0.0])
    right_target: List[float] = field(default_factory=lambda: [0.4, -0.4, 0.0, 0.0])
    total_steps: int = 50
    step_duration: float = 0.1
    frame: int = 2
    use_custom_ik_param: bool = True
    ik_param: IKSolveParam = field(default_factory=IKSolveParam)
    timeout: float = 30.0