from typing import Dict, Any, Optional
from core.interfaces import ISkill, IHardware
from core.domain import Result
from core.domain.skill_params import SkillParams

class PlaceSkill(ISkill):
    def __init__(self, hardware: IHardware, config: Optional[Dict] = None):
        self._hardware = hardware
        self._config = config or {}
        self._name = "place"
        self._is_finished = False
    
    @property
    def name(self) -> str:
        return self._name
    
    def initialize(self, params: SkillParams) -> Result:
        self._is_finished = False
        return Result.ok()
    
    def execute(self) -> Result:
        result = self._hardware.control_gripper(100, 100)  # 假设 100 是打开状态
        self._is_finished = True
        return result
    
    def cancel(self) -> Result:
        self._is_finished = True
        return Result.ok("Cancelled")
    
    def is_finished(self) -> bool:
        return self._is_finished
