# kuavo_application_framework/skills/base/skill_base.py

import time
from core.interfaces.i_skill import ISkill
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.common.logger import get_logger

logger = get_logger(__name__)

class SkillBase(ISkill):
    """
    技能通用基类。
    提供了超时监控、日志记录和异常处理的默认实现。
    具体的原子技能应继承此类。
    """
    
    def __init__(self, name: str):
        self._name = name
        self._start_time = 0
        self._params: SkillParams = None

    @property
    def name(self) -> str:
        return self._name

    def initialize(self, params: SkillParams) -> Result:
        self._params = params
        self._start_time = time.time()
        logger.info(f"Skill [{self.name}] initialized.")
        return self.on_initialize(params)

    def execute(self) -> Result:
        # 1. 超时检查
        if time.time() - self._start_time > self._params.timeout:
            logger.error(f"Skill [{self.name}] timed out.")
            return Result.fail("Timeout")
        
        # 2. 执行具体逻辑
        try:
            return self.on_execute()
        except Exception as e:
            logger.exception(f"Skill [{self.name}] execution error: {e}")
            return Result.fail(str(e))

    def cancel(self) -> Result:
        logger.warning(f"Skill [{self.name}] cancelled.")
        return self.on_cancel()

    def is_finished(self) -> bool:
        return self.on_is_finished()

    # --- 子类需要实现的钩子方法 ---
    def on_initialize(self, params: SkillParams) -> Result:
        return Result.ok()

    def on_execute(self) -> Result:
        raise NotImplementedError

    def on_cancel(self) -> Result:
        return Result.ok()

    def on_is_finished(self) -> bool:
        return False