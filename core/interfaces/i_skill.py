# kuavo_application_framework/core/interfaces/i_skill.py
from abc import ABC, abstractmethod
from ..domain.result import Result
from ..domain.skill_params import SkillParams

class ISkill(ABC):
    """
    所有原子技能和组合技能的根接口。
    定义了技能的标准生命周期：初始化 -> 执行 -> (可选)取消。
    """
    
    @property
    @abstractmethod
    def name(self) -> str:
        """技能的唯一标识符，用于注册和查找"""
        pass

    @abstractmethod
    def initialize(self, params: SkillParams) -> Result:
        """
        技能初始化。
        :param params: 技能执行所需的参数配置
        """
        pass

    @abstractmethod
    def execute(self) -> Result:
        """
        执行技能的核心逻辑。
        :return: 执行结果（成功/失败/运行中）
        """
        pass

    @abstractmethod
    def cancel(self) -> Result:
        """
        中断技能执行。
        通常在急停或任务切换时调用，需确保机器人进入安全状态。
        """
        pass

    @abstractmethod
    def is_finished(self) -> bool:
        """检查技能是否已结束（无论成功或失败）"""
        pass