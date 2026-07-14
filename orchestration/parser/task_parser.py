# kuavo_application_framework/orchestration/parser/task_parser.py

import yaml
from pathlib import Path
from typing import List, Dict, Any, Optional
from core.common.logger import get_logger
from core.common.exceptions import ConfigurationError
from core.common.registry import Registry
from core.domain.skill_params import SkillParams
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

class TaskStep:
    """代表任务中的一个执行步骤"""
    def __init__(self, skill_name: str, params: Dict[str, Any]):
        self.skill_name = skill_name
        self.params = params

class TaskDefinition:
    """代表一个完整的任务定义"""
    def __init__(self, name: str, steps: List[TaskStep]):
        self.name = name
        self.steps = steps

class TaskParser:
    """
    任务解析器。
    负责从 YAML 文件中加载任务配置，并预实例化技能对象。
    """

    @staticmethod
    def load_task(file_path: str, hardware, perception=None) -> TaskDefinition:
        path = Path(file_path)
        if not path.exists():
            raise ConfigurationError(f"Task file not found: {file_path}")

        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        if not data or 'steps' not in data:
            raise ConfigurationError("Invalid task format: missing 'steps'")

        task_name = data.get('name', 'unnamed_task')
        steps_data = data['steps']
        
        instantiated_steps = []
        for step_data in steps_data:
            skill_instance = TaskParser._create_skill_from_config(
                step_data, hardware, perception
            )
            if skill_instance:
                instantiated_steps.append(skill_instance)

        logger.info(f"Task '{task_name}' loaded with {len(instantiated_steps)} steps.")
        return TaskDefinition(name=task_name, steps=instantiated_steps)

    @staticmethod
    def _create_skill_from_config(step_data: Dict, hardware, perception) -> Optional[SkillBase]:
        skill_name = step_data.get('skill')
        if not skill_name:
            logger.warning("Skipping step with no 'skill' defined.")
            return None

        try:
            # 1. 从注册表获取技能类
            SkillClass = Registry.get('skills', skill_name)
            
            # 2. 实例化技能 (注入硬件和感知依赖)
            # 注意：这里假设技能的构造函数接受 hardware 和 perception
            skill_obj = SkillClass(hardware=hardware, perception=perception)
            
            # 3. 处理参数 (将 dict 转换为对应的 SkillParams 子类)
            # 实际项目中可能需要更复杂的映射逻辑，这里简化处理
            params_dict = step_data.get('params', {})
            
            # 暂时先不调用 initialize，留给引擎在执行前调用，或者在这里调用：
            # from core.domain.skill_params import SkillParams
            # params_obj = SkillParams(**params_dict) 
            # skill_obj.initialize(params_obj)
            
            # 为了灵活性，我们把原始参数字典暂存在技能对象中，或者由引擎传递
            skill_obj._raw_params = params_dict 
            
            return skill_obj

        except KeyError:
            logger.error(f"Skill '{skill_name}' not found in registry.")
            return None
        except Exception as e:
            logger.error(f"Failed to create skill '{skill_name}': {e}")
            return None