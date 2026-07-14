# kuavo_application_framework/orchestration/engine/behavior_tree_engine.py

import time
from typing import List, Optional
from core.common.logger import get_logger
from core.domain.result import Result
from orchestration.parser.task_parser import TaskDefinition
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

class BehaviorTreeEngine:
    """
    轻量级行为树引擎。
    负责按顺序执行任务步骤，并监控执行状态。
    """

    def __init__(self):
        self.current_task: Optional[TaskDefinition] = None
        self.current_step_index: int = 0
        self.is_running: bool = False
        self._current_skill: Optional[SkillBase] = None

    def load_task(self, task_def: TaskDefinition) -> Result:
        """加载一个已解析的任务定义"""
        if not task_def.steps:
            return Result.fail("Task has no steps to execute")
        
        self.current_task = task_def
        self.current_step_index = 0
        self.is_running = False
        logger.info(f"Task '{task_def.name}' loaded into engine.")
        return Result.ok()

    def start(self) -> Result:
        """启动任务执行"""
        if not self.current_task:
            return Result.fail("No task loaded")
        
        self.is_running = True
        logger.info(f"Starting task: {self.current_task.name}")
        return Result.ok()

    def tick(self) -> Result:
        """
        引擎的主循环驱动函数 (Tick)。
        应用层应以固定频率（如 50Hz）调用此函数。
        """
        if not self.is_running:
            return Result.fail("Engine is not running")

        if self.current_step_index >= len(self.current_task.steps):
            self.is_running = False
            logger.info("All steps completed successfully.")
            return Result.ok("Task Finished")

        # 获取当前步骤的技能实例
        skill = self.current_task.steps[self.current_step_index]
        
        # 如果是新步骤，先进行初始化
        if self._current_skill != skill:
            logger.info(f"Initializing step {self.current_step_index + 1}: {skill.name}")
            # 这里将原始参数字典转换为 SkillParams 对象传给技能
            from core.domain.skill_params import SkillParams
            params = SkillParams(**getattr(skill, '_raw_params', {}))
            init_res = skill.initialize(params)
            
            if not init_res.success:
                logger.error(f"Step initialization failed: {init_res.message}")
                self.is_running = False
                return Result.fail(f"Init failed at step {self.current_step_index + 1}")
            
            self._current_skill = skill

        # 执行技能的逻辑
        exec_res = skill.execute()

        # 根据执行结果决定下一步
        if exec_res.success:
            # 检查技能是否真正结束（针对非阻塞技能）
            if skill.is_finished():
                logger.info(f"Step {self.current_step_index + 1} finished.")
                self.current_step_index += 1
                self._current_skill = None # 重置以便下一个技能初始化
                return Result.ok("Step Completed")
            else:
                return Result.ok("Executing...") # 技能还在运行中
        else:
            # 执行失败
            logger.error(f"Step execution failed: {exec_res.message}")
            self.is_running = False
            skill.cancel() # 尝试取消当前技能以进入安全状态
            return Result.fail(exec_res.message)

    def cancel(self) -> Result:
        """紧急中断当前任务"""
        logger.warning("Engine received CANCEL command!")
        self.is_running = False
        if self._current_skill:
            self._current_skill.cancel()
        return Result.ok("Task Cancelled")

    def get_progress(self) -> float:
        """获取任务完成进度 (0.0 - 1.0)"""
        if not self.current_task or not self.current_task.steps:
            return 0.0
        return self.current_step_index / len(self.current_task.steps)

    def execute(self, task_config: dict) -> Result:
        """执行任务配置"""
        try:
            # 创建硬件实例
            from adapters.hardware.factory import HardwareFactory
            hardware_config = task_config.get('hardware', {})
            hardware = HardwareFactory.create_hardware(hardware_config)
            
            # 初始化硬件
            hardware.initialize()
            
            # 加载任务
            from orchestration.parser.task_parser import TaskParser
            # 注意：这里我们假设任务配置中包含 steps 字段
            # 实际项目中可能需要从文件加载
            import tempfile
            import yaml
            
            # 将任务配置写入临时文件
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
                yaml.dump(task_config, f)
                temp_file_path = f.name
            
            # 加载任务
            task_def = TaskParser.load_task(temp_file_path, hardware)
            
            # 清理临时文件
            import os
            os.unlink(temp_file_path)
            
            # 加载任务到引擎
            load_result = self.load_task(task_def)
            if not load_result.success:
                return load_result
            
            # 启动任务
            start_result = self.start()
            if not start_result.success:
                return start_result
            
            # 执行任务直到完成
            while self.is_running:
                result = self.tick()
                if not result.success:
                    return result
                # 短暂休眠，避免占用过多CPU
                time.sleep(0.01)
            
            # 关闭硬件
            hardware.shutdown()
            
            return Result.ok("Task executed successfully")
        except Exception as e:
            logger.error(f"Failed to execute task: {str(e)}")
            return Result.fail(f"Failed to execute task: {str(e)}")