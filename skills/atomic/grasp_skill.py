# LeTools/skills/atomic/grasp_skill.py
from typing import Optional
from core.interfaces.i_hardware import IHardware
from core.domain.result import Result
from core.domain.enums import ArmSide
from core.domain.end_effector import GripperCommand, HandFingerCommand
from core.common.logger import get_logger

logger = get_logger(__name__)

class GraspSkill:
    """
    原子抓取技能。
    负责协调末端执行器完成“张开”和“闭合”动作。
    """
    def __init__(self, hardware: IHardware):
        self.hardware = hardware

    def execute_grasp(self, side: ArmSide = ArmSide.RIGHT, force: float = 1.0) -> Result:
        """
        执行抓取动作（闭合夹爪）。
        :param side: 左臂或右臂
        :param force: 抓取力度 (A)
        """
        if not self.hardware.is_connected:
            return Result.fail("Hardware not connected")

        logger.info(f"Executing grasp on {side.value} arm with force {force}A...")
        
        # 1. 构建夹爪指令：位置 100 (完全闭合), 速度 50, 指定力度
        cmd = GripperCommand(position=100.0, velocity=50.0, effort=force)
        
        # 2. 调用适配器层接口
        result = self.hardware.control_end_effector(side, cmd)
        
        if result.success:
            logger.info("Grasp command sent successfully.")
        else:
            logger.error(f"Grasp failed: {result.message}")
            
        return result

    def execute_release(self, side: ArmSide = ArmSide.RIGHT) -> Result:
        """
        执行释放动作（张开夹爪）。
        :param side: 左臂或右臂
        """
        if not self.hardware.is_connected:
            return Result.fail("Hardware not connected")

        logger.info(f"Executing release on {side.value} arm...")
        
        # 1. 构建夹爪指令：位置 0 (完全张开)
        cmd = GripperCommand(position=0.0, velocity=80.0, effort=0.5)
        
        # 2. 调用适配器层接口
        result = self.hardware.control_end_effector(side, cmd)
        
        if result.success:
            logger.info("Release command sent successfully.")
        else:
            logger.error(f"Release failed: {result.message}")
            
        return result
