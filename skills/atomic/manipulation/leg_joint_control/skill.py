from typing import List
from dataclasses import dataclass, field
from core.interfaces.i_skill import ISkill
from core.domain.result import Result
from core.common.logger import get_logger
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware

logger = get_logger(__name__)

@dataclass
class LegJointControlParams:
    timeout: float = 30.0
    joint_names: List[str] = field(default_factory=lambda: ['joint1', 'joint2', 'joint3', 'joint4'])
    joint_positions_list: List[List[float]] = field(default_factory=lambda: [[14.90, -32.01, 18.03, 0.0], [14.90, -32.01, 18.03, 90.0], [0.0, 0.0, 0.0, 0.0]])
    wait_time_offset: float = 0.5

class LegJointControlSkill(ISkill):
    def __init__(self, hardware: LejuWheeledArmHardware):
        self.hardware = hardware
        self._params: LegJointControlParams = None
        self._is_finished = False
        self._is_canceled = False
    
    @property
    def name(self) -> str:
        return "leg_joint_control"
    
    def initialize(self, params: LegJointControlParams) -> Result:
        """初始化技能"""
        self._params = params
        self._is_finished = False
        self._is_canceled = False
        logger.info(f"Initialized LegJointControlSkill with {len(params.joint_names)} joints")
        return Result.ok()
    
    def execute(self) -> Result:
        """执行技能主逻辑"""
        if self._is_canceled:
            return Result.fail("Skill has been canceled")
        
        if self._is_finished:
            return Result.ok("Leg joint control already finished")
        
        try:
            for i, positions in enumerate(self._params.joint_positions_list):
                if self._is_canceled:
                    return Result.fail("Skill canceled during execution")
                
                print(f"\n发布测试数据{i+1}:")
                print(f"  关节角度: {positions}")
                
                result = self.send_joint_positions(self._params.joint_names, positions)
                
                if not result.success:
                    logger.error(f"Failed to send joint positions: {result.message}")
                    return result
                
                print(f"  到达时间: {result.message.split('reach time: ')[-1] if 'reach time:' in result.message else 'unknown'}")
            
            print("\n测试数据发布完成！")
            self._is_finished = True
            return Result.ok("Leg joint control completed successfully")
        
        except Exception as e:
            logger.error(f"Failed to execute leg joint control: {str(e)}")
            return Result.fail(f"Failed to execute leg joint control: {str(e)}")
    
    def send_joint_positions(self, joint_names: List[str], positions: List[float]) -> Result:
        """
        发送单个关节位置
        :param joint_names: 关节名称列表
        :param positions: 关节位置列表
        :return: Result，包含到达时间或错误信息
        """
        return self.hardware.send_leg_joint_positions(joint_names, positions)
    
    def cancel(self) -> Result:
        """取消技能执行"""
        self._is_canceled = True
        logger.info("LegJointControlSkill canceled")
        return Result.ok("Skill canceled successfully")
    
    def is_finished(self) -> bool:
        """检查技能是否完成"""
        return self._is_finished
    
    @property
    def params(self) -> LegJointControlParams:
        """获取技能参数"""
        return self._params