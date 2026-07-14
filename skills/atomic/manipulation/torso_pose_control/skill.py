from typing import List, Dict
from dataclasses import dataclass, field
from core.interfaces.i_skill import ISkill
from core.domain.result import Result
from core.common.logger import get_logger
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware

logger = get_logger(__name__)

@dataclass
class TorsoPoseControlParams:
    timeout: float = 30.0
    initial_torso_pose: List[float] = field(default_factory=lambda: [0.196123, 0.0005, 0.789919])
    pose_list: List[Dict[str, List[float]]] = field(default_factory=lambda: [
        {'linear': [0.0, 0.0, 0.4], 'angular': [0.0, 0.0, 0.0]},
        {'linear': [0.2, 0.0, 0.4], 'angular': [0.0, 0.0, 0.0]},
        {'linear': [0.2, 0.0, 0.4], 'angular': [0.0, 0.0, 60.0]},
        {'linear': [0.2, 0.0, 0.4], 'angular': [0.0, 0.0, -60.0]},
        {'linear': [0.2, 0.0, 0.4], 'angular': [0.0, -30.0, 0.0]},
        {'linear': [0.2, 0.0, 0.4], 'angular': [0.0, 30.0, 0.0]},
        {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0]}
    ])
    wait_time_offset: float = 0.5

class TorsoPoseControlSkill(ISkill):
    def __init__(self, hardware: LejuWheeledArmHardware):
        self.hardware = hardware
        self._params: TorsoPoseControlParams = None
        self._is_finished = False
        self._is_canceled = False
    
    @property
    def name(self) -> str:
        return "torso_pose_control"
    
    def initialize(self, params: TorsoPoseControlParams) -> Result:
        """初始化技能"""
        self._params = params
        self._is_finished = False
        self._is_canceled = False
        logger.info(f"Initialized TorsoPoseControlSkill with {len(params.pose_list)} poses")
        return Result.ok()
    
    def execute(self) -> Result:
        """执行技能主逻辑"""
        if self._is_canceled:
            return Result.fail("Skill has been canceled")
        
        if self._is_finished:
            return Result.ok("Torso pose control already finished")
        
        try:
            initial = self._params.initial_torso_pose
            
            for pose in self._params.pose_list:
                if self._is_canceled:
                    return Result.fail("Skill canceled during execution")
                
                offset_linear = pose['linear']
                angular = pose['angular']
                
                actual_linear = [
                    initial[0] + offset_linear[0],
                    initial[1] + offset_linear[1],
                    initial[2] + offset_linear[2]
                ]
                
                print(f"Published: [{offset_linear[0]}, {offset_linear[1]}, {offset_linear[2]}, {angular[0]}, {angular[1]}, {angular[2]}]")
                
                result = self.send_torso_pose(actual_linear, angular)
                
                if not result.success:
                    logger.error(f"Failed to send torso pose: {result.message}")
                    return result
            
            self._is_finished = True
            return Result.ok("Torso pose control completed successfully")
        
        except Exception as e:
            logger.error(f"Failed to execute torso pose control: {str(e)}")
            return Result.fail(f"Failed to execute torso pose control: {str(e)}")
    
    def send_torso_pose(self, linear: List[float], angular: List[float]) -> Result:
        """
        发送躯干位姿指令
        :param linear: 线性位置 [x, y, z]
        :param angular: 角度 [roll, pitch, yaw]（度）
        :return: Result，包含到达时间或错误信息
        """
        return self.hardware.send_torso_pose(linear, angular)
    
    def cancel(self) -> Result:
        """取消技能执行"""
        self._is_canceled = True
        logger.info("TorsoPoseControlSkill canceled")
        return Result.ok("Skill canceled successfully")
    
    def is_finished(self) -> bool:
        """检查技能是否完成"""
        return self._is_finished
    
    @property
    def params(self) -> TorsoPoseControlParams:
        """获取技能参数"""
        return self._params