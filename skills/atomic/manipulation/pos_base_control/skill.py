from typing import List, Dict
from dataclasses import dataclass, field
from core.interfaces.i_skill import ISkill
from core.domain.result import Result
from core.common.logger import get_logger
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware

logger = get_logger(__name__)

@dataclass
class PosBaseControlParams:
    timeout: float = 30.0
    pose_list: List[Dict[str, float]] = field(default_factory=lambda: [
        {'x': 0.0, 'y': 2.0, 'yaw': 1.57},
        {'x': 0.0, 'y': 2.0, 'yaw': 1.57},
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    ])
    wait_time_offset: float = 0.5

class PosBaseControlSkill(ISkill):
    def __init__(self, hardware: LejuWheeledArmHardware):
        self.hardware = hardware
        self._params: PosBaseControlParams = None
        self._is_finished = False
        self._is_canceled = False
    
    @property
    def name(self) -> str:
        return "pos_base_control"
    
    def initialize(self, params: PosBaseControlParams) -> Result:
        """初始化技能"""
        self._params = params
        self._is_finished = False
        self._is_canceled = False
        logger.info(f"Initialized PosBaseControlSkill with {len(params.pose_list)} poses")
        return Result.ok()
    
    def execute(self) -> Result:
        """执行技能主逻辑"""
        if self._is_canceled:
            return Result.fail("Skill has been canceled")
        
        if self._is_finished:
            return Result.ok("Base position control already finished")
        
        try:
            for i, pose in enumerate(self._params.pose_list):
                if self._is_canceled:
                    return Result.fail("Skill canceled during execution")
                
                x = pose['x']
                y = pose['y']
                yaw = pose['yaw']
                
                print(f"\n发布测试数据{i+1}:")
                print(f"  位置: ({x}, {y})")
                print(f"  偏航角: {yaw}")
                
                result = self.send_base_position(x, y, yaw)
                
                if not result.success:
                    logger.error(f"Failed to send base position: {result.message}")
                    return result
                
                print(f"  到达时间: {result.message.split('reach time: ')[-1] if 'reach time:' in result.message else 'unknown'}")
            
            print("\n测试数据发布完成！请检查C++程序的输出。")
            self._is_finished = True
            return Result.ok("Base position control completed successfully")
        
        except Exception as e:
            logger.error(f"Failed to execute base position control: {str(e)}")
            return Result.fail(f"Failed to execute base position control: {str(e)}")
    
    def send_base_position(self, x: float, y: float, yaw: float) -> Result:
        """
        发送底盘位置指令
        :param x: X 位置（米）
        :param y: Y 位置（米）
        :param yaw: 偏航角（弧度）
        :return: Result，包含到达时间或错误信息
        """
        return self.hardware.send_base_position(x, y, yaw)
    
    def cancel(self) -> Result:
        """取消技能执行"""
        self._is_canceled = True
        logger.info("PosBaseControlSkill canceled")
        return Result.ok("Skill canceled successfully")
    
    def is_finished(self) -> bool:
        """检查技能是否完成"""
        return self._is_finished
    
    @property
    def params(self) -> PosBaseControlParams:
        """获取技能参数"""
        return self._params