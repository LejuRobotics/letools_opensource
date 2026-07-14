import numpy as np
from typing import List
from dataclasses import dataclass
from core.interfaces.i_skill import ISkill
from core.interfaces.i_hardware import IHardware
from core.domain.result import Result
from core.common.logger import get_logger
from core.common.registry import register_skill

logger = get_logger(__name__)

@dataclass
class TwoArmJointSweepParams:
    rate_hz: int = 20
    num_joints: int = 7
    target_idx: int = 3
    start_deg: float = 0.0
    end_deg: float = 90.0
    period_sec: float = 4.0

@register_skill('two_arm_joint_sweep')
class TwoArmJointSweepSkill(ISkill):
    @property
    def name(self) -> str:
        return 'two_arm_joint_sweep'
    
    def __init__(self, hardware: IHardware):
        self.hardware = hardware
        self._params = None
        self._is_initialized = False
        self._is_running = False

    def initialize(self, params: TwoArmJointSweepParams) -> Result:
        self._params = params
        self._is_initialized = True
        logger.info(f"Initializing TwoArmJointSweepSkill with params: rate_hz={params.rate_hz}, num_joints={params.num_joints}, "
                   f"target_idx={params.target_idx}, start_deg={params.start_deg}, end_deg={params.end_deg}, "
                   f"period_sec={params.period_sec}")
        return Result.ok("TwoArmJointSweepSkill initialized")

    def execute(self) -> Result:
        if not self._is_initialized:
            return Result.fail("Skill not initialized")
        
        import rospy
        
        if not rospy.core.is_initialized():
            rospy.init_node('two_arm_hand_pose_cmd_sweeper', anonymous=True)
        
        self._is_running = True
        rate = rospy.Rate(self._params.rate_hz)
        
        period_sec = self._params.period_sec
        half_period = max(1e-3, period_sec / 2.0)
        amplitude = (self._params.end_deg - self._params.start_deg)
        
        logger.info(f"Publishing to /mm/two_arm_hand_pose_cmd with Frame=joint Space")
        logger.info(f"Sweeping joint {self._params.target_idx} from {self._params.start_deg:.1f} deg to {self._params.end_deg:.1f} deg")
        
        t0 = rospy.Time.now().to_sec()
        
        try:
            while self._is_running and not rospy.is_shutdown():
                t = rospy.Time.now().to_sec() - t0
                phase = t % max(1e-3, period_sec)
                
                if phase < half_period:
                    val_deg = self._params.start_deg + amplitude * (phase / half_period)
                else:
                    val_deg = self._params.end_deg - amplitude * ((phase - half_period) / half_period)
                
                left_j = [0.0] * self._params.num_joints
                right_j = [0.0] * self._params.num_joints
                
                if 0 <= self._params.target_idx < self._params.num_joints:
                    left_j[self._params.target_idx] = float(-val_deg)
                    right_j[self._params.target_idx] = float(-val_deg)
                
                result = self.hardware.send_two_arm_joint_command(
                    left_j, right_j, 
                    frame=5,
                    use_custom_ik_param=False
                )
                
                if not result.success:
                    logger.warn(f"Failed to send joint command: {result.message}")
                
                rate.sleep()
            
            return Result.ok("Joint sweep completed successfully")
        except rospy.ROSInterruptException:
            return Result.ok("Joint sweep interrupted")

    def cancel(self) -> Result:
        self._is_running = False
        return Result.ok("Joint sweep cancelled")

    def is_finished(self) -> bool:
        return not self._is_running