import numpy as np
import tf.transformations as tf_trans
from typing import List
from dataclasses import dataclass, field
from core.interfaces.i_skill import ISkill
from core.interfaces.i_hardware import IHardware
from core.domain.result import Result
from core.common.logger import get_logger
from core.common.registry import register_skill

logger = get_logger(__name__)


@dataclass
class IKSolveParam:
    major_optimality_tol: float = 1e-6
    major_feasibility_tol: float = 1e-6
    minor_feasibility_tol: float = 1e-6
    major_iterations_limit: int = 1000
    oritation_constraint_tol: float = 0.01
    pos_constraint_tol: float = 0.01
    pos_cost_weight: float = 1.0


@dataclass
class SimpleTwoArmPublisherLocalParams:
    left_start: List[float] = field(default_factory=lambda: [0.0, 0.4, 0.0, 0.0])
    left_target: List[float] = field(default_factory=lambda: [0.4, 0.4, 0.0, 0.0])
    right_start: List[float] = field(default_factory=lambda: [0.0, -0.4, 0.0, 0.0])
    right_target: List[float] = field(default_factory=lambda: [0.4, -0.4, 0.0, 0.0])
    total_steps: int = 50
    step_duration: float = 0.1
    frame: int = 2
    use_custom_ik_param: bool = True
    ik_param: IKSolveParam = field(default_factory=IKSolveParam)
    timeout: float = 30.0


@register_skill('simple_two_arm_publisher_local')
class SimpleTwoArmPublisherLocalSkill(ISkill):
    def __init__(self, hardware: IHardware):
        self.hardware = hardware
        self.params: SimpleTwoArmPublisherLocalParams = None
        self._is_finished = False

    @property
    def name(self) -> str:
        return "simple_two_arm_publisher_local"

    def initialize(self, params: SimpleTwoArmPublisherLocalParams) -> Result:
        if not isinstance(params, SimpleTwoArmPublisherLocalParams):
            return Result.fail("Invalid parameter type for SimpleTwoArmPublisherLocalSkill")
        
        self.params = params
        self._is_finished = False
        logger.info(f"Initialized SimpleTwoArmPublisherLocalSkill")
        return Result.ok()

    def create_simple_pose(self, x, y, z, roll=0, pitch=0, yaw=0):
        from kuavo_msgs.msg import armHandPose
        
        pose = armHandPose()
        pose.pos_xyz = [x, y, z]
        
        quat = tf_trans.quaternion_from_euler(
            np.radians(roll),
            np.radians(pitch),
            np.radians(yaw)
        )
        
        quat_norm = np.linalg.norm(quat)
        if quat_norm > 1e-8:
            quat_normalized = quat / quat_norm
        else:
            quat_normalized = np.array([0.0, 0.0, 0.0, 1.0])
        
        pose.quat_xyzw = [quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]]
        pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        pose.joint_angles = [0.0] * 7
        
        return pose

    def create_default_ik_param(self):
        from kuavo_msgs.msg import ikSolveParam
        
        param = ikSolveParam()
        param.major_optimality_tol = self.params.ik_param.major_optimality_tol
        param.major_feasibility_tol = self.params.ik_param.major_feasibility_tol
        param.minor_feasibility_tol = self.params.ik_param.minor_feasibility_tol
        param.major_iterations_limit = self.params.ik_param.major_iterations_limit
        param.oritation_constraint_tol = self.params.ik_param.oritation_constraint_tol
        param.pos_constraint_tol = self.params.ik_param.pos_constraint_tol
        param.pos_cost_weight = self.params.ik_param.pos_cost_weight
        return param

    def interpolate_pose(self, start_pose, target_pose, t):
        pos = [
            start_pose[0] + (target_pose[0] - start_pose[0]) * t,
            start_pose[1] + (target_pose[1] - start_pose[1]) * t,
            start_pose[2] + (target_pose[2] - start_pose[2]) * t
        ]
        
        yaw = start_pose[3] + (target_pose[3] - start_pose[3]) * t
        
        return pos + [yaw]

    def execute(self) -> Result:
        if self._is_finished:
            return Result.ok("Task already finished")

        try:
            import rospy
            
            left_start = self.params.left_start
            left_target = self.params.left_target
            right_start = self.params.right_start
            right_target = self.params.right_target
            total_steps = self.params.total_steps
            step_duration = self.params.step_duration
            
            logger.info("开始双臂位姿插值发布...")
            logger.info(f"插值步数: {total_steps} 步")
            logger.info(f"每步时长: {step_duration} 秒")
            logger.info(f"总时长: {total_steps * step_duration} 秒")
            logger.info("左手目标: [1.44, 2.0, 0.8], yaw=30°")
            logger.info("右手目标: [0.56, 2.0, 0.8], yaw=30°")
            
            rate = rospy.Rate(10)
            
            for step in range(total_steps + 1):
                if rospy.is_shutdown():
                    break
                
                t = step / total_steps
                
                left_current = self.interpolate_pose(left_start, left_target, t)
                right_current = self.interpolate_pose(right_start, right_target, t)
                
                result = self.hardware.send_two_arm_hand_pose(
                    [left_current[0], left_current[1], left_current[2], 0, 0, left_current[3]],
                    [right_current[0], right_current[1], right_current[2], 0, 0, right_current[3]],
                    self.params.frame,
                    self.params.use_custom_ik_param,
                    self.create_default_ik_param()
                )
                
                if not result.success:
                    return result
                
                if step % 10 == 0 or step == total_steps:
                    logger.info(f"步骤 {step}/{total_steps} (t={t:.2f})")
                    logger.info(f"  左手: [{left_current[0]:.2f}, {left_current[1]:.2f}, {left_current[2]:.2f}], yaw={left_current[3]:.1f}°")
                    logger.info(f"  右手: [{right_current[0]:.2f}, {right_current[1]:.2f}, {right_current[2]:.2f}], yaw={right_current[3]:.1f}°")
                
                rate.sleep()
            
            logger.info("插值发布完成！")
            logger.info("机器人应该已经到达目标位置")
            
            self._is_finished = True
            return Result.ok("Simple two arm publisher local executed successfully")
        except Exception as e:
            import traceback
            logger.error(f"Failed to execute simple two arm publisher local: {str(e)}")
            logger.error(f"Traceback: {traceback.format_exc()}")
            return Result.fail(f"Failed to execute simple two arm publisher local: {str(e)}")

    def cancel(self) -> Result:
        logger.warning("SimpleTwoArmPublisherLocalSkill cancelled!")
        self._is_finished = True
        return Result.ok("Cancelled")

    def is_finished(self) -> bool:
        return self._is_finished