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
class TwoArmHandPoseControlParams:
    left_target: List[float] = field(default_factory=lambda: [0.4, 0.150, 0.65, 0.0, -90.0, 0.0])
    right_target: List[float] = field(default_factory=lambda: [0.4, -0.150, 0.65, 0.0, -90.0, 0.0])
    frame: int = 1
    use_custom_ik_param: bool = True
    ik_param: IKSolveParam = field(default_factory=IKSolveParam)
    timeout: float = 30.0


@register_skill('two_arm_hand_pose_control')
class TwoArmHandPoseControlSkill(ISkill):
    def __init__(self, hardware: IHardware):
        self.hardware = hardware
        self.params: TwoArmHandPoseControlParams = None
        self._is_finished = False

    @property
    def name(self) -> str:
        return "two_arm_hand_pose_control"

    def initialize(self, params: TwoArmHandPoseControlParams) -> Result:
        if not isinstance(params, TwoArmHandPoseControlParams):
            return Result.fail("Invalid parameter type for TwoArmHandPoseControlSkill")
        
        self.params = params
        self._is_finished = False
        logger.info(f"Initialized TwoArmHandPoseControlSkill")
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

    def slerp_quaternion(self, q1, q2, t):
        dot = np.dot(q1, q2)
        
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        theta_0 = np.arccos(np.abs(dot))
        sin_theta_0 = np.sin(theta_0)
        
        theta = theta_0 * t
        s1 = np.sin(theta) / sin_theta_0
        s0 = np.cos(theta) - dot * s1
        
        return s0 * q1 + s1 * q2

    def interpolate_pose(self, start_pose, target_pose, t):
        pos = [
            start_pose[0] + (target_pose[0] - start_pose[0]) * t,
            start_pose[1] + (target_pose[1] - start_pose[1]) * t,
            start_pose[2] + (target_pose[2] - start_pose[2]) * t
        ]
        
        start_quat = tf_trans.quaternion_from_euler(
            np.radians(start_pose[3]), np.radians(start_pose[4]), np.radians(start_pose[5])
        )
        target_quat = tf_trans.quaternion_from_euler(
            np.radians(target_pose[3]), np.radians(target_pose[4]), np.radians(target_pose[5])
        )
        
        interpolated_quat = self.slerp_quaternion(start_quat, target_quat, t)
        
        roll, pitch, yaw = tf_trans.euler_from_quaternion(interpolated_quat)
        
        return pos + [np.degrees(roll), np.degrees(pitch), np.degrees(yaw)]

    def execute(self) -> Result:
        if self._is_finished:
            return Result.ok("Task already finished")

        try:
            left_target = self.params.left_target
            right_target = self.params.right_target
            
            logger.info("发送双臂目标位姿...")
            logger.info(f"左手目标: {left_target[:3]}, RPY=[{left_target[3]:.1f}°, {left_target[4]:.1f}°, {left_target[5]:.1f}°]")
            logger.info(f"右手目标: {right_target[:3]}, RPY=[{right_target[3]:.1f}°, {right_target[4]:.1f}°, {right_target[5]:.1f}°]")
            
            result = self.hardware.send_two_arm_hand_pose(
                left_target,
                right_target,
                self.params.frame,
                self.params.use_custom_ik_param,
                self.create_default_ik_param()
            )
            
            if result.success:
                logger.info("目标位姿发送完成！")
                logger.info("机器人双臂应该开始移动到目标位置")
                self._is_finished = True
                return Result.ok("Two arm hand pose control executed successfully")
            else:
                return result
        except Exception as e:
            import traceback
            logger.error(f"Failed to execute two arm hand pose control: {str(e)}")
            logger.error(f"Traceback: {traceback.format_exc()}")
            return Result.fail(f"Failed to execute two arm hand pose control: {str(e)}")

    def cancel(self) -> Result:
        logger.warning("TwoArmHandPoseControlSkill cancelled!")
        self._is_finished = True
        return Result.ok("Cancelled")

    def is_finished(self) -> bool:
        return self._is_finished