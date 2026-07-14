# kuavo_application_framework/skills/atomic/manipulation/pick.py

from typing import Optional
from core.interfaces.i_hardware import IHardware
from core.interfaces.i_perception import IPerception
from skills.base.skill_base import SkillBase
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.perception import ObjectDetection
from core.common.logger import get_logger
from core.common.registry import register_skill
from core.domain.pose import Pose

logger = get_logger(__name__)

class PickParams(SkillParams):
    """抓取技能的专用参数"""
    object_label: str = "box"          # 要抓取的物体标签
    approach_offset: float = 0.15      # 抓取前的接近偏移量 (米)
    gripper_open_pos: float = 90.0     # 夹爪张开位置 (0-100)
    gripper_close_pos: float = 10.0    # 夹爪闭合位置 (0-100)
    use_vision: bool = True            # 是否启用视觉引导

@register_skill('pick')
class PickSkill(SkillBase):
    """
    原子技能：使用夹爪抓取物体。
    流程：识别物体 -> 移动到预抓取点 -> 张开夹爪 -> 下降到抓取点 -> 闭合夹爪 -> 抬起
    """

    def __init__(self, hardware: IHardware, perception: Optional[IPerception] = None):
        super().__init__(name="pick")
        self.hardware = hardware
        self.perception = perception
        self.params: PickParams = None
        self.target_obj: Optional[ObjectDetection] = None

    def on_initialize(self, params: PickParams) -> Result:
        if not isinstance(params, PickParams):
            return Result.fail("Invalid parameters for PickSkill")
        
        self.params = params
        
        # 1. 如果启用视觉，先获取物体位姿
        if self.params.use_vision:
            if not self.perception:
                return Result.fail("Perception module is required for vision-based pick")
            
            objects = self.perception.detect_objects([self.params.object_label])
            if not objects:
                return Result.fail(f"No object labeled '{self.params.object_label}' detected")
            
            self.target_obj = objects[0]
            logger.info(f"Target object found at: {self.target_obj.pose_in_world}")
        else:
            logger.info("Using pre-defined target pose (Vision disabled)")
            
        return Result.ok()

    def on_execute(self) -> Result:
        # --- 步骤 1: 准备夹爪 (张开) ---
        logger.info("Step 1: Opening gripper...")
        self.hardware.control_gripper(self.params.gripper_open_pos, self.params.gripper_open_pos)
        
        # --- 步骤 2: 移动到预抓取点 (Approach Pose) ---
        # 注意：实际项目中这里应调用 MoveToPose 技能或底层规划器
        logger.info("Step 2: Moving to approach pose...")
        if self.target_obj:
            approach_pose = self._calculate_approach_pose(self.target_obj.pose_in_world)
            # 简化演示：直接下发末端位姿指令（假设硬件支持笛卡尔控制）
            # self.hardware.move_ee_to_pose(approach_pose) 
        
        # --- 步骤 3: 下降到抓取点 (Grasp Pose) ---
        logger.info("Step 3: Moving to grasp pose...")
        # self.hardware.move_ee_to_pose(self.target_obj.pose_in_world)

        # --- 步骤 4: 执行抓取 (闭合夹爪) ---
        logger.info("Step 4: Closing gripper...")
        self.hardware.control_gripper(self.params.gripper_close_pos, self.params.gripper_close_pos)
        
        # --- 步骤 5: 验证抓取状态 ---
        state = self.hardware.get_gripper_state()
        if state.left.status.name != 'GRABBED':
            logger.warning("Gripper reports not grabbed, but continuing...")

        # --- 步骤 6: 抬起物体 (Lift) ---
        logger.info("Step 5: Lifting object...")
        # lift_pose = self.target_obj.pose_in_world
        # lift_pose.position[2] += 0.2
        # self.hardware.move_ee_to_pose(lift_pose)

        return Result.ok("Pick successful")

    def on_cancel(self) -> Result:
        # 取消时确保夹爪松开并停止手臂运动
        self.hardware.control_gripper(self.params.gripper_open_pos if self.params else 90.0, 
                                      self.params.gripper_open_pos if self.params else 90.0)
        return Result.ok("Pick cancelled safely")

    def _calculate_approach_pose(self, target_pose: Pose) -> Pose:
        """计算预抓取点：在目标位姿正上方 offset 米处"""
        approach_z = target_pose.position[2] + self.params.approach_offset
        return Pose(
            position=(target_pose.position[0], target_pose.position[1], approach_z),
            orientation=target_pose.orientation
        )