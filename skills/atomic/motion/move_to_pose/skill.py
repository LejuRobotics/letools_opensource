# kuavo_application_framework/skills/atomic/motion/move_to_pose/skill.py

import time
from core.interfaces.i_skill import ISkill
from core.interfaces.i_motion_skill import IMotionSkill
from core.interfaces.i_hardware import IHardware
from core.domain.result import Result
from core.domain.skill_params import MoveToPoseParams
from core.domain.pose import Pose
from core.common.logger import get_logger
from core.common.math_utils import calculate_distance, is_pose_reached
from core.common.registry import register_skill

logger = get_logger(__name__)

@register_skill('move_to_pose')
class MoveToPoseSkill(IMotionSkill):
    """
    移动到指定位姿的原子技能。
    适用于底盘移动或全身位姿调整。
    """

    def __init__(self, hardware: IHardware):
        self.hardware = hardware
        self.params: MoveToPoseParams = None
        self._is_finished = False
        self._start_time = 0

    @property
    def name(self) -> str:
        return "move_to_pose"

    def initialize(self, params: MoveToPoseParams) -> Result:
        """初始化技能参数"""
        if not isinstance(params, MoveToPoseParams):
            return Result.fail("Invalid parameter type for MoveToPoseSkill")
        
        self.params = params
        self._is_finished = False
        self._start_time = time.time()
        
        logger.info(f"Initialized MoveToPoseSkill with target: {params.target_pose}")
        return Result.ok()

    def execute(self) -> Result:
        """
        执行移动逻辑。
        这是一个非阻塞式的执行示例，通常由行为树引擎循环调用 tick()。
        """
        if self._is_finished:
            return Result.ok("Task already finished")

        # 1. 检查超时
        if time.time() - self._start_time > self.params.timeout:
            self._is_finished = True
            return Result.fail("MoveToPose timed out")

        # 2. 获取当前状态
        try:
            current_pose = self.hardware.get_base_pose()
        except Exception as e:
            return Result.fail(f"Failed to get base pose: {str(e)}")

        # 3. 判断是否到达
        if is_pose_reached(current_pose, self.params.target_pose, 
                           self.params.tolerance_pos):
            self._is_finished = True
            logger.info("Target pose reached.")
            return Result.ok("Target reached")

        # 4. 下发控制指令 (这里简化为速度控制，实际可结合 PID 或底层规划器)
        # 计算简单的比例控制速度
        dist = calculate_distance(current_pose, self.params.target_pose)
        speed = min(dist * 0.5, self.params.speed_factor) # 简单的 P 控制器
        
        # 假设目标是二维平面移动
        vx = speed
        vyaw = 0.0 
        
        result = self.hardware.move_base(vx, vyaw, 0.0)
        if not result.success:
            return Result.fail(f"Hardware control failed: {result.message}")

        return Result.ok("Moving...")

    def cancel(self) -> Result:
        """中断移动并急停"""
        logger.warning("MoveToPoseSkill cancelled!")
        self.hardware.move_base(0.0, 0.0, 0.0) # 停止底盘
        self._is_finished = True
        return Result.ok("Cancelled")

    def is_finished(self) -> bool:
        return self._is_finished

    # --- IMotionSkill 专用方法 ---
    def set_target_pose(self, pose: Pose) -> None:
        if self.params:
            self.params.target_pose = pose

    def is_arrived(self, tolerance_pos: float = 0.05, tolerance_yaw: float = 0.05) -> bool:
        if not self.params:
            return False
        current = self.hardware.get_base_pose()
        return is_pose_reached(current, self.params.target_pose, tolerance_pos)