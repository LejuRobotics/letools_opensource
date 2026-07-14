import time
from typing import List
from dataclasses import dataclass, field
from core.interfaces.i_skill import ISkill
from core.interfaces.i_hardware import IHardware
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.common.logger import get_logger
from core.common.registry import register_skill

# 添加必要的导入
import numpy as np
from scipy.interpolate import CubicSpline

logger = get_logger(__name__)

@dataclass
class ArmTrajectoryParams(SkillParams):
    """手臂轨迹控制的专用参数"""
    joint_names: List[str] = field(default_factory=list)  # 关节名称列表
    time_points: List[float] = field(default_factory=list)  # 时间点列表
    joint_angles_list: List[List[float]] = field(default_factory=list)  # 关节角度列表
    quick_mode: int = 2  # 快速模式类型: 0-关闭, 1-下肢快, 2-上肢快, 3-上下肢快
    timeout: float = 30.0  # 超时时间（秒）
    use_hardware_trajectory: bool = False  # 是否使用硬件适配器的轨迹发送方法

@register_skill('arm_trajectory')
class ArmTrajectorySkill(ISkill):
    """
    手臂轨迹控制技能。
    用于执行预设的关节轨迹，支持快速模式设置。
    """

    def __init__(self, hardware: IHardware):
        self.hardware = hardware
        self.params: ArmTrajectoryParams = None
        self._is_finished = False
        self._start_time = 0

    @property
    def name(self) -> str:
        return "arm_trajectory"

    def initialize(self, params: ArmTrajectoryParams) -> Result:
        """初始化技能参数"""
        if not isinstance(params, ArmTrajectoryParams):
            return Result.fail("Invalid parameter type for ArmTrajectorySkill")
        
        self.params = params
        self._is_finished = False
        self._start_time = time.time()
        
        # 设置快速模式
        result = self.hardware.set_arm_quick_mode(params.quick_mode)
        if not result.success:
            return result
        
        logger.info(f"Initialized ArmTrajectorySkill with {len(params.joint_names)} joints")
        return Result.ok()
    def execute(self) -> Result:
        """执行轨迹"""
        if self._is_finished:
            return Result.ok("Task already finished")

        # 检查是否使用硬件轨迹发送方法
        if self.params.use_hardware_trajectory:
            return self.execute_with_hardware_trajectory()
        else:
            return self.execute_with_skill_trajectory()

    def execute_with_skill_trajectory(self) -> Result:
        """使用技能内置的轨迹执行逻辑"""
        # 导入必要的模块
        import time
        import numpy as np
        from scipy.interpolate import CubicSpline
        
        # 检查超时
        if time.time() - self._start_time > self.params.timeout:
            self._is_finished = True
            return Result.fail("ArmTrajectory timed out")

        try:
            # 打印调用信息，与原脚本一致
            print(f"call set_arm_quick_mode:{self.params.quick_mode}")
            
            # 设置快速模式
            quick_mode_result = self.hardware.set_arm_quick_mode(self.params.quick_mode)
            if quick_mode_result.success:
                logger.info(f"Successfully enabled {self.params.quick_mode} quick mode")
            else:
                logger.warning(f"Failed to enable {self.params.quick_mode} quick mode")
            
            # 转换为numpy数组
            times = np.array(self.params.time_points)
            angles = np.array(self.params.joint_angles_list).T  # 转置以便每行对应一个关节
            
            # 创建插值器
            interpolators = [CubicSpline(times, angles[i]) for i in range(len(self.params.joint_names))]
            
            # 生成轨迹点
            trajectory_points = []
            for t in np.arange(times[0], times[-1], 0.1):  # 50Hz
                # 计算当前关节位置
                current_angles = [interp(t) for interp in interpolators]
                trajectory_points.append(current_angles)
            
            # 使用硬件适配器发送轨迹
            result = self.hardware.send_joint_trajectory(
                self.params.joint_names,
                np.arange(times[0], times[-1], 0.1).tolist(),
                trajectory_points
            )
            
            # 禁用快速模式
            print("call set_arm_quick_mode:0")
            disable_result = self.hardware.set_arm_quick_mode(0)
            if disable_result.success:
                logger.info("Successfully enabled 0 quick mode")
            else:
                logger.warning("Failed to enable 0 quick mode")
            
            self._is_finished = True
            if result.success:
                return Result.ok("Trajectory executed successfully (using skill method)")
            else:
                return result
        except Exception as e:
            # 禁用快速模式
            self.hardware.set_arm_quick_mode(0)
            return Result.fail(f"Failed to execute trajectory: {str(e)}")

    def execute_with_hardware_trajectory(self) -> Result:
        """使用硬件适配器的轨迹发送方法"""
        # 导入必要的模块
        import time
        
        # 检查超时
        if time.time() - self._start_time > self.params.timeout:
            self._is_finished = True
            return Result.fail("ArmTrajectory timed out")

        try:
            # 打印调用信息，与原脚本一致
            print(f"call set_arm_quick_mode:{self.params.quick_mode}")
            
            # 设置快速模式
            quick_mode_result = self.hardware.set_arm_quick_mode(self.params.quick_mode)
            if quick_mode_result.success:
                logger.info(f"Successfully enabled {self.params.quick_mode} quick mode")
            else:
                logger.warning(f"Failed to enable {self.params.quick_mode} quick mode")
            
            # 使用硬件适配器的轨迹发送方法
            result = self.hardware.send_joint_trajectory(
                self.params.joint_names,
                self.params.time_points,
                self.params.joint_angles_list
            )
            
            # 禁用快速模式
            print("call set_arm_quick_mode:0")
            disable_result = self.hardware.set_arm_quick_mode(0)
            if disable_result.success:
                logger.info("Successfully enabled 0 quick mode")
            else:
                logger.warning("Failed to enable 0 quick mode")
            
            self._is_finished = True
            if result.success:
                return Result.ok("Trajectory executed successfully (using hardware method)")
            else:
                return result
        except Exception as e:
            # 禁用快速模式
            self.hardware.set_arm_quick_mode(0)
            return Result.fail(f"Failed to execute trajectory: {str(e)}")



    def cancel(self) -> Result:
        """中断轨迹执行"""
        logger.warning("ArmTrajectorySkill cancelled!")
        # 禁用快速模式
        self.hardware.set_arm_quick_mode(0)
        self._is_finished = True
        return Result.ok("Cancelled")

    def is_finished(self) -> bool:
        return self._is_finished