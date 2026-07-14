import math
from typing import Optional
from dataclasses import dataclass
from core.interfaces.i_skill import ISkill
from core.domain.result import Result
from core.common.logger import get_logger
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware

logger = get_logger(__name__)

@dataclass
class VelControlParams:
    linear_speed: float = 0.4
    angular_speed: float = 0.5
    yaw_tolerance: float = 0.005
    target_yaw_offset: float = math.pi / 2.0
    control_frequency: int = 20
    max_duration: float = 30.0
    observation_topic: str = "mobile_manipulator_mpc_observation"

class VelControlSkill(ISkill):
    def __init__(self, hardware: LejuWheeledArmHardware):
        self.hardware = hardware
        self._params: VelControlParams = None
        self._is_finished = False
        self._is_canceled = False
        self._initial_yaw = None
        self._target_yaw = None
        self._rotation_completed = False
    
    @property
    def name(self) -> str:
        return "vel_control"
    
    def initialize(self, params: VelControlParams) -> Result:
        """初始化技能"""
        self._params = params
        self._is_finished = False
        self._is_canceled = False
        self._initial_yaw = None
        self._target_yaw = None
        self._rotation_completed = False
        
        result = self.hardware.subscribe_observation(params.observation_topic)
        if not result.success:
            return result
        
        return Result.ok()
    
    def execute(self) -> Result:
        """执行技能主逻辑"""
        if self._is_canceled:
            return Result.fail("Skill has been canceled")
        
        if self._is_finished:
            return Result.ok("Velocity control already finished")
        
        try:
            import rospy
            
            rate = rospy.Rate(self._params.control_frequency)
            print("等待接收observation数据...")
            
            wait_timeout = 3.0
            start_wait_time = rospy.Time.now().to_sec()
            
            while not rospy.is_shutdown() and not self._is_canceled:
                current_yaw = self.get_current_yaw()
                if current_yaw is not None:
                    break
                
                if rospy.Time.now().to_sec() - start_wait_time > wait_timeout:
                    logger.warning("等待observation数据超时，将使用模拟数据进行测试")
                    return self.execute_simulation_mode()
                
                rate.sleep()
            
            if self._is_canceled:
                return Result.fail("Skill canceled during wait")
            
            if self.get_current_yaw() is None:
                logger.warning("observation数据不可用，将使用模拟数据进行测试")
                return self.execute_simulation_mode()
            
            self._initial_yaw = self.get_current_yaw()
            self._target_yaw = self._initial_yaw + self._params.target_yaw_offset
            
            print(f"开始执行控制序列...")
            print(f"初始yaw: {math.degrees(self._initial_yaw):.1f}°, 目标yaw: {math.degrees(self._target_yaw):.1f}°")
            
            start_time = rospy.Time.now().to_sec()
            
            while not rospy.is_shutdown() and not self._is_finished and not self._is_canceled:
                current_time = rospy.Time.now().to_sec()
                
                if current_time - start_time > self._params.max_duration:
                    print("控制超时，退出循环")
                    break
                
                current_yaw = self.get_current_yaw()
                if current_yaw is None:
                    rospy.logwarn("无法获取当前yaw角")
                    rate.sleep()
                    continue
                
                linear_x, linear_y, angular_z = self.calculate_velocities(current_yaw)
                
                self.hardware.publish_cmd_vel(linear_x, linear_y, angular_z)
                
                rate.sleep()
            
            self._is_finished = True
            return Result.ok("Velocity control completed successfully")
        
        except Exception as e:
            logger.error(f"Failed to execute velocity control: {str(e)}")
            return Result.fail(f"Failed to execute velocity control: {str(e)}")
    
    def execute_simulation_mode(self) -> Result:
        """在没有observation数据时使用模拟模式执行"""
        import rospy
        
        rate = rospy.Rate(self._params.control_frequency)
        self._initial_yaw = 0.0
        self._target_yaw = self._params.target_yaw_offset
        
        print("使用模拟数据执行控制序列...")
        
        start_time = rospy.Time.now().to_sec()
        current_sim_yaw = 0.0
        
        while not rospy.is_shutdown() and not self._is_finished and not self._is_canceled:
            current_time = rospy.Time.now().to_sec()
            
            if current_time - start_time > self._params.max_duration:
                print("控制超时，退出循环")
                break
            
            current_sim_yaw = min(self._target_yaw, current_sim_yaw + self._params.angular_speed / self._params.control_frequency)
            
            linear_x, linear_y, angular_z = self.calculate_velocities(current_sim_yaw)
            
            self.hardware.publish_cmd_vel(linear_x, linear_y, angular_z)
            
            if abs(self._target_yaw - current_sim_yaw) < self._params.yaw_tolerance:
                print("旋转完成")
                break
            
            rate.sleep()
        
        self._is_finished = True
        return Result.ok("Velocity control completed in simulation mode")
    
    def get_current_yaw(self) -> Optional[float]:
        """获取当前 yaw 角（从 observation）"""
        return self.hardware.get_current_yaw_from_observation()
    
    def normalize_angle(self, angle: float) -> float:
        """角度归一化到 [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_velocities(self, current_yaw: float) -> tuple:
        """计算线速度和角速度"""
        yaw_error = self._target_yaw - current_yaw
        yaw_error = self.normalize_angle(yaw_error)
        
        if abs(yaw_error) > self._params.yaw_tolerance:
            angular_z = self._params.angular_speed if yaw_error > 0 else -self._params.angular_speed
            
            print(f"旋转中... 当前角度: {math.degrees(current_yaw):.1f}°, 目标: {math.degrees(self._target_yaw):.1f}°, 误差: {math.degrees(yaw_error):.1f}°")
        else:
            angular_z = 0.0
            self._rotation_completed = True
        
        delta_yaw = current_yaw - self._initial_yaw
        linear_x = self._params.linear_speed * math.sin(delta_yaw)
        linear_y = self._params.linear_speed * math.cos(delta_yaw)
        
        return linear_x, linear_y, angular_z
    
    def cancel(self) -> Result:
        """取消技能执行"""
        self._is_canceled = True
        self.hardware.publish_cmd_vel(0.0, 0.0, 0.0)
        logger.info("VelControlSkill canceled")
        return Result.ok("Skill canceled successfully")
    
    def is_finished(self) -> bool:
        """检查技能是否完成"""
        return self._is_finished
    
    @property
    def params(self) -> VelControlParams:
        """获取技能参数"""
        return self._params