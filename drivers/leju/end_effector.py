# LeTools/drivers/leju/end_effector.py
import rospy
from typing import Optional
from core.domain.end_effector import EndEffectorType, GripperCommand, HandFingerCommand, EndEffectorState, GripperStatus
from core.domain.result import Result
from core.common.logger import get_logger

logger = get_logger(__name__)

class LejuEndEffector:
    """
    乐聚末端执行器驱动。
    支持二指夹爪 (Leju Claw) 和灵巧手 (Qiangnao Hand) 的底层通讯。
    """
    def __init__(self, config: dict):
        self.config = config
        self._ee_type = EndEffectorType(config.get('type', 'leju_claw'))
        self._connected = False
        
        # ROS 发布者/服务代理
        self._claw_service = None
        self._hand_pub = None
        self._state_sub = None
        self._current_state = EndEffectorState()

    def connect(self) -> bool:
        """初始化 ROS 节点并建立连接"""
        if not rospy.core.is_initialized():
            rospy.init_node('leju_end_effector_driver', anonymous=True)
        
        try:
            if self._ee_type == EndEffectorType.LEJU_CLAW:
                rospy.wait_for_service('/control_robot_leju_claw', timeout=3.0)
                from kuavo_msgs.srv import controlLejuClaw
                self._claw_service = rospy.ServiceProxy('/control_robot_leju_claw', controlLejuClaw)
                logger.info("Connected to Leju Claw service.")
            elif self._ee_type == EndEffectorType.QIANGNAO_HAND:
                from kuavo_msgs.msg import robotHandPosition
                self._hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
                rospy.sleep(1.0) # 等待发布者注册
                logger.info("Connected to Qiangnao Hand topic.")
            
            self._connected = True
            return True
        except Exception as e:
            logger.error(f"Failed to connect end effector: {e}")
            return False

    def disconnect(self) -> None:
        self._connected = False
        logger.info("End effector disconnected.")

    def send_command(self, side: str, cmd: GripperCommand) -> Result:
        """发送通用夹爪指令"""
        if not self._connected:
            return Result.fail("Driver not connected")

        try:
            if self._ee_type == EndEffectorType.LEJU_CLAW:
                from kuavo_msgs.srv import controlLejuClawRequest
                from kuavo_msgs.msg import endEffectorData
                
                req = controlLejuClawRequest()
                data = endEffectorData()
                data.name = [f"{side}_claw"]
                data.position = [cmd.position]
                data.velocity = [cmd.velocity]
                data.effort = [cmd.effort]
                req.data = data
                
                resp = self._claw_service(req)
                return Result.ok() if resp.success else Result.fail(resp.message)
            
            return Result.fail(f"Gripper command not supported for type: {self._ee_type}")
        except Exception as e:
            return Result.fail(f"Send command error: {e}")

    def send_hand_command(self, left_cmd: HandFingerCommand, right_cmd: HandFingerCommand) -> Result:
        """发送灵巧手指令"""
        if not self._connected or self._ee_type != EndEffectorType.QIANGNAO_HAND:
            return Result.fail("Hand driver not ready")

        try:
            from kuavo_msgs.msg import robotHandPosition
            msg = robotHandPosition()
            msg.left_hand_position = left_cmd.positions
            msg.right_hand_position = right_cmd.positions
            self._hand_pub.publish(msg)
            return Result.ok("Hand command published")
        except Exception as e:
            return Result.fail(f"Publish hand command error: {e}")

    def get_state(self) -> EndEffectorState:
        """获取当前末端状态（需配合订阅器实现）"""
        return self._current_state
