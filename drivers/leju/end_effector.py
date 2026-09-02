# LeTools/drivers/leju/end_effector.py
import rospy
from typing import Union

from core.domain.end_effector import (
    DualGripperCommand,
    EndEffectorState,
    EndEffectorType,
    GripperCommand,
    HandFingerCommand,
)
from core.domain.result import Result
from core.common.logger import get_logger
from core.common.ros_environment import ensure_local_ros_python_path

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
        self._last_error = ""

    @property
    def last_error(self) -> str:
        """最近一次连接失败的原因，供 Adapter 向上层传播。"""
        return self._last_error

    def connect(self) -> bool:
        """初始化 ROS 节点并建立连接"""
        if not rospy.core.is_initialized():
            rospy.init_node('leju_end_effector_driver', anonymous=True)
        
        try:
            # 兼容未在当前 shell 中 source 本仓库 catkin setup.bash 的 Python 入口。
            # 正式应用入口会统一执行一次；Driver 再做兜底，保证被单独复用时也可靠。
            try:
                generated_path = ensure_local_ros_python_path()
            except FileNotFoundError as path_error:
                # Driver 也允许使用调用方已 source 的外部 catkin 工作空间。
                generated_path = str(path_error)
            if self._ee_type == EndEffectorType.LEJU_CLAW:
                from kuavo_msgs.srv import controlLejuClaw
                rospy.wait_for_service('/control_robot_leju_claw', timeout=3.0)
                self._claw_service = rospy.ServiceProxy('/control_robot_leju_claw', controlLejuClaw)
                logger.info("Connected to Leju Claw service.")
            elif self._ee_type == EndEffectorType.QIANGNAO_HAND:
                from kuavo_msgs.msg import robotHandPosition
                self._hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
                rospy.sleep(1.0) # 等待发布者注册
                logger.info("Connected to Qiangnao Hand topic.")
            
            self._connected = True
            self._last_error = ""
            return True
        except Exception as e:
            path_hint = locals().get("generated_path", "本仓库 ROS Python 包目录")
            self._last_error = (
                f"{type(e).__name__}: {e}; 已检查生成包目录 {path_hint}。"
                "若从其他入口运行，请先 source "
                "infrastructure/ros_packages/devel/setup.bash"
            )
            logger.error(f"Failed to connect end effector: {self._last_error}")
            return False

    def disconnect(self) -> None:
        self._connected = False
        logger.info("End effector disconnected.")

    def send_command(
        self,
        side: str,
        cmd: Union[GripperCommand, DualGripperCommand],
    ) -> Result:
        """发送通用夹爪指令"""
        if not self._connected:
            return Result.fail("Driver not connected")

        try:
            if self._ee_type == EndEffectorType.LEJU_CLAW:
                from kuavo_msgs.srv import controlLejuClawRequest
                from kuavo_msgs.msg import endEffectorData

                if isinstance(cmd, DualGripperCommand):
                    if side != "both":
                        return Result.fail(
                            "DualGripperCommand requires side=both"
                        )
                    targets = (
                        ("left_claw", cmd.left_position),
                        ("right_claw", cmd.right_position),
                    )
                    active_targets = [
                        (name, position)
                        for name, position in targets
                        if position is not None
                    ]
                    if not active_targets:
                        return Result.fail("DualGripperCommand has no active side")
                    names = [item[0] for item in active_targets]
                    positions = [item[1] for item in active_targets]
                else:
                    side_names = {
                        "left": ["left_claw"],
                        "right": ["right_claw"],
                        "both": ["left_claw", "right_claw"],
                    }
                    if side not in side_names:
                        return Result.fail(
                            f"Invalid gripper side: {side}; expected left, right or both"
                        )
                    names = side_names[side]
                    positions = [cmd.position] * len(names)

                count = len(names)
                req = controlLejuClawRequest()
                data = endEffectorData()
                data.name = names
                data.position = positions
                data.velocity = [cmd.velocity] * count
                data.effort = [cmd.effort] * count
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
