# LeTools/drivers/leju/end_effector.py
import math
import threading
import time
from typing import Union

import rospy

from core.domain.end_effector import (
    DualGripperCommand,
    EndEffectorState,
    EndEffectorType,
    GripperCommand,
    GripperStatus,
    HandFingerCommand,
    SG100ControlMode,
    SG100HandCommand,
    SG100_JOINT_COUNT,
)
from core.domain.result import Result
from core.common.logger import get_logger
from core.common.ros_environment import ensure_local_ros_python_path

logger = get_logger(__name__)

class LejuEndEffector:
    """
    乐聚末端执行器驱动。
    支持二指夹爪 (Leju Claw)、强脑灵巧手 (Qiangnao Hand) 和
    黑漫 SG100 11-DOF 灵巧手 (SG100 Hand) 的底层通讯。
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
        self._claw_positions = None
        self._claw_state_condition = threading.Condition()

        # SG100 专用：命令发布者与状态订阅者
        self._sg100_pub = None
        self._sg100_state_sub = None
        self._last_sg100_side = "left"  # 最近下发侧，供 get_state() 返回默认侧
        # 双侧独立状态缓存，避免发右手后左手状态被覆盖丢失（见 _on_sg100_state）
        self._sg100_states = {
            "left": EndEffectorState(),
            "right": EndEffectorState(),
        }

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
                from kuavo_msgs.msg import lejuClawState

                rospy.wait_for_service('/control_robot_leju_claw', timeout=3.0)
                self._claw_service = rospy.ServiceProxy('/control_robot_leju_claw', controlLejuClaw)
                with self._claw_state_condition:
                    self._claw_positions = None
                self._state_sub = rospy.Subscriber(
                    '/leju_claw_state',
                    lejuClawState,
                    self._on_claw_state,
                    queue_size=1,
                )
                logger.info("Connected to Leju Claw service.")
            elif self._ee_type == EndEffectorType.QIANGNAO_HAND:
                from kuavo_msgs.msg import robotHandPosition
                self._hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
                rospy.sleep(1.0) # 等待发布者注册
                logger.info("Connected to Qiangnao Hand topic.")
            elif self._ee_type == EndEffectorType.SG100_HAND:
                from kuavo_msgs.msg import SG100HandCommand, SG100HandState
                self._sg100_pub = rospy.Publisher('/sg100_hand_command', SG100HandCommand, queue_size=10)
                self._sg100_state_sub = rospy.Subscriber('/sg100_hand_state', SG100HandState, self._on_sg100_state, queue_size=10)
                rospy.sleep(1.0)  # 等待发布者/订阅者注册
                # #2 对端存活校验：SG100 为话题接口，无法像 LEJU_CLAW 用
                # wait_for_service 探测。这里检查命令话题是否有订阅者；
                # 无订阅者通常意味着 SG100 驱动节点未启动，publish 会静默丢失。
                # 即使当前无订阅者也只告警不阻断（驱动可能稍后启动），
                # 由 send_sg100_command 在实际下发时再次校验。
                if self._sg100_pub.get_num_connections() == 0:
                    logger.warning(
                        "SG100 命令话题 /sg100_hand_command 暂无订阅者，"
                        "请确认 SG100 驱动节点已启动；下发命令时将再次校验"
                    )
                logger.info("Connected to SG100 Hand command/state topics.")

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
        if self._state_sub is not None:
            try:
                self._state_sub.unregister()
            except Exception:
                pass
            self._state_sub = None
        with self._claw_state_condition:
            self._claw_positions = None
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
                    if cmd.left_position is None and cmd.right_position is None:
                        return Result.fail("DualGripperCommand has no active side")
                    positions_result = self._complete_claw_positions(
                        cmd.left_position,
                        cmd.right_position,
                    )
                    if not positions_result.success:
                        return positions_result
                    positions = positions_result.data
                    names = ["left_claw", "right_claw"]
                else:
                    if side not in ("left", "right", "both"):
                        return Result.fail(
                            f"Invalid gripper side: {side}; expected left, right or both"
                        )
                    if side == "both":
                        positions = [cmd.position, cmd.position]
                    else:
                        positions_result = self._complete_claw_positions(
                            cmd.position if side == "left" else None,
                            cmd.position if side == "right" else None,
                        )
                        if not positions_result.success:
                            return positions_result
                        positions = positions_result.data
                    names = ["left_claw", "right_claw"]

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

    def _complete_claw_positions(self, left_position, right_position) -> Result:
        """用反馈补齐未控制侧，避免其被服务端默认置为 0（张开）。"""
        if left_position is not None and right_position is not None:
            return Result.ok(data=[left_position, right_position])

        state_result = self._get_claw_positions()
        if not state_result.success:
            return state_result
        current_left, current_right = state_result.data
        return Result.ok(
            data=[
                current_left if left_position is None else left_position,
                current_right if right_position is None else right_position,
            ]
        )

    def _get_claw_positions(self, timeout=None) -> Result:
        """返回缓存的左右位置；尚无反馈时等待首帧，超时则失败。"""
        if self._ee_type != EndEffectorType.LEJU_CLAW:
            return Result.fail("Current end effector is not Leju Claw")

        if timeout is None:
            timeout = self.config.get("claw_state_timeout", 1.0)
        try:
            timeout = float(timeout)
        except (TypeError, ValueError):
            return Result.fail("claw_state_timeout must be a positive number")
        if not math.isfinite(timeout) or timeout <= 0.0:
            return Result.fail("claw_state_timeout must be a positive number")

        deadline = time.monotonic() + timeout
        with self._claw_state_condition:
            while self._claw_positions is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return Result.fail(
                        "Timed out waiting for /leju_claw_state; "
                        "single-side command was not sent"
                    )
                self._claw_state_condition.wait(remaining)
            return Result.ok(data=list(self._claw_positions))

    def _on_claw_state(self, msg) -> None:
        """缓存左右夹爪的实际位置。"""
        try:
            data = msg.data
            raw_positions = list(data.position)
            names = list(data.name)
            if names and len(names) == len(raw_positions):
                position_by_name = dict(zip(names, raw_positions))
                raw_positions = [
                    position_by_name["left_claw"],
                    position_by_name["right_claw"],
                ]
            elif len(raw_positions) >= 2:
                raw_positions = raw_positions[:2]
            else:
                raise ValueError("state does not contain both claw positions")

            positions = [float(value) for value in raw_positions]
            if any(
                not math.isfinite(value) or not 0.0 <= value <= 100.0
                for value in positions
            ):
                raise ValueError(f"invalid claw positions: {positions}")
            with self._claw_state_condition:
                self._claw_positions = positions
                self._claw_state_condition.notify_all()
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            logger.warning("Ignore invalid /leju_claw_state message: %s", exc)

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
        # SG100：返回最近下发侧的双侧缓存状态
        if self._ee_type == EndEffectorType.SG100_HAND:
            return self._sg100_states.get(
                self._last_sg100_side, self._sg100_states["left"]
            )
        return self._current_state

    def send_sg100_command(self, side: str, cmd: SG100HandCommand) -> Result:
        """
        发布 SG100 单手灵巧手命令到 ``/sg100_hand_command``。

        单手模式：仅填充指定侧的位置与增益，另一侧 enable_mask=0、
        positions 为空列表（SG100 驱动收到空列表会忽略该侧）。

        :param side: ArmSide.value，``"left"`` 或 ``"right"``
        :param cmd: SG100 单手命令
        """
        if not self._connected or self._sg100_pub is None:
            return Result.fail("SG100 hand driver not ready")
        # 阻抗模式安全闸门：零刚度会让手部在重力下失控下垂（见 SG100HandCommand 校验）
        # 含混合模式：只要有任何关节走 JOINT_IMPEDANCE，kp/kd 必须提供
        impedance_joints = cmd.impedance_joint_indices()
        if impedance_joints and (cmd.kp is None or cmd.kd is None):
            return Result.fail(
                f"关节 {impedance_joints} 为 JOINT_IMPEDANCE 模式，"
                "必须显式提供 kp 与 kd，否则关节将因零刚度失控下垂"
            )
        try:
            from kuavo_msgs.msg import SG100HandCommand as ROSCmd
            from std_msgs.msg import Header
            msg = ROSCmd()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.control_mode = int(cmd.control_mode.value)
            self._last_sg100_side = side  # 供 get_state() 返回默认侧

            # #1 SG100 单手模式仅支持 left/right；bare else 会让 side="both" 静默
            # 落入右手分支，导致左手被禁用却仍返回 Result.ok
            if side not in ("left", "right"):
                return Result.fail(
                    f"SG100 单手模式不支持 side={side}，请使用 left 或 right"
                )
            active = side
            inactive = "right" if side == "left" else "left"

            # 活动侧：填充位置与使能掩码
            setattr(msg, f"{active}_hand_positions", list(cmd.positions))
            setattr(msg, f"{active}_enable_mask", int(cmd.enable_mask))
            # 非活动侧：mask=0、空位置数组（SG100 驱动收到空列表会忽略该侧）
            setattr(msg, f"{inactive}_hand_positions", [])
            setattr(msg, f"{inactive}_enable_mask", 0)

            # #7 显式设置活动侧 per-joint control_mode 数组：
            # 混合模式优先用 per_joint_modes，否则标量 control_mode 复制到全关节
            if cmd.per_joint_modes is not None:
                mode_arr = [int(m.value) for m in cmd.per_joint_modes]
            else:
                mode_arr = [int(cmd.control_mode.value)] * SG100_JOINT_COUNT
            setattr(msg, f"{active}_hand_control_mode", mode_arr)

            # 增益转发：仅当存在阻抗关节时才填充 kp/kd/torque_ff。
            # #1 修复：原条件 `control_mode == JOINT_IMPEDANCE or per_joint_modes
            # is not None` 过宽——纯位置模式 + 用户误传 kp 时也会转发，而 C++
            # 节点 use_pos 分支会用 per_joint_kp 覆盖全局 pid_p 调参，导致
            # 位置关节获得非预期刚度。改为只看是否有阻抗关节。
            if impedance_joints:
                if cmd.kp is not None:
                    setattr(msg, f"{active}_hand_kp", list(cmd.kp))
                if cmd.kd is not None:
                    setattr(msg, f"{active}_hand_kd", list(cmd.kd))
                if cmd.torque_ff is not None:
                    setattr(msg, f"{active}_hand_torque_ff", list(cmd.torque_ff))

            # #2 发布前校验对端订阅：无驱动节点时 publish 会静默丢失
            if self._sg100_pub.get_num_connections() == 0:
                return Result.fail(
                    "SG100 命令话题 /sg100_hand_command 无订阅者，"
                    "请确认 SG100 驱动节点已启动"
                )
            self._sg100_pub.publish(msg)
            return Result.ok("SG100 hand command published")
        except Exception as e:
            self._last_error = str(e)
            logger.exception("Publish SG100 hand command failed")
            return Result.fail(f"Publish SG100 hand command error: {e}")

    def _on_sg100_state(self, msg) -> None:
        """
        SG100 状态回调：双侧独立解析，避免发右手后左手状态被覆盖丢失。

        每侧分别读取连接状态、关节位置与错误码，映射到独立的
        ``EndEffectorState`` 缓存（``_sg100_states``）。``_current_state``
        跟踪最近下发侧，供 get_state() 返回（兼容既有单侧读取调用方）。

        状态映射：
          - 连接在线 → MOVING（在线但未确认到达目标，避免行为树误判
            运动完成而提前提升/搬运）
          - 错误码非零 → ERROR
          - 未连接 → UNKNOWN
        """
        for side in ("left", "right"):
            try:
                connected = getattr(msg, f"{side}_hand_connected", False)
                positions = getattr(msg, f"{side}_hand_positions", None)
                error = getattr(msg, f"{side}_error_code", 0)
                state = EndEffectorState()
                if connected:
                    # 在线即视为运动中，未确认到位（非 REACHED）
                    state.status = GripperStatus.MOVING
                    # 用 is not None 判断，空列表 [] 也应保留为合法零位置
                    state.finger_positions = (
                        list(positions) if positions is not None else None
                    )
                if error != 0:
                    state.status = GripperStatus.ERROR
                self._sg100_states[side] = state
            except Exception:
                logger.exception("Parse SG100 hand %s state failed", side)
                # 解析失败时该侧置 ERROR，避免保留过时数据误导上层
                err_state = EndEffectorState()
                err_state.status = GripperStatus.ERROR
                self._sg100_states[side] = err_state
        # _current_state 跟踪最近下发侧，兼容 get_state() 单侧读取
        self._current_state = self._sg100_states.get(
            self._last_sg100_side, self._sg100_states["left"]
        )
