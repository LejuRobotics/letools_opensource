"""双臂关节与末端测试共用配置和校验，不管理生命周期、不发送运动指令。"""

import os
from typing import List, Optional

from core.common.logger import get_logger
from core.domain.joint_state import validate_arm_joint, validate_arm_joint_trajectory
from core.domain.pose import validate_arm_pose, validate_arm_frame
from core.domain.result import Result

logger = get_logger(__name__)

HARDWARE_CONFIG = {
    "robot_type": "leju_wheeled",
    "sdk_managers_whitelist": ["low"],
    "skip_camera": True,
    "skip_end_effector": True,
    "skip_state_manager": True,
    "skip_force_publishers": True,
}


def validate_joint_target(joint: List[float]) -> bool:
    """校验人工填写的14维目标，不提供默认真机角度。"""
    try:
        validate_arm_joint(joint)
    except ValueError as exc:
        logger.error("请检查 TARGET_JOINT（deg）: %s", exc)
        return False
    return True


def validate_trajectory(times: List[float], q_frames: List[List[float]]) -> bool:
    """校验人工填写的时间点及关节帧。"""
    try:
        validate_arm_joint_trajectory(times, q_frames)
    except ValueError as exc:
        logger.error("请检查 TRAJECTORY_TIMES（秒）和 TRAJECTORY_FRAMES（deg）: %s", exc)
        return False
    return True


def check_joint_result(result: Result, *, feedback: bool = False) -> bool:
    """检查发送或14维关节反馈结果，不检查物理到位。"""
    if not result.success:
        logger.error("测试失败: %s", result.message)
        return False
    if feedback:
        try:
            joint = validate_arm_joint(result.data)
        except ValueError as exc:
            logger.error("关节反馈无效: %s", exc)
            return False
        logger.info("双臂关节反馈（deg，左7右7）: %s", joint)
        logger.warning("读取的是SDK缓存，不保证首帧或新鲜度")
        return True
    if result.data is not None:
        logger.error("发送接口的 data 应为 None: %r", result.data)
        return False
    logger.info("发送成功: %s；不表示机械运动已到位", result.message)
    return True


def validate_control_mode(control_mode: Optional[int]) -> bool:
    """模式必须由操作人员填写；模式1可能触发运动。"""
    if type(control_mode) is not int or control_mode not in (0, 1, 2):
        logger.error("请填写 CONTROL_MODE：整数0固定、1AutoSwing、2外部控制")
        return False
    return True


def check_mode_set_result(result: Result) -> bool:
    """模式服务返回成功不表示机械到位，不自动切回其他模式。"""
    if not result.success:
        logger.error("模式设置失败: %s", result.message)
        return False
    if result.data is not None:
        logger.error("模式设置接口的data应为None: %r", result.data)
        return False
    logger.info("模式设置接口成功: %s；不表示物理到位", result.message)
    return True


def check_mode_result(result: Result) -> bool:
    """校验SDK模式字典，不把缓存模式当成控制器在线证明。"""
    if not result.success:
        logger.error("模式查询失败: %s", result.message)
        return False
    data = result.data
    names = {0: "ArmFixed", 1: "AutoSwing", 2: "ExternalControl"}
    if (not isinstance(data, dict) or type(data.get("mode")) is not int
            or data["mode"] not in names or data.get("name") != names[data["mode"]]):
        logger.error("手臂模式格式无效: %r", data)
        return False
    logger.info("手臂控制模式: %s (%s)", data["mode"], data["name"])
    logger.warning("SDK查询可能回退到缓存；不表示控制器在线或机械到位")
    return True


def require_live_node() -> bool:
    """人工验证时拒绝把 dry-run 当作实际发送成功。"""
    if os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes"):
        logger.error("请取消 STUDIO_DRY_RUN，再人工运行 Node 测试")
        return False
    return True


def validate_pose_target(left_pose: List[float], right_pose: List[float], frame: str) -> bool:
    """双臂位姿必须均由操作人员填写，位置m、ZYX欧拉角deg。"""
    try:
        validate_arm_pose(left_pose)
        validate_arm_pose(right_pose)
        validate_arm_frame(frame)
    except ValueError as exc:
        logger.error("请检查 LEFT_POSE、RIGHT_POSE（m/deg）及 FRAME: %s", exc)
        return False
    return True


def validate_query(frame: str, source: str, fk_frame: Optional[str]) -> bool:
    """选择FK时需要确认模型基准；不自动猜测或回退来源。"""
    try:
        validate_arm_frame(frame)
        if source not in ("tf", "fk"):
            raise ValueError("SOURCE 必须为 tf 或 fk")
        if source == "fk" and (not isinstance(fk_frame, str) or not fk_frame.strip()):
            raise ValueError("选择fk时必须填写已确认的 FK_FRAME")
    except ValueError as exc:
        logger.error("末端反馈查询参数无效: %s", exc)
        return False
    return True


def validate_ik_input(
    left_pose: List[float], right_pose: List[float], frame: str,
    ik_frame: Optional[str], joint: Optional[List[float]],
) -> bool:
    """纯计算目标和模型基准校验，不读取硬件。"""
    if not validate_pose_target(left_pose, right_pose, frame):
        return False
    if not validate_query(frame, "fk", ik_frame):
        return False
    return joint is None or validate_joint_target(joint)


def check_pose_result(result: Result, *, frame: str, source: Optional[str] = None) -> bool:
    """source为空时检查发送结果，否则检查双臂反馈及来源信息。"""
    if not result.success:
        logger.error("测试失败: %s", result.message)
        return False
    if source is None:
        if result.data is not None:
            logger.error("发送接口的 data 应为 None: %r", result.data)
            return False
        logger.info("双臂末端目标已发送（%s）；不表示到位", frame)
        return True
    try:
        data = result.data
        if not isinstance(data, dict):
            raise ValueError("反馈必须为双臂位姿字典")
        left = validate_arm_pose(data.get("left_pose"))
        right = validate_arm_pose(data.get("right_pose"))
        if data.get("frame") != validate_arm_frame(frame) or data.get("source") != source:
            raise ValueError("返回的frame/source与请求不一致")
    except ValueError as exc:
        logger.error("末端反馈无效: %s", exc)
        return False
    logger.info("双臂末端反馈：frame=%s, source=%s", data["frame"], data["source"])
    logger.info("左臂 [x,y,z,yaw,pitch,roll]（m/deg）: %s", left)
    logger.info("右臂 [x,y,z,yaw,pitch,roll]（m/deg）: %s", right)
    if source == "fk":
        logger.info("FK参考系: %s；需人工确认工具偏移与TF末端点是否一致", data.get("fk_frame"))
    logger.warning("不保证反馈首帧、新鲜度或物理到位")
    return True
