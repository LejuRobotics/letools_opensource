"""头部测试共用配置和校验。"""

import math
import os

from core.common.logger import get_logger
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


def validate_target(yaw: float, pitch: float) -> bool:
    """运行前校验人工填写的目标。"""
    try:
        valid = math.isfinite(float(yaw)) and math.isfinite(float(pitch))
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        logger.error("请先在当前 setter 脚本中填写 TARGET_YAW、TARGET_PITCH（deg）")
    return valid


def check_result(result: Result, *, feedback: bool = False) -> bool:
    """打印并校验接口结果。"""
    if not result.success:
        logger.error("测试失败: %s", result.message)
        return False
    if feedback:
        data = result.data
        try:
            valid = (isinstance(data, dict) and set(data) == {"yaw", "pitch"}
                     and all(math.isfinite(float(value)) for value in data.values()))
        except (TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            logger.error("反馈格式无效: %r", data)
            return False
        logger.info("反馈角（deg）: yaw=%.3f, pitch=%.3f", data["yaw"], data["pitch"])
    else:
        if result.data is not None:
            logger.error("setter 的 data 应为 None: %r", result.data)
            return False
        logger.info("目标发送成功")
    return True


def require_live_node() -> bool:
    """避免将节点 dry-run 的 SUCCESS 当成真实发送/读取成功。"""
    if os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes"):
        logger.error("请取消 STUDIO_DRY_RUN，再人工运行 Node 测试")
        return False
    return True