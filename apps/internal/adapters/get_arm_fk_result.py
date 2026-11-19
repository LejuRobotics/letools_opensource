"""adapters 层双臂FK计算测试，不发送运动；顶部填写参数后直接运行。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_pose_result, validate_query, validate_joint_target,
)
from core.common.logger import init_logging, get_logger

logger = get_logger(__name__)

# 纯计算指定关节的末端，不读取当前关节，不发送运动。
JOINT = []  # 14维关节角（deg），左臂7个、右臂7个。
FRAME = "local"
FK_FRAME = None  # 模型基准对应整机TF frame，必须人工确认。


def test_get_arm_fk_result(hardware) -> bool:
    """计算给定关节的双臂末端（m/deg），不读取实时关节。"""
    logger.info("=== adapters: get_arm_fk_result ===")
    result = hardware.get_arm_fk_result(joint=JOINT, frame=FRAME, fk_frame=FK_FRAME)
    return check_pose_result(result, frame=FRAME, source="fk")


def main() -> int:
    init_logging()
    if not validate_joint_target(JOINT):
        return 1
    if not validate_query(FRAME, "fk", FK_FRAME):
        return 1
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_get_arm_fk_result(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())
