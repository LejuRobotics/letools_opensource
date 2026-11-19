"""adapters 层双臂关节测试：发送单个关节目标。

由操作人员填写顶部参数后直接运行，不接受命令行参数，不等待到位。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_joint_result, validate_joint_target,
)
from core.common.logger import init_logging, get_logger

logger = get_logger(__name__)

# 操作人员确认关节范围及现场安全后填写，单位：deg。
TARGET_JOINT = [
    # 0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0,
    # 0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0
    -29.637976500529323, 0.0, 0.0, -29.637976500529316, 0.0, 0.0, -29.637976500529316, -29.63797650052942, 0.0, 0.0, -29.637976500529405, 0.0, 0.0, -29.637976500529412
]  # 14个关节角：左臂7个、右臂7个。


def test_set_arm_joint(hardware) -> bool:
    """发送14维双臂关节角。"""
    logger.info("=== adapters: set_arm_joint ===")
    result = hardware.set_arm_joint(joint=TARGET_JOINT)
    return check_joint_result(result)


def main() -> int:
    init_logging()
    if not validate_joint_target(TARGET_JOINT):
        return 1
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_set_arm_joint(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())
