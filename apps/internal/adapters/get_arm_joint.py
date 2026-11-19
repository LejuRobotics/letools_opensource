"""adapters 层双臂测试：get_arm_joint。

按头部模板直接运行，不接受命令行参数；由操作人员执行实际硬件测试。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_joint_result,
)
from core.common.logger import init_logging, get_logger

logger = get_logger(__name__)



def test_get_arm_joint(hardware) -> bool:
    """读取14维反馈角（deg）。"""
    logger.info("=== adapters: get_arm_joint ===")
    result = hardware.get_arm_joint()
    return check_joint_result(result, feedback=True)


def main() -> int:
    init_logging()
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_get_arm_joint(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())
