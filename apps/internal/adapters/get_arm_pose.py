"""adapters 层双臂测试：get_arm_pose。

按头部模板直接运行，不接受命令行参数；由操作人员执行实际硬件测试。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_pose_result, validate_query,
)
from core.common.logger import init_logging, get_logger

logger = get_logger(__name__)

FRAME = "local"  # local/base_link 或 world/odom（不是map）。
SOURCE = "fk"  # tf或fk，不会在失败时自动切换。
FK_FRAME = "waist_yaw_link"  # SOURCE=fk时填写已确认的模型输出对应TF frame。


def test_get_arm_pose(hardware) -> bool:
    """读取双臂末端（m/deg），校验反馈来源和坐标系。"""
    logger.info("=== adapters: get_arm_pose ===")
    result = hardware.get_arm_pose(frame=FRAME, source=SOURCE, fk_frame=FK_FRAME)
    return check_pose_result(result, frame=FRAME, source=SOURCE)


def main() -> int:
    init_logging()
    if not validate_query(FRAME, SOURCE, FK_FRAME):
        return 1
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_get_arm_pose(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())
