"""adapters 层双臂测试：set_arm_pose。

按头部模板直接运行，不接受命令行参数；由操作人员执行实际硬件测试。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_pose_result, validate_pose_target,
)
from core.common.logger import init_logging, get_logger

logger = get_logger(__name__)

# 人工确认安全范围后填写，两个位姿均必填，无默认真机目标。
LEFT_POSE = [0.56, 0.25, 0.91, 0, -90, 0]  # [x,y,z,yaw,pitch,roll]，xyz米，欧拉角deg。
RIGHT_POSE = [0.56, -0.25, 0.91, 0, -90, 0]
FRAME = "local"  # local/base_link 或 world/odom。


def test_set_arm_pose(hardware) -> bool:
    """发送双臂末端（m/deg），不等待到位。"""
    logger.info("=== adapters: set_arm_pose ===")
    result = hardware.set_arm_pose(left_pose=LEFT_POSE, right_pose=RIGHT_POSE, frame=FRAME)
    return check_pose_result(result, frame=FRAME)


def main() -> int:
    init_logging()
    if not validate_pose_target(LEFT_POSE, RIGHT_POSE, FRAME):
        return 1
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_set_arm_pose(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())
