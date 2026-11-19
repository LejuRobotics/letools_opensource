"""adapters 层双臂IK计算测试，不发送运动；顶部填写参数后直接运行。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_joint_result, validate_ik_input,
)
from core.common.logger import init_logging, get_logger

logger = get_logger(__name__)

# 人工填写双臂目标，纯计算不会发送关节或末端指令。
LEFT_POSE = [0.7, 0.3, 0.7, 0, -90, 0]  # [x,y,z,yaw,pitch,roll]，m/deg。
RIGHT_POSE = [0.7, -0.3, 0.7, 0, -90, 0]
FRAME = "local"
IK_FRAME = "waist_yaw_link"  # 模型基准对应整机TF frame，必须人工确认。
JOINT = None  # 可选14维q0（deg），不是运动目标。


def test_get_arm_ik_result(hardware) -> bool:
    """计算双臂IK关节解（deg），不发送运动。"""
    logger.info("=== adapters: get_arm_ik_result ===")
    result = hardware.get_arm_ik_result(left_pose=LEFT_POSE, right_pose=RIGHT_POSE, frame=FRAME, ik_frame=IK_FRAME, joint=JOINT)
    return check_joint_result(result, feedback=True)


def main() -> int:
    init_logging()
    if not validate_ik_input(LEFT_POSE, RIGHT_POSE, FRAME, IK_FRAME, JOINT):
        return 1
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_get_arm_ik_result(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())
