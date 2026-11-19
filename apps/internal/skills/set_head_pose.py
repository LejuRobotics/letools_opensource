"""skills 层头部测试：发送头部姿态。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._head_test import HARDWARE_CONFIG, check_result, validate_target
from core.common.logger import init_logging, get_logger
from skills.atomic.refactored_sdk.set_head_pose import SetHeadPoseParams, SetHeadPoseSkill

logger = get_logger(__name__)

# 操作人员确认现场安全和关节范围后填写，单位：deg。
TARGET_YAW = 50.0
TARGET_PITCH = -20.0


def test_set_head_pose(hardware) -> bool:
    """发送头部姿态 yaw/pitch（deg）。"""
    logger.info("=== skills: set_head_pose ===")
    skill = SetHeadPoseSkill(hardware=hardware)
    params = SetHeadPoseParams(yaw=TARGET_YAW, pitch=TARGET_PITCH)
    initialized = skill.initialize(params)
    if not initialized.success:
        logger.error("技能初始化失败: %s", initialized.message)
        return False
    result = skill.execute()
    if result.success and not skill.is_finished():
        logger.error("单次执行后技能未结束")
        return False
    return check_result(result, feedback=False)


def main() -> int:
    init_logging()
    if not validate_target(TARGET_YAW, TARGET_PITCH):
        return 1
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_set_head_pose(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())