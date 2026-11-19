"""skills 层头部测试：获取头部姿态。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._head_test import HARDWARE_CONFIG, check_result
from core.common.logger import init_logging, get_logger
from skills.atomic.refactored_sdk.get_head_pose import GetHeadPoseParams, GetHeadPoseSkill

logger = get_logger(__name__)


def test_get_head_pose(hardware) -> bool:
    """获取并检查头部姿态 yaw/pitch（deg）。"""
    logger.info("=== skills: get_head_pose ===")
    skill = GetHeadPoseSkill(hardware=hardware)
    params = GetHeadPoseParams()
    initialized = skill.initialize(params)
    if not initialized.success:
        logger.error("技能初始化失败: %s", initialized.message)
        return False
    result = skill.execute()
    if result.success and not skill.is_finished():
        logger.error("单次执行后技能未结束")
        return False
    return check_result(result, feedback=True)


def main() -> int:
    init_logging()
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_get_head_pose(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())