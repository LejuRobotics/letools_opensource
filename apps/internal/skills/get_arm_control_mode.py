"""skills 层双臂测试：get_arm_control_mode。

按头部模板直接运行，不接受命令行参数；由操作人员执行实际硬件测试。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_mode_result,
)
from core.common.logger import init_logging, get_logger
from skills.atomic.refactored_sdk.get_arm_control_mode import GetArmControlModeParams, GetArmControlModeSkill

logger = get_logger(__name__)



def test_get_arm_control_mode(hardware) -> bool:
    """查询SDK手臂模式0/1/2及名称，不切换模式。"""
    logger.info("=== skills: get_arm_control_mode ===")
    skill = GetArmControlModeSkill(hardware=hardware)
    params = GetArmControlModeParams()
    initialized = skill.initialize(params)
    if not initialized.success:
        logger.error("技能初始化失败: %s", initialized.message)
        return False
    result = skill.execute()
    if result.success and not skill.is_finished():
        logger.error("单次执行后技能未结束")
        return False
    return check_mode_result(result)


def main() -> int:
    init_logging()
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_get_arm_control_mode(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())
