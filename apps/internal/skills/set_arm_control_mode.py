"""skills 层手臂模式测试：设置手臂控制模式。

由操作人员填写顶部参数后直接运行，不接受命令行参数，不等待到位。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_mode_set_result, validate_control_mode,
)
from core.common.logger import init_logging, get_logger
from skills.atomic.refactored_sdk.set_arm_control_mode import SetArmControlModeParams, SetArmControlModeSkill

logger = get_logger(__name__)

# 人工确认现场安全后填写；模式1可能触发手臂运动，重复设置不保证再次重置。
CONTROL_MODE = None  # 0固定、1AutoSwing、2外部控制。


def test_set_arm_control_mode(hardware) -> bool:
    """设置手臂模式，不等待到位或自动恢复。"""
    logger.info("=== skills: set_arm_control_mode ===")
    skill = SetArmControlModeSkill(hardware=hardware)
    params = SetArmControlModeParams(control_mode=CONTROL_MODE)
    initialized = skill.initialize(params)
    if not initialized.success:
        logger.error("技能初始化失败: %s", initialized.message)
        return False
    result = skill.execute()
    if result.success and not skill.is_finished():
        logger.error("单次执行后技能未结束")
        return False
    return check_mode_set_result(result)


def main() -> int:
    init_logging()
    if not validate_control_mode(CONTROL_MODE):
        return 1
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_set_arm_control_mode(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())
