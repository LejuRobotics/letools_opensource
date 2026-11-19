"""skills 层双臂关节测试：发送原生带时间点轨迹。

由操作人员填写顶部参数后直接运行，不接受命令行参数，不等待到位。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_joint_result, validate_trajectory,
)
from core.common.logger import init_logging, get_logger
from skills.atomic.refactored_sdk.set_arm_joint_trajectory import SetArmJointTrajectoryParams, SetArmJointTrajectorySkill

logger = get_logger(__name__)

# 操作人员确认关节范围及现场安全后填写；无默认真机目标。
TRAJECTORY_TIMES = []  # 相对轨迹起点的时间点（秒），大于0且严格递增。
TRAJECTORY_FRAMES = []  # N×14关节角（deg），每帧左臂7个、右臂7个。


def test_set_arm_joint_trajectory(hardware) -> bool:
    """发送轨迹，保留每帧时间，不重采样。"""
    logger.info("=== skills: set_arm_joint_trajectory ===")
    skill = SetArmJointTrajectorySkill(hardware=hardware)
    params = SetArmJointTrajectoryParams(times=TRAJECTORY_TIMES, q_frames=TRAJECTORY_FRAMES)
    initialized = skill.initialize(params)
    if not initialized.success:
        logger.error("技能初始化失败: %s", initialized.message)
        return False
    result = skill.execute()
    if result.success and not skill.is_finished():
        logger.error("单次执行后技能未结束")
        return False
    return check_joint_result(result)


def main() -> int:
    init_logging()
    if not validate_trajectory(TRAJECTORY_TIMES, TRAJECTORY_FRAMES):
        return 1
    from adapters.hardware.factory import HardwareFactory

    hardware = HardwareFactory.create_hardware(config=dict(HARDWARE_CONFIG))
    try:
        initialized = hardware.initialize()
        if not initialized.success:
            logger.error("硬件初始化失败: %s", initialized.message)
            return 1
        return 0 if test_set_arm_joint_trajectory(hardware) else 1
    except Exception:
        logger.exception("测试异常")
        return 1
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    sys.exit(main())
