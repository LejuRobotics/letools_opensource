"""双臂末端单次位姿测试（技能层）

被测对象: skills.atomic.refactored_sdk.arm_ee_dual_timed.ArmEEDualTimedSkill
技能生命周期: initialize(ArmEEDualTimedParams) → execute() → is_finished()
底层路径: SkillBase → hardware.send_arm_ee_local_timed()
         → timed_command_mixin → TimedCmdManager（需 whitelist=['timed']）

测试流程对齐 test_arm_ee_local.py:
- default → forward
- 两段动作之间保持手臂外部控制，第二段完成后统一释放
"""

import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import get_logger, init_logging
from core.domain.enums import MPCControlMode

init_logging()

from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware,
    run_skill,
)
from skills.atomic.refactored_sdk.arm_ee_dual_timed import (
    ArmEEDualTimedParams,
    ArmEEDualTimedSkill,
)

logger = get_logger(__name__)

DEFAULT_LEFT = [1.1, 0.1, 0.7, 0.0, -110.0, 0.0]
DEFAULT_RIGHT = [1.1, -0.1, 0.7, 0.0, -110.0, 0.0]

# 双臂前伸
FORWARD_LEFT = [1.18, 0.1, 0.8, 0.0, -125.0, 0.0]
FORWARD_RIGHT = [1.18, -0.1, 0.8, 0.0, -125.0, 0.0]


def run_local_pose(
    hardware,
    left_pose,
    right_pose,
    prepare_arm_control,
    release_arm_control,
):
    """通过技能层发送一组双臂 local 位姿。"""
    skill = ArmEEDualTimedSkill(hardware=hardware)
    return run_skill(
        skill,
        ArmEEDualTimedParams(
            frame="local",
            left_pose=left_pose,
            right_pose=right_pose,
            desire_time=3.0,
            prepare_arm_control=prepare_arm_control,
            release_arm_control=release_arm_control,
        ),
    )


def test_sequence(hardware):
    """执行 default → forward，并仅在整个序列结束时释放手臂控制。"""
    return run_local_pose(
        hardware,
        DEFAULT_LEFT,
        DEFAULT_RIGHT,
        prepare_arm_control=True,
        release_arm_control=False,
    ) and run_local_pose(
        hardware,
        FORWARD_LEFT,
        FORWARD_RIGHT,
        prepare_arm_control=False,
        release_arm_control=True,
    )


def main():
    hardware = build_hardware(whitelist=["timed"])
    all_passed = False
    initialized = False
    try:
        result = hardware.initialize()
        initialized = result.success
        if not result.success:
            logger.error("硬件初始化失败: %s", result.message)
        else:
            result = hardware.set_mpc_mode(MPCControlMode.ARM_EE_ONLY)
            if not result.success:
                logger.error("设置 MPC 模式失败: %s", result.message)
            else:
                all_passed = test_sequence(hardware)
                if all_passed:
                    logger.info("双臂末端局部位姿（技能层）测试完成")
                else:
                    logger.error("部分测试失败")
    finally:
        if initialized:
            result = hardware.set_mpc_mode(MPCControlMode.NO_CONTROL)
            if not result.success:
                logger.warning("释放 MPC 控制失败: %s", result.message)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
