"""末端独立控制前置设置测试（技能层）

被测对象: skills.atomic.refactored_sdk.arm_ee_setup_jibot.ArmEESetupJibotSkill
技能生命周期: initialize(ArmEESetupJibotParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.set_arm_control_mode(2) +
         hardware.set_mpc_mode(ARM_EE_ONLY) + hardware._ensure_ee_publisher()
         → mode_service_mixin（ROS 服务，无 SDK 管理器）；T4 对齐 whitelist=['timed']

无 dry-run 短路：该技能调用的全是 ROS 服务/话题，必须真机/仿真执行。

测试用例说明:
- test_setup: 执行末端独立控制前置设置（3 步：外部控制模式→ARM_EE_ONLY→预创建 Publisher）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.arm_ee_setup_jibot import (
    ArmEESetupJibotParams, ArmEESetupJibotSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_setup(hardware):
    """末端独立控制前置设置（技能层）"""
    skill = ArmEESetupJibotSkill(hardware=hardware)
    return run_skill(skill, ArmEESetupJibotParams(timeout=30.0))


def main():
    # mode_service_mixin 走 ROS 服务，无 SDK 管理器；T4 对齐用 timed
    hardware = build_hardware(whitelist=['timed'])
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        all_passed = test_setup(hardware)
        if all_passed:
            logger.info("🎉 末端独立控制前置设置（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=True)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
