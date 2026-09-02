"""手臂归位测试（技能层）

被测对象: skills.atomic.refactored_sdk.arm_reset_sdk.ArmResetSdkSkill
技能生命周期: initialize(ArmResetSdkParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.arm_reset() (自动 MPC 管理)

测试用例说明:
- test_arm_reset: 双臂从当前位置回到默认归位姿势（自动管理 MPC 模式）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.arm_reset_sdk import ArmResetSdkParams, ArmResetSdkSkill
from apps.test_kuavo_5w_skills._scaffold import build_hardware, skill_setup, skill_teardown, run_skill

logger = get_logger(__name__)


def test_arm_reset(hardware):
    """手臂归位（技能层）"""
    skill = ArmResetSdkSkill(hardware=hardware)
    return run_skill(skill, ArmResetSdkParams())


def main():
    hardware = build_hardware(whitelist=['arm'])  # 手臂归位只需要 arm 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        all_passed = test_arm_reset(hardware)
        if all_passed:
            logger.info("🎉 手臂归位（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=True)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
