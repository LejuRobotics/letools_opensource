"""躯干复位测试（技能层）

被测对象: skills.atomic.refactored_sdk.torso_reset_sdk.TorsoResetSdkSkill
技能生命周期: initialize(TorsoResetSdkParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.reset_torso_to_initial()
         → torso_control_mixin（ROS 服务 /mobile_manipulator_reset_torso，无 SDK 管理器）→ whitelist=[]

测试用例说明:
- test_reset: 躯干复位到初始位姿
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.torso_reset_sdk import (
    TorsoResetSdkParams, TorsoResetSdkSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_reset(hardware):
    """躯干复位到初始位姿（技能层）"""
    skill = TorsoResetSdkSkill(hardware=hardware)
    return run_skill(skill, TorsoResetSdkParams(timeout=30.0))


def main():
    # torso_control_mixin 走 ROS 服务，无 SDK 管理器
    hardware = build_hardware(whitelist=[])
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False)
        all_passed = test_reset(hardware)
        if all_passed:
            logger.info("🎉 躯干复位（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
