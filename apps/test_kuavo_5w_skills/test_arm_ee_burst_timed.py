"""连发末端位姿测试（技能层）

被测对象: skills.atomic.refactored_sdk.arm_ee_burst_timed.ArmEEBurstTimedSkill
技能生命周期: initialize(ArmEEBurstTimedParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → 循环 hardware.send_timed_{side}_arm_ee_{frame}()
         → timed_command_mixin → TimedCmdManager（需 whitelist=['timed']）

测试用例说明:
- test_left_world: 左臂世界系连发 3 航点（默认），每段 3 秒 + 1 秒等待
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.arm_ee_burst_timed import (
    ArmEEBurstTimedParams, ArmEEBurstTimedSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_left_world(hardware):
    """左臂世界系连发 3 航点（技能层）"""
    skill = ArmEEBurstTimedSkill(hardware=hardware)
    # 使用默认航点 [[0.3,0.25,0.5,0,0,0],[0.5,0.25,0.5,0,0,0],[0.3,0.25,0.7,0,0,0]]
    return run_skill(skill, ArmEEBurstTimedParams(
        side="left", frame="world", desire_time=3.0, settle_time=1.0))


def main():
    hardware = build_hardware(whitelist=['timed'])  # 连发末端位姿只需要 timed 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        all_passed = test_left_world(hardware)
        if all_passed:
            logger.info("🎉 连发末端位姿（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=True)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
