"""底盘目标移动测试（技能层）

被测对象: skills.atomic.refactored_sdk.base_move_to_target_jibot.BaseMoveToTargetJibotSkill
技能生命周期: initialize(BaseMoveToTargetJibotParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.base_move_to_target_jibot(x, y, theta, options)
         → jibot/chassis_mixin（ROS 服务 /move_base/*，无 SDK 管理器）→ whitelist=[]

与 base_move_relative 的区别: 目标坐标系为世界系（绝对目标位姿），而非本体相对位移。

测试用例说明:
- test_move_to_target: 底盘移动到世界系目标 (1.0, 0.0, 0°)
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.base_move_to_target_jibot import (
    BaseMoveToTargetJibotParams, BaseMoveToTargetJibotSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_move_to_target(hardware):
    """底盘移动到世界系目标 (1.0, 0.0, 0°)（技能层）"""
    skill = BaseMoveToTargetJibotSkill(hardware=hardware)
    return run_skill(skill, BaseMoveToTargetJibotParams(
        x=1.0, y=0.0, theta=0.0, timeout=60.0))


def main():
    # chassis_mixin 走 ROS 服务，无 SDK 管理器
    hardware = build_hardware(whitelist=[])
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False)
        all_passed = test_move_to_target(hardware)
        if all_passed:
            logger.info("🎉 底盘目标移动（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
