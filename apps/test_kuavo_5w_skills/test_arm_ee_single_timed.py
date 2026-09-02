"""单次末端位姿测试（技能层）

被测对象: skills.atomic.refactored_sdk.arm_ee_single_timed.ArmEESingleTimedSkill
技能生命周期: initialize(ArmEESingleTimedParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.send_timed_{side}_arm_ee_{frame}(pose, desire_time)
         → timed_command_mixin → TimedCmdManager（需 whitelist=['timed']）

测试用例说明:
- test_left_world: 左臂世界系发送单点 [0.3, 0.25, 0.5, 0, 0, 0]，期望 3 秒
- test_right_local: 右臂局部系发送单点 [0.3, -0.25, 0.5, 0, 0, 0]，期望 3 秒
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.arm_ee_single_timed import (
    ArmEESingleTimedParams, ArmEESingleTimedSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)

# 默认末端位姿 [x, y, z, yaw, pitch, roll]（位置米，姿态度）
_LEFT_WORLD = [0.5, 0.25, 0.8, 0.0, 0.0, 0.0]
_RIGHT_LOCAL = [0.5, -0.25, 0.8, 0.0, 0.0, 0.0]


def test_left_world(hardware):
    """左臂世界系单点（技能层）"""
    skill = ArmEESingleTimedSkill(hardware=hardware)
    return run_skill(skill, ArmEESingleTimedParams(
        side="left", frame="world", pose=_LEFT_WORLD, desire_time=3.0))


def test_right_local(hardware):
    """右臂局部系单点（技能层）"""
    skill = ArmEESingleTimedSkill(hardware=hardware)
    return run_skill(skill, ArmEESingleTimedParams(
        side="right", frame="local", pose=_RIGHT_LOCAL, desire_time=3.0))


def main():
    hardware = build_hardware(whitelist=['timed'])  # 末端位姿控制只需要 timed 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        all_passed = True
        all_passed &= test_left_world(hardware)
        all_passed &= test_right_local(hardware)
        if all_passed:
            logger.info("🎉 单次末端位姿（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=True)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
