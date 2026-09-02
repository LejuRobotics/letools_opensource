"""末端位姿直发测试（技能层）

被测对象: skills.atomic.refactored_sdk.arm_ee_pose_jibot.ArmEePoseJibotSkill
技能生命周期: initialize(ArmEePoseJibotParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.send_both_ee_poses(left, right, frame)
         → arm_control_mixin（ROS 话题 /mm/two_arm_hand_pose_cmd，无 SDK 管理器）
frame int 映射: 0=KEEP_CURRENT, 1=WORLD, 2=LOCAL

测试用例说明:
- test_both_local: 双臂局部系发送 [1.4,0.25,1.0] / [1.4,-0.25,1.0]，frame=2(LOCAL)
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.arm_ee_pose_jibot import (
    ArmEePoseJibotParams, ArmEePoseJibotSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_both_local(hardware):
    """双臂局部系末端位姿直发（技能层）"""
    skill = ArmEePoseJibotSkill(hardware=hardware)
    return run_skill(skill, ArmEePoseJibotParams(
        side="both", left_x=1.4, left_y=0.25, left_z=1.0,
        right_x=1.4, right_y=-0.25, right_z=1.0, frame=2, timeout=30.0))


def main():
    # arm_control_mixin 走 ROS 话题直发，无 SDK 管理器；但仍需 low/arm 以对齐 T4
    hardware = build_hardware(whitelist=['low', 'arm'])
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        all_passed = test_both_local(hardware)
        if all_passed:
            logger.info("🎉 末端位姿直发（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=True)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
