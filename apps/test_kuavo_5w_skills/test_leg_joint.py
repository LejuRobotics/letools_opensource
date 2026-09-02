"""下肢关节控制测试（技能层）

被测对象: skills.atomic.refactored_sdk.leg_joint_sdk.LegJointSdkSkill
技能生命周期: initialize(LegJointSdkParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.send_leg_joint_sdk(joint_angles, total_time)

测试用例说明:
- test_zero_position: 下肢 4 个关节回到 0° 零位，持续 3 秒
- test_target_pose: 下肢设为源脚本验证过的目标姿态 [14.90, -32.01, 18.03, -90.0]°，持续 3 秒
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.leg_joint_sdk import LegJointSdkParams, LegJointSdkSkill
from apps.test_kuavo_5w_skills._scaffold import build_hardware, skill_setup, skill_teardown, run_skill

logger = get_logger(__name__)

# 源脚本 case_wheel_test_torso_joint.py 中的目标关节角度（度）
TARGET_JOINT_ANGLES = [14.90, -32.01, 18.03, -30.0]


def test_zero_position(hardware):
    """回到零位（技能层）"""
    skill = LegJointSdkSkill(hardware=hardware)
    return run_skill(skill, LegJointSdkParams(
        joint_angles=[0.0, 0.0, 0.0, 0.0], total_time=3.0))


def test_target_pose(hardware):
    """源脚本验证过的目标姿态（技能层）"""
    skill = LegJointSdkSkill(hardware=hardware)
    return run_skill(skill, LegJointSdkParams(
        joint_angles=TARGET_JOINT_ANGLES, total_time=3.0))


def main():
    hardware = build_hardware(whitelist=['low'])  # 下肢关节控制只需要 low 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False, need_torso_reset=True)
        all_passed = True
        all_passed &= test_zero_position(hardware)
        all_passed &= test_target_pose(hardware)
        if all_passed:
            logger.info("🎉 下肢关节控制（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
