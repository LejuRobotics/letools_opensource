"""手臂关节轨迹测试（技能层）

被测对象: skills.atomic.refactored_sdk.arm_joint_traj_sdk.ArmJointTrajSdkSkill
技能生命周期: initialize(ArmJointTrajSdkParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.send_arm_joint_traj_sdk(
         joint_traj, total_time) → sdk_control_mixin → ArmSDKManager（需 whitelist=['arm']）

测试用例说明:
- test_default_traj: 默认 2 点关节轨迹（零位 + 目标姿态），总时长 3 秒
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.arm_joint_traj_sdk import (
    ArmJointTrajSdkParams, ArmJointTrajSdkSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_default_traj(hardware):
    """默认 2 点关节轨迹（技能层）"""
    skill = ArmJointTrajSdkSkill(hardware=hardware)
    # 使用默认 joint_traj（零位 + 目标姿态，每点 14 关节角），总时长 3 秒
    return run_skill(skill, ArmJointTrajSdkParams(total_time=3.0))


def main():
    hardware = build_hardware(whitelist=['arm'])  # SDK 轨迹只需要 arm 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        all_passed = test_default_traj(hardware)
        if all_passed:
            logger.info("🎉 手臂关节轨迹（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=True)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
