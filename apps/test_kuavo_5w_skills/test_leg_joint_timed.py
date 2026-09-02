"""下肢关节控制测试（技能层，TimedCmd 路径）

被测对象: skills.atomic.refactored_sdk.leg_joint_timed.LegJointTimedSkill
技能生命周期: initialize(LegJointTimedParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.send_leg_joint_timed(joint_angles, desire_time)
         → TimedCmdManager（planner_index=3，服务端 Ruckig 时间同步规划）

与 test_leg_joint.py（SDK 路径）的区别:
- SDK 路径: send_leg_joint_sdk → 100Hz 插值循环下发 → 走停卡顿（被本路径替代）
- TimedCmd 路径: 一次下发目标关节角 + desire_time，服务端规划完整轨迹连续执行
- 服务规划完即返回 actual_time（非阻塞），技能内部 sleep(actual_time + settle_time) 等待

测试用例说明:
- test_zero_position: 下肢 4 个关节回到 0° 零位，desire_time 3 秒
- test_target_pose: 下肢设为源脚本验证过的目标姿态 [14.90, -32.01, 18.03, -30.0]°，desire_time 3 秒
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.leg_joint_timed import (
    LegJointTimedParams,
    LegJointTimedSkill,
)
from apps.test_kuavo_5w_skills._scaffold import build_hardware, skill_setup, skill_teardown, run_skill

logger = get_logger(__name__)

# 源脚本 case_wheel_test_torso_joint.py 中的目标关节角度（度）
TARGET_JOINT_ANGLES = [14.90, -32.01, 18.03, -30.0]


def test_zero_position(hardware):
    """回到零位（技能层，TimedCmd）"""
    skill = LegJointTimedSkill(hardware=hardware)
    return run_skill(skill, LegJointTimedParams(
        joint_angles=[0.0, 0.0, 0.0, 0.0], desire_time=3.0))


def test_target_pose(hardware):
    """源脚本验证过的目标姿态（技能层，TimedCmd）"""
    skill = LegJointTimedSkill(hardware=hardware)
    return run_skill(skill, LegJointTimedParams(
        joint_angles=TARGET_JOINT_ANGLES, desire_time=3.0))


def main():
    hardware = build_hardware(whitelist=['timed'])  # TimedCmd 下肢路径只需要 timed 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False, need_torso_reset=True)
        all_passed = True
        all_passed &= test_zero_position(hardware)
        all_passed &= test_target_pose(hardware)
        if all_passed:
            logger.info("🎉 下肢关节控制（技能层，TimedCmd）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
