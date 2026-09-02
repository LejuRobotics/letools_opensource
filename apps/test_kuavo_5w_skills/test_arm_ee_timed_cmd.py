"""双臂定时指令测试（技能层）

被测对象: skills.atomic.refactored_sdk.arm_ee_timed_cmd.ArmEETimedCmdSkill
技能生命周期: initialize(ArmEETimedCmdParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.send_arm_ee_{local,world}_timed(
         left_pose, right_pose, desire_time) → timed_command_mixin → TimedCmdManager
         （需 whitelist=['timed']）

与单次末端位姿（single_timed）的区别: TimedCmd 同时下发左右臂航点，desireTime 字段让
C++ 规划器做整体时间最优插值，适合双臂协同。

测试用例说明:
- test_local: 双臂局部系 1 航点，期望 3 秒
- test_world: 双臂世界系 1 航点，期望 3 秒
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.arm_ee_timed_cmd import (
    ArmEETimedCmdParams, ArmEETimedCmdSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)

# 双臂对称航点 [x, y, z, yaw, pitch, roll]（位置米，姿态弧度）
_LEFT_LOCAL = [0.3, 0.25, 0.5, 0.0, 0.0, 0.0]
_RIGHT_LOCAL = [0.3, -0.25, 0.5, 0.0, 0.0, 0.0]
_LEFT_WORLD = [0.3, 0.25, 0.5, 0.0, 0.0, 0.0]
_RIGHT_WORLD = [0.3, -0.25, 0.5, 0.0, 0.0, 0.0]


def test_local(hardware):
    """双臂局部系定时指令（技能层）"""
    skill = ArmEETimedCmdSkill(hardware=hardware)
    return run_skill(skill, ArmEETimedCmdParams(
        left_waypoints=[_LEFT_LOCAL], right_waypoints=[_RIGHT_LOCAL],
        desire_time=3.0, frame="local"))


def test_world(hardware):
    """双臂世界系定时指令（技能层）"""
    skill = ArmEETimedCmdSkill(hardware=hardware)
    return run_skill(skill, ArmEETimedCmdParams(
        left_waypoints=[_LEFT_WORLD], right_waypoints=[_RIGHT_WORLD],
        desire_time=3.0, frame="world"))


def main():
    hardware = build_hardware(whitelist=['timed'])  # 定时指令只需要 timed 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        all_passed = True
        all_passed &= test_local(hardware)
        all_passed &= test_world(hardware)
        if all_passed:
            logger.info("🎉 双臂定时指令（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=True)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
