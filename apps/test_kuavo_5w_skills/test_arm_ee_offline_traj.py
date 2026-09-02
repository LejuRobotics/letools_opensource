"""离线时间最优末端轨迹测试（技能层）

被测对象: skills.atomic.refactored_sdk.arm_ee_offline_traj.ArmEEOfflineTrajSkill
技能生命周期: initialize(ArmEEOfflineTrajParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → enable(True) → set_offline_trajectory → sleep(total_time)
         → enable(False) → timed_command_mixin → TimedCmdManager（需 whitelist=['timed']）

与连发（burst_timed）的区别: 离线一次性提交整条带时间戳轨迹，底层 Ruckig 预规划整体
时间最优，二阶连续三阶可导，适合密集航点 / 示教回放 / 涂胶。

测试用例说明:
- test_left_world: 左臂世界系离线 4 点轨迹（默认 traj/times），总时长取 times[-1]=3s
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.arm_ee_offline_traj import (
    ArmEEOfflineTrajParams, ArmEEOfflineTrajSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_left_world(hardware):
    """左臂世界系离线 4 点轨迹（技能层）"""
    skill = ArmEEOfflineTrajSkill(hardware=hardware)
    # 使用默认 traj/times: 4 点方形轨迹, times=[0,1,2,3]
    return run_skill(skill, ArmEEOfflineTrajParams(
        side="left", frame="world", total_time=0.0, post_settle=0.5))


def main():
    hardware = build_hardware(whitelist=['timed'])  # 离线轨迹只需要 timed 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=True)
        all_passed = test_left_world(hardware)
        if all_passed:
            logger.info("🎉 离线时间最优末端轨迹（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=True)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
