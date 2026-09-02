"""到达检查测试（技能层）

被测对象: skills.atomic.refactored_sdk.check_arrived_jibot.CheckArrivedJibotSkill
技能生命周期: initialize(CheckArrivedJibotParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.check_arrived_jibot(task_id, blocking, timeout)
         → jibot/chassis_mixin（ROS 服务 /move_base/*，无 SDK 管理器）→ whitelist=[]

blocking=True 时阻塞至导航任务完成或超时；返回 arrived/status/message。

测试用例说明:
- test_non_blocking: 非阻塞查询当前导航状态（task_id=""，blocking=False）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.check_arrived_jibot import (
    CheckArrivedJibotParams, CheckArrivedJibotSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_non_blocking(hardware):
    """非阻塞查询导航到达状态（技能层）"""
    skill = CheckArrivedJibotSkill(hardware=hardware)
    return run_skill(skill, CheckArrivedJibotParams(
        task_id="", blocking=False, timeout=20.0))


def main():
    # chassis_mixin 走 ROS 服务，无 SDK 管理器
    hardware = build_hardware(whitelist=[])
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False)
        all_passed = test_non_blocking(hardware)
        if all_passed:
            logger.info("🎉 到达检查（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
