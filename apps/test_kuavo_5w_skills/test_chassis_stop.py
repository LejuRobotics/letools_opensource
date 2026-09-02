"""底盘停止测试（技能层）

被测对象: skills.atomic.refactored_sdk.chassis_stop.ChassisStopSkill
技能生命周期: initialize(ChassisStopParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.enable_vel_control_jibot(enable)
         → jibot/chassis_mixin（ROS 服务 /enable_vel_control，无 SDK 管理器）→ whitelist=[]

enable=True 停止导航并接管速度控制；enable=False 交还导航控制权。

测试用例说明:
- test_stop: 停止底盘导航（enable=True）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.chassis_stop import (
    ChassisStopParams, ChassisStopSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_stop(hardware):
    """停止底盘导航（技能层）"""
    skill = ChassisStopSkill(hardware=hardware)
    return run_skill(skill, ChassisStopParams(enable=True, timeout=5.0))


def main():
    # chassis_mixin 走 ROS 服务，无 SDK 管理器
    hardware = build_hardware(whitelist=[])
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False)
        all_passed = test_stop(hardware)
        if all_passed:
            logger.info("🎉 底盘停止（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
