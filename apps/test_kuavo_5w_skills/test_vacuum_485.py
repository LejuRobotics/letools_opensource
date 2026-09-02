"""气泵485控制测试（技能层）

被测对象: skills.atomic.refactored_sdk.vacuum_485.Vacuum485Skill
技能生命周期: initialize(Vacuum485Params) → execute() → is_finished()
底层路径: SkillBase → on_execute → adapters.vacuum_485.{blow/suck/power_off}()
         （ROS Trigger 服务，直接调模块函数，无 IHardware 方法、无 SDK 管理器）→ whitelist=[]

action: "blow"=吹气, "suck"=吸气, "power_off"=断电全关。

测试用例说明:
- test_suck: 吸气（action="suck"）
- test_power_off: 断电全关（action="power_off"）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.vacuum_485 import (
    Vacuum485Params, Vacuum485Skill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_suck(hardware):
    """吸气（技能层）"""
    skill = Vacuum485Skill(hardware=hardware)
    return run_skill(skill, Vacuum485Params(action="suck", timeout=10.0))


def test_power_off(hardware):
    """断电全关（技能层）"""
    skill = Vacuum485Skill(hardware=hardware)
    return run_skill(skill, Vacuum485Params(action="power_off", timeout=10.0))


def main():
    # vacuum_485 调 adapters 模块函数，无 SDK 管理器
    hardware = build_hardware(whitelist=[])
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False)
        all_passed = True
        all_passed &= test_suck(hardware)
        all_passed &= test_power_off(hardware)
        if all_passed:
            logger.info("🎉 气泵485控制（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
