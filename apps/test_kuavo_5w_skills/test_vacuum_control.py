"""真空吸盘控制测试（技能层）

被测对象: skills.atomic.refactored_sdk.vacuum_control.VacuumControlSkill
技能生命周期: initialize(VacuumControlParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → control_vacuum_pump/control_relay（ROS/Modbus）
         （直接调模块函数，无 IHardware 方法、无 SDK 管理器）→ whitelist=[]，need_arm=False

action: "suck"=吸气（开泵），"release"=松开（关泵 + 破真空脉冲）。

测试用例说明:
- test_suck: 吸气（action="suck"）
- test_release: 松开破真空（action="release"）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.vacuum_control import (
    VacuumControlParams, VacuumControlSkill)
from apps.test_kuavo_5w_skills._scaffold import (
    build_hardware, skill_setup, skill_teardown, run_skill)

logger = get_logger(__name__)


def test_suck(hardware):
    """吸气开泵（技能层）"""
    skill = VacuumControlSkill(hardware=hardware)
    return run_skill(skill, VacuumControlParams(action="suck", timeout=10.0))


def test_release(hardware):
    """松开破真空（技能层）"""
    skill = VacuumControlSkill(hardware=hardware)
    return run_skill(skill, VacuumControlParams(action="release", timeout=10.0))


def main():
    # vacuum_control 调模块函数，无 SDK 管理器
    hardware = build_hardware(whitelist=[])
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False)
        all_passed = True
        all_passed &= test_suck(hardware)
        all_passed &= test_release(hardware)
        if all_passed:
            logger.info("🎉 真空吸盘控制（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
