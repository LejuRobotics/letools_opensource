"""头部控制测试（技能层）

被测对象: skills.atomic.refactored_sdk.head_control_sdk.HeadControlSdkSkill
技能生命周期: initialize(HeadControlSdkParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.control_head_sdk(yaw, pitch)

测试用例说明:
- test_center: 头部回到正前方位置（yaw=0°, pitch=0°）
- test_look_left: 头部向左转 30°（yaw=+30°）
- test_look_right: 头部向右转 30°（yaw=-30°）
- test_look_up: 头部向上抬 20°（pitch=+20°）
- test_look_down: 头部向下低 20°（pitch=-20°）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.head_control_sdk import HeadControlSdkParams, HeadControlSdkSkill
from apps.test_kuavo_5w_skills._scaffold import build_hardware, skill_setup, skill_teardown, run_skill

logger = get_logger(__name__)


def test_center(hardware):
    """头部居中（技能层）"""
    skill = HeadControlSdkSkill(hardware=hardware)
    return run_skill(skill, HeadControlSdkParams(yaw_deg=0.0, pitch_deg=0.0))


def test_look_left(hardware):
    """左转 30°（技能层）"""
    skill = HeadControlSdkSkill(hardware=hardware)
    return run_skill(skill, HeadControlSdkParams(yaw_deg=30.0, pitch_deg=0.0))


def test_look_right(hardware):
    """右转 30°（技能层）"""
    skill = HeadControlSdkSkill(hardware=hardware)
    return run_skill(skill, HeadControlSdkParams(yaw_deg=-30.0, pitch_deg=0.0))


def test_look_up(hardware):
    """上抬 20°（技能层）"""
    skill = HeadControlSdkSkill(hardware=hardware)
    return run_skill(skill, HeadControlSdkParams(yaw_deg=0.0, pitch_deg=20.0))


def test_look_down(hardware):
    """下低 20°（技能层）"""
    skill = HeadControlSdkSkill(hardware=hardware)
    return run_skill(skill, HeadControlSdkParams(yaw_deg=0.0, pitch_deg=-20.0))


def main():
    hardware = build_hardware(whitelist=['low'])  # 头部控制只需要 low 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False)
        all_passed = True
        all_passed &= test_center(hardware)
        all_passed &= test_look_left(hardware)
        all_passed &= test_look_right(hardware)
        all_passed &= test_look_up(hardware)
        all_passed &= test_look_down(hardware)
        if all_passed:
            logger.info("🎉 头部控制（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
