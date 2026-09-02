"""底盘相对位姿测试（技能层）

被测对象: skills.atomic.refactored_sdk.base_pose_local.BasePoseLocalSkill
技能生命周期: initialize(BasePoseLocalParams) → execute() → is_finished()
底层路径: SkillBase → on_execute → hardware.send_base_pose(x, y, yaw, frame=LOCAL)

测试用例说明:
- test_move_forward: 底盘在本体坐标系下前进 0.3m（x=+0.3, y=0, yaw=0）
- test_move_backward: 底盘后退 0.3m（x=-0.3, y=0, yaw=0）
- test_rotate: 底盘原地旋转（x=0, y=0, yaw=15.0 度）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from core.domain.enums import FrameType
from skills.atomic.refactored_sdk.base_pose_local import BasePoseLocalParams, BasePoseLocalSkill
from apps.test_kuavo_5w_skills._scaffold import build_hardware, skill_setup, skill_teardown, run_skill

logger = get_logger(__name__)


def test_move_forward(hardware):
    """前进 0.3m（技能层）"""
    skill = BasePoseLocalSkill(hardware=hardware)
    return run_skill(skill, BasePoseLocalParams(
        x=0.3, y=0.0, yaw=0.0, frame=FrameType.LOCAL, timeout=60.0))


def test_move_backward(hardware):
    """后退 0.3m（技能层）"""
    skill = BasePoseLocalSkill(hardware=hardware)
    return run_skill(skill, BasePoseLocalParams(
        x=-0.3, y=0.0, yaw=0.0, frame=FrameType.LOCAL, timeout=60.0))


def test_rotate(hardware):
    """原地旋转 15°（技能层）"""
    skill = BasePoseLocalSkill(hardware=hardware)
    return run_skill(skill, BasePoseLocalParams(
        x=0.0, y=0.0, yaw=15.0, frame=FrameType.LOCAL, timeout=60.0))


def main():
    hardware = build_hardware(whitelist=['low'])  # 底盘控制只需要 low 管理器
    try:
        hardware.initialize()
        skill_setup(hardware, need_arm=False)
        all_passed = True
        all_passed &= test_move_forward(hardware)
        all_passed &= test_move_backward(hardware)
        all_passed &= test_rotate(hardware)
        if all_passed:
            logger.info("🎉 底盘相对位姿（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        skill_teardown(hardware, need_arm=False)
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
