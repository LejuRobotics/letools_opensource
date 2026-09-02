"""等待测试（技能层，纯编排，无硬件依赖）

被测对象: skills.atomic.refactored_sdk.wait_seconds.WaitSecondsSkill
技能生命周期: initialize(WaitSecondsParams) → execute() → is_finished()
底层路径: 纯编排技能，on_execute 内部 time.time() 计时，不调任何 hardware 方法

说明:
- WaitSecondsSkill 是纯编排技能（__init__ 无 hardware 参数），用于验证
  「无硬件依赖」技能也能被 run_skill 生命周期运行器驱动。
- 配合 STUDIO_DRY_RUN=1 可无 ROS/无硬件跑（on_execute/on_is_finished 会短路返回）。
"""
import os
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.wait_seconds import WaitSecondsParams, WaitSecondsSkill
from apps.test_kuavo_5w_skills._scaffold import run_skill

logger = get_logger(__name__)


def test_wait_short():
    """等待 0.5 秒（技能层，纯编排）"""
    skill = WaitSecondsSkill()  # 纯编排技能，无需 hardware
    return run_skill(skill, WaitSecondsParams(duration_sec=0.5, timeout=10.0))


def main():
    # 纯编排技能默认走 dry-run 路径（无硬件依赖），也可设 STUDIO_DRY_RUN=1 显式开启
    all_passed = test_wait_short()
    if all_passed:
        logger.info("🎉 等待（技能层，纯编排）测试完成")
    else:
        logger.error("⚠️ 部分测试失败")
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
