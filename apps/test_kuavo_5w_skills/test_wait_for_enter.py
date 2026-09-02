"""等待 Enter 测试（技能层）

被测对象: skills.atomic.refactored_sdk.wait_for_enter.WaitForEnterSkill
技能生命周期: initialize(WaitForEnterParams) → execute() → is_finished()
底层路径: 纯编排工具技能（无硬件依赖）；on_execute → input() 等待用户按 Enter
STUDIO_DRY_RUN=1 时短路（不阻塞，直接成功）。

测试用例说明:
- test_dry_run: dry-run 模式短路（STUDIO_DRY_RUN=1，不阻塞）
"""
import os
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from skills.atomic.refactored_sdk.wait_for_enter import (
    WaitForEnterParams, WaitForEnterSkill)
from apps.test_kuavo_5w_skills._scaffold import run_skill

logger = get_logger(__name__)


def test_dry_run():
    """dry-run 短路（技能层，无硬件）"""
    # wait_for_enter 在 dry-run 下直接短路成功，不阻塞
    skill = WaitForEnterSkill()
    return run_skill(skill, WaitForEnterParams(
        message="按 Enter 继续...", timeout=3600.0))


def main():
    # 纯编排技能，无 hardware；设 dry-run 以短路 input() 等待
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        all_passed = test_dry_run()
        if all_passed:
            logger.info("🎉 等待 Enter（技能层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        os.environ.pop("STUDIO_DRY_RUN", None)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
