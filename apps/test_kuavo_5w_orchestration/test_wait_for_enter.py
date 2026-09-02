"""按Enter继续节点测试（编排层）

被测对象: orchestration.nodes.wait_for_enter.WaitForEnter
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 纯编排节点 → WaitForEnterSkill（无 hardware）；STUDIO_DRY_RUN=1 时
         节点在 initialise() 短路（不构造技能），update() 直接返 SUCCESS。
STUDIO_DRY_RUN=1 为纯编排节点测试，无需硬件/仿真。

测试用例说明:
- test_dry_run: dry-run 下按Enter继续（message="测试暂停，按 Enter 继续..."）
"""
import os
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.wait_for_enter import WaitForEnter
from apps.test_kuavo_5w_orchestration._scaffold import run_node

logger = get_logger(__name__)


def test_dry_run():
    """dry-run 下按Enter继续（编排层节点）"""
    node = WaitForEnter(
        "t_wait", "按Enter继续", "ns",
        {"message": "测试暂停，按 Enter 继续...", "timeout": 5.0})
    return run_node(node)


def main():
    # 纯编排节点，dry-run 下 initialise() 短路；不构造 hardware、不调 set_hardware_config
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        all_passed = test_dry_run()
        if all_passed:
            logger.info("🎉 按Enter继续（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        del os.environ["STUDIO_DRY_RUN"]
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
