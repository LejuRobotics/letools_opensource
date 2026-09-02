"""等待节点测试（编排层，纯编排，无硬件依赖）

被测对象: orchestration.nodes.wait_seconds.WaitSeconds
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点内部 WaitSecondsSkill() → time.time() 计时（不调 hardware）

说明:
- WaitSeconds 节点驱动的 WaitSecondsSkill 是纯编排技能，无硬件依赖。
- 配合 STUDIO_DRY_RUN=1 可无 ROS/无硬件跑（update 会短路返回 SUCCESS）。
- 节点从 self.params 读取 duration_sec。
"""
import os
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.wait_seconds import WaitSeconds
from apps.test_kuavo_5w_orchestration._scaffold import run_node

logger = get_logger(__name__)


def test_wait_short():
    """等待 0.5 秒（编排层节点，纯编排）"""
    node = WaitSeconds("t_wait", "等待", "ns", {"duration_sec": 0.5})
    return run_node(node)


def main():
    # 纯编排节点默认走 dry-run 路径（无硬件依赖），也可设 STUDIO_DRY_RUN=1 显式开启
    os.environ["STUDIO_DRY_RUN"] = "1"
    try:
        all_passed = test_wait_short()
    finally:
        del os.environ["STUDIO_DRY_RUN"]
    if all_passed:
        logger.info("🎉 等待（编排层，纯编排）测试完成")
    else:
        logger.error("⚠️ 部分测试失败")
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
