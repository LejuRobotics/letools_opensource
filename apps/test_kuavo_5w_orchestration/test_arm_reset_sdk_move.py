"""手臂归位节点测试（编排层）

被测对象: orchestration.nodes.arm_reset_sdk_move.ArmResetSdkMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点内部 get_shared_hardware() → ArmResetSdkSkill → hardware.arm_reset()

测试用例说明:
- test_arm_reset: 构造节点并跑完整生命周期，验证节点能驱动 arm_reset_sdk 技能
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.arm_reset_sdk_move import ArmResetSdkMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_arm_reset():
    """手臂归位（编排层节点）"""
    node = ArmResetSdkMove("t_arm_reset", "手臂归位", "ns", {})
    return run_node(node)


def main():
    set_hardware_config(whitelist=['arm'])  # 手臂归位只需要 arm 管理器
    try:
        node_setup(need_arm=True)
        all_passed = test_arm_reset()
        if all_passed:
            logger.info("🎉 手臂归位（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=True)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
