"""底盘目标移动节点测试（编排层）

被测对象: orchestration.nodes.base_move_to_target_jibot_move.BaseMoveToTargetJibotMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → BaseMoveToTargetJibotSkill → hardware.base_move_to_target_jibot()
         → jibot/chassis_mixin（ROS 服务，无 SDK 管理器）→ whitelist=[]
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_move_to_target: 底盘移动到世界系目标 (1.0, 0.0, 0°)
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.base_move_to_target_jibot_move import BaseMoveToTargetJibotMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_move_to_target():
    """底盘移动到世界系目标 (1.0, 0.0, 0°)（编排层节点）"""
    node = BaseMoveToTargetJibotMove(
        "t_target", "底盘目标移动", "ns",
        {"x": 1.0, "y": 0.0, "theta": 0.0, "timeout": 60.0})
    return run_node(node)


def main():
    # chassis_mixin 走 ROS 服务，无 SDK 管理器
    set_hardware_config(whitelist=[])
    try:
        node_setup(need_arm=False)
        all_passed = test_move_to_target()
        if all_passed:
            logger.info("🎉 底盘目标移动（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=False)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
