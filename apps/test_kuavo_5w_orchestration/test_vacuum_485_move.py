"""气泵485控制节点测试（编排层）

被测对象: orchestration.nodes.vacuum_485_move.Vacuum485Move
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → Vacuum485Skill → hardware（adapters.vacuum_485 模块函数）
         → ROS Trigger 服务（无 SDK 管理器）→ whitelist=[]
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_suck: 吸气（action="suck"，通道2继电器开）
- test_power_off: 断电（action="power_off"，所有继电器关）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.vacuum_485_move import Vacuum485Move
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_suck():
    """吸气（编排层节点）"""
    node = Vacuum485Move("t_suck", "气泵吸气", "ns", {"action": "suck", "timeout": 5.0})
    return run_node(node)


def test_power_off():
    """断电（编排层节点）"""
    node = Vacuum485Move("t_off", "气泵断电", "ns", {"action": "power_off", "timeout": 5.0})
    return run_node(node)


def main():
    # vacuum_485 走 ROS Trigger 服务，无 SDK 管理器
    set_hardware_config(whitelist=[])
    try:
        node_setup(need_arm=False)
        all_passed = True
        all_passed &= test_suck()
        all_passed &= test_power_off()
        if all_passed:
            logger.info("🎉 气泵485控制（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=False)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
