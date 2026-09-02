"""气泵控制节点测试（编排层）

被测对象: orchestration.nodes.vacuum_control_move.VacuumControlMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → VacuumControlSkill → hardware
         （control_vacuum_pump / control_relay，ROS 服务，无 SDK 管理器）→ whitelist=[]
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_suck: 吸气（action="suck"，开气泵）
- test_release: 松开（action="release"，关气泵 + 破真空继电器脉冲）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.vacuum_control_move import VacuumControlMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_suck():
    """吸气（编排层节点）"""
    node = VacuumControlMove("t_suck", "气泵吸气", "ns", {"action": "suck", "timeout": 5.0})
    return run_node(node)


def test_release():
    """松开含破真空（编排层节点）"""
    node = VacuumControlMove("t_rel", "气泵松开", "ns", {"action": "release", "timeout": 5.0})
    return run_node(node)


def main():
    # vacuum_control 走 ROS 服务，无 SDK 管理器
    set_hardware_config(whitelist=[])
    try:
        node_setup(need_arm=False)
        all_passed = True
        all_passed &= test_suck()
        all_passed &= test_release()
        if all_passed:
            logger.info("🎉 气泵控制（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=False)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
