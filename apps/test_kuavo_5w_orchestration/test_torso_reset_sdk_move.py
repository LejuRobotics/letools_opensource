"""躯干复位节点测试（编排层）

被测对象: orchestration.nodes.torso_reset_sdk_move.TorsoResetSdkMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → TorsoResetSdkSkill → hardware.reset_torso_to_initial()
         → torso_control_mixin（ROS 服务 /mobile_manipulator_reset_torso，无 SDK 管理器）→ whitelist=[]
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_reset: 躯干复位（无参数）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.torso_reset_sdk_move import TorsoResetSdkMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_reset():
    """躯干复位（编排层节点）"""
    node = TorsoResetSdkMove("t_torso", "躯干复位", "ns", {})
    return run_node(node)


def main():
    # torso_control_mixin 走 ROS 服务，无 SDK 管理器
    set_hardware_config(whitelist=[])
    try:
        node_setup(need_arm=False)
        all_passed = test_reset()
        if all_passed:
            logger.info("🎉 躯干复位（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=False)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
