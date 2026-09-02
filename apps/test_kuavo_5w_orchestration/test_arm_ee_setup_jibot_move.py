"""末端独立控制前置设置节点测试（编排层）

被测对象: orchestration.nodes.arm_ee_setup_jibot_move.ArmEeSetupJibotMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → ArmEESetupJibotSkill →
         hardware.set_arm_control_mode(2) + set_mpc_mode(ARM_EE_ONLY) + _ensure_ee_publisher()
         → mode_service_mixin（ROS 服务，无 SDK 管理器）；T4 对齐 whitelist=['timed']
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_setup: 执行末端独立控制前置设置（3 步：外部控制模式→ARM_EE_ONLY→预创建 Publisher）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.arm_ee_setup_jibot_move import ArmEeSetupJibotMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_setup():
    """末端独立控制前置设置（编排层节点）"""
    node = ArmEeSetupJibotMove("t_setup", "末端独立控制前置设置", "ns", {})
    return run_node(node)


def main():
    # mode_service_mixin 走 ROS 服务，无 SDK 管理器；T4 对齐用 timed
    set_hardware_config(whitelist=['timed'])
    try:
        node_setup(need_arm=True)
        all_passed = test_setup()
        if all_passed:
            logger.info("🎉 末端独立控制前置设置（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=True)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
