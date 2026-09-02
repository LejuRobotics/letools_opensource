"""底盘相对移动节点测试（编排层）

被测对象: orchestration.nodes.base_move_relative_jibot_move.BaseMoveRelativeJibotMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → BaseMoveRelativeJibotSkill → hardware.base_move_relative_jibot()
         → jibot/chassis_mixin（ROS 服务，无 SDK 管理器）→ whitelist=[]
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_move_forward: 底盘相对前进 0.2m（x=+0.2, y=0, theta=0）
- test_rotate: 底盘原地旋转 10°（x=0, y=0, theta=10）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.base_move_relative_jibot_move import BaseMoveRelativeJibotMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_move_forward():
    """底盘相对前进 0.2m（编排层节点）"""
    node = BaseMoveRelativeJibotMove(
        "t_fwd", "底盘前进", "ns",
        {"x": 0.2, "y": 0.0, "theta": 0.0, "timeout": 60.0})
    return run_node(node)


def test_rotate():
    """底盘原地旋转 10°（编排层节点）"""
    node = BaseMoveRelativeJibotMove(
        "t_rot", "底盘旋转", "ns",
        {"x": 0.0, "y": 0.0, "theta": 10.0, "timeout": 60.0})
    return run_node(node)


def main():
    # chassis_mixin 走 ROS 服务，无 SDK 管理器
    set_hardware_config(whitelist=[])
    try:
        node_setup(need_arm=False)
        all_passed = True
        all_passed &= test_move_forward()
        all_passed &= test_rotate()
        if all_passed:
            logger.info("🎉 底盘相对移动（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=False)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
