"""单次末端位姿节点测试（编排层）

被测对象: orchestration.nodes.arm_ee_single_timed_move.ArmEeSingleTimedMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → ArmEESingleTimedSkill → hardware.send_timed_*_arm_ee_*()
         → timed_command_mixin → TimedCmdManager（需 whitelist=['timed']）
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_left_world: 左臂世界系单点（pose=[0.3,0.25,0.5,0,0,0]，desire_time=3.0）
- test_right_local: 右臂局部系单点（pose=[0.3,-0.25,0.5,0,0,0]，desire_time=3.0）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.arm_ee_single_timed_move import ArmEeSingleTimedMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_left_world():
    """左臂世界系单点（编排层节点）"""
    node = ArmEeSingleTimedMove(
        "t_lw", "左臂世界系单点", "ns",
        {"side": "left", "frame": "world",
         "pose": "[0.3,0.25,0.5,0,0,0]", "desire_time": 3.0})
    return run_node(node)


def test_right_local():
    """右臂局部系单点（编排层节点）"""
    node = ArmEeSingleTimedMove(
        "t_rl", "右臂局部系单点", "ns",
        {"side": "right", "frame": "local",
         "pose": "[0.3,-0.25,0.5,0,0,0]", "desire_time": 3.0})
    return run_node(node)


def main():
    set_hardware_config(whitelist=['timed'])  # 末端位姿只需要 timed 管理器
    try:
        node_setup(need_arm=True)
        all_passed = True
        all_passed &= test_left_world()
        all_passed &= test_right_local()
        if all_passed:
            logger.info("🎉 单次末端位姿（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=True)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
