"""双臂定时指令节点测试（编排层）

被测对象: orchestration.nodes.arm_ee_timed_cmd_move.ArmEeTimedCmdMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → ArmEETimedCmdSkill → hardware.send_arm_ee_{local,world}_timed()
         → timed_command_mixin → TimedCmdManager（需 whitelist=['timed']）
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_local: 双臂局部系 1 航点（left/right_waypoints JSON），desire_time=3.0
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.arm_ee_timed_cmd_move import ArmEeTimedCmdMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)

# 双臂对称单航点 JSON（位置米，姿态弧度）
_LEFT = "[[0.3,0.25,0.5,0,0,0]]"
_RIGHT = "[[0.3,-0.25,0.5,0,0,0]]"


def test_local():
    """双臂局部系定时指令（编排层节点）"""
    node = ArmEeTimedCmdMove(
        "t_local", "双臂局部系定时指令", "ns",
        {"left_waypoints": _LEFT, "right_waypoints": _RIGHT,
         "desire_time": 3.0, "frame": "local"})
    return run_node(node)


def main():
    set_hardware_config(whitelist=['timed'])  # 定时指令只需要 timed 管理器
    try:
        node_setup(need_arm=True)
        all_passed = test_local()
        if all_passed:
            logger.info("🎉 双臂定时指令（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=True)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
