"""离线时间最优末端轨迹节点测试（编排层）

被测对象: orchestration.nodes.arm_ee_offline_traj_move.ArmEeOfflineTrajMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → ArmEEOfflineTrajSkill → enable/set_offline_trajectory/sleep
         → timed_command_mixin → TimedCmdManager（需 whitelist=['timed']）
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_left_world: 左臂世界系离线 4 点轨迹（默认 traj/times JSON），total_time=0 取 times[-1]
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.arm_ee_offline_traj_move import ArmEeOfflineTrajMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)

# 默认离线轨迹 4 点（方形）+ 时间戳，与技能默认对齐
_DEFAULT_TRAJ = "[[0.3,0.25,0.5,0,0,0],[0.5,0.25,0.5,0,0,0],[0.5,0.25,0.7,0,0,0],[0.3,0.25,0.7,0,0,0]]"
_DEFAULT_TIMES = "[0,1,2,3]"


def test_left_world():
    """左臂世界系离线 4 点轨迹（编排层节点）"""
    node = ArmEeOfflineTrajMove(
        "t_lw", "左臂世界系离线轨迹", "ns",
        {"side": "left", "frame": "world", "traj": _DEFAULT_TRAJ,
         "times": _DEFAULT_TIMES, "total_time": 0.0, "post_settle": 0.5})
    return run_node(node)


def main():
    set_hardware_config(whitelist=['timed'])  # 离线轨迹只需要 timed 管理器
    try:
        node_setup(need_arm=True)
        all_passed = test_left_world()
        if all_passed:
            logger.info("🎉 离线时间最优末端轨迹（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=True)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
