"""末端轨迹-世界系节点测试（编排层）

被测对象: orchestration.nodes.arm_ee_traj_world_sdk_move.ArmEeTrajWorldSdkMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → ArmEETrajWorldSdkSkill → hardware.send_arm_ee_traj_sdk(frame="world")
         → sdk_control_mixin → ArmSDKManager（需 whitelist=['arm']）
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_default_traj: 双臂世界系 2 点轨迹（默认 left_traj/right_traj JSON），total_time=3.0
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.arm_ee_traj_world_sdk_move import ArmEeTrajWorldSdkMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)

# 默认 2 点轨迹 JSON（每点 [x,y,z,yaw,pitch,roll,time]，与技能默认对齐）
_LEFT_TRAJ = "[[0.3,0.25,0.5,0,0,0,1.0],[0.5,0.25,0.5,0,0,0,1.0]]"
_RIGHT_TRAJ = "[[0.3,-0.25,0.5,0,0,0,1.0],[0.5,-0.25,0.5,0,0,0,1.0]]"


def test_default_traj():
    """双臂世界系 2 点轨迹（编排层节点）"""
    node = ArmEeTrajWorldSdkMove(
        "t_traj", "双臂世界系轨迹", "ns",
        {"left_traj": _LEFT_TRAJ, "right_traj": _RIGHT_TRAJ, "total_time": 3.0})
    return run_node(node)


def main():
    set_hardware_config(whitelist=['arm'])  # SDK 轨迹只需要 arm 管理器
    try:
        node_setup(need_arm=True)
        all_passed = test_default_traj()
        if all_passed:
            logger.info("🎉 末端轨迹-世界系（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=True)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
