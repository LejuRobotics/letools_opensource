"""手臂关节轨迹节点测试（编排层）

被测对象: orchestration.nodes.arm_joint_traj_sdk_move.ArmJointTrajSdkMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → ArmJointTrajSdkSkill → hardware.send_arm_joint_traj_sdk()
         → sdk_control_mixin → ArmSDKManager（需 whitelist=['arm']）
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

节点默认从全局黑板读 ArmJointTrajectories（use_board_trajectory=true）。为避免黑板依赖，
本测试设 use_board_trajectory=false，直接内联 joint_traj（每点 14 关节角）。

测试用例说明:
- test_inline_traj: 内联 2 点关节轨迹（零位 + 目标姿态），total_time=3.0
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.arm_joint_traj_sdk_move import ArmJointTrajSdkMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)

# 内联 2 点关节轨迹 JSON（每点 14 关节角，与技能默认对齐：零位 + 目标姿态）
_JOINT_TRAJ = "[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[-30.0,20.0,15.0,-45.0,25.0,10.0,-35.0,-30.0,-20.0,-15.0,-45.0,-25.0,-10.0,-35.0]]"


def test_inline_traj():
    """内联 2 点关节轨迹（编排层节点）"""
    node = ArmJointTrajSdkMove(
        "t_traj", "手臂关节轨迹", "ns",
        {"use_board_trajectory": "false", "joint_traj": _JOINT_TRAJ, "total_time": 3.0})
    return run_node(node)


def main():
    set_hardware_config(whitelist=['arm'])  # SDK 轨迹只需要 arm 管理器
    try:
        node_setup(need_arm=True)
        all_passed = test_inline_traj()
        if all_passed:
            logger.info("🎉 手臂关节轨迹（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=True)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
