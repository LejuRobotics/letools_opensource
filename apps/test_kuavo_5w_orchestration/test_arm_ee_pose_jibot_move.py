"""末端位姿直发节点测试（编排层）

被测对象: orchestration.nodes.arm_ee_pose_jibot_move.ArmEePoseJibotMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → ArmEePoseJibotSkill → hardware.send_both_ee_poses()
         → arm_control_mixin（ROS 话题 /mm/two_arm_hand_pose_cmd，无 SDK 管理器）
frame int 映射: 0=KEEP_CURRENT, 1=WORLD, 2=LOCAL
STUDIO_DRY_RUN=1 时节点在 initialise() 短路，update() 直接返 SUCCESS。

测试用例说明:
- test_both_local: 双臂局部系末端位姿（left/right 对称，frame=2=LOCAL）
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.arm_ee_pose_jibot_move import ArmEePoseJibotMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_both_local():
    """双臂局部系末端位姿（编排层节点）"""
    node = ArmEePoseJibotMove(
        "t_pose", "双臂局部系末端位姿", "ns",
        {"side": "both", "left_x": 1.4, "left_y": 0.25, "left_z": 1.0,
         "right_x": 1.4, "right_y": -0.25, "right_z": 1.0, "frame": 2})
    return run_node(node)


def main():
    # arm_control_mixin 走 ROS 话题直发，无 SDK 管理器；对齐 T4 用 low+arm
    set_hardware_config(whitelist=['low', 'arm'])
    try:
        node_setup(need_arm=True)
        all_passed = test_both_local()
        if all_passed:
            logger.info("🎉 末端位姿直发（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=True)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
