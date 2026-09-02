"""下肢关节节点测试（编排层，TimedCmd 路径）

被测对象: orchestration.nodes.leg_joint_timed_move.LegJointTimedMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点内部 get_shared_hardware() → LegJointTimedSkill
         → hardware.send_leg_joint_timed()（planner_index=3，服务端 Ruckig 规划）

节点从 self.params 读取 j0/j1/j2/j3/total_time（与 LegJointSdkMove 完全兼容），
其中 total_time 映射到 timed 接口的 desire_time。

关节顺序: [knee, leg, waist_pitch, waist_yaw]，单位度（对齐 timed/03_leg 测试脚本）。

测试用例说明:
- test_zero_position: 下肢 4 关节回 0° 零位，持续 3 秒
- test_target_pose: 下肢设为源脚本验证过的姿态 [14.90, -32.01, 18.03, -30.0]°，持续 3 秒
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.leg_joint_timed_move import LegJointTimedMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_zero_position():
    """回到零位（编排层节点）"""
    node = LegJointTimedMove("t_zero", "零位", "ns",
                             {"j0": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "total_time": 3.0})
    return run_node(node)


def test_target_pose():
    """源脚本验证过的目标姿态（编排层节点）"""
    node = LegJointTimedMove("t_target", "目标姿态", "ns",
                             {"j0": 14.90, "j1": -32.01, "j2": 18.03, "j3": -30.0,
                              "total_time": 3.0})
    return run_node(node)


def main():
    set_hardware_config(whitelist=['timed'])  # TimedCmd 下肢路径只需要 timed 管理器
    try:
        node_setup(need_arm=False, need_torso_reset=True)
        all_passed = True
        all_passed &= test_zero_position()
        all_passed &= test_target_pose()
        if all_passed:
            logger.info("🎉 下肢关节控制（TimedCmd，编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=False)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
