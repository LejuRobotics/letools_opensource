"""nodes 层双臂关节测试：发送原生带时间点轨迹。

由操作人员填写顶部参数后直接运行，不接受命令行参数，不等待到位。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_joint_result, validate_trajectory, require_live_node,
)
from core.common.logger import init_logging, get_logger
from orchestration.nodes.set_arm_joint_trajectory import SetArmJointTrajectory
from py_trees.common import Status
from core.domain.result import Result

logger = get_logger(__name__)

# 操作人员确认关节范围及现场安全后填写；无默认真机目标。
TRAJECTORY_TIMES = []  # 相对轨迹起点的时间点（秒），大于0且严格递增。
TRAJECTORY_FRAMES = []  # N×14关节角（deg），每帧左臂7个、右臂7个。


def test_set_arm_joint_trajectory() -> bool:
    """发送轨迹，保留每帧时间，不重采样。"""
    logger.info("=== nodes: set_arm_joint_trajectory ===")
    node = SetArmJointTrajectory(
        "test_set_arm_joint_trajectory", "双臂关节测试", "arm_joints_test",
        {"times": TRAJECTORY_TIMES, "q_frames": TRAJECTORY_FRAMES},
    )
    node.tick_once()
    if node.status != Status.SUCCESS:
        logger.error("节点失败: %s", node.feedback_message)
        return False
    result = Result.ok(node.feedback_message)
    return check_joint_result(result)


def main() -> int:
    init_logging()
    if not validate_trajectory(TRAJECTORY_TIMES, TRAJECTORY_FRAMES):
        return 1
    if not require_live_node():
        return 1
    from orchestration.shared_hardware import set_hardware_config, reset_shared_hardware

    set_hardware_config(dict(HARDWARE_CONFIG))
    try:
        # 节点内部获取共享硬件并触发初始化。
        return 0 if test_set_arm_joint_trajectory() else 1
    except Exception:
        logger.exception("Node 测试异常")
        return 1
    finally:
        reset_shared_hardware()


if __name__ == "__main__":
    sys.exit(main())
