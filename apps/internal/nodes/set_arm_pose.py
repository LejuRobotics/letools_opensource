"""nodes 层双臂测试：set_arm_pose。

按头部模板直接运行，不接受命令行参数；由操作人员执行实际硬件测试。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, require_live_node, check_pose_result, validate_pose_target,
)
from core.common.logger import init_logging, get_logger
from orchestration.nodes.set_arm_pose import SetArmPose
from py_trees.common import Status
from core.domain.result import Result

logger = get_logger(__name__)

# 人工确认安全范围后填写，两个位姿均必填，无默认真机目标。
LEFT_POSE = []  # [x,y,z,yaw,pitch,roll]，xyz米，欧拉角deg。
RIGHT_POSE = []
FRAME = "local"  # local/base_link 或 world/odom。


def test_set_arm_pose() -> bool:
    """发送双臂末端（m/deg），不等待到位。"""
    logger.info("=== nodes: set_arm_pose ===")
    node = SetArmPose(
        "test_set_arm_pose", "双臂测试", "arm_test", {"left_pose": LEFT_POSE, "right_pose": RIGHT_POSE, "frame": FRAME},
    )
    node.tick_once()
    if node.status != Status.SUCCESS:
        logger.error("节点失败: %s", node.feedback_message)
        return False
    result = Result.ok(node.feedback_message)
    return check_pose_result(result, frame=FRAME)


def main() -> int:
    init_logging()
    if not validate_pose_target(LEFT_POSE, RIGHT_POSE, FRAME):
        return 1
    if not require_live_node():
        return 1
    from orchestration.shared_hardware import set_hardware_config, reset_shared_hardware

    set_hardware_config(dict(HARDWARE_CONFIG))
    try:
        # 节点内部获取共享硬件，getter从指定黑板键读取反馈。
        return 0 if test_set_arm_pose() else 1
    except Exception:
        logger.exception("Node 测试异常")
        return 1
    finally:
        reset_shared_hardware()


if __name__ == "__main__":
    sys.exit(main())
