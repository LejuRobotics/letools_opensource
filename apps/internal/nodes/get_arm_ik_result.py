"""nodes 层双臂IK计算测试，不发送运动；顶部填写参数后直接运行。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, require_live_node, check_joint_result, validate_ik_input,
)
from core.common.logger import init_logging, get_logger
from orchestration.nodes.get_arm_ik_result import GetArmIkResult
from py_trees.common import Status
from core.domain.result import Result

logger = get_logger(__name__)

# 人工填写双臂目标，纯计算不会发送关节或末端指令。
LEFT_POSE = []  # [x,y,z,yaw,pitch,roll]，m/deg。
RIGHT_POSE = []
FRAME = "local"
IK_FRAME = None  # 模型基准对应整机TF frame，必须人工确认。
JOINT = None  # 可选14维q0（deg），不是运动目标。

OUTPUT_KEY = "arm_ik_result"


def test_get_arm_ik_result() -> bool:
    """计算双臂IK关节解（deg），不发送运动。"""
    logger.info("=== nodes: get_arm_ik_result ===")
    node = GetArmIkResult(
        "test_get_arm_ik_result", "双臂测试", "arm_test", {"left_pose": LEFT_POSE, "right_pose": RIGHT_POSE, "frame": FRAME, "ik_frame": IK_FRAME, "joint": JOINT, "output_key": OUTPUT_KEY},
    )
    node.tick_once()
    if node.status != Status.SUCCESS:
        logger.error("节点失败: %s", node.feedback_message)
        return False
    result = Result.ok(node.feedback_message, data=node.global_blackboard.get(OUTPUT_KEY))
    return check_joint_result(result, feedback=True)


def main() -> int:
    init_logging()
    if not validate_ik_input(LEFT_POSE, RIGHT_POSE, FRAME, IK_FRAME, JOINT):
        return 1
    if not isinstance(OUTPUT_KEY, str) or not OUTPUT_KEY.strip():
        logger.error("OUTPUT_KEY不能为空")
        return 1
    if not require_live_node():
        return 1
    from orchestration.shared_hardware import set_hardware_config, reset_shared_hardware

    set_hardware_config(dict(HARDWARE_CONFIG))
    try:
        # 节点内部获取共享硬件，getter从指定黑板键读取反馈。
        return 0 if test_get_arm_ik_result() else 1
    except Exception:
        logger.exception("Node 测试异常")
        return 1
    finally:
        reset_shared_hardware()


if __name__ == "__main__":
    sys.exit(main())
