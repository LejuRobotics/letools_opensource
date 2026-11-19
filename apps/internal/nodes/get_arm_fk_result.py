"""nodes 层双臂FK计算测试，不发送运动；顶部填写参数后直接运行。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, require_live_node, check_pose_result, validate_query, validate_joint_target,
)
from core.common.logger import init_logging, get_logger
from orchestration.nodes.get_arm_fk_result import GetArmFkResult
from py_trees.common import Status
from core.domain.result import Result

logger = get_logger(__name__)

# 纯计算指定关节的末端，不读取当前关节，不发送运动。
JOINT = []  # 14维关节角（deg），左臂7个、右臂7个。
FRAME = "local"
FK_FRAME = None  # 模型基准对应整机TF frame，必须人工确认。

OUTPUT_KEY = "arm_fk_result"


def test_get_arm_fk_result() -> bool:
    """计算给定关节的双臂末端（m/deg），不读取实时关节。"""
    logger.info("=== nodes: get_arm_fk_result ===")
    node = GetArmFkResult(
        "test_get_arm_fk_result", "双臂测试", "arm_test", {"joint": JOINT, "frame": FRAME, "fk_frame": FK_FRAME, "output_key": OUTPUT_KEY},
    )
    node.tick_once()
    if node.status != Status.SUCCESS:
        logger.error("节点失败: %s", node.feedback_message)
        return False
    result = Result.ok(node.feedback_message, data=node.global_blackboard.get(OUTPUT_KEY))
    return check_pose_result(result, frame=FRAME, source="fk")


def main() -> int:
    init_logging()
    if not validate_joint_target(JOINT):
        return 1
    if not validate_query(FRAME, "fk", FK_FRAME):
        return 1
    if not isinstance(OUTPUT_KEY, str) or not OUTPUT_KEY.strip():
        logger.error("OUTPUT_KEY 不能为空")
        return 1
    if not require_live_node():
        return 1
    from orchestration.shared_hardware import set_hardware_config, reset_shared_hardware

    set_hardware_config(dict(HARDWARE_CONFIG))
    try:
        # 节点内部获取共享硬件，getter从指定黑板键读取反馈。
        return 0 if test_get_arm_fk_result() else 1
    except Exception:
        logger.exception("Node 测试异常")
        return 1
    finally:
        reset_shared_hardware()


if __name__ == "__main__":
    sys.exit(main())
