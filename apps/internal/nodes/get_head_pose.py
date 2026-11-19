"""nodes 层头部测试：获取头部姿态。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._head_test import HARDWARE_CONFIG, check_result, require_live_node
from core.common.logger import init_logging, get_logger
from core.domain.result import Result
from orchestration.nodes.get_head_pose import GetHeadPose
from py_trees.common import Status

logger = get_logger(__name__)

OUTPUT_KEY = "head_pose"


def test_get_head_pose() -> bool:
    """获取并检查头部姿态 yaw/pitch（deg）。"""
    logger.info("=== nodes: get_head_pose ===")
    node = GetHeadPose("test_get_head_pose", "头部测试", "head_test", {"output_key": OUTPUT_KEY})
    node.tick_once()
    if node.status != Status.SUCCESS:
        logger.error("节点失败: %s", node.feedback_message)
        return False
    result = Result.ok(node.feedback_message, data=node.global_blackboard.get(OUTPUT_KEY))
    return check_result(result, feedback=True)


def main() -> int:
    init_logging()
    if not require_live_node():
        return 1
    from orchestration.shared_hardware import set_hardware_config, reset_shared_hardware

    set_hardware_config(dict(HARDWARE_CONFIG))
    try:
        # 节点内部获取共享硬件并触发初始化。
        return 0 if test_get_head_pose() else 1
    except Exception:
        logger.exception("Node 测试异常")
        return 1
    finally:
        reset_shared_hardware()


if __name__ == "__main__":
    sys.exit(main())