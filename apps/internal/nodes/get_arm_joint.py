"""nodes 层双臂测试：get_arm_joint。

按头部模板直接运行，不接受命令行参数；由操作人员执行实际硬件测试。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_joint_result, require_live_node,
)
from core.common.logger import init_logging, get_logger
from orchestration.nodes.get_arm_joint import GetArmJoint
from py_trees.common import Status
from core.domain.result import Result

logger = get_logger(__name__)

OUTPUT_KEY = "arm_joint"


def test_get_arm_joint() -> bool:
    """读取14维反馈角（deg）。"""
    logger.info("=== nodes: get_arm_joint ===")
    node = GetArmJoint(
        "test_get_arm_joint", "双臂测试", "arm_test", {"output_key": OUTPUT_KEY},
    )
    node.tick_once()
    if node.status != Status.SUCCESS:
        logger.error("节点失败: %s", node.feedback_message)
        return False
    result = Result.ok(node.feedback_message, data=node.global_blackboard.get(OUTPUT_KEY))
    return check_joint_result(result, feedback=True)


def main() -> int:
    init_logging()
    if not isinstance(OUTPUT_KEY, str) or not OUTPUT_KEY.strip():
        logger.error("OUTPUT_KEY 不能为空")
        return 1
    if not require_live_node():
        return 1
    from orchestration.shared_hardware import set_hardware_config, reset_shared_hardware

    set_hardware_config(dict(HARDWARE_CONFIG))
    try:
        # 节点内部获取共享硬件，getter从指定黑板键读取反馈。
        return 0 if test_get_arm_joint() else 1
    except Exception:
        logger.exception("Node 测试异常")
        return 1
    finally:
        reset_shared_hardware()


if __name__ == "__main__":
    sys.exit(main())
