"""nodes 层双臂关节测试：发送单个关节目标。

由操作人员填写顶部参数后直接运行，不接受命令行参数，不等待到位。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_joint_result, validate_joint_target, require_live_node,
)
from core.common.logger import init_logging, get_logger
from orchestration.nodes.set_arm_joint import SetArmJoint
from py_trees.common import Status
from core.domain.result import Result

logger = get_logger(__name__)

# 操作人员确认关节范围及现场安全后填写，单位：deg。
TARGET_JOINT = []  # 14个关节角：左臂7个、右臂7个。


def test_set_arm_joint() -> bool:
    """发送14维双臂关节角。"""
    logger.info("=== nodes: set_arm_joint ===")
    node = SetArmJoint(
        "test_set_arm_joint", "双臂关节测试", "arm_joint_test",
        {"joint": TARGET_JOINT},
    )
    node.tick_once()
    if node.status != Status.SUCCESS:
        logger.error("节点失败: %s", node.feedback_message)
        return False
    result = Result.ok(node.feedback_message)
    return check_joint_result(result)


def main() -> int:
    init_logging()
    if not validate_joint_target(TARGET_JOINT):
        return 1
    if not require_live_node():
        return 1
    from orchestration.shared_hardware import set_hardware_config, reset_shared_hardware

    set_hardware_config(dict(HARDWARE_CONFIG))
    try:
        # 节点内部获取共享硬件并触发初始化。
        return 0 if test_set_arm_joint() else 1
    except Exception:
        logger.exception("Node 测试异常")
        return 1
    finally:
        reset_shared_hardware()


if __name__ == "__main__":
    sys.exit(main())
