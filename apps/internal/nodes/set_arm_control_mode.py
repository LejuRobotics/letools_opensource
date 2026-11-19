"""nodes 层手臂模式测试：设置手臂控制模式。

由操作人员填写顶部参数后直接运行，不接受命令行参数，不等待到位。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from apps.internal._arm_test import (
    HARDWARE_CONFIG, check_mode_set_result, validate_control_mode, require_live_node,
)
from core.common.logger import init_logging, get_logger
from orchestration.nodes.set_arm_control_mode import SetArmControlMode
from py_trees.common import Status
from core.domain.result import Result

logger = get_logger(__name__)

# 人工确认现场安全后填写；模式1可能触发手臂运动，重复设置不保证再次重置。
CONTROL_MODE = None  # 0固定、1AutoSwing、2外部控制。


def test_set_arm_control_mode() -> bool:
    """设置手臂模式，不等待到位或自动恢复。"""
    logger.info("=== nodes: set_arm_control_mode ===")
    node = SetArmControlMode(
        "test_set_arm_control_mode", "手臂模式测试", "arm_mode_test",
        {"control_mode": CONTROL_MODE},
    )
    node.tick_once()
    if node.status != Status.SUCCESS:
        logger.error("节点失败: %s", node.feedback_message)
        return False
    result = Result.ok(node.feedback_message)
    return check_mode_set_result(result)


def main() -> int:
    init_logging()
    if not validate_control_mode(CONTROL_MODE):
        return 1
    if not require_live_node():
        return 1
    from orchestration.shared_hardware import set_hardware_config, reset_shared_hardware

    set_hardware_config(dict(HARDWARE_CONFIG))
    try:
        # 节点内部获取共享硬件并触发初始化。
        return 0 if test_set_arm_control_mode() else 1
    except Exception:
        logger.exception("Node 测试异常")
        return 1
    finally:
        reset_shared_hardware()


if __name__ == "__main__":
    sys.exit(main())
