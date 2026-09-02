"""底盘相对位姿节点测试（编排层）

被测对象: orchestration.nodes.base_pose_local_move.BasePoseLocalMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点内部 get_shared_hardware() → BasePoseLocalSkill → hardware.send_base_pose(frame=LOCAL)

节点从 self.params 读取 x/y/yaw（frame 固定 LOCAL）。

测试用例说明:
- test_move_forward: 前进 0.3m
- test_move_backward: 后退 0.3m
- test_rotate: 原地旋转 15°
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.base_pose_local_move import BasePoseLocalMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_move_forward():
    """前进 0.3m（编排层节点）"""
    node = BasePoseLocalMove("t_fwd", "前进", "ns",
                             {"x": 0.3, "y": 0.0, "yaw": 0.0})
    return run_node(node)


def test_move_backward():
    """后退 0.3m（编排层节点）"""
    node = BasePoseLocalMove("t_bwd", "后退", "ns",
                             {"x": -0.3, "y": 0.0, "yaw": 0.0})
    return run_node(node)


def test_rotate():
    """原地旋转 15°（编排层节点）"""
    node = BasePoseLocalMove("t_rot", "旋转", "ns",
                             {"x": 0.0, "y": 0.0, "yaw": 15.0})
    return run_node(node)


def main():
    set_hardware_config(whitelist=['low'])  # 底盘控制只需要 low 管理器
    try:
        node_setup(need_arm=False)
        all_passed = True
        all_passed &= test_move_forward()
        all_passed &= test_move_backward()
        all_passed &= test_rotate()
        if all_passed:
            logger.info("🎉 底盘相对位姿（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=False)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
