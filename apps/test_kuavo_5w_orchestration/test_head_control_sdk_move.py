"""头部控制节点测试（编排层）

被测对象: orchestration.nodes.head_control_sdk_move.HeadControlSdkMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点内部 get_shared_hardware() → HeadControlSdkSkill → hardware.control_head_sdk()

节点从 self.params 读取 yaw_deg/pitch_deg，故构造时传 params dict。

测试用例说明:
- test_center: 头部回到正前方（yaw=0°, pitch=0°）
- test_look_left: 头部左转 30°
- test_look_right: 头部右转 30°
- test_look_up: 头部上抬 20°
- test_look_down: 头部下低 20°
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()

from orchestration.nodes.head_control_sdk_move import HeadControlSdkMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, node_teardown, run_node)

logger = get_logger(__name__)


def test_center():
    """头部居中（编排层节点）"""
    node = HeadControlSdkMove("t_center", "头部居中", "ns",
                              {"yaw_deg": 0.0, "pitch_deg": 0.0})
    return run_node(node)


def test_look_left():
    """左转 30°（编排层节点）"""
    node = HeadControlSdkMove("t_left", "左转", "ns",
                              {"yaw_deg": 30.0, "pitch_deg": 0.0})
    return run_node(node)


def test_look_right():
    """右转 30°（编排层节点）"""
    node = HeadControlSdkMove("t_right", "右转", "ns",
                              {"yaw_deg": -30.0, "pitch_deg": 0.0})
    return run_node(node)


def test_look_up():
    """上抬 20°（编排层节点）"""
    node = HeadControlSdkMove("t_up", "上抬", "ns",
                              {"yaw_deg": 0.0, "pitch_deg": 20.0})
    return run_node(node)


def test_look_down():
    """下低 20°（编排层节点）"""
    node = HeadControlSdkMove("t_down", "下低", "ns",
                              {"yaw_deg": 0.0, "pitch_deg": -20.0})
    return run_node(node)


def main():
    set_hardware_config(whitelist=['low'])  # 头部控制只需要 low 管理器
    try:
        node_setup(need_arm=False)
        all_passed = True
        all_passed &= test_center()
        all_passed &= test_look_left()
        all_passed &= test_look_right()
        all_passed &= test_look_up()
        all_passed &= test_look_down()
        if all_passed:
            logger.info("🎉 头部控制（编排层）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")
    finally:
        node_teardown(need_arm=False)
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
