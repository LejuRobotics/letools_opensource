"""双臂末端局部位姿节点测试（编排层）

被测对象: orchestration.nodes.arm_ee_local_timed_move.ArmEeLocalTimedMove
节点生命周期: initialise() → 循环 update() → SUCCESS/FAILURE
底层路径: 节点 get_shared_hardware() → ArmEEDualTimedSkill
         → hardware.send_arm_ee_local_timed()
         → timed_command_mixin → TimedCmdManager（需 whitelist=['timed']）

测试流程对齐 test_arm_ee_local.py:
- 一个 sequence 节点内部依次执行 default → forward → default
- 执行前请先把折叠臂apps/test_kuavo_5w_sdk_adapter/timed/03_leg/test_leg_joint.py运行到[42, -25, 0, 0]这个高度。
- 节点主动设置 ARM_EE_ONLY，序列结束后才释放手臂控制
"""

import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import get_logger, init_logging
from core.domain.enums import MPCControlMode

init_logging()

from apps.test_kuavo_5w_orchestration._scaffold import (
    run_node,
    set_hardware_config,
)
from orchestration.nodes.arm_ee_local_timed_move import ArmEeLocalTimedMove
from orchestration.shared_hardware import reset_shared_hardware

logger = get_logger(__name__)


def _build_sequence_node():
    return ArmEeLocalTimedMove(
        "t_sequence",
        "双臂局部位姿序列",
        "ns",
        {
            "pose_name": "sequence",
            "desire_time": 3.0,
            "prepare_mpc_control": True,
            "prepare_arm_control": True,
            "release_arm_control": True,
        },
    )


def test_sequence():
    """单节点完成 default → forward，并在两段之间保持外部控制。"""
    node = _build_sequence_node()
    return run_node(node)


def main():
    set_hardware_config(whitelist=["timed"])
    all_passed = False
    node = _build_sequence_node()
    try:
        all_passed = run_node(node)
        if all_passed:
            logger.info("双臂末端局部位姿（编排层）测试完成")
        else:
            logger.error("部分测试失败")
    finally:
        try:
            if node._hardware is not None:
                result = node._hardware.set_mpc_mode(MPCControlMode.NO_CONTROL)
                if not result.success:
                    logger.warning("释放 MPC 控制失败: %s", result.message)
        finally:
            reset_shared_hardware()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
