"""
右臂关节控制测试（7D）

使用的 Adapter 方法: hardware.send_right_arm_joint_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_right_arm_joint → planner_index=9

测试用例说明:
- test_zero: 右臂 7 个关节全部回到 0° 零位，持续 2 秒
- test_forward: 右臂前伸（肩关节 30°），持续 2 秒
- test_up: 右臂上举（肘关节 90°），持续 2 秒
"""
import sys
import time
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()
from adapters.hardware.factory import HardwareFactory
from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown

logger = get_logger(__name__)

ZERO = [0.0] * 7
FORWARD = [-30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
UP = [0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0]


def test_zero(hardware):
    logger.info("=== 测试：右臂零位 ===")
    result = hardware.send_right_arm_joint_timed(joint_angles=ZERO, desire_time=2.0)
    if result.success:
        logger.info("✅ 右臂零位成功")
    else:
        logger.error(f"❌ 右臂零位失败: {result.message}")
    time.sleep(3.0)


def test_forward(hardware):
    logger.info("=== 测试：右臂前伸 ===")
    result = hardware.send_right_arm_joint_timed(joint_angles=FORWARD, desire_time=2.0)
    if result.success:
        logger.info("✅ 右臂前伸成功")
    else:
        logger.error(f"❌ 右臂前伸失败: {result.message}")
    time.sleep(3.0)


def test_up(hardware):
    logger.info("=== 测试：右臂上举 ===")
    result = hardware.send_right_arm_joint_timed(joint_angles=UP, desire_time=2.0)
    if result.success:
        logger.info("✅ 右臂上举成功")
    else:
        logger.error(f"❌ 右臂上举失败: {result.message}")
    time.sleep(3.0)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'sdk_managers_whitelist': ['timed'],
        'skip_end_effector': True,
        'skip_camera': True,
        'skip_state_manager': True,
        'skip_force_publishers': True,
    })
    try:
        hardware.initialize()
        factory_setup(hardware, need_arm=True)
        test_zero(hardware)
        test_forward(hardware)
        test_up(hardware)
        test_zero(hardware)
        logger.info("🎉 右臂关节测试完成")
        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
