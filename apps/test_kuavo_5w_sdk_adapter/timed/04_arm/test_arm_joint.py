"""
双臂关节控制测试（14D）

使用的 Adapter 方法: hardware.send_arm_joint_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_arm_joint → planner_index=8+9 (自动拆分)

测试用例说明:
- test_zero: 双臂 14 个关节全部回到 0° 零位，持续 2 秒
- test_arms_forward: 双臂前伸（肩关节 30°，其余 0°），持续 2 秒
- test_arms_up: 双臂上举（肘关节 90°，其余 0°），持续 2 秒
- test_arms_spread: 双臂向两侧展开（第二关节 90°），持续 2 秒
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

# 14 个关节: [左臂7, 右臂7]  #数据已测试，符合预期
ZERO_POSE = [0.0] * 14
ARMS_FORWARD = [-30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] * 2
ARMS_UP = [0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0] * 2
ARMS_SPREAD = [0.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0, 0.0, 0.0] 


def test_zero(hardware):
    logger.info("=== 测试：双臂零位 ===")
    result = hardware.send_arm_joint_timed(joint_angles=ZERO_POSE, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 双臂零位成功")
    else:
        logger.error(f"❌ 双臂零位失败: {result.message}")
    time.sleep(3.0)


def test_arms_forward(hardware):
    logger.info("=== 测试：双臂前伸 ===")
    result = hardware.send_arm_joint_timed(joint_angles=ARMS_FORWARD, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 双臂前伸成功")
    else:
        logger.error(f"❌ 双臂前伸失败: {result.message}")
    time.sleep(3.0)


def test_arms_up(hardware):
    logger.info("=== 测试：双臂上举 ===")
    result = hardware.send_arm_joint_timed(joint_angles=ARMS_UP, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 双臂上举成功")
    else:
        logger.error(f"❌ 双臂上举失败: {result.message}")
    time.sleep(3.0)


def test_arms_spread(hardware):
    logger.info("=== 测试：双臂展开 ===")
    result = hardware.send_arm_joint_timed(joint_angles=ARMS_SPREAD, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 双臂展开成功")
    else:
        logger.error(f"❌ 双臂展开失败: {result.message}")
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
        test_arms_forward(hardware)
        test_arms_up(hardware)
        test_arms_spread(hardware)
        test_zero(hardware)
        logger.info("🎉 双臂关节测试完成")
        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
