"""
左臂末端世界坐标系控制测试（单臂 6D）

使用的 Adapter 方法: hardware.send_left_arm_ee_world_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_left_arm_ee_world → planner_index=4

测试用例说明:
- test_default: 左臂末端回到默认位姿（x=0.3m, y=0.25m, z=0.5m），持续 3 秒
- test_forward: 左臂末端向前伸到 x=0.5m，持续 3 秒
- test_up: 左臂末端向上抬到 z=0.7m，持续 3 秒
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


# 位姿格式: [x, y, z, yaw, pitch, roll]（位置：米，角度：度）
DEFAULT = [0.3, 0.25, 0.5, 0, 0, 0]
FORWARD = [0.5, 0.25, 0.5, 0, 0, 0]
UP = [0.3, 0.25, 0.7, 0, 0, 0]


def test_default(hardware):
    logger.info("=== 测试：左臂末端默认位姿 (WORLD) ===")
    result = hardware.send_left_arm_ee_world_timed(pose=DEFAULT, desire_time=3.0)
    if result.success:
        logger.info("✅ 左臂默认位姿成功")
    else:
        logger.error(f"❌ 左臂默认位姿失败: {result.message}")
    time.sleep(4.0)


def test_forward(hardware):
    logger.info("=== 测试：左臂末端前伸 (WORLD) ===")
    result = hardware.send_left_arm_ee_world_timed(pose=FORWARD, desire_time=3.0)
    if result.success:
        logger.info("✅ 左臂前伸成功")
    else:
        logger.error(f"❌ 左臂前伸失败: {result.message}")
    time.sleep(4.0)


def test_up(hardware):
    logger.info("=== 测试：左臂末端上抬 (WORLD) ===")
    result = hardware.send_left_arm_ee_world_timed(pose=UP, desire_time=3.0)
    if result.success:
        logger.info("✅ 左臂上抬成功")
    else:
        logger.error(f"❌ 左臂上抬失败: {result.message}")
    time.sleep(4.0)


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
        test_default(hardware)
        test_forward(hardware)
        test_up(hardware)
        test_default(hardware)
        logger.info("🎉 左臂末端世界系测试完成")
        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
