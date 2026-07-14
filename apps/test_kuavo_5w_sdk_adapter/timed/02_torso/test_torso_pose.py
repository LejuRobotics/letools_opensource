"""
躯干位姿控制测试

使用的 Adapter 方法: hardware.send_torso_pose_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_torso_pose → planner_index=2

测试用例说明:
- test_raise_torso: 躯干垂直抬升 0.05m（z=+0.05），持续 2 秒
- test_rotate_torso: 躯干绕 z 轴旋转 0.2rad（yaw=0.2），持续 2 秒
- test_pitch_torso: 躯干俯仰 0.1rad（pitch=0.1），持续 2 秒
- test_reset_torso: 躯干恢复到初始位姿（所有参数归零），持续 2 秒
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


def test_raise_torso(hardware):
    """抬升躯干"""
    logger.info("=== 测试：抬升躯干 z=0.30m ===")
    result = hardware.send_torso_pose_timed(x=0.0, z=1, yaw=0.0, pitch=0.0, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 抬升成功")
    else:
        logger.error(f"❌ 抬升失败: {result.message}")
    time.sleep(3.0)


def test_rotate_torso(hardware):
    """旋转躯干"""
    logger.info("=== 测试：旋转躯干 yaw=0.4rad ===")
    result = hardware.send_torso_pose_timed(x=0.0, z=1, yaw=0.4, pitch=0.0, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 旋转成功")
    else:
        logger.error(f"❌ 旋转失败: {result.message}")
    time.sleep(3.0)


def test_pitch_torso(hardware):
    """俯仰躯干"""
    logger.info("=== 测试：俯仰躯干 pitch=0.4rad ===")
    result = hardware.send_torso_pose_timed(x=0.0, z=1, yaw=0.4, pitch=0.5, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 俯仰成功")
    else:
        logger.error(f"❌ 俯仰失败: {result.message}")
    time.sleep(3.0)


def test_reset_torso(hardware):
    """恢复躯干初始位姿"""
    logger.info("=== 测试：恢复初始位姿 ===")
    result = hardware.send_torso_pose_timed(x=0.0, z=0.0, yaw=0.0, pitch=0.0, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 恢复成功")
    else:
        logger.error(f"❌ 恢复失败: {result.message}")
    time.sleep(3.0)


def main():
    hardware = HardwareFactory.create_hardware(
        config={
            'robot_type': 'leju_wheeled',
            'angle_unit': 'rad',
            'sdk_managers_whitelist': ['timed'],
            'skip_end_effector': True,
            'skip_camera': True,
            'skip_state_manager': True,
            'skip_force_publishers': True,
        }
    )
    try:
        hardware.initialize()
        factory_setup(hardware, need_arm=False, need_torso_reset=True)
        test_raise_torso(hardware)
        test_rotate_torso(hardware)
        test_pitch_torso(hardware)
        test_reset_torso(hardware)
        logger.info("🎉 躯干位姿测试完成")
    finally:
        factory_teardown(hardware, need_arm=False)
        hardware.shutdown()


if __name__ == "__main__":
    main()
