"""
下肢关节控制测试（TimedCmd 路径）

使用的 Adapter 方法: hardware.send_leg_joint_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_leg_joint → planner_index=3
参考源脚本: case_wheel_leg_move.py（pick_place_box/）

测试用例说明:
源脚本使用关节 1-3 基准值 [14.90, -32.01, 18.03]，只变化第 4 关节。
- test_zero_position: 下肢 4 个关节回到 0° 零位
- test_source_pose_1: [14.90, -32.01, 18.03, 0.0]°（源脚本关键点 1/4）
- test_source_pose_2: [14.90, -32.01, 18.03, 30.0]°（源脚本关键点 2）
- test_source_pose_3: [14.90, -32.01, 18.03, -30.0]°（源脚本关键点 3）
- test_reset: 恢复零位
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

# 源脚本 case_wheel_leg_move.py 中验证过的关节 1-3 基准值（度）
SOURCE_BASE_JOINTS = [14.90, -32.01, 18.03]


def test_zero_position(hardware):
    """零位"""
    logger.info("=== 测试：下肢零位 ===")
    result = hardware.send_leg_joint_timed(joint_angles=[0.0, 0.0, 0.0, 0.0], desire_time=2.0)
    if result.success:
        logger.info(f"✅ 零位成功")
    else:
        logger.error(f"❌ 零位失败: {result.message}")
    time.sleep(3.0)


def test_source_pose_1(hardware):
    """源脚本关键点 1/4：关节4=0°"""
    pose = SOURCE_BASE_JOINTS + [0.0]
    logger.info(f"=== 测试：源脚本姿态 {pose}° ===")
    result = hardware.send_leg_joint_timed(joint_angles=pose, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 姿态 1 成功")
    else:
        logger.error(f"❌ 姿态 1 失败: {result.message}")
    time.sleep(3.0)


def test_source_pose_2(hardware):
    """源脚本关键点 2：关节4=30°"""
    pose = SOURCE_BASE_JOINTS + [30.0]
    logger.info(f"=== 测试：源脚本姿态 {pose}° ===")
    result = hardware.send_leg_joint_timed(joint_angles=pose, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 姿态 2 成功")
    else:
        logger.error(f"❌ 姿态 2 失败: {result.message}")
    time.sleep(3.0)


def test_source_pose_3(hardware):
    """源脚本关键点 3：关节4=-30°"""
    pose = SOURCE_BASE_JOINTS + [-30.0]
    logger.info(f"=== 测试：源脚本姿态 {pose}° ===")
    result = hardware.send_leg_joint_timed(joint_angles=pose, desire_time=2.0)
    if result.success:
        logger.info(f"✅ 姿态 3 成功")
    else:
        logger.error(f"❌ 姿态 3 失败: {result.message}")
    time.sleep(3.0)


def test_reset(hardware):
    """恢复零位"""
    logger.info("=== 测试：恢复零位 ===")
    result = hardware.send_leg_joint_timed(joint_angles=[0.0, 0.0, 0.0, 0.0], desire_time=2.0)
    if result.success:
        logger.info(f"✅ 恢复成功")
    else:
        logger.error(f"❌ 恢复失败: {result.message}")
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
        factory_setup(hardware, need_arm=False, need_torso_reset=True)
        test_zero_position(hardware)
        test_source_pose_1(hardware)
        test_source_pose_2(hardware)
        test_source_pose_3(hardware)
        test_reset(hardware)
        logger.info("🎉 下肢关节测试完成")
    finally:
        factory_teardown(hardware, need_arm=False)
        hardware.shutdown()


if __name__ == "__main__":
    main()
