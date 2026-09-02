"""
折叠臂关节控制测试（TimedCmd 路径）

使用的 Adapter 方法: hardware.send_leg_joint_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_leg_joint → planner_index=3
参考源脚本: case_wheel_leg_move.py（pick_place_box/）

测试用例说明:
关节顺序: [knee_joint, leg_joint, waist_pitch_joint, waist_yaw_joint]，单位为度。
- test_zero_position: 4 个关节运动到 0° 目标
- test_leg_joints_1: 运动到第 1 组折叠臂关节目标
- test_leg_joints_2: 在第 1 组基础上将 waist_yaw_joint 目标增加 20°
- test_leg_joints_3: 在第 2 组基础上将 waist_pitch_joint 目标增加 20°
- 最后再次运动到 0° 关节目标
"""
import sys
from pathlib import Path
project_root = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()
logger = get_logger(__name__)

import time
from core.domain.enums import MPCControlMode
from adapters.hardware.factory import HardwareFactory
from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown


# 折叠臂关节目标，顺序为 [knee, leg, waist_pitch, waist_yaw]，单位：deg
LEG_JOINTS_0 = [0.0, 0.0, 0.0, 0.0]  # 四关节 0° 目标，不是 reset_torso_to_initial() 的复位目标
LEG_JOINTS_1 = [29.00, -41.11, 12.21, 0.0]  # 对应折叠臂抬升、前伸后记录的关节目标
LEG_JOINTS_2 = [29.00, -41.11, 12.21, 20.0]  # waist_yaw 由 0° 变为 +20°
LEG_JOINTS_3 = [29.00, -41.11, 32.21, 20.0]  # waist_pitch 由 12.21° 变为 32.21°

def test_zero_position(hardware):
    """全零位"""
    logger.info("=== 测试：折叠臂零位 ===")
    result = hardware.send_leg_joint_timed(joint_angles=LEG_JOINTS_0, desire_time=3.0)
    actual_time = float((result.data or {}).get("actual_time", 3.0))
    time.sleep(actual_time)
    if result.success:
        logger.info(f"✅ 零位成功")
    else:
        logger.error(f"❌ 零位失败: {result.message}")
    time.sleep(0.5)


def test_leg_joints_1(hardware):
    """第 1 组折叠臂关节角"""
    logger.info(f"=== 测试：第 1 组折叠臂关节角 {LEG_JOINTS_1}° ===")
    result = hardware.send_leg_joint_timed(joint_angles=LEG_JOINTS_1, desire_time=2.0)
    actual_time = float((result.data or {}).get("actual_time", 2.0))
    time.sleep(actual_time)
    if result.success:
        logger.info(f"✅ 第 1 组折叠臂关节角成功")
    else:
        logger.error(f"❌ 第 1 组折叠臂关节角失败: {result.message}")
    time.sleep(0.5)


def test_leg_joints_2(hardware):
    """第 2 组折叠臂关节角"""
    logger.info(f"=== 测试：第 2 组折叠臂关节角 {LEG_JOINTS_2}° ===")
    result = hardware.send_leg_joint_timed(joint_angles=LEG_JOINTS_2, desire_time=2.0)
    actual_time = float((result.data or {}).get("actual_time", 2.0))
    time.sleep(actual_time)
    if result.success:
        logger.info(f"✅ 第 2 组折叠臂关节角成功")
    else:
        logger.error(f"❌ 第 2 组折叠臂关节角失败: {result.message}")
    time.sleep(0.5)


def test_leg_joints_3(hardware):
    """第 3 组折叠臂关节角"""
    logger.info(f"=== 测试：第 3 组折叠臂关节角 {LEG_JOINTS_3}° ===")
    result = hardware.send_leg_joint_timed(joint_angles=LEG_JOINTS_3, desire_time=3.0)
    actual_time = float((result.data or {}).get("actual_time", 2.0))
    time.sleep(actual_time)
    if result.success:
        logger.info(f"✅ 第 3 组折叠臂关节角成功")
    else:
        logger.error(f"❌ 第 3 组折叠臂关节角失败: {result.message}")
    time.sleep(0.5)


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
        factory_setup(hardware, need_arm_reset=False, need_torso_reset=False, focus_ee=False, focus_z=False, mpc_mode=MPCControlMode.ARM_ONLY)

        test_zero_position(hardware)
        test_leg_joints_1(hardware)
        test_leg_joints_2(hardware)
        test_leg_joints_3(hardware)
        test_zero_position(hardware)

    finally:
        factory_teardown(hardware, need_arm_reset=False,need_torso_reset=False)
        logger.info("🎉 折叠臂关节测试完成")
        hardware.shutdown()


if __name__ == "__main__":
    main()
