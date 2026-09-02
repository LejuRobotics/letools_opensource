"""
双臂末端世界坐标系控制测试（12D 合并）

使用的 Adapter 方法: hardware.send_arm_ee_world_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_arm_ee_world → planner_index=4+5 (自动拆分)

测试用例说明:
- test_default: 双臂末端回到默认位姿（左 x=0.3m, 右 x=0.3m），持续 3 秒
- test_forward: 双臂末端同时向前伸到 x=0.5m，持续 3 秒
- test_up: 双臂末端同时向上抬到 z=0.7m，持续 3 秒
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


# 位姿格式: [x, y, z, yaw, pitch, roll]（位置：米，角度：度）

# 默认手臂末端位姿（近似初始位置）
DEFAULT_LEFT = [0.3, 0.25, 0.5, 0, 0, 0]
DEFAULT_RIGHT = [0.3, -0.25, 0.5, 0, 0, 0]

# 双臂前伸
FORWARD_LEFT = [0.5, 0.25, 0.5, 0, 0, 0]
FORWARD_RIGHT = [0.5, -0.25, 0.5, 0, 0, 0]

# 双臂上抬
UP_LEFT = [0.3, 0.25, 0.7, 0, 0, 0]
UP_RIGHT = [0.3, -0.25, 0.7, 0, 0, 0]


def test_default(hardware):
    logger.info("=== 测试：双臂末端默认位姿 (WORLD) ===")
    result = hardware.send_arm_ee_world_timed(
        left_pose=DEFAULT_LEFT, right_pose=DEFAULT_RIGHT, desire_time=3.0
    )
    actual_time = float((result.data or {}).get("actual_time", 3.0))
    time.sleep(actual_time)
    if result.success:
        logger.info("✅ 默认位姿成功")
    else:
        logger.error(f"❌ 默认位姿失败: {result.message}")
    time.sleep(0.5)


def test_forward(hardware):
    logger.info("=== 测试：双臂末端前伸 (WORLD) ===")
    result = hardware.send_arm_ee_world_timed(
        left_pose=FORWARD_LEFT, right_pose=FORWARD_RIGHT, desire_time=3.0
    )
    actual_time = float((result.data or {}).get("actual_time", 3.0))
    time.sleep(actual_time)
    if result.success:
        logger.info("✅ 双臂前伸成功")
    else:
        logger.error(f"❌ 双臂前伸失败: {result.message}")
    time.sleep(0.5)


def test_up(hardware):
    logger.info("=== 测试：双臂末端上抬 (WORLD) ===")
    result = hardware.send_arm_ee_world_timed(
        left_pose=UP_LEFT, right_pose=UP_RIGHT, desire_time=3.0
    )
    actual_time = float((result.data or {}).get("actual_time", 3.0))
    time.sleep(actual_time)
    if result.success:
        logger.info("✅ 双臂上抬成功")
    else:
        logger.error(f"❌ 双臂上抬失败: {result.message}")
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
        factory_setup(hardware, need_arm_reset=False, need_torso_reset=True, focus_ee=False, focus_z=False, mpc_mode=MPCControlMode.ARM_EE_ONLY)

        test_default(hardware)
        test_forward(hardware)
        test_up(hardware)
        test_default(hardware)

    finally:
        factory_teardown(hardware, need_arm_reset=True, need_torso_reset=False)
        logger.info("🎉 双臂末端世界系测试完成")
        hardware.shutdown()


if __name__ == "__main__":
    main()
