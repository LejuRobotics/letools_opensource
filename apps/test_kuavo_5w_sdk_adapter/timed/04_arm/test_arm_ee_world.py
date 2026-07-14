"""
双臂末端世界坐标系控制测试（12D 合并）

使用的 Adapter 方法: hardware.send_arm_ee_world_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_arm_ee_world → planner_index=4+5 (自动拆分)
cmd_vec 格式: [Lx,Ly,Lz,Lyaw,Lpitch,Lroll, Rx,Ry,Rz,Ryaw,Rpitch,Rroll] (位置：米，角度：用户单位)

测试用例说明:
- test_default: 双臂末端回到默认位姿（左 x=0.3m, 右 x=0.3m），持续 3 秒
- test_forward: 双臂末端同时向前伸到 x=0.5m，持续 3 秒
- test_up: 双臂末端同时向上抬到 z=0.7m，持续 3 秒

前置条件（setup_arm_control 中完成）:
1. 设置 MPC 模式为 ARM_ONLY
2. 设置笛卡尔跟踪焦点为末端(EE)
3. 重置手臂到初始位置（mode 1），再切换到外部控制器模式（mode 2）
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
    if result.success:
        logger.info(f"✅ 默认位姿成功")
    else:
        logger.error(f"❌ 默认位姿失败: {result.message}")
    time.sleep(4.0)


def test_forward(hardware):
    logger.info("=== 测试：双臂末端前伸 (WORLD) ===")
    result = hardware.send_arm_ee_world_timed(
        left_pose=FORWARD_LEFT, right_pose=FORWARD_RIGHT, desire_time=3.0
    )
    if result.success:
        logger.info(f"✅ 双臂前伸成功")
    else:
        logger.error(f"❌ 双臂前伸失败: {result.message}")
    time.sleep(4.0)


def test_up(hardware):
    logger.info("=== 测试：双臂末端上抬 (WORLD) ===")
    result = hardware.send_arm_ee_world_timed(
        left_pose=UP_LEFT, right_pose=UP_RIGHT, desire_time=3.0
    )
    if result.success:
        logger.info(f"✅ 双臂上抬成功")
    else:
        logger.error(f"❌ 双臂上抬失败: {result.message}")
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
        logger.info("🎉 双臂末端世界系测试完成")
        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
