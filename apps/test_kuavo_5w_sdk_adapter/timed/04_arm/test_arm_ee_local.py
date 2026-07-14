"""
双臂末端局部坐标系控制测试（12D 合并）

使用的 Adapter 方法: hardware.send_arm_ee_local_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_arm_ee_local → planner_index=6+7 (自动拆分)
cmd_vec 格式: [Lx,Ly,Lz,Lyaw,Lpitch,Lroll, Rx,Ry,Rz,Ryaw,Rpitch,Rroll] (位置：米，角度：用户单位)

测试用例说明:
- test_default: 双臂末端回到默认位姿（左 x=0.3m, 右 x=0.3m），持续 3 秒
- test_forward: 双臂末端同时向前伸到 x=0.5m，持续 3 秒
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

# 局部系下的默认位姿（手臂在身侧，无旋转）
DEFAULT_LEFT = [0.5, 0.4, 0.7, 0, 0, 0]
DEFAULT_RIGHT = [0.5, -0.4, 0.7, 0, 0, 0]

# 双臂前伸（pitch=-90°）
FORWARD_LEFT = [0.5, 0.2, 0.7, 0, -90, 0]
FORWARD_RIGHT = [0.5, -0.2, 0.7, 0, -90, 0]


def test_default(hardware):
    logger.info("=== 测试：双臂末端默认位姿 (LOCAL) ===")
    result = hardware.send_arm_ee_local_timed(
        left_pose=DEFAULT_LEFT, right_pose=DEFAULT_RIGHT, desire_time=3.0
    )
    if result.success:
        logger.info(f"✅ 默认位姿成功")
    else:
        logger.error(f"❌ 默认位姿失败: {result.message}")
    time.sleep(4.0)


def test_forward(hardware):
    logger.info("=== 测试：双臂末端前伸 (LOCAL) ===")
    result = hardware.send_arm_ee_local_timed(
        left_pose=FORWARD_LEFT, right_pose=FORWARD_RIGHT, desire_time=3.0
    )
    if result.success:
        logger.info(f"✅ 双臂前伸成功")
    else:
        logger.error(f"❌ 双臂前伸失败: {result.message}")
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
        test_default(hardware)
        logger.info("🎉 双臂末端局部系测试完成")
        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
