"""
Ruckig 规划器参数设置测试

使用的 Adapter 方法: hardware.set_ruckig_params_timed() + hardware.send_base_velocity_timed()
底层路径: _timed_cmd_manager.set_ruckig_planner_params → ROS 服务 /mobile_manipulator_set_ruckig_planner_params

测试用例说明:
- test_set_ruckig_params: 为底盘规划器（planner_index=0）设置 Ruckig 参数（速度/加速度/加加速度限制）
- test_motion_after_ruckig: 设置 Ruckig 参数后执行前进 0.3m，验证运动效果
"""
import sys
import time
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()
from adapters.hardware.factory import HardwareFactory
from core.domain.enums import FrameType

logger = get_logger(__name__)


def test_set_ruckig_params(hardware):
    """设置底盘规划器的 Ruckig 参数"""
    logger.info("=== 测试：设置 Ruckig 参数 (planner_index=0) ===")
    result = hardware.set_ruckig_params_timed(
        planner_index=0,
        is_sync=True,
        velocity_max=[0.2, 0.2, 0.2],
        acceleration_max=[2.0, 2.0, 1.5],
        jerk_max=[20.0, 15.0, 12.0],
    )
    if result.success:
        logger.info("✅ Ruckig 参数设置成功")
    else:
        logger.error(f"❌ Ruckig 参数设置失败: {result.message}")
    time.sleep(1.0)


def test_motion_after_ruckig(hardware):
    """设置 Ruckig 后执行运动验证效果"""
    logger.info("=== 测试：设置 Ruckig 后执行前进 0.3m ===")
    result = hardware.send_base_velocity_timed(vx=0.3, vy=0.0, vyaw=0.0, frame=FrameType.WORLD)
    if result.success:
        logger.info(f"✅ 前进成功，实际时间: {result.data.get('actual_time', 'N/A')}s")
    else:
        logger.error(f"❌ 前进失败: {result.message}")
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
        test_set_ruckig_params(hardware)
        test_motion_after_ruckig(hardware)
        logger.info("🎉 Ruckig 参数测试完成")
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
