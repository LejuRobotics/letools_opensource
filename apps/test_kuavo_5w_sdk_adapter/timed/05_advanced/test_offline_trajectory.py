"""
离线轨迹测试

使用的 Adapter 方法: hardware.set_offline_trajectory_timed() + hardware.enable_offline_trajectory_timed()
底层路径: _timed_cmd_manager.set_offline_trajectory → ROS 服务 /mobile_manipulator_timed_offline_traj

planner_index 与 cmd_vec 维度:
- 0 (左臂): 6 维 [x, y, z, yaw, pitch, roll]
- 1 (右臂): 6 维 [x, y, z, yaw, pitch, roll]
- 2 (躯干): 4 维 [x, y, z, yaw]

测试用例说明:
- test_set_trajectory: 设置离线轨迹（左臂 4 个关键点，6D cmd_vec）
- test_enable: 启用离线轨迹，机器人开始自动执行预设轨迹
- test_disable: 禁用离线轨迹，机器人停止自动执行
"""
import sys
import time
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()
from adapters.hardware.factory import HardwareFactory

logger = get_logger(__name__)


def test_set_trajectory(hardware):
    """设置离线轨迹（左臂 4 个关键点，6D cmd_vec: [x, y, z, yaw, pitch, roll]）"""
    logger.info("=== 测试：设置离线轨迹 ===")
    trajectories = [
        {
            'planner_index': 0,  # 左臂
            'frame': 0,          # 世界系
            'timed_traj': [
                {'desire_time': 0.0, 'cmd_vec': [0.3, 0.2, 0.3, 0.0, 0.0, 0.0]},
                {'desire_time': 1.0, 'cmd_vec': [0.4, 0.2, 0.35, 0.1, 0.0, 0.0]},
                {'desire_time': 2.0, 'cmd_vec': [0.5, 0.2, 0.4, 0.2, 0.0, 0.0]},
                {'desire_time': 3.0, 'cmd_vec': [0.6, 0.2, 0.45, 0.3, 0.0, 0.0]},
            ],
        }
    ]
    result = hardware.set_offline_trajectory_timed(trajectories)
    if result.success:
        logger.info("✅ 离线轨迹设置成功")
    else:
        logger.error(f"❌ 离线轨迹设置失败: {result.message}")
    time.sleep(1.0)


def test_enable(hardware):
    """启用离线轨迹"""
    logger.info("=== 测试：启用离线轨迹 ===")
    result = hardware.enable_offline_trajectory_timed(enable=True)
    if result.success:
        logger.info("✅ 离线轨迹已启用")
    else:
        logger.error(f"❌ 启用失败: {result.message}")
    time.sleep(5.0)


def test_disable(hardware):
    """禁用离线轨迹"""
    logger.info("=== 测试：禁用离线轨迹 ===")
    result = hardware.enable_offline_trajectory_timed(enable=False)
    if result.success:
        logger.info("✅ 离线轨迹已禁用")
    else:
        logger.error(f"❌ 禁用失败: {result.message}")
    time.sleep(1.0)


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
        from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown
        factory_setup(hardware, need_arm=True)
        test_set_trajectory(hardware)
        test_enable(hardware)
        test_disable(hardware)
        logger.info("🎉 离线轨迹测试完成")
        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
