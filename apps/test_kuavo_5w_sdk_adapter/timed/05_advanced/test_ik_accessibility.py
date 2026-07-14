"""
IK 可达性检查测试

使用的 Adapter 方法: hardware.check_ik_accessibility_timed()
底层路径: _timed_cmd_manager.check_ik_accessibility → ROS 服务 /mobile_manipulator_ik_accessibility_check

测试用例说明:
- test_reachable_pose: 检查左臂世界坐标系下可达位姿（x=0.4m, y=0.25m, z=0.5m），应返回可达
- test_unreachable_pose: 检查超出工作空间的位姿（x=2.0m, y=0, z=2.0m），应返回不可达
- test_right_arm: 检查右臂世界坐标系下可达位姿（x=0.4m, y=-0.25m, z=0.5m）
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


def test_reachable_pose(hardware):
    """检查可达位姿"""
    logger.info("=== 测试：检查可达位姿（左臂世界系） ===")
    pose = [0.4, 0.25, 0.5, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
    result = hardware.check_ik_accessibility_timed(
        is_left=True,
        is_local=False,
        is_whole_body=True,
        pose_desired=pose,
        total_time_desired=3.0,
    )
    if result.success:
        data = result.data
        if data.get('success'):
            logger.info(f"✅ 位姿可达: lin_err={data['best_linear_error']:.6f}m")
        else:
            logger.warning(f"⚠️ 位姿不可达（位置优先: {data.get('pos_priority_access')}）")
    else:
        logger.error(f"❌ IK 检查失败: {result.message}")
    time.sleep(1.0)


def test_unreachable_pose(hardware):
    """检查不可达位姿（超出工作空间）"""
    logger.info("=== 测试：检查不可达位姿（超出范围） ===")
    pose = [2.0, 0.0, 2.0, 0.0, 0.0, 0.0]  # 超出工作空间
    result = hardware.check_ik_accessibility_timed(
        is_left=True,
        is_local=False,
        is_whole_body=True,
        pose_desired=pose,
        total_time_desired=3.0,
    )
    if result.success:
        data = result.data
        if data.get('success'):
            logger.warning(f"⚠️ 预期不可达但返回可达")
        else:
            logger.info(f"✅ 正确判断为不可达: lin_err={data['best_linear_error']:.6f}m")
    else:
        logger.error(f"❌ IK 检查失败: {result.message}")
    time.sleep(1.0)


def test_right_arm(hardware):
    """检查右臂可达位姿"""
    logger.info("=== 测试：检查右臂可达位姿 ===")
    pose = [0.4, -0.25, 0.5, 0.0, 0.0, 0.0]
    result = hardware.check_ik_accessibility_timed(
        is_left=False,
        is_local=False,
        is_whole_body=True,
        pose_desired=pose,
        total_time_desired=3.0,
    )
    if result.success:
        data = result.data
        reachable = "可达" if data.get('success') else "不可达"
        logger.info(f"右臂位姿: {reachable}")
    else:
        logger.error(f"❌ IK 检查失败: {result.message}")
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
        test_reachable_pose(hardware)
        test_unreachable_pose(hardware)
        test_right_arm(hardware)
        logger.info("🎉 IK 可达性测试完成")
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
