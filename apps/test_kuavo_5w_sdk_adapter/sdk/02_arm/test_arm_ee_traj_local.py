"""
手臂末端轨迹测试（局部坐标系，多关键点 + 自动 MPC）

使用的 Adapter 方法: hardware.send_arm_ee_traj_sdk(frame='base_link')
底层路径: SDK 直调 → _arm_sdk_manager.move_eef_traj_auto → robot_sdk.control.control_robot_end_effector_pose

测试用例说明:
- test_forward_trajectory: 双臂末端从默认位姿 (x=0.3m) 向前伸到 (x=0.5m)，持续 3 秒
- test_return_trajectory: 双臂末端从前伸位姿回到默认位姿，持续 3 秒
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

DEFAULT_POSE = [0.3, 0.25, 0.5, 0.0, 0.0, 0.0, 1.0]
FORWARD_POSE = [0.5, 0.25, 0.5, 0.0, 0.0, 0.0, 1.0]


def test_forward_trajectory(hardware):
    """双臂前伸轨迹（局部系）"""
    logger.info("=== 测试：双臂末端前伸轨迹 (LOCAL) ===")
    left_traj = [DEFAULT_POSE, FORWARD_POSE]
    right_traj = [[0.3, -0.25, 0.5, 0, 0, 0, 1], [0.5, -0.25, 0.5, 0, 0, 0, 1]]
    result = hardware.send_arm_ee_traj_sdk(
        left_traj=left_traj, right_traj=right_traj, total_time=3.0, frame='base_link'
    )
    if result.success:
        logger.info("✅ 前伸轨迹成功")
    else:
        logger.error(f"❌ 前伸轨迹失败: {result.message}")
    time.sleep(4.0)
    return result.success


def test_return_trajectory(hardware):
    """返回默认位姿"""
    logger.info("=== 测试：返回默认位姿 (LOCAL) ===")
    left_traj = [FORWARD_POSE, DEFAULT_POSE]
    right_traj = [[0.6, -0.25, 0.5, 0, 0, 0, 1], [0.3, -0.25, 0.5, 0, 0, 0, 1]]
    result = hardware.send_arm_ee_traj_sdk(
        left_traj=left_traj, right_traj=right_traj, total_time=3.0, frame='base_link'
    )
    if result.success:
        logger.info("✅ 返回轨迹成功")
    else:
        logger.error(f"❌ 返回轨迹失败: {result.message}")
    time.sleep(4.0)
    return result.success


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'sdk_managers_whitelist': ['arm'],
        'skip_end_effector': True,
        'skip_camera': True,
        'skip_state_manager': True,
        'skip_force_publishers': True,
    })
    all_passed = True
    try:
        hardware.initialize()
        # === 脚手架: 前置设置 ===
        from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown
        factory_setup(hardware, need_arm=True)

        all_passed &= test_forward_trajectory(hardware)
        all_passed &= test_return_trajectory(hardware)
        if all_passed:
            logger.info("🎉 手臂末端轨迹（局部系）测试完成")
        else:
            logger.error("⚠️ 部分测试失败")

        # === 脚手架: 后置复位 ===
        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()
    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
