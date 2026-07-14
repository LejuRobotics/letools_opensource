"""
手臂关节轨迹测试（多关键点 + 自动 MPC）

使用的 Adapter 方法: hardware.send_arm_joint_traj_sdk()
底层路径: SDK 直调 → _arm_sdk_manager.move_joint_traj_auto → robot_sdk.control.control_arm_joint_positions

测试用例说明:
参考源脚本 case_wheel_test_arm.py 的 pick-and-place 关节运动流程，
使用项目中已验证的关节角度（度），左右臂镜像对称。
- test_pre_pick: 双臂从自然下垂(HOME)运动到展开姿态(双臂张开前伸)，持续 3 秒
- test_pick: 双臂从展开姿态收拢到弯曲姿态(双臂弯曲回收)，持续 3 秒
- test_return: 双臂从弯曲姿态回到自然下垂(HOME)，持续 3 秒
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

# 14 个关节: [左臂 J0-J6, 右臂 J0-J6]（度）
# 关节顺序: shoulder_yaw, shoulder_pitch, shoulder_roll, elbow_pitch, wrist_yaw, wrist_pitch, wrist_roll
# 镜像规则: J0,J3,J6 保持原值；J1,J2,J4,J5 取负号

# 初始位姿（自然下垂）
HOME_POSE = [0.0] * 14

# 展开双臂（双臂张开前伸）— 项目中已验证的展开姿态
# 来源: test_kuavo_5w/03_arm_control/test_arm_joint.py, test_kuavo_5w_app/03_arm_control/test_arm_joint.py 等
SPREAD_POSE = [
    -30.0, 20.0, 15.0, -45.0, 25.0, 10.0, -35.0,   # 左臂
    -30.0, -20.0, -15.0, -45.0, -25.0, -10.0, -35.0  # 右臂
]

# 弯曲双臂（双臂收拢弯曲）— 项目中已验证的弯曲姿态
# 来源: test_kuavo_5w/03_arm_control/test_arm_joint.py, test_kuavo_5w_app/03_arm_control/test_arm_joint.py 等
BEND_POSE = [
    -20.0, 30.0, -25.0, -20.0, 40.0, -15.0, 25.0,   # 左臂
    -20.0, -30.0, 25.0, -20.0, -40.0, 15.0, 25.0     # 右臂
]


def test_pre_pick(hardware):
    """双臂从自然下垂到展开姿态"""
    logger.info("=== 测试：HOME → 展开双臂（张开前伸） ===")
    joint_traj = [HOME_POSE, SPREAD_POSE]
    result = hardware.send_arm_joint_traj_sdk(joint_traj=joint_traj, total_time=3.0)
    if result.success:
        logger.info("✅ 展开轨迹成功")
    else:
        logger.error(f"❌ 展开轨迹失败: {result.message}")
    time.sleep(4.0)
    return result.success


def test_pick(hardware):
    """双臂从展开姿态收拢到弯曲姿态"""
    logger.info("=== 测试：展开 → 弯曲收拢 ===")
    joint_traj = [SPREAD_POSE, BEND_POSE]
    result = hardware.send_arm_joint_traj_sdk(joint_traj=joint_traj, total_time=3.0)
    if result.success:
        logger.info("✅ 收拢轨迹成功")
    else:
        logger.error(f"❌ 收拢轨迹失败: {result.message}")
    time.sleep(4.0)
    return result.success


def test_return(hardware):
    """双臂从弯曲姿态回到自然下垂"""
    logger.info("=== 测试：弯曲 → 归位 ===")
    joint_traj = [BEND_POSE, HOME_POSE]
    result = hardware.send_arm_joint_traj_sdk(joint_traj=joint_traj, total_time=3.0)
    if result.success:
        logger.info("✅ 归位轨迹成功")
    else:
        logger.error(f"❌ 归位轨迹失败: {result.message}")
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

        all_passed &= test_pre_pick(hardware)
        all_passed &= test_pick(hardware)
        all_passed &= test_return(hardware)
        if all_passed:
            logger.info("🎉 手臂关节轨迹测试完成")
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
