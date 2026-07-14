"""
下肢关节控制测试（自动 MPC 模式管理 + 100Hz 循环下发）

使用的 Adapter 方法: hardware.send_leg_joint_sdk()
底层路径: SDK 直调 → _low_level_sdk_manager.move_wheel_lower_joint_auto → robot_sdk.control.control_wheel_lower_joint

测试用例说明:
参考源脚本 case_wheel_test_torso_joint.py → TorsoAPI._move_wheel_lower_joint。
adapter 内部自动完成：设置 MPC 模式(ArmOnly) → 读取当前关节位置 → 100Hz 循环插值下发。
- test_zero_position: 下肢 4 个关节回到 0° 零位，持续 3 秒
- test_target_pose: 下肢设为源脚本验证过的目标姿态 [14.90, -32.01, 18.03, -90.0]°，持续 3 秒
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

# 源脚本 case_wheel_test_torso_joint.py 中的目标关节角度（度）
TARGET_JOINT_ANGLES = [14.90, -32.01, 18.03, -90.0]


def test_zero_position(hardware):
    """回到零位"""
    logger.info("=== 测试：下肢零位 [0, 0, 0, 0]° ===")
    result = hardware.send_leg_joint_sdk(joint_angles=[0.0, 0.0, 0.0, 0.0], total_time=3.0)
    if result.success:
        logger.info("✅ 零位成功")
    else:
        logger.error(f"❌ 零位失败: {result.message}")
    time.sleep(1.0)


def test_target_pose(hardware):
    """源脚本验证过的目标姿态"""
    logger.info(f"=== 测试：下肢目标姿态 {TARGET_JOINT_ANGLES}° ===")
    result = hardware.send_leg_joint_sdk(joint_angles=TARGET_JOINT_ANGLES, total_time=3.0)
    if result.success:
        logger.info("✅ 目标姿态成功")
    else:
        logger.error(f"❌ 目标姿态失败: {result.message}")
    time.sleep(1.0)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'sdk_managers_whitelist': ['low'],# 下肢关节控制只需要 low 管理器
        'skip_end_effector': True,# 下肢关节控制不需要末端执行器
        'skip_camera': True,# 下肢关节控制不需要相机
        'skip_state_manager': True,# 下肢关节控制不需要状态管理器
        'skip_force_publishers': True,# 下肢关节控制不需要力控发布器
    })
    try:
        hardware.initialize()
        factory_setup(hardware, need_arm=False, need_torso_reset=True)
        test_zero_position(hardware)
        test_target_pose(hardware)
        logger.info("🎉 下肢关节（SDK 直调）测试完成")
    finally:
        factory_teardown(hardware, need_arm=False)
        hardware.shutdown()


if __name__ == "__main__":
    main()
