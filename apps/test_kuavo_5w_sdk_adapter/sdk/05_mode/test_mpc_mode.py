"""
MPC 模式设置测试

使用的 Adapter 方法: hardware.set_mpc_mode_sdk()
底层路径: SDK 直调 → _arm_sdk_manager.set_mpc_mode → robot_sdk.control.set_manipulation_mpc_mode

注意：传入字符串参数 ('ArmOnly', 'NoControl', 'BaseArm')，不是枚举类型。
      SDK 实际枚举：NoControl(0), ArmOnly(1), BaseOnly(2), BaseArm(3), ERROR(-1)。
      'BaseArm' 控制底盘+手臂同时受控。

测试用例说明:
- test_arm_only: 设置 MPC 为 ArmOnly 模式（仅手臂受控，躯干和下肢保持默认）
- test_no_control: 设置 MPC 为 NoControl 模式（所有部件不受控，自由状态）
- test_base_arm: 设置 MPC 为 BaseArm 模式（全身 MPC 控制，底盘+手臂同时受控）
- test_invalid_mode: 传入无效模式字符串（应正确返回失败结果）
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


def test_arm_only(hardware):
    """设置 ArmOnly 模式"""
    logger.info("=== 测试：设置 MPC 模式为 ArmOnly ===")
    result = hardware.set_mpc_mode_sdk('ArmOnly')
    if result.success:
        logger.info("✅ ArmOnly 模式设置成功")
    else:
        logger.error(f"❌ ArmOnly 模式设置失败: {result.message}")
    time.sleep(1.0)


def test_no_control(hardware):
    """设置 NoControl 模式"""
    logger.info("=== 测试：设置 MPC 模式为 NoControl ===")
    result = hardware.set_mpc_mode_sdk('NoControl')
    if result.success:
        logger.info("✅ NoControl 模式设置成功")
    else:
        logger.error(f"❌ NoControl 模式设置失败: {result.message}")
    time.sleep(1.0)


def test_base_arm(hardware):
    """设置 BaseArm 模式"""
    logger.info("=== 测试：设置 MPC 模式为 BaseArm ===")
    result = hardware.set_mpc_mode_sdk('BaseArm')
    if result.success:
        logger.info("✅ BaseArm 模式设置成功")
    else:
        logger.error(f"❌ BaseArm 模式设置失败: {result.message}")
    time.sleep(1.0)


def test_invalid_mode(hardware):
    """测试无效模式"""
    logger.info("=== 测试：无效模式（应返回失败） ===")
    result = hardware.set_mpc_mode_sdk('InvalidMode')
    if result.success:
        logger.warning("⚠️ 无效模式意外成功")
    else:
        logger.info(f"✅ 无效模式正确返回失败: {result.message}")
    time.sleep(0.5)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'sdk_managers_whitelist': ['arm'],# MPC 模式设置只需要 arm 管理器
        'skip_end_effector': True,# MPC 模式设置不需要末端执行器
        'skip_camera': True,# MPC 模式设置不需要相机
        'skip_state_manager': True,# MPC 模式设置不需要状态管理器
        'skip_force_publishers': True,# MPC 模式设置不需要力控发布器
    })
    try:
        hardware.initialize()
        test_arm_only(hardware)
        test_no_control(hardware)
        test_base_arm(hardware)
        test_invalid_mode(hardware)
        logger.info("🎉 MPC 模式测试完成")
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
