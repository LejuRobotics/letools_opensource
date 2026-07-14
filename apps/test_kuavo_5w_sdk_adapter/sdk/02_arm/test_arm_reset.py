"""
手臂归位测试

使用的 Adapter 方法: hardware.arm_reset()
底层路径: SDK 直调 → _arm_sdk_manager.arm_reset → robot_sdk.control.arm_reset (自动 MPC 管理)

测试用例说明:
- test_arm_reset: 双臂从当前位置回到默认归位姿势（自动管理 MPC 模式）
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


def test_arm_reset(hardware):
    """手臂归位"""
    logger.info("=== 测试：手臂归位 ===")
    result = hardware.arm_reset()
    if result.success:
        logger.info("✅ 手臂归位成功")
    else:
        logger.error(f"❌ 手臂归位失败: {result.message}")
    time.sleep(3.0)
    return result.success


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'sdk_managers_whitelist': ['arm'],# 手臂归位只需要 arm 管理器
        'skip_end_effector': True,# 手臂归位不需要末端执行器
        'skip_camera': True,# 手臂归位不需要相机
        'skip_state_manager': True,# 手臂归位不需要状态管理器
        'skip_force_publishers': True,# 手臂归位不需要力控发布器
    })
    all_passed = True
    try:
        hardware.initialize()
        # === 脚手架: 前置设置 ===
        from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown
        factory_setup(hardware, need_arm=True)

        all_passed &= test_arm_reset(hardware)
        if all_passed:
            logger.info("🎉 手臂归位测试完成")
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
