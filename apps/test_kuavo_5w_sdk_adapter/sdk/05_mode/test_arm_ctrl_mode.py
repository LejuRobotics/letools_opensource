"""
手臂控制模式测试

使用的 Adapter 方法: hardware.set_arm_control_mode()
底层路径: ROS 服务 /wheel_arm_change_arm_ctrl_mode (kuavo_msgs/changeArmCtrlMode)

注意：set_arm_control_mode 无 _sdk 变体（与 enable_quick_mode 一致），
直接调用标准方法是适配器架构限制，不属于架构违规。

测试用例说明:
- test_keep_pose: 设置手臂控制模式为 0（保持当前位置）
- test_auto_swing: 设置手臂控制模式为 1（重置到初始位置）
- test_external_control: 设置手臂控制模式为 2（外部控制器）
- test_invalid_mode: 传入无效模式 99（应正确返回失败结果）
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


def test_keep_pose(hardware):
    """设置保持当前位置控制模式 (mode=0)"""
    logger.info("=== 测试：手臂控制模式 - 保持当前位置 (mode=0) ===")
    result = hardware.set_arm_control_mode(0)
    if result.success:
        logger.info("✅ 保持当前位置模式设置成功")
    else:
        logger.error(f"❌ 保持当前位置模式设置失败: {result.message}")
    time.sleep(0.5)


def test_auto_swing(hardware):
    """设置重置到初始位置模式 (mode=1)"""
    logger.info("=== 测试：手臂控制模式 - 重置到初始位置 (mode=1) ===")
    result = hardware.set_arm_control_mode(1)
    if result.success:
        logger.info("✅ 重置到初始位置模式设置成功")
    else:
        logger.error(f"❌ 重置到初始位置模式设置失败: {result.message}")
    time.sleep(2.0)


def test_external_control(hardware):
    """设置外部控制器模式 (mode=2)"""
    logger.info("=== 测试：手臂控制模式 - 外部控制器 (mode=2) ===")
    result = hardware.set_arm_control_mode(2)
    if result.success:
        logger.info("✅ 外部控制器模式设置成功")
    else:
        logger.error(f"❌ 外部控制器模式设置失败: {result.message}")
    time.sleep(0.5)


def test_invalid_mode(hardware):
    """测试无效模式 (mode=99，应返回失败)"""
    logger.info("=== 测试：无效手臂控制模式 (mode=99) ===")
    result = hardware.set_arm_control_mode(99)
    if result.success:
        logger.warning("⚠️ 无效模式意外成功")
    else:
        logger.info(f"✅ 无效模式正确返回失败: {result.message}")
    time.sleep(0.5)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'skip_sdk_managers': True,# 手臂控制模式不需要 SDK 管理器
        'skip_end_effector': True,# 手臂控制模式不需要末端执行器
        'skip_camera': True,# 手臂控制模式不需要相机
        'skip_state_manager': True,# 手臂控制模式不需要状态管理器
        'skip_force_publishers': True,# 手臂控制模式不需要力控发布器
    })
    try:
        hardware.initialize()

        # === 脚手架: 前置设置 ===
        factory_setup(hardware, need_arm=False, need_torso_reset=True)

        test_keep_pose(hardware)
        test_auto_swing(hardware)
        test_external_control(hardware)
        test_invalid_mode(hardware)

        # === 脚手架: 后置复位 ===
        factory_teardown(hardware, need_arm=False)

        logger.info("🎉 手臂控制模式测试完成")
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
