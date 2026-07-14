"""
底盘位置控制测试（局部坐标系，SDK 直调）

使用的 Adapter 方法: hardware.send_base_position_local_sdk()
底层路径: SDK 直调 → _low_level_sdk_manager.control_base_position_local → robot_sdk.control.control_base_position_local

测试用例说明:
- test_move_forward: 底盘在本体坐标系下前进 0.5m（x=+0.5, y=0, yaw=0）
- test_move_lateral: 底盘在本体坐标系下左移 0.3m（x=0, y=+0.3, yaw=0）
- test_rotate: 底盘在本体坐标系下原地旋转 0.5rad（x=0, y=0, yaw=+0.5）
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


def test_move_forward(hardware):
    """前进 0.5m（本体系）"""
    logger.info("=== 测试：前进 0.5m (LOCAL) ===")
    result = hardware.send_base_position_local_sdk(x=0.5, y=0.0, yaw=0.0)
    if result.success:
        logger.info("✅ 前进成功")
    else:
        logger.error(f"❌ 前进失败: {result.message}")
    time.sleep(3.0)


def test_move_lateral(hardware):
    """左移 0.3m（本体系）"""
    logger.info("=== 测试：左移 0.3m (LOCAL) ===")
    result = hardware.send_base_position_local_sdk(x=0.0, y=0.3, yaw=0.0)
    if result.success:
        logger.info("✅ 左移成功")
    else:
        logger.error(f"❌ 左移失败: {result.message}")
    time.sleep(3.0)


def test_rotate(hardware):
    """旋转 0.5rad（本体系）"""
    logger.info("=== 测试：旋转 0.5rad (LOCAL) ===")
    result = hardware.send_base_position_local_sdk(x=0.0, y=0.0, yaw=0.5)
    if result.success:
        logger.info("✅ 旋转成功")
    else:
        logger.error(f"❌ 旋转失败: {result.message}")
    time.sleep(3.0)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'angle_unit': 'rad',
        'sdk_managers_whitelist': ['low'],# 底盘位置控制只需要 low 管理器
        'skip_end_effector': True,# 底盘位置控制不需要末端执行器
        'skip_camera': True,# 底盘位置控制不需要相机
        'skip_state_manager': True,# 底盘位置控制不需要状态管理器
        'skip_force_publishers': True,# 底盘位置控制不需要力控发布器
    })
    try:
        hardware.initialize()
        # === 脚手架: 前置设置 ===
        from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown
        factory_setup(hardware, need_arm=False)

        test_move_forward(hardware)
        test_move_lateral(hardware)
        test_rotate(hardware)
        logger.info("🎉 底盘位置（局部系，SDK 直调）测试完成")

        # === 脚手架: 后置复位 ===
        factory_teardown(hardware, need_arm=False)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
