"""
底盘速度控制测试（SDK 直调）

使用的 Adapter 方法: hardware.send_base_velocity_sdk()
底层路径: SDK 直调 → _low_level_sdk_manager.control_base_velocity → robot_sdk.control.control_base_velocity

注意：此方法走 SDK 直调路径，与 send_base_velocity_timed (TimedCmd 路径) 不同。

测试用例说明:
- test_forward: 底盘前进 0.3 m/s（vx=+0.3）
- test_backward: 底盘后退 0.3 m/s（vx=-0.3）
- test_lateral: 底盘左移 0.2 m/s（vy=+0.2）
- test_rotation: 底盘原地旋转 0.3 rad/s（vyaw=+0.3）
- test_stop: 底盘停止（vx=0, vy=0, vyaw=0）
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


def test_forward(hardware):
    """前进 0.3 m/s"""
    logger.info("=== 测试：前进 0.3 m/s ===")
    result = hardware.send_base_velocity_sdk(vx=0.3, vy=0.0, vyaw=0.0)
    if result.success:
        logger.info("✅ 前进成功")
    else:
        logger.error(f"❌ 前进失败: {result.message}")
    time.sleep(2.0)


def test_backward(hardware):
    """后退 0.3 m/s"""
    logger.info("=== 测试：后退 0.3 m/s ===")
    result = hardware.send_base_velocity_sdk(vx=-0.3, vy=0.0, vyaw=0.0)
    if result.success:
        logger.info("✅ 后退成功")
    else:
        logger.error(f"❌ 后退失败: {result.message}")
    time.sleep(2.0)


def test_lateral(hardware):
    """左移 0.2 m/s"""
    logger.info("=== 测试：左移 0.2 m/s ===")
    result = hardware.send_base_velocity_sdk(vx=0.0, vy=0.2, vyaw=0.0)
    if result.success:
        logger.info("✅ 左移成功")
    else:
        logger.error(f"❌ 左移失败: {result.message}")
    time.sleep(2.0)


def test_rotation(hardware):
    """旋转 0.3 rad/s"""
    logger.info("=== 测试：旋转 0.3 rad/s ===")
    result = hardware.send_base_velocity_sdk(vx=0.0, vy=0.0, vyaw=0.3)
    if result.success:
        logger.info("✅ 旋转成功")
    else:
        logger.error(f"❌ 旋转失败: {result.message}")
    time.sleep(2.0)


def test_stop(hardware):
    """停止"""
    logger.info("=== 测试：停止 ===")
    result = hardware.send_base_velocity_sdk(vx=0.0, vy=0.0, vyaw=0.0)
    if result.success:
        logger.info("✅ 停止成功")
    else:
        logger.error(f"❌ 停止失败: {result.message}")
    time.sleep(1.0)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'angle_unit': 'rad',
        'sdk_managers_whitelist': ['low'],# 底盘速度控制只需要 low 管理器
        'skip_end_effector': True,# 底盘速度控制不需要末端执行器
        'skip_camera': True,# 底盘速度控制不需要相机
        'skip_state_manager': True,# 底盘速度控制不需要状态管理器
        'skip_force_publishers': True,# 底盘速度控制不需要力控发布器
    })
    try:
        hardware.initialize()
        # === 脚手架: 前置设置 ===
        from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown
        factory_setup(hardware, need_arm=False)

        test_forward(hardware)
        test_backward(hardware)
        test_lateral(hardware)
        test_rotation(hardware)
        test_stop(hardware)
        logger.info("🎉 底盘速度（SDK 直调）测试完成")

        # === 脚手架: 后置复位 ===
        factory_teardown(hardware, need_arm=False)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
