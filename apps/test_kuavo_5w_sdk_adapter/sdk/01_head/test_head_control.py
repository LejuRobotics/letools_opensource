"""
头部控制测试

使用的 Adapter 方法: hardware.control_head_sdk()
底层路径: SDK 直调 → _low_level_sdk_manager.control_head → robot_sdk.control.control_head

测试用例说明:
- test_center: 头部回到正前方位置（yaw=0°, pitch=0°）
- test_look_left: 头部向左转 30°（yaw=+30°）
- test_look_right: 头部向右转 30°（yaw=-30°）
- test_look_up: 头部向上抬 20°（pitch=+20°）
- test_look_down: 头部向下低 20°（pitch=-20°）
- test_scan_sequence: 头部依次执行左→右→居中扫描动作
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


def test_center(hardware):
    """头部居中"""
    logger.info("=== 测试：头部居中 ===")
    result = hardware.control_head_sdk(yaw=0.0, pitch=0.0)
    if result.success:
        logger.info("✅ 头部居中成功")
    else:
        logger.error(f"❌ 头部居中失败: {result.message}")
    time.sleep(1.0)


def test_look_left(hardware):
    """左转 30°"""
    logger.info("=== 测试：左转 30° ===")
    result = hardware.control_head_sdk(yaw=30.0, pitch=0.0)
    if result.success:
        logger.info("✅ 左转成功")
    else:
        logger.error(f"❌ 左转失败: {result.message}")
    time.sleep(1.0)


def test_look_right(hardware):
    """右转 30°"""
    logger.info("=== 测试：右转 30° ===")
    result = hardware.control_head_sdk(yaw=-30.0, pitch=0.0)
    if result.success:
        logger.info("✅ 右转成功")
    else:
        logger.error(f"❌ 右转失败: {result.message}")
    time.sleep(1.0)


def test_look_up(hardware):
    """抬头 20°"""
    logger.info("=== 测试：抬头 20° ===")
    result = hardware.control_head_sdk(yaw=0.0, pitch=20.0)
    if result.success:
        logger.info("✅ 抬头成功")
    else:
        logger.error(f"❌ 抬头失败: {result.message}")
    time.sleep(1.0)


def test_look_down(hardware):
    """低头 20°"""
    logger.info("=== 测试：低头 20° ===")
    result = hardware.control_head_sdk(yaw=0.0, pitch=-20.0)
    if result.success:
        logger.info("✅ 低头成功")
    else:
        logger.error(f"❌ 低头失败: {result.message}")
    time.sleep(1.0)


def test_scan_sequence(hardware):
    """扫描序列：左 → 右 → 居中"""
    logger.info("=== 测试：扫描序列 ===")
    for yaw in [30.0, -30.0, 0.0]:
        result = hardware.control_head_sdk(yaw=yaw, pitch=0.0)
        if result.success:
            logger.info(f"✅ yaw={yaw}° 成功")
        else:
            logger.error(f"❌ yaw={yaw}° 失败: {result.message}")
        time.sleep(1.5)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'sdk_managers_whitelist': ['low'],
        'skip_end_effector': True,
        'skip_camera': True,
        'skip_state_manager': True,
        'skip_force_publishers': True,
    })
    try:
        hardware.initialize()
        test_center(hardware)
        test_look_left(hardware)
        test_look_right(hardware)
        test_look_up(hardware)
        test_look_down(hardware)
        test_scan_sequence(hardware)
        logger.info("🎉 头部控制测试完成")
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
