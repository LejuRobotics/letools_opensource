"""
快速模式测试

使用的 Adapter 方法: hardware.enable_quick_mode()
底层路径: ROS 服务 /enable_lb_arm_quick_mode

注意：快速模式用于快速切换手臂控制模式，提高响应速度。

测试用例说明:
- test_enable_quick_mode: 启用快速模式（enable=True）
- test_disable_quick_mode: 禁用快速模式（enable=False）
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


def test_enable_quick_mode(hardware):
    """启用快速模式"""
    logger.info("=== 测试：启用快速模式 ===")
    result = hardware.enable_quick_mode(enable=True)
    if result.success:
        logger.info("✅ 快速模式已启用")
    else:
        logger.error(f"❌ 启用快速模式失败: {result.message}")
    time.sleep(1.0)


def test_disable_quick_mode(hardware):
    """禁用快速模式"""
    logger.info("=== 测试：禁用快速模式 ===")
    result = hardware.enable_quick_mode(enable=False)
    if result.success:
        logger.info("✅ 快速模式已禁用")
    else:
        logger.error(f"❌ 禁用快速模式失败: {result.message}")
    time.sleep(1.0)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'skip_sdk_managers': True,# 快速模式不需要 SDK 管理器
        'skip_end_effector': True,# 快速模式不需要末端执行器
        'skip_camera': True,# 快速模式不需要相机
        'skip_state_manager': True,# 快速模式不需要状态管理器
        'skip_force_publishers': True,# 快速模式不需要力控发布器
    })
    try:
        hardware.initialize()
        test_enable_quick_mode(hardware)
        test_disable_quick_mode(hardware)
        logger.info("🎉 快速模式测试完成")
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
