"""
底盘局部坐标系控制测试

使用的 Adapter 方法: hardware.send_base_velocity_timed(frame=LOCAL)
底层路径: TimedCmd → _timed_cmd_manager.send_chassis_local → planner_index=1

测试用例说明:
- test_forward: 底盘在本体坐标系下前进 0.3m（vx=+0.3）
- test_backward: 底盘在本体坐标系下后退 0.3m（vx=-0.3）
- test_lateral: 底盘在本体坐标系下左移 0.2m（vy=+0.2）
- test_rotation: 底盘在本体坐标系下原地旋转 0.3rad（vyaw=+0.3）
"""
import sys
import time
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()
from adapters.hardware.factory import HardwareFactory
from core.domain.enums import FrameType

logger = get_logger(__name__)


def test_forward(hardware):
    """前进 0.3m（本体系）"""
    logger.info("=== 测试：前进 0.3m (LOCAL) ===")
    result = hardware.send_base_velocity_timed(vx=0.3, vy=0.0, vyaw=0.0, frame=FrameType.LOCAL)
    if result.success:
        logger.info(f"✅ 前进成功")
    else:
        logger.error(f"❌ 前进失败: {result.message}")
    time.sleep(2.0)


def test_backward(hardware):
    """后退 0.3m（本体系）"""
    logger.info("=== 测试：后退 0.3m (LOCAL) ===")
    result = hardware.send_base_velocity_timed(vx=-0.3, vy=0.0, vyaw=0.0, frame=FrameType.LOCAL)
    if result.success:
        logger.info(f"✅ 后退成功")
    else:
        logger.error(f"❌ 后退失败: {result.message}")
    time.sleep(2.0)


def test_lateral(hardware):
    """左移 0.2m（本体系）"""
    logger.info("=== 测试：左移 0.2m (LOCAL) ===")
    result = hardware.send_base_velocity_timed(vx=0.0, vy=0.2, vyaw=0.0, frame=FrameType.LOCAL)
    if result.success:
        logger.info(f"✅ 左移成功")
    else:
        logger.error(f"❌ 左移失败: {result.message}")
    time.sleep(2.0)


def test_rotation(hardware):
    """旋转 0.3rad（本体系）"""
    logger.info("=== 测试：旋转 0.3rad (LOCAL) ===")
    result = hardware.send_base_velocity_timed(vx=0.0, vy=0.0, vyaw=0.3, frame=FrameType.LOCAL)
    if result.success:
        logger.info(f"✅ 旋转成功")
    else:
        logger.error(f"❌ 旋转失败: {result.message}")
    time.sleep(2.0)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'angle_unit': 'rad',
        'sdk_managers_whitelist': ['timed'],
        'skip_end_effector': True,
        'skip_camera': True,
        'skip_state_manager': True,
        'skip_force_publishers': True,
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
        logger.info("🎉 底盘局部系测试完成")

        # === 脚手架: 后置复位 ===
        factory_teardown(hardware, need_arm=False)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
