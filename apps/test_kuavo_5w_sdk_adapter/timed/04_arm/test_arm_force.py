"""
手臂力控测试（TimedCmd 路径）

使用的 Adapter 方法: hardware.send_arm_force_timed()
底层路径: TimedCmd → _timed_cmd_manager.send_arm_force → planner_index=arm_force
参考源脚本: case_wheel_arm_force.py（box_weight_kg=6.0, interpolation_speed=2000.0）

测试用例说明:
- test_disable_empty_detect: 关闭挥空检测（施加期望力前必须关闭）
- test_apply_weight_force: 双臂施加 6kg 向下期望力（58.8N z），持续 3s，对齐源脚本 box_weight_kg=6.0
- test_release_force: 撤销所有力（六维力全部归零）
- test_restore_empty_detect: 恢复挥空检测

注意：send_arm_force_timed() 使用 N（牛顿）作为单位，走 TimedCmd → ROS 服务路径。
另外还有 ROS 话题路径的力控接口：set_ee_force(side, force_kg, torque)（kg 单位，内部 ×9.8 转 N），
走 /desired_ee_force/{left,right} 话题，对齐源脚本 LBForceController。两种接口用途不同。
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

# 6kg 向下力: 6 * 9.8 = 58.8N z 方向（对齐源脚本 box_weight_kg=6.0）
WEIGHT_FORCE = [0.0, 0.0, -58.8, 0.0, 0.0, 0.0]
ZERO_FORCE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_disable_empty_detect(hardware):
    """关闭挥空检测"""
    logger.info("=== 测试：关闭挥空检测 ===")
    result = hardware.enable_force_empty_detect(False)
    if result.success:
        logger.info("✅ 挥空检测已关闭")
    else:
        logger.error(f"❌ 关闭挥空检测失败: {result.message}")
    time.sleep(0.5)


def test_apply_weight_force(hardware):
    """施加 6kg 向下期望力（58.8N z），持续 3s"""
    logger.info("=== 测试：施加 6kg 向下期望力 ===")
    logger.info(f"力向量: {WEIGHT_FORCE} (58.8N z = 6kg)")
    result = hardware.send_arm_force_timed(force=WEIGHT_FORCE, desire_time=3.0)
    if result.success:
        logger.info("✅ 6kg 期望力施加成功")
    else:
        logger.error(f"❌ 期望力施加失败: {result.message}")
    time.sleep(4.0)


def test_release_force(hardware):
    """撤销所有力"""
    logger.info("=== 测试：撤销力 ===")
    result = hardware.send_arm_force_timed(force=ZERO_FORCE, desire_time=2.0)
    if result.success:
        logger.info("✅ 撤销力成功")
    else:
        logger.error(f"❌ 撤销力失败: {result.message}")
    time.sleep(3.0)


def test_restore_empty_detect(hardware):
    """恢复挥空检测"""
    logger.info("=== 测试：恢复挥空检测 ===")
    result = hardware.enable_force_empty_detect(True)
    if result.success:
        logger.info("✅ 挥空检测已恢复")
    else:
        logger.error(f"❌ 恢复挥空检测失败: {result.message}")
    time.sleep(0.5)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'sdk_managers_whitelist': ['timed'],
        'skip_end_effector': True,
        'skip_camera': True,
        'skip_state_manager': True,
        'skip_force_publishers': True,
    })
    try:
        hardware.initialize()
        factory_setup(hardware, need_arm=True)

        test_disable_empty_detect(hardware)
        test_apply_weight_force(hardware)
        test_release_force(hardware)
        test_restore_empty_detect(hardware)

        logger.info("🎉 手臂力控测试完成")
        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
