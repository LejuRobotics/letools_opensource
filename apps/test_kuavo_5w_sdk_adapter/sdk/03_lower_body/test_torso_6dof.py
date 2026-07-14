"""
躯干 6DOF 控制测试（SDK 直调）

使用的 Adapter 方法: hardware.send_torso_6dof_sdk()
底层路径: SDK 直调 → _low_level_sdk_manager.control_torso_6dof → robot_sdk.control.control_torso_6dof

测试用例说明:
- test_raise_torso: 躯干垂直抬升（需要 100Hz 循环调用）
- test_rotate_torso: 躯干绕 z 轴旋转
- test_100hz_loop: 以 100Hz 频率持续发送（演示正确的循环控制用法）
- test_reset: 躯干恢复到初始位姿

重要：SDK 躯干控制是瞬时指令，必须：
1. 以 100Hz 频率持续发送指令
2. 从当前位置插值到目标位置
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


def interpolate_and_send(hardware, start_pose, end_pose, duration=2.0):
    """从起始位姿插值到目标位姿，以 100Hz 频率发送指令
    
    Args:
        hardware: 硬件实例
        start_pose: 起始位姿 (x, y, z, roll, pitch, yaw)
        end_pose: 目标位姿 (x, y, z, roll, pitch, yaw)
        duration: 运动时长（秒）
    """
    freq = 100.0
    dt = 1.0 / freq
    num_samples = int(duration * freq)
    
    for i in range(num_samples):
        t = i / (num_samples - 1) if num_samples > 1 else 1.0
        # 线性插值
        pose = [
            start_pose[j] * (1 - t) + end_pose[j] * t
            for j in range(6)
        ]
        result = hardware.send_torso_6dof_sdk(
            x=pose[0], y=pose[1], z=pose[2],
            roll=pose[3], pitch=pose[4], yaw=pose[5]
        )
        if not result.success:
            logger.error(f"第 {i} 次调用失败: {result.message}")
            break
        time.sleep(dt)


def test_raise_torso(hardware, current_pose):
    """抬升躯干（从当前位置插值到目标位置）"""
    logger.info("=== 测试：抬升躯干 z 方向 ===")
    # 目标：在当前位置基础上抬升 z
    target_pose = list(current_pose)
    target_pose[2] = 1.0  # z = 1.0m
    
    logger.info(f"当前位姿: {current_pose}")
    logger.info(f"目标位姿: {target_pose}")
    
    interpolate_and_send(hardware, current_pose, target_pose, duration=2.0)
    logger.info("✅ 抬升测试完成")
    return target_pose


def test_rotate_torso(hardware, current_pose):
    """旋转躯干（从当前位置插值到目标位置）"""
    import math
    logger.info("=== 测试：旋转躯干 yaw ===")
    # 目标：在当前位置基础上旋转 yaw
    target_pose = list(current_pose)
    target_pose[5] = math.radians(45.0)  # yaw = 45度
    
    logger.info(f"当前位姿: {current_pose}")
    logger.info(f"目标位姿: {target_pose}")
    
    interpolate_and_send(hardware, current_pose, target_pose, duration=2.0)
    logger.info("✅ 旋转测试完成")
    return target_pose


def test_100hz_loop(hardware, current_pose):
    """100Hz 循环控制（从当前位置到目标位置）"""
    import math
    logger.info("=== 测试：100Hz 循环控制（z + pitch） ===")
    
    target_pose = list(current_pose)
    target_pose[2] = 1.0  # z = 1.0m
    target_pose[4] = math.radians(5.0)  # pitch = 5度
    
    logger.info(f"当前位姿: {current_pose}")
    logger.info(f"目标位姿: {target_pose}")
    
    interpolate_and_send(hardware, current_pose, target_pose, duration=2.0)
    logger.info("✅ 100Hz 循环完成")
    return target_pose


def test_reset(hardware, current_pose):
    """恢复初始位姿"""
    logger.info("=== 测试：恢复初始位姿 ===")
    # 目标位姿：全零（初始位置）
    initial_pose = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]  # 默认 z=0.8m
    
    logger.info(f"当前位姿: {current_pose}")
    logger.info(f"目标位姿: {initial_pose}")
    
    interpolate_and_send(hardware, current_pose, initial_pose, duration=2.0)
    logger.info("✅ 恢复完成")
    return initial_pose


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'angle_unit': 'rad',
        'sdk_managers_whitelist': ['low'],
        'skip_end_effector': True,
        'skip_camera': True,
        'skip_state_manager': True,
        'skip_force_publishers': True,
    })
    try:
        hardware.initialize()
        # 注意：跳过躯干重置，避免后仰
        factory_setup(hardware, need_arm=False, need_torso_reset=False)
        time.sleep(0.5)
        
        # 初始位姿（可根据实际情况调整或从硬件读取）
        current_pose = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]  # x, y, z, roll, pitch, yaw
        
        # 执行测试
        current_pose = test_raise_torso(hardware, current_pose)
        time.sleep(1.0)
        
        current_pose = test_rotate_torso(hardware, current_pose)
        time.sleep(1.0)
        
        current_pose = test_100hz_loop(hardware, current_pose)
        time.sleep(1.0)
        
        test_reset(hardware, current_pose)
        
        logger.info("🎉 躯干 6DOF（SDK 直调）测试完成")
    finally:
        factory_teardown(hardware, need_arm=False)
        hardware.shutdown()


if __name__ == "__main__":
    main()
