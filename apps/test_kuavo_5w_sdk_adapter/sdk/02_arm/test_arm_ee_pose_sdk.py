"""
手臂末端位姿控制测试（SDK 单次直调，30Hz 循环）

使用的 Adapter 方法: hardware.send_ee_pose_sdk()
底层路径: SDK 直调 → _low_level_sdk_manager.control_robot_end_effector_pose → robot_sdk.control.control_robot_end_effector_pose

测试用例说明:
- test_forward_ee_pose: 双臂末端从默认位姿 (x=0.3m) 向前伸到 (x=0.5m)，30Hz 循环 2 秒
- test_up_ee_pose: 双臂末端从默认位姿 (z=0.5m) 向上抬到 (z=0.7m)，30Hz 循环 2 秒
- test_return_ee_pose: 双臂末端从前伸位姿回到默认位姿，30Hz 循环 2 秒
- test_single_arm_ee_pose: 单臂控制（左臂插值，右臂显式保持静止避免跳变）

重要：SDK 末端位姿控制是瞬时指令，必须：
1. 以 30Hz 频率持续发送指令
2. 从当前位姿插值到目标位姿
3. 使用前需将 MPC 模式设为 ArmOnly
"""
import sys
import time
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()
from adapters.hardware.factory import HardwareFactory
from core.domain.pose import Pose6D
from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown

logger = get_logger(__name__)

# 位姿列表格式: [x, y, z, yaw, pitch, roll]（弧度）
DEFAULT_LEFT = [0.3, 0.25, 0.5, 0.0, 0.0, 0.0]
DEFAULT_RIGHT = [0.3, -0.25, 0.5, 0.0, 0.0, 0.0]
FORWARD_LEFT = [0.5, 0.25, 0.5, 0.0, 0.0, 0.0]
FORWARD_RIGHT = [0.5, -0.25, 0.5, 0.0, 0.0, 0.0]
UP_LEFT = [0.5, 0.25, 0.7, 0.0, 0.0, 0.0]
UP_RIGHT = [0.5, -0.25, 0.7, 0.0, 0.0, 0.0]


def _to_pose6d(pose_list):
    """[x, y, z, yaw, pitch, roll] → Pose6D"""
    return Pose6D(x=pose_list[0], y=pose_list[1], z=pose_list[2],
                  yaw=pose_list[3], pitch=pose_list[4], roll=pose_list[5])


def _interpolate(start, end, t):
    """线性插值两个位姿列表"""
    return [start[j] * (1 - t) + end[j] * t for j in range(6)]


def send_ee_pose_loop(hardware, start_left, end_left, start_right, end_right,
                      duration=2.0, frame='world'):
    """以 30Hz 频率从起始位姿插值到目标位姿，持续发送双臂末端位姿指令

    Args:
        hardware: 硬件实例
        start_left: 左臂起始位姿 [x, y, z, yaw, pitch, roll]
        end_left: 左臂目标位姿
        start_right: 右臂起始位姿
        end_right: 右臂目标位姿
        duration: 运动时长（秒）
        frame: 坐标系 ('world' 或 'base_link')
    """
    freq = 30.0
    dt = 1.0 / freq
    num_samples = int(duration * freq)

    for i in range(num_samples):
        t = i / (num_samples - 1) if num_samples > 1 else 1.0
        left_pose = _to_pose6d(_interpolate(start_left, end_left, t))
        right_pose = _to_pose6d(_interpolate(start_right, end_right, t))
        result = hardware.send_ee_pose_sdk(
            left_pose=left_pose, right_pose=right_pose, frame=frame
        )
        if not result.success:
            logger.error(f"第 {i} 次调用失败: {result.message}")
            return False
        time.sleep(dt)
    return True


def test_forward_ee_pose(hardware):
    """双臂前伸（30Hz 循环）"""
    logger.info("=== 测试：双臂末端前伸 (SDK 单次直调) ===")
    ok = send_ee_pose_loop(hardware, DEFAULT_LEFT, FORWARD_LEFT,
                           DEFAULT_RIGHT, FORWARD_RIGHT, duration=2.0)
    if ok:
        logger.info("✅ 前伸成功")
    else:
        logger.error("❌ 前伸失败")
    time.sleep(1.0)
    return ok


def test_up_ee_pose(hardware):
    """双臂上抬（30Hz 循环）"""
    logger.info("=== 测试：双臂末端上抬 (SDK 单次直调) ===")
    ok = send_ee_pose_loop(hardware, FORWARD_LEFT, UP_LEFT,
                           FORWARD_RIGHT, UP_RIGHT, duration=2.0)
    if ok:
        logger.info("✅ 上抬成功")
    else:
        logger.error("❌ 上抬失败")
    time.sleep(1.0)
    return ok


def test_return_ee_pose(hardware):
    """返回默认位姿（30Hz 循环）"""
    logger.info("=== 测试：返回默认位姿 (SDK 单次直调) ===")
    ok = send_ee_pose_loop(hardware, UP_LEFT, DEFAULT_LEFT,
                           UP_RIGHT, DEFAULT_RIGHT, duration=2.0)
    if ok:
        logger.info("✅ 返回成功")
    else:
        logger.error("❌ 返回失败")
    time.sleep(1.0)
    return ok


def test_single_arm_ee_pose(hardware):
    """单臂控制（左臂插值，右臂保持静止）

    为避免右臂跳变，这里显式传入右臂当前位姿（DEFAULT_RIGHT），
    而非依赖 send_ee_pose_sdk 的自动填充默认位姿 (x=0.1, y=-0.3, z=0.7)——
    后者与测试序列位姿不一致会导致右臂跳动。
    """
    logger.info("=== 测试：单臂末端位姿 (SDK 单次直调) ===")
    # 左臂从默认前伸到 FORWARD_LEFT，右臂始终保持 DEFAULT_RIGHT 不动
    right_hold = _to_pose6d(DEFAULT_RIGHT)
    freq = 30.0
    dt = 1.0 / freq
    num_samples = int(2.0 * freq)
    ok = True
    for i in range(num_samples):
        t = i / (num_samples - 1) if num_samples > 1 else 1.0
        left_pose = _to_pose6d(_interpolate(DEFAULT_LEFT, FORWARD_LEFT, t))
        result = hardware.send_ee_pose_sdk(
            left_pose=left_pose, right_pose=right_hold, frame='world'
        )
        if not result.success:
            logger.error(f"第 {i} 次调用失败: {result.message}")
            ok = False
            break
        time.sleep(dt)
    if ok:
        logger.info("✅ 单臂控制成功（左臂前伸，右臂保持静止）")
    else:
        logger.error("❌ 单臂控制失败")
    time.sleep(1.0)
    # 单臂测试后左臂回到默认
    send_ee_pose_loop(hardware, FORWARD_LEFT, DEFAULT_LEFT,
                      DEFAULT_RIGHT, DEFAULT_RIGHT, duration=1.0)
    return ok


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'angle_unit': 'rad',  # SDK 直调用弧度
        'sdk_managers_whitelist': ['low', 'arm'],  # low=末端位姿直调, arm=MPC模式管理+归位
        'skip_end_effector': True,
        'skip_camera': True,
        'skip_state_manager': True,
        'skip_force_publishers': True,
    })
    all_passed = True
    try:
        hardware.initialize()
        # === 脚手架: 前置设置 ===
        factory_setup(hardware, need_arm=True)
        time.sleep(0.5)

        # SDK 单次直调需要手动设置 MPC 模式为 ArmOnly
        logger.info("设置 MPC 模式: ArmOnly")
        result = hardware.set_mpc_mode_sdk('ArmOnly')
        if not result.success:
            logger.error(f"MPC 模式设置失败: {result.message}")
            return
        time.sleep(1.0)

        # === 测试用例 ===
        #建议测试自己需要的用例，多个用例同时测试压力过大
        # 如需多用例测试，需降低发送频率，多停 2s，让机器人侧 MPC backlog 排空，避免跨用例累积背压
        all_passed &= test_forward_ee_pose(hardware)
        time.sleep(2.0)
        all_passed &= test_up_ee_pose(hardware)
        time.sleep(2.0)
        all_passed &= test_return_ee_pose(hardware)
        time.sleep(2.0)
        all_passed &= test_single_arm_ee_pose(hardware)

        # 恢复 MPC 模式
        logger.info("恢复 MPC 模式: NoControl")
        hardware.set_mpc_mode_sdk('NoControl')
        time.sleep(1.0)

        if all_passed:
            logger.info("🎉 手臂末端位姿（SDK 单次直调）测试完成")
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
