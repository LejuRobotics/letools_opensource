"""
状态反馈测试

使用的 Adapter 方法: hardware.get_reach_time() / get_mpc_observation() / get_mpc_control_mode()
    get_body_acceleration() / get_joint_torque() / get_ee_poses()
底层路径: StateFeedbackMixin 中的状态查询方法

测试用例说明:
- test_reach_times: 覆盖全部5种到达时间（cmd_pose/torso_pose/arm_joint/leg_joint/arm_ee）
- test_get_mpc_observation: 获取 MPC 观测状态（base_pose、joint_positions 等）
- test_get_mpc_control_mode: 获取 MPC 控制模式（0-4 有效范围）
- test_get_body_acceleration: 获取本体加速度（linear/angular）
- test_get_ee_poses: 获取左右手末端执行器位姿（left_ee、right_ee）
- test_get_joint_torque: 获取各关节力矩值
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


def test_reach_times(hardware):
    """覆盖全部5种到达时间反馈"""
    logger.info("=== 测试：全部5种到达时间反馈 ===")

    reach_types = ['cmd_pose', 'torso_pose', 'arm_joint', 'leg_joint', 'arm_ee']
    results = {}

    for rtype in reach_types:
        reach_time = hardware.get_reach_time(rtype)
        if reach_time is not None:
            logger.info(f"  ✅ {rtype} 到达时间: {reach_time:.3f} s")
            results[rtype] = True
        else:
            logger.warning(f"  ⚠️  {rtype} 到达时间: 未收到数据（可能需先发送对应控制指令）")
            results[rtype] = False

    passed = sum(1 for v in results.values() if v)
    logger.info(f"到达时间覆盖: {passed}/{len(reach_types)} 个话题有数据")
    time.sleep(0.5)


def test_get_mpc_observation(hardware):
    """获取 MPC 观测状态"""
    logger.info("=== 测试：获取 MPC 观测状态 ===")
    obs = hardware.get_mpc_observation()
    if obs is not None:
        logger.info(f"✅ MPC 观测成功:")
        logger.info(f"   base_pose: {obs.get('base_pose', 'N/A')}")
        logger.info(f"   joint_positions: {len(obs.get('joint_positions', []))} 个关节")
    else:
        logger.warning("⚠️  MPC 观测: 未收到数据")
    time.sleep(0.5)


def test_get_mpc_control_mode(hardware):
    """获取 MPC 控制模式"""
    logger.info("=== 测试：获取 MPC 控制模式 ===")
    mode = hardware.get_mpc_control_mode()
    if mode is not None:
        mode_names = {
            0: "NO_CONTROL",
            1: "ARM_ONLY",
            2: "BASE_ONLY",
            3: "BASE_ARM",
            4: "ARM_EE_ONLY"
        }
        mode_name = mode_names.get(mode, f"UNKNOWN({mode})")
        logger.info(f"✅ MPC 控制模式: {mode} - {mode_name}")
        if mode not in (0, 1, 2, 3, 4):
            logger.warning(f"⚠️  MPC 控制模式值超出有效范围 (0-4): {mode}")
    else:
        logger.warning("⚠️  MPC 控制模式: 未收到数据")
    time.sleep(0.5)


def test_get_body_acceleration(hardware):
    """获取本体加速度"""
    logger.info("=== 测试：获取本体加速度 ===")
    accel = hardware.get_body_acceleration()
    if accel is not None:
        linear = accel.get('linear', {})
        angular = accel.get('angular', {})
        logger.info(f"✅ 本体加速度获取成功:")
        logger.info(f"   线性: x={linear.get('x', 'N/A'):.4f}, y={linear.get('y', 'N/A'):.4f}, z={linear.get('z', 'N/A'):.4f} m/s²")
        logger.info(f"   角速度: x={angular.get('x', 'N/A'):.4f}, y={angular.get('y', 'N/A'):.4f}, z={angular.get('z', 'N/A'):.4f} rad/s²")
    else:
        logger.warning("⚠️  本体加速度: 未收到数据")
    time.sleep(0.5)


def test_get_ee_poses(hardware):
    """获取末端执行器位姿"""
    logger.info("=== 测试：获取末端执行器位姿 ===")
    poses = hardware.get_ee_poses()
    if poses is not None:
        logger.info(f"✅ 末端位姿获取成功: {len(poses)} 个末端")
        for i, ee in enumerate(poses):
            pos = ee.get('position', {})
            logger.info(f"   末端 {i+1}: x={pos.get('x', 'N/A'):.3f}, y={pos.get('y', 'N/A'):.3f}, z={pos.get('z', 'N/A'):.3f} m")
    else:
        logger.warning("⚠️  末端位姿: 未收到数据")
    time.sleep(0.5)


def test_get_joint_torque(hardware):
    """获取关节力矩"""
    logger.info("=== 测试：获取关节力矩 ===")
    torque = hardware.get_joint_torque()
    if torque is not None:
        torques = torque.get('torques', [])
        logger.info(f"✅ 关节力矩获取成功: {len(torques)} 个关节")
        if torques:
            logger.info(f"   前5个力矩值: {torques[:5]}...")
    else:
        logger.warning("⚠️  关节力矩: 未收到数据")
    time.sleep(0.5)


# ========== 修复项 1.8: 深度 MPC/WBC 话题测试 (10个新增方法) ==========

def test_get_wbc_observation(hardware):
    """获取WBC观测状态"""
    logger.info("=== 测试：获取WBC观测状态 ===")
    obs = hardware.get_wbc_observation()
    if obs is not None:
        logger.info("✅ WBC观测状态获取成功")
    else:
        logger.warning("⚠️  WBC观测状态: 未收到数据")
    time.sleep(0.3)


def test_get_mpc_target_input(hardware):
    """获取MPC目标输入"""
    logger.info("=== 测试：获取MPC目标输入 ===")
    data = hardware.get_mpc_target_input()
    if data is not None:
        logger.info("✅ MPC目标输入获取成功")
    else:
        logger.warning("⚠️  MPC目标输入: 未收到数据")
    time.sleep(0.3)


def test_get_mpc_target_state(hardware):
    """获取MPC目标状态"""
    logger.info("=== 测试：获取MPC目标状态 ===")
    data = hardware.get_mpc_target_state()
    if data is not None:
        logger.info("✅ MPC目标状态获取成功")
    else:
        logger.warning("⚠️  MPC目标状态: 未收到数据")
    time.sleep(0.3)


def test_get_optimized_state_mrt(hardware):
    """获取MRT优化状态"""
    logger.info("=== 测试：获取MRT优化状态 ===")
    data = hardware.get_optimized_state_mrt()
    if data is not None:
        logger.info("✅ MRT优化状态获取成功")
    else:
        logger.warning("⚠️  MRT优化状态: 未收到数据")
    time.sleep(0.3)


def test_get_optimized_state_kinemic(hardware):
    """获取运动学限制优化状态"""
    logger.info("=== 测试：获取运动学限制优化状态 ===")
    data = hardware.get_optimized_state_kinemic()
    if data is not None:
        logger.info("✅ 运动学限制优化状态获取成功")
    else:
        logger.warning("⚠️  运动学限制优化状态: 未收到数据")
    time.sleep(0.3)


def test_get_optimized_input_mrt(hardware):
    """获取MRT优化输入"""
    logger.info("=== 测试：获取MRT优化输入 ===")
    data = hardware.get_optimized_input_mrt()
    if data is not None:
        logger.info("✅ MRT优化输入获取成功")
    else:
        logger.warning("⚠️  MRT优化输入: 未收到数据")
    time.sleep(0.3)


def test_get_optimized_input_kinemic(hardware):
    """获取运动学限制优化输入"""
    logger.info("=== 测试：获取运动学限制优化输入 ===")
    data = hardware.get_optimized_input_kinemic()
    if data is not None:
        logger.info("✅ 运动学限制优化输入获取成功")
    else:
        logger.warning("⚠️  运动学限制优化输入: 未收到数据")
    time.sleep(0.3)


def test_get_ee_target_6d(hardware):
    """获取末端目标6D位姿"""
    logger.info("=== 测试：获取末端目标6D位姿 ===")
    data = hardware.get_ee_target_6d()
    if data is not None:
        logger.info(f"✅ 末端目标6D位姿获取成功: {len(data) if isinstance(data, list) else 'N/A'}")
    else:
        logger.warning("⚠️  末端目标6D位姿: 未收到数据")
    time.sleep(0.3)


def test_get_joint_acc(hardware):
    """获取关节加速度"""
    logger.info("=== 测试：获取关节加速度 ===")
    data = hardware.get_joint_acc()
    if data is not None:
        acc = data.get('accelerations', [])
        logger.info(f"✅ 关节加速度获取成功: {len(acc)} 个关节")
    else:
        logger.warning("⚠️  关节加速度: 未收到数据")
    time.sleep(0.3)


def test_get_torso_target_6d(hardware):
    """获取躯干目标6D位姿"""
    logger.info("=== 测试：获取躯干目标6D位姿 ===")
    data = hardware.get_torso_target_6d()
    if data is not None:
        logger.info("✅ 躯干目标6D位姿获取成功")
    else:
        logger.warning("⚠️  躯干目标6D位姿: 未收到数据")
    time.sleep(0.3)


def main():
    hardware = HardwareFactory.create_hardware(config={
        'robot_type': 'leju_wheeled',
        'skip_sdk_managers': True,# 状态反馈不需要 SDK 管理器
        'skip_end_effector': True,# 状态反馈不需要末端执行器
        'skip_camera': True,# 状态反馈不需要相机
        'skip_force_publishers': True,# 状态反馈不需要力控发布器
    })
    try:
        hardware.initialize()
        time.sleep(2.0)  # 等待ROS订阅建立

        # === 脚手架: 前置设置 ===
        from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown
        factory_setup(hardware, need_torso_reset=True)

        test_reach_times(hardware)
        test_get_mpc_observation(hardware)
        test_get_mpc_control_mode(hardware)
        test_get_body_acceleration(hardware)
        test_get_ee_poses(hardware)
        test_get_joint_torque(hardware)

        # 修复项 1.8: 深度 MPC/WBC 话题测试
        test_get_wbc_observation(hardware)
        test_get_mpc_target_input(hardware)
        test_get_mpc_target_state(hardware)
        test_get_optimized_state_mrt(hardware)
        test_get_optimized_state_kinemic(hardware)
        test_get_optimized_input_mrt(hardware)
        test_get_optimized_input_kinemic(hardware)
        test_get_ee_target_6d(hardware)
        test_get_joint_acc(hardware)
        test_get_torso_target_6d(hardware)

        logger.info("🎉 状态反馈测试完成")

        # === 脚手架: 后置复位 ===
        factory_teardown(hardware, need_arm=False)

    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
