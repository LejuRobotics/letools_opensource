"""
Ruckig 规划器参数配置测试

使用的 Adapter 方法: hardware.set_ruckig_planner_params() + hardware.send_timed_left_arm_joint()
底层路径: TimedCmdManager → /mobile_manipulator_set_ruckig_planner_params

测试用例说明:
- test_01_base_pose_planner_params: 配置底盘位置规划器（Ruckig 编号 0）
- test_02_left_arm_joint_planner_params: 配置左臂关节规划器（Ruckig 编号 6）
- test_03_right_arm_joint_planner_params: 配置右臂关节规划器（Ruckig 编号 7）
- test_04_leg_joint_planner_params: 配置下肢关节规划器（Ruckig 编号 3）
- test_05_torso_cartesian_planner_params: 配置躯干规划器（Ruckig 编号 2）
- test_06_custom_min_params: 配置底盘非对称的最小速度、最小加速度
- test_07_verify_with_motion: 发送左臂关节指令，等待返回的规划时长

注意: Ruckig 参数编号 6/7 与左右臂 TimedCmd 运动编号 8/9 不同。
参数配置失败会停止测试；运动用例仅检查请求是否成功，不量测实际轨迹。
使用本目录的 Adapter 脚手架，前后复位手臂、躯干，退出时释放 MPC 控制。
修改的 Ruckig 限制不会自动恢复。

运行方式:
    python3 apps/test_kuavo_5w_adapter/04_timed_commands/test_ruckig_params.py
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()
logger = get_logger(__name__)

import time
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from apps.test_kuavo_5w_adapter._scaffold import (
    adapter_setup,
    adapter_teardown,
    check_services_available,
)


# 左臂 7 个关节的目标角度（度）；速度、加速度、急动度限制使用弧度单位。
LEFT_ARM_JOINTS = [-20.0, 15.0, 10.0, -35.0, 20.0, 8.0, -25.0]
DESIRE_TIME = 3.0


def test_01_base_pose_planner_params(hardware):
    """配置底盘位置规划器（3D: x、y、yaw）。"""
    logger.info("=== 测试：底盘位置规划器参数 (planner_index=0) ===")
    result = hardware.set_ruckig_planner_params(
        planner_index=0,
        is_sync=True,
        velocity_max=[0.2, 0.2, 0.2],
        acceleration_max=[2.0, 2.0, 1.5],
        jerk_max=[20.0, 15.0, 12.0],
    )
    if result.success:
        logger.info("✅ 底盘位置规划器参数设置成功")
    else:
        logger.error(f"❌ 底盘位置规划器参数设置失败: {result.message}")
        raise RuntimeError(result.message)
    time.sleep(2.0)


def test_02_left_arm_joint_planner_params(hardware):
    """配置左臂关节规划器（7D）。"""
    logger.info("=== 测试：左臂关节规划器参数 (planner_index=6) ===")
    result = hardware.set_ruckig_planner_params(
        planner_index=6,
        is_sync=True,
        velocity_max=[1.0] * 7,
        acceleration_max=[5.0] * 7,
        jerk_max=[50.0] * 7,
    )
    if result.success:
        logger.info("✅ 左臂关节规划器参数设置成功")
    else:
        logger.error(f"❌ 左臂关节规划器参数设置失败: {result.message}")
        raise RuntimeError(result.message)
    time.sleep(2.0)


def test_03_right_arm_joint_planner_params(hardware):
    """配置右臂关节规划器（7D）。"""
    logger.info("=== 测试：右臂关节规划器参数 (planner_index=7) ===")
    result = hardware.set_ruckig_planner_params(
        planner_index=7,
        is_sync=True,
        velocity_max=[1.0] * 7,
        acceleration_max=[5.0] * 7,
        jerk_max=[50.0] * 7,
    )
    if result.success:
        logger.info("✅ 右臂关节规划器参数设置成功")
    else:
        logger.error(f"❌ 右臂关节规划器参数设置失败: {result.message}")
        raise RuntimeError(result.message)
    time.sleep(2.0)


def test_04_leg_joint_planner_params(hardware):
    """配置下肢关节规划器（4D）。"""
    logger.info("=== 测试：下肢关节规划器参数 (planner_index=3) ===")
    result = hardware.set_ruckig_planner_params(
        planner_index=3,
        is_sync=True,
        velocity_max=[2.0] * 4,
        acceleration_max=[10.0] * 4,
        jerk_max=[100.0] * 4,
    )
    if result.success:
        logger.info("✅ 下肢关节规划器参数设置成功")
    else:
        logger.error(f"❌ 下肢关节规划器参数设置失败: {result.message}")
        raise RuntimeError(result.message)
    time.sleep(2.0)


def test_05_torso_cartesian_planner_params(hardware):
    """配置躯干规划器（4D: x、z、yaw、pitch）。"""
    logger.info("=== 测试：躯干规划器参数 (planner_index=2) ===")
    result = hardware.set_ruckig_planner_params(
        planner_index=2,
        is_sync=True,
        velocity_max=[0.1, 0.1, 0.2, 0.2],
        acceleration_max=[1.0, 1.0, 2.0, 2.0],
        jerk_max=[10.0, 10.0, 20.0, 20.0],
    )
    if result.success:
        logger.info("✅ 躯干规划器参数设置成功")
    else:
        logger.error(f"❌ 躯干规划器参数设置失败: {result.message}")
        raise RuntimeError(result.message)
    time.sleep(2.0)


def test_06_custom_min_params(hardware):
    """配置底盘非对称的最小速度、最小加速度。"""
    logger.info("=== 测试：底盘自定义最小值参数 (planner_index=0) ===")
    result = hardware.set_ruckig_planner_params(
        planner_index=0,
        is_sync=False,
        velocity_max=[0.3, 0.3, 0.3],
        acceleration_max=[3.0, 3.0, 2.0],
        jerk_max=[30.0, 25.0, 20.0],
        velocity_min=[-0.1, -0.1, -0.1],
        acceleration_min=[-1.0, -1.0, -0.5],
    )
    if result.success:
        logger.info("✅ 底盘自定义最小值参数设置成功")
    else:
        logger.error(f"❌ 底盘自定义最小值参数设置失败: {result.message}")
        raise RuntimeError(result.message)
    time.sleep(3.0)


def test_07_verify_with_motion(hardware):
    """发送左臂关节指令，供观察配置后的运动效果。"""
    logger.info("=== 测试：配置 Ruckig 参数后执行左臂运动 ===")
    logger.info(f"左臂目标角度: {LEFT_ARM_JOINTS}°")
    result = hardware.send_timed_left_arm_joint(
        joint_angles=LEFT_ARM_JOINTS,
        desire_time=DESIRE_TIME,
    )
    if not result.success:
        logger.error(f"❌ 左臂指令失败: {result.message}")
        raise RuntimeError(result.message)
    actual_time = float((result.data or {}).get("actual_time", DESIRE_TIME))
    logger.info(f"✅ 左臂指令已接受，规划时长: {actual_time:.2f}s")
    time.sleep(actual_time)
    time.sleep(0.5)


def main():
    hardware = LejuWheeledArmHardware()
    setup_started = False
    try:
        result = hardware.initialize()
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")
        ok, missing = check_services_available([
            '/mobile_manipulator_set_ruckig_planner_params',
        ])
        if not ok:
            raise RuntimeError(f"ROS 服务不可用: {missing}，请启动控制器进程")
        time.sleep(2.0)
        setup_started = True
        adapter_setup(hardware, need_arm=True, need_torso_reset=True)

        # 与 test_left_arm_joint.py 一样，在这里明确选择、排列测试函数。
        test_01_base_pose_planner_params(hardware)
        test_02_left_arm_joint_planner_params(hardware)
        test_03_right_arm_joint_planner_params(hardware)
        test_04_leg_joint_planner_params(hardware)
        test_05_torso_cartesian_planner_params(hardware)
        test_06_custom_min_params(hardware)
        test_07_verify_with_motion(hardware)
        logger.info("🎉 Ruckig 参数测试完成")

    finally:
        try:
            if setup_started:
                adapter_teardown(hardware, need_arm=True, restore_mpc=True)
        finally:
            hardware.shutdown()


if __name__ == "__main__":
    main()
