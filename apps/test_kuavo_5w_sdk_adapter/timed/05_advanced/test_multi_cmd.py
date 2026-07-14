"""
多指令并发测试

使用的 Adapter 方法: hardware.send_timed_multi_commands()
底层路径: ROS 服务 /mobile_manipulator_timed_multi_cmd（多指令合并）

角度单位: 本脚本使用角度制（度），由 _convert_multi_cmd_angles 内部统一转换为弧度。

测试用例说明:
- test_base_plus_arm: 底盘前进 0.3m + 双臂前伸（肩关节 30°）同步执行，持续 3 秒
- test_base_plus_leg: 底盘旋转 0.2rad + 下肢站立姿态（膝关节微曲）同步执行，持续 2 秒
- test_arm_plus_leg: 下肢+双臂关节组合 (planner 3+8+9)，多组测试用例 + 零位复位
- test_chassis_plus_arm_ee: 底盘+双臂EE局部系组合 (planner 0+6+7)，5组测试用例
- test_chassis_torso_arm_ee: 底盘+躯干+双臂EE四规划器组合 (planner 0+2+6+7)，5组测试用例
- test_torso_arm_ee: 躯干+双臂EE三规划器组合 (planner 2+6+7)，6组测试用例
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


def _get_torso_initial_pose():
    """获取躯干初始位姿（直接调用ROS服务）"""
    import rospy
    from kuavo_msgs.srv import getLbTorsoInitialPose, getLbTorsoInitialPoseRequest

    try:
        rospy.wait_for_service('/mobile_manipulator_get_torso_initial_pose', timeout=5.0)
        service_client = rospy.ServiceProxy(
            '/mobile_manipulator_get_torso_initial_pose', getLbTorsoInitialPose
        )
        req = getLbTorsoInitialPoseRequest()
        req.getFromService = True
        response = service_client(req)
        if response.success:
            return list(response.torsoPose)
        else:
            logger.error("无法获取躯干初始位姿")
            return None
    except Exception as e:
        logger.error(f"获取躯干初始位姿失败: {e}")
        return None


def test_base_plus_arm(hardware):
    """底盘 + 手臂同步运动"""
    logger.info("=== 测试：底盘前进 + 双臂前伸（同步） ===")
    commands = [
        {
            'planner_index': 0,  # 底盘世界系
            'desire_time': 3.0,
            'cmd_vec': [0.3, 0.0, 0.0],
        },
        {
            'planner_index': 8,  # 左臂关节
            'desire_time': 3.0,
            'cmd_vec': [-30.0] + [0.0] * 6,
        },
        {
            'planner_index': 9,  # 右臂关节
            'desire_time': 3.0,
            'cmd_vec': [-30.0] + [0.0] * 6,
        },
    ]
    result = hardware.send_timed_multi_commands(commands, is_sync=True)
    if result.success:
        logger.info(f"✅ 多指令成功，实际时间: {result.data.get('actual_time', 'N/A')}s")
    else:
        logger.error(f"❌ 多指令失败: {result.message}")
    time.sleep(4.0)


def test_base_plus_leg(hardware):
    """底盘 + 下肢同步运动"""
    logger.info("=== 测试：底盘旋转 + 下肢姿态（同步） ===")
    commands = [
        {
            'planner_index': 0,  # 底盘世界系
            'desire_time': 2.0,
            'cmd_vec': [0.0, 0.0, 11.46],  # yaw=0.2rad≈11.46°（_convert_multi_cmd_angles 内部转弧度）
        },
        {
            'planner_index': 3,  # 下肢关节
            'desire_time': 2.0,
            'cmd_vec': [-10.0, 20.0, -10.0, 20.0],
        },
    ]
    result = hardware.send_timed_multi_commands(commands, is_sync=True)
    if result.success:
        logger.info(f"✅ 底盘+下肢成功")
    else:
        logger.error(f"❌ 底盘+下肢失败: {result.message}")
    time.sleep(3.0)


def test_arm_plus_leg(hardware):
    """下肢 + 双臂关节组合运动 (planner 3+8+9)"""
    logger.info("=== 测试：下肢 + 双臂关节组合（同步） ===")

    test_cases = [
        ("测试1-正常组合", 4.0,
         [14.90, -32.01, 18.03, 0.0],
         [-30, 20, 15, -45, 25, 10, -35, -30, -20, -15, -45, -25, -10, -35]),
        ("测试2-正常组合", 4.0,
         [14.90, -32.01, 18.03, 30.0],
         [-20, 30, -25, -20, 40, -15, 25, -20, -30, 25, -20, -40, 15, 25]),
        ("测试3-零位复位", 4.0,
         [0.0] * 4,
         [0.0] * 14),
    ]

    for name, desire_time, leg_deg, arm_deg in test_cases:
        logger.info(f"  {name}: desire_time={desire_time}s")

        commands = [
            {'planner_index': 3, 'desire_time': desire_time, 'cmd_vec': list(leg_deg)},
            {'planner_index': 8, 'desire_time': desire_time, 'cmd_vec': list(arm_deg[:7])},
            {'planner_index': 9, 'desire_time': desire_time, 'cmd_vec': list(arm_deg[7:14])},
        ]
        result = hardware.send_timed_multi_commands(commands, is_sync=True)
        if result.success:
            logger.info(f"  ✅ {name} 成功")
        else:
            logger.error(f"  ❌ {name} 失败: {result.message}")
        time.sleep(desire_time + 0.5)


def test_chassis_plus_arm_ee(hardware):
    """底盘 + 双臂EE局部系组合运动 (planner 0+6+7)"""
    logger.info("=== 测试：底盘 + 双臂EE局部系组合（同步） ===")

    test_cases = [
        ("底盘不动+手臂展开", 3.0,
         [0.0, 0.0, 0.0],
         [0.1, 0.4, 0.7, 0.0, 0.0, 0.0],
         [0.1, -0.4, 0.7, 0.0, 0.0, 0.0]),
        ("底盘后退+手臂前摆", 4.0,
         [-0.3, 0.0, 0.0],
         [0.3, 0.2, 0.85, 0.0, -90, 0.0],
         [0.3, -0.2, 0.85, 0.0, -90, 0.0]),
        ("底盘前进+手臂收回", 4.0,
         [0.0, 0.0, 0.0],
         [0.5, 0.2, 0.7, 0.0, -90, 0.0],
         [0.5, -0.2, 0.7, 0.0, -90, 0.0]),
        ("底盘旋转+手臂前伸", 5.0,
         [-0.3, 0.0, 90.0],  # yaw=1.57rad≈90°
         [1.2, 0.2, 0.85, 0.0, -90, 0.0],
         [1.2, -0.2, 0.85, 0.0, -90, 0.0]),
        ("零位复位", 3.0,
         [0.0, 0.0, 0.0],
         [0.5, 0.2, 0.85, 0.0, -90, 0.0],
         [0.5, -0.2, 0.85, 0.0, -90, 0.0]),
    ]

    for name, desire_time, base_pose, left_pose, right_pose in test_cases:
        logger.info(f"  {name}: desire_time={desire_time}s")

        commands = [
            {'planner_index': 0, 'desire_time': desire_time, 'cmd_vec': base_pose},
            {'planner_index': 6, 'desire_time': desire_time, 'cmd_vec': list(left_pose)},
            {'planner_index': 7, 'desire_time': desire_time, 'cmd_vec': list(right_pose)},
        ]
        result = hardware.send_timed_multi_commands(commands, is_sync=True)
        if result.success:
            logger.info(f"  ✅ {name} 成功")
        else:
            logger.error(f"  ❌ {name} 失败: {result.message}")
        time.sleep(desire_time + 0.5)


def test_chassis_torso_arm_ee(hardware):
    """底盘 + 躯干 + 双臂EE 四规划器组合运动 (planner 0+2+6+7)"""
    logger.info("=== 测试：底盘 + 躯干 + 双臂EE 四规划器组合（同步） ===")

    initial_torso_pos = _get_torso_initial_pose()
    if initial_torso_pos is None:
        logger.error("  无法获取躯干初始位姿，跳过测试")
        return

    test_cases = [
        ("初始抬高+手臂展开", 5.0,
         [0.0, 0.0, 0.0],
         [0.0, 0.3, 0.0, 0.0],
         [0.1, 0.4, 1.0, 0.0, 0.0, 0.0],
         [0.1, -0.4, 1.0, 0.0, 0.0, 0.0]),
        ("底盘后退+手臂前摆", 5.0,
         [-0.3, 0.0, 0.0],
         [0.2, 0.3, 0.0, 0.0],
         [0.5, 0.2, 1.15, 0.0, -90, 0.0],
         [0.5, -0.2, 1.15, 0.0, -90, 0.0]),
        ("底盘前进+偏航30°", 5.0,
         [0.0, 0.0, 0.0],
         [0.2, 0.3, 30.0, 0.0],  # torso yaw: 0.52356rad≈30°
         [0.3, 0.2, 1.0, 0.0, -90, 0.0],
         [0.3, -0.2, 1.0, 0.0, -90, 0.0]),
        ("底盘旋转+俯仰", 6.0,
         [-0.3, 0.0, 90.0],  # chassis yaw: 1.57rad≈90°
         [0.2, 0.3, 0.0, 30.0],  # torso pitch: 0.524rad≈30°
         [1.2, 0.2, 1.15, 0.0, -90, 0.0],
         [1.2, -0.2, 1.15, 0.0, -90, 0.0]),
        ("复位", 5.0,
         [0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0, 0.0],
         [0.1, 0.4, 0.7, 0.0, 0.0, 0.0],
         [0.1, -0.4, 0.7, 0.0, 0.0, 0.0]),
    ]

    for name, desire_time, base_pose, torso_rel, left_pose, right_pose in test_cases:
        lx, lz, az, ay = torso_rel
        abs_x = initial_torso_pos[0] + lx
        abs_z = initial_torso_pos[2] + lz
        torso_pose = [abs_x, abs_z, az, ay]

        logger.info(f"  {name}: desire_time={desire_time}s, torso_abs=[{abs_x:.3f}, {abs_z:.3f}, {az:.2f}, {ay:.2f}]")

        commands = [
            {'planner_index': 0, 'desire_time': desire_time, 'cmd_vec': base_pose},
            {'planner_index': 2, 'desire_time': desire_time, 'cmd_vec': torso_pose},
            {'planner_index': 6, 'desire_time': desire_time, 'cmd_vec': list(left_pose)},
            {'planner_index': 7, 'desire_time': desire_time, 'cmd_vec': list(right_pose)},
        ]
        result = hardware.send_timed_multi_commands(commands, is_sync=True)
        if result.success:
            logger.info(f"  ✅ {name} 成功")
        else:
            logger.error(f"  ❌ {name} 失败: {result.message}")
        time.sleep(desire_time + 0.5)


def test_torso_arm_ee(hardware):
    """躯干 + 双臂EE 三规划器组合运动 (planner 2+6+7)"""
    logger.info("=== 测试：躯干 + 双臂EE 三规划器组合（同步） ===")

    initial_torso_pos = _get_torso_initial_pose()
    if initial_torso_pos is None:
        logger.error("  无法获取躯干初始位姿，跳过测试")
        return

    test_cases = [
        ("初始抬高+手臂展开", 4.0,
         [0.0, 0.3, 0.0, 0.0],
         [0.1, 0.4, 1.0, 0.0, 0.0, 0.0],
         [0.1, -0.4, 1.0, 0.0, 0.0, 0.0]),
        ("前移+手臂前摆", 4.0,
         [0.2, 0.3, 0.0, 0.0],
         [0.5, 0.2, 1.15, 0.0, -90, 0.0],
         [0.5, -0.2, 1.15, 0.0, -90, 0.0]),
        ("偏航30°+手臂收回", 4.0,
         [0.2, 0.3, 30.0, 0.0],  # torso yaw: 0.52356rad≈30°
         [0.3, 0.2, 1.0, 0.0, -90, 0.0],
         [0.3, -0.2, 1.0, 0.0, -90, 0.0]),
        ("俯仰+手臂前伸", 4.0,
         [0.2, 0.3, 0.0, 30.0],  # torso pitch: 0.524rad≈30°
         [1.2, 0.2, 1.15, 0.0, -90, 0.0],
         [1.2, -0.2, 1.15, 0.0, -90, 0.0]),
        ("综合姿态", 5.0,
         [0.2, 0.3, 30.0, 30.0],  # yaw=0.52356rad≈30°, pitch=0.524rad≈30°
         [0.5, 0.2, 1.15, 0.0, -90, 0.0],
         [0.5, -0.2, 1.15, 0.0, -90, 0.0]),
        ("复位", 4.0,
         [0.0, 0.0, 0.0, 0.0],
         [0.1, 0.4, 0.7, 0.0, 0.0, 0.0],
         [0.1, -0.4, 0.7, 0.0, 0.0, 0.0]),
    ]

    for name, desire_time, torso_rel, left_pose, right_pose in test_cases:
        lx, lz, az, ay = torso_rel
        abs_x = initial_torso_pos[0] + lx
        abs_z = initial_torso_pos[2] + lz
        torso_pose = [abs_x, abs_z, az, ay]

        logger.info(f"  {name}: desire_time={desire_time}s, torso_abs=[{abs_x:.3f}, {abs_z:.3f}, {az:.2f}, {ay:.2f}]")

        commands = [
            {'planner_index': 2, 'desire_time': desire_time, 'cmd_vec': torso_pose},
            {'planner_index': 6, 'desire_time': desire_time, 'cmd_vec': list(left_pose)},
            {'planner_index': 7, 'desire_time': desire_time, 'cmd_vec': list(right_pose)},
        ]
        result = hardware.send_timed_multi_commands(commands, is_sync=True)
        if result.success:
            logger.info(f"  ✅ {name} 成功")
        else:
            logger.error(f"  ❌ {name} 失败: {result.message}")
        time.sleep(desire_time + 0.5)


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

        test_base_plus_arm(hardware)
        test_base_plus_leg(hardware)
        test_arm_plus_leg(hardware)
        test_chassis_plus_arm_ee(hardware)
        test_chassis_torso_arm_ee(hardware)
        test_torso_arm_ee(hardware)
        logger.info("🎉 多指令并发测试完成")

        factory_teardown(hardware, need_arm=True)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
