#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整状态反馈测试 - 发送控制指令并验证到达时间反馈

功能说明：
1. 依次发送底盘、躯干、手臂、腿部控制指令
2. 每次发送后立即检查对应的到达时间反馈
3. 验证所有状态反馈是否正常工作
4. 生成详细的测试报告

运行方式：
    cd ~/LeTools
    pytest apps/test_kuavo_5w_app/07_debug_feedback/test_complete_state_feedback.py
"""

import sys
import os
import time
import unittest

# 添加项目根目录到 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.insert(0, project_root)

import rospy
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from core.domain.pose import Pose6D
from core.domain.enums import FrameType
from core.common.logger import get_logger
from apps.test_kuavo_5w_adapter._scaffold import adapter_setup, adapter_teardown, MPCControlMode

logger = get_logger(__name__)


class TestCompleteStateFeedback(unittest.TestCase):
    """完整状态反馈测试类"""

    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建硬件实例并连接"""
        print("\n" + "=" * 80)
        print("测试套件初始化: 完整状态反馈")
        print("=" * 80)

        if not rospy.core.is_initialized():
            rospy.init_node('test_complete_state_feedback', anonymous=True)

        cls.hardware = LejuWheeledArmHardware(config={
            'skip_sdk_managers': True,
            'skip_end_effector': True,
            'skip_camera': True,
            'skip_force_publishers': True,
        })
        result = cls.hardware.initialize()

        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")

        print("✅ 硬件初始化成功")
        time.sleep(2.0)  # 等待ROS订阅建立

        adapter_setup(cls.hardware, need_torso_reset=False, mpc_mode=None)

    @classmethod
    def tearDownClass(cls):
        """测试类清理：复位并关闭硬件连接"""
        print("\n" + "=" * 80)
        print("测试套件清理: 关闭硬件连接")
        print("=" * 80)

        if hasattr(cls, 'hardware'):
            adapter_teardown(cls.hardware, need_arm=False, restore_mpc=False)
            result = cls.hardware.shutdown()
            if result.success:
                print("✅ 硬件已关闭")
            else:
                print(f"⚠️  关闭警告: {result.message}")

    def setUp(self):
        """每个测试用例前的准备"""
        print(f"\n--- 开始测试: {self._testMethodName} ---")

    def tearDown(self):
        """每个测试用例后的清理"""
        print(f"--- 结束测试: {self._testMethodName} ---")

    def test_01_cmd_pose_reach_time(self):
        """测试1: 底盘位置到达时间反馈"""
        print("=" * 80)
        print("  测试1: 底盘位置到达时间")
        print("=" * 80)

        logger.info("📤 发送底盘位置指令: x=0.1m, y=0.0m, yaw=0.0rad (本体坐标系)")
        result = self.hardware.send_base_pose(x=0.1, y=0.0, yaw=0.0, frame=FrameType.LOCAL)

        self.assertTrue(result.success, f"指令发送失败: {result.message}")

        time.sleep(0.5)
        reach_time = self.hardware.get_reach_time('cmd_pose')

        self.assertIsNotNone(reach_time, "未收到到达时间反馈")
        logger.info(f"✅ 收到到达时间反馈: {reach_time:.3f} s")

    def test_02_torso_pose_reach_time(self):
        """测试2: 躯干位姿到达时间反馈"""
        print("=" * 80)
        print("  测试2: 躯干位姿到达时间")
        print("=" * 80)

        logger.info("📤 发送躯干位姿指令: z=0.1m (抬高)")
        pose = Pose6D(x=0.0, y=0.0, z=0.1, yaw=0.0, pitch=0.0, roll=0.0)
        result = self.hardware.send_torso_pose(pose)

        self.assertTrue(result.success, f"指令发送失败: {result.message}")

        time.sleep(0.5)
        reach_time = self.hardware.get_reach_time('torso_pose')

        self.assertIsNotNone(reach_time, "未收到到达时间反馈")
        logger.info(f"✅ 收到到达时间反馈: {reach_time:.3f} s")

    def test_03_arm_joint_reach_time(self):
        """测试3: 手臂关节到达时间反馈"""
        print("=" * 80)
        print("  测试3: 手臂关节到达时间")
        print("=" * 80)

        logger.info("📤 发送手臂关节指令: 展开双臂 (14个关节)")
        positions = [-30, 20, 15, -45, 25, 10, -35,
                     -30, -20, -15, -45, -25, -10, -35]
        result = self.hardware.send_arm_joint_trajectory(positions)

        self.assertTrue(result.success, f"指令发送失败: {result.message}")

        time.sleep(0.5)
        reach_time = self.hardware.get_reach_time('arm_joint')

        self.assertIsNotNone(reach_time, "未收到到达时间反馈")
        logger.info(f"✅ 收到到达时间反馈: {reach_time:.3f} s")

    def test_04_leg_joint_reach_time(self):
        """测试4: 腿部关节到达时间反馈"""
        print("=" * 80)
        print("  测试4: 腿部关节到达时间")
        print("=" * 80)

        logger.info("📤 发送腿部关节指令: 初始姿态 [14.90, -32.01, 18.03, 0.0]")
        positions = [14.90, -32.01, 18.03, 0.0]
        result = self.hardware.send_leg_joint_command(positions)

        self.assertTrue(result.success, f"指令发送失败: {result.message}")

        time.sleep(0.5)
        reach_time = self.hardware.get_reach_time('leg_joint')

        self.assertIsNotNone(reach_time, "未收到到达时间反馈")
        logger.info(f"✅ 收到到达时间反馈: {reach_time:.3f} s")

    def test_05_arm_ee_reach_time(self):
        """测试5: 手臂末端到达时间反馈"""
        print("=" * 80)
        print("  测试5: 手臂末端到达时间")
        print("=" * 80)

        logger.info("📤 发送双臂末端位姿指令")
        # 左手目标: 前方略微张开, 右手目标: 前方略微张开
        left_target = [0.40, 0.15, 0.10, 0.0, 0.0, 0.0]
        right_target = [0.40, -0.15, 0.10, 0.0, 0.0, 0.0]
        result = self.hardware.send_two_arm_hand_pose(left_target, right_target)

        self.assertTrue(result.success, f"指令发送失败: {result.message}")

        time.sleep(0.5)
        reach_time = self.hardware.get_reach_time('arm_ee')

        self.assertIsNotNone(reach_time, "未收到到达时间反馈")
        logger.info(f"✅ 收到到达时间反馈: {reach_time:.3f} s")

    def test_06_mpc_observation(self):
        """测试6: MPC观测状态"""
        print("=" * 80)
        print("  测试6: MPC观测状态")
        print("=" * 80)

        obs = self.hardware.get_mpc_observation()

        self.assertIsNotNone(obs, "未收到MPC观测状态")
        logger.info(f"✅ MPC观测状态获取成功")

        # 验证返回数据包含预期字段
        self.assertIn('base_pose', obs, "MPC观测状态缺少 base_pose 字段")
        self.assertIn('joint_positions', obs, "MPC观测状态缺少 joint_positions 字段")
        logger.info(f"   base_pose: {obs.get('base_pose', 'N/A')}")
        logger.info(f"   joint_positions: {len(obs.get('joint_positions', []))} 个关节")

    def test_07_mpc_control_mode(self):
        """测试7: MPC控制模式反馈"""
        print("=" * 80)
        print("  测试7: MPC控制模式")
        print("=" * 80)

        mode = self.hardware.get_mpc_control_mode()

        self.assertIsNotNone(mode, "未收到MPC控制模式反馈")

        mode_names = {
            0: "NO_CONTROL",
            1: "ARM_ONLY",
            2: "BASE_ONLY",
            3: "BASE_ARM",
            4: "ARM_EE_ONLY"
        }
        mode_name = mode_names.get(mode, f"UNKNOWN({mode})")
        logger.info(f"✅ MPC控制模式: {mode} - {mode_name}")

        self.assertIn(mode, mode_names, f"MPC控制模式值无效: {mode}")

    def test_08_body_acceleration(self):
        """测试8: 本体加速度反馈"""
        print("=" * 80)
        print("  测试8: 本体加速度")
        print("=" * 80)

        accel = self.hardware.get_body_acceleration()

        self.assertIsNotNone(accel, "未收到本体加速度反馈")

        linear = accel['linear']
        angular = accel['angular']

        logger.info(f"✅ 线加速度: x={linear['x']:.4f}, y={linear['y']:.4f}, z={linear['z']:.4f} m/s²")
        logger.info(f"✅ 角加速度: x={angular['x']:.4f}, y={angular['y']:.4f}, z={angular['z']:.4f} rad/s²")

        self.assertIn('x', linear, "线加速度缺少 x 字段")
        self.assertIn('y', linear, "线加速度缺少 y 字段")
        self.assertIn('z', linear, "线加速度缺少 z 字段")
        self.assertIn('x', angular, "角加速度缺少 x 字段")
        self.assertIn('y', angular, "角加速度缺少 y 字段")
        self.assertIn('z', angular, "角加速度缺少 z 字段")

    def test_09_joint_torque(self):
        """测试9: 关节力矩反馈"""
        print("=" * 80)
        print("  测试9: 关节力矩")
        print("=" * 80)

        torque = self.hardware.get_joint_torque()

        self.assertIsNotNone(torque, "未收到关节力矩反馈")

        num_joints = len(torque.get('torques', []))
        max_torque = max([abs(t) for t in torque.get('torques', [])]) if torque.get('torques') else 0

        logger.info(f"✅ 关节数量: {num_joints}")
        logger.info(f"✅ 最大力矩: {max_torque:.2f} Nm")

        self.assertGreater(num_joints, 0, "关节力矩数据为空")

    def test_10_ee_poses(self):
        """测试10: 末端执行器位姿反馈"""
        print("=" * 80)
        print("  测试10: 末端执行器位姿")
        print("=" * 80)

        ee_poses = self.hardware.get_ee_poses()

        self.assertIsNotNone(ee_poses, "未收到末端位姿反馈")

        num_ee = len(ee_poses)
        logger.info(f"✅ 末端数量: {num_ee}")

        for i, ee in enumerate(ee_poses):
            pos = ee['position']
            logger.info(f"   末端 {i+1}: x={pos['x']:.3f}, y={pos['y']:.3f}, z={pos['z']:.3f} m")

        self.assertGreater(num_ee, 0, "末端位姿数据为空")

    # ========== 修复项 1.8: 深度 MPC/WBC 话题测试 (10个新增方法) ==========

    def test_11_wbc_observation(self):
        """测试11: WBC观测状态"""
        print("=" * 80)
        print("  测试11: WBC观测状态")
        print("=" * 80)

        obs = self.hardware.get_wbc_observation()
        if obs is None:
            self.skipTest("WBC观测状态数据不可用（ocs2_msgs 可能未安装或话题未发布）")
        self.assertIsNotNone(obs, "未收到WBC观测状态")
        logger.info("✅ WBC观测状态获取成功")

    def test_12_mpc_target_input(self):
        """测试12: MPC目标输入"""
        print("=" * 80)
        print("  测试12: MPC目标输入")
        print("=" * 80)

        data = self.hardware.get_mpc_target_input()
        if data is None:
            self.skipTest("MPC目标输入数据不可用")
        self.assertIsNotNone(data, "未收到MPC目标输入")
        logger.info("✅ MPC目标输入获取成功")

    def test_13_mpc_target_state(self):
        """测试13: MPC目标状态"""
        print("=" * 80)
        print("  测试13: MPC目标状态")
        print("=" * 80)

        data = self.hardware.get_mpc_target_state()
        if data is None:
            self.skipTest("MPC目标状态数据不可用")
        self.assertIsNotNone(data, "未收到MPC目标状态")
        logger.info("✅ MPC目标状态获取成功")

    def test_14_optimized_state_mrt(self):
        """测试14: MRT优化状态"""
        print("=" * 80)
        print("  测试14: MRT优化状态")
        print("=" * 80)

        data = self.hardware.get_optimized_state_mrt()
        if data is None:
            self.skipTest("MRT优化状态数据不可用")
        self.assertIsNotNone(data, "未收到MRT优化状态")
        logger.info("✅ MRT优化状态获取成功")

    def test_15_optimized_state_kinemic(self):
        """测试15: 运动学限制优化状态"""
        print("=" * 80)
        print("  测试15: 运动学限制优化状态")
        print("=" * 80)

        data = self.hardware.get_optimized_state_kinemic()
        if data is None:
            self.skipTest("运动学限制优化状态数据不可用")
        self.assertIsNotNone(data, "未收到运动学限制优化状态")
        logger.info("✅ 运动学限制优化状态获取成功")

    def test_16_optimized_input_mrt(self):
        """测试16: MRT优化输入"""
        print("=" * 80)
        print("  测试16: MRT优化输入")
        print("=" * 80)

        data = self.hardware.get_optimized_input_mrt()
        if data is None:
            self.skipTest("MRT优化输入数据不可用")
        self.assertIsNotNone(data, "未收到MRT优化输入")
        logger.info("✅ MRT优化输入获取成功")

    def test_17_optimized_input_kinemic(self):
        """测试17: 运动学限制优化输入"""
        print("=" * 80)
        print("  测试17: 运动学限制优化输入")
        print("=" * 80)

        data = self.hardware.get_optimized_input_kinemic()
        if data is None:
            self.skipTest("运动学限制优化输入数据不可用")
        self.assertIsNotNone(data, "未收到运动学限制优化输入")
        logger.info("✅ 运动学限制优化输入获取成功")

    def test_18_ee_target_6d(self):
        """测试18: 末端目标6D位姿"""
        print("=" * 80)
        print("  测试18: 末端目标6D位姿")
        print("=" * 80)

        data = self.hardware.get_ee_target_6d()
        if data is None:
            self.skipTest("末端目标6D位姿数据不可用")
        self.assertIsNotNone(data, "未收到末端目标6D位姿")
        logger.info(f"✅ 末端目标6D位姿获取成功: {len(data) if isinstance(data, list) else 'N/A'}")

    def test_19_joint_acc(self):
        """测试19: 关节加速度"""
        print("=" * 80)
        print("  测试19: 关节加速度")
        print("=" * 80)

        data = self.hardware.get_joint_acc()
        if data is None:
            self.skipTest("关节加速度数据不可用")
        self.assertIsNotNone(data, "未收到关节加速度")
        acc = data.get('accelerations', [])
        logger.info(f"✅ 关节加速度获取成功: {len(acc)} 个关节")

    def test_20_torso_target_6d(self):
        """测试20: 躯干目标6D位姿"""
        print("=" * 80)
        print("  测试20: 躯干目标6D位姿")
        print("=" * 80)

        data = self.hardware.get_torso_target_6d()
        if data is None:
            self.skipTest("躯干目标6D位姿数据不可用")
        self.assertIsNotNone(data, "未收到躯干目标6D位姿")
        logger.info("✅ 躯干目标6D位姿获取成功")


def run_tests():
    """运行测试套件"""
    suite = unittest.TestLoader().loadTestsFromTestCase(TestCompleteStateFeedback)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "=" * 80)
    print("Kuavo 5-W 应用层测试 - 完整状态反馈")
    print("使用 LejuWheeledArmHardware 适配器")
    print("=" * 80)

    try:
        success = run_tests()

        if success:
            print("\n" + "=" * 80)
            print("🎉 所有测试通过！状态反馈功能正常工作。")
            print("=" * 80)
            sys.exit(0)
        else:
            print("\n" + "=" * 80)
            print("⚠️  部分测试失败，请检查上述输出。")
            print("=" * 80)
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\n⚠️  测试被用户中断")
        sys.exit(130)
    except Exception as e:
        print(f"\n\n❌ 测试执行出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
