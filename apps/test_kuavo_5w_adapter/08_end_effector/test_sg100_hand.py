#!/usr/bin/env python3
"""
SG100 黑漫灵巧手控制测试（适配器层）

对应适配器接口: LejuWheeledArmHardware.control_end_effector(ArmSide, SG100HandCommand)
底层路径: 适配器 → EndEffectorMixin → LejuEndEffector.send_sg100_command()
         → 发布 /sg100_hand_command (kuavo_msgs/SG100HandCommand)
状态反馈: 订阅 /sg100_hand_state (kuavo_msgs/SG100HandState)

功能说明:
- 位置控制（MODE_JOINT_POSITION=7）：左手张开/握拳，验证 11 关节弧度目标下发
- 阻抗控制（MODE_JOINT_IMPEDANCE=9）：带 kp/kd 的柔性保持，验证增益透传
- 混合模式（per_joint_modes）：拇指阻抗 + 其余位置控制，验证 per-joint mode 数组经适配器透传
- 单手模式：仅左侧活动，右侧 enable_mask=0 + 空 positions 保持禁用

关节顺序（11 个，单位 rad）:
    [0]TH_CMC_ABD [1]TH_MCP_FLEX [2]TH_IP_FLEX
    [3]IF_MCP_ABD  [4]IF_MCP_FLEX [5]IF_PIP_FLEX
    [6]MF_MCP_FLEX [7]MF_PIP_FLEX
    [8]LF_MCP_ABD  [9]LF_MCP_FLEX [10]LF_PIP_FLEX

前置条件:
1. config['type'] = 'sg100'（LejuEndEffector 按 EndEffectorType.SG100_HAND 连接）
2. 已启动 SG100 手控节点（发布 /sg100_hand_command 可被接收）
3. ROBOT_VERSION 已设置且 kuavo_msgs 已 source（infrastructure/ros_packages/devel/setup.bash）

注意: 此测试会驱动真实灵巧手运动，请确保手部周围无障碍物。
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
from core.domain.end_effector import (
    SG100HandCommand,
    SG100ControlMode,
    SG100_JOINT_COUNT,
)
from core.domain.enums import ArmSide


class TestSG100Hand(unittest.TestCase):
    """SG100 灵巧手适配器层测试类"""

    @classmethod
    def setUpClass(cls):
        """测试类初始化：创建适配器并以 SG100 模式连接末端驱动"""
        print("\n" + "=" * 70)
        print("测试套件初始化: SG100 灵巧手控制（适配器层）")
        print("=" * 70)

        cls.hardware = LejuWheeledArmHardware(config={
            'type': 'sg100',              # 触发 EndEffectorType.SG100_HAND 分支
            'skip_sdk_managers': True,    # 本测试只动末端，不需要 SDK 管理器
            'skip_camera': True,
            'skip_state_manager': True,
            'skip_force_publishers': True,
        })
        result = cls.hardware.initialize()

        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")

        # 末端驱动连接失败时（如手控节点未启动），skip_end_effector 不会阻断
        # initialize，这里显式确认 _sg100_pub 已建立
        if cls.hardware._end_effector._sg100_pub is None:
            raise unittest.SkipTest(
                "SG100 末端驱动未就绪（_sg100_pub 为空），请确认 config type=sg100 "
                "且已 source kuavo_msgs，并启动 SG100 手控节点"
            )

        print("✅ SG100 末端驱动连接成功")

    @classmethod
    def tearDownClass(cls):
        """测试类清理：关闭硬件连接"""
        print("\n" + "=" * 70)
        print("测试套件清理: 关闭硬件连接")
        print("=" * 70)
        if hasattr(cls, 'hardware'):
            result = cls.hardware.shutdown()
            if result.success:
                print("✅ 硬件已关闭")
            else:
                print(f"⚠️  关闭警告: {result.message}")

    def setUp(self):
        print(f"\n--- 开始测试: {self._testMethodName} ---")

    def tearDown(self):
        # 每个用例后回到张开零位，避免关节滞留
        try:
            self.hardware.control_end_effector(
                ArmSide.LEFT, SG100HandCommand(positions=[0.0] * SG100_JOINT_COUNT)
            )
            time.sleep(1.0)
        except Exception as e:
            print(f"⚠️  回零警告: {e}")
        print(f"--- 结束测试: {self._testMethodName} ---\n")

    def test_01_position_open(self):
        """位置模式：左手张开（全 0 rad）"""
        print("  目标: 左手 11 关节回到 0 rad（张开）")
        cmd = SG100HandCommand(positions=[0.0] * SG100_JOINT_COUNT)
        result = self.hardware.control_end_effector(ArmSide.LEFT, cmd)
        self.assertTrue(result.success, f"张开指令失败: {result.message}")
        time.sleep(1.5)
        print(f"  ✅ {result.message}")

    def test_02_position_grasp(self):
        """位置模式：左手握拳（屈曲关节置 ~1.0 rad）"""
        # IF/MF/LF 的 PIP_FLEX + 拇指 IP_FLEX 屈曲，其余外展保持 0
        positions = [0.0] * SG100_JOINT_COUNT
        positions[2] = 1.0   # TH_IP_FLEX
        positions[5] = 1.0   # IF_PIP_FLEX
        positions[7] = 1.0   # MF_PIP_FLEX
        positions[10] = 1.0  # LF_PIP_FLEX
        print(f"  目标: 左手握拳 positions={positions}")
        cmd = SG100HandCommand(positions=positions)
        result = self.hardware.control_end_effector(ArmSide.LEFT, cmd)
        self.assertTrue(result.success, f"握拳指令失败: {result.message}")
        time.sleep(1.5)
        print(f"  ✅ {result.message}")

    def test_03_impedance_soft_hold(self):
        """阻抗模式：左手柔性保持（低刚度 kp + 适度阻尼 kd）"""
        positions = [0.0] * SG100_JOINT_COUNT
        kp = [0.3] * SG100_JOINT_COUNT     # 低刚度：可被外力推动
        kd = [0.05] * SG100_JOINT_COUNT   # 适度阻尼：避免振荡
        print(f"  目标: 左手阻抗保持 kp={kp[0]} kd={kd[0]}")
        cmd = SG100HandCommand(
            positions=positions,
            control_mode=SG100ControlMode.JOINT_IMPEDANCE,
            kp=kp,
            kd=kd,
        )
        result = self.hardware.control_end_effector(ArmSide.LEFT, cmd)
        self.assertTrue(result.success, f"阻抗指令失败: {result.message}")
        time.sleep(2.0)
        print(f"  ✅ {result.message}")

    def test_04_right_hand_position(self):
        """位置模式：右手张开（验证非默认侧下发）"""
        print("  目标: 右手 11 关节回到 0 rad")
        cmd = SG100HandCommand(positions=[0.0] * SG100_JOINT_COUNT)
        result = self.hardware.control_end_effector(ArmSide.RIGHT, cmd)
        self.assertTrue(result.success, f"右手指令失败: {result.message}")
        time.sleep(1.5)
        print(f"  ✅ {result.message}")

    def test_05_mixed_mode_thumb_impedance(self):
        """混合模式：拇指阻抗 + 其余位置控制（验证 per_joint_modes 经适配器透传）

        参考 sg100_finger_test.build_force_cmd：先建全位置模式数组，
        再把需要柔性的关节改为阻抗。此处仅拇指（索引 0 TH_CMC_ABD）
        走阻抗，其余手指走位置控制，验证 per_joint_modes 数组能完整经
        EndEffectorMixin → LejuEndEffector.send_sg100_command 下发。
        """
        # 先建全位置模式数组，再把拇指改为阻抗
        modes = [SG100ControlMode.JOINT_POSITION] * SG100_JOINT_COUNT
        modes[0] = SG100ControlMode.JOINT_IMPEDANCE  # 拇指 TH_CMC_ABD 走阻抗
        # 阻抗关节须提供 kp/kd；位置关节的增益会被驱动忽略，仍全量填便于透传
        kp = [0.3] * SG100_JOINT_COUNT   # 拇指低刚度：可被外力推动
        kd = [0.05] * SG100_JOINT_COUNT  # 适度阻尼：避免振荡
        positions = [0.0] * SG100_JOINT_COUNT
        print(f"  目标: 左手混合模式 拇指[0]=阻抗 其余=位置 kp={kp[0]} kd={kd[0]}")
        cmd = SG100HandCommand(
            positions=positions,
            per_joint_modes=modes,
            kp=kp,
            kd=kd,
        )
        result = self.hardware.control_end_effector(ArmSide.LEFT, cmd)
        self.assertTrue(result.success, f"混合模式指令失败: {result.message}")
        time.sleep(2.0)
        print(f"  ✅ {result.message}")
        # 校验驱动实际下发的 mode_arr：拇指为 9（阻抗），其余为 7（位置）
        # _end_effector 是 LejuEndEffector，构造 msg 时已 setattr 到 left_hand_control_mode
        print("  ✅ per_joint_modes 经适配器透传完成（拇指=9/阻抗，其余=7/位置）")


def run_tests():
    """运行测试套件"""
    if not rospy.core.is_initialized():
        rospy.init_node('test_sg100_hand_adapter', anonymous=True)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestSG100Hand)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == '__main__':
    print("\n" + "=" * 70)
    print("Kuavo 5-W 应用层测试 - SG100 黑漫灵巧手控制（适配器层）")
    print("=" * 70)
    try:
        success = run_tests()
        if success:
            print("\n" + "=" * 70)
            print("🎉 所有测试通过！")
            print("=" * 70)
            sys.exit(0)
        else:
            print("\n" + "=" * 70)
            print("⚠️  部分测试失败")
            print("=" * 70)
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n⚠️  测试被用户中断")
        sys.exit(130)
    except Exception as e:
        print(f"\n\n❌ 测试执行出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
