#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本: Ruckig 规划器参数配置

功能描述：
通过适配器设置不同规划器的速度/加速度/急动度限制参数，优化运动性能。
使用 /mobile_manipulator_set_ruckig_planner_params 服务调整规划器行为。

注意事项：
1. 参数会影响所有使用该规划器的指令（包括时序指令）
2. 速度/加速度/急动度的维度必须与规划器自由度匹配
3. 建议先使用保守参数，逐步调整到最优值
4. **无需设置MPC控制模式** - 系统默认以BaseArm模式运行

底层对应：
- ROS服务: /mobile_manipulator_set_ruckig_planner_params
- 服务类型: kuavo_msgs/setRuckigPlannerParams
- 参考脚本: kuavo-ros-opensource/src/demo/test_kuavo_wheel_real/ruckig_setting_test.py

运行方式：
    cd ~/LeTools
    python3 apps/test_kuavo_5w_app/04_timed_commands/test_ruckig_params.py
"""

import sys
import os
import time
import unittest
import logging

# 添加项目根目录到 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.insert(0, project_root)

from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('test_ruckig_params')


def print_separator(title):
    """打印分隔线"""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


class TestRuckigParams(unittest.TestCase):
    """测试 Ruckig 规划器参数配置"""
    
    @classmethod
    def setUpClass(cls):
        """初始化硬件适配器"""
        logger.info("🔧 初始化硬件适配器...")
        cls.hardware = LejuWheeledArmHardware()

        # 初始化硬件
        result = cls.hardware.initialize()
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")

        # 环境检测: 检查必要的 ROS 服务是否可用
        from apps.test_kuavo_5w_adapter._scaffold import check_services_available
        ok, missing = check_services_available([
            '/mobile_manipulator_set_ruckig_planner_params',
        ])
        if not ok:
            raise unittest.SkipTest(f"ROS 服务不可用: {missing}，请启动控制器进程")

        # 等待ROS节点初始化
        time.sleep(2)

        # 脚手架: 前置设置（躯干复位）
        from apps.test_kuavo_5w_adapter._scaffold import adapter_setup
        adapter_setup(cls.hardware, need_torso_reset=True)

        logger.info("✅ 硬件适配器初始化完成")
    
    @classmethod
    def tearDownClass(cls):
        """清理资源"""
        logger.info("🧹 清理资源...")
        if hasattr(cls, 'hardware'):
            # === 脚手架: 后置复位 ===
            from apps.test_kuavo_5w_adapter._scaffold import adapter_teardown
            adapter_teardown(cls.hardware, need_arm=False, restore_mpc=True)

            del cls.hardware
        logger.info("✅ 资源清理完成")
    
    def test_01_base_pose_planner_params(self):
        """测试1: 底盘位置规划器参数配置"""
        print_separator("测试1: 底盘位置规划器参数配置 (planner_index=0)")
        
        print("\n配置底盘位置运动的 Ruckig 参数...")
        logger.info("📤 设置 Base Pose Planner 参数")
        
        try:
            # 配置底盘位置规划器 (3维: x, y, yaw)
            result = self.hardware.set_ruckig_planner_params(
                planner_index=0,      # 底盘世界系位置运动
                is_sync=True,         # 同步模式
                velocity_max=[0.2, 0.2, 0.2],        # 最大速度 [m/s, m/s, rad/s]
                acceleration_max=[2.0, 2.0, 1.5],    # 最大加速度 [m/s², m/s², rad/s²]
                jerk_max=[20.0, 15.0, 12.0]          # 最大急动度 [m/s³, m/s³, rad/s³]
            )
            
            if result.success:
                logger.info(f"✅ 底盘位置规划器参数设置成功")
                message = result.data.get('message', '') if result.data else ''
                logger.info(f"   消息: {message}")
                
                # 验证成功
                self.assertTrue(result.success, "底盘位置规划器参数应该设置成功")
            else:
                logger.error(f"❌ 底盘位置规划器参数设置失败: {result.message}")
                self.fail(f"参数设置失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 底盘位置规划器参数配置异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"参数配置异常: {e}")
    
    def test_02_left_arm_joint_planner_params(self):
        """测试2: 左臂关节规划器参数配置"""
        print_separator("测试2: 左臂关节规划器参数配置 (planner_index=8)")
        
        print("\n配置左臂关节运动的 Ruckig 参数...")
        logger.info("📤 设置 Left Arm Joint Planner 参数")
        
        try:
            # 等待一下，避免频繁调用
            time.sleep(2)
            
            # 配置左臂关节规划器 (7维关节)
            result = self.hardware.set_ruckig_planner_params(
                planner_index=8,      # 左臂上肢关节运动
                is_sync=True,         # 同步模式
                velocity_max=[1.0] * 7,        # 7个关节的最大速度 [rad/s]
                acceleration_max=[5.0] * 7,    # 最大加速度 [rad/s²]
                jerk_max=[50.0] * 7            # 最大急动度 [rad/s³]
            )
            
            if result.success:
                logger.info(f"✅ 左臂关节规划器参数设置成功")
                message = result.data.get('message', '') if result.data else ''
                logger.info(f"   消息: {message}")
                
                # 验证成功
                self.assertTrue(result.success, "左臂关节规划器参数应该设置成功")
            else:
                logger.error(f"❌ 左臂关节规划器参数设置失败: {result.message}")
                self.fail(f"参数设置失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 左臂关节规划器参数配置异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"参数配置异常: {e}")
    
    def test_03_right_arm_joint_planner_params(self):
        """测试3: 右臂关节规划器参数配置"""
        print_separator("测试3: 右臂关节规划器参数配置 (planner_index=9)")
        
        print("\n配置右臂关节运动的 Ruckig 参数...")
        logger.info("📤 设置 Right Arm Joint Planner 参数")
        
        try:
            # 等待一下，避免频繁调用
            time.sleep(2)
            # 配置右臂关节规划器 (7维关节)
            result = self.hardware.set_ruckig_planner_params(
                planner_index=9,      # 右臂上肢关节运动
                is_sync=True,         # 同步模式
                velocity_max=[1.0] * 7,        # 7个关节的最大速度 [rad/s]
                acceleration_max=[5.0] * 7,    # 最大加速度 [rad/s²]
                jerk_max=[50.0] * 7            # 最大急动度 [rad/s³]
            )
            
            if result.success:
                logger.info(f"✅ 右臂关节规划器参数设置成功")
                message = result.data.get('message', '') if result.data else ''
                logger.info(f"   消息: {message}")
                
                # 验证成功
                self.assertTrue(result.success, "右臂关节规划器参数应该设置成功")
            else:
                logger.error(f"❌ 右臂关节规划器参数设置失败: {result.message}")
                self.fail(f"参数设置失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 右臂关节规划器参数配置异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"参数配置异常: {e}")
    
    def test_04_leg_joint_planner_params(self):
        """测试4: 下肢关节规划器参数配置"""
        print_separator("测试4: 下肢关节规划器参数配置 (planner_index=3)")
        
        print("\n配置下肢关节运动的 Ruckig 参数...")
        logger.info("📤 设置 Leg Joint Planner 参数")
        
        try:
            # 等待一下，避免频繁调用
            time.sleep(2)
            # 配置下肢关节规划器 (4维关节)
            result = self.hardware.set_ruckig_planner_params(
                planner_index=3,      # 下肢关节运动
                is_sync=True,         # 同步模式
                velocity_max=[2.0] * 4,        # 4个关节的最大速度 [rad/s]
                acceleration_max=[10.0] * 4,   # 最大加速度 [rad/s²]
                jerk_max=[100.0] * 4           # 最大急动度 [rad/s³]
            )
            
            if result.success:
                logger.info(f"✅ 下肢关节规划器参数设置成功")
                message = result.data.get('message', '') if result.data else ''
                logger.info(f"   消息: {message}")
                
                # 验证成功
                self.assertTrue(result.success, "下肢关节规划器参数应该设置成功")
            else:
                logger.error(f"❌ 下肢关节规划器参数设置失败: {result.message}")
                self.fail(f"参数设置失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 下肢关节规划器参数配置异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"参数配置异常: {e}")
    
    def test_05_torso_cartesian_planner_params(self):
        """测试5: 躯干笛卡尔规划器参数配置"""
        print_separator("测试5: 躯干笛卡尔规划器参数配置 (planner_index=2)")
        
        print("\n配置躯干笛卡尔运动的 Ruckig 参数...")
        logger.info("📤 设置 Torso Cartesian Planner 参数")
        
        try:
            # 等待一下，避免频繁调用
            time.sleep(2)
            # 配置躯干笛卡尔规划器 (4维: x, z, yaw, pitch)
            result = self.hardware.set_ruckig_planner_params(
                planner_index=2,      # 躯干笛卡尔局部系运动
                is_sync=True,         # 同步模式
                velocity_max=[0.1, 0.1, 0.2, 0.2],     # [m/s, m/s, rad/s, rad/s]
                acceleration_max=[1.0, 1.0, 2.0, 2.0], # [m/s², m/s², rad/s², rad/s²]
                jerk_max=[10.0, 10.0, 20.0, 20.0]      # [m/s³, m/s³, rad/s³, rad/s³]
            )
            
            if result.success:
                logger.info(f"✅ 躯干笛卡尔规划器参数设置成功")
                message = result.data.get('message', '') if result.data else ''
                logger.info(f"   消息: {message}")
                
                # 验证成功
                self.assertTrue(result.success, "躯干笛卡尔规划器参数应该设置成功")
            else:
                logger.error(f"❌ 躯干笛卡尔规划器参数设置失败: {result.message}")
                self.fail(f"参数设置失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 躯干笛卡尔规划器参数配置异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"参数配置异常: {e}")
    
    def test_06_custom_min_params(self):
        """测试6: 自定义最小速度/加速度参数"""
        print_separator("测试6: 自定义最小速度/加速度参数")
        
        print("\n配置带自定义最小值的 Ruckig 参数...")
        logger.info("📤 设置带自定义最小值的规划器参数")
        
        try:
            # 等待一下，避免频繁调用
            time.sleep(2)
            # 配置底盘位置规划器，指定最小速度和加速度
            result = self.hardware.set_ruckig_planner_params(
                planner_index=0,      # 底盘世界系位置运动
                is_sync=False,        # 异步模式
                velocity_max=[0.3, 0.3, 0.3],        # 最大速度
                acceleration_max=[3.0, 3.0, 2.0],    # 最大加速度
                jerk_max=[30.0, 25.0, 20.0],         # 最大急动度
                velocity_min=[-0.1, -0.1, -0.1],     # 最小速度（非对称）
                acceleration_min=[-1.0, -1.0, -0.5]  # 最小加速度（非对称）
            )
            
            if result.success:
                logger.info(f"✅ 自定义最小值参数设置成功")
                message = result.data.get('message', '') if result.data else ''
                logger.info(f"   消息: {message}")
                logger.info(f"   说明: 使用了非对称的最小速度/加速度限制")
                
                # 验证成功
                self.assertTrue(result.success, "自定义最小值参数应该设置成功")
            else:
                logger.error(f"❌ 自定义最小值参数设置失败: {result.message}")
                self.fail(f"参数设置失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 自定义最小值参数配置异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"参数配置异常: {e}")
    
    def test_07_verify_with_motion(self):
        """测试7: 验证参数生效 - 发送手臂时序指令"""
        print_separator("测试7: 验证参数生效 - 发送手臂时序指令")
        
        print("\n发送手臂时序指令，验证新参数是否生效...")
        logger.info("📤 发送测试指令验证 Ruckig 参数")
        
        try:
            # 等待一下，确保之前的参数设置完成
            time.sleep(3)

            # 发送左臂时序指令
            left_arm_angles_deg = [-20, 15, 10, -35, 20, 8, -25]

            logger.info(f"   左臂目标角度: {[round(a, 1) for a in left_arm_angles_deg]}°")

            result = self.hardware.send_timed_left_arm_joint(
                joint_angles=left_arm_angles_deg,
                desire_time=3.0
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 3.0) if result.data else 3.0
                logger.info(f"✅ 左臂指令执行成功")
                logger.info(f"   实际执行时间: {actual_time:.2f}s")
                logger.info(f"   说明: 如果参数生效，运动应该更平滑或更快")
                
                # 等待运动完成
                wait_time = actual_time + 0.5
                logger.info(f"⏱️  等待 {wait_time} 秒让机器人运动...")
                time.sleep(wait_time)
                
                # 验证成功
                self.assertTrue(result.success, "手臂时序指令应该执行成功")
            else:
                logger.error(f"❌ 左臂指令执行失败: {result.message}")
                self.fail(f"手臂指令失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 验证指令异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"验证指令异常: {e}")


def main():
    """主函数"""
    print_separator("Ruckig 规划器参数配置测试")
    
    print("\n📋 测试说明:")
    print("  - 测试不同规划器的 Ruckig 参数配置")
    print("  - 使用 /mobile_manipulator_set_ruckig_planner_params 服务")
    print("  - 包含7个测试用例:")
    print("    1. 底盘位置规划器 (planner_index=0)")
    print("    2. 左臂关节规划器 (planner_index=8)")
    print("    3. 右臂关节规划器 (planner_index=9)")
    print("    4. 下肢关节规划器 (planner_index=3)")
    print("    5. 躯干笛卡尔规划器 (planner_index=2)")
    print("    6. 自定义最小速度/加速度参数")
    print("    7. 验证参数生效 - 发送手臂时序指令")
    
    print("\n✨ Ruckig 规划器优势:")
    print("  - 时间最优轨迹规划")
    print("  - 二阶连续和三阶可导")
    print("  - 保证运动平滑性")
    print("  - 支持速度和加速度限制")
    
    print("\n💡 提示:")
    print("  - planner_index: 0-9 对应不同规划器")
    print("  - 参数维度必须与规划器自由度匹配")
    print("  - 建议从保守参数开始，逐步优化")
    print("  - 修改后的参数对所有后续指令生效")
    
    # 运行单元测试
    unittest.main(verbosity=2)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  测试被用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"\n\n❌ 测试出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
