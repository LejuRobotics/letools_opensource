#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本: 混合指令序列（底盘+躯干+腿部组合）

功能描述：
通过适配器发送混合的时序控制指令序列。
组合底盘位置、躯干位姿和腿部关节控制，实现复杂的协调运动。

注意事项：
1. 每条指令包含期望执行时间
2. 指令按顺序执行，前一条完成后才执行下一条
3. 需要合理设置等待时间，确保机器人稳定
4. 建议先在仿真环境中测试

运行方式：
    cd ~/LeTools
    python3 apps/test_kuavo_5w_app/04_timed_commands/test_mixed_commands.py
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
from core.domain.enums import FrameType

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('test_mixed_commands')


def print_separator(title):
    """打印分隔线"""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


class TestMixedCommands(unittest.TestCase):
    """测试混合指令序列"""
    
    @classmethod
    def setUpClass(cls):
        """初始化硬件适配器"""
        logger.info("🔧 初始化硬件适配器...")
        cls.hardware = LejuWheeledArmHardware(config={
            'skip_camera': True,# 混合指令序列不需要相机
            'skip_end_effector': True,# 混合指令序列不需要末端执行器
            'skip_state_manager': True,# 混合指令序列不需要状态管理器
            'skip_force_publishers': True,# 混合指令序列不需要力控发布器
            'sdk_managers_whitelist': ['timed'],# 混合指令序列只需要 timed 管理器
        })

        # 初始化硬件
        result = cls.hardware.initialize()
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")

        # 环境检测: 检查必要的 ROS 服务是否可用
        from apps.test_kuavo_5w_adapter._scaffold import check_services_available
        ok, missing = check_services_available([
            '/mobile_manipulator_mpc_control',
            '/wheel_arm_change_arm_ctrl_mode',
        ])
        if not ok:
            raise unittest.SkipTest(f"ROS 服务不可用: {missing}，请启动控制器进程")

        # 等待ROS节点初始化
        time.sleep(2)

        # 脚手架: 前置设置（躯干复位 + MPC 模式 + 手臂控制）
        from apps.test_kuavo_5w_adapter._scaffold import adapter_setup
        from core.domain.enums import MPCControlMode
        adapter_setup(cls.hardware, need_arm=True, mpc_mode=MPCControlMode.BASE_ARM)

        logger.info("✅ 硬件适配器初始化完成")
    
    @classmethod
    def tearDownClass(cls):
        """清理资源"""
        logger.info("🧹 清理资源...")
        if hasattr(cls, 'hardware'):
            # === 脚手架: 后置复位 ===
            from apps.test_kuavo_5w_adapter._scaffold import adapter_teardown
            adapter_teardown(cls.hardware, need_arm=True, restore_mpc=True)

            del cls.hardware
        logger.info("✅ 资源清理完成")
    
    def test_mixed_sequence_basic(self):
        """测试基础混合指令序列：底盘+躯干+腿部"""
        print_separator("测试: 基础混合指令序列")
        
        success_count = 0
        total_steps = 6
        
        try:
            # 步骤1: 底盘前进
            print("\n--- 步骤1: 底盘前进 0.2m ---")
            logger.info("📤 发送底盘前进指令")
            result = self.hardware.send_timed_base_pose(
                x=0.2, y=0.0, yaw=0.0,
                desire_time=3.0,
                frame=FrameType.LOCAL
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 3.0) if result.data else 3.0
                logger.info(f"✅ 底盘前进成功，实际时间: {actual_time:.2f}s")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 底盘前进失败: {result.message}")
                return
            
            # 步骤2: 躯干向前倾斜
            print("\n--- 步骤2: 躯干向前倾斜 ---")
            logger.info("📤 发送躯干倾斜指令")
            result = self.hardware.send_timed_torso_pose(
                x=0.1, z=1.4, yaw=0.0, pitch=8.6,
                desire_time=2.0
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 2.0) if result.data else 2.0
                logger.info(f"✅ 躯干倾斜成功，实际时间: {actual_time:.2f}s")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 躯干倾斜失败: {result.message}")
                return
            
            # 步骤3: 腿部微蹲
            print("\n--- 步骤3: 腿部微蹲 ---")
            logger.info("📤 发送腿部微蹲指令")
            joint_angles_deg = [11.46, -22.92, 11.46, 0.0]  # 约 [0.2, -0.4, 0.2, 0.0] 弧度
            result = self.hardware.send_timed_leg_joint(
                joint_angles=joint_angles_deg,
                desire_time=2.5
            )

            if result.success:
                actual_time = result.data.get('actual_time', 2.5) if result.data else 2.5
                logger.info(f"✅ 腿部微蹲成功，实际时间: {actual_time:.2f}s, 角度: {joint_angles_deg}°")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 腿部微蹲失败: {result.message}")
                return
            
            # 步骤4: 底盘后退
            print("\n--- 步骤4: 底盘后退 0.2m ---")
            logger.info("📤 发送底盘后退指令")
            result = self.hardware.send_timed_base_pose(
                x=-0.2, y=0.0, yaw=0.0,
                desire_time=3.0,
                frame=FrameType.LOCAL
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 3.0) if result.data else 3.0
                logger.info(f"✅ 底盘后退成功，实际时间: {actual_time:.2f}s")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 底盘后退失败: {result.message}")
                return
            
            # 步骤5: 躯干恢复
            print("\n--- 步骤5: 躯干恢复直立 ---")
            logger.info("📤 发送躯干恢复指令")
            result = self.hardware.send_timed_torso_pose(
                x=0.0, z=1.4, yaw=0.0, pitch=0.0,
                desire_time=2.0
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 2.0) if result.data else 2.0
                logger.info(f"✅ 躯干恢复成功，实际时间: {actual_time:.2f}s")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 躯干恢复失败: {result.message}")
                return
            
            # 步骤6: 腿部恢复站立
            print("\n--- 步骤6: 腿部恢复站立 ---")
            logger.info("📤 发送腿部恢复指令")
            joint_angles_deg = [0.0, 0.0, 0.0, 0.0]
            result = self.hardware.send_timed_leg_joint(
                joint_angles=joint_angles_deg,
                desire_time=2.5
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 2.5) if result.data else 2.5
                logger.info(f"✅ 腿部恢复成功，实际时间: {actual_time:.2f}s")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 腿部恢复失败: {result.message}")
                return
            
        except Exception as e:
            logger.error(f"❌ 混合指令序列异常: {e}")
            import traceback
            traceback.print_exc()
        
        # 验证所有步骤都成功
        self.assertEqual(success_count, total_steps, 
                        f"应该完成所有{total_steps}个步骤，但只完成了{success_count}个")
        logger.info(f"📊 测试结果: {success_count}/{total_steps} 个步骤成功")
        logger.info("🎉 混合指令序列测试完成！")
    
    def test_mixed_sequence_coordinate(self):
        """测试协调运动：底盘旋转+躯干调整"""
        print_separator("测试: 协调运动序列")
        
        success_count = 0
        total_steps = 4
        
        try:
            # 步骤1: 底盘左转
            print("\n--- 步骤1: 底盘左转 17度 ---")
            logger.info("📤 发送底盘左转指令")
            result = self.hardware.send_timed_base_pose(
                x=0.0, y=0.0, yaw=17,
                desire_time=3.0,
                frame=FrameType.LOCAL
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 3.0) if result.data else 3.0
                logger.info(f"✅ 底盘左转成功，实际时间: {actual_time:.2f}s")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 底盘左转失败: {result.message}")
                return
            
            # 步骤2: 躯干向右偏转（补偿）
            print("\n--- 步骤2: 躯干向右偏转 ---")
            logger.info("📤 发送躯干偏转指令")
            result = self.hardware.send_timed_torso_pose(
                x=0.0, z=1.4, yaw=-8.6, pitch=0.0,
                desire_time=2.0
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 2.0) if result.data else 2.0
                logger.info(f"✅ 躯干偏转成功，实际时间: {actual_time:.2f}s")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 躯干偏转失败: {result.message}")
                return
            
            # 步骤3: 躯干恢复
            print("\n--- 步骤3: 躯干恢复 ---")
            logger.info("📤 发送躯干恢复指令")
            result = self.hardware.send_timed_torso_pose(
                x=0.0, z=1.4, yaw=0.0, pitch=0.0,
                desire_time=2.0
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 2.0) if result.data else 2.0
                logger.info(f"✅ 躯干恢复成功，实际时间: {actual_time:.2f}s")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 躯干恢复失败: {result.message}")
                return
            
            # 步骤4: 底盘右转回正
            print("\n--- 步骤4: 底盘右转回正 ---")
            logger.info("📤 发送底盘右转指令")
            result = self.hardware.send_timed_base_pose(
                x=0.0, y=0.0, yaw=-17,
                desire_time=3.0,
                frame=FrameType.LOCAL
            )
            
            if result.success:
                actual_time = result.data.get('actual_time', 3.0) if result.data else 3.0
                logger.info(f"✅ 底盘右转成功，实际时间: {actual_time:.2f}s")
                time.sleep(actual_time + 0.5)
                success_count += 1
            else:
                logger.error(f"❌ 底盘右转失败: {result.message}")
                return
            
        except Exception as e:
            logger.error(f"❌ 协调运动序列异常: {e}")
            import traceback
            traceback.print_exc()
        
        # 验证所有步骤都成功
        self.assertEqual(success_count, total_steps, 
                        f"应该完成所有{total_steps}个步骤，但只完成了{success_count}个")
        logger.info(f"📊 测试结果: {success_count}/{total_steps} 个步骤成功")
        logger.info("🎉 协调运动序列测试完成！")


def main():
    """主函数"""
    print_separator("混合指令序列测试")
    
    print("\n📋 测试说明:")
    print("  - 测试底盘、躯干、腿部的协调运动")
    print("  - 每条指令包含期望执行时间")
    print("  - 指令按顺序执行，前一条完成后才执行下一条")
    print("  - 包含两个测试用例:")
    print("    1. 基础混合序列：前进→倾斜→微蹲→后退→恢复")
    print("    2. 协调运动序列：左转→偏转→恢复→右转")
    
    print("\n✨ 使用真正的时序指令服务:")
    print("  - 集成 /mobile_manipulator_timed_single_cmd 服务")
    print("  - MPC 精确计时，提高时序精度")
    print("  - 返回实际执行时间 actualTime")
    
    print("\n⚠️  注意: 混合指令需要合理设置时间参数")
    print("  建议先在仿真环境中测试，再在真机上运行")
    
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
