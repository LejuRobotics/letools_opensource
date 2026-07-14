#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本: 多指令并发序列（下肢+双臂组合控制）

功能描述：
通过适配器发送多条定时指令，实现下肢和双臂的协调同步运动。
使用 /mobile_manipulator_timed_multi_cmd 服务同时控制多个规划器。

注意事项：
1. 使用 send_timed_multi_commands() 接口
2. 支持同步模式（所有指令同时完成）和异步模式
3. 每条指令包含规划器索引、期望时间和命令向量
4. 角度单位：度（内部自动转换为弧度）
5. **无需设置MPC控制模式** - 系统默认以BaseArm模式运行
6. **无需启用快速模式** - 时序指令服务可直接使用

底层对应：
- ROS服务: /mobile_manipulator_timed_multi_cmd
- 服务类型: kuavo_msgs/lbMultiTimedPosCmd
- 参考脚本: kuavo-ros-opensource/src/demo/test_kuavo_wheel_real/timedCmd_example/multiCmd_example/cmd_arm_leg_joint_test.py

运行方式：
    cd ~/LeTools
    python3 apps/test_kuavo_5w_app/04_timed_commands/test_multi_cmd_sequence.py
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
logger = logging.getLogger('test_multi_cmd_sequence')


def print_separator(title):
    """打印分隔线"""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


class TestMultiCmdSequence(unittest.TestCase):
    """测试多指令并发序列"""
    
    @classmethod
    def setUpClass(cls):
        """初始化硬件适配器"""
        logger.info("🔧 初始化硬件适配器...")
        cls.hardware = LejuWheeledArmHardware(config={
            'skip_camera': True,
            'skip_end_effector': True,
            'skip_state_manager': True,
            'skip_force_publishers': True,
            'sdk_managers_whitelist': ['timed'],
        })
        
        # 初始化硬件
        result = cls.hardware.initialize()
        if not result.success:
            raise RuntimeError(f"硬件初始化失败: {result.message}")
        
        # 等待ROS节点初始化
        time.sleep(2)
        
        # 注意：根据轮臂V1.4新功能说明文档，MPC控制模式已废弃
        # 系统默认以BaseArm模式运行，无需手动设置
        
        logger.info("✅ 硬件适配器初始化完成")
    
    @classmethod
    def tearDownClass(cls):
        """清理资源"""
        logger.info("🧹 清理资源...")
        if hasattr(cls, 'hardware'):
            # 注意：MPC模式和快速模式在新版本中已废弃，无需恢复
            # === 脚手架: 后置复位 ===
            from apps.test_kuavo_5w_adapter._scaffold import adapter_teardown
            adapter_teardown(cls.hardware, need_arm=True, restore_mpc=True)

            del cls.hardware
        logger.info("✅ 资源清理完成")
    
    def test_01_sync_arm_leg_motion(self):
        """测试1: 同步模式 - 下肢+双臂协调运动"""
        print_separator("测试1: 同步模式 - 下肢+双臂协调运动")
        
        # 测试用例列表： (名称, 时间, 下肢角度[4个], 左臂角度[7个], 右臂角度[7个])
        # 注意：角度单位为度，内部会转换为弧度
        test_cases = [
            ("展开姿势", 4.0, 
             [14.90, -32.01, 18.03, 0.0],   # 下肢
             [-30, 20, 15, -45, 25, 10, -35],      # 左臂
             [-30,-20,-15, -45,-25,-10, -35]),     # 右臂
            ("弯曲收回", 4.0, 
             [14.90, -32.01, 18.03, 30.0],  # 下肢
             [-20, 30, -25, -20, 40, -15, 25],     # 左臂
             [-20,-30, 25, -20,-40, 15,  25]),     # 右臂
            ("零位姿势", 3.0, 
             [0.0] * 4,   # 下肢零位
             [0.0] * 7,   # 左臂零位
             [0.0] * 7),  # 右臂零位
        ]
        
        success_count = 0

        print("\n开始发布多指令并发序列（同步模式）...")

        for idx, (name, desire_time, leg_deg, left_arm_deg, right_arm_deg) in enumerate(test_cases, 1):
            print(f"\n--- 第{idx}组测试: {name} ---")
            logger.info(f"📤 发送指令: {name}, 期望时间={desire_time}s, 同步模式=True")
            logger.info(f"   下肢角度: {[round(a, 1) for a in leg_deg]}°")
            logger.info(f"   左臂角度: {[round(a, 1) for a in left_arm_deg]}°")
            logger.info(f"   右臂角度: {[round(a, 1) for a in right_arm_deg]}°")

            try:
                # 构建多指令列表（角度单位为度，适配器内部转换）
                commands = [
                    {
                        'planner_index': 3,  # 下肢关节运动
                        'desire_time': desire_time,
                        'cmd_vec': leg_deg
                    },
                    {
                        'planner_index': 8,  # 左臂上肢关节运动
                        'desire_time': desire_time,
                        'cmd_vec': left_arm_deg
                    },
                    {
                        'planner_index': 9,  # 右臂上肢关节运动
                        'desire_time': desire_time,
                        'cmd_vec': right_arm_deg
                    }
                ]
                
                # 发送多指令（同步模式）
                result = self.hardware.send_timed_multi_commands(
                    commands=commands,
                    is_sync=True  # 同步模式：所有指令同时完成
                )
                
                if result.success:
                    logger.info(f"✅ {name} 指令发送成功")
                    success_count += 1
                    
                    # 从 Result 中获取实际执行时间
                    actual_time = result.data.get('actual_time', desire_time) if result.data else desire_time
                    logger.info(f"   实际执行时间: {actual_time:.2f}s")
                    logger.info(f"   同步模式: 所有关节同时到达目标位置")
                    
                    # 等待运动完成（使用实际时间 + 缓冲）
                    wait_time = actual_time + 0.5
                    logger.info(f"⏱️  等待 {wait_time} 秒让机器人运动...")
                    time.sleep(wait_time)
                    logger.info(f"✅ {name} 完成!")
                else:
                    logger.error(f"❌ {name} 指令发送失败: {result.message}")
                    break
                    
            except Exception as e:
                logger.error(f"❌ {name} 异常: {e}")
                import traceback
                traceback.print_exc()
                break
        
        # 验证至少有一条指令成功
        self.assertGreater(success_count, 0, "至少应该有一条指令成功执行")
        logger.info(f"📊 测试结果: {success_count}/{len(test_cases)} 条指令成功")
    
    def test_02_async_independent_motion(self):
        """测试2: 异步模式 - 各关节独立运动"""
        print_separator("测试2: 异步模式 - 各关节独立运动")
        
        print("\n开始发布多指令并发序列（异步模式）...")
        logger.info("📤 发送异步指令序列")
        logger.info("   说明: 各关节按各自时间执行，不等待其他关节")
        
        try:
            # 构建不同时间的指令
            commands = [
                {
                    'planner_index': 3,  # 下肢
                    'desire_time': 3.0,  # 下肢3秒
                    'cmd_vec': [10.0, -20.0, 10.0, 0.0]
                },
                {
                    'planner_index': 8,  # 左臂
                    'desire_time': 5.0,  # 左臂5秒
                    'cmd_vec': [-15, 10, 10, -30, 15, 5, -20]
                },
                {
                    'planner_index': 9,  # 右臂
                    'desire_time': 4.0,  # 右臂4秒
                    'cmd_vec': [-15, -10, -10, -30, -15, -5, -20]
                }
            ]
            
            logger.info(f"   下肢: 3.0s, 左臂: 5.0s, 右臂: 4.0s")
            
            # 发送多指令（异步模式）
            result = self.hardware.send_timed_multi_commands(
                commands=commands,
                is_sync=False  # 异步模式：各指令按各自时间执行
            )
            
            if result.success:
                logger.info(f"✅ 异步指令发送成功")
                
                # 从 Result 中获取实际执行时间（应该是最大的那个）
                actual_time = result.data.get('actual_time', 5.0) if result.data else 5.0
                logger.info(f"   实际执行时间: {actual_time:.2f}s")
                logger.info(f"   异步模式: 各关节独立运动，总时间为最长时间")
                
                # 等待运动完成
                wait_time = actual_time + 0.5
                logger.info(f"⏱️  等待 {wait_time} 秒让机器人运动...")
                time.sleep(wait_time)
                logger.info(f"✅ 异步运动完成!")
                
                # 验证成功
                self.assertTrue(result.success, "异步指令应该成功执行")
            else:
                logger.error(f"❌ 异步指令发送失败: {result.message}")
                self.fail(f"异步指令失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 异步指令异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"异步指令异常: {e}")
    
    def test_03_chassis_torso_coordination(self):
        """测试3: 底盘+躯干协调运动"""
        print_separator("测试3: 底盘+躯干协调运动")
        
        print("\n开始发布底盘+躯干协调指令...")
        logger.info("📤 发送底盘+躯干协调指令")
        
        try:
            # 底盘前进 + 躯干前倾
            commands = [
                {
                    'planner_index': 1,  # 底盘局部系位置运动
                    'desire_time': 4.0,
                    'cmd_vec': [0.3, 0.0, 0.0]  # x=0.3m, y=0, yaw=0
                },
                {
                    'planner_index': 2,  # 躯干笛卡尔局部系运动
                    'desire_time': 4.0,
                    'cmd_vec': [0.1, 1.4, 0.0, 11.46]  # x=0.1m, z=1.4m, yaw=0, pitch=11.46度
                }
            ]

            logger.info(f"   底盘: 前进0.3m, 躯干: 前倾11.46度")
            
            # 发送多指令（同步模式）
            result = self.hardware.send_timed_multi_commands(
                commands=commands,
                is_sync=True
            )
            
            if result.success:
                logger.info(f"✅ 底盘+躯干协调指令发送成功")
                
                actual_time = result.data.get('actual_time', 4.0) if result.data else 4.0
                logger.info(f"   实际执行时间: {actual_time:.2f}s")
                
                # 等待运动完成
                wait_time = actual_time + 0.5
                logger.info(f"⏱️  等待 {wait_time} 秒让机器人运动...")
                time.sleep(wait_time)
                logger.info(f"✅ 协调运动完成!")
                
                # 验证成功
                self.assertTrue(result.success, "协调指令应该成功执行")
            else:
                logger.error(f"❌ 协调指令发送失败: {result.message}")
                self.fail(f"协调指令失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 协调指令异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"协调指令异常: {e}")


def main():
    """主函数"""
    print_separator("多指令并发序列测试")
    
    print("\n📋 测试说明:")
    print("  - 测试多指令并发控制功能")
    print("  - 使用 /mobile_manipulator_timed_multi_cmd 服务")
    print("  - 支持同步模式（同时完成）和异步模式（独立执行）")
    print("  - 包含三个测试用例:")
    print("    1. 同步模式 - 下肢+双臂协调运动")
    print("    2. 异步模式 - 各关节独立运动")
    print("    3. 底盘+躯干协调运动")
    
    print("\n✨ 多指令并发优势:")
    print("  - 精确的时间同步控制")
    print("  - 减少通信延迟和抖动")
    print("  - 实现复杂的协调运动")
    print("  - MPC统一规划，保证运动学连续性")
    
    print("\n💡 提示:")
    print("  - planner_index: 0-9 对应不同规划器")
    print("  - is_sync=True: 所有指令同时完成")
    print("  - is_sync=False: 各指令按各自时间执行")
    print("  - 角度单位: 度（内部自动转换为弧度）")
    
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
