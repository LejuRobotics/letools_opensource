#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本: 离线轨迹测试（预定义复杂轨迹）

功能描述：
通过适配器设置并执行预定义的离线轨迹，实现复杂的协调运动。
使用 /mobile_manipulator_timed_offline_traj 服务缓存轨迹，然后启动执行。

注意事项：
1. 第一帧时间必须为0
2. 时间必须严格递增
3. 命令向量维度必须与规划器匹配（左/右臂6维，躯干4维）
4. 设置轨迹后需调用 enable_offline_trajectory(True) 启动执行
5. **无需设置MPC控制模式** - 系统默认以BaseArm模式运行

底层对应：
- ROS服务: /mobile_manipulator_timed_offline_traj (设置轨迹)
- ROS服务: /mobile_manipulator_timed_offline_traj_enable (启用执行)
- 服务类型: kuavo_msgs/lbMultiTimedOfflineTraj, std_srvs/SetBool
- 参考脚本: kuavo-ros-opensource/src/demo/test_kuavo_wheel_real/cmd_offline_traj_test.py

运行方式：
    cd ~/LeTools
    python3 apps/test_kuavo_5w_app/04_timed_commands/test_offline_trajectory.py
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
logger = logging.getLogger('test_offline_trajectory')


def print_separator(title):
    """打印分隔线"""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


class TestOfflineTrajectory(unittest.TestCase):
    """测试离线轨迹功能"""
    
    @classmethod
    def setUpClass(cls):
        """初始化硬件适配器"""
        logger.info("🔧 初始化硬件适配器...")
        cls.hardware = LejuWheeledArmHardware(config={
            'skip_camera': True,# 离线轨迹测试不需要相机
            'skip_end_effector': True,# 离线轨迹测试不需要末端执行器
            'skip_state_manager': True,# 离线轨迹测试不需要状态管理器
            'skip_force_publishers': True,# 离线轨迹测试不需要力控发布器
            'sdk_managers_whitelist': ['timed'],# 离线轨迹测试只需要 timed 管理器
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
            '/mobile_manipulator_timed_offline_traj',
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
            # 禁用离线轨迹
            try:
                cls.hardware.enable_offline_trajectory(False)
            except:
                pass

            # === 脚手架: 后置复位 ===
            from apps.test_kuavo_5w_adapter._scaffold import adapter_teardown
            adapter_teardown(cls.hardware, need_arm=True, restore_mpc=True)

            del cls.hardware
        logger.info("✅ 资源清理完成")
    
    def test_01_left_arm_cartesian_trajectory(self):
        """测试1: 左臂笛卡尔轨迹（世界系）"""
        print_separator("测试1: 左臂笛卡尔轨迹（世界系）")
        
        print("\n设置并执行左臂笛卡尔轨迹...")
        logger.info("📤 设置左臂离线轨迹")
        
        try:
            # 左臂轨迹 - 笛卡尔空间运动（6维: x, y, z, yaw, pitch, roll）
            trajectory = [{
                'planner_index': 0,      # 左臂笛卡尔世界系运动
                'frame': 0,              # 世界系
                'timed_traj': [
                    {'desire_time': 0.0, 'cmd_vec': [0.3, 0.2, 0.3, 0.0, 0.0, 0.0]},
                    {'desire_time': 1.0, 'cmd_vec': [0.4, 0.2, 0.35, 0.1, 0.0, 0.0]},
                    {'desire_time': 2.0, 'cmd_vec': [0.5, 0.2, 0.4, 0.2, 0.0, 0.0]},
                    {'desire_time': 3.0, 'cmd_vec': [0.6, 0.2, 0.45, 0.3, 0.0, 0.0]}
                ]
            }]
            
            logger.info(f"   轨迹点数: {len(trajectory[0]['timed_traj'])}")
            logger.info(f"   总时长: {trajectory[0]['timed_traj'][-1]['desire_time']:.2f}s")
            logger.info(f"   起始点: {trajectory[0]['timed_traj'][0]['cmd_vec']}")
            logger.info(f"   终点点: {trajectory[0]['timed_traj'][-1]['cmd_vec']}")
            
            # 设置轨迹
            result = self.hardware.set_offline_trajectory(trajectory)
            
            if result.success:
                logger.info(f"✅ 左臂轨迹设置成功")
                message = result.data.get('message', '') if result.data else ''
                logger.info(f"   消息: {message}")
                
                # 验证成功
                self.assertTrue(result.success, "左臂轨迹应该设置成功")
                
                # 启动执行
                logger.info("🚀 启动轨迹执行...")
                enable_result = self.hardware.enable_offline_trajectory(True)
                
                if enable_result.success:
                    logger.info(f"✅ 轨迹执行已启动")
                    
                    # 等待运动完成
                    total_time = trajectory[0]['timed_traj'][-1]['desire_time']
                    wait_time = total_time + 1.0
                    logger.info(f"⏱️  等待 {wait_time} 秒让机器人运动...")
                    time.sleep(wait_time)
                    logger.info(f"✅ 左臂轨迹执行完成!")
                else:
                    logger.error(f"❌ 轨迹启动失败: {enable_result.message}")
                    self.fail(f"轨迹启动失败: {enable_result.message}")
            else:
                logger.error(f"❌ 左臂轨迹设置失败: {result.message}")
                self.fail(f"轨迹设置失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 左臂轨迹异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"左臂轨迹异常: {e}")
    
    def test_02_right_arm_cartesian_trajectory(self):
        """测试2: 右臂笛卡尔轨迹（世界系）"""
        print_separator("测试2: 右臂笛卡尔轨迹（世界系）")
        
        print("\n设置并执行右臂笛卡尔轨迹...")
        logger.info("📤 设置右臂离线轨迹")
        
        try:
            # 右臂轨迹
            trajectory = [{
                'planner_index': 1,      # 右臂笛卡尔世界系运动
                'frame': 0,              # 世界系
                'timed_traj': [
                    {'desire_time': 0.0, 'cmd_vec': [0.3, -0.2, 0.3, 0.0, 0.0, 0.0]},
                    {'desire_time': 1.0, 'cmd_vec': [0.4, -0.25, 0.35, 0.0, 0.1, 0.0]},
                    {'desire_time': 2.0, 'cmd_vec': [0.5, -0.3, 0.4, 0.0, 0.2, 0.0]},
                    {'desire_time': 3.0, 'cmd_vec': [0.6, -0.35, 0.45, 0.0, 0.3, 0.0]}
                ]
            }]
            
            logger.info(f"   轨迹点数: {len(trajectory[0]['timed_traj'])}")
            logger.info(f"   总时长: {trajectory[0]['timed_traj'][-1]['desire_time']:.2f}s")
            
            # 设置轨迹
            result = self.hardware.set_offline_trajectory(trajectory)
            
            if result.success:
                logger.info(f"✅ 右臂轨迹设置成功")
                
                # 启动执行
                logger.info("🚀 启动轨迹执行...")
                enable_result = self.hardware.enable_offline_trajectory(True)
                
                if enable_result.success:
                    logger.info(f"✅ 轨迹执行已启动")
                    
                    # 等待运动完成
                    total_time = trajectory[0]['timed_traj'][-1]['desire_time']
                    wait_time = total_time + 1.0
                    logger.info(f"⏱️  等待 {wait_time} 秒让机器人运动...")
                    time.sleep(wait_time)
                    logger.info(f"✅ 右臂轨迹执行完成!")
                    
                    # 验证成功
                    self.assertTrue(result.success, "右臂轨迹应该设置成功")
                else:
                    logger.error(f"❌ 轨迹启动失败: {enable_result.message}")
                    self.fail(f"轨迹启动失败: {enable_result.message}")
            else:
                logger.error(f"❌ 右臂轨迹设置失败: {result.message}")
                self.fail(f"轨迹设置失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 右臂轨迹异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"右臂轨迹异常: {e}")
    
    def test_03_dual_arm_torso_coordination(self):
        """测试3: 双臂+躯干协同轨迹"""
        print_separator("测试3: 双臂+躯干协同轨迹")
        
        print("\n设置并执行双臂+躯干协同轨迹...")
        logger.info("📤 设置双臂+躯干离线轨迹")
        
        try:
            # 双臂+躯干轨迹
            trajectories = [
                {
                    'planner_index': 0,      # 左臂
                    'frame': 1,              # 世界系
                    'timed_traj': [
                        {'desire_time': 0.0, 'cmd_vec': [0.3, 0.2, 0.1, 0.0, 0.0, 0.0]},
                        {'desire_time': 1.0, 'cmd_vec': [0.3, 0.2, 0.1, 0.0, 0.0, 0.0]},
                        {'desire_time': 2.0, 'cmd_vec': [0.3, 0.2, 0.1, 0.0, 0.0, 0.0]},
                        {'desire_time': 3.0, 'cmd_vec': [0.3, 0.2, 0.1, 0.0, 0.0, 0.0]},
                        {'desire_time': 4.0, 'cmd_vec': [0.3, 0.2, 0.1, 0.0, 0.0, 0.0]},
                    ]
                },
                {
                    'planner_index': 1,      # 右臂
                    'frame': 1,              # 局部系
                    'timed_traj': [
                        {'desire_time': 0.0, 'cmd_vec': [0.3, -0.2, 0.1, 0.0, 0.0, 0.0]},
                        {'desire_time': 1.0, 'cmd_vec': [0.3, -0.2, 0.1, 0.0, 0.0, 0.0]},
                        {'desire_time': 2.0, 'cmd_vec': [0.3, -0.2, 0.1, 0.0, 0.0, 0.0]},
                        {'desire_time': 3.0, 'cmd_vec': [0.3, -0.2, 0.1, 0.0, 0.0, 0.0]},
                        {'desire_time': 4.0, 'cmd_vec': [0.3, -0.2, 0.1, 0.0, 0.0, 0.0]},
                    ]
                },
                {
                    'planner_index': 2,      # 躯干
                    'frame': 0,              # 世界系
                    'timed_traj': [
                        {'desire_time': 0.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
                        {'desire_time': 1.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
                        {'desire_time': 2.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
                        {'desire_time': 3.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
                        {'desire_time': 4.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
                    ]
                }
            ]
            
            logger.info(f"   左臂轨迹点数: {len(trajectories[0]['timed_traj'])}")
            logger.info(f"   右臂轨迹点数: {len(trajectories[1]['timed_traj'])}")
            logger.info(f"   躯干轨迹点数: {len(trajectories[2]['timed_traj'])}")
            logger.info(f"   总时长: {trajectories[0]['timed_traj'][-1]['desire_time']:.2f}s")
            
            # 设置轨迹
            result = self.hardware.set_offline_trajectory(trajectories)
            
            if result.success:
                logger.info(f"✅ 双臂+躯干轨迹设置成功")
                message = result.data.get('message', '') if result.data else ''
                logger.info(f"   消息: {message}")
                
                # 验证成功
                self.assertTrue(result.success, "双臂+躯干轨迹应该设置成功")
                
                # 启动执行
                logger.info("🚀 启动轨迹执行...")
                enable_result = self.hardware.enable_offline_trajectory(True)
                
                if enable_result.success:
                    logger.info(f"✅ 轨迹执行已启动")
                    logger.info(f"   说明: 左臂、右臂、躯干同时运动")
                    
                    # 等待运动完成
                    total_time = trajectories[0]['timed_traj'][-1]['desire_time']
                    wait_time = total_time + 1.0
                    logger.info(f"⏱️  等待 {wait_time} 秒让机器人运动...")
                    time.sleep(wait_time)
                    logger.info(f"✅ 协同轨迹执行完成!")
                else:
                    logger.error(f"❌ 轨迹启动失败: {enable_result.message}")
                    self.fail(f"轨迹启动失败: {enable_result.message}")
            else:
                logger.error(f"❌ 双臂+躯干轨迹设置失败: {result.message}")
                self.fail(f"轨迹设置失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ 双臂+躯干轨迹异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"双臂+躯干轨迹异常: {e}")
    
    def test_04_trajectory_validation_error(self):
        """测试4: 轨迹验证错误 - 第一帧时间不为0"""
        print_separator("测试4: 轨迹验证错误 - 第一帧时间不为0")
        
        print("\n测试轨迹验证功能（故意发送错误数据）...")
        logger.info("📤 发送错误的轨迹数据（第一帧时间不为0）")
        
        try:
            # 错误的轨迹 - 第一帧时间不为0
            trajectory = [{
                'planner_index': 0,
                'frame': 0,
                'timed_traj': [
                    {'desire_time': 0.5, 'cmd_vec': [0.3, 0.2, 0.3, 0.0, 0.0, 0.0]},  # 错误：应该为0
                    {'desire_time': 1.0, 'cmd_vec': [0.4, 0.2, 0.35, 0.1, 0.0, 0.0]},
                ]
            }]
            
            # 设置轨迹（应该失败）
            result = self.hardware.set_offline_trajectory(trajectory)
            
            if not result.success:
                logger.info(f"✅ 正确检测到错误: {result.message}")
                logger.info(f"   说明: 第一帧时间必须为0")
                
                # 验证失败（这是预期的）
                self.assertFalse(result.success, "错误的轨迹应该被拒绝")
            else:
                logger.error(f"❌ 未检测到错误，轨迹设置成功（不应该发生）")
                self.fail("错误的轨迹应该被拒绝")
                
        except Exception as e:
            logger.error(f"❌ 验证测试异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"验证测试异常: {e}")
    
    def test_05_trajectory_dimension_error(self):
        """测试5: 轨迹验证错误 - 命令向量维度不匹配"""
        print_separator("测试5: 轨迹验证错误 - 命令向量维度不匹配")
        
        print("\n测试轨迹验证功能（故意发送错误维度）...")
        logger.info("📤 发送错误的轨迹数据（维度不匹配）")
        
        try:
            # 错误的轨迹 - 左臂应该是6维，但只给了4维
            trajectory = [{
                'planner_index': 0,  # 左臂（需要6维）
                'frame': 0,
                'timed_traj': [
                    {'desire_time': 0.0, 'cmd_vec': [0.3, 0.2, 0.3, 0.0]},  # 错误：只有4维
                    {'desire_time': 1.0, 'cmd_vec': [0.4, 0.2, 0.35, 0.1]},
                ]
            }]
            
            # 设置轨迹（应该失败）
            result = self.hardware.set_offline_trajectory(trajectory)
            
            if not result.success:
                logger.info(f"✅ 正确检测到错误: {result.message}")
                logger.info(f"   说明: 左臂需要6维命令向量")
                
                # 验证失败（这是预期的）
                self.assertFalse(result.success, "维度错误的轨迹应该被拒绝")
            else:
                logger.error(f"❌ 未检测到错误，轨迹设置成功（不应该发生）")
                self.fail("维度错误的轨迹应该被拒绝")
                
        except Exception as e:
            logger.error(f"❌ 验证测试异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"验证测试异常: {e}")


def main():
    """主函数"""
    print_separator("离线轨迹测试")
    
    print("\n📋 测试说明:")
    print("  - 测试离线轨迹设置和执行功能")
    print("  - 使用 /mobile_manipulator_timed_offline_traj 服务设置轨迹")
    print("  - 使用 /mobile_manipulator_timed_offline_traj_enable 服务启动执行")
    print("  - 包含5个测试用例:")
    print("    1. 左臂笛卡尔轨迹（世界系）")
    print("    2. 右臂笛卡尔轨迹（世界系）")
    print("    3. 双臂+躯干协同轨迹")
    print("    4. 轨迹验证错误 - 第一帧时间不为0")
    print("    5. 轨迹验证错误 - 命令向量维度不匹配")
    
    print("\n✨ 离线轨迹优势:")
    print("  - 预定义复杂轨迹，一次性缓存")
    print("  - 支持多规划器协同运动")
    print("  - 精确的时间控制")
    print("  - 适合重复执行的固定动作")
    
    print("\n💡 提示:")
    print("  - planner_index: 0=左臂笛卡尔, 1=右臂笛卡尔, 2=躯干")
    print("  - frame: 0=世界系, 1=局部系")
    print("  - 第一帧时间必须为0")
    print("  - 时间必须严格递增")
    print("  - 设置后需调用 enable_offline_trajectory(True) 启动")
    
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
