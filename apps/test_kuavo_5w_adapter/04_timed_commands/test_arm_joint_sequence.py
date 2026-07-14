#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本: 手臂关节时序指令序列

功能描述：
通过适配器发送带时间参数的双臂关节控制指令序列。
控制上肢14个关节的角度（单位：度）。

注意事项：
1. 使用 planner_index=8（左臂）、planner_index=9（右臂）
2. 分别发送左臂和右臂命令
3. 关节角度单位为度（degrees），内部会自动转换为弧度
4. 每条指令包含期望执行时间和目标关节角度
5. **无需设置MPC控制模式** - 系统默认以BaseArm模式运行（V1.4新功能）
6. **无需启用快速模式** - 时序指令服务可直接使用
7. 如需调整运动参数，可使用 set_ruckig_planner_params() 配置规划器

底层对应：
- ROS服务: /mobile_manipulator_timed_single_cmd
- 服务类型: kuavo_msgs/lbTimedPosCmd
- 参考脚本: kuavo-ros-opensource/src/demo/test_kuavo_wheel_real/timedCmd_example/cmd_arm_joint_test.py

运行方式：
    cd ~/LeTools
    python3 apps/test_kuavo_5w_app/04_timed_commands/test_arm_joint_sequence.py
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
logger = logging.getLogger('test_arm_joint_sequence')


def print_separator(title):
    """打印分隔线"""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


class TestArmJointSequence(unittest.TestCase):
    """测试手臂关节时序指令序列"""
    
    @classmethod
    def setUpClass(cls):
        """初始化硬件适配器"""
        logger.info("🔧 初始化硬件适配器...")

        # 按需跳过不需要的组件，只初始化时序管理器即可
        config = {
            'skip_camera': True,# 手臂关节时序指令序列不需要相机
            'skip_end_effector': True,# 手臂关节时序指令序列不需要末端执行器
            'skip_state_manager': True,# 手臂关节时序指令序列不需要状态管理器
            'skip_force_publishers': True,# 手臂关节时序指令序列不需要力控发布器
            'sdk_managers_whitelist': ['timed'],# 只初始化sdk时序管理器
        }
        cls.hardware = LejuWheeledArmHardware(config=config)

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
        adapter_setup(cls.hardware, need_arm=True, mpc_mode=MPCControlMode.ARM_ONLY)

        logger.info("✅ 硬件适配器初始化完成")
    
    @classmethod
    def tearDownClass(cls):
        """清理资源"""
        logger.info("🧹 清理资源...")
        if hasattr(cls, 'hardware'):
            # 注意：MPC模式和快速模式在新版本中已废弃，无需恢复
            # 如需重置机器人状态，可使用 reset_torso_to_initial() 服务

            # === 脚手架: 后置复位 ===
            from apps.test_kuavo_5w_adapter._scaffold import adapter_teardown
            adapter_teardown(cls.hardware, need_arm=True, restore_mpc=True)

            del cls.hardware
        logger.info("✅ 资源清理完成")
    
    def test_arm_joint_sequence(self):
        """测试手臂关节时序指令序列"""
        print_separator("测试: 手臂关节时序指令序列")
        
        # 测试用例列表： (名称, 时间, 左臂角度[7个], 右臂角度[7个])
        # 关节顺序: [肩俯仰, 肩侧摆, 肩偏航, 肘俯仰, 腕偏航, 腕俯仰, 腕滚转]
        test_cases = [
            ("零位姿势", 3.0, 
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # 左臂
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # 右臂
            ("展开双臂", 4.0, 
             [-30, 20, 15, -45, 25, 10, -35],        # 左臂
             [-30,-20,-15, -45,-25,-10, -35]),       # 右臂
            ("弯曲收回", 4.0, 
             [-20, 30, -25, -20, 40, -15, 25],       # 左臂
             [-20,-30, 25, -20,-40, 15,  25]),       # 右臂
            ("回到零位", 3.0, 
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # 左臂
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # 右臂
        ]
        
        success_count = 0
        
        print("\n开始发布手臂关节时序指令...")
        
        for idx, (name, desire_time, left_arm_deg, right_arm_deg) in enumerate(test_cases, 1):
            print(f"\n--- 第{idx}组测试: {name} ---")
            logger.info(f"📤 发送指令: {name}, 期望时间={desire_time}s")
            logger.info(f"   左臂角度: {left_arm_deg}°")
            logger.info(f"   右臂角度: {right_arm_deg}°")
            
            try:
                # 分别发送左臂和右臂命令
                logger.info(f"  [左臂]")
                result_left = self.hardware.send_timed_left_arm_joint(
                    joint_angles=left_arm_deg,
                    desire_time=desire_time
                )
                
                if not result_left.success:
                    logger.error(f"❌ {name} 左臂指令发送失败: {result_left.message}")
                    break
                
                logger.info(f"  [右臂]")
                result_right = self.hardware.send_timed_right_arm_joint(
                    joint_angles=right_arm_deg,
                    desire_time=desire_time
                )
                
                if result_right.success:
                    logger.info(f"✅ {name} 指令发送成功")
                    success_count += 1
                    
                    # 从 Result 中获取实际执行时间（取最大值）
                    actual_time_left = result_left.data.get('actual_time', desire_time) if result_left.data else desire_time
                    actual_time_right = result_right.data.get('actual_time', desire_time) if result_right.data else desire_time
                    actual_time = max(actual_time_left, actual_time_right)
                    
                    logger.info(f"   左臂实际时间: {actual_time_left:.2f}s")
                    logger.info(f"   右臂实际时间: {actual_time_right:.2f}s")
                    logger.info(f"   最大执行时间: {actual_time:.2f}s")
                    
                    # 等待运动完成（使用最大时间 + 缓冲）
                    wait_time = actual_time + 0.5
                    logger.info(f"⏱️  等待 {wait_time} 秒让手臂关节运动...")
                    time.sleep(wait_time)
                    logger.info(f"✅ {name} 完成!")
                else:
                    logger.error(f"❌ {name} 右臂指令发送失败: {result_right.message}")
                    break
                    
            except Exception as e:
                logger.error(f"❌ {name} 异常: {e}")
                break
        
        # 验证至少有一条指令成功
        self.assertGreater(success_count, 0, "至少应该有一条指令成功执行")
        logger.info(f"📊 测试结果: {success_count}/{len(test_cases)} 条指令成功")


def main():
    """主函数"""
    print_separator("手臂关节时序指令序列测试")
    
    print("\n📋 测试说明:")
    print("  - 测试双臂关节的时序控制")
    print("  - 分别发送左臂和右臂命令")
    print("  - 每条指令包含期望执行时间和目标关节角度")
    print("  - 指令按顺序执行，每条指令完成后才执行下一条")
    print("  - 关节角度单位: 度 (degrees)，内部自动转换为弧度")
    print("  - 关节数量: 14个（左右臂各7个）")
    print("  - ✅ 无需设置MPC模式 - 系统默认BaseArm模式")
    print("  - ✅ 无需启用快速模式 - 时序服务可直接使用")
    
    print("\n✨ 使用时序指令服务:")
    print("  - 集成 /mobile_manipulator_timed_single_cmd 服务")
    print("  - MPC 精确计时，提高时序精度")
    print("  - 返回实际执行时间 actualTime")
    print("  - planner_index=8: 左臂上肢关节运动")
    print("  - planner_index=9: 右臂上肢关节运动")
    print("\n💡 提示:")
    print("  - 如手臂不动，可能需要配置 Ruckig 规划器参数")
    print("  - 参考: ruckig_setting_test.py 设置速度/加速度限制")
    
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
