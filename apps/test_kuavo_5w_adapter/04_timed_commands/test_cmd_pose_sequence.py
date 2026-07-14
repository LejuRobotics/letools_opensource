#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本: cmd_pose 时序指令序列（躯干位姿）

功能描述：
通过适配器发送带时间参数的躯干位姿控制指令序列。
控制躯干相对于基座的位姿（x, z, yaw, pitch）。

注意事项：
1. 使用 planner_index=2（躯干笛卡尔局部系运动）
2. y 和 roll 自由度不起作用
3. 每条指令包含期望执行时间和目标位姿

运行方式：
    cd ~/LeTools
    python3 apps/test_kuavo_5w_app/04_timed_commands/test_cmd_pose_sequence.py
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
logger = logging.getLogger('test_cmd_pose_sequence')


def print_separator(title):
    """打印分隔线"""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


class TestCmdPoseSequence(unittest.TestCase):
    """测试躯干位姿时序指令序列"""
    
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

        # 脚手架: 前置设置（躯干复位 + MPC 模式）
        from apps.test_kuavo_5w_adapter._scaffold import adapter_setup
        from core.domain.enums import MPCControlMode
        adapter_setup(cls.hardware, mpc_mode=MPCControlMode.BASE_ARM)

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
    
    def test_torso_pose_sequence(self):
        """测试躯干位姿时序指令序列"""
        print_separator("测试: 躯干位姿时序指令序列 (planner_index=2)")
        
        # 测试用例列表： (名称, 时间, [x, z, yaw, pitch])
        # 注意：planner_index=2 需要4维向量 [x, z, yaw, pitch]
        # 角度单位为度（degrees），适配器内部自动转换为弧度
        test_cases = [
            ("初始位置", 3.0, [0.0, 1.4, 0.0, 0.0]),
            ("向前倾斜", 3.0, [0.3, 1.4, 0.0, 17.19]),
            ("向后移动", 3.0, [-0.2, 1.4, 0.0, 0.0]),
            ("向上抬升", 3.0, [0.0, 1.5, 0.0, 0.0]),
            ("回到初始", 3.0, [0.0, 1.4, 0.0, 0.0]),
        ]
        
        success_count = 0
        
        print("\n开始发布躯干位姿时序指令...")
        
        for idx, (name, desire_time, pose_data) in enumerate(test_cases, 1):
            print(f"\n--- 第{idx}组测试: {name} ---")
            logger.info(f"📤 发送指令: {name}, 期望时间={desire_time}s")
            logger.info(f"   位姿: x={pose_data[0]:.2f}m, z={pose_data[1]:.2f}m, "
                       f"yaw={pose_data[2]:.2f}deg, pitch={pose_data[3]:.2f}deg")
            
            try:
                # 使用新的时序指令接口
                result = self.hardware.send_timed_torso_pose(
                    x=pose_data[0],
                    z=pose_data[1],
                    yaw=pose_data[2],
                    pitch=pose_data[3],
                    desire_time=desire_time
                )
                
                if result.success:
                    logger.info(f"✅ {name} 指令发送成功")
                    success_count += 1
                    
                    # 从 Result 中获取实际执行时间
                    actual_time = result.data.get('actual_time', desire_time) if result.data else desire_time
                    logger.info(f"   实际执行时间: {actual_time:.2f}s")
                    
                    # 等待运动完成（使用实际时间 + 缓冲）
                    wait_time = actual_time + 0.5
                    logger.info(f"⏱️  等待 {wait_time} 秒让躯干运动...")
                    time.sleep(wait_time)
                    logger.info(f"✅ {name} 完成!")
                else:
                    logger.error(f"❌ {name} 指令发送失败: {result.message}")
                    break
                    
            except Exception as e:
                logger.error(f"❌ {name} 异常: {e}")
                break
        
        # 验证至少有一条指令成功
        self.assertGreater(success_count, 0, "至少应该有一条指令成功执行")
        logger.info(f"📊 测试结果: {success_count}/{len(test_cases)} 条指令成功")


def main():
    """主函数"""
    print_separator("躯干位姿时序指令序列测试")
    
    print("\n📋 测试说明:")
    print("  - 测试躯干位姿的时序控制")
    print("  - 每条指令包含期望执行时间和目标位姿")
    print("  - 指令按顺序执行，每条指令完成后才执行下一条")
    print("  - planner_index=2: 躯干笛卡尔局部系运动")
    print("  - cmd_vec 格式: [x(m), z(m), yaw(deg), pitch(deg)]")
    print("  - 注意: y 和 roll 自由度不起作用")
    
    print("\n✨ 使用真正的时序指令服务:")
    print("  - 集成 /mobile_manipulator_timed_single_cmd 服务")
    print("  - MPC 精确计时，提高时序精度")
    print("  - 返回实际执行时间 actualTime")
    
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
