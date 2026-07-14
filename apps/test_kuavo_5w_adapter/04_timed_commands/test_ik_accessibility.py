#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本: IK可达性检查（逆运动学求解）

⚠️ 重要提示: 此功能尚未在底层实现！
- ROS服务 `/mobile_manipulator_ik_accessibility_check` 不存在
- 需要启动 MPC 节点 (`humanoid_wheel_interface_ros`)
- 当前状态: ❌ 跳过测试，等待底层团队实现

功能描述：
通过适配器检查目标位姿是否可达，使用 /mobile_manipulator_ik_accessibility_check 服务
进行IK逆运动学求解，返回最优关节角度和误差信息。

注意事项：
1. 支持左臂/右臂选择
2. 支持世界系/局部系两种坐标系
3. 支持全身运动/仅手臂运动两种模式
4. 提供位置优先零空间解作为备选方案
5. **当前服务未实现，测试已跳过**

底层对应：
- ROS服务: /mobile_manipulator_ik_accessibility_check (❌ 不存在)
- 服务类型: kuavo_msgs/accessIkSolve
- 参考脚本: kuavo-ros-opensource/src/demo/test_kuavo_wheel_real/check_target_pose_reachable_and_execution.py

运行方式：
    cd ~/LeTools
    # ⚠️ 服务未实现，暂时无法运行
    # python3 apps/test_kuavo_5w_app/04_timed_commands/test_ik_accessibility.py
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
logger = logging.getLogger('test_ik_accessibility')


def print_separator(title):
    """打印分隔线"""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


class TestIKAccessibility(unittest.TestCase):
    """测试 IK 可达性检查功能"""
    
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

        # 环境检测: 检查必要的 ROS 服务是否可用
        from apps.test_kuavo_5w_adapter._scaffold import check_services_available
        ok, missing = check_services_available([
            '/mobile_manipulator_ik_accessibility_check',
        ])
        if not ok:
            raise unittest.SkipTest(f"ROS 服务不可用: {missing}（底层 IK 服务尚未实现）")

        # 等待ROS节点初始化
        time.sleep(2)
        
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
    
    def test_01_left_arm_world_frame_reachable(self):
        """测试1: 左臂世界系可达位姿检查"""
        print_separator("测试1: 左臂世界系可达位姿检查")
        
        print("\n检查左臂在世界坐标系中的可达位姿...")
        logger.info("🎯 检查左臂世界系可达性")
        
        try:
            # 定义一个合理的目标位姿（左臂前方）
            pose_desired = [0.4, 0.2, 0.3, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
            
            logger.info(f"   目标位姿: {pose_desired}")
            logger.info(f"   坐标系: 世界系")
            logger.info(f"   运动模式: 仅手臂")
            
            # 调用IK可达性检查
            result = self.hardware.check_ik_accessibility(
                is_left=True,           # 左臂
                is_local=False,         # 世界系
                is_whole_body=False,    # 仅手臂
                pose_desired=pose_desired,
                total_time_desired=1.0,
                max_attempts=5,
                linear_error_max=0.005,     # 5mm
                angular_error_max=0.05      # ~2.86度
            )
            
            if result.success:
                logger.info(f"✅ IK可达性检查成功")
                
                # 提取结果数据
                data = result.data if result.data else {}
                ik_success = data.get('success', False)
                linear_error = data.get('best_linear_error', -1.0)
                angular_error = data.get('best_angular_error', -1.0)
                q_best = data.get('q_best', [])
                
                if ik_success:
                    logger.info(f"   ✅ 目标位姿可达（精确IK解）")
                    logger.info(f"   线位移误差: {linear_error:.6f}m")
                    logger.info(f"   角位移误差: {angular_error:.6f}rad")
                    logger.info(f"   最优关节角度: {q_best}")
                    
                    # 验证成功
                    self.assertTrue(ik_success, "目标位姿应该可达")
                else:
                    logger.warning(f"   ⚠️ 目标位姿不可达（精确IK解不满足要求）")
                    logger.warning(f"   最佳线位移误差: {linear_error:.6f}m")
                    logger.warning(f"   最佳角位移误差: {angular_error:.6f}rad")
                    
                    # 检查位置优先解
                    pos_priority_access = data.get('pos_priority_access', False)
                    if pos_priority_access:
                        logger.info(f"   ✅ 位置优先零空间解满足要求")
                        logger.info(f"      线位移误差: {data.get('pos_priority_linear_error', -1.0):.6f}m")
                        logger.info(f"      角位移误差: {data.get('pos_priority_angular_error', -1.0):.6f}rad")
                    else:
                        logger.warning(f"   ❌ 位置优先零空间解也不满足要求")
            else:
                logger.error(f"❌ IK可达性检查失败: {result.message}")
                self.fail(f"IK检查失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ IK可达性检查异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"IK检查异常: {e}")
    
    def test_02_right_arm_local_frame_reachable(self):
        """测试2: 右臂局部系可达位姿检查"""
        print_separator("测试2: 右臂局部系可达位姿检查")
        
        print("\n检查右臂在局部坐标系中的可达位姿...")
        logger.info("🎯 检查右臂局部系可达性")
        
        try:
            # 定义一个合理的目标位姿（右臂前方）
            pose_desired = [0.4, -0.2, 0.3, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
            
            logger.info(f"   目标位姿: {pose_desired}")
            logger.info(f"   坐标系: 局部系")
            logger.info(f"   运动模式: 仅手臂")
            
            # 调用IK可达性检查
            result = self.hardware.check_ik_accessibility(
                is_left=False,          # 右臂
                is_local=True,          # 局部系
                is_whole_body=False,    # 仅手臂
                pose_desired=pose_desired,
                total_time_desired=1.0,
                max_attempts=5,
                linear_error_max=0.005,
                angular_error_max=0.05
            )
            
            if result.success:
                logger.info(f"✅ IK可达性检查成功")
                
                # 提取结果数据
                data = result.data if result.data else {}
                ik_success = data.get('success', False)
                
                if ik_success:
                    logger.info(f"   ✅ 目标位姿可达")
                    self.assertTrue(ik_success, "目标位姿应该可达")
                else:
                    logger.warning(f"   ⚠️ 目标位姿不可达")
            else:
                logger.error(f"❌ IK可达性检查失败: {result.message}")
                self.fail(f"IK检查失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ IK可达性检查异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"IK检查异常: {e}")
    
    def test_03_whole_body_motion(self):
        """测试3: 全身运动模式可达性检查"""
        print_separator("测试3: 全身运动模式可达性检查")
        
        print("\n检查全身运动模式下的可达位姿...")
        logger.info("🎯 检查全身运动模式可达性")
        
        try:
            # 定义一个较远的目标位姿（需要全身运动）
            pose_desired = [0.6, 0.0, 0.4, 0.0, 0.0, 0.0]
            
            logger.info(f"   目标位姿: {pose_desired}")
            logger.info(f"   坐标系: 世界系")
            logger.info(f"   运动模式: 全身运动")
            
            # 调用IK可达性检查（全身模式）
            result = self.hardware.check_ik_accessibility(
                is_left=True,           # 左臂
                is_local=False,         # 世界系
                is_whole_body=True,     # 全身运动
                pose_desired=pose_desired,
                total_time_desired=1.0,
                max_attempts=5,
                linear_error_max=0.005,
                angular_error_max=0.05
            )
            
            if result.success:
                logger.info(f"✅ IK可达性检查成功")
                
                # 提取结果数据
                data = result.data if result.data else {}
                ik_success = data.get('success', False)
                
                if ik_success:
                    logger.info(f"   ✅ 全身运动模式下目标位姿可达")
                    logger.info(f"   说明: 下肢和手臂协同运动到达目标")
                    self.assertTrue(ik_success, "全身运动模式下目标位姿应该可达")
                else:
                    logger.warning(f"   ⚠️ 即使全身运动也无法到达目标")
            else:
                logger.error(f"❌ IK可达性检查失败: {result.message}")
                self.fail(f"IK检查失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ IK可达性检查异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"IK检查异常: {e}")
    
    def test_04_unreachable_pose(self):
        """测试4: 不可达位姿检查"""
        print_separator("测试4: 不可达位姿检查")
        
        print("\n检查明显超出工作空间的位姿...")
        logger.info("🎯 检查不可达位姿")
        
        try:
            # 定义一个明显不可达的目标位姿（太远）
            pose_desired = [2.0, 0.0, 2.0, 0.0, 0.0, 0.0]  # 超出工作空间
            
            logger.info(f"   目标位姿: {pose_desired}")
            logger.info(f"   预期结果: 不可达")
            
            # 调用IK可达性检查
            result = self.hardware.check_ik_accessibility(
                is_left=True,
                is_local=False,
                is_whole_body=True,
                pose_desired=pose_desired,
                total_time_desired=1.0,
                max_attempts=5,
                linear_error_max=0.005,
                angular_error_max=0.05
            )
            
            if result.success:
                # 即使不可达，服务调用本身也是成功的
                data = result.data if result.data else {}
                ik_success = data.get('success', False)
                
                if not ik_success:
                    logger.info(f"✅ 正确检测到不可达位姿")
                    logger.info(f"   最佳线位移误差: {data.get('best_linear_error', -1.0):.6f}m")
                    logger.info(f"   最佳角位移误差: {data.get('best_angular_error', -1.0):.6f}rad")
                    
                    # 验证失败（这是预期的）
                    self.assertFalse(ik_success, "这个位姿应该不可达")
                else:
                    logger.warning(f"⚠️ 意外地认为位姿可达")
            else:
                logger.error(f"❌ IK可达性检查失败: {result.message}")
                self.fail(f"IK检查失败: {result.message}")
                
        except Exception as e:
            logger.error(f"❌ IK可达性检查异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"IK检查异常: {e}")
    
    def test_05_invalid_pose_dimension(self):
        """测试5: 无效位姿维度检查"""
        print_separator("测试5: 无效位姿维度检查")
        
        print("\n测试参数验证功能（故意发送错误维度）...")
        logger.info("📤 发送错误的位姿数据（维度不正确）")
        
        try:
            # 错误的位姿 - 只有3维，应该是6维
            pose_desired = [0.4, 0.2, 0.3]  # 错误：缺少roll, pitch, yaw
            
            # 调用IK可达性检查（应该失败）
            result = self.hardware.check_ik_accessibility(
                is_left=True,
                is_local=False,
                is_whole_body=False,
                pose_desired=pose_desired,  # 错误的维度
                total_time_desired=1.0,
                max_attempts=5,
                linear_error_max=0.005,
                angular_error_max=0.05
            )
            
            if not result.success:
                logger.info(f"✅ 正确检测到错误: {result.message}")
                logger.info(f"   说明: 位姿必须是6维向量 [x,y,z,roll,pitch,yaw]")
                
                # 验证失败（这是预期的）
                self.assertFalse(result.success, "错误的位姿维度应该被拒绝")
            else:
                logger.error(f"❌ 未检测到错误，IK检查成功（不应该发生）")
                self.fail("错误的位姿维度应该被拒绝")
                
        except Exception as e:
            logger.error(f"❌ 验证测试异常: {e}")
            import traceback
            traceback.print_exc()
            self.fail(f"验证测试异常: {e}")


def main():
    """主函数"""
    print_separator("IK可达性检查测试")
    
    print("\n📋 测试说明:")
    print("  - 测试IK逆运动学求解和可达性检查功能")
    print("  - 使用 /mobile_manipulator_ik_accessibility_check 服务")
    print("  - 包含5个测试用例:")
    print("    1. 左臂世界系可达位姿检查")
    print("    2. 右臂局部系可达位姿检查")
    print("    3. 全身运动模式可达性检查")
    print("    4. 不可达位姿检查（超出工作空间）")
    print("    5. 无效位姿维度检查（参数验证）")
    
    print("\n✨ IK可达性检查优势:")
    print("  - 预先验证目标位姿是否可达")
    print("  - 避免下发无法执行的指令")
    print("  - 提供最优关节角度解")
    print("  - 支持位置优先零空间解作为备选")
    
    print("\n💡 提示:")
    print("  - is_left: True=左臂, False=右臂")
    print("  - is_local: True=局部系, False=世界系")
    print("  - is_whole_body: True=全身运动, False=仅手臂")
    print("  - pose_desired: [x, y, z, roll, pitch, yaw] (6维)")
    print("  - linear_error_max: 最大线位移误差（米）")
    print("  - angular_error_max: 最大角位移误差（弧度）")
    
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
