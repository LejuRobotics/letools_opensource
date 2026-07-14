#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态数据验证工具 - 实时显示从适配器获取的状态数据

功能：
1. 实时查询并显示所有状态数据
2. 提供数据合理性检查
3. 帮助判断获取的数据是否正确

运行方式：
    cd ~/LeTools
    python3 apps/test_kuavo_5w_app/07_debug_feedback/verify_state_data.py
"""

import sys
import os
import time
import signal
import numpy as np

# 添加项目根目录到 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.insert(0, project_root)

from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from core.common.logger import get_logger

logger = get_logger(__name__)


def print_separator(title=""):
    """打印分隔线"""
    print("\n" + "=" * 80)
    if title:
        print(f"  {title}")
        print("=" * 80)


def verify_reach_time(hardware, topic_type, name):
    """验证到达时间数据"""
    reach_time = hardware.get_reach_time(topic_type)
    
    print(f"\n📊 {name}:")
    if reach_time is not None:
        print(f"   值: {reach_time:.3f} s")
        
        # 合理性检查
        if reach_time < 0:
            print(f"   ❌ 错误: 到达时间为负数 ({reach_time})")
            return False
        elif reach_time > 10.0:
            print(f"   ⚠️  警告: 到达时间过大 ({reach_time:.1f}s > 10s)")
            return False
        else:
            print(f"   ✅ 正常: 在合理范围内 (0-10s)")
            return True
    else:
        print(f"   ⚠️  无数据: 未收到反馈（可能未发送控制指令）")
        return None


def verify_mpc_control_mode(hardware):
    """验证MPC控制模式"""
    mode = hardware.get_mpc_control_mode()
    
    print(f"\n📊 MPC控制模式:")
    if mode is not None:
        mode_names = {
            0: "NO_CONTROL (无控制)",
            1: "ARM_ONLY (仅手臂)",
            2: "BASE_ONLY (仅基座)",
            3: "BASE_ARM (基座+手臂)",
            4: "ARM_EE_ONLY (仅手臂末端)"
        }
        mode_name = mode_names.get(mode, f"UNKNOWN({mode})")
        print(f"   值: {mode} - {mode_name}")
        
        if mode in mode_names:
            print(f"   ✅ 有效: 是已知的MPC模式")
            return True
        else:
            print(f"   ❌ 错误: 未知的MPC模式 ({mode})")
            return False
    else:
        print(f"   ⚠️  无数据: 未收到MPC控制模式反馈")
        return None


def verify_body_acceleration(hardware):
    """验证本体加速度"""
    accel = hardware.get_body_acceleration()
    
    print(f"\n📊 本体加速度:")
    if accel is not None:
        linear = accel['linear']
        angular = accel['angular']
        
        print(f"   线加速度:")
        print(f"     x = {linear['x']:.4f} m/s²")
        print(f"     y = {linear['y']:.4f} m/s²")
        print(f"     z = {linear['z']:.4f} m/s² (注意: bodyAcc不包含重力)")
        
        print(f"   角加速度:")
        print(f"     x = {angular['x']:.4f} rad/s²")
        print(f"     y = {angular['y']:.4f} rad/s²")
        print(f"     z = {angular['z']:.4f} rad/s²")
        
        # 合理性检查
        linear_mag = np.sqrt(linear['x']**2 + linear['y']**2 + linear['z']**2)
        
        print(f"\n   数据分析:")
        print(f"     线加速度幅值: {linear_mag:.4f} m/s²")
        print(f"     注意: /humanoid_wheel/bodyAcc 不包含重力加速度")
        print(f"     这是底盘的平动和旋转加速度，不是IMU数据")
        
        if linear_mag < 10.0:
            print(f"   ✅ 正常: 加速度在合理范围内")
            return True
        else:
            print(f"   ⚠️  警告: 加速度过大")
            return False
    else:
        print(f"   ⚠️  无数据: 未收到本体加速度反馈")
        return None


def verify_joint_torque(hardware):
    """验证关节力矩"""
    torque = hardware.get_joint_torque()
    
    print(f"\n📊 关节力矩:")
    if torque is not None:
        names = torque['names']
        torques = torque['torques']
        
        print(f"   关节数量: {len(names)}")
        print(f"   力矩详情:")
        
        for i, (name, t) in enumerate(zip(names, torques)):
            status = "✅" if abs(t) < 50 else "⚠️"
            print(f"     [{i:2d}] {name:30s}: {t:8.3f} Nm {status}")
        
        # 统计分析
        abs_torques = [abs(t) for t in torques]
        max_torque = max(abs_torques)
        avg_torque = np.mean(abs_torques)
        
        print(f"\n   统计分析:")
        print(f"     最大力矩: {max_torque:.2f} Nm")
        print(f"     平均力矩: {avg_torque:.2f} Nm")
        print(f"     力矩范围: [{min(torques):.2f}, {max(torques):.2f}] Nm")
        
        if max_torque < 100:
            print(f"   ✅ 正常: 力矩在合理范围内")
            return True
        else:
            print(f"   ⚠️  警告: 存在较大力矩 ({max_torque:.1f} Nm)")
            return False
    else:
        print(f"   ⚠️  无数据: 未收到关节力矩反馈")
        return None


def verify_ee_poses(hardware):
    """验证末端位姿"""
    ee_poses = hardware.get_ee_poses()
    
    print(f"\n📊 末端执行器位姿:")
    if ee_poses is not None:
        print(f"   末端数量: {len(ee_poses)}")
        
        all_valid = True
        for i, ee in enumerate(ee_poses):
            pos = ee['position']
            
            print(f"\n   末端 {i+1}:")
            print(f"     位置 (m):")
            print(f"       x = {pos['x']:.4f}")
            print(f"       y = {pos['y']:.4f}")
            print(f"       z = {pos['z']:.4f}")
            
            # 支持两种格式：四元数和欧拉角
            if 'orientation' in ee:
                # 四元数格式
                ori = ee['orientation']
                print(f"     姿态 (四元数):")
                print(f"       x = {ori['x']:.4f}")
                print(f"       y = {ori['y']:.4f}")
                print(f"       z = {ori['z']:.4f}")
                print(f"       w = {ori['w']:.4f}")
                
                # 验证四元数归一化
                quat_norm = np.sqrt(ori['x']**2 + ori['y']**2 + ori['z']**2 + ori['w']**2)
                normalized = abs(quat_norm - 1.0) < 0.01
                
                print(f"\n     验证:")
                print(f"       四元数模长: {quat_norm:.4f} {'✅ 归一化' if normalized else '❌ 未归一化'}")
                
                if not normalized:
                    all_valid = False
            elif 'orientation_euler' in ee:
                # 欧拉角格式 [yaw, pitch, roll]
                euler = ee['orientation_euler']
                print(f"     姿态 (欧拉角):")
                print(f"       yaw = {euler['yaw']:.4f} rad ({np.degrees(euler['yaw']):.2f}°)")
                print(f"       pitch = {euler['pitch']:.4f} rad ({np.degrees(euler['pitch']):.2f}°)")
                print(f"       roll = {euler['roll']:.4f} rad ({np.degrees(euler['roll']):.2f}°)")
            else:
                print(f"     ⚠️  未知的姿态格式")
                all_valid = False
        
        if all_valid:
            print(f"\n   ✅ 正常: 所有末端位姿数据有效")
            return True
        else:
            print(f"\n   ❌ 错误: 存在未归一化的四元数")
            return False
    else:
        print(f"   ⚠️  无数据: 未收到末端位姿反馈")
        return None


def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    print("\n\n⚠️  收到中断信号，正在关闭...")
    sys.exit(0)

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def check_ros_topics():
    """检查ROS话题是否存在"""
    import subprocess
    
    print("\n🔍 检查ROS话题...")
    
    topics_to_check = [
        '/lb_cmd_pose_reach_time',
        '/lb_torso_pose_reach_time',
        '/lb_arm_joint_reach_time/left',
        '/lb_leg_joint_reach_time',
        '/lb_arm_ee_reach_time/left',
        '/mobile_manipulator_mpc_control_mode',
        '/body_acc',
        '/torque',
        '/humanoid_wheel/eePoses',
    ]
    
    try:
        # 获取所有ROS话题列表
        result = subprocess.run(['rostopic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        available_topics = result.stdout.strip().split('\n')
        
        print(f"   找至 {len(available_topics)} 个ROS话题")
        print(f"\n   期望的话题状态:")
        
        for topic in topics_to_check:
            exists = topic in available_topics
            status = "✅ 存在" if exists else "❌ 不存在"
            print(f"     {status}: {topic}")
        
        return available_topics
    except Exception as e:
        print(f"   ⚠️  无法获取ROS话题列表: {e}")
        return []


def main():
    """主函数"""
    print_separator("状态数据验证工具")
    print("本工具用于验证实时获取的机器人状态数据是否正确")
    print("按 Ctrl+C 退出\n")
    
    hardware = None
    try:
        # 首先检查ROS话题
        available_topics = check_ros_topics()
        
        if not available_topics:
            print("\n❌ 错误: 无法获取ROS话题列表")
            print("   请确保:")
            print("   1. ROS Master 已启动 (roscore)")
            print("   2. 机器人驱动正在运行")
            print("   3. 已 source ROS 环境 (source devel/setup.bash)")
            return
        
        # 初始化硬件
        hardware = LejuWheeledArmHardware(config={
            'skip_sdk_managers': True,
            'skip_end_effector': True,
            'skip_camera': True,
            'skip_force_publishers': True,
        })
        result = hardware.initialize()
        
        if not result.success:
            print(f"❌ 硬件初始化失败: {result.message}")
            return
        
        print("✅ 硬件初始化成功")
        print("等待状态订阅建立...")
        time.sleep(2.0)
        
        # 循环显示状态数据
        iteration = 0
        while True:
            iteration += 1
            print_separator(f"第 {iteration} 次查询")
            
            # 验证各种状态
            results = []
            
            # 1. 到达时间
            results.append(verify_reach_time(hardware, 'cmd_pose', '底盘位置到达时间'))
            results.append(verify_reach_time(hardware, 'torso_pose', '躯干位姿到达时间'))
            results.append(verify_reach_time(hardware, 'arm_joint', '手臂关节到达时间'))
            results.append(verify_reach_time(hardware, 'leg_joint', '腿部关节到达时间'))
            results.append(verify_reach_time(hardware, 'arm_ee', '手臂末端到达时间'))
            
            # 2. MPC控制模式
            results.append(verify_mpc_control_mode(hardware))
            
            # 3. 本体加速度
            results.append(verify_body_acceleration(hardware))
            
            # 4. 关节力矩
            results.append(verify_joint_torque(hardware))
            
            # 5. 末端位姿
            results.append(verify_ee_poses(hardware))
            
            # 统计
            valid = sum(1 for r in results if r is True)
            invalid = sum(1 for r in results if r is False)
            no_data = sum(1 for r in results if r is None)
            
            print_separator("统计摘要")
            print(f"  ✅ 有效数据: {valid} 个")
            print(f"  ❌ 异常数据: {invalid} 个")
            print(f"  ⚠️  无数据: {no_data} 个")
            print(f"  📊 总计: {len(results)} 个状态")
            
            if invalid > 0:
                print(f"\n  ⚠️  发现 {invalid} 个异常数据，请检查上述输出")
            
            print("\n下次查询将在 3 秒后... (按 Ctrl+C 退出)")
            time.sleep(3.0)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断，正在关闭...")
    except Exception as e:
        print(f"\n\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if hardware is not None:
            try:
                hardware.shutdown()
                print("✅ 硬件连接已关闭")
            except Exception as e:
                print(f"⚠️  关闭硬件时出错: {e}")
        print("\n程序已退出")
        sys.exit(0)


if __name__ == '__main__':
    main()
