#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS话题诊断工具 - 检查状态反馈相关的话题是否存在和发布数据

运行方式：
    cd ~/LeTools
    python3 apps/test_kuavo_5w_app/07_debug_feedback/diagnose_ros_topics.py
"""

import subprocess
import sys


def check_ros_master():
    """检查ROS Master是否运行"""
    print("=" * 80)
    print("  1. 检查 ROS Master")
    print("=" * 80)
    
    try:
        result = subprocess.run(['rosnode', 'list'], 
                              capture_output=True, text=True, timeout=3)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            print(f"✅ ROS Master 正在运行")
            print(f"   活动节点数: {len([n for n in nodes if n])}")
            return True
        else:
            print(f"❌ ROS Master 未运行")
            print(f"   请先启动: roscore")
            return False
    except Exception as e:
        print(f"❌ 无法连接到 ROS Master: {e}")
        return False


def list_all_topics():
    """列出所有ROS话题"""
    print("\n" + "=" * 80)
    print("  2. 所有 ROS 话题列表")
    print("=" * 80)
    
    try:
        result = subprocess.run(['rostopic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            topics = [t for t in result.stdout.strip().split('\n') if t]
            print(f"   共找到 {len(topics)} 个话题:\n")
            for topic in sorted(topics):
                print(f"     - {topic}")
            return topics
        else:
            print(f"❌ 无法获取话题列表")
            return []
    except Exception as e:
        print(f"❌ 错误: {e}")
        return []


def check_specific_topics():
    """检查特定的状态反馈话题"""
    print("\n" + "=" * 80)
    print("  3. 状态反馈相关话题检查")
    print("=" * 80)
    
    topics_to_check = {
        '到达时间': [
            '/lb_cmd_pose_reach_time',
            '/lb_torso_pose_reach_time',
            '/lb_arm_joint_reach_time/left',
            '/lb_leg_joint_reach_time',
            '/lb_arm_ee_reach_time/left',
        ],
        'MPC相关': [
            '/mobile_manipulator_mpc_control_mode',
            '/mobile_manipulator_mpc_observation',
        ],
        '传感器数据': [
            '/body_acc',
            '/joint_acc',
            '/torque',
        ],
        '位姿数据': [
            '/humanoid_wheel/eePoses',
            '/lb_ee_target_6d',
            '/lb_torso_target_6d',
        ],
    }
    
    # 获取所有话题
    try:
        result = subprocess.run(['rostopic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        available_topics = set(t for t in result.stdout.strip().split('\n') if t)
    except:
        available_topics = set()
    
    all_exist = True
    
    for category, topics in topics_to_check.items():
        print(f"\n   📂 {category}:")
        for topic in topics:
            exists = topic in available_topics
            status = "✅" if exists else "❌"
            print(f"     {status} {topic}")
            if not exists:
                all_exist = False
    
    return all_exist


def check_topic_info(topic_name):
    """检查话题的详细信息"""
    print(f"\n   🔍 话题信息: {topic_name}")
    
    try:
        # 获取话题类型
        result = subprocess.run(['rostopic', 'info', topic_name], 
                              capture_output=True, text=True, timeout=3)
        if result.returncode == 0:
            print(f"      {result.stdout.strip()}")
        else:
            print(f"      ⚠️  无法获取话题信息")
    except Exception as e:
        print(f"      ⚠️  错误: {e}")


def main():
    """主函数"""
    print("\n" + "=" * 80)
    print("  ROS 话题诊断工具")
    print("=" * 80)
    print("\n本工具用于诊断状态反馈相关的ROS话题是否正常\n")
    
    # 1. 检查ROS Master
    master_running = check_ros_master()
    if not master_running:
        print("\n❌ 请先启动 ROS Master (roscore)")
        sys.exit(1)
    
    # 2. 列出所有话题
    all_topics = list_all_topics()
    
    # 3. 检查特定话题
    all_exist = check_specific_topics()
    
    # 4. 总结
    print("\n" + "=" * 80)
    print("  诊断总结")
    print("=" * 80)
    
    if all_exist:
        print("\n✅ 所有期望的话题都存在")
        print("\n如果测试脚本仍然获取不到数据，可能原因:")
        print("   1. 话题存在但没有发布者（数据为空）")
        print("   2. 消息类型不匹配")
        print("   3. 订阅者初始化时机问题")
        print("\n建议:")
        print("   运行以下命令检查话题是否有数据:")
        print("   rostopic echo /body_acc    # 查看本体加速度")
        print("   rostopic hz /body_acc      # 查看发布频率")
    else:
        print("\n⚠️  部分话题不存在")
        print("\n可能原因:")
        print("   1. 机器人驱动未完全启动")
        print("   2. 某些功能模块未启用")
        print("   3. 话题名称与实际不符")
        print("\n建议:")
        print("   1. 检查机器人驱动日志")
        print("   2. 确认哪些功能已启用")
        print("   3. 使用 'rostopic list' 查找实际的话题名称")
    
    print("\n" + "=" * 80)


if __name__ == '__main__':
    main()
