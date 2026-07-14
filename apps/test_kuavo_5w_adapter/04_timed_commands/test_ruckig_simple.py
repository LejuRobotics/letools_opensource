#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版 Ruckig 参数配置测试

功能：
- 只测试 planner_index=0 (底盘)
- 添加足够的延迟避免节点崩溃
- 验证基本功能是否正常

运行方式：
    cd ~/LeTools
    python3 apps/test_kuavo_5w_app/04_timed_commands/test_ruckig_simple.py
"""

import sys
import os
import time

# 添加项目根目录到 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.insert(0, project_root)

from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware

print("=" * 80)
print("  简化版 Ruckig 参数配置测试")
print("=" * 80)
print()

# 初始化硬件
print("🔧 初始化硬件适配器...")
hardware = LejuWheeledArmHardware()
result = hardware.initialize()
if not result.success:
    print(f"❌ 硬件初始化失败: {result.message}")
    sys.exit(1)

print("✅ 硬件初始化成功")
print()

# 等待ROS节点稳定
print("⏳ 等待ROS节点稳定 (3秒)...")
time.sleep(3)

# 测试1: 底盘位置规划器
print("\n" + "=" * 80)
print("  测试1: 底盘位置规划器 (planner_index=0)")
print("=" * 80)

try:
    result = hardware.set_ruckig_planner_params(
        planner_index=0,
        is_sync=True,
        velocity_max=[0.2, 0.2, 0.2],
        acceleration_max=[2.0, 2.0, 1.5],
        jerk_max=[20.0, 15.0, 12.0]
    )
    
    if result.success:
        print("✅ 测试1通过: 底盘位置规划器参数设置成功")
        print(f"   消息: {result.data.get('message', '') if result.data else ''}")
    else:
        print(f"❌ 测试1失败: {result.message}")
        
except Exception as e:
    print(f"❌ 测试1异常: {e}")
    import traceback
    traceback.print_exc()

# 等待
print("\n⏳ 等待5秒，让节点稳定...")
time.sleep(5)

# 测试2: 尝试左臂（如果节点还活着）
print("\n" + "=" * 80)
print("  测试2: 左臂关节规划器 (planner_index=8)")
print("=" * 80)

try:
    result = hardware.set_ruckig_planner_params(
        planner_index=8,
        is_sync=True,
        velocity_max=[1.0] * 7,
        acceleration_max=[5.0] * 7,
        jerk_max=[50.0] * 7
    )
    
    if result.success:
        print("✅ 测试2通过: 左臂关节规划器参数设置成功")
    else:
        print(f"❌ 测试2失败: {result.message}")
        
except Exception as e:
    print(f"❌ 测试2异常: {e}")
    import traceback
    traceback.print_exc()

# 清理
print("\n🧹 清理资源...")
# === 脚手架: 后置复位 ===
from apps.test_kuavo_5w_adapter._scaffold import adapter_teardown
adapter_teardown(hardware, need_arm=False, restore_mpc=True)

del hardware

print("\n" + "=" * 80)
print("  测试完成")
print("=" * 80)
