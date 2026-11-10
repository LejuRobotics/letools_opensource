#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
真空控制系统 - 测试与示例代码

使用方法:
    python test_vacuum.py
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))
import time
from vacuum_control import control_relay, control_vacuum_pump, read_pressure_kpa


def print_menu():
    """打印菜单"""
    print("\n" + "="*60)
    print("          真空控制系统 - 测试工具")
    print("="*60)
    print("1. 吸气 (控制气泵吸住)")
    print("2. 松开 (控制气泵放下)")
    print("3. 读取气压值")
    print("4. 循环测试: 吸->等3秒->放")
    print("q. 退出")
    print("="*60)


def test_vacuum_on():
    """测试吸气"""
    print("\n[操作] 正在启动气泵吸气...")
    result = control_vacuum_pump(relay_index=0, action='ON', duration=0.0)
    if result:
        print("[成功] 气泵已开启，正在吸气！")
    else:
        print("[失败] 气泵开启失败！")


def test_vacuum_off():
    """测试松开"""
    print("\n[操作] 正在关闭气泵并破真空...")
    
    # 1. 关闭气泵
    result1 = control_vacuum_pump(relay_index=0, action='OFF')
    
    # 2. 触发继电器1用于破真空
    result2 = control_relay(relay_index=1, action='ON')
    time.sleep(0.2)
    result3 = control_relay(relay_index=1, action='OFF')
    
    if result1:
        print("[成功] 气泵已关闭！")
    else:
        print("[失败] 气泵关闭失败！")


def test_read_pressure():
    """测试读取气压"""
    print("\n[操作] 正在读取气压传感器...")
    left_pressure = read_pressure_kpa(sensor_id=2)
    right_pressure = read_pressure_kpa(sensor_id=1)
    
    print(f"  左臂气压: {left_pressure:.2f} kPa" if left_pressure is not None else "  左臂气压: 读取失败")
    print(f"  右臂气压: {right_pressure:.2f} kPa" if right_pressure is not None else "  右臂气压: 读取失败")


def test_cycle():
    """循环测试"""
    print("\n[操作] 开始循环测试...")
    print("  步骤1: 吸气...")
    test_vacuum_on()
    
    print("\n  等待3秒...")
    time.sleep(3)
    
    print("\n  读取当前气压...")
    test_read_pressure()
    
    print("\n  步骤2: 松开...")
    test_vacuum_off()
    print("\n[完成] 循环测试结束！")


def simple_use_case():
    """简单使用示例（无交互）"""
    print("[简单示例] 开始示例使用...")
    
    # 1. 吸气
    print("  1. 吸气...")
    control_vacuum_pump(relay_index=0, action='ON')
    time.sleep(2)
    
    # 2. 读取气压
    print("  2. 读取气压...")
    left = read_pressure_kpa(2)
    right = read_pressure_kpa(1)
    print(f"    左臂: {left}, 右臂: {right}")
    time.sleep(1)
    
    # 3. 松开
    print("  3. 松开...")
    control_vacuum_pump(relay_index=0, action='OFF')
    control_relay(relay_index=1, action='ON')
    time.sleep(0.2)
    control_relay(relay_index=1, action='OFF')
    
    print("[简单示例] 完成！")


def main():
    """主函数"""
    print("\n[初始化] 真空控制系统已准备好！")
    print("[提示] 完全纯 Python 实现，无 ROS 依赖")
    
    while True:
        print_menu()
        choice = input("\n请输入选项 (1-4, q): ").strip().lower()
        
        if choice == '1':
            test_vacuum_on()
        elif choice == '2':
            test_vacuum_off()
        elif choice == '3':
            test_read_pressure()
        elif choice == '4':
            test_cycle()
        elif choice == 'q':
            print("\n[退出] 正在退出...")
            break
        else:
            print("\n[错误] 无效选项！请重新输入。")
        
        time.sleep(0.5)


if __name__ == "__main__":
    import sys
    import os
    # 确保能找到 vacuum_control 模块
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    main()
