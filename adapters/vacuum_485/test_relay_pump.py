#!/usr/bin/env python3
"""
气泵继电器控制 — 交互式测试示例

用法:
    python test_relay_pump.py

依赖:
    - ROS master 已启动
    - 下位机继电器服务已就绪
"""

import os
import sys

# 将项目根目录加入 Python 路径，确保直接运行时也能导入 adapters 包
_project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

import rospy
from adapters.vacuum_485.relay_pump_client import RelayPumpClient


if __name__ == "__main__":
    rospy.init_node('relay_pump_upper_client', anonymous=True)

    try:
        pump = RelayPumpClient()
    except Exception as e:
        rospy.logfatal(f"客户端初始化失败: {e}")
        rospy.logfatal("请检查：下位机服务是否启动、ROS_MASTER_URI 及参数配置")
        exit(1)

    print("\n气泵控制客户端已启动，输入命令控制")
    while not rospy.is_shutdown():
        print("\n" + "=" * 36)
        print("  1 → 吹气")
        print("  2 → 吸气")
        print("  3 → 循环测试 (吸气→断电)")
        print("  4 → 断电全关")
        print("  q → 退出")
        print("=" * 36)

        try:
            cmd = input("请输入选项: ").strip()
        except (EOFError, KeyboardInterrupt):
            rospy.loginfo("用户中断输入，退出")
            break

        if cmd == "1":
            pump.blow()
        elif cmd == "2":
            pump.suck()
        elif cmd == "3":
            pump.cycle_test()
        elif cmd == "4":
            pump.power_off()
        elif cmd.lower() == "q":
            print("退出气泵控制程序")
            break
        else:
            print("无效选项，请输入 1-4 或 q")
