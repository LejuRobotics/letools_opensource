#!/bin/bash
# scripts/run_arm_trajectory.sh

cd "$(dirname "$0")/.."

# 设置 ROS 环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

# 运行应用
python3 apps/arm_trajectory/main.py