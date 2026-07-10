#!/bin/bash
# scripts/run_camera_demo.sh

cd "$(dirname "$0")/.."

# 设置 ROS 环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

# 运行应用
python3 apps/camera_demo/main.py
