#!/bin/bash
# kuavo_tf2_web_republisher 编译脚本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "编译 kuavo_tf2_web_republisher"
echo "=========================================="

# 检查 ROS 环境
if [ -z "${ROS_DISTRO}" ]; then
    echo "错误：ROS 环境未配置，请先运行："
    echo "  source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查 catkin_tools 是否安装
if ! command -v catkin &> /dev/null; then
    echo "[1/4] 安装 catkin_tools..."
    sudo apt-get update && sudo apt-get install -y python3-catkin-tools
fi

# 进入工作空间目录
cd "${SCRIPT_DIR}"

# 确保 src 目录存在
if [ ! -d "src" ]; then
    echo "错误：src 目录不存在"
    exit 1
fi

# 检查 kuavo_msgs 是否存在
if [ ! -d "src/kuavo_msgs" ]; then
    echo "错误：src/kuavo_msgs 目录不存在"
    exit 1
fi

# 检查 kuavo_tf2_web_republisher package.xml 是否存在
if [ ! -f "src/kuavo_tf2_web_republisher/package.xml" ]; then
    echo "错误：src/kuavo_tf2_web_republisher/package.xml 不存在"
    exit 1
fi

# 初始化 catkin 工作空间
echo "[2/4] 初始化 catkin 工作空间..."
catkin init

# 清理旧的构建文件
echo "[3/4] 清理旧的构建文件..."
catkin clean -y

# 使用 catkin build 编译（kuavo_msgs 在同一工作空间内，会自动处理依赖）
echo "[4/4] 使用 catkin build 编译所有包"
# catkin build kuavo_tf2_web_republisher -DCMAKE_BUILD_TYPE=Release --cmake-args -DCMAKE_CXX_FLAGS="-O2"
catkin build  -DCMAKE_BUILD_TYPE=Release --cmake-args -DCMAKE_CXX_FLAGS="-O2"
# 检查编译是否成功
EXECUTABLE="devel/lib/kuavo_tf2_web_republisher/kuavo_tf2_web_republisher"
if [ ! -f "${EXECUTABLE}" ]; then
    echo "错误：编译失败"
    exit 1
fi

echo ""
echo "=========================================="
echo "编译完成！"
echo "=========================================="
echo ""
echo "启动服务："
echo "  cd ${SCRIPT_DIR}"
echo "  source devel/setup.bash"
echo "  rosrun kuavo_tf2_web_republisher kuavo_tf2_web_republisher"
echo ""