#!/bin/bash
# kuavo_tf2_web_republisher 启动脚本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "启动 kuavo_tf2_web_republisher"
echo "=========================================="

# 检查 ROS 环境
if [ -z "${ROS_DISTRO}" ]; then
    echo "错误：ROS 环境未配置"
    echo "请先运行："
    echo "  source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查包是否已编译
EXECUTABLE="devel/lib/kuavo_tf2_web_republisher/kuavo_tf2_web_republisher"
if [ ! -f "${EXECUTABLE}" ]; then
    echo "错误：未找到编译后的可执行文件"
    echo "请先运行："
    echo "  cd ${SCRIPT_DIR}"
    echo "  ./build.sh"
    exit 1
fi

# 设置 ROS 包路径
echo "设置 ROS 环境..."
source "${SCRIPT_DIR}/devel/setup.bash"

# 检查 ROS Master
echo "检查 ROS Master..."
if ! rostopic list > /dev/null 2>&1; then
    echo "警告：ROS Master 未启动"
    echo "建议启动 ROS Master："
    echo "  roscore &"
    read -p "是否继续启动服务？(y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "已取消启动"
        exit 0
    fi
fi

# 检查服务是否已存在
echo "检查服务状态..."
if rosservice list 2>/dev/null | grep -q "/republish_tfs"; then
    echo "警告：服务 /republish_tfs 已在运行"
    read -p "是否重启服务？(y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "停止现有服务..."
        rosnode kill /kuavo_tf2_web_republisher 2>/dev/null || true
        sleep 2
    else
        echo "已取消启动"
        exit 0
    fi
fi

# 启动服务
echo "启动 TF2 Web Republisher 服务..."
$EXECUTABLE &
PID=$!

# 等待服务启动
echo "等待服务启动..."
sleep 2

# 检查服务是否启动成功
if rosservice list 2>/dev/null | grep -q "/republish_tfs"; then
    echo ""
    echo "=========================================="
    echo "✅ 服务启动成功！"
    echo "=========================================="
    echo "服务进程 ID: ${PID}"
    echo "服务名称: /kuavo_tf2_web_republisher"
    echo "服务接口: /republish_tfs"
    echo ""
    echo "停止服务命令："
    echo "  kill ${PID}"
    echo "  或"
    echo "  rosnode kill /kuavo_tf2_web_republisher"
    echo ""
    
    # 保持脚本运行
    wait $PID
else
    echo ""
    echo "=========================================="
    echo "❌ 服务启动失败！"
    echo "=========================================="
    echo "请检查以下内容："
    echo "1. ROS Master 是否正常运行"
    echo "2. 编译是否成功"
    echo "3. 查看日志获取详细错误信息"
    kill $PID 2>/dev/null || true
    exit 1
fi