#!/bin/bash
# init_config.sh - LeTools SDK 配置初始化向导

set -e

SCRIPT_DIR=$(dirname "$(realpath "$0")")
CONFIG_FILE="$SCRIPT_DIR/sdk_config.sh"
TEMPLATE_FILE="$SCRIPT_DIR/sdk_config.sh.template"

echo "=================================================="
echo "  LeTools SDK 配置初始化向导"
echo "=================================================="
echo ""

if [ -f "$CONFIG_FILE" ]; then
    echo -e "⚠️  配置文件已存在: $CONFIG_FILE"
    echo ""
    read -p "是否重新配置？[y/N]: " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "使用现有配置"
        exit 0
    fi
    echo ""
fi

if [ ! -f "$TEMPLATE_FILE" ]; then
    echo -e "❌ 未找到模板文件: $TEMPLATE_FILE"
    exit 1
fi

echo "📋 LeTools 目录结构："
echo "  LeTools/"
echo "  └── drivers/"
echo "      └── leju/"
echo "          └── kuavo_humanoid_sdk/ (Submodule)"
echo "              └── src/"
echo "                  └── kuavo_humanoid_sdk/  ← SDK 实际源码"
echo ""
echo "回答以下问题以生成配置文件："
echo "(直接按 Enter 使用默认值)"
echo ""

# 读取当前配置（如果有）
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE" 2>/dev/null || true
fi

# ROS devel 路径
echo "1. ROS devel 目录路径"
echo "   相对于 LeTools 项目根目录的路径"
read -p "   路径 [默认: ${SDK_ROS_DEVEL_PATH:-infrastructure/ros_packages/devel/}]: " input
SDK_ROS_DEVEL_PATH=${input:-${SDK_ROS_DEVEL_PATH:-infrastructure/ros_packages/devel/}}

# 版本获取策略
echo ""
echo "2. Git 版本获取策略"
echo "   a) 从上游 submodule 获取（✅ 推荐，与代码来源一致）"
echo "   b) 从 LeTools 项目根目录获取"
echo "   c) 自动选择（先 project，后 upstream）"
echo "   d) 使用默认版本"
current_strategy="a"
if [ "${SDK_VERSION_STRATEGY:-upstream}" = "project" ]; then
    current_strategy="b"
elif [ "${SDK_VERSION_STRATEGY:-upstream}" = "auto" ]; then
    current_strategy="c"
elif [ "${SDK_VERSION_STRATEGY:-upstream}" = "fallback" ]; then
    current_strategy="d"
fi
read -p "   选择 [a/b/c/d] [默认: $current_strategy]: " input
input=${input:-$current_strategy}
case $input in
    b) SDK_VERSION_STRATEGY="project" ;;
    c) SDK_VERSION_STRATEGY="auto" ;;
    d) SDK_VERSION_STRATEGY="fallback" ;;
    *) SDK_VERSION_STRATEGY="upstream" ;;
esac

# pip 镜像
echo ""
echo "3. pip 镜像源"
echo "   1) 清华大学镜像（国内推荐）"
echo "   2) 阿里云镜像"
echo "   3) PyPI 官方"
current_mirror="1"
if [[ "${SDK_PIP_MIRROR:-}" == *"aliyun"* ]]; then
    current_mirror="2"
elif [[ "${SDK_PIP_MIRROR:-}" == *"pypi.org"* ]]; then
    current_mirror="3"
fi
read -p "   选择 [1/2/3] [默认: $current_mirror]: " input
input=${input:-$current_mirror}
case $input in
    2) SDK_PIP_MIRROR="https://mirrors.aliyun.com/pypi/simple/" ;;
    3) SDK_PIP_MIRROR="https://pypi.org/simple/" ;;
    *) SDK_PIP_MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple/" ;;
esac

# 生成配置文件
cat > "$CONFIG_FILE" << EOF
#!/bin/bash
# SDK 配置文件 - LeTools
# 生成时间: $(date)
# 
# 注意：此文件已加入 .gitignore，不会提交到版本控制

# 路径配置
export SDK_ROS_DEVEL_PATH="$SDK_ROS_DEVEL_PATH"
export SDK_ROS_INSTALLED_PATH="infrastructure/ros_packages/installed/lib/python3/dist-packages"

# SDK 版本选择配置（留空则使用 sdk_version.env 中的官方配套版本）
export SDK_REPO_BRANCH=""
export SDK_REPO_TAG=""

# Git 版本获取配置
export SDK_VERSION_STRATEGY="$SDK_VERSION_STRATEGY"
export SDK_DEFAULT_VERSION="1.4.4"

# pip 配置
export SDK_BACKUP_PIP_SOURCE=true
export SDK_PIP_MIRROR="$SDK_PIP_MIRROR"

# 调试配置
export SDK_VERBOSE=false

# 高级配置
export SDK_MSG_PACKAGES="kuavo_msgs ocs2_msgs"
export SDK_EXTRAS=""
EOF

chmod +x "$CONFIG_FILE"

echo ""
echo -e "✅ 配置文件已生成: $CONFIG_FILE"
echo ""
echo "📝 下一步："
echo "   1. 确认 ROS 消息包已编译:"
echo "      cd infrastructure/ros_packages"
echo "      catkin build kuavo_msgs ocs2_msgs"
echo ""
echo "   2. 安装 SDK:"
echo "      cd ../.."
echo "      ./scripts/install_sdk.sh"
echo ""
echo "💡 如需修改配置："
echo "   - 直接编辑: vim $CONFIG_FILE"
echo "   - 重新运行: $0"
