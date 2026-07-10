#!/bin/bash
# setup_sdk.sh - Kuavo Humanoid SDK 安装包装脚本 (LeTools)
# 
# 功能：
# 1. 加载配置
# 2. 计算路径（适配 sparse checkout 结构）
# 3. 从上游 submodule 获取版本号
# 4. 验证环境
# 5. 调用原始 install.sh

set -e

SCRIPT_DIR=$(dirname "$(realpath "$0")")

# ============================================
# 加载配置
# ============================================

CONFIG_FILE="$SCRIPT_DIR/sdk_config.sh"
TEMPLATE_FILE="$SCRIPT_DIR/sdk_config.sh.template"

if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    echo -e "\033[32m✅ 已加载配置文件: sdk_config.sh\033[0m"
elif [ -f "$TEMPLATE_FILE" ]; then
    echo -e "\033[33m⚠️  未找到配置文件，请先运行初始化向导...\033[0m"
    echo "正在启动配置向导..."
    "$SCRIPT_DIR/init_config.sh"
    source "$CONFIG_FILE"
else
    echo -e "\033[31m❌ 未找到配置文件或模板\033[0m"
    exit 1
fi

# ============================================
# 计算路径
# ============================================

# 向上回溯两层到达项目根目录 (scripts/kuavo_humanoid_sdk_tools -> scripts -> LeTools)
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../..")

# SDK 实际目录（sparse checkout 结构）
SDK_ACTUAL_DIR="$PROJECT_DIR/drivers/leju/kuavo_humanoid_sdk/src/kuavo_humanoid_sdk"

# ROS devel 目录
DEVEL_DIR="$PROJECT_DIR/$SDK_ROS_DEVEL_PATH"

echo ""
echo "📂 路径信息："
echo "  项目根目录: $PROJECT_DIR"
echo "  SDK 源码目录: $SDK_ACTUAL_DIR"
echo "  ROS devel: $DEVEL_DIR"
echo ""

# 验证 SDK 目录是否存在
if [ ! -d "$SDK_ACTUAL_DIR" ]; then
    echo -e "\033[31m❌ SDK 目录不存在: $SDK_ACTUAL_DIR\033[0m"
    echo "请确认 submodule 已初始化且 sparse checkout 配置正确。"
    exit 1
fi

# ============================================
# 获取版本号（从上游 submodule）
# ============================================

echo "🔍 获取 SDK 版本..."

VERSION=""
SDK_GIT_DIR="$PROJECT_DIR/drivers/leju/kuavo_humanoid_sdk"

if [ "$SDK_VERSION_STRATEGY" = "upstream" ]; then
    # 从上游 submodule 获取
    if git -C "$SDK_GIT_DIR" rev-parse --git-dir &>/dev/null; then
        VERSION=$(git -C "$SDK_GIT_DIR" describe --tags --always 2>/dev/null)
        if [ -n "$VERSION" ]; then
            echo -e "\033[32m✅ 从上游 submodule 获取版本: $VERSION\033[0m"
        fi
    fi
fi

if [ -z "$VERSION" ]; then
    VERSION="$SDK_DEFAULT_VERSION"
    echo -e "\033[33m⚠️  无法从上游获取版本，使用默认: $VERSION\033[0m"
fi

# 获取分支信息
BRANCH=$(git -C "$SDK_GIT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")

# 格式化版本（与原 install.sh 逻辑一致）
# 移除 hash 后缀
VERSION_FORMATTED=$(echo "$VERSION" | sed 's/-g[0-9a-f]\+//')

if [ "$BRANCH" == "beta" ]; then
    VERSION_FORMATTED=$(echo "$VERSION_FORMATTED" | sed 's/-/b/g')
    if [[ ! "$VERSION_FORMATTED" == *"b"* ]]; then
        VERSION_FORMATTED="${VERSION_FORMATTED}b0"
    fi
elif [ "$BRANCH" == "master" ]; then
    VERSION_FORMATTED=$(echo "$VERSION_FORMATTED" | sed 's/-/.post/g')
else
    VERSION_FORMATTED=$(echo "$VERSION_FORMATTED" | sed 's/-/a/g')
    if [[ ! "$VERSION_FORMATTED" == *"a"* ]]; then
        VERSION_FORMATTED="${VERSION_FORMATTED}a0"
    fi
fi

echo -e "\033[32m📦 最终版本: $VERSION_FORMATTED (分支: $BRANCH)\033[0m"
echo ""

# ============================================
# 验证 ROS 消息包
# ============================================

echo "🔍 验证 ROS 消息包..."

IFS=' ' read -r -a MSG_ARRAY <<< "$SDK_MSG_PACKAGES"
for msg_pkg in "${MSG_ARRAY[@]}"; do
    if [ -d "$DEVEL_DIR/.private/$msg_pkg/lib/python3/dist-packages" ]; then
        MSG_SRC_DIR="$DEVEL_DIR/.private/$msg_pkg/lib/python3/dist-packages"
    else
        MSG_SRC_DIR="$DEVEL_DIR/lib/python3/dist-packages"
    fi
    
    if [ -d "$MSG_SRC_DIR/$msg_pkg" ]; then
        echo -e "  ✅ $msg_pkg: 已找到"
    else
        echo -e "  \033[31m❌ $msg_pkg: 未找到\033[0m"
        echo ""
        echo "请先编译 ROS 消息包："
        echo "  cd $PROJECT_DIR/$(dirname $SDK_ROS_DEVEL_PATH)"
        echo "  catkin build $msg_pkg"
        exit 1
    fi
done

echo ""

# ============================================
# 调用原始 install.sh
# ============================================

echo "🚀 开始安装 SDK..."
echo ""

# 进入 SDK 实际目录并执行安装
cd "$SDK_ACTUAL_DIR"

# 【关键修复】：备份并修改 install.sh 以适应当前环境
cp install.sh install.sh.bak

# 1. 强制设置 DEVEL_DIR
sed -i "s|^DEVEL_DIR=.*|DEVEL_DIR=\"$DEVEL_DIR\"|" install.sh

# 2. 强制修改 MSG_PACKAGES (只包含基础包)
sed -i 's/^MSG_PACKAGES=.*/MSG_PACKAGES="kuavo_msgs ocs2_msgs"/' install.sh

# 构建 extras 参数
EXTRAS_ARG=""
if [ -n "$SDK_EXTRAS" ]; then
    EXTRAS_ARG="--extras $SDK_EXTRAS"
fi

# 执行安装
if KUAVO_HUMANOID_SDK_VERSION="$VERSION_FORMATTED" ./install.sh $EXTRAS_ARG; then
    echo ""
    echo -e "\033[32m==================================================\033[0m"
    echo -e "\033[32m🎉 SDK 安装成功！\033[0m"
    echo -e "\033[32m==================================================\033[0m"
    echo ""
    echo "💡 使用方式："
    echo "  from kuavo_humanoid_sdk import KuavoRobot"
else
    echo -e "\033[31m❌ SDK 安装失败\033[0m"
    mv install.sh.bak install.sh
    exit 1
fi

# 恢复原始 install.sh
mv install.sh.bak install.sh
