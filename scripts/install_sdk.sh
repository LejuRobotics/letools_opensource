#!/bin/bash
# install_sdk.sh - LeTools SDK 一键安装脚本

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/..")
SDK_DIR="$PROJECT_DIR/drivers/leju/kuavo_humanoid_sdk"
TOOLS_DIR="$SCRIPT_DIR/kuavo_humanoid_sdk_tools"
STALE_MODULE_DIR="$PROJECT_DIR/.git/modules/drivers/leju/kuavo_humanoid_sdk"

echo "=================================================="
echo "  LeTools SDK 一键安装工具"
echo "=================================================="
echo ""

# 1. 自动赋予执行权限
echo "🔧 检查并设置脚本权限..."
chmod +x "$TOOLS_DIR"/*.sh 2>/dev/null || true

# 2. 检查并初始化 Submodule
if [ ! -d "$SDK_DIR/.git" ]; then
    echo "📥 检测到 SDK submodule 未初始化，正在初始化 (dev 分支)..."
    if [ -d "$STALE_MODULE_DIR" ]; then
        echo "🧹 清理残留的 submodule 数据..."
        rm -rf "$STALE_MODULE_DIR"
    fi
    cd "$PROJECT_DIR"
    git submodule update --init --depth=1 drivers/leju/kuavo_humanoid_sdk
    
    cd "$SDK_DIR"
    git fetch --depth=1 origin dev
    git checkout -B dev FETCH_HEAD
    git branch -D HEAD 2>/dev/null || true
    git config core.sparseCheckout true
    GIT_DIR=$(git rev-parse --git-dir)
    mkdir -p "$GIT_DIR/info"
    echo "src/kuavo_humanoid_sdk/*" > "$GIT_DIR/info/sparse-checkout"
    git read-tree -mu HEAD
    echo -e "\033[32m✅ Submodule 初始化完成\033[0m"
else
    cd "$SDK_DIR"
    echo "🔄 更新 SDK 到最新 dev 分支..."
    git fetch origin dev
    git checkout -B dev FETCH_HEAD
    git read-tree -mu HEAD
fi

# 3. 自动生成/更新配置文件
CONFIG_FILE="$TOOLS_DIR/sdk_config.sh"
TEMPLATE_FILE="$TOOLS_DIR/sdk_config.sh.template"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "⚙️  未找到配置文件，正在从模板生成..."
    cp "$TEMPLATE_FILE" "$CONFIG_FILE"
    chmod +x "$CONFIG_FILE"
    echo -e "\033[32m✅ 配置文件已生成: $CONFIG_FILE\033[0m"
fi

# 4. 调用核心安装脚本
echo ""
echo "🚀 开始执行 SDK 安装流程..."
echo ""
if ! "$TOOLS_DIR/setup_sdk.sh"; then
    echo ""
    echo -e "\033[33m⚠️  SDK 安装未完全成功，请按上方提示解决依赖问题后重新运行此脚本。\033[0m"
    exit 1
fi

echo ""
echo "=================================================="
echo "  🎉 安装全部完成！"
echo "=================================================="
echo ""
echo "💡 验证安装："
echo "  python3 -c \"from kuavo_humanoid_sdk import KuavoRobot; print('SDK Ready!')\""
