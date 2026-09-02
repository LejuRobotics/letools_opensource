#!/bin/bash
# install_local_sdk.sh - 安装本地已拷贝的 Kuavo Humanoid SDK

set -e

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/..")
TOOLS_DIR="$SCRIPT_DIR/kuavo_humanoid_sdk_tools"
CONFIG_FILE="$TOOLS_DIR/sdk_config.sh"
TEMPLATE_FILE="$TOOLS_DIR/sdk_config.sh.template"
SDK_GIT_DIR="$PROJECT_DIR/drivers/leju/kuavo_humanoid_sdk"
TMP_DEVEL_DIR=""
TMP_INSTALL_LOG=""

cleanup_tmp_devel() {
    if [ -n "$TMP_DEVEL_DIR" ] && [ -d "$TMP_DEVEL_DIR" ]; then
        rm -rf "$TMP_DEVEL_DIR"
    fi

    if [ -n "$TMP_INSTALL_LOG" ] && [ -f "$TMP_INSTALL_LOG" ]; then
        rm -f "$TMP_INSTALL_LOG"
    fi
}

trap cleanup_tmp_devel EXIT

echo "=================================================="
echo "  LeTools 本地 SDK 安装工具"
echo "=================================================="
echo ""

echo "🔧 检查并设置脚本权限..."
chmod +x "$TOOLS_DIR"/*.sh 2>/dev/null || true

if [ ! -f "$CONFIG_FILE" ]; then
    echo "⚙️  未找到配置文件，正在从模板生成..."
    cp "$TEMPLATE_FILE" "$CONFIG_FILE"
    chmod +x "$CONFIG_FILE"
    echo -e "\033[32m✅ 配置文件已生成: $CONFIG_FILE\033[0m"
fi
source "$CONFIG_FILE"

if [ -f "$SDK_GIT_DIR/install.sh" ] && [ -d "$SDK_GIT_DIR/kuavo_humanoid_sdk" ]; then
    SDK_ACTUAL_DIR="$SDK_GIT_DIR"
elif [ -f "$SDK_GIT_DIR/src/kuavo_humanoid_sdk/install.sh" ]; then
    SDK_ACTUAL_DIR="$SDK_GIT_DIR/src/kuavo_humanoid_sdk"
else
    echo -e "\033[31m❌ 未找到本地 SDK 安装入口: $SDK_GIT_DIR\033[0m"
    echo "   期望存在以下任一文件："
    echo "     $SDK_GIT_DIR/install.sh"
    echo "     $SDK_GIT_DIR/src/kuavo_humanoid_sdk/install.sh"
    exit 1
fi

DEVEL_DIR="$PROJECT_DIR/$SDK_ROS_DEVEL_PATH"
INSTALLED_DIR="$PROJECT_DIR/$SDK_ROS_INSTALLED_PATH"
BUNDLED_MSG_DIR="$SDK_ACTUAL_DIR/kuavo_humanoid_sdk/msg"

echo ""
echo "📂 路径信息："
echo "  项目根目录: $PROJECT_DIR"
echo "  SDK 源码目录: $SDK_ACTUAL_DIR"
echo "  ROS devel: $DEVEL_DIR"
echo ""

echo "🔍 获取 SDK 版本..."
VERSION=""
if git -C "$SDK_GIT_DIR" rev-parse --git-dir >/dev/null 2>&1; then
    VERSION=$(git -C "$SDK_GIT_DIR" describe --tags --exact-match 2>/dev/null || true)
    if [ -z "$VERSION" ]; then
        VERSION=$(git -C "$SDK_GIT_DIR" describe --tags --always 2>/dev/null || true)
    fi
fi
if [ -z "$VERSION" ]; then
    VERSION="${SDK_DEFAULT_VERSION:-0.0.0}"
    echo -e "\033[33m⚠️  无法从本地仓库获取版本，使用默认: $VERSION\033[0m"
else
    echo -e "\033[32m✅ 从本地 SDK 获取版本: $VERSION\033[0m"
fi

BRANCH=$(git -C "$SDK_GIT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "local")
VERSION_FORMATTED=$(echo "$VERSION" | sed 's/-g[0-9a-f]\+//')
if [[ "$VERSION" =~ ^[0-9a-fA-F]{7,40}$ ]]; then
    # A shallow clone without tags produces a bare commit hash.  A Python
    # distribution version must be PEP 440 compliant, so keep the hash as a
    # local version identifier rather than passing it to pip verbatim.
    VERSION_FORMATTED="${SDK_DEFAULT_VERSION:-0.0.0}+g${VERSION,,}"
elif [ "$BRANCH" == "beta" ]; then
    VERSION_FORMATTED=$(echo "$VERSION_FORMATTED" | sed 's/-/b/g')
    if [[ ! "$VERSION_FORMATTED" == *"b"* ]]; then
        VERSION_FORMATTED="${VERSION_FORMATTED}b0"
    fi
elif [ "$BRANCH" == "master" ]; then
    VERSION_FORMATTED=$(echo "$VERSION_FORMATTED" | sed 's/-/.post/g')
elif [ "$BRANCH" == "HEAD" ] && [[ "$VERSION" =~ ^[0-9] ]]; then
    VERSION_FORMATTED="$VERSION"
fi

if ! python3 -c 'from packaging.version import Version; import sys; Version(sys.argv[1])' "$VERSION_FORMATTED"; then
    echo -e "\033[33m⚠️  无法将 SDK 版本转换为 PEP 440 格式，回退到默认版本\033[0m"
    VERSION_FORMATTED="${SDK_DEFAULT_VERSION:-0.0.0}"
fi

echo -e "\033[32m📦 最终版本: $VERSION_FORMATTED (分支: $BRANCH)\033[0m"
echo ""

echo "🔍 验证 ROS 消息包..."
IFS=' ' read -r -a MSG_ARRAY <<< "$SDK_MSG_PACKAGES"
MISSING_MSG_PACKAGES=()
for msg_pkg in "${MSG_ARRAY[@]}"; do
    if [ -d "$DEVEL_DIR/.private/$msg_pkg/lib/python3/dist-packages" ]; then
        MSG_SRC_DIR="$DEVEL_DIR/.private/$msg_pkg/lib/python3/dist-packages"
    else
        MSG_SRC_DIR="$DEVEL_DIR/lib/python3/dist-packages"
    fi

    if [ -d "$MSG_SRC_DIR/$msg_pkg" ]; then
        echo -e "  ✅ $msg_pkg: 已找到"
    else
        MISSING_MSG_PACKAGES+=("$msg_pkg")
    fi
done

if [ "${#MISSING_MSG_PACKAGES[@]}" -gt 0 ]; then
    if [ -d "$BUNDLED_MSG_DIR" ]; then
        TMP_DEVEL_DIR=$(mktemp -d)
        TMP_DIST_PACKAGES="$TMP_DEVEL_DIR/lib/python3/dist-packages"
        mkdir -p "$TMP_DIST_PACKAGES"

        for msg_pkg in "${MISSING_MSG_PACKAGES[@]}"; do
            if [ -d "$BUNDLED_MSG_DIR/$msg_pkg" ]; then
                cp -r "$BUNDLED_MSG_DIR/$msg_pkg" "$TMP_DIST_PACKAGES/"
                echo -e "  \033[33m⚠️  $msg_pkg 未在 ROS devel 中找到，改用 SDK 自带消息定义\033[0m"
            else
                echo -e "  \033[31m❌ $msg_pkg: 未找到\033[0m"
                echo ""
                echo "请先编译 ROS 消息包："
                echo "  cd $PROJECT_DIR/$(dirname "$SDK_ROS_DEVEL_PATH")"
                echo "  catkin build $msg_pkg"
                exit 1
            fi
        done

        DEVEL_DIR="$TMP_DEVEL_DIR"
        echo -e "\033[32m✅ 已使用 SDK 自带消息包构建临时 devel 目录: $DEVEL_DIR\033[0m"
    else
        for msg_pkg in "${MISSING_MSG_PACKAGES[@]}"; do
            echo -e "  \033[31m❌ $msg_pkg: 未找到\033[0m"
        done
        exit 1
    fi
fi

echo ""
echo "🚀 开始安装本地 SDK..."
echo ""

cd "$SDK_ACTUAL_DIR"
cp install.sh install.sh.bak

sed -i "s|^SDK_PROJECT_DIR=.*|SDK_PROJECT_DIR=\"$SDK_ACTUAL_DIR\"|" install.sh
sed -i "s|^DEVEL_DIR=.*|DEVEL_DIR=\"$DEVEL_DIR\"|" install.sh
sed -i "s|^INSTALLED_DIR=.*|INSTALLED_DIR=\"$INSTALLED_DIR\"|" install.sh
sed -i "s|^BRANCH=.*|BRANCH=\"$BRANCH\"|" install.sh
sed -i "s|^VERSION=.*|VERSION=\"$VERSION_FORMATTED\"|" install.sh
sed -i 's/^MSG_PACKAGES=.*/MSG_PACKAGES="kuavo_msgs ocs2_msgs"/' install.sh
sed -i 's|^    echo -e "\\033\[33mWarning: VERSION format is invalid, attempting to get version from git...\\033\[0m"$|    :|' install.sh
sed -i 's|^    get_version_from_git VERSION$|    :|' install.sh
sed -i "s|^check_and_format_version \"\\\$BRANCH\" VERSION|VERSION=\"$VERSION_FORMATTED\"|" install.sh

EXTRAS_ARG=""
if [ -n "$SDK_EXTRAS" ]; then
    EXTRAS_ARG="--extras $SDK_EXTRAS"
fi

TMP_INSTALL_LOG=$(mktemp)
if KUAVO_HUMANOID_SDK_VERSION="$VERSION_FORMATTED" ./install.sh $EXTRAS_ARG | tee "$TMP_INSTALL_LOG"; then
    if ! grep -q "Installation successful" "$TMP_INSTALL_LOG"; then
        mv install.sh.bak install.sh
        echo -e "\033[31m❌ 本地 SDK 安装失败（pip 未报告安装成功）\033[0m"
        exit 1
    fi
    mv install.sh.bak install.sh
    echo ""
    echo "=================================================="
    echo "  🎉 本地 SDK 安装完成！"
    echo "=================================================="
    echo ""
    echo "💡 验证安装："
    echo "  python3 -c \"from kuavo_humanoid_sdk import KuavoRobot; print('SDK Ready')\""
else
    mv install.sh.bak install.sh
    echo -e "\033[31m❌ 本地 SDK 安装失败\033[0m"
    exit 1
fi
