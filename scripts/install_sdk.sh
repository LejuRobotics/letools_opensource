#!/bin/bash
# install_sdk.sh - LeTools SDK 一键安装脚本

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/..")
SDK_DIR="$PROJECT_DIR/drivers/leju/kuavo_humanoid_sdk"
TOOLS_DIR="$SCRIPT_DIR/kuavo_humanoid_sdk_tools"
CONFIG_FILE="$TOOLS_DIR/sdk_config.sh"
TEMPLATE_FILE="$TOOLS_DIR/sdk_config.sh.template"
PIN_FILE="$TOOLS_DIR/sdk_version.env"
SUBMODULE_PATH="drivers/leju/kuavo_humanoid_sdk"
MODULE_DIR="$PROJECT_DIR/.git/modules/$SUBMODULE_PATH"
SUBMODULE_URL=$(git config -f "$PROJECT_DIR/.gitmodules" --get submodule."$SUBMODULE_PATH".url 2>/dev/null || echo "https://gitcode.com/OpenLET/kuavo-ros-opensource.git")

echo "=================================================="
echo "  LeTools SDK 一键安装工具"
echo "=================================================="
echo ""

echo "🔧 检查并设置脚本权限..."
chmod +x "$TOOLS_DIR"/*.sh 2>/dev/null || true

if [ ! -f "$PIN_FILE" ]; then
    echo -e "\033[31m❌ 缺少版本锁定清单: $PIN_FILE\033[0m"
    exit 1
fi
source "$PIN_FILE"
PIN_BRANCH="$SDK_REPO_BRANCH"
PIN_TAG="$SDK_REPO_TAG"
if [ -z "$PIN_BRANCH" ] || [ -z "$PIN_TAG" ]; then
    echo -e "\033[31m❌ $PIN_FILE 中 SDK_REPO_BRANCH 与 SDK_REPO_TAG 必须同时设置\033[0m"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "⚙️  未找到配置文件，正在从模板生成..."
    cp "$TEMPLATE_FILE" "$CONFIG_FILE"
    chmod +x "$CONFIG_FILE"
    echo -e "\033[32m✅ 配置文件已生成: $CONFIG_FILE\033[0m"
fi
source "$CONFIG_FILE"

if [ -n "$SDK_REPO_BRANCH" ] && [ -n "$SDK_REPO_TAG" ]; then
    TARGET_BRANCH="$SDK_REPO_BRANCH"
    TARGET_TAG="$SDK_REPO_TAG"
    echo "📌 使用本地覆盖版本：分支 $TARGET_BRANCH / tag $TARGET_TAG（官方配套：$PIN_BRANCH / $PIN_TAG）"
else
    if [ -n "$SDK_REPO_BRANCH" ] || [ -n "$SDK_REPO_TAG" ]; then
        echo -e "\033[33m⚠️  sdk_config.sh 中 SDK_REPO_BRANCH 与 SDK_REPO_TAG 需同时设置才生效，当前只设置了一项，已忽略，使用官方配套版本\033[0m"
    fi
    TARGET_BRANCH="$PIN_BRANCH"
    TARGET_TAG="$PIN_TAG"
    echo "📌 LeTools 配套 SDK 版本：分支 $TARGET_BRANCH / tag $TARGET_TAG"
fi

echo "🔍 校验分支与 tag 的对应关系..."
TAG_COMMIT=$(git ls-remote --tags "$SUBMODULE_URL" "refs/tags/$TARGET_TAG" | awk 'NR==1{print $1}')
BRANCH_COMMIT=$(git ls-remote --heads "$SUBMODULE_URL" "refs/heads/$TARGET_BRANCH" | awk 'NR==1{print $1}')
[ -n "$TAG_COMMIT" ] || { echo -e "\033[31m❌ 远程仓库不存在 tag: $TARGET_TAG\033[0m"; exit 1; }
[ -n "$BRANCH_COMMIT" ] || { echo -e "\033[31m❌ 远程仓库不存在分支: $TARGET_BRANCH\033[0m"; exit 1; }

TMP_CHECK_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_CHECK_DIR"' EXIT
git clone --quiet --filter=blob:none --no-checkout --branch "$TARGET_BRANCH" "$SUBMODULE_URL" "$TMP_CHECK_DIR" 2>/dev/null
git -C "$TMP_CHECK_DIR" fetch --quiet --depth=1 origin "refs/tags/$TARGET_TAG:refs/tags/$TARGET_TAG" 2>/dev/null || true
if ! git -C "$TMP_CHECK_DIR" merge-base --is-ancestor "$TARGET_TAG" "origin/$TARGET_BRANCH" 2>/dev/null; then
    echo -e "\033[31m❌ tag $TARGET_TAG 不在分支 $TARGET_BRANCH 上（或不是该分支的祖先）\033[0m"
    exit 1
fi
echo -e "\033[32m✅ 配对校验通过：tag $TARGET_TAG 属于分支 $TARGET_BRANCH\033[0m"

repair_broken_submodule() {
    echo "🧹 检测到 SDK submodule 元数据损坏，正在自动修复..."
    cd "$PROJECT_DIR" || exit 1
    git submodule deinit -f "$SUBMODULE_PATH" >/dev/null 2>&1 || true
    rm -rf "$SDK_DIR"
    rm -rf "$MODULE_DIR"
    echo "✅ 已清理损坏的 submodule 残留"
}

ensure_submodule_ready() {
    if [ -e "$SDK_DIR/.git" ] && git -C "$SDK_DIR" rev-parse --git-dir >/dev/null 2>&1; then
        return 0
    fi

    if [ -e "$SDK_DIR/.git" ] && ! git -C "$SDK_DIR" rev-parse --git-dir >/dev/null 2>&1; then
        repair_broken_submodule
    fi

    echo "📥 初始化 SDK submodule ($TARGET_BRANCH @ $TARGET_TAG)..."
    cd "$PROJECT_DIR" || exit 1
    git submodule update --init --force "$SUBMODULE_PATH" || return 1

    if [ ! -e "$SDK_DIR/.git" ] || ! git -C "$SDK_DIR" rev-parse --git-dir >/dev/null 2>&1; then
        echo -e "\033[31m❌ submodule 初始化后仍不可用: $SDK_DIR\033[0m"
        return 1
    fi
}

ensure_submodule_ready || exit 1
cd "$SDK_DIR" || exit 1

CURRENT_TAG=$(git describe --tags --exact-match 2>/dev/null || true)
if [ "$CURRENT_TAG" = "$TARGET_TAG" ]; then
    echo "🔒 SDK 已处于配套版本 $TARGET_TAG，跳过更新"
else
    # 切换 tag 前检查本地未提交修改，避免 checkout 静默失败导致版本错乱
    LOCAL_CHANGES=$(git status --porcelain 2>/dev/null)
    if [ -n "$LOCAL_CHANGES" ]; then
        echo -e "\033[31m❌ SDK submodule 存在未提交的本地修改，无法切换到 $TARGET_TAG：\033[0m"
        echo "$LOCAL_CHANGES" | sed 's/^/   /'
        echo ""
        echo "   请先处理后再重新运行本脚本："
        echo "     保留修改:  cd $SDK_DIR && git stash"
        echo "     放弃修改:  cd $SDK_DIR && git checkout -- ."
        echo "   处理完后执行: ./scripts/install_sdk.sh"
        exit 1
    fi

    echo "🔄 更新 SDK 到配套版本 $TARGET_BRANCH @ $TARGET_TAG..."
    git fetch --depth=1 origin "refs/tags/$TARGET_TAG:refs/tags/$TARGET_TAG" || die "拉取 tag 失败: $TARGET_TAG"
    git checkout -B sdk-install "refs/tags/$TARGET_TAG" || die "切换 tag 失败: $TARGET_TAG（当前仍处于 $(git describe --tags --exact-match 2>/dev/null || git rev-parse --short HEAD)，未变更）"
fi

git config core.sparseCheckout true
GIT_DIR=$(git rev-parse --git-dir)
[ -n "$GIT_DIR" ] || { echo -e "\033[31m❌ 无法解析 SDK git 目录\033[0m"; exit 1; }
mkdir -p "$GIT_DIR/info"
echo "src/kuavo_humanoid_sdk/*" > "$GIT_DIR/info/sparse-checkout"
git read-tree -mu HEAD
echo -e "\033[32m✅ Submodule 初始化完成\033[0m"

echo ""
echo "🚀 开始执行 SDK 安装流程..."
echo ""
# 透传配套 tag 给 setup_sdk.sh，使其版本号直接使用干净的 tag（不带 a0 后缀）
if ! SDK_REPO_TAG="$TARGET_TAG" "$TOOLS_DIR/setup_sdk.sh"; then
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
echo "  python3 -c \"from kuavo_humanoid_sdk import KuavoRobot; print('SDK Ready')\""
