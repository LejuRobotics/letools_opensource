#!/bin/bash
# setup_letools_learning.sh - 一键添加/初始化 LeTools-Learning 子模块
#
# 用法:
#   ./scripts/setup_letools_learning.sh            # 添加并初始化子模块
#   ./scripts/setup_letools_learning.sh --update   # 更新子模块到 main 最新
#
# 幂等:已配置则初始化/更新,未配置则添加。

set -e

# ============================================
# 配置
# ============================================

SUBMODULE_URL="https://github.com/LejuRobotics/LeTools-Learning"
SUBMODULE_NAME="LeTools-Learning"
SUBMODULE_BRANCH="main"

# ============================================
# 路径自适应:脚本在 scripts/ 下,项目根在上一级
# ============================================

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/..")

cd "$PROJECT_DIR"

# 颜色
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
NC='\033[0m'

ok()   { echo -e "${GREEN}✅ $1${NC}"; }
warn() { echo -e "${YELLOW}⚠️  $1${NC}"; }
err()  { echo -e "${RED}❌ $1${NC}"; }

# ============================================
# 前置检查
# ============================================

if ! git rev-parse --git-dir &>/dev/null; then
    err "当前目录不是 git 仓库: $PROJECT_DIR"
    exit 1
fi

UPDATE_MODE=false
if [ "$1" = "--update" ]; then
    UPDATE_MODE=true
fi

# ============================================
# 状态判断
# ============================================

has_gitmodules_entry() {
    git config -f .gitmodules --get "submodule.${SUBMODULE_NAME}.url" &>/dev/null
}

is_registered_gitlink() {
    # 暂存区/树里是否有该路径的 gitlink (mode 160000)
    git ls-files --stage "$SUBMODULE_NAME" 2>/dev/null | grep -q '160000'
}

dir_is_submodule() {
    git -C "$SUBMODULE_NAME" rev-parse --git-dir &>/dev/null
}

echo "📂 项目根目录: $PROJECT_DIR"
echo "   子模块: $SUBMODULE_NAME"
echo "   远端:   $SUBMODULE_URL (分支: $SUBMODULE_BRANCH)"
echo ""

# ============================================
# 残留清理:目录在、但既没 .gitmodules 条目也没 gitlink
# ============================================

if [ -d "$SUBMODULE_NAME" ] && ! has_gitmodules_entry && ! is_registered_gitlink; then
    warn "检测到 $SUBMODULE_NAME 目录残留但未注册为子模块,正在清理..."
    rm -rf "$SUBMODULE_NAME"
    rm -rf "$(git rev-parse --git-dir)/modules/$SUBMODULE_NAME" 2>/dev/null || true
    ok "残留已清理"
    echo ""
fi

# ============================================
# 主流程
# ============================================

if ! has_gitmodules_entry; then
    # 情况 A:全新添加
    echo "🚀 添加子模块..."
    git submodule add "$SUBMODULE_URL" "$SUBMODULE_NAME"
    git config -f .gitmodules "submodule.${SUBMODULE_NAME}.branch" "$SUBMODULE_BRANCH"
    git add .gitmodules
    ok "子模块已添加并跟踪 $SUBMODULE_BRANCH 分支"
    echo ""
    echo "💡 接下来请提交:"
    echo "   git commit -m 'feat: 添加 $SUBMODULE_NAME 子模块'"
elif $UPDATE_MODE; then
    # 情况 B:已配置,更新到最新
    echo "🔄 更新子模块到 $SUBMODULE_BRANCH 最新..."
    git submodule update --remote "$SUBMODULE_NAME"
    ok "子模块已更新,请提交新 commit:"
    echo "   git add $SUBMODULE_NAME"
    echo "   git commit -m 'chore: bump $SUBMODULE_NAME'"
else
    # 情况 C:已配置,确保已初始化
    echo "📥 初始化子模块..."
    git submodule update --init --recursive "$SUBMODULE_NAME"
    ok "子模块已就绪"
fi

echo ""

# ============================================
# 验证
# ============================================

echo "🔍 验证..."
if dir_is_submodule && has_gitmodules_entry; then
    COMMIT=$(git -C "$SUBMODULE_NAME" rev-parse --short HEAD 2>/dev/null || echo "unknown")
    BRANCH=$(git -C "$SUBMODULE_NAME" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "detached")
    ok "$SUBMODULE_NAME 就绪 (commit: $COMMIT, HEAD: $BRANCH)"
else
    err "$SUBMODULE_NAME 验证失败,请检查"
    exit 1
fi
