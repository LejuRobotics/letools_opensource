#!/bin/bash
# common.sh - sdk_tools 公共函数库。由同目录脚本 source，不单独执行。
#
# 约定：
#   - 版本权威源是 sdk_version.env（配套分支 + tag）；本地覆盖写在 sdk_config.sh。
#   - SDK 源码位于 drivers/leju/kuavo_humanoid_sdk（git submodule），只检出
#     src/kuavo_humanoid_sdk 子目录，只下载该目录的文件（约 10MB，而非整仓 3.5GB）。
#   - 不调用、不修改上游的 install.sh：消息包拷贝与 pip 安装由本库直接完成。

SDK_TOOLS_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
PROJECT_DIR=$(realpath "$SDK_TOOLS_DIR/../..")
SUBMODULE_PATH="drivers/leju/kuavo_humanoid_sdk"
SDK_DIR="$PROJECT_DIR/$SUBMODULE_PATH"
SDK_SUBDIR="src/kuavo_humanoid_sdk"          # 上游仓库内 SDK 包所在子目录
SDK_DIST_NAME="kuavo-humanoid-sdk"           # pip 包名
SDK_IMPORT_NAME="kuavo_humanoid_sdk"         # import 名
SDK_CONFLICT_DIST="kuavo-humanoid-sdk-ws"    # 与本 SDK 冲突的包
DEFAULT_SDK_URL="https://gitcode.com/OpenLET/kuavo-ros-opensource.git"
CONFIG_FILE="$SDK_TOOLS_DIR/sdk_config.sh"

PIN_FILE="$SDK_TOOLS_DIR/sdk_version.env"    # 版本锁定清单（入库，随 LeTools 发版）

# ---------------------------------------------------------------- 输出

if [ -t 1 ]; then
    C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'; C_OFF=$'\033[0m'
else
    C_RED=""; C_GREEN=""; C_YELLOW=""; C_OFF=""
fi

info() { echo "$*"; }
ok()   { echo "${C_GREEN}✅ $*${C_OFF}"; }
warn() { echo "${C_YELLOW}⚠️  $*${C_OFF}" >&2; }
die()  { echo "${C_RED}❌ $*${C_OFF}" >&2; exit 1; }

# confirm <提示> —— ASSUME_YES=true 时直接通过；非交互终端且未加 --yes 时拒绝。
confirm() {
    [ "${ASSUME_YES:-false}" = true ] && return 0
    [ -t 0 ] || die "$1（非交互环境请加 --yes）"
    local reply
    read -r -p "$1 [y/N]: " reply
    [[ "$reply" =~ ^[Yy]$ ]]
}

# ---------------------------------------------------------------- 退出清理

_CLEANUP_DIRS=()
_PROMISOR_ON=false

_cleanup() {
    local d
    for d in "${_CLEANUP_DIRS[@]}"; do rm -rf "$d"; done
    [ "$_PROMISOR_ON" = true ] && promisor_off
    return 0
}
trap _cleanup EXIT

# ---------------------------------------------------------------- 配置与版本

# read_env_value <文件> <KEY> —— 只解析 KEY="VALUE" 行，不执行文件内容。
read_env_value() {
    sed -n -E "s/^(export[[:space:]]+)?$2=\"?([^\"#[:space:]]*)\"?.*$/\2/p" "$1" | tail -n 1
}

load_config() {
    # 默认值；sdk_config.sh 里只需写要改的项（见 sdk_config.sh.example）
    SDK_REPO_BRANCH=""
    SDK_REPO_TAG=""
    SDK_ROS_DEVEL_PATH="infrastructure/ros_packages/devel"
    SDK_MSG_PACKAGES="kuavo_msgs ocs2_msgs"
    SDK_PIP_MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple/"
    SDK_PYTHON="python3"
    SDK_EXTRAS=""
    if [ -f "$CONFIG_FILE" ]; then
        # shellcheck disable=SC1090
        source "$CONFIG_FILE"
    fi

    [ -f "$PIN_FILE" ] || die "缺少版本锁定清单: $PIN_FILE"
    PIN_BRANCH=$(read_env_value "$PIN_FILE" SDK_REPO_BRANCH)
    PIN_TAG=$(read_env_value "$PIN_FILE" SDK_REPO_TAG)
    [ -n "$PIN_BRANCH" ] && [ -n "$PIN_TAG" ] ||
        die "$PIN_FILE 中 SDK_REPO_BRANCH 与 SDK_REPO_TAG 必须同时设置"

    case "$SDK_ROS_DEVEL_PATH" in
        /*) DEVEL_DIR="$SDK_ROS_DEVEL_PATH" ;;
        *)  DEVEL_DIR="$PROJECT_DIR/$SDK_ROS_DEVEL_PATH" ;;
    esac
}

# resolve_target [命令行tag] [命令行branch] —— 优先级：命令行 > sdk_config.sh > sdk_version.env
resolve_target() {
    local cli_tag="${1:-}" cli_branch="${2:-}"
    if [ -n "$cli_tag" ]; then
        TARGET_TAG="$cli_tag"; TARGET_BRANCH="${cli_branch:-$PIN_BRANCH}"; TARGET_SOURCE="命令行指定"
    elif [ -n "$cli_branch" ]; then
        die "--branch 需要与 --tag 一起使用"
    elif [ -n "$SDK_REPO_BRANCH" ] && [ -n "$SDK_REPO_TAG" ]; then
        TARGET_TAG="$SDK_REPO_TAG"; TARGET_BRANCH="$SDK_REPO_BRANCH"; TARGET_SOURCE="本地覆盖 sdk_config.sh"
    else
        if [ -n "$SDK_REPO_BRANCH" ] || [ -n "$SDK_REPO_TAG" ]; then
            warn "sdk_config.sh 中 SDK_REPO_BRANCH 与 SDK_REPO_TAG 需同时设置才生效，已忽略，使用配套版本"
        fi
        TARGET_TAG="$PIN_TAG"; TARGET_BRANCH="$PIN_BRANCH"; TARGET_SOURCE="配套版本 sdk_version.env"
    fi
}

# set_config_override <branch> <tag> —— 两者都为空表示清除覆盖
set_config_override() {
    local key value
    [ -f "$CONFIG_FILE" ] || echo "# sdk_tools 本地配置（不入库）。可用项见 sdk_config.sh.example" > "$CONFIG_FILE"
    for key in SDK_REPO_BRANCH SDK_REPO_TAG; do
        [ "$key" = SDK_REPO_BRANCH ] && value="$1" || value="$2"
        if grep -q -E "^(export[[:space:]]+)?$key=" "$CONFIG_FILE"; then
            sed -i -E "s|^(export[[:space:]]+)?$key=.*|$key=\"$value\"|" "$CONFIG_FILE"
        else
            echo "$key=\"$value\"" >> "$CONFIG_FILE"
        fi
    done
}

# pep440_version <tag 或 git describe 输出> —— 转成 pip 可接受的版本号
pep440_version() {
    local v="${1#v}"
    if [[ "$v" =~ ^[0-9]+(\.[0-9]+)*((a|b|rc|\.post|\.dev)[0-9]+)?(\+[A-Za-z0-9.]+)?$ ]]; then
        echo "$v"
    elif [[ "$v" =~ ^([0-9]+(\.[0-9]+)*)-([0-9]+)-g([0-9a-f]+)$ ]]; then
        echo "${BASH_REMATCH[1]}.post${BASH_REMATCH[3]}+g${BASH_REMATCH[4]}"   # tag 之后又有提交
    elif [[ "$v" =~ ^[0-9a-f]{7,40}$ ]]; then
        echo "0.0.0+g$v"                                                       # 只有 commit 哈希
    else
        echo "0.0.0+$(echo "$v" | tr -c 'A-Za-z0-9\n' '.' | sed -E 's/\.+/./g; s/^\.//; s/\.$//')"
    fi
}

# ---------------------------------------------------------------- git：仓库定位

project_is_git() {
    [ "$(git -C "$PROJECT_DIR" rev-parse --show-toplevel 2>/dev/null)" = "$PROJECT_DIR" ]
}

# SDK 目录本身是一个 git 仓库（而不是被解析到父仓库 LeTools）
sdk_is_own_repo() {
    local top
    top=$(git -C "$SDK_DIR" rev-parse --show-toplevel 2>/dev/null) || return 1
    [ "$(realpath "$top")" = "$SDK_DIR" ]
}

# submodule 仓库数据所在目录（.git/modules/...）；LeTools 不是 git 仓库时输出为空
module_dir() {
    project_is_git || return 0
    local common
    common=$(git -C "$PROJECT_DIR" rev-parse --git-common-dir)
    case "$common" in /*) ;; *) common="$PROJECT_DIR/$common" ;; esac
    realpath -m "$common/modules/$SUBMODULE_PATH"
}

sdk_remote_url() {
    git -C "$PROJECT_DIR" config --get "submodule.$SUBMODULE_PATH.url" 2>/dev/null ||
        git config -f "$PROJECT_DIR/.gitmodules" --get "submodule.$SUBMODULE_PATH.url" 2>/dev/null ||
        echo "$DEFAULT_SDK_URL"
}

git_sdk() { git -C "$SDK_DIR" "$@"; }

# ---------------------------------------------------------------- git：远程校验

# verify_pairing —— 校验 tag、分支存在且 tag 属于该分支；成功后设置 TARGET_COMMIT
verify_pairing() {
    local url refs branch_commit tmp
    url=$(sdk_remote_url)
    refs=$(git ls-remote "$url" "refs/tags/$TARGET_TAG*" "refs/heads/$TARGET_BRANCH" 2>/dev/null) ||
        die "无法访问 SDK 远程仓库: $url（无网络时可用 install_local.sh 安装本地源码）"
    # 附注 tag 取其指向的 commit（^{} 行），轻量 tag 直接取本行
    TARGET_COMMIT=$(awk -v r="refs/tags/$TARGET_TAG^{}" '$2==r{print $1}' <<<"$refs")
    [ -n "$TARGET_COMMIT" ] || TARGET_COMMIT=$(awk -v r="refs/tags/$TARGET_TAG" '$2==r{print $1}' <<<"$refs")
    branch_commit=$(awk -v r="refs/heads/$TARGET_BRANCH" '$2==r{print $1}' <<<"$refs")
    [ -n "$TARGET_COMMIT" ] || die "远程仓库不存在 tag: $TARGET_TAG（可用 version.sh list 查看）"
    [ -n "$branch_commit" ] || die "远程仓库不存在分支: $TARGET_BRANCH"

    # 祖先判断只需 commit 图：--filter=tree:0 不取任何目录和文件
    tmp=$(mktemp -d); _CLEANUP_DIRS+=("$tmp")
    git clone --quiet --filter=tree:0 --single-branch --no-checkout --branch "$TARGET_BRANCH" "$url" "$tmp" 2>/dev/null ||
        die "拉取分支 $TARGET_BRANCH 的提交历史失败: $url"
    git -C "$tmp" rev-parse -q --verify "refs/tags/$TARGET_TAG^{commit}" >/dev/null 2>&1 ||
        git -C "$tmp" fetch --quiet --depth=1 origin "refs/tags/$TARGET_TAG:refs/tags/$TARGET_TAG" 2>/dev/null || true
    git -C "$tmp" merge-base --is-ancestor "refs/tags/$TARGET_TAG" "origin/$TARGET_BRANCH" 2>/dev/null ||
        die "tag $TARGET_TAG 不在分支 $TARGET_BRANCH 上（或不是该分支的祖先）"
}

# ---------------------------------------------------------------- git：精简检出

# 按需下载开关。只在本工具拉取/检出期间打开，结束后关闭：
# 浅克隆里唯一的提交看起来像“新增了全部文件”，开着的话 git show / git log -p / IDE 的
# 提交视图等任何查看该提交内容的操作，都会把整仓约 1.3GB 的文件按需下载下来。
promisor_on() {
    git_sdk config remote.origin.promisor true
    git_sdk config remote.origin.partialclonefilter blob:none
    _PROMISOR_ON=true
}

promisor_off() {
    git_sdk config --unset remote.origin.promisor 2>/dev/null || true
    git_sdk config --unset remote.origin.partialclonefilter 2>/dev/null || true
    _PROMISOR_ON=false
}

# ensure_source_repo —— 保证 SDK 目录是可用的 git 仓库，尽量复用本地已有数据。
# 若工作区是新建/重建的（内容需要从仓库恢复），设置 _WORKTREE_NEW=true。
ensure_source_repo() {
    local module url
    _WORKTREE_NEW=false
    sdk_is_own_repo && return 0
    module=$(module_dir)

    if [ -e "$SDK_DIR/.git" ]; then
        # .git 残留但指向的仓库数据已不可用
        info "🧹 SDK 目录的 git 元数据已损坏，清理后重建..."
        rm -rf "$SDK_DIR"
    elif [ -d "$SDK_DIR" ] && [ -n "$(ls -A "$SDK_DIR")" ]; then
        die "$SDK_DIR 非空且不是 git 仓库（可能是手动拷贝的 SDK 源码）。
   安装这份本地源码:  scripts/sdk_tools/install_local.sh
   改为在线安装:      先执行 scripts/sdk_tools/remove.sh --source"
    fi
    mkdir -p "$SDK_DIR"
    _WORKTREE_NEW=true

    if [ -n "$module" ] && [ -f "$module/HEAD" ]; then
        # 仓库数据还在（如 remove.sh --source 之后）：重新挂上工作区即可，不重新下载
        info "📥 复用本地已有的 SDK 仓库数据..."
        git -C "$PROJECT_DIR" submodule --quiet init "$SUBMODULE_PATH"
        echo "gitdir: $(realpath --relative-to="$SDK_DIR" "$module")" > "$SDK_DIR/.git"
        git --git-dir="$module" config core.worktree "$(realpath --relative-to="$module" "$SDK_DIR")"
    else
        info "📥 初始化 SDK 仓库（只下载 SDK 目录，不拉取整仓）..."
        url=$(sdk_remote_url)
        git init --quiet "$SDK_DIR"
        git_sdk remote add origin "$url"
        if project_is_git; then
            rm -rf "$module"   # 不完整的残留（如被中断的 clone），否则无法迁入
            git -C "$PROJECT_DIR" submodule --quiet init "$SUBMODULE_PATH"
            git -C "$PROJECT_DIR" submodule absorbgitdirs "$SUBMODULE_PATH" >/dev/null 2>&1 ||
                die "无法把 SDK 仓库登记为 submodule: $SUBMODULE_PATH"
        fi
    fi
    sdk_is_own_repo || die "SDK 仓库初始化后仍不可用: $SDK_DIR"
}

# sync_source —— 把 SDK 源码切到 TARGET_TAG（需先 verify_pairing 得到 TARGET_COMMIT）
sync_source() {
    local git_dir head changes
    ensure_source_repo

    # sparse 必须先于 checkout 生效，否则会检出并下载整仓。
    # 根目录的 .gitignore 一并检出：__pycache__ 等忽略规则在里面，缺了它
    # import 之后 git status 会把 SDK 子模块显示为“有未跟踪内容”。
    git_dir=$(git_sdk rev-parse --absolute-git-dir)
    git_sdk config core.sparseCheckout true
    mkdir -p "$git_dir/info"
    printf '%s\n' "/.gitignore" "$SDK_SUBDIR/*" > "$git_dir/info/sparse-checkout"

    promisor_on
    head=$(git_sdk rev-parse -q --verify "HEAD^{commit}" 2>/dev/null || true)
    if [ "$head" = "$TARGET_COMMIT" ]; then
        info "🔒 SDK 源码已是 $TARGET_TAG"
    else
        # 有本地修改时不强切，避免覆盖用户改动或切到一半导致版本错乱
        changes=$(git_sdk status --porcelain --untracked-files=no 2>/dev/null || true)
        if [ -n "$head" ] && [ "$_WORKTREE_NEW" != true ] && [ -n "$changes" ]; then
            echo "$changes" | sed 's/^/   /' >&2
            die "SDK 源码有未提交的本地修改，无法切换到 $TARGET_TAG。请先处理：
   保留修改:  git -C $SDK_DIR stash
   放弃修改:  git -C $SDK_DIR checkout -- ."
        fi
        info "🔄 获取 SDK $TARGET_TAG ..."
        # --no-tags：否则 git 会顺带把其它 tag 的目录树也拉下来（实测 0.8MB → 9.6MB）
        git_sdk fetch --quiet --depth=1 --filter=blob:none --no-tags origin "+refs/tags/$TARGET_TAG:refs/tags/$TARGET_TAG" ||
            die "拉取 tag 失败: $TARGET_TAG"
        git_sdk checkout --quiet --detach "refs/tags/$TARGET_TAG" || die "切换到 $TARGET_TAG 失败，源码未变更"
    fi
    if [ "$_WORKTREE_NEW" = true ]; then
        git_sdk reset --quiet --hard    # 工作区是空的，从仓库恢复 SDK 目录内容
    else
        git_sdk read-tree -mu HEAD      # 旧的完整检出：收敛到只保留 SDK 目录
    fi
    promisor_off

    [ "$(git_sdk rev-parse HEAD)" = "$TARGET_COMMIT" ] || die "SDK 源码未处于 $TARGET_TAG（$TARGET_COMMIT）"
    [ -f "$SDK_DIR/$SDK_SUBDIR/setup.py" ] || die "SDK 源码不完整: 缺少 $SDK_DIR/$SDK_SUBDIR/setup.py"
}

# ---------------------------------------------------------------- Python 环境

sdk_py() { "$SDK_PYTHON" "$@"; }

pip_cmd() {
    local sub="$1"; shift
    local extra=(--disable-pip-version-check)
    [ "$sub" = install ] && [ -n "$SDK_PIP_MIRROR" ] && extra+=(--index-url "$SDK_PIP_MIRROR")
    "$SDK_PYTHON" -m pip "$sub" "${extra[@]}" "$@"
}

# dist_version <pip包名> —— 已安装则输出版本号，否则输出空
dist_version() {
    (cd / && sdk_py - "$1" <<'EOF'
import sys
try:
    from importlib.metadata import version
except ImportError:
    from importlib_metadata import version
for name in (sys.argv[1], sys.argv[1].replace("-", "_")):
    try:
        print(version(name)); break
    except Exception:
        pass
EOF
    ) 2>/dev/null || true
}

# sdk_import_location —— `import kuavo_humanoid_sdk` 实际会用到的包目录的上级；找不到输出空。
# 只查找不导入（导入 SDK 会在当前目录创建 log/）。
sdk_import_location() {
    (cd / && sdk_py - "$SDK_IMPORT_NAME" <<'EOF'
import importlib.util, os, sys
sys.path = [p for p in sys.path if p not in ("", ".")]
spec = importlib.util.find_spec(sys.argv[1])
if spec and spec.submodule_search_locations:
    print(os.path.dirname(os.path.realpath(list(spec.submodule_search_locations)[0])))
EOF
    ) 2>/dev/null || true
}

# 清掉旧版 pip（setup.py develop 方式）留下的 SDK 链接：easy-install.pth 里的路径和 .egg-link 文件。
# 源码目录被删掉重建后这些链接会悬空，系统自带的 pip 20 遇到它们会在重装时直接报错退出。
scrub_stale_links() {
    sdk_py - "$SDK_IMPORT_NAME" "$SDK_DIST_NAME" <<'EOF' || true
import os, site, sys
pattern = sys.argv[1].lower()
dirs = set()
try:
    dirs.add(site.getusersitepackages())
except Exception:
    pass
if os.geteuid() == 0:
    dirs.update(site.getsitepackages())
for d in dirs:
    for name in (sys.argv[2], sys.argv[2].replace("-", "_")):
        link = os.path.join(d, name + ".egg-link")
        if os.path.isfile(link):
            os.remove(link)
            print("🧹 已清理残留的 %s" % link)
    pth = os.path.join(d, "easy-install.pth")
    if not os.path.isfile(pth):
        continue
    with open(pth) as f:
        lines = f.readlines()
    keep = [l for l in lines if pattern not in l.lower()]
    if len(keep) != len(lines):
        with open(pth, "w") as f:
            f.writelines(keep)
        print("🧹 已清理 %s 中残留的 SDK 路径" % pth)
EOF
}

# 卸载 pip 中的 SDK（未安装时只清残留链接）
uninstall_sdk_dist() {
    if [ -n "$(dist_version "$SDK_DIST_NAME")" ]; then
        pip_cmd uninstall -y "$SDK_DIST_NAME" || die "卸载 $SDK_DIST_NAME 失败"
    fi
    scrub_stale_links
}

# ---------------------------------------------------------------- ROS 消息包

# msg_source_dir <pkg> —— 输出 devel 中该消息包 Python 代码所在的 dist-packages 目录
msg_source_dir() {
    local d
    for d in "$DEVEL_DIR/.private/$1/lib/python3/dist-packages" "$DEVEL_DIR/lib/python3/dist-packages"; do
        [ -d "$d/$1" ] && { echo "$d"; return 0; }
    done
    return 1
}

# check_msg_packages <包目录> <allow_bundled:true|false>
# 在动任何东西之前确认消息包齐全。allow_bundled=true 时，devel 里没有的可用源码自带的。
check_msg_packages() {
    local pkg_dir="$1" allow_bundled="$2" pkg missing=()
    for pkg in $SDK_MSG_PACKAGES; do
        if msg_source_dir "$pkg" >/dev/null; then
            continue
        elif [ "$allow_bundled" = true ] && [ -f "$pkg_dir/$SDK_IMPORT_NAME/msg/$pkg/__init__.py" ]; then
            warn "$pkg 未在 ROS devel 中找到，沿用 SDK 源码自带的消息定义"
        else
            missing+=("$pkg")
        fi
    done
    [ "${#missing[@]}" -eq 0 ] || die "未找到 ROS 消息包: ${missing[*]}（查找位置: $DEVEL_DIR）
   请先编译:  cd $(dirname "$DEVEL_DIR") && catkin build ${missing[*]}"
}

# copy_msg_packages <包目录> —— 把 devel 里的消息包拷进 kuavo_humanoid_sdk/msg/（布局与上游 install.sh 一致）
copy_msg_packages() {
    local dest="$1/$SDK_IMPORT_NAME/msg" pkg src
    mkdir -p "$dest"
    for pkg in $SDK_MSG_PACKAGES; do
        src=$(msg_source_dir "$pkg") || continue    # devel 里没有：保留源码自带的（已由 check 把关）
        rm -rf "${dest:?}/$pkg"
        cp -r "$src/$pkg" "$dest/"
        cat > "$dest/$pkg/__init__.py" <<'EOF'
import os
import sys

# Add package path to sys.path if not already there
package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if package_path not in sys.path:
    sys.path.append(package_path)
EOF
    done
}

# ---------------------------------------------------------------- 安装 SDK 包

ensure_system_deps() {
    # pyaudio 没有 ARM 预编译包，pip 装需要编译；优先用系统包
    if ! (cd / && sdk_py -c "import pyaudio") >/dev/null 2>&1 && command -v apt-get >/dev/null 2>&1; then
        info "🔧 安装 python3-pyaudio（需要 sudo）..."
        sudo apt-get install -y python3-pyaudio ||
            { sudo apt-get update && sudo apt-get install -y python3-pyaudio; } ||
            warn "python3-pyaudio 安装失败，将由 pip 尝试安装 pyaudio"
    fi
    # 系统自带的 requests 过旧会与 SDK 依赖冲突
    if ! (cd / && sdk_py -c "
import requests
assert tuple(int(x) for x in requests.__version__.split('.')[:2]) >= (2, 25)") >/dev/null 2>&1; then
        pip_cmd install --upgrade "requests>=2.25.0" || warn "requests 升级失败，继续安装"
    fi
}

# install_sdk_package <包目录> <版本号> <allow_bundled_msgs> <force:true|false>
# 包目录即包含 setup.py 的目录。以 editable 方式安装，import 直接指向该目录。
install_sdk_package() {
    local pkg_dir="$1" version="$2" allow_bundled="$3" force="$4"
    local installed location target="$pkg_dir"

    [ -f "$pkg_dir/setup.py" ] && [ -d "$pkg_dir/$SDK_IMPORT_NAME" ] || die "不是有效的 SDK 包目录: $pkg_dir"
    sdk_py -m pip --version >/dev/null 2>&1 || die "$SDK_PYTHON 没有可用的 pip"
    check_msg_packages "$pkg_dir" "$allow_bundled"

    if [ -n "$(dist_version "$SDK_CONFLICT_DIST")" ]; then
        confirm "检测到已安装 $SDK_CONFLICT_DIST，与 $SDK_DIST_NAME 冲突，是否卸载它并继续？" || die "已取消"
        pip_cmd uninstall -y "$SDK_CONFLICT_DIST" || die "卸载 $SDK_CONFLICT_DIST 失败"
    fi

    info "📦 更新 ROS 消息包: $SDK_MSG_PACKAGES"
    copy_msg_packages "$pkg_dir"
    chmod -R a+rX "$pkg_dir" 2>/dev/null || true

    installed=$(dist_version "$SDK_DIST_NAME")
    location=$(sdk_import_location)
    if [ "$force" != true ] && [ "$installed" = "$version" ] && [ "$location" = "$(realpath "$pkg_dir")" ]; then
        ok "pip 中已是 $SDK_DIST_NAME $version 且指向本目录，跳过重装（需要强制重装请加 --force）"
        return 0
    fi

    ensure_system_deps
    [ -z "$installed" ] || info "♻️  卸载已安装的 $SDK_DIST_NAME $installed（${location:-位置未知}）..."
    uninstall_sdk_dist
    rm -rf "$pkg_dir/build" "$pkg_dir/dist"
    [ -n "$SDK_EXTRAS" ] && target="$pkg_dir[$SDK_EXTRAS]"

    info "🚀 pip install -e（版本 $version${SDK_EXTRAS:+，extras: $SDK_EXTRAS}）..."
    KUAVO_HUMANOID_SDK_VERSION="$version" pip_cmd install -e "$target" || die "pip 安装失败"

    installed=$(dist_version "$SDK_DIST_NAME")
    location=$(sdk_import_location)
    [ "$installed" = "$version" ] || warn "pip 记录的版本是 ${installed:-空}，预期 $version"
    [ "$location" = "$(realpath "$pkg_dir")" ] ||
        warn "import $SDK_IMPORT_NAME 实际指向 ${location:-空}，而不是 $pkg_dir（检查 PYTHONPATH 或其他安装）"
    ok "已安装 $SDK_DIST_NAME $installed"
}
