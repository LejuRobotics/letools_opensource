#!/bin/bash
# install_local.sh - 安装本地已有的 Kuavo Humanoid SDK 源码（全程不访问 SDK 远程仓库）
#
# 适用于无法访问远程仓库、或要安装一份手动拷贝/本地修改过的 SDK。
# 以 editable 方式安装，import 直接指向源码所在目录，不做拷贝。

set -euo pipefail
source "$(dirname "$(realpath "$0")")/common.sh"

usage() {
    cat <<EOF
用法: $0 [选项]

选项:
  --source <dir>     SDK 源码目录（默认: $SUBMODULE_PATH）
                     可以是 SDK 包目录本身（含 setup.py），也可以是上游仓库根目录
                     （SDK 位于其 $SDK_SUBDIR 下）
  --version <ver>    指定安装版本号（默认自动识别，见下）
  --extras <extras>  安装可选依赖: audio | vision | full
  --force            即使 pip 中已是该版本也重新安装
  --yes              跳过确认提示
  -h, --help         显示本帮助

版本号识别顺序: --version > 源码目录自身 git 仓库的 tag > 配套版本加 +local 标记
ROS 消息包优先取自 ROS devel；devel 里没有时沿用源码自带的消息定义。
EOF
}

SOURCE="$SDK_DIR"; CLI_VERSION=""; CLI_EXTRAS=""; FORCE=false; ASSUME_YES=false
while [ "$#" -gt 0 ]; do
    case "$1" in
        --source)  SOURCE="${2:?--source 需要参数}"; shift ;;
        --version) CLI_VERSION="${2:?--version 需要参数}"; shift ;;
        --extras)  CLI_EXTRAS="${2:?--extras 需要参数}"; shift ;;
        --force)   FORCE=true ;;
        --yes)     ASSUME_YES=true ;;
        -h|--help) usage; exit 0 ;;
        *)         usage >&2; die "未知参数: $1" ;;
    esac
    shift
done

START=$SECONDS
load_config
[ -n "$CLI_EXTRAS" ] && SDK_EXTRAS="$CLI_EXTRAS"
command -v "$SDK_PYTHON" >/dev/null || die "未找到 Python: $SDK_PYTHON"

[ -d "$SOURCE" ] || die "源码目录不存在: $SOURCE"
SOURCE=$(realpath "$SOURCE")
if [ -f "$SOURCE/setup.py" ] && [ -d "$SOURCE/$SDK_IMPORT_NAME" ]; then
    PKG_DIR="$SOURCE"
elif [ -f "$SOURCE/$SDK_SUBDIR/setup.py" ] && [ -d "$SOURCE/$SDK_SUBDIR/$SDK_IMPORT_NAME" ]; then
    PKG_DIR="$SOURCE/$SDK_SUBDIR"
else
    die "未在 $SOURCE 找到 SDK 包。期望存在以下之一：
   $SOURCE/setup.py
   $SOURCE/$SDK_SUBDIR/setup.py"
fi

# 版本号。源码目录没有自己的 .git 时，git 会向上解析到 LeTools 仓库，必须排除，
# 否则会把 LeTools 的 tag 当成 SDK 版本。
SRC_TOP=$(git -C "$SOURCE" rev-parse --show-toplevel 2>/dev/null || true)
if [ -n "$CLI_VERSION" ]; then
    VERSION=$(pep440_version "$CLI_VERSION"); VERSION_FROM="--version 指定"
elif [ -n "$SRC_TOP" ] && [ "$(realpath "$SRC_TOP")" != "$PROJECT_DIR" ] &&
     DESCRIBED=$(git -C "$SOURCE" describe --tags --exact-match 2>/dev/null || git -C "$SOURCE" describe --tags --always 2>/dev/null); then
    VERSION=$(pep440_version "$DESCRIBED"); VERSION_FROM="源码 git: $DESCRIBED"
else
    VERSION="$PIN_TAG+local"; VERSION_FROM="无法识别，按配套版本加 +local 标记"
    warn "无法从源码识别 SDK 版本，记为 $VERSION。如已知真实版本，请用 --version 指定"
fi

info "📂 SDK 源码: $PKG_DIR"
info "📌 安装版本: $VERSION（$VERSION_FROM）"

install_sdk_package "$PKG_DIR" "$VERSION" true "$FORCE"

echo ""
ok "完成，用时 $((SECONDS - START)) 秒"
info "验证:  $SDK_PYTHON -c \"from kuavo_humanoid_sdk import KuavoRobot; print('SDK Ready')\"  （需先 source ROS 工作空间的 devel/setup.bash）"
