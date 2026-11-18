#!/bin/bash
# install.sh - 在线安装 Kuavo Humanoid SDK（首次安装、升级、回退都用它）
#
# 按 sdk_version.env 锁定的配套版本，从远程仓库获取 SDK 源码并以 editable 方式安装。
# 只下载 SDK 目录（约 10MB）。重复运行是安全的：已是目标版本时不重新下载、不重装。

set -euo pipefail
source "$(dirname "$(realpath "$0")")/common.sh"

usage() {
    cat <<EOF
用法: $0 [选项]

选项:
  --tag <tag>        本次安装指定 tag（不改配置；持久切换请用 version.sh use）
  --branch <branch>  与 --tag 配合，指定该 tag 所属分支（默认用配套分支）
  --extras <extras>  安装可选依赖: audio | vision | full
  --force            即使 pip 中已是目标版本也重新安装
  --yes              跳过确认提示
  -h, --help         显示本帮助

版本来源优先级: --tag > sdk_config.sh 本地覆盖 > sdk_version.env 配套版本
无网络时请用 install_local.sh 安装本地源码。
EOF
}

CLI_TAG=""; CLI_BRANCH=""; CLI_EXTRAS=""; FORCE=false; ASSUME_YES=false
while [ "$#" -gt 0 ]; do
    case "$1" in
        --tag)     CLI_TAG="${2:?--tag 需要参数}"; shift ;;
        --branch)  CLI_BRANCH="${2:?--branch 需要参数}"; shift ;;
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
resolve_target "$CLI_TAG" "$CLI_BRANCH"
PKG_DIR="$SDK_DIR/$SDK_SUBDIR"

info "📌 目标版本: $TARGET_BRANCH / $TARGET_TAG（$TARGET_SOURCE）"

# 先做不需要网络的检查，缺东西立刻报错，而不是下载完才发现
command -v git >/dev/null || die "未找到 git"
command -v "$SDK_PYTHON" >/dev/null || die "未找到 Python: $SDK_PYTHON"
check_msg_packages "$PKG_DIR" false

info "🔍 校验 tag 与分支..."
verify_pairing
sync_source
ok "SDK 源码就绪: $TARGET_TAG（${TARGET_COMMIT:0:9}）"

install_sdk_package "$PKG_DIR" "$(pep440_version "$TARGET_TAG")" false "$FORCE"

echo ""
ok "完成，用时 $((SECONDS - START)) 秒"
info "验证:  $SDK_PYTHON -c \"from kuavo_humanoid_sdk import KuavoRobot; print('SDK Ready')\"  （需先 source ROS 工作空间的 devel/setup.bash）"
info "状态:  scripts/sdk_tools/version.sh"
