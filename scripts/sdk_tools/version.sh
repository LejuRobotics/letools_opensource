#!/bin/bash
# version.sh - Kuavo Humanoid SDK 版本管理：查看状态、列出可用版本、切换、恢复配套版本

set -euo pipefail
source "$(dirname "$(realpath "$0")")/common.sh"

usage() {
    cat <<EOF
用法: $0 [命令]

命令:
  status                       （默认）显示配套版本、本地覆盖、源码版本、pip 安装版本是否一致；
                               一致返回 0，不一致返回 1
  list                         列出远程仓库的全部 tag
  use <tag> [--branch <b>]     切换到指定版本并安装，写入 sdk_config.sh 作为本地覆盖
                               （--branch 默认用配套分支；tag 必须属于该分支）
  reset                        清除本地覆盖，装回 sdk_version.env 的配套版本
  pin <tag> [--branch <b>]     【维护者】修改 sdk_version.env 的配套版本（发版时用）
  -h, --help                   显示本帮助

use / reset 会调用 install.sh，其后的参数（如 --force、--yes）原样透传。
EOF
}

# 源码当前版本：输出 "<tag 或 describe>|<commit 短哈希>"；不是 git 仓库则两项为空
source_version() {
    sdk_is_own_repo || { echo "|"; return 0; }
    local head
    head=$(git_sdk rev-parse -q --verify "HEAD^{commit}" 2>/dev/null) || { echo "|"; return 0; }
    echo "$(git_sdk describe --tags --exact-match 2>/dev/null || git_sdk describe --tags --always 2>/dev/null || echo "?")|${head:0:9}"
}

cmd_status() {
    local src src_tag src_commit installed location module module_size verdict=0
    resolve_target "" ""
    src=$(source_version); src_tag="${src%%|*}"; src_commit="${src##*|}"
    installed=$(dist_version "$SDK_DIST_NAME")
    location=$(sdk_import_location)
    module=$(module_dir)
    module_size=$([ -n "$module" ] && [ -d "$module" ] && du -sh "$module" | cut -f1 || true)

    echo "配套版本  $PIN_BRANCH / $PIN_TAG    （$(realpath --relative-to="$PROJECT_DIR" "$PIN_FILE")）"
    if [ "$TARGET_SOURCE" = "本地覆盖 sdk_config.sh" ]; then
        echo "本地覆盖  $TARGET_BRANCH / $TARGET_TAG    （$(realpath --relative-to="$PROJECT_DIR" "$CONFIG_FILE")）"
    else
        echo "本地覆盖  无"
    fi
    if [ -n "$src_commit" ]; then
        echo "源码      $src_tag  $src_commit${module_size:+  仓库数据 $module_size}"
    elif [ -f "$SDK_DIR/$SDK_SUBDIR/setup.py" ] || [ -f "$SDK_DIR/setup.py" ]; then
        echo "源码      本地拷贝（非 git，版本未知）"
    else
        echo "源码      无"
    fi
    if [ -n "$installed" ]; then
        echo "pip 安装  $installed  → ${location:-无法定位}"
    else
        echo "pip 安装  未安装"
    fi
    echo ""

    if [ -z "$installed" ]; then
        warn "SDK 未安装"; verdict=1
    else
        if [ "$installed" != "$(pep440_version "$TARGET_TAG")" ]; then
            warn "pip 安装的版本是 $installed，目标版本是 $TARGET_TAG"; verdict=1
        fi
        case "$location" in
            "$SDK_DIR"|"$SDK_DIR"/*) ;;
            *) warn "pip 安装的 SDK 不指向本项目的源码目录: ${location:-无法定位}"; verdict=1 ;;
        esac
        if [ -n "$src_commit" ] && [ "$src_tag" != "$TARGET_TAG" ]; then
            warn "源码处于 $src_tag，目标版本是 $TARGET_TAG"; verdict=1
        fi
    fi
    if [ "$verdict" -eq 0 ]; then
        ok "一致：已安装目标版本 $TARGET_TAG"
    else
        info "修复:  scripts/sdk_tools/install.sh"
    fi
    return "$verdict"
}

cmd_list() {
    local url tags tag mark src
    url=$(sdk_remote_url)
    tags=$(git ls-remote --tags --refs "$url" 2>/dev/null | sed 's|.*refs/tags/||' | sort -V) ||
        die "无法访问 SDK 远程仓库: $url"
    resolve_target "" ""
    src=$(source_version)
    info "远程仓库 $url 的 tag："
    for tag in $tags; do
        mark=""
        [ "$tag" = "$PIN_TAG" ] && mark+="  ← 配套版本"
        [ "$tag" = "$TARGET_TAG" ] && [ "$TARGET_TAG" != "$PIN_TAG" ] && mark+="  ← 本地覆盖"
        [ "$tag" = "${src%%|*}" ] && mark+="  ← 当前源码"
        echo "  $tag$mark"
    done
}

# parse_tag_args <tag> [--branch <b>] [其余参数...] —— 设置 ARG_TAG / ARG_BRANCH / ARG_REST
parse_tag_args() {
    ARG_TAG=""; ARG_BRANCH=""; ARG_REST=()
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --branch) ARG_BRANCH="${2:?--branch 需要参数}"; shift ;;
            -*)       ARG_REST+=("$1") ;;
            *)        [ -z "$ARG_TAG" ] || die "多余的参数: $1"; ARG_TAG="$1" ;;
        esac
        shift
    done
    [ -n "$ARG_TAG" ] || { usage >&2; die "缺少 <tag>"; }
    ARG_BRANCH="${ARG_BRANCH:-$PIN_BRANCH}"
}

cmd_use() {
    parse_tag_args "$@"
    if [ "$ARG_TAG" = "$PIN_TAG" ] && [ "$ARG_BRANCH" = "$PIN_BRANCH" ]; then
        info "$ARG_BRANCH / $ARG_TAG 就是配套版本，按 reset 处理"
        cmd_reset "${ARG_REST[@]}"
        return
    fi
    # 先装成功再写配置：tag 不存在或不属于该分支时，配置保持不变
    "$SDK_TOOLS_DIR/install.sh" --tag "$ARG_TAG" --branch "$ARG_BRANCH" "${ARG_REST[@]}"
    set_config_override "$ARG_BRANCH" "$ARG_TAG"
    ok "已切换到 $ARG_BRANCH / $ARG_TAG，并记入本地覆盖（恢复配套版本: $0 reset）"
}

cmd_reset() {
    if [ -f "$CONFIG_FILE" ]; then
        set_config_override "" ""
    fi
    "$SDK_TOOLS_DIR/install.sh" "$@"
    ok "已恢复配套版本 $PIN_BRANCH / $PIN_TAG"
}

cmd_pin() {
    parse_tag_args "$@"
    TARGET_TAG="$ARG_TAG"; TARGET_BRANCH="$ARG_BRANCH"
    info "🔍 校验 $TARGET_BRANCH / $TARGET_TAG ..."
    verify_pairing
    sed -i -E "s|^SDK_REPO_BRANCH=.*|SDK_REPO_BRANCH=\"$TARGET_BRANCH\"|; s|^SDK_REPO_TAG=.*|SDK_REPO_TAG=\"$TARGET_TAG\"|" "$PIN_FILE"
    ok "配套版本已改为 $TARGET_BRANCH / $TARGET_TAG（${TARGET_COMMIT:0:9}）: $PIN_FILE"
    info "发版前还需："
    info "  1. scripts/sdk_tools/install.sh                 # 按新版本安装并回归验证"
    info "  2. git add $PIN_FILE $SUBMODULE_PATH   # 提交锁定清单与 submodule 指针"
    info "  3. python3 -m pytest tests/test_release_consistency.py   # 检查文档中的版本号是否同步"
}

load_config
CMD="${1:-status}"
[ "$#" -gt 0 ] && shift
case "$CMD" in
    status)    cmd_status ;;
    list)      cmd_list ;;
    use)       cmd_use "$@" ;;
    reset)     cmd_reset "$@" ;;
    pin)       cmd_pin "$@" ;;
    -h|--help) usage ;;
    *)         usage >&2; die "未知命令: $CMD" ;;
esac
