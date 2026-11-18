#!/bin/bash
# remove.sh - 卸载 Kuavo Humanoid SDK，可选连同源码与仓库数据一起清除

set -euo pipefail
source "$(dirname "$(realpath "$0")")/common.sh"

usage() {
    cat <<EOF
用法: $0 [--source | --purge] [--yes]

默认只卸载 pip 中的 $SDK_DIST_NAME，源码保留。

选项:
  --source   同时删除 SDK 源码目录（$SUBMODULE_PATH）；
             仓库数据保留，再次安装时无需重新下载
  --purge    彻底清除：源码 + .git/modules 下的仓库数据 + submodule 登记，
             之后再安装等同于全新安装
  --yes      跳过确认提示
  -h, --help 显示本帮助
EOF
}

REMOVE_SOURCE=false; PURGE=false; ASSUME_YES=false
while [ "$#" -gt 0 ]; do
    case "$1" in
        --source)  REMOVE_SOURCE=true ;;
        --purge)   PURGE=true; REMOVE_SOURCE=true ;;
        --yes)     ASSUME_YES=true ;;
        -h|--help) usage; exit 0 ;;
        *)         usage >&2; die "未知参数: $1" ;;
    esac
    shift
done

load_config
MODULE=$(module_dir)
INSTALLED=$(dist_version "$SDK_DIST_NAME")
LOCATION=$(sdk_import_location)

# 删除前再确认一次路径，防止变量异常时误删
case "$SDK_DIR" in */drivers/leju/kuavo_humanoid_sdk) ;; *) die "拒绝操作未预期的源码路径: $SDK_DIR" ;; esac
case "$MODULE" in ""|*/modules/drivers/leju/kuavo_humanoid_sdk) ;; *) die "拒绝操作未预期的仓库数据路径: $MODULE" ;; esac

info "将执行："
if [ -n "$INSTALLED" ]; then
    info "  - 卸载 pip 包 $SDK_DIST_NAME $INSTALLED（${LOCATION:-位置未知}）"
    case "$LOCATION" in
        ""|"$SDK_DIR"|"$SDK_DIR"/*) ;;
        *) warn "当前安装的 SDK 指向本项目之外的目录。卸载后，依赖那份 SDK 的程序将无法 import，直到重新安装" ;;
    esac
else
    info "  - pip 中未安装 $SDK_DIST_NAME，跳过卸载"
fi
if [ "$REMOVE_SOURCE" = true ]; then
    info "  - 删除源码目录 $SDK_DIR"
    if sdk_is_own_repo && [ -n "$(git_sdk status --porcelain --untracked-files=no 2>/dev/null)" ]; then
        warn "源码目录中有未提交的本地修改，删除后无法恢复："
        git_sdk status --porcelain --untracked-files=no | sed 's/^/     /' >&2
    fi
fi
if [ "$PURGE" = true ] && [ -n "$MODULE" ]; then
    info "  - 删除仓库数据 $MODULE（$(du -sh "$MODULE" 2>/dev/null | cut -f1 || echo 不存在)）并取消 submodule 登记"
fi
confirm "确认继续？" || { info "已取消"; exit 0; }

uninstall_sdk_dist
[ -z "$INSTALLED" ] || ok "已卸载 $SDK_DIST_NAME"

if [ "$REMOVE_SOURCE" = true ]; then
    rm -rf "$SDK_DIR"
    # 保留空目录：submodule 路径整个消失时 git 会把它记为“已删除”，容易被误提交
    mkdir -p "$SDK_DIR"
    ok "已删除 SDK 源码"
fi

if [ "$PURGE" = true ] && [ -n "$MODULE" ]; then
    rm -rf "$MODULE"
    git -C "$PROJECT_DIR" config --remove-section "submodule.$SUBMODULE_PATH" 2>/dev/null || true
    ok "已删除 SDK 仓库数据"
fi

info "重新安装:  scripts/sdk_tools/install.sh"
