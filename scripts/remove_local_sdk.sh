#!/bin/bash
# remove_local_sdk.sh - 卸载本地 Kuavo Humanoid SDK，并可选择删除项目内 SDK 源码

set -euo pipefail

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/..")
SDK_SOURCE_DIR="$PROJECT_DIR/drivers/leju/kuavo_humanoid_sdk"
REMOVE_SOURCE=false
ASSUME_YES=false

usage() {
    cat <<EOF
用法: $0 [--remove-source] [--yes]

默认仅卸载当前 Python 环境中的 kuavo-humanoid-sdk。

选项:
  --remove-source  同时删除项目内 SDK 源码目录：$SDK_SOURCE_DIR
  --yes            跳过确认提示；建议仅用于自动化环境
  -h, --help       显示本帮助
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --remove-source) REMOVE_SOURCE=true ;;
        --yes) ASSUME_YES=true ;;
        -h|--help) usage; exit 0 ;;
        *)
            echo "未知参数: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

echo "将卸载当前 Python 环境中的 kuavo-humanoid-sdk。"
if [ "$REMOVE_SOURCE" = true ]; then
    echo "还将永久删除项目内 SDK 源码目录：$SDK_SOURCE_DIR"
fi

if [ "$ASSUME_YES" != true ]; then
    read -r -p "确认继续？输入 yes: " CONFIRMATION
    if [ "$CONFIRMATION" != "yes" ]; then
        echo "已取消。"
        exit 0
    fi
fi

if python3 -m pip show kuavo-humanoid-sdk >/dev/null 2>&1; then
    python3 -m pip uninstall -y kuavo-humanoid-sdk
else
    echo "未检测到已安装的 kuavo-humanoid-sdk。"
fi

if [ "$REMOVE_SOURCE" = true ] && [ -e "$SDK_SOURCE_DIR" ]; then
    case "$SDK_SOURCE_DIR" in
        "$PROJECT_DIR"/drivers/leju/kuavo_humanoid_sdk) ;;
        *)
            echo "拒绝删除未预期的路径: $SDK_SOURCE_DIR" >&2
            exit 1
            ;;
    esac
    rm -rf "$SDK_SOURCE_DIR"
    echo "已删除 SDK 源码目录。"
fi

echo "SDK 清理完成。"
