#!/usr/bin/env bash
# LeTools environment self-check. Read-only: it never installs or changes packages.
set -u

STRICT=0
[[ "${1:-}" == "--strict" ]] && STRICT=1
PASS=0
WARN=0
FAIL=0

ok() { echo "[OK] $*"; PASS=$((PASS + 1)); }
warn() { local code="$1"; shift; echo "[WARN][$code] $*"; WARN=$((WARN + 1)); }
fail() { local code="$1"; shift; echo "[FAIL][$code] $*"; FAIL=$((FAIL + 1)); }
has() { command -v "$1" >/dev/null 2>&1; }

echo "=== LeTools 环境自检 ==="

if [[ "$(uname -s 2>/dev/null)" == "Linux" ]]; then ok "操作系统: Linux"; else fail E001 "仅支持Linux"; fi
if [[ -r /etc/os-release ]]; then . /etc/os-release; echo "[INFO] Distribution: ${PRETTY_NAME:-unknown}"; fi

for command_name in python3 git; do
    if has "$command_name"; then ok "$command_name: $(command -v "$command_name")"; else fail E002 "缺少$command_name"; fi
done
for command_name in catkin docker; do
    if has "$command_name"; then ok "$command_name已安装"; else warn W001 "$command_name未安装或不在PATH"; fi
done

if [[ -f /opt/ros/noetic/setup.bash ]]; then ok "ROS Noetic已安装"; else fail E003 "缺少/opt/ros/noetic/setup.bash"; fi
if [[ -n "${CONDA_PREFIX:-}" ]]; then warn W002 "Conda已激活: $CONDA_PREFIX；编译ROS前建议conda deactivate"; else ok "未检测到Conda覆盖"; fi

if has python3; then
    python3 -c 'import sys; assert sys.version_info >= (3, 8)' >/dev/null 2>&1 && ok "Python版本>=3.8" || fail E004 "Python版本低于3.8"
    python3 -c 'import numpy; v=tuple(int(x) for x in numpy.__version__.split(".")[:2]); assert (1, 19) <= v < (1, 27)' >/dev/null 2>&1 && ok "NumPy版本范围正确" || warn W003 "NumPy应满足>=1.19.5,<1.27.0"
    python3 -c 'import em' >/dev/null 2>&1 && ok "empy可导入" || warn W004 "缺少empy；Ubuntu可安装python3-empy"
    python3 -c 'import kuavo_msgs, ocs2_msgs' >/dev/null 2>&1 && ok "ROS消息包可导入" || warn W005 "kuavo_msgs/ocs2_msgs不可导入；请source devel/setup.bash"
    python3 -c 'from kuavo_humanoid_sdk import KuavoRobot' >/dev/null 2>&1 && ok "Kuavo SDK可导入" || warn W006 "Kuavo SDK不可导入；请运行scripts/install_sdk.sh"
fi

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION_FILE="$PROJECT_ROOT/scripts/kuavo_humanoid_sdk_tools/sdk_version.env"
if [[ -f "$VERSION_FILE" ]]; then
    ok "找到SDK版本锁定文件"
    grep -E '^SDK_REPO_(BRANCH|TAG)=' "$VERSION_FILE" || fail E005 "SDK版本字段缺失"
else
    fail E005 "缺少sdk_version.env"
fi

[[ -f "$PROJECT_ROOT/infrastructure/ros_packages/devel/setup.bash" ]] && ok "LeTools ROS工作空间已编译" || warn W007 "LeTools devel/setup.bash不存在"
[[ -n "${ROBOT_VERSION:-}" ]] && ok "ROBOT_VERSION=${ROBOT_VERSION}" || warn W008 "ROBOT_VERSION未设置；启动仿真前按机器人型号设置"

if has rostopic && rostopic list >/dev/null 2>&1; then
    ok "ROS master可连接"
    has rosservice && rosservice list 2>/dev/null | grep -q mobile_manipulator && ok "检测到mobile_manipulator服务" || warn W010 "未检测到mobile_manipulator服务；仿真可能未启动"
else
    warn W009 "ROS master不可连接；仅做dry-run时可忽略"
fi

if has nvidia-smi && nvidia-smi >/dev/null 2>&1; then
    ok "NVIDIA驱动可用"
    has nvidia-ctk && ok "NVIDIA Container Toolkit已安装" || warn W011 "nvidia-ctk不可用；仅GPU容器需要"
else
    warn W011 "未检测到NVIDIA GPU；CPU/dry-run仍可使用"
fi

if has df; then
    FREE_KB="$(df -Pk "$PROJECT_ROOT" | awk 'NR==2 {print $4}')"
    [[ "${FREE_KB:-0}" -ge 10485760 ]] && ok "项目盘剩余空间>=10GB" || warn W012 "项目盘剩余空间不足10GB"
fi

echo "=== Summary: OK=$PASS WARN=$WARN FAIL=$FAIL ==="
if [[ "$FAIL" -gt 0 ]]; then exit 1; fi
if [[ "$STRICT" -eq 1 && "$WARN" -gt 0 ]]; then exit 2; fi
exit 0
