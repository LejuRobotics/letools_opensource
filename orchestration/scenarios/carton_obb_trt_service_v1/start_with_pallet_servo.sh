#!/usr/bin/env bash
# 按顺序启动共享纸箱 OBB 服务，再启动托盘伺服场景。
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../../.." && pwd)"
SERVICE_SCRIPT="${SCENARIO_DIR}/start_all.sh"
PALLET_SCRIPT="${REPO_ROOT}/orchestration/scenarios/pallet_servo_real_v1/start_all.sh"
BOARD="${SCENARIO_DIR}/board.json"

SERVICE_PID=""
PALLET_PID=""
DETECTORS_READY_FILE="/tmp/letools_pallet_detectors_ready_$$"

source_if_exists() {
  local setup_file="$1"
  [[ -f "${setup_file}" ]] || return 0
  local had_nounset=0
  case "$-" in *u*) had_nounset=1; set +u ;; esac
  # shellcheck disable=SC1090
  source "${setup_file}"
  [[ "${had_nounset}" == "1" ]] && set -u
}

board_get() {
  python3 - "${BOARD}" "$1" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1], encoding="utf-8"))
wanted = sys.argv[2]
for group in data.values():
    if not isinstance(group, list):
        continue
    for item in group:
        if isinstance(item, dict) and item.get("key") == wanted:
            print(item["value"])
            raise SystemExit(0)
raise SystemExit(f"board.json 缺少键：{wanted}")
PY
}

process_is_running() {
  local pid="$1"
  kill -0 "${pid}" 2>/dev/null || return 1
  [[ ! -r "/proc/${pid}/stat" ]] || [[ "$(awk '{print $3}' "/proc/${pid}/stat")" != "Z" ]]
}

stop_process() {
  local pid="$1"
  local name="$2"
  process_is_running "${pid}" || {
    wait "${pid}" 2>/dev/null || true
    return 0
  }

  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 30); do
    process_is_running "${pid}" || break
    sleep 0.1
  done
  if process_is_running "${pid}"; then
    echo "  ${name} 未响应 SIGINT，升级为 SIGTERM ..."
    kill -TERM "${pid}" 2>/dev/null || true
  fi
  for _ in $(seq 1 20); do
    process_is_running "${pid}" || break
    sleep 0.1
  done
  if process_is_running "${pid}"; then
    echo "  ${name} 未响应 SIGTERM，升级为 SIGKILL ..."
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local exit_code=$?
  trap - EXIT HUP INT TERM
  rm -f "${DETECTORS_READY_FILE}"

  if [[ -n "${PALLET_PID}" ]]; then
    echo
    echo "正在停止托盘伺服场景 ..."
    stop_process "${PALLET_PID}" "托盘伺服场景"
  fi
  if [[ -n "${SERVICE_PID}" ]]; then
    echo "正在停止共享纸箱 OBB 服务 ..."
    stop_process "${SERVICE_PID}" "共享纸箱 OBB 服务"
  fi
  exit "${exit_code}"
}

trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

source_if_exists "/opt/ros/noetic/setup.bash"
source_if_exists "${REPO_ROOT}/infrastructure/ros_packages/devel/setup.bash"
export ROS_PACKAGE_PATH="${REPO_ROOT}/infrastructure/ros_packages/src:${ROS_PACKAGE_PATH:-}"

# Noetic setup.bash 会把未设置的 ROS_MASTER_URI 填成 http://localhost:11311。
# 现场 Kuavo 的 master 通常在下位机主机名 kuavo_master；若本机能解析该名，
# 且当前仍是 Noetic 默认 localhost，则改用现场默认，避免「脚本起得来但连不上」。
# 用户已显式 export 非 localhost 的 URI 时不改动；本机仿真（无 kuavo_master）也不改。
prefer_field_ros_master() {
  case "${ROS_MASTER_URI:-}" in
    ""|http://localhost:11311|http://127.0.0.1:11311) ;;
    *) return 0 ;;
  esac
  if getent hosts kuavo_master >/dev/null 2>&1 \
      || grep -Eq '^[[:space:]]*[^#].*\bkuavo_master\b' /etc/hosts 2>/dev/null; then
    export ROS_MASTER_URI="http://kuavo_master:11311"
    echo "ROS_MASTER_URI 从 Noetic 默认 localhost 调整为现场：${ROS_MASTER_URI}"
  fi
}
prefer_field_ros_master

# --------------------------------------------------------------------------- #
# 整条链路从 **carton 虚拟环境** 启动（不是 base 环境）。
#
# 两个子脚本都是 bash，会继承这里的 PATH / VIRTUAL_ENV / PYTHONNOUSERSITE。
# OBB 启动脚本会把当前 `command -v python3` 作为 roslaunch 的 launch-prefix，
# 避免 Catkin 生成脚本中的绝对 shebang（#!/usr/bin/python3）绕过 venv。
#
# 默认使用 /media/data/venvs/carton；其他机器可通过 CARTON_VENV_DIR 覆盖。
# ⚠️ activate 里已设 PYTHONNOUSERSITE=1，用来屏蔽 ~/.local 里的旧包；
#    别把这一行删掉。
# --------------------------------------------------------------------------- #
VENV_DIR="${CARTON_VENV_DIR:-/media/data/venvs/carton}"
VENV_ACTIVATE="${VENV_DIR}/bin/activate"
if [[ ! -f "${VENV_ACTIVATE}" ]]; then
  echo "找不到 carton 虚拟环境：${VENV_ACTIVATE}" >&2
  echo "  请设置 CARTON_VENV_DIR，或把虚拟环境部署到默认目录。" >&2
  exit 2
fi
source_if_exists "${VENV_ACTIVATE}"

# 兼容从固定路径打包、再解压到其他目录的 venv。activate 可能仍含旧绝对路径，
# 启动阶段以用户指定的 CARTON_VENV_DIR 为准修正运行环境。
export VIRTUAL_ENV="${VENV_DIR}"
export PATH="${VENV_DIR}/bin:${_OLD_VIRTUAL_PATH:-${PATH}}"
export YOLO_CONFIG_DIR="${VENV_DIR}/config/ultralytics"
export MPLCONFIGDIR="${VENV_DIR}/config/matplotlib"
hash -r
if [[ ! -x "${VENV_DIR}/bin/python3" ]]; then
  echo "carton 虚拟环境缺少可执行的 python3：${VENV_DIR}/bin/python3" >&2
  exit 2
fi

# Jetson 上先 import cv2/numpy 再 import torch，会触发
#   libgomp.so.1: cannot allocate memory in static TLS block
# 预加载 libgomp 即可。~/.bashrc 里已有同一行；这里显式设一次，保证从
# 非交互 shell / 其他启动方式进来也生效。
case ":${LD_PRELOAD:-}:" in
  *":/usr/lib/aarch64-linux-gnu/libgomp.so.1:"*) ;;
  *) export LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:${LD_PRELOAD}}" ;;
esac

echo "使用虚拟环境：${VIRTUAL_ENV}（$(command -v python3)）"

[[ -x "${SERVICE_SCRIPT}" ]] || {
  echo "找不到可执行脚本：${SERVICE_SCRIPT}" >&2
  exit 2
}
[[ -x "${PALLET_SCRIPT}" ]] || {
  echo "找不到可执行脚本：${PALLET_SCRIPT}" >&2
  exit 2
}

INFER_SERVICE="$(board_get service_name)"
SERVO_SERVICE="$(board_get servo_service_name)"

echo "[1/2] 启动共享纸箱 OBB（先等推理，伺服在同脚本内继续拉起）..."
bash "${SERVICE_SCRIPT}" &
SERVICE_PID=$!

# 检测器只依赖推理链路（相机/图 + /carton_obb/infer），不必等伺服注册完。
# start_all.sh 在 infer 就绪后会继续起 /carton_obb/servo；这里一等到 infer
# 就并行拉托盘/纸箱检测器，与伺服启动重叠，缩短总等待。
echo "      等待推理服务注册：${INFER_SERVICE}"
SERVICE_READY_TIMEOUT="$(( $(board_get camera_startup_timeout) + 30 ))"
READY=0
for _ in $(seq 1 "${SERVICE_READY_TIMEOUT}"); do
  if ! process_is_running "${SERVICE_PID}"; then
    echo "  ✗ 共享纸箱 OBB 启动脚本提前退出" >&2
    exit 1
  fi
  if rosservice info "${INFER_SERVICE}" >/dev/null 2>&1; then
    READY=1
    break
  fi
  sleep 1
done
if [[ "${READY}" != "1" ]]; then
  echo "  ✗ 等待推理服务超时（${SERVICE_READY_TIMEOUT}s）：${INFER_SERVICE}" >&2
  exit 1
fi
echo "      ✓ 推理服务已就绪（伺服 ${SERVO_SERVICE} 继续在后台注册）"

echo
echo "[2/2] 并行启动托盘/纸箱检测器（行为树由海程主流程提供）..."
rm -f "${DETECTORS_READY_FILE}"
PALLET_DETECTORS_READY_FILE="${DETECTORS_READY_FILE}" \
  bash "${PALLET_SCRIPT}" --detectors-only &
PALLET_PID=$!

echo "      等待两个检测器预热完成（通常 30~40s，首次加载可能更久）..."
DETECTORS_READY=0
SERVO_READY=0
if rosservice info "${SERVO_SERVICE}" >/dev/null 2>&1; then
  SERVO_READY=1
fi
for elapsed in $(seq 1 40); do
  if ! process_is_running "${SERVICE_PID}"; then
    echo "  ✗ 共享纸箱 OBB 启动脚本在检测器预热期间提前退出" >&2
    exit 1
  fi
  if ! process_is_running "${PALLET_PID}"; then
    echo "  ✗ 托盘/纸箱检测器启动脚本在预热期间提前退出" >&2
    exit 1
  fi
  if [[ "${SERVO_READY}" != "1" ]] \
      && rosservice info "${SERVO_SERVICE}" >/dev/null 2>&1; then
    SERVO_READY=1
    echo "      ✓ 伺服服务已就绪：${SERVO_SERVICE}"
  fi
  if [[ -f "${DETECTORS_READY_FILE}" ]]; then
    DETECTORS_READY=1
    break
  fi
  if (( elapsed % 10 == 0 )); then
    echo "      检测器仍在预热：已等待 ${elapsed}s / 40s（伺服=$([[ ${SERVO_READY} == 1 ]] && echo 已就绪 || echo 启动中)）"
  fi
  sleep 1
done
# 检测器先好了也再给伺服一点时间，避免提示「全部就绪」时伺服其实还没挂上。
if [[ "${SERVO_READY}" != "1" ]]; then
  for _ in $(seq 1 20); do
    if ! process_is_running "${SERVICE_PID}"; then
      echo "  ✗ 共享纸箱 OBB 启动脚本在等待伺服时提前退出" >&2
      exit 1
    fi
    if rosservice info "${SERVO_SERVICE}" >/dev/null 2>&1; then
      SERVO_READY=1
      echo "      ✓ 伺服服务已就绪：${SERVO_SERVICE}"
      break
    fi
    sleep 1
  done
fi

if [[ "${DETECTORS_READY}" != "1" ]]; then
  echo "  ⚠️ 检测器 40s 内未报告预热完成，将继续在后台等待相机/模型。" >&2
fi
if [[ "${SERVO_READY}" != "1" ]]; then
  echo "  ⚠️ 伺服 ${SERVO_SERVICE} 尚未注册成功，将继续在后台等待。" >&2
fi
if [[ "${DETECTORS_READY}" == "1" && "${SERVO_READY}" == "1" ]]; then
  echo
  echo "============================================================"
  echo "  ✓ OBB 推理、OBB 伺服、托盘和纸箱检测器全部就绪"
  echo "  ✓ 当前为常驻待命状态，可以启动海程纸箱主流程"
  echo "============================================================"
elif [[ "${DETECTORS_READY}" == "1" ]]; then
  echo
  echo "============================================================"
  echo "  ✓ OBB 推理与托盘/纸箱检测器已就绪；伺服仍在后台启动"
  echo "  ✓ 主流程若马上调用 ${SERVO_SERVICE}，请再确认该服务已注册"
  echo "============================================================"
fi

# 任一场景异常退出，都立即结束总流程并清理另一个场景。
while true; do
  if ! process_is_running "${SERVICE_PID}"; then
    if wait "${SERVICE_PID}"; then
      SERVICE_CODE=0
    else
      SERVICE_CODE=$?
    fi
    SERVICE_PID=""
    echo "  ✗ 共享纸箱 OBB 服务已退出（状态码 ${SERVICE_CODE}）" >&2
    [[ "${SERVICE_CODE}" != "0" ]] || SERVICE_CODE=1
    exit "${SERVICE_CODE}"
  fi
  if ! process_is_running "${PALLET_PID}"; then
    if wait "${PALLET_PID}"; then
      PALLET_CODE=0
    else
      PALLET_CODE=$?
    fi
    PALLET_PID=""
    echo "  ✗ 托盘/纸箱检测器启动脚本已退出（状态码 ${PALLET_CODE}）" >&2
    [[ "${PALLET_CODE}" != "0" ]] || PALLET_CODE=1
    exit "${PALLET_CODE}"
  fi
  sleep 1
done

