#!/usr/bin/env bash
# 一键启动共享纸箱 OBB TensorRT 推理服务。
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../../.." && pwd)"
BOARD="${SCENARIO_DIR}/board.json"
NODE_NAME="/carton_obb_trt_service"
SERVO_NODE_NAME="/carton_obb_servo_service"
TF_NODE_NAME="/carton_obb_camera_tf_bridge"
CAMERA_PID=""
TF_PID=""
SERVICE_PID=""
SERVO_PID=""
CAMERA_OWNED=0
TF_OWNED=0

DRY_RUN=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help)
      sed -n '1,80p' "${BASH_SOURCE[0]}"
      exit 0
      ;;
    *)
      echo "未知参数：$1（只支持 --dry-run）" >&2
      exit 2
      ;;
  esac
done

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

path, wanted = sys.argv[1], sys.argv[2]
data = json.load(open(path, encoding="utf-8"))
for group in data.values():
    if not isinstance(group, list):
        continue
    for item in group:
        if isinstance(item, dict) and item.get("key") == wanted:
            value = item.get("value")
            if isinstance(value, bool):
                print("true" if value else "false")
            else:
                print(value)
            raise SystemExit(0)
raise SystemExit(f"board.json 缺少键：{wanted}")
PY
}

resolve_repo_path() {
  local value="$1"
  if [[ "${value}" = /* ]]; then
    printf '%s' "${value}"
  else
    printf '%s/%s' "${REPO_ROOT}" "${value}"
  fi
}

topic_has_publishers() {
  local topic_info
  topic_info="$(rostopic info "$1" 2>/dev/null)" || return 1
  sed -n '/^Publishers:/,/^Subscribers:/p' <<<"${topic_info}" \
    | grep -q '^ \* '
}

wait_for_message() {
  timeout "$2" rostopic echo -n 1 "$1" >/dev/null 2>&1
}

wait_for_service() {
  local service_name="$1"
  local timeout_seconds="$2"
  local process_id="$3"
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    if ! kill -0 "${process_id}" 2>/dev/null; then
      return 1
    fi
    if rosservice info "${service_name}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

pid_is_running() {
  local process_id="$1"
  local process_state
  kill -0 "${process_id}" 2>/dev/null || return 1
  if [[ -r "/proc/${process_id}/stat" ]]; then
    process_state="$(awk '{print $3}' "/proc/${process_id}/stat" 2>/dev/null || true)"
    [[ "${process_state}" != "Z" ]] || return 1
  fi
  return 0
}

wait_for_process_exit() {
  local process_id="$1"
  local attempts="$2"
  for _ in $(seq 1 "${attempts}"); do
    pid_is_running "${process_id}" || return 0
    sleep 0.1
  done
  return 1
}

terminate_process() {
  local process_id="$1"
  local description="$2"
  pid_is_running "${process_id}" || {
    wait "${process_id}" 2>/dev/null || true
    return 0
  }

  kill -INT "${process_id}" 2>/dev/null || true
  if ! wait_for_process_exit "${process_id}" 30; then
    echo "  ${description} 未响应 SIGINT，升级为 SIGTERM ..."
    kill -TERM "${process_id}" 2>/dev/null || true
  fi
  if ! wait_for_process_exit "${process_id}" 20; then
    echo "  ${description} 未响应 SIGTERM，升级为 SIGKILL ..."
    kill -KILL "${process_id}" 2>/dev/null || true
  fi
  wait_for_process_exit "${process_id}" 10 || {
    echo "  ✗ 无法停止 ${description}（PID ${process_id}）" >&2
    return 1
  }
  wait "${process_id}" 2>/dev/null || true
}

tf_is_available() {
  local parent_frame="$1"
  local child_frame="$2"
  local output
  output="$(
    timeout 3 rosrun tf tf_echo "${parent_frame}" "${child_frame}" 2>&1 \
      || true
  )"
  grep -q -- "- Translation:" <<<"${output}"
}

cleanup() {
  local exit_code=$?
  trap - EXIT HUP INT TERM

  if [[ -n "${SERVO_PID}" ]] && kill -0 "${SERVO_PID}" 2>/dev/null; then
    echo
    echo "正在停止 OBB 视觉伺服服务 ..."
    terminate_process "${SERVO_PID}" "OBB 视觉伺服服务" || true
  fi
  if [[ -n "${SERVICE_PID}" ]] && kill -0 "${SERVICE_PID}" 2>/dev/null; then
    echo
    echo "正在停止 OBB 推理服务 ..."
    terminate_process "${SERVICE_PID}" "OBB 推理服务" || true
  fi
  if [[ "${TF_OWNED}" == "1" && -n "${TF_PID}" ]] \
      && kill -0 "${TF_PID}" 2>/dev/null; then
    echo "正在停止本场景启动的头部相机 TF 桥接 ..."
    timeout 3 rosnode kill "${TF_NODE_NAME}" >/dev/null 2>&1 || true
    terminate_process "${TF_PID}" "头部相机 TF 桥接" || true
  fi
  if [[ "${CAMERA_OWNED}" == "1" && -n "${CAMERA_PID}" ]] \
      && kill -0 "${CAMERA_PID}" 2>/dev/null; then
    echo "正在停止本场景启动的公共头部相机 ..."
    terminate_process "${CAMERA_PID}" "公共头部相机" || true
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
export PYTHONPATH="${REPO_ROOT}:${REPO_ROOT}/infrastructure/ros_packages/devel/lib/python3/dist-packages:${PYTHONPATH:-}"

ENGINE_PATH="$(resolve_repo_path "$(board_get engine_path)")"
SERVICE_NAME="$(board_get service_name)"
WARMUP="$(board_get warmup)"
PYTHON_BIN="$(board_get python)"
if [[ -z "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi
MAX_FRAME_AGE="$(board_get max_frame_age)"
SERVO_SERVICE_NAME="$(board_get servo_service_name)"
SERVO_SCRIPT="$(resolve_repo_path "$(board_get servo_script)")"
SERVO_RESULT_TOPIC="$(board_get servo_result_topic)"
SERVO_RESULT_FRAME="$(board_get servo_result_frame)"
SERVO_CAPTURE_DIR_VALUE="$(board_get servo_capture_dir)"
SERVO_CAPTURE_DIR=""
if [[ -n "${SERVO_CAPTURE_DIR_VALUE}" ]]; then
  SERVO_CAPTURE_DIR="$(resolve_repo_path "${SERVO_CAPTURE_DIR_VALUE}")"
fi
SERVO_CAPTURE_INTERVAL="$(board_get servo_capture_interval)"
START_CAMERA_IF_MISSING="$(board_get start_camera_if_missing)"
CAMERA_LAUNCH="$(resolve_repo_path "$(board_get camera_launch)")"
COLOR_TOPIC="$(board_get color_topic)"
DEPTH_TOPIC="$(board_get depth_topic)"
CAMERA_STARTUP_TIMEOUT="$(board_get camera_startup_timeout)"
COLOR_WIDTH="$(board_get color_width)"
COLOR_HEIGHT="$(board_get color_height)"
COLOR_FPS="$(board_get color_fps)"
DEPTH_WIDTH="$(board_get depth_width)"
DEPTH_HEIGHT="$(board_get depth_height)"
DEPTH_FPS="$(board_get depth_fps)"
PUBLISH_TF_BRIDGE="$(board_get publish_tf_bridge)"
TF_PARENT_FRAME="$(board_get tf_parent_frame)"
TF_CHILD_FRAME="$(board_get tf_child_frame)"
TF_TRANSFORM_ARGS="$(board_get tf_transform_args)"
TF_STARTUP_TIMEOUT="$(board_get tf_startup_timeout)"
read -r -a TF_ARGS <<<"${TF_TRANSFORM_ARGS}"

echo "[1/6] 检查配置与运行环境 ..."
[[ -f "${BOARD}" ]] || { echo "  ✗ 找不到 ${BOARD}" >&2; exit 2; }
[[ -f "${ENGINE_PATH}" ]] || {
  echo "  ✗ 找不到 TensorRT engine：${ENGINE_PATH}" >&2
  exit 2
}
[[ "${WARMUP}" =~ ^[0-9]+$ ]] || {
  echo "  ✗ warmup 必须是非负整数，当前值：${WARMUP}" >&2
  exit 2
}
[[ "${SERVICE_NAME}" = /* ]] || {
  echo "  ✗ service_name 必须以 / 开头，当前值：${SERVICE_NAME}" >&2
  exit 2
}
[[ "${SERVO_SERVICE_NAME}" = /* ]] || {
  echo "  ✗ servo_service_name 必须以 / 开头，当前值：${SERVO_SERVICE_NAME}" >&2
  exit 2
}
[[ "${SERVO_RESULT_TOPIC}" = /* ]] || {
  echo "  ✗ servo_result_topic 必须以 / 开头，当前值：${SERVO_RESULT_TOPIC}" >&2
  exit 2
}
[[ -f "${SERVO_SCRIPT}" ]] || {
  echo "  ✗ 找不到视觉伺服服务端脚本：${SERVO_SCRIPT}" >&2
  exit 2
}
[[ -n "${SERVO_RESULT_FRAME}" && "${SERVO_RESULT_FRAME}" != *[[:space:]]* ]] || {
  echo "  ✗ servo_result_frame 不能为空或包含空白字符" >&2
  exit 2
}
python3 - "${SERVO_CAPTURE_INTERVAL}" <<'PY' || {
import math
import sys
value = float(sys.argv[1])
raise SystemExit(0 if math.isfinite(value) and value > 0.0 else 1)
PY
  echo "  ✗ servo_capture_interval 必须是正有限数，当前值：${SERVO_CAPTURE_INTERVAL}" >&2
  exit 2
}
[[ "${START_CAMERA_IF_MISSING}" =~ ^(true|false)$ ]] || {
  echo "  ✗ start_camera_if_missing 必须是 true 或 false" >&2
  exit 2
}
[[ "${PUBLISH_TF_BRIDGE}" =~ ^(true|false)$ ]] || {
  echo "  ✗ publish_tf_bridge 必须是 true 或 false" >&2
  exit 2
}
[[ -f "${CAMERA_LAUNCH}" ]] || {
  echo "  ✗ 找不到公共相机 launch：${CAMERA_LAUNCH}" >&2
  exit 2
}
[[ "${CAMERA_STARTUP_TIMEOUT}" =~ ^[1-9][0-9]*$ ]] || {
  echo "  ✗ camera_startup_timeout 必须是正整数" >&2
  exit 2
}
[[ "${TF_STARTUP_TIMEOUT}" =~ ^[1-9][0-9]*$ ]] || {
  echo "  ✗ tf_startup_timeout 必须是正整数" >&2
  exit 2
}
[[ -n "${TF_PARENT_FRAME}" && "${TF_PARENT_FRAME}" != *[[:space:]]* ]] || {
  echo "  ✗ tf_parent_frame 不能为空或包含空白字符" >&2
  exit 2
}
[[ -n "${TF_CHILD_FRAME}" && "${TF_CHILD_FRAME}" != *[[:space:]]* ]] || {
  echo "  ✗ tf_child_frame 不能为空或包含空白字符" >&2
  exit 2
}
[[ "${TF_PARENT_FRAME}" != "${TF_CHILD_FRAME}" ]] || {
  echo "  ✗ TF 父子 frame 不能相同：${TF_PARENT_FRAME}" >&2
  exit 2
}
for value in \
  "${COLOR_WIDTH}" "${COLOR_HEIGHT}" "${COLOR_FPS}" \
  "${DEPTH_WIDTH}" "${DEPTH_HEIGHT}" "${DEPTH_FPS}"; do
  [[ "${value}" =~ ^[1-9][0-9]*$ ]] || {
    echo "  ✗ 相机分辨率和帧率必须是正整数" >&2
    exit 2
  }
done
python3 - "${MAX_FRAME_AGE}" <<'PY'
import sys

if float(sys.argv[1]) <= 0.0:
    raise SystemExit("max_frame_age 必须大于 0")
PY
python3 - "${TF_ARGS[@]}" <<'PY'
import sys

if len(sys.argv) != 7:
    raise SystemExit("tf_transform_args 必须恰好包含 6 个数值")
try:
    [float(value) for value in sys.argv[1:]]
except ValueError as error:
    raise SystemExit(f"tf_transform_args 包含非数值参数：{error}")
PY
if [[ -z "${PYTHON_BIN}" || ! -x "${PYTHON_BIN}" ]]; then
  echo "  ✗ python 不可执行：${PYTHON_BIN}" >&2
  exit 2
fi
if ! rospack find carton_obb >/dev/null 2>&1; then
  echo "  ✗ ROS 找不到 carton_obb 包。" >&2
  echo "    先执行：cd ${REPO_ROOT}/infrastructure/ros_packages && catkin build box_detection_msgs carton_obb" >&2
  exit 2
fi
if ! python3 -c "from box_detection_msgs.srv import InferCartonObb, ServoCartonObb" >/dev/null 2>&1; then
  echo "  ✗ InferCartonObb/ServoCartonObb 服务类型尚未生成。" >&2
  echo "    先执行：cd ${REPO_ROOT}/infrastructure/ros_packages && catkin build box_detection_msgs carton_obb" >&2
  exit 2
fi
if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  echo "  ✗ ROS_MASTER_URI 未设置，请先启动 roscore 或 source 机器人 bringup。" >&2
  exit 2
fi
if ! rosnode list >/dev/null 2>&1; then
  echo "  ✗ 无法连接 ROS master：${ROS_MASTER_URI}" >&2
  exit 2
fi

echo "  ✓ engine=${ENGINE_PATH}"
echo "  ✓ service=${SERVICE_NAME}"
echo "  ✓ servo_service=${SERVO_SERVICE_NAME}"
echo "  ✓ warmup=${WARMUP}"
echo "  ✓ color_topic=${COLOR_TOPIC}"
echo "  ✓ depth_topic=${DEPTH_TOPIC}"
echo "  ✓ start_camera_if_missing=${START_CAMERA_IF_MISSING}"
echo "  ✓ publish_tf_bridge=${PUBLISH_TF_BRIDGE}"
echo "  ✓ tf_bridge=${TF_PARENT_FRAME} -> ${TF_CHILD_FRAME}"
echo "  ✓ ROS_MASTER_URI=${ROS_MASTER_URI}"

if [[ "${DRY_RUN}" == "1" ]]; then
  echo
  echo "--dry-run：检查通过，不启动服务。"
  echo "相机缺失时执行："
  printf 'roslaunch %q color_width:=%q color_height:=%q color_fps:=%q depth_width:=%q depth_height:=%q depth_fps:=%q\n' \
    "${CAMERA_LAUNCH}" "${COLOR_WIDTH}" "${COLOR_HEIGHT}" "${COLOR_FPS}" \
    "${DEPTH_WIDTH}" "${DEPTH_HEIGHT}" "${DEPTH_FPS}"
  if [[ "${PUBLISH_TF_BRIDGE}" == "true" ]]; then
    echo "TF 桥接缺失时执行："
    printf 'rosrun tf2_ros static_transform_publisher'
    printf ' %q' \
      "${TF_ARGS[@]}" "${TF_PARENT_FRAME}" "${TF_CHILD_FRAME}" \
      "__name:=${TF_NODE_NAME#/}"
    printf '\n'
  fi
  echo "推理服务命令："
  printf 'roslaunch carton_obb carton_obb_trt_service.launch engine_path:=%q service_name:=%q warmup:=%q python:=%q image_topic:=%q camera_timeout:=%q max_frame_age:=%q\n' \
    "${ENGINE_PATH}" "${SERVICE_NAME}" "${WARMUP}" "${PYTHON_BIN}" \
    "${COLOR_TOPIC}" "${CAMERA_STARTUP_TIMEOUT}" "${MAX_FRAME_AGE}"
  echo "视觉伺服服务命令："
  printf 'python3 %q --serve-servo --inference-service %q --servo-service-name %q --topic %q --depth-topic %q --servo-result-topic %q --servo-result-frame %q --servo-capture-dir %q --servo-capture-interval %q --no-display\n' \
    "${SERVO_SCRIPT}" "${SERVICE_NAME}" "${SERVO_SERVICE_NAME}" \
    "${COLOR_TOPIC}" "${DEPTH_TOPIC}" "${SERVO_RESULT_TOPIC}" \
    "${SERVO_RESULT_FRAME}" "${SERVO_CAPTURE_DIR}" \
    "${SERVO_CAPTURE_INTERVAL}"
  exit 0
fi

LOG_DIR="${REPO_ROOT}/log/carton_obb_trt_service"
mkdir -p "${LOG_DIR}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
CAMERA_LOG="${LOG_DIR}/camera_${TIMESTAMP}.log"
TF_LOG="${LOG_DIR}/tf_bridge_${TIMESTAMP}.log"
SERVICE_LOG="${LOG_DIR}/service_${TIMESTAMP}.log"
SERVO_LOG="${LOG_DIR}/servo_${TIMESTAMP}.log"

echo "[2/6] 检查公共头部相机 ..."
COLOR_HAS_PUBLISHER=0
DEPTH_HAS_PUBLISHER=0
topic_has_publishers "${COLOR_TOPIC}" && COLOR_HAS_PUBLISHER=1
topic_has_publishers "${DEPTH_TOPIC}" && DEPTH_HAS_PUBLISHER=1

if [[ "${COLOR_HAS_PUBLISHER}" == "1" || "${DEPTH_HAS_PUBLISHER}" == "1" ]]; then
  if [[ "${COLOR_HAS_PUBLISHER}" != "1" || "${DEPTH_HAS_PUBLISHER}" != "1" ]]; then
    echo "  ✗ 检测到不完整的公共 RGB-D 相机发布者，不会重复启动硬件。" >&2
    echo "    color publisher=${COLOR_HAS_PUBLISHER}, depth publisher=${DEPTH_HAS_PUBLISHER}" >&2
    exit 2
  fi
  if ! wait_for_message "${COLOR_TOPIC}" "${CAMERA_STARTUP_TIMEOUT}"; then
    echo "  ✗ ${COLOR_TOPIC} 有发布者但未收到图像" >&2
    exit 2
  fi
  if ! wait_for_message "${DEPTH_TOPIC}" "${CAMERA_STARTUP_TIMEOUT}"; then
    echo "  ✗ ${DEPTH_TOPIC} 有发布者但未收到图像" >&2
    exit 2
  fi
  echo "  ✓ 复用现有公共头部相机，本场景退出时不会关闭它"
else
  if [[ "${START_CAMERA_IF_MISSING}" != "true" ]]; then
    echo "  ✗ 公共头部相机未运行，且禁止自动启动" >&2
    exit 2
  fi
  if ! rospack find orbbec_camera >/dev/null 2>&1; then
    echo "  ✗ ROS 找不到 orbbec_camera 包" >&2
    exit 2
  fi

  echo "  未检测到相机发布者，正在启动公共 Orbbec RGB-D ..."
  roslaunch "${CAMERA_LAUNCH}" \
    "color_width:=${COLOR_WIDTH}" \
    "color_height:=${COLOR_HEIGHT}" \
    "color_fps:=${COLOR_FPS}" \
    "depth_width:=${DEPTH_WIDTH}" \
    "depth_height:=${DEPTH_HEIGHT}" \
    "depth_fps:=${DEPTH_FPS}" \
    > >(tee -a "${CAMERA_LOG}") 2>&1 &
  CAMERA_PID=$!
  CAMERA_OWNED=1

  if ! wait_for_message "${COLOR_TOPIC}" "${CAMERA_STARTUP_TIMEOUT}"; then
    echo "  ✗ 等待头部彩色首帧超时，日志：${CAMERA_LOG}" >&2
    exit 2
  fi
  if ! wait_for_message "${DEPTH_TOPIC}" "${CAMERA_STARTUP_TIMEOUT}"; then
    echo "  ✗ 等待头部深度首帧超时，日志：${CAMERA_LOG}" >&2
    exit 2
  fi
  echo "  ✓ 公共头部相机已就绪，日志：${CAMERA_LOG}"
fi

echo "[3/6] 检查头部相机 TF 桥接 ..."
if [[ "${PUBLISH_TF_BRIDGE}" != "true" ]]; then
  echo "  ✓ 已按配置跳过 TF 桥接"
elif tf_is_available "${TF_PARENT_FRAME}" "${TF_CHILD_FRAME}"; then
  echo "  ✓ 复用现有 TF：${TF_PARENT_FRAME} -> ${TF_CHILD_FRAME}"
else
  if ! rospack find tf2_ros >/dev/null 2>&1; then
    echo "  ✗ ROS 找不到 tf2_ros 包" >&2
    exit 2
  fi
  echo "  正在启动静态 TF：${TF_PARENT_FRAME} -> ${TF_CHILD_FRAME}"
  rosrun tf2_ros static_transform_publisher \
    "${TF_ARGS[@]}" "${TF_PARENT_FRAME}" "${TF_CHILD_FRAME}" \
    "__name:=${TF_NODE_NAME#/}" \
    > >(tee -a "${TF_LOG}") 2>&1 &
  TF_PID=$!
  TF_OWNED=1

  TF_READY=0
  TF_DEADLINE=$((SECONDS + TF_STARTUP_TIMEOUT))
  while (( SECONDS < TF_DEADLINE )); do
    if ! kill -0 "${TF_PID}" 2>/dev/null; then
      echo "  ✗ 静态 TF 进程提前退出，日志：${TF_LOG}" >&2
      exit 2
    fi
    if tf_is_available "${TF_PARENT_FRAME}" "${TF_CHILD_FRAME}"; then
      TF_READY=1
      break
    fi
  done
  if [[ "${TF_READY}" != "1" ]]; then
    echo "  ✗ 等待静态 TF 超时：${TF_PARENT_FRAME} -> ${TF_CHILD_FRAME}" >&2
    echo "    日志：${TF_LOG}" >&2
    exit 2
  fi
  echo "  ✓ TF 桥接已就绪，日志：${TF_LOG}"
fi

echo "[4/6] 清理上一轮残留 OBB 节点 ..."
for stale_node in "${SERVO_NODE_NAME}" "${NODE_NAME}"; do
  stale_info=""
  stale_pid=""
  if ! rosnode list 2>/dev/null | grep -qx "${stale_node}"; then
    continue
  fi
  if ! stale_info="$(rosnode info "${stale_node}" 2>/dev/null)"; then
    echo "  检测到 ROS Master 残留注册，正在清理：${stale_node}"
    printf 'y\n' | rosnode cleanup >/dev/null 2>&1 || true
    if ! rosnode list 2>/dev/null | grep -qx "${stale_node}"; then
      echo "  ✓ 已清理失效注册：${stale_node}"
      continue
    fi
  else
    stale_pid="$(
      awk '/^[[:space:]]*Pid:[[:space:]]*[0-9]+/ {print $2; exit}' \
        <<<"${stale_info}"
    )"
  fi
  rosnode kill "${stale_node}" >/dev/null 2>&1 || true
  for _ in $(seq 1 20); do
    if ! rosnode list 2>/dev/null | grep -qx "${stale_node}"; then
      break
    fi
    sleep 0.25
  done
  if rosnode list 2>/dev/null | grep -qx "${stale_node}"; then
    expected_command="carton_obb_trt_service"
    if [[ "${stale_node}" == "${SERVO_NODE_NAME}" ]]; then
      expected_command="realtime_carton_obb_trt.py"
    fi
    if [[ "${stale_pid}" =~ ^[0-9]+$ ]] \
        && [[ -r "/proc/${stale_pid}/cmdline" ]] \
        && tr '\0' ' ' <"/proc/${stale_pid}/cmdline" \
          | grep -Fq "${expected_command}"; then
      echo "  rosnode kill 未停止进程，正在按 PID 清理：${stale_pid}"
      terminate_process "${stale_pid}" "${stale_node}" || exit 2
      printf 'y\n' | rosnode cleanup >/dev/null 2>&1 || true
    fi
    if rosnode list 2>/dev/null | grep -qx "${stale_node}"; then
      echo "  ✗ 残留节点无法停止：${stale_node}" >&2
      exit 2
    fi
  fi
  echo "  ✓ 旧节点已停止：${stale_node}"
done

echo "[5/6] 启动共享 TensorRT 推理服务 ..."
echo "  节点：${NODE_NAME}"
echo "  服务：${SERVICE_NAME}"
echo "  最新帧：${COLOR_TOPIC}（最大接收年龄 ${MAX_FRAME_AGE}s）"
echo "  日志：${SERVICE_LOG}"
echo "  结束：Ctrl+C"
echo

roslaunch carton_obb carton_obb_trt_service.launch \
  "engine_path:=${ENGINE_PATH}" \
  "service_name:=${SERVICE_NAME}" \
  "warmup:=${WARMUP}" \
  "python:=${PYTHON_BIN}" \
  "image_topic:=${COLOR_TOPIC}" \
  "camera_timeout:=${CAMERA_STARTUP_TIMEOUT}" \
  "max_frame_age:=${MAX_FRAME_AGE}" \
  > >(tee -a "${SERVICE_LOG}") 2>&1 &
SERVICE_PID=$!
if ! wait_for_service \
    "${SERVICE_NAME}" "${CAMERA_STARTUP_TIMEOUT}" "${SERVICE_PID}"; then
  echo "  ✗ 等待推理服务超时或进程提前退出，日志：${SERVICE_LOG}" >&2
  exit 2
fi
echo "  ✓ 推理服务已就绪"

echo
echo "[6/6] 启动阻塞式 OBB 视觉伺服服务 ..."
echo "  节点：${SERVO_NODE_NAME}"
echo "  服务：${SERVO_SERVICE_NAME}"
echo "  响应坐标系：${SERVO_RESULT_FRAME}"
echo "  结果话题：${SERVO_RESULT_TOPIC}"
echo "  抓取图像：${SERVO_CAPTURE_DIR:-关闭}"
echo "  日志：${SERVO_LOG}"
echo "  结束：Ctrl+C"
echo

python3 -u "${SERVO_SCRIPT}" \
  --serve-servo \
  --inference-service "${SERVICE_NAME}" \
  --servo-service-name "${SERVO_SERVICE_NAME}" \
  --topic "${COLOR_TOPIC}" \
  --depth-topic "${DEPTH_TOPIC}" \
  --servo-result-topic "${SERVO_RESULT_TOPIC}" \
  --servo-result-frame "${SERVO_RESULT_FRAME}" \
  --servo-capture-dir "${SERVO_CAPTURE_DIR}" \
  --servo-capture-interval "${SERVO_CAPTURE_INTERVAL}" \
  --no-display \
  > >(tee -a "${SERVO_LOG}") 2>&1 &
SERVO_PID=$!
if ! wait_for_service \
    "${SERVO_SERVICE_NAME}" "${CAMERA_STARTUP_TIMEOUT}" "${SERVO_PID}"; then
  echo "  ✗ 等待视觉伺服服务超时或进程提前退出，日志：${SERVO_LOG}" >&2
  exit 2
fi
echo "  ✓ 视觉伺服服务已就绪，可调用 ${SERVO_SERVICE_NAME}"

wait -n "${SERVICE_PID}" "${SERVO_PID}"
