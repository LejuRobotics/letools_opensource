#!/usr/bin/env bash

set -eo pipefail

export LD_LIBRARY_PATH="/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
export LD_PRELOAD="/lib/aarch64-linux-gnu/libGLdispatch.so.0${LD_PRELOAD:+:${LD_PRELOAD}}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KUAVO_STUDIO_DIR="${KUAVO_STUDIO_DIR:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
BASKET_GDRNPP_DIR="${KUAVO_STUDIO_DIR}/third_party/basket_vision/basket_gdrnpp"

export TORCH_HOME="${TORCH_HOME:-${KUAVO_STUDIO_DIR}/third_party/basket_vision/torch}"
export PYTHONPATH="${BASKET_GDRNPP_DIR}:${BASKET_GDRNPP_DIR}/detectron2:${PYTHONPATH:-}"

WAIT_TOPIC="${1:-/camera/color/image_raw}"
ENV_NAME="${2:-gdrn}"
PYTHON_SCRIPT="${3:-${KUAVO_STUDIO_DIR}/third_party/basket_vision/basket_gdrnpp/core/gdrn_modeling/demo/inference_service_vis_2_mult_inst_10_shared.py}"
READY_SERVICE_1="${4:-/infer_basket_pose}"
READY_SERVICE_2="${5:-/infer_top_basket_ids}"
READY_TIMEOUT_SEC="${6:-180}"
UV_ACTIVATE_SCRIPT="${UV_ACTIVATE_SCRIPT:-${KUAVO_STUDIO_DIR}/third_party/basket_vision/basket_gdrnpp/${ENV_NAME}/bin/activate}"
BASKET_BOX_CONFIG_YAML="${BASKET_BOX_CONFIG_YAML:-core/gdrn_modeling/demo/box_configs/basket_5.yaml}"

RUN_ID="${BASKET_VISION_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
SERVICE_NAME="${BASKET_VISION_SERVICE_NAME:-gdrn_inference}"
LOG_ROOT="${BASKET_VISION_LOG_ROOT:-${KUAVO_STUDIO_DIR}/third_party/basket_vision/logs}"
RUN_DIR="${BASKET_VISION_RUN_DIR:-${LOG_ROOT}/${SERVICE_NAME}/${RUN_ID}}"
IMAGE_DIR="${BASKET_VISION_IMAGE_DIR:-${RUN_DIR}/images}"
INTERNAL_LOG_DIR="${BASKET_VISION_INTERNAL_LOG_DIR:-${RUN_DIR}/internal}"
ROS_LOG_DIR="${ROS_LOG_DIR:-${RUN_DIR}/ros_logs}"
SERVICE_OUTPUT_DIR="${BASKET_VISION_SERVICE_OUTPUT_DIR:-${RUN_DIR}/service_outputs}"
SINGLE_OUTPUT_DIR="${BASKET_VISION_SINGLE_OUTPUT_DIR:-${SERVICE_OUTPUT_DIR}/infer_basket_pose}"
TOP_OUTPUT_DIR="${BASKET_VISION_TOP_OUTPUT_DIR:-${SERVICE_OUTPUT_DIR}/infer_top_basket_ids}"
START_LOG_FILE="${RUN_DIR}/start_gdrn_inference.log"

mkdir -p "${IMAGE_DIR}" "${INTERNAL_LOG_DIR}" "${ROS_LOG_DIR}" "${SINGLE_OUTPUT_DIR}" "${TOP_OUTPUT_DIR}"

export BASKET_VISION_RUN_ID="${RUN_ID}"
export BASKET_VISION_SERVICE_NAME="${SERVICE_NAME}"
export BASKET_VISION_LOG_ROOT="${LOG_ROOT}"
export BASKET_VISION_RUN_DIR="${RUN_DIR}"
export BASKET_VISION_IMAGE_DIR="${IMAGE_DIR}"
export BASKET_VISION_INTERNAL_LOG_DIR="${INTERNAL_LOG_DIR}"
export BASKET_VISION_SERVICE_OUTPUT_DIR="${SERVICE_OUTPUT_DIR}"
export BASKET_VISION_SINGLE_OUTPUT_DIR="${SINGLE_OUTPUT_DIR}"
export BASKET_VISION_TOP_OUTPUT_DIR="${TOP_OUTPUT_DIR}"
export ROS_LOG_DIR

if [ "${BASKET_VISION_LOG_TEE_ENABLED:-1}" = "1" ] && [ -z "${BASKET_VISION_LOG_TEE_ACTIVE:-}" ]; then
  export BASKET_VISION_LOG_TEE_ACTIVE=1
  exec > >(tee -a "${START_LOG_FILE}") 2>&1
fi

cat > "${RUN_DIR}/run_info.txt" <<EOF
run_id=${RUN_ID}
service_name=${SERVICE_NAME}
kuavo_studio_dir=${KUAVO_STUDIO_DIR}
wait_topic=${WAIT_TOPIC}
python_script=${PYTHON_SCRIPT}
box_config_yaml=${BASKET_BOX_CONFIG_YAML}
ready_service_1=${READY_SERVICE_1}
ready_service_2=${READY_SERVICE_2}
uv_activate_script=${UV_ACTIVATE_SCRIPT}
image_dir=${IMAGE_DIR}
internal_log_dir=${INTERNAL_LOG_DIR}
ros_log_dir=${ROS_LOG_DIR}
service_output_dir=${SERVICE_OUTPUT_DIR}
single_output_dir=${SINGLE_OUTPUT_DIR}
top_output_dir=${TOP_OUTPUT_DIR}
start_log_file=${START_LOG_FILE}
EOF

echo "[gdrn] run id: ${RUN_ID}"
echo "[gdrn] run dir: ${RUN_DIR}"
echo "[gdrn] start log: ${START_LOG_FILE}"
echo "[gdrn] image dir: ${IMAGE_DIR}"
echo "[gdrn] internal log dir: ${INTERNAL_LOG_DIR}"
echo "[gdrn] ros log dir: ${ROS_LOG_DIR}"
echo "[gdrn] service output dir: ${SERVICE_OUTPUT_DIR}"
echo "[gdrn] single output dir: ${SINGLE_OUTPUT_DIR}"
echo "[gdrn] top output dir: ${TOP_OUTPUT_DIR}"

ROS_DISTRO_NAME="${ROS_DISTRO:-noetic}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WORKSPACE_SETUP="${KUAVO_STUDIO_DIR}/infrastructure/ros_packages/devel/setup.bash"
BASKET_VISION_WS_SETUP="${KUAVO_STUDIO_DIR}/third_party/basket_vision/basket_vision_ws/devel/setup.bash"

if [ -f "${ROS_SETUP}" ]; then
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
fi

if [ -f "${WORKSPACE_SETUP}" ]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_SETUP}"
fi

if [ -f "${BASKET_VISION_WS_SETUP}" ]; then
  # shellcheck disable=SC1090
  source "${BASKET_VISION_WS_SETUP}"
fi

echo "[gdrn] waiting for topic: ${WAIT_TOPIC}"

until timeout 30 rostopic echo -n 1 "${WAIT_TOPIC}" >/dev/null 2>&1; do
  echo "[gdrn] still waiting for topic: ${WAIT_TOPIC}"
  sleep 1
done

echo "[gdrn] topic is ready"

echo "[gdrn] starting python script: ${PYTHON_SCRIPT}"

if [ -f "${UV_ACTIVATE_SCRIPT}" ]; then
  echo "[gdrn] using activate script: ${UV_ACTIVATE_SCRIPT}"
  (
    # shellcheck disable=SC1090
    source "${UV_ACTIVATE_SCRIPT}"
    exec python -u "${PYTHON_SCRIPT}" \
      _proj_root:="${KUAVO_STUDIO_DIR}/third_party/basket_vision/basket_gdrnpp" \
      _box_config_yaml:="${BASKET_BOX_CONFIG_YAML}" \
      _save_outputs:=true \
      _inference_image_width:="${BASKET_INFERENCE_IMAGE_WIDTH:-640}" \
      _inference_image_height:="${BASKET_INFERENCE_IMAGE_HEIGHT:-480}" \
      _single_save_dir:="${SINGLE_OUTPUT_DIR}" \
      _top_save_dir:="${TOP_OUTPUT_DIR}"
  ) &
elif command -v uv >/dev/null 2>&1; then
  echo "[gdrn] activate script not found, fallback to uv run"
  (
    cd "$(dirname "${PYTHON_SCRIPT}")"
    uv run python -u "${PYTHON_SCRIPT}"
  ) &
else
  echo "[gdrn] error: no usable Python environment found"
  echo "[gdrn] expected activate script: ${UV_ACTIVATE_SCRIPT}"
  echo "[gdrn] also tried fallback command: uv run"
  exit 1
fi

PY_PID=$!

cleanup() {
  if kill -0 "${PY_PID}" >/dev/null 2>&1; then
    echo "[gdrn] stopping python process ${PY_PID}"
    kill "${PY_PID}" >/dev/null 2>&1 || true
    wait "${PY_PID}" || true
  fi
}

trap cleanup INT TERM

echo "[gdrn] python process started, pid=${PY_PID}"
echo "[gdrn] waiting for services to become ready:"
echo "[gdrn]   - ${READY_SERVICE_1}"
echo "[gdrn]   - ${READY_SERVICE_2}"

ready=false
elapsed=0
while [ "${elapsed}" -lt "${READY_TIMEOUT_SEC}" ]; do
  if ! kill -0 "${PY_PID}" >/dev/null 2>&1; then
    echo "[gdrn] python process exited before reporting ready"
    wait "${PY_PID}"
    exit $?
  fi

  service_1_ready=false
  service_2_ready=false

  if rosservice info "${READY_SERVICE_1}" >/dev/null 2>&1; then
    service_1_ready=true
  fi
  if rosservice info "${READY_SERVICE_2}" >/dev/null 2>&1; then
    service_2_ready=true
  fi

  if [ "${service_1_ready}" = true ] && [ "${service_2_ready}" = true ]; then
    ready=true
    break
  fi

  if [ $((elapsed % 5)) -eq 0 ]; then
    echo "[gdrn] still waiting... ${READY_SERVICE_1}=${service_1_ready}, ${READY_SERVICE_2}=${service_2_ready}, elapsed=${elapsed}s"
  fi

  sleep 1
  elapsed=$((elapsed + 1))
done

if [ "${ready}" = true ]; then
  echo "[gdrn] inference service is ready"
else
  echo "[gdrn] warning: readiness check timed out after ${READY_TIMEOUT_SEC}s, keep process running"
fi

wait "${PY_PID}"
