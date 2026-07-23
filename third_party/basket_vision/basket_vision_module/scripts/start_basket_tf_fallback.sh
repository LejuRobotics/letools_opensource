#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KUAVO_STUDIO_DIR="${KUAVO_STUDIO_DIR:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"

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

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://kuavo_master:11311}"
export ROS_IP="${ROS_IP:-192.168.26.12}"

LOG_ROOT="${BASKET_VISION_LOG_ROOT:-${KUAVO_STUDIO_DIR}/third_party/basket_vision/logs}"
TF_LOG_DIR="${BASKET_VISION_TF_LOG_DIR:-${LOG_ROOT}/tf}"
mkdir -p "${TF_LOG_DIR}"

VERIFY_TARGET="${BASKET_CAMERA_VERIFY_TARGET:-base_link}"
VERIFY_SOURCE="${BASKET_CAMERA_VERIFY_SOURCE:-camera_color_optical_frame}"
VERIFY_TIMEOUT="${BASKET_CAMERA_VERIFY_TIMEOUT:-5}"

NODE_NAME="${BASKET_CAMERA_TF_NODE_NAME:-basket_camera_base_link_to_camera_link_tf}"
PARENT_FRAME="${BASKET_CAMERA_PARENT_FRAME:-base_link}"
CHILD_FRAME="${BASKET_CAMERA_CHILD_FRAME:-camera_link}"

# Fallback from the upper-machine biped_s42 URDF zero pose:
# base_link -> neck_motor: xyz=(-0.0185, 0, 0.6014), rpy=(0, 0, 0)
# head -> camera: xyz=(0.0967509784707853, 0.0175003248712456, 0.125953265112721),
# rpy=(0, 0.488692190558413, 0). The old launch aligns camera -> camera_link
# with identity. tf2_ros static_transform_publisher argument order is:
# x y z yaw pitch roll parent child.
TF_X="${BASKET_CAMERA_TF_X:-0.0782509784707853}"
TF_Y="${BASKET_CAMERA_TF_Y:-0.0175003248712456}"
TF_Z="${BASKET_CAMERA_TF_Z:-0.727353265112721}"
TF_YAW="${BASKET_CAMERA_TF_YAW:-0}"
TF_PITCH="${BASKET_CAMERA_TF_PITCH:-0.488692190558413}"
TF_ROLL="${BASKET_CAMERA_TF_ROLL:-0}"

CHECK_LOG="${TF_LOG_DIR}/check_${NODE_NAME}.log"
RUN_LOG="${TF_LOG_DIR}/${NODE_NAME}.log"

transform_available() {
  local target_frame="$1"
  local source_frame="$2"
  local output_file="$3"

  timeout "${VERIFY_TIMEOUT}" rosrun tf tf_echo "${target_frame}" "${source_frame}" >"${output_file}" 2>&1 || true
  grep -q "Translation:" "${output_file}"
}

start_static_tf() {
  local node_name="$1"
  local log_file="$2"
  local x="$3"
  local y="$4"
  local z="$5"
  local yaw="$6"
  local pitch="$7"
  local roll="$8"
  local parent_frame="$9"
  local child_frame="${10}"

  if rosnode list 2>/dev/null | grep -qx "/${node_name}"; then
    echo "[tf] killing previous fallback node: /${node_name}"
    rosnode kill "/${node_name}" >/dev/null 2>&1 || true
    sleep 1
  fi

  nohup rosrun tf2_ros static_transform_publisher \
    "${x}" "${y}" "${z}" \
    "${yaw}" "${pitch}" "${roll}" \
    "${parent_frame}" "${child_frame}" "__name:=${node_name}" \
    >"${log_file}" 2>&1 &

  echo "[tf] fallback pid=$!"
  echo "[tf] fallback log=${log_file}"
}

echo "[tf] checking ${VERIFY_TARGET} <- ${VERIFY_SOURCE}"
if transform_available "${VERIFY_TARGET}" "${VERIFY_SOURCE}" "${CHECK_LOG}"; then
  echo "[tf] existing transform is available, fallback not started"
  tail -20 "${CHECK_LOG}" || true
else

  echo "[tf] missing ${VERIFY_TARGET} <- ${VERIFY_SOURCE}, starting fallback"
  echo "[tf] fallback: ${PARENT_FRAME} -> ${CHILD_FRAME}"
  echo "[tf] args: ${TF_X} ${TF_Y} ${TF_Z} ${TF_YAW} ${TF_PITCH} ${TF_ROLL} ${PARENT_FRAME} ${CHILD_FRAME}"

  start_static_tf \
    "${NODE_NAME}" "${RUN_LOG}" \
    "${TF_X}" "${TF_Y}" "${TF_Z}" \
    "${TF_YAW}" "${TF_PITCH}" "${TF_ROLL}" \
    "${PARENT_FRAME}" "${CHILD_FRAME}"

  sleep 1

  if transform_available "${VERIFY_TARGET}" "${VERIFY_SOURCE}" "${CHECK_LOG}"; then
    echo "[tf] transform is available after fallback"
    tail -20 "${CHECK_LOG}" || true
  else
    echo "[tf] error: transform is still unavailable after fallback"
    cat "${CHECK_LOG}" || true
    exit 1
  fi
fi
