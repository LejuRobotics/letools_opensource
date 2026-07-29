#!/usr/bin/env bash
# Create the isolated Basket Vision Python runtime validated for JetPack 5.1.4.
# This script never installs apt packages or modifies the system Python.

set -euo pipefail

if [ "$#" -gt 2 ]; then
  echo "Usage: $0 [venv-directory] [wheel-directory]" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BASKET_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BASKET_ROOT="$(cd "${BASKET_VISION_ROOT:-${DEFAULT_BASKET_ROOT}}" && pwd)"
GDRN_ROOT="${BASKET_ROOT}/basket_gdrnpp"
ENV_DIR="${1:-${BASKET_VISION_ENV:-${GDRN_ROOT}/.venv}}"
WHEEL_DIR="${2:-${BASKET_VISION_WHEEL_DIR:-${BASKET_ROOT}/wheels}}"
SYSTEM_PYTHON="${BASKET_VISION_SYSTEM_PYTHON:-/usr/bin/python3.8}"
LOCK_FILE="${GDRN_ROOT}/requirements/jetpack5_py38_runtime.txt"
WHEEL_HASH_FILE="${GDRN_ROOT}/requirements/jetpack5_py38_binary_wheels.sha256"

TORCH_WHEEL="torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
TORCHVISION_WHEEL="torchvision-0.15.1-cp38-cp38-linux_aarch64.whl"
DETECTRON2_WHEEL="detectron2-0.6-cp38-cp38-linux_aarch64.whl"

fail() {
  printf '[FAIL] %s\n' "$1" >&2
  exit 1
}

info() {
  printf '[INFO] %s\n' "$1"
}

require_file() {
  [ -f "$1" ] || fail "required file is missing: $1"
}

[ "$(uname -m)" = "aarch64" ] \
  || fail "this profile requires aarch64 (actual: $(uname -m))"

OS_VERSION="$(. /etc/os-release 2>/dev/null; printf '%s' "${VERSION_ID:-unknown}")"
[ "${OS_VERSION}" = "20.04" ] \
  || fail "this profile requires Ubuntu 20.04 (actual: ${OS_VERSION})"

JETPACK_VERSION="$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null || true)"
[ "${JETPACK_VERSION}" = "5.1.4-b17" ] \
  || fail "this profile requires JetPack 5.1.4-b17 (actual: ${JETPACK_VERSION:-not installed})"

L4T_VERSION="$(dpkg-query -W -f='${Version}' nvidia-l4t-core 2>/dev/null || true)"
[[ "${L4T_VERSION}" == 35.6.0-* ]] \
  || fail "this profile requires L4T 35.6.0 (actual: ${L4T_VERSION:-not installed})"

[ -x "${SYSTEM_PYTHON}" ] \
  || fail "Python 3.8 is not executable: ${SYSTEM_PYTHON}"
"${SYSTEM_PYTHON}" -c \
  'import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 8) else 1)' \
  || fail "${SYSTEM_PYTHON} is not Python 3.8"
"${SYSTEM_PYTHON}" -c 'import ensurepip' 2>/dev/null \
  || fail "Python ensurepip is unavailable; install python3.8-venv first"

ROS_SETUP="/opt/ros/noetic/setup.bash"
CV_BRIDGE_SO="/opt/ros/noetic/lib/python3/dist-packages/cv_bridge/boost/cv_bridge_boost.so"
require_file "${ROS_SETUP}"
require_file "${CV_BRIDGE_SO}"
ldd "${CV_BRIDGE_SO}" 2>/dev/null | grep -q 'libpython3\.8' \
  || fail "cv_bridge is not linked to libpython3.8"
ldd "${CV_BRIDGE_SO}" 2>/dev/null | grep -q 'libboost_python38' \
  || fail "cv_bridge is not linked to libboost_python38"

require_file "${LOCK_FILE}"
require_file "${WHEEL_HASH_FILE}"
require_file "${WHEEL_DIR}/${TORCH_WHEEL}"
require_file "${WHEEL_DIR}/${TORCHVISION_WHEEL}"
require_file "${WHEEL_DIR}/${DETECTRON2_WHEEL}"

info "verifying Jetson binary wheels"
(
  cd "${WHEEL_DIR}"
  sha256sum -c "${WHEEL_HASH_FILE}"
) || fail "Jetson binary wheel SHA256 validation failed"

if [ -e "${ENV_DIR}" ]; then
  [ -f "${ENV_DIR}/pyvenv.cfg" ] \
    || fail "target exists but is not a venv: ${ENV_DIR}"
  [ -x "${ENV_DIR}/bin/python" ] \
    || fail "venv Python is not executable: ${ENV_DIR}/bin/python"
  info "reusing existing venv: ${ENV_DIR}"
else
  info "creating isolated venv: ${ENV_DIR}"
  "${SYSTEM_PYTHON}" -m venv "${ENV_DIR}"
fi

if [ ! -f "${ENV_DIR}/bin/activate" ]; then
  info "repairing missing venv activation scripts"
  "${SYSTEM_PYTHON}" -m venv --without-pip "${ENV_DIR}"
fi
[ -f "${ENV_DIR}/bin/activate" ] \
  || fail "venv activation script is missing: ${ENV_DIR}/bin/activate"

grep -Eqi '^include-system-site-packages[[:space:]]*=[[:space:]]*false$' \
  "${ENV_DIR}/pyvenv.cfg" \
  || fail "venv must be created without --system-site-packages: ${ENV_DIR}"

ENV_PYTHON="${ENV_DIR}/bin/python"
"${ENV_PYTHON}" -c \
  'import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 8) and sys.prefix != sys.base_prefix else 1)' \
  || fail "target is not an isolated Python 3.8 venv: ${ENV_DIR}"

if ! "${ENV_PYTHON}" -m pip --version >/dev/null 2>&1; then
  info "bootstrapping pip in the existing venv"
  "${ENV_PYTHON}" -m ensurepip --upgrade
fi

export PYTHONNOUSERSITE=1
export PIP_DISABLE_PIP_VERSION_CHECK=1

info "installing pinned packaging tools"
"${ENV_PYTHON}" -m pip install --upgrade \
  pip==24.3.1 \
  setuptools==75.3.2 \
  wheel==0.45.1

info "installing validated PyTorch and TorchVision wheels"
"${ENV_PYTHON}" -m pip install --no-deps \
  "${WHEEL_DIR}/${TORCH_WHEEL}" \
  "${WHEEL_DIR}/${TORCHVISION_WHEEL}"

info "installing the pinned Python runtime"
"${ENV_PYTHON}" -m pip install -r "${LOCK_FILE}"

info "installing the validated Detectron2 wheel"
"${ENV_PYTHON}" -m pip install --no-deps \
  "${WHEEL_DIR}/${DETECTRON2_WHEEL}"

"${ENV_PYTHON}" - <<'PY'
import importlib.metadata as metadata
import site
import sys

expected = {
    "torch": "2.0.0+nv23.5",
    "torchvision": "0.15.1",
    "detectron2": "0.6",
}
for distribution, version in expected.items():
    actual = metadata.version(distribution)
    if actual != version:
        raise SystemExit(f"{distribution}: expected {version}, got {actual}")
if sys.version_info[:2] != (3, 8) or sys.prefix == sys.base_prefix:
    raise SystemExit("installer did not produce an isolated Python 3.8 venv")
if site.ENABLE_USER_SITE:
    raise SystemExit("user-site packages are unexpectedly enabled")
PY

printf '\n[PASS] Basket Vision Python environment installed\n'
printf 'venv: %s\n' "${ENV_DIR}"
printf 'next: build basket_vision_ws, then run verify_jetpack5_py38_runtime.sh\n'
