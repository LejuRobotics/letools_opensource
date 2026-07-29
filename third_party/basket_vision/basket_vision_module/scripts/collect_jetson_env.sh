#!/usr/bin/env bash
# Read-only host fingerprint collection for Basket Vision deployment.
# This script does not install packages, contact a ROS master, access cameras,
# load model weights, or start/stop ROS or robot processes.

set -uo pipefail

if [ "$#" -gt 1 ]; then
  echo "Usage: $0 [python-executable]" >&2
  exit 2
fi

PYTHON_BIN="${1:-$(command -v python3 2>/dev/null || true)}"

section() {
  printf '\n[%s]\n' "$1"
}

section "IDENTITY"
hostnamectl 2>/dev/null || hostname
date -Is

section "OS_KERNEL_ARCH"
cat /etc/os-release 2>/dev/null || true
uname -a
uname -m
getconf GNU_LIBC_VERSION 2>/dev/null || true

section "JETSON_L4T"
cat /etc/nv_tegra_release 2>/dev/null || true
if [ -r /proc/device-tree/model ]; then
  tr -d '\000' </proc/device-tree/model
fi
printf '\n'
apt-cache policy nvidia-jetpack nvidia-l4t-core 2>/dev/null || true

section "CPU_MEMORY_DISK"
lscpu 2>/dev/null || true
free -h 2>/dev/null || true
df -hT / /home /media/data 2>/dev/null || true

section "CUDA_CUDNN_TENSORRT"
if command -v nvcc >/dev/null 2>&1; then
  nvcc --version
elif [ -x /usr/local/cuda/bin/nvcc ]; then
  /usr/local/cuda/bin/nvcc --version
elif [ -x /usr/local/cuda-11.4/bin/nvcc ]; then
  /usr/local/cuda-11.4/bin/nvcc --version
else
  echo "nvcc: not found"
fi
readlink -f /usr/local/cuda 2>/dev/null || true
cat /usr/local/cuda/version.json 2>/dev/null || true
dpkg -l 2>/dev/null \
  | grep -E '^ii[[:space:]]+(nvidia-l4t-(core|cuda|cudnn|tensorrt)|cuda-|libcudnn|libnvinfer)' \
  | head -100 || true

section "COMPILERS"
gcc --version 2>/dev/null | head -1 || true
g++ --version 2>/dev/null | head -1 || true
cmake --version 2>/dev/null | head -1 || true
make --version 2>/dev/null | head -1 || true

section "ROS"
if [ -f /opt/ros/noetic/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  set -u
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-}"
  printf 'ROS_PYTHON_VERSION=%s\n' "${ROS_PYTHON_VERSION:-}"
  rosversion -d 2>/dev/null || true
else
  echo "/opt/ros/noetic/setup.bash: not found"
fi
dpkg -l 2>/dev/null \
  | grep -E '^ii[[:space:]]+ros-noetic-(ros-base|rospy|tf[[:space:]]|tf2-ros|cv-bridge|apriltag-ros|image-transport|sensor-msgs|geometry-msgs)' \
  || true

section "CV_BRIDGE_ABI"
CV_BRIDGE_SO="/opt/ros/noetic/lib/python3/dist-packages/cv_bridge/boost/cv_bridge_boost.so"
if [ -f "${CV_BRIDGE_SO}" ]; then
  ldd "${CV_BRIDGE_SO}" 2>/dev/null \
    | grep -E 'python|boost_python|opencv|not found' || true
else
  echo "${CV_BRIDGE_SO}: not found"
fi

section "PYTHON_COMMANDS"
for name in python python3 python3.8 python3.9 python3.10 pip pip3 uv conda; do
  path="$(command -v "${name}" 2>/dev/null || true)"
  if [ -n "${path}" ]; then
    printf '%s\t%s\t%s\n' "${name}" "${path}" "$(readlink -f "${path}")"
  fi
done

section "SELECTED_PYTHON"
if [ -n "${PYTHON_BIN}" ] && [ -x "${PYTHON_BIN}" ]; then
  "${PYTHON_BIN}" - <<'PY'
import importlib.metadata as metadata
import platform
import site
import sys
import sysconfig

print("version:", sys.version.replace("\n", " "))
print("executable:", sys.executable)
print("prefix:", sys.prefix)
print("base_prefix:", sys.base_prefix)
print("soabi:", sysconfig.get_config_var("SOABI"))
print("machine:", platform.machine())
print("user_site_enabled:", site.ENABLE_USER_SITE)

packages = (
    "numpy",
    "opencv-python",
    "Pillow",
    "scipy",
    "PyYAML",
    "torch",
    "torchvision",
    "pytorch-lightning",
    "mmcv",
    "detectron2",
    "ultralytics",
    "fvcore",
    "iopath",
    "timm",
    "rospkg",
    "catkin-pkg",
)
for package in packages:
    try:
        print(f"{package}={metadata.version(package)}")
    except metadata.PackageNotFoundError:
        print(f"{package}=NOT_INSTALLED")
PY
else
  echo "Python executable is not usable: ${PYTHON_BIN:-<empty>}"
fi

section "VIRTUAL_ENVS"
find /home /media/data -maxdepth 5 -type f -name pyvenv.cfg -print 2>/dev/null || true

printf '\nCollection complete. No deployment pass/fail decision was made.\n'
