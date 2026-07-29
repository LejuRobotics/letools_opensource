#!/usr/bin/env bash
# Python-runtime-only preflight for Basket Vision.
#
# Usage:
#   source /path/to/gdrn/bin/activate
#   bash diagnose_jetson_orin_env.sh
#
# Or pass the Python executable explicitly:
#   bash diagnose_jetson_orin_env.sh /path/to/gdrn/bin/python
#
# This script does not inspect, subscribe to, or wait for ROS topics.  It does
# not start ROS nodes, access cameras, install packages, or build any code.

set -uo pipefail

if [ "$#" -gt 1 ]; then
  echo "Usage: $0 [python-executable]" >&2
  exit 2
fi

if [ "$#" -eq 1 ]; then
  PYTHON_BIN="$1"
elif [ -n "${VIRTUAL_ENV:-}" ]; then
  PYTHON_BIN="$VIRTUAL_ENV/bin/python"
else
  PYTHON_BIN="$(command -v python)"
fi

if [ ! -x "$PYTHON_BIN" ]; then
  echo "[FAIL] Python executable is not usable: $PYTHON_BIN" >&2
  exit 2
fi

if [ -f /opt/ros/noetic/setup.bash ]; then
  # This only adds ROS Python paths; it does not contact a ROS master or start
  # any ROS process.
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
else
  echo "[WARN] /opt/ros/noetic/setup.bash not found; ROS imports will fail."
fi

echo "========== Basket Vision Python runtime check =========="
echo "python: $PYTHON_BIN"
if command -v uv >/dev/null 2>&1; then
  echo "uv: $(uv --version)"
else
  echo "uv: not found on PATH"
fi
echo "No ROS topic, camera, node, or model-weight operation is performed."

"$PYTHON_BIN" - <<'PY'
import importlib
import sys

failed = []

print("Python:", sys.version.replace("\n", " "))
print("Executable:", sys.executable)
print("Prefix:", sys.prefix)
print("Base prefix:", sys.base_prefix)

for module_name in (
    "numpy",
    "cv2",
    "PIL",
    "scipy",
    "yaml",
    "mmcv",
    "fvcore",
    "iopath",
    "transforms3d",
    "ultralytics",
    "rospy",
    "tf",
    "tf2_ros",
    "detectron2",
):
    try:
        module = importlib.import_module(module_name)
        version = getattr(module, "__version__", "installed")
        print(f"[OK]   {module_name}: {version}")
    except Exception as exc:
        failed.append(module_name)
        print(f"[FAIL] {module_name}: {type(exc).__name__}: {exc}")

try:
    import cv_bridge
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import Image

    print(f"[OK]   cv_bridge + CvBridge: {cv_bridge.__file__}")
    print(f"[OK]   ROS messages: Pose={Pose.__name__}, Image={Image.__name__}")
except Exception as exc:
    failed.append("cv_bridge/ROS messages")
    print(f"[FAIL] cv_bridge/ROS messages: {type(exc).__name__}: {exc}")

try:
    import torch
    import torchvision
    from torchvision.ops import nms

    print(f"[OK]   torch: {torch.__version__}")
    print(f"[OK]   torchvision: {torchvision.__version__}")
    print(f"[INFO] torch CUDA runtime: {torch.version.cuda}")
    print(f"[INFO] CUDA available: {torch.cuda.is_available()}")
    print(f"[INFO] CuDNN: {torch.backends.cudnn.version()}")
    print(f"[INFO] CXX11 ABI: {torch._C._GLIBCXX_USE_CXX11_ABI}")

    if not torch.cuda.is_available():
        failed.append("torch.cuda")
        print("[FAIL] torch.cuda: CUDA is unavailable")
    else:
        boxes = torch.tensor([[0.0, 0.0, 10.0, 10.0]], device="cuda")
        scores = torch.tensor([0.9], device="cuda")
        print(f"[OK]   CUDA NMS: {nms(boxes, scores, 0.5)}")
except Exception as exc:
    failed.append("torch/torchvision CUDA NMS")
    print(f"[FAIL] torch/torchvision CUDA NMS: {type(exc).__name__}: {exc}")

if failed:
    print("\nRESULT: FAIL")
    print("Missing or unusable:", ", ".join(failed))
    raise SystemExit(1)

print("\nRESULT: PASS")
PY
