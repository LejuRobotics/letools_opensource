#!/usr/bin/env bash
# Strict, non-motion runtime verification for the validated JetPack 5 profile.
# It imports models and ROS interfaces but does not contact a ROS master,
# subscribe to topics, access cameras, or start ROS nodes. Set
# BASKET_VERIFY_MODEL_LOAD=1 to load the GDRN and YOLO weights as well.

set -uo pipefail

if [ "$#" -gt 1 ]; then
  echo "Usage: $0 [python-executable]" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]-$0}")" && pwd)"
DEFAULT_BASKET_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BASKET_ROOT="${BASKET_VISION_ROOT:-${DEFAULT_BASKET_ROOT}}"
GDRN_ROOT="${BASKET_ROOT}/basket_gdrnpp"
BASKET_WS_SETUP="${BASKET_ROOT}/basket_vision_ws/devel/setup.bash"
BASKET_WS_DEVEL="${BASKET_ROOT}/basket_vision_ws/devel"
if [ "$#" -eq 1 ]; then
  PYTHON_BIN="$1"
elif [ -n "${VIRTUAL_ENV:-}" ]; then
  PYTHON_BIN="${VIRTUAL_ENV}/bin/python"
else
  PYTHON_BIN="$(command -v python3 2>/dev/null || true)"
fi

failures=()

fail() {
  failures+=("$1")
  printf '[FAIL] %s\n' "$1"
}

pass() {
  printf '[OK]   %s\n' "$1"
}

if [ ! -x "${PYTHON_BIN}" ]; then
  echo "[FAIL] Python executable is not usable: ${PYTHON_BIN}" >&2
  exit 2
fi

if [ "$(uname -m)" = "aarch64" ]; then
  pass "architecture: aarch64"
else
  fail "architecture must be aarch64 (actual: $(uname -m))"
fi

OS_VERSION="$(. /etc/os-release 2>/dev/null; printf '%s' "${VERSION_ID:-unknown}")"
if [ "${OS_VERSION}" = "20.04" ]; then
  pass "Ubuntu version: 20.04"
else
  fail "Ubuntu version must be 20.04 (actual: ${OS_VERSION})"
fi

JETPACK_VERSION="$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null || true)"
if [ "${JETPACK_VERSION}" = "5.1.4-b17" ]; then
  pass "JetPack version: 5.1.4-b17"
else
  fail "JetPack version must be 5.1.4-b17 (actual: ${JETPACK_VERSION:-not installed})"
fi

L4T_VERSION="$(dpkg-query -W -f='${Version}' nvidia-l4t-core 2>/dev/null || true)"
if [[ "${L4T_VERSION}" == 35.6.0-* ]]; then
  pass "L4T version: ${L4T_VERSION}"
else
  fail "L4T version must be 35.6.0 (actual: ${L4T_VERSION:-not installed})"
fi

CV_BRIDGE_SO="/opt/ros/noetic/lib/python3/dist-packages/cv_bridge/boost/cv_bridge_boost.so"
if [ -f "${CV_BRIDGE_SO}" ] \
  && ldd "${CV_BRIDGE_SO}" 2>/dev/null | grep -q 'libpython3\.8' \
  && ldd "${CV_BRIDGE_SO}" 2>/dev/null | grep -q 'libboost_python38'; then
  pass "cv_bridge ABI: Python 3.8"
else
  fail "cv_bridge is missing or is not linked for Python 3.8"
fi

if [ -f /opt/ros/noetic/setup.bash ]; then
  set +u
  unset _CATKIN_SETUP_DIR
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  ros_setup_status=$?
  set -u
  if [ "${ros_setup_status}" -eq 0 ]; then
    pass "ROS Noetic setup"
  else
    fail "ROS Noetic setup failed"
  fi
else
  fail "/opt/ros/noetic/setup.bash is missing"
fi

if [ -f "${BASKET_WS_SETUP}" ]; then
  set +u
  unset _CATKIN_SETUP_DIR
  # shellcheck disable=SC1090
  source "${BASKET_WS_SETUP}"
  basket_ws_status=$?
  set -u
  if [ "${basket_ws_status}" -eq 0 ]; then
    pass "basket_vision_ws setup"
  else
    fail "basket_vision_ws setup is stale or invalid; rebuild it in place"
  fi
else
  fail "basket_vision_ws is not built; run catkin_make in basket_vision_ws"
fi

case ":${CMAKE_PREFIX_PATH:-}:" in
  *":${BASKET_WS_DEVEL}:"*)
    pass "basket_vision_ws is the active catkin overlay"
    ;;
  *)
    fail "basket_vision_ws setup points to another build location; rebuild it in place"
    ;;
esac

export PYTHONDONTWRITEBYTECODE=1
export PYTHONNOUSERSITE=1
export BASKET_VISION_ROOT="${BASKET_ROOT}"
export BASKET_GDRNPP_DIR="${GDRN_ROOT}"
export PYTHONPATH="${GDRN_ROOT}:${GDRN_ROOT}/detectron2:${PYTHONPATH:-}"

if "${PYTHON_BIN}" - <<'PY'
import importlib.metadata as metadata
import os
import site
import sys
import traceback

failures = []


def check(name, condition, actual=None):
    if condition:
        print(f"[OK]   {name}" + (f": {actual}" if actual is not None else ""))
    else:
        message = name + (f" (actual: {actual})" if actual is not None else "")
        failures.append(message)
        print(f"[FAIL] {message}")


def is_below(path, directory):
    path = os.path.realpath(path)
    directory = os.path.realpath(directory)
    try:
        return os.path.commonpath((path, directory)) == directory
    except ValueError:
        return False


def package_version(distribution, expected):
    try:
        package = metadata.distribution(distribution)
        actual = package.version
    except metadata.PackageNotFoundError:
        check(f"{distribution}=={expected}", False, "not installed")
        return
    check(f"{distribution}=={expected}", actual == expected, actual)
    check(
        f"{distribution} installed in Basket Vision venv",
        is_below(str(package.locate_file("")), sys.prefix),
        str(package.locate_file("")),
    )


check("Python 3.8", sys.version_info[:2] == (3, 8), sys.version.split()[0])
check("isolated venv", sys.prefix != sys.base_prefix, sys.prefix)
check("user-site packages disabled", not site.ENABLE_USER_SITE, site.ENABLE_USER_SITE)
user_site = site.getusersitepackages()
check(
    "user-site path absent from sys.path",
    not any(is_below(path, user_site) for path in sys.path if path),
    user_site,
)

expected_packages = {
    "numpy": "1.24.4",
    "opencv-python": "4.11.0.86",
    "Pillow": "9.5.0",
    "scipy": "1.10.1",
    "PyYAML": "6.0.2",
    "torch": "2.0.0+nv23.5",
    "torchvision": "0.15.1",
    "detectron2": "0.6",
    "pytorch-lightning": "1.9.5",
    "mmcv": "1.7.1",
    "ultralytics": "8.3.163",
    "fvcore": "0.1.5.post20221221",
    "iopath": "0.1.10",
    "timm": "0.6.13",
    "transforms3d": "0.4.2",
}
for distribution, expected in expected_packages.items():
    package_version(distribution, expected)

try:
    import torch
    import torchvision
    from torchvision.ops import nms

    check("torch CUDA available", torch.cuda.is_available(), torch.cuda.is_available())
    check("torch CUDA runtime 11.4", torch.version.cuda == "11.4", torch.version.cuda)
    check("cuDNN 8.6", torch.backends.cudnn.version() == 8600, torch.backends.cudnn.version())
    check("CXX11 ABI", bool(torch._C._GLIBCXX_USE_CXX11_ABI), torch._C._GLIBCXX_USE_CXX11_ABI)
    if torch.cuda.is_available():
        check("GPU capability 8.7", torch.cuda.get_device_capability(0) == (8, 7), torch.cuda.get_device_capability(0))
        boxes = torch.tensor([[0.0, 0.0, 10.0, 10.0]], device="cuda")
        scores = torch.tensor([0.9], device="cuda")
        result = nms(boxes, scores, 0.5)
        check("TorchVision CUDA NMS", result.tolist() == [0], result)
except Exception as exc:
    failures.append(f"PyTorch/TorchVision CUDA validation: {exc}")
    print(f"[FAIL] PyTorch/TorchVision CUDA validation: {type(exc).__name__}: {exc}")

try:
    from detectron2.data import MetadataCatalog
    from detectron2.layers import paste_masks_in_image
    from detectron2.structures import BoxMode

    check("Detectron2 runtime imports", all((MetadataCatalog, paste_masks_in_image, BoxMode)))
except Exception as exc:
    failures.append(f"Detectron2 imports: {exc}")
    print(f"[FAIL] Detectron2 imports: {type(exc).__name__}: {exc}")

try:
    import numpy as np
    from cv_bridge import CvBridge

    bridge = CvBridge()
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
    restored = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
    check("cv_bridge BGR8 round trip", restored.shape == image.shape and restored.dtype == image.dtype)
except Exception as exc:
    failures.append(f"cv_bridge round trip: {exc}")
    print(f"[FAIL] cv_bridge round trip: {type(exc).__name__}: {exc}")

try:
    import basket_vision_msgs
    import rospy
    import tf
    import tf2_ros
    from apriltag_ros.msg import AprilTagDetectionArray
    from basket_vision_msgs.srv import InferBasketPose
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import Image

    check("ROS Python interfaces", all((rospy, tf, tf2_ros, AprilTagDetectionArray, InferBasketPose, Pose, Image)))
    expected_messages = os.path.join(
        os.environ["BASKET_VISION_ROOT"],
        "basket_vision_ws",
        "devel",
        "lib",
        "python3",
        "dist-packages",
    )
    check(
        "basket_vision_msgs generated by current workspace",
        is_below(basket_vision_msgs.__file__, expected_messages),
        basket_vision_msgs.__file__,
    )
except Exception as exc:
    failures.append(f"ROS Python interfaces: {exc}")
    print(f"[FAIL] ROS Python interfaces: {type(exc).__name__}: {exc}")

try:
    demo_dir = os.path.join(os.environ["BASKET_GDRNPP_DIR"], "core", "gdrn_modeling", "demo")
    sys.path.insert(0, demo_dir)
    import inference_service_vis_2_mult_inst_10_shared as service_module

    check("Basket Vision main-node import", hasattr(service_module, "SharedBasketPoseServiceNode"))
except Exception as exc:
    failures.append(f"Basket Vision main-node import: {exc}")
    print(f"[FAIL] Basket Vision main-node import: {type(exc).__name__}: {exc}")
    traceback.print_exc()

if failures:
    print("\nPYTHON RESULT: FAIL")
    for failure in failures:
        print(" -", failure)
    raise SystemExit(1)

print("\nPYTHON RESULT: PASS")
PY
then
  pass "Python/CUDA/ROS runtime"
else
  fail "Python/CUDA/ROS runtime validation failed"
fi

required_assets=(
  "${GDRN_ROOT}/output_basket_5/convnext_a6_AugCosyAAEGray_BG05_mlL1_DMask_amodalClipBox_classAware_basket.py"
  "${GDRN_ROOT}/output_basket_5/model_final_5.pth.1"
  "${GDRN_ROOT}/yolo_basket_5_weights/best_5.pt"
  "${GDRN_ROOT}/datasets/BOP_DATASETS/basket/models/models_info.json"
  "${GDRN_ROOT}/datasets/BOP_DATASETS/basket/models/obj_000001.ply"
  "${GDRN_ROOT}/datasets/BOP_DATASETS/basket/models/obj_000002.ply"
  "${GDRN_ROOT}/datasets/BOP_DATASETS/basket/models/obj_000003.ply"
  "${GDRN_ROOT}/datasets/BOP_DATASETS/basket/models/obj_000004.ply"
  "${GDRN_ROOT}/datasets/BOP_DATASETS/basket/models/obj_000005.ply"
)
for asset in "${required_assets[@]}"; do
  if [ -s "${asset}" ]; then
    pass "asset: ${asset#${BASKET_ROOT}/}"
  else
    fail "missing or empty asset: ${asset}"
  fi
done

if [ "${BASKET_VERIFY_MODEL_LOAD:-0}" = "1" ]; then
  if "${PYTHON_BIN}" - <<'PY'
import os
import shutil
import sys
import tempfile

import torch
from ultralytics import YOLO

gdrn_root = os.environ["BASKET_GDRNPP_DIR"]
demo_dir = os.path.join(gdrn_root, "core", "gdrn_modeling", "demo")
sys.path.insert(0, demo_dir)
from predictor_gdrn import GdrnPredictor

config_source = os.path.join(
    gdrn_root,
    "output_basket_5",
    "convnext_a6_AugCosyAAEGray_BG05_mlL1_DMask_amodalClipBox_classAware_basket.py",
)
checkpoint = os.path.join(gdrn_root, "output_basket_5", "model_final_5.pth.1")
yolo_weights = os.path.join(gdrn_root, "yolo_basket_5_weights", "best_5.pt")
object_models = os.path.join(gdrn_root, "datasets", "BOP_DATASETS", "basket", "models")

with tempfile.TemporaryDirectory(prefix="basket_vision_model_check_") as temp_dir:
    config_copy = os.path.join(temp_dir, "basket_model_check.py")
    shutil.copyfile(config_source, config_copy)
    with open(config_copy, "a", encoding="utf-8") as config_file:
        config_file.write(f"\nOUTPUT_DIR = {os.path.join(temp_dir, 'output')!r}\n")

    predictor = GdrnPredictor(
        config_file_path=config_copy,
        ckpt_file_path=checkpoint,
        camera_json_path=None,
        path_to_obj_models=object_models,
    )
    detector = YOLO(yolo_weights)
    print(f"GDRN parameters: {sum(parameter.numel() for parameter in predictor.model.parameters())}")
    print(f"YOLO task: {detector.task}")
    del detector
    del predictor

if torch.cuda.is_available():
    torch.cuda.empty_cache()
PY
  then
    pass "GDRN and YOLO weight loading"
  else
    fail "GDRN or YOLO weight loading failed"
  fi
fi

if [ "${BASKET_VERIFY_HASHES:-0}" = "1" ]; then
  HASH_FILE="${GDRN_ROOT}/requirements/basket_model_assets.sha256"
  if [ -f "${HASH_FILE}" ] && (cd "${BASKET_ROOT}" && sha256sum -c "${HASH_FILE#${BASKET_ROOT}/}"); then
    pass "model asset SHA256"
  else
    fail "model asset SHA256 validation failed"
  fi
fi

if [ "${#failures[@]}" -ne 0 ]; then
  printf '\nRESULT: FAIL\n'
  printf ' - %s\n' "${failures[@]}"
  exit 1
fi

printf '\nRESULT: PASS\n'
