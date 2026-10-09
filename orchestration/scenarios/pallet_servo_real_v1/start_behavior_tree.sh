#!/usr/bin/env bash
# 真机跑这个场景的统一入口。
#
# 路径**全部由脚本位置推出来**（REPO_ROOT），不写死部署路径 —— 仓库到哪台机器上
# 都能用。仓库里另外几份 start_behavior_tree.sh 里出现的 /media/data/LeTools
# 是那台机器上的部署约定，这里靠 REPO_ROOT 就够了。
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../../.." && pwd)"

source_if_exists() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    local had_nounset=0
    case "$-" in
      *u*) had_nounset=1; set +u ;;
    esac
    # shellcheck disable=SC1090
    source "${setup_file}"
    if [[ "${had_nounset}" == "1" ]]; then
      set -u
    fi
  fi
}

source_if_exists "/opt/ros/noetic/setup.bash"
# 这个工作空间**必须编译过**：core/common/ros_environment.py 在它不存在时直接抛
# FileNotFoundError，而 perception_adapter.py 是模块级 `from apriltag_ros.msg ...`
# —— 抓帧明明用不到 apriltag，import 不进来整个硬件层照样起不来。
source_if_exists "${REPO_ROOT}/infrastructure/ros_packages/devel/setup.bash"
export ROS_PACKAGE_PATH="${REPO_ROOT}/infrastructure/ros_packages/src:${ROS_PACKAGE_PATH:-}"
export PYTHONPATH="${REPO_ROOT}/infrastructure/ros_packages/devel/lib/python3/dist-packages:${PYTHONPATH:-}"

# ⚠️ **这里必须设，检测器那边不是这样。** 三个检测器各有自己的 launch，
# `<env>` 已经替它们设了这一对（`pallet_detection.launch` / `carton_box_yolo.launch`
# / `box_detection.launch` 的 `<env>` 是**覆盖**继承来的环境，所以你不 export
# 它们也是 1 线程）；**而本脚本是直接 `python3` 起的，前面没有 roslaunch**，
# 唯一替这个进程设线程数的地方就是这里。
#
# 这个进程里的 OpenBLAS 活不少：`node_pallet_servo` 的叠加图渲染（`cv2` + `numpy`，
# 见 `skills/atomic/perception/pallet_servo/render.py`）与 `pallet_frame` 的窗平均
# 都在它里面跑。不设 = 按核数开线程，而这些矩阵都太小、多线程是纯开销，
# 还会和 ROS 回调线程抢核 —— 实测差 1.6x。
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1

if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  echo "ROS_MASTER_URI 没设 —— 先起 roscore，或者 source 机器人自己的 bringup。" >&2
  echo "（只跑离线回放的话不需要它，用 --dry-run。）" >&2
  exit 2
fi
if [[ -z "${ROS_IP:-}" && -z "${ROS_HOSTNAME:-}" ]]; then
  echo "ROS_IP 或 ROS_HOSTNAME 必须设一个（多机时节点之间要靠它互相找到）。" >&2
  exit 2
fi

echo "REPO_ROOT=${REPO_ROOT}" >&2
echo "SCENARIO_DIR=${SCENARIO_DIR}" >&2

# ⚠️ **不要用 `exec`**：exec 会用 python 替换掉这个 bash 进程，于是调用方
# （`start_all.sh`）收不到"行为树跑完了"这个点，也就没法清理它起的检测器。
# 不用 exec 时 bash 在前台等 python，Ctrl+C 照样传到整个进程组。
python3 "${REPO_ROOT}/apps/test_upper_init/run_behavior_tree_json.py" \
  --scenario "${SCENARIO_DIR}" "$@"
