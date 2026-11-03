#!/usr/bin/env bash
# 一条命令起整条托盘伺服链路：两个检测器（托盘 + 纸箱 OBB）+ 行为树。
#
# 参数从**同目录的 board.json** 读（唯一真值源）：检测器那部分当 roslaunch
# 命令行 arg 传下去，行为树那部分由行为树进程自己经黑板读（READ_BOARD）。
#
# **不含 roscore 与相机** —— roscore 由机器人 bringup 或操作员自己起，
# 脚本只检查它在不在。
#
# ============================================================================
# 设计口径（2026-10-18 按现场流程重写）
# ============================================================================
#
# * **启动不等任何检测数据。** 这条链路是在**码放之前**起的，那时不一定看得见
#   托盘和纸箱。旧版会 `wait_topic` 等到第一帧检测结果才往下走 —— 托盘被手里
#   那个箱子挡死时必然超时，脚本在行为树起来之前就退出了。
# * **检测器默认待命**（launch 传 `gated:=true`）：启动时各跑**一帧预热**，
#   然后停手；只有 `rosservice call /pallet_servo/slot N` 激活档位之后才出数，
#   `slot: 0` 之后回到待命。门控由行为树里的伺服节点广播（`/pallet_servo/detect_mode`）。
# * **预热在启动时做、并且等它做完**：两个检测器的首帧实测 **28~35s**（CUDA
#   内核/显存初始化），这笔钱不能等操作员 call 了服务才付。
#
# ============================================================================
# 现场流程
# ============================================================================
#   1. 机器人启动 → `bash start_all.sh`
#   2. 机器人去抓箱子           —— **不用 call 服务**，检测器待命
#   3. 到放置位置              —— `rosservice call /pallet_servo/slot "slot: N"`
#   4. 放完                    —— `rosservice call /pallet_servo/slot "slot: 0"`
#   5. 回到 2，直到 5 个箱子放完
#
# 用法：
#   bash start_all.sh               # 起两个检测器 → 前台跑行为树
#   bash start_all.sh --dry-run     # 只检查参数与 roscore，不起任何东西
#   bash start_all.sh --detectors-only  # 只起两个检测器，行为树由外部主流程提供
#   CARTON_VENV_DIR=/path/to/venv bash start_all.sh   # 指定 carton 虚拟环境
#      （默认 /media/data/venvs/carton；不存在时再试 $HOME/ztz/venvs/carton）
#
# 退出时（正常结束或 Ctrl+C）检测器会被杀掉。
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../../.." && pwd)"
BOARD="${SCENARIO_DIR}/board.json"
TREE="${SCENARIO_DIR}/py_tree.json"

DRY_RUN=0
DETECTORS_ONLY=0
DETECTORS_READY_FILE="${PALLET_DETECTORS_READY_FILE:-}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=1; shift ;;
    --detectors-only) DETECTORS_ONLY=1; shift ;;
    -h|--help) sed -n '1,50p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)
      echo "未知参数：$1（只认 --dry-run / --detectors-only）" >&2
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
  return 0
}

source_if_exists "/opt/ros/noetic/setup.bash"
source_if_exists "${REPO_ROOT}/infrastructure/ros_packages/devel/setup.bash"
export ROS_PACKAGE_PATH="${REPO_ROOT}/infrastructure/ros_packages/src:${ROS_PACKAGE_PATH:-}"

# --------------------------------------------------------------------------- #
# 整条链路从 **carton 虚拟环境** 启动（不是 base 环境）
#
# ⚠️ **本脚本只裸调 `python3`**，这一段唯一的职责是：让 `python3` 真的是 venv 的那个。
#    `(carton)` 的 activate 是**按固定路径打包**的（写死 `VIRTUAL_ENV=/media/data/venvs/carton`），
#    解压到别的目录后它把**一个不存在的 bin** 塞进 PATH —— `python3` 会静默回落到
#    /usr/bin/python3，而提示符照样显示 `(carton)`，从外面看不出任何异常。
#    更坑的是 activate 还 `export PYTHONNOUSERSITE=1`（有意屏蔽 ~/.local 里的旧包），
#    于是用 system python 时**连 `pip install --user` 装进来的包都看不见**：
#    `pip install colorlog` 报 Successfully，行为树照样 ModuleNotFoundError。
#    2026-10-25 现场就是栽在这里（两个检测器只打一条 WARN 就降级，行为树直接崩）。
#
# 口径与 orchestration/scenarios/carton_obb_trt_service_v1/start_with_pallet_servo.sh 一致：
# 默认 /media/data/venvs/carton，可用 CARTON_VENV_DIR 覆盖；activate 之后**再按解析出的
# 目录强制修正 VIRTUAL_ENV / PATH** —— 这样 `command -v python3`（下面作为 roslaunch 的
# launch-prefix 传下去）和子进程 start_behavior_tree.sh 用的就都是 venv 里的 python3。
#
# ⚠️ activate 里已设 PYTHONNOUSERSITE=1，用来屏蔽 ~/.local 里的旧包；别删。
#    它也是下面「colorlog 必须在 venv 里」那条硬检查成立的前提。
# --------------------------------------------------------------------------- #
resolve_venv() {
  # CARTON_VENV_DIR 一旦**显式设置**就只认它（设错要报，不能悄悄换成别的 venv）；
  # 没设时按「打包时的固定路径 -> 本机的解压落法」顺序找。
  local cand
  if [[ -n "${CARTON_VENV_DIR:-}" ]]; then
    if [[ -x "${CARTON_VENV_DIR}/bin/python3" ]]; then
      printf '%s' "${CARTON_VENV_DIR}"
      return 0
    fi
    return 1
  fi
  for cand in "/media/data/venvs/carton" "${HOME}/ztz/venvs/carton"; do
    if [[ -x "${cand}/bin/python3" ]]; then
      printf '%s' "${cand}"
      return 0
    fi
  done
  return 1
}

if ! VENV_DIR="$(resolve_venv)"; then
  if [[ -n "${CARTON_VENV_DIR:-}" ]]; then
    echo "  ✗ CARTON_VENV_DIR=${CARTON_VENV_DIR} 里没有可执行的 bin/python3" >&2
  else
    echo "  ✗ 找不到 carton 虚拟环境（试过：/media/data/venvs/carton、\$HOME/ztz/venvs/carton）" >&2
    echo "    请设置 CARTON_VENV_DIR，或把虚拟环境部署到默认目录。" >&2
  fi
  exit 2
fi

# activate 可能被裁掉/漏解压 —— 那就只修 PATH，不 source，别硬失败。
if [[ -f "${VENV_DIR}/bin/activate" ]]; then
  source_if_exists "${VENV_DIR}/bin/activate"
fi
export VIRTUAL_ENV="${VENV_DIR}"
# 用 activate 记下的原始 PATH 打底（没 source 就用在场的），再前置**解析出的**真实 bin。
export PATH="${VENV_DIR}/bin:${_OLD_VIRTUAL_PATH:-${PATH}}"
export YOLO_CONFIG_DIR="${VENV_DIR}/config/ultralytics"
export MPLCONFIGDIR="${VENV_DIR}/config/matplotlib"
hash -r

VENV_PYTHON="${VENV_DIR}/bin/python3"
# 硬检查：行为树 core/common/logger.py 是**模块顶层** `import colorlog`（没有回退），
# 缺了就在这里当场说清楚，别等到 [4/4] 抛 traceback、把「环境问题」看成「代码坏了」。
if ! "${VENV_PYTHON}" -c 'import colorlog' >/dev/null 2>&1; then
  echo "  ✗ ${VENV_DIR} 里的 python3 找不到 colorlog —— 行为树会在 import 阶段直接崩。" >&2
  echo "    修：${VENV_PYTHON} -m pip install colorlog -i https://pypi.tuna.tsinghua.edu.cn/simple" >&2
  exit 2
fi

# --------------------------------------------------------------------------- #
# 从 board.json 取一个键的值。
# 用 python3 而不是 jq —— jq 不是每台机器都有，python3 是必然有的。
# --------------------------------------------------------------------------- #
board_get() {
  python3 - "$BOARD" "$1" <<'PY'
import json, sys
path, want = sys.argv[1], sys.argv[2]
data = json.load(open(path, encoding="utf-8"))
for group in data.values():
    if not isinstance(group, list):
        continue
    for item in group:
        if isinstance(item, dict) and item.get("key") == want:
            value = item["value"]
            if isinstance(value, bool):
                print("true" if value else "false")
            elif isinstance(value, list):
                print(json.dumps(value))
            else:
                print(value)
            raise SystemExit(0)
raise SystemExit(3)
PY
}

# 板子上没有这个键 → 打一条 WARNING，返回空串（launch 用自己的默认值）。
# 与节点侧 READ_BOARD 缺键告警同一条纪律：**缺了要说，不能静默**。
board_get_or_warn() {
  local key="$1" value
  if ! value="$(board_get "${key}")"; then
    echo "  ⚠️  board.json 里没有 '$key' —— 用 launch 自己的默认值" >&2
    value=""
  fi
  printf '%s' "${value}"
}

# ★ 双向核对：板子上有、py_tree.json 没引用的键是**死配置** —— 这个值根本不
#   起作用。板上加了一个键却忘了在 tree 里引用，会静默无效。
check_dead_keys() {
  python3 - "$BOARD" "$TREE" <<'PY'
import json, sys
board = json.load(open(sys.argv[1], encoding="utf-8"))
tree = json.load(open(sys.argv[2], encoding="utf-8"))
used = set()

def walk(node):
    if isinstance(node, dict):
        for key, value in (node.get("params") or {}).items():
            if isinstance(value, dict) and str(value.get("source", "")).upper() == "READ_BOARD":
                used.add(value.get("board_key", key))
        for value in node.values():
            walk(value)
    elif isinstance(node, list):
        for item in node:
            walk(item)

walk(tree)
board_keys = [item["key"] for group in board.values() if isinstance(group, list)
              for item in group if isinstance(item, dict) and "key" in item]

# 本脚本自己会从板子上取的键 —— 它们走命令行传给检测器，本来就不在 tree 里引用。
# 豁免名单**在这里手工维护**，不按组名猜：组是"这个参数在说哪件事"（`pallet` 组里
# 既有行为树读的 `pallet_frame`、也有检测器读的 `long_side_parallel`），拿组名当
# 豁免依据会漏掉同组里走命令行的那几个、每次启动刷一批假 WARNING。
# ⚠️ **在这里加了新键、却忘了在下面 `board_get_or_warn` 里真的取它**，它就又变成
#    死配置且不再告警 —— 改这一处时要连着看下面 start_detector 那几段。
detector_keys = {
    "pallet_size_mm", "long_side_parallel", "normal",
    "fit_normal", "normal_refit",
    # D 闸门（2026-10-15 加）：参考平面距离与容差。它**贴 `pallet_` 前缀**，
    # 得排在 box 组前面才拿得到（板子按键名取值、第一个匹配的组优先）。
    "pallet_d_mm", "pallet_d_tol",
    # 托盘 OBB（2026-10-14 换成 YOLO-OBB 路线后新增；旧掩码路线的
    # deck_z_mm / refine_support_min 已拔掉）。**一律带 pallet_ 前缀** ——
    # 板子按键名取值、第一个匹配的组优先（pallet 组在 box 组前面），
    # 与 box 组重名会让托盘和纸箱共用一个值。
    "pallet_model_path", "pallet_imgsz", "pallet_conf",
    "pallet_nms_iou", "pallet_obb_pick",
    # 推理后端（2026-10-20）：engine 优先 / 强制 / 强制 ONNX。
    "pallet_infer_backend", "pallet_engine_path", "pallet_pose_engine_path",
    # 两种前端（2026-10-18）：detector 切 obb/pose；pose 那组全部带 pallet_ 前缀。
    # refine 拆成两个键 —— start_all 按 detector 自动选，命令行仍然只传一个
    # `refine:=` 给 launch。
    "pallet_detector",
    "pallet_pose_model_path", "pallet_pose_conf", "pallet_pose_kconf",
    "pallet_pose_edge_refine", "pallet_pose_pick",
    "pallet_pose_refine", "pallet_refine",
    # 纸箱那组（2026-10-13 换成 OBB 之后改名：判据不再是 conf 而是距离）
    "z_near_mm", "z_far_mm",
    "score_w_dcenter", "score_w_area", "score_w_depth",
    "d_near_px", "d_far_px",
    "model_path", "conf",
}
dead = [k for k in board_keys if k not in used and k not in detector_keys]
if dead:
    print(f"  ⚠️  board.json 里这些键没有被 py_tree.json 引用，**不起作用**：{dead}")
PY
}

# --------------------------------------------------------------------------- #
# [1/4] 检查 roscore
# --------------------------------------------------------------------------- #
echo "[1/4] 检查前置条件 ..."
echo "  ✓ 虚拟环境：${VENV_DIR}"
echo "    python3  ：${VENV_PYTHON}（colorlog 已就绪）"
if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  echo "  ✗ ROS_MASTER_URI 没设 —— 先起 roscore，或 source 机器人自己的 bringup" >&2
  exit 2
fi
if [[ -z "${ROS_IP:-}" && -z "${ROS_HOSTNAME:-}" ]]; then
  echo "  ✗ ROS_IP 或 ROS_HOSTNAME 必须设一个（多机时节点靠它互相找到）" >&2
  exit 2
fi
if [[ ! -f "${BOARD}" ]]; then
  echo "  ✗ 找不到 ${BOARD}" >&2
  exit 2
fi
check_dead_keys
echo "  ✓ ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "  ✓ board.json 可读"

if [[ "${DRY_RUN}" == "1" ]]; then
  echo
  echo "--dry-run：以下是从 board.json 读到的值（不启动任何东西）"
  for key in pallet_size_mm ref_edges long_side_parallel normal \
             fit_normal normal_refit pallet_d_mm pallet_d_tol \
             pallet_model_path pallet_imgsz pallet_conf pallet_nms_iou pallet_obb_pick \
             pallet_infer_backend pallet_engine_path \
             pallet_detector pallet_pose_model_path pallet_pose_engine_path pallet_pose_conf \
             pallet_pose_kconf pallet_pose_edge_refine pallet_pose_pick \
             pallet_pose_refine pallet_refine \
             z_near_mm z_far_mm score_w_dcenter score_w_area \
             score_w_depth d_near_px d_far_px model_path conf; do
    printf '  %-20s %s\n' "${key}" "$(board_get_or_warn "${key}")"
  done
  echo
  echo "两个检测器都会起（gated:=true）；脚本等预热完成（~35s）再起行为树。"
  exit 0
fi

# --------------------------------------------------------------------------- #
# 日志：三个进程（托盘检测 / 纸箱检测 / 行为树）**共用同一个带时间戳的文件**
#
# 目标是"排查一次故障只看一个文件"。做法是**脚本分配文件名**、通过环境变量传给
# 所有子进程（它们都是本脚本的子进程，环境自动继承；launch 里的 `<env>` 只覆盖
# 单个变量，不会清空环境），各进程以 append 打开 —— Linux 的 `O_APPEND` 保证
# 每次 write 落到文件末尾、不会互相覆盖。
#
# ⚠️ **必须同时关掉检测器的控制台日志**（`LETOOLS_LOG_CONSOLE=0`）：下面
# `start_detector` 把它们 stdout **也**重定向进这个文件，而 logger 的控制台
# handler 同样写 stdout —— 不关的话**每条日志会在同一个文件里出现两次**
# （一次详细格式、一次控制台格式），文件难读、50MB 也翻倍地用掉。
# 行为树进程**不关**：它跑在前台，终端就是操作员的实时视图，不能没有。
#
# `LETOOLS_LOG_ROLLOVER=new_file` = 写满 50MB **另开一个新文件**（不改名成
# `.1/.2`、不删旧的）；旧文件由 `purge_days=3` 按 mtime 清理。
# 三个变量都留了环境覆盖口，现场想调不用改脚本。
# --------------------------------------------------------------------------- #
LOG_DIR="${REPO_ROOT}/log/pallet_log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/pallet_$(date +%Y%m%d_%H%M%S).log"
export LETOOLS_LOG_DIR="${LOG_DIR}"
export LETOOLS_LOG_FILE="${LOG_FILE}"
export LETOOLS_LOG_MAX_BYTES="${LETOOLS_LOG_MAX_BYTES:-52428800}"   # 50MB
export LETOOLS_LOG_PURGE_DAYS="${LETOOLS_LOG_PURGE_DAYS:-3}"        # 3 天
export LETOOLS_LOG_ROLLOVER="${LETOOLS_LOG_ROLLOVER:-new_file}"
export LETOOLS_LOG_CONSOLE=0
echo "  日志：${LOG_FILE}"
echo "        （三个进程共用；写满 $(( ${LETOOLS_LOG_MAX_BYTES} / 1048576 ))MB "
echo "         另开一个新文件，${LETOOLS_LOG_PURGE_DAYS} 天前的自动清理）"

# --------------------------------------------------------------------------- #
# [2/4] 清掉上一轮的残留 —— **杀掉重启，不沿用**
#
# 为什么不沿用：这两个检测器的参数**全部来自 board.json**，而脚本每次都是从头
# 读板子。上一轮跑着的那个实例用的是**上一版的 board.json**（或者上一版的代码）——
# 沿用等于"改了板子却不生效"，而且看不出来。杀掉重启是唯一能保证"板子 -> 行为"
# 一致的做法。
#
# ⚠️ **判据是 `rosnode list` + `grep -qx`，不是 `rosnode ping`。**
# 2026-09-29 实测：`rosnode ping` 的**退出码没有意义** —— 它对不存在的节点只打
# 一行 `cannot ping [...]: unknown node` 就 `return False`，而
# `_rosnode_cmd_ping()` **把返回值丢掉了**（`/opt/ros/noetic/lib/.../rosnode/__init__.py`
# 第 314 行 `return False` vs 第 771 行直接调完不管），`rosnodemain` 最后
# `sys.exit(... or 0)` —— 所以"节点不存在"照样退出码 0。**只有 master 连不上
# 才非 0。** 拿它当 `if` 条件会**恒为真**（只要 master 活着），写成循环就永远
# 只报第一个名字。`rosnode info` 同样的坑。
#
# `rosnode list` 不一样：它把**实际存在的节点名逐行打出来**，所以
# `grep -qx "/名字"` 是个**有意义的判据**。master 连不上时它会非 0 退出
# （`ERROR: Unable to communicate with master!`），这里据此直接报错退出 ——
# 前置条件那一步已经检查过 `ROS_MASTER_URI` 设了，但不代表 master 活着。
# --------------------------------------------------------------------------- #
nodes_alive() {
  # 打印当前活着的、我们关心的那几个节点（一行一个，去重）。
  # ⚠️ **这里是「清残留」的判据，节点名换了必须跟着改** —— 漏一个的后果是
  # 上一轮的旧实例接着跑，而脚本以为清干净了（旧实例的参数是上一版 board.json 的）。
  local all
  all="$(rosnode list 2>/dev/null)" || return 1        # master 不通 -> 非 0
  printf '%s\n' "${all}" \
    | grep -xE '/(pallet_detection|carton_obb_detect)' || true
}

# 等两个检测器**注册到 master**（不是"出数"）。只有它们真的起来了，后面
# call 服务才可能出东西；起不来就在这里当场报，别等到现场以为"伺服坏了"。
wait_detector_nodes() {
  local timeout=25 elapsed=0 missing=""
  while (( elapsed < timeout )); do
    local all
    all="$(rosnode list 2>/dev/null)" || true
    missing=""
    for n in /pallet_detection /carton_obb_detect; do
      printf '%s\n' "${all}" | grep -qx "${n}" || missing="${missing} ${n}"
    done
    [[ -z "${missing}" ]] && return 0
    sleep 1
    elapsed=$((elapsed + 1))
  done
  echo "  ✗ 等了 ${timeout}s，这些检测器节点没起来：${missing}" >&2
  echo "    看日志：grep -E '\\[(pallet_detection|carton_obb)\\]' ${LOG_FILE} | tail -30" >&2
  return 1
}

# 等两个检测器**预热完成**（不是注册完成）。
# ⚠️ 节点注册是瞬间的，首帧推理要 **28~35s**（CUDA 初始化）。不等这一步的话，
#    脚本会打印"已就绪"，而操作员立刻 call slot 会有 20~30s 完全没结果 ——
#    现场会误判成"检测/阈值坏了"（2026-10-18 真机踩过）。
# 判据是共用日志里两行「预热完成」：检测器只往 LOG_FILE 写（控制台已关），
# 两次启动各自新建日志文件，所以不会数到上一轮的。
# 超时**不拦**（与相机缺失同一口径：相机可能比链路晚起，检测器会等第一帧），
# 但必须说清楚"还没热"。
wait_detector_warmup() {
  local timeout=40 elapsed=0
  local pallet_ready=0 carton_ready=0
  while (( elapsed < timeout )); do
    grep -q '\[pallet_detection\].*预热完成' "${LOG_FILE}" 2>/dev/null \
      && pallet_ready=1
    grep -q '\[carton_obb\].*预热完成' "${LOG_FILE}" 2>/dev/null \
      && carton_ready=1
    if (( pallet_ready == 1 && carton_ready == 1 )); then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  echo "  ⚠️ 等了 ${timeout}s 预热仍未完成：" >&2
  echo "     托盘检测器=$([[ ${pallet_ready} == 1 ]] && echo 已完成 || echo 未完成)，纸箱检测器=$([[ ${carton_ready} == 1 ]] && echo 已完成 || echo 未完成)" >&2
  echo "     检测器会继续等第一帧；先确认 /camera/color/image_raw。" >&2
  return 1
}

echo "[2/4] 清掉上一轮的残留节点 ..."
if ! rosnode list >/dev/null 2>&1; then
  echo "  ✗ 连不上 ROS master（ROS_MASTER_URI=${ROS_MASTER_URI}）—— 先起 roscore" >&2
  exit 2
fi

RESIDUAL="$(nodes_alive)"
if [[ -n "${RESIDUAL}" ]]; then
  echo "      已在跑、要重启的："
  printf '        %s\n' ${RESIDUAL}
  echo "      （它们的参数是上一版 board.json 的，沿用等于「改了板子却不生效」）"

  while read -r node; do
    [[ -n "${node}" ]] && rosnode kill "${node}" >/dev/null 2>&1 || true
  done <<< "${RESIDUAL}"

  # 等它们真的退出（最多 10s）—— `rosnode kill` 是异步的：发的是 SIGINT，
  # 节点要自己走完 rospy 的关闭流程才会从 master 上消失。
  for _ in $(seq 1 20); do
    [[ -z "$(nodes_alive)" ]] && break
    sleep 0.5
  done

  # 还有残留 = 进程已经死了、注册信息还挂在 master 上（`rosnode kill` 够不着
  # 它 —— 它连不上那个 XML-RPC URI）。**这一步必须做**：不清掉的话新实例起来
  # 会报 "name is already registered"。
  # ⚠️ `rosnode cleanup` **没有 `--yes`**（实测 `no such option`），它是要人敲
  # `y` 的交互命令 —— 这里喂 `yes` 把它自动化。
  if [[ -n "$(nodes_alive)" ]]; then
    echo "      还有杀不掉的（多半是进程已死、注册信息还挂着）—— 清注册表"
    yes y 2>/dev/null | rosnode cleanup >/dev/null 2>&1 || true
    sleep 1
  fi

  # 最后一道：**不干净就退出，别硬起** —— 硬起的结果是新节点撞名字起不来，
  # 报出来的错与真正的原因差很远。
  LEFT="$(nodes_alive)"
  if [[ -n "${LEFT}" ]]; then
    echo "  ✗ 这些节点杀不掉也清不掉：" >&2
    printf '        %s\n' ${LEFT} >&2
    echo "    手工处理：rosnode info <名字> 看它在哪台机器上" >&2
    exit 2
  fi
  echo "      ✓ 残留已清空"
else
  echo "  ✓ 两个检测器都还没起"
fi

# --------------------------------------------------------------------------- #
# [3/4] 起两个检测器（**待命**，启动预热一帧）
#
# ⚠️ **两个都起、都传 `gated:=true`**（2026-10-18）：
#   * 不门控地"按档位决定起不起"做不到 —— 档位是运行中才 call 的，而起进程在
#     那之前；
#   * 待命不等于白烧 GPU：空闲时检测器**不算**，只在启动时跑一帧预热
#     （托盘首帧 3.5s、纸箱首帧 27.8s，必须挪到启动时）。
#   * 门控由伺服节点广播：0=全停 / 1=只要纸箱 / 2=托盘+纸箱。
# --------------------------------------------------------------------------- #
echo "[3/4] 启动两个检测器（待命；启动预热一帧）..."

# 先把**从板子读到的关键值**打出来 —— 参数是命令行 arg 传下去的，不看这一眼
# 就不知道实际生效的是什么。逐条列（不用 for 循环）：这里的键集与上面 `--dry-run`
# 的键集、`detector_keys` 豁免名单**必须一致**，漏一个就是"传了但看不见"。
show_board_params() {
  echo "      纸箱检测器读到的值："
  printf '        %-18s %s\n' score_w_dcenter "$(board_get_or_warn score_w_dcenter)"
  printf '        %-18s %s\n' d_near_px      "$(board_get_or_warn d_near_px)"
  printf '        %-18s %s\n' d_far_px       "$(board_get_or_warn d_far_px)"
  echo "      托盘检测器读到的值："
  printf '        %-18s %s\n' pallet_size_mm  "$(board_get_or_warn pallet_size_mm)"
  printf '        %-18s %s\n' long_side_parallel "$(board_get_or_warn long_side_parallel)"
  printf '        %-18s %s\n' normal         "$(board_get_or_warn normal)"
  printf '        %-18s %s\n' fit_normal     "$(board_get_or_warn fit_normal)"
  printf '        %-18s %s\n' normal_refit   "$(board_get_or_warn normal_refit)"
  printf '        %-18s %s\n' pallet_d_mm    "$(board_get_or_warn pallet_d_mm)"
  printf '        %-18s %s\n' pallet_d_tol   "$(board_get_or_warn pallet_d_tol)"
  printf '        %-18s %s\n' pallet_model_path "$(board_get_or_warn pallet_model_path)"
  printf '        %-18s %s\n' pallet_imgsz   "$(board_get_or_warn pallet_imgsz)"
  printf '        %-18s %s\n' pallet_conf    "$(board_get_or_warn pallet_conf)"
  printf '        %-18s %s\n' pallet_nms_iou "$(board_get_or_warn pallet_nms_iou)"
  printf '        %-18s %s\n' pallet_obb_pick "$(board_get_or_warn pallet_obb_pick)"
  printf '        %-18s %s\n' pallet_detector "$(board_get_or_warn pallet_detector)"
  printf '        %-18s %s\n' pallet_pose_model_path "$(board_get_or_warn pallet_pose_model_path)"
  printf '        %-18s %s\n' pallet_pose_conf "$(board_get_or_warn pallet_pose_conf)"
  printf '        %-18s %s\n' pallet_pose_kconf "$(board_get_or_warn pallet_pose_kconf)"
  printf '        %-18s %s\n' pallet_pose_edge_refine "$(board_get_or_warn pallet_pose_edge_refine)"
  printf '        %-18s %s\n' pallet_pose_pick "$(board_get_or_warn pallet_pose_pick)"
  printf '        %-18s %s\n' pallet_pose_refine "$(board_get_or_warn pallet_pose_refine)"
  printf '        %-18s %s\n' pallet_refine "$(board_get_or_warn pallet_refine)"
}
show_board_params
echo
PIDS=()
cleanup() {
  local code=$?
  echo
  echo "收尾：停止已起的检测器 ..."
  [[ -z "${DETECTORS_READY_FILE}" ]] || rm -f "${DETECTORS_READY_FILE}"
  for pid in "${PIDS[@]:-}"; do
    [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  exit "${code}"
}
trap cleanup EXIT INT TERM

start_detector() {
  local name="$1"; shift
  # ⚠️ 追加进**共用的日志文件**，不再丢到 /tmp —— 那里既没轮转、也没清理，
  # 现场排查时根本想不到去翻它（改名前的写法就是 `>/tmp/start_all_*.log`）。
  # 无缓冲输出保证「预热完成」能立即写入日志，供就绪检测实时读取。
  PYTHONUNBUFFERED=1 stdbuf -oL -eL roslaunch "$@" >>"${LOG_FILE}" 2>&1 &
  PIDS+=("$!")
  echo "      ${name}  pid=$!  日志 ${LOG_FILE}"
}

# 两种前端（2026-10-18）：按 board 上的 pallet_detector 选 refine 的取值。
# pose 路线默认 pallet_pose_refine=false（edge_refine 已在 detect 内做完）；
# obb 路线保持 pallet_refine=true（现行行为）。launch 侧只有一个 refine arg。
detector="$(board_get_or_warn pallet_detector)"
detector="${detector:-obb}"           # 缺键时行为树/launch 的默认值也是 obb
detector="$(printf '%s' "${detector}" | tr 'A-Z' 'a-z')"   # 与节点一样大小写不敏感
DETECTOR_PYTHON="$(command -v python3)"
if [ "${detector}" = "pose" ]; then
  refine_key=pallet_pose_refine
else
  refine_key=pallet_refine
fi

start_detector pallet_detection pallet_detection pallet_detection.launch \
  "python:=${DETECTOR_PYTHON}" \
  "gated:=true" \
  "detector:=${detector}" \
  "refine:=$(board_get_or_warn ${refine_key})" \
  "target_mm:=$(board_get_or_warn pallet_size_mm)" \
  "long_side_parallel:=$(board_get_or_warn long_side_parallel)" \
  "normal:=$(board_get_or_warn normal)" \
  "fit_normal:=$(board_get_or_warn fit_normal)" \
  "normal_refit:=$(board_get_or_warn normal_refit)" \
  "d_mm:=$(board_get_or_warn pallet_d_mm)" \
  "d_tol:=$(board_get_or_warn pallet_d_tol)" \
  "model_path:=$(board_get_or_warn pallet_model_path)" \
  "engine_path:=$(board_get_or_warn pallet_engine_path)" \
  "infer_backend:=$(board_get_or_warn pallet_infer_backend)" \
  "imgsz:=$(board_get_or_warn pallet_imgsz)" \
  "conf:=$(board_get_or_warn pallet_conf)" \
  "nms_iou:=$(board_get_or_warn pallet_nms_iou)" \
  "obb_pick:=$(board_get_or_warn pallet_obb_pick)" \
  "pose_model_path:=$(board_get_or_warn pallet_pose_model_path)" \
  "pose_engine_path:=$(board_get_or_warn pallet_pose_engine_path)" \
  "pose_conf:=$(board_get_or_warn pallet_pose_conf)" \
  "pose_kconf:=$(board_get_or_warn pallet_pose_kconf)" \
  "pose_edge_refine:=$(board_get_or_warn pallet_pose_edge_refine)" \
  "pose_pick:=$(board_get_or_warn pallet_pose_pick)"

# 纸箱 OBB：一个节点直接出 /box/detection 的四角（旧的两个包已合并/删除）
start_detector carton_obb carton_obb carton_obb.launch \
  "gated:=true" \
  "model_path:=$(board_get_or_warn model_path)" \
  "conf:=$(board_get_or_warn conf)" \
  "z_near_mm:=$(board_get_or_warn z_near_mm)" \
  "z_far_mm:=$(board_get_or_warn z_far_mm)" \
  "score_w_dcenter:=$(board_get_or_warn score_w_dcenter)" \
  "score_w_area:=$(board_get_or_warn score_w_area)" \
  "score_w_depth:=$(board_get_or_warn score_w_depth)" \
  "d_near_px:=$(board_get_or_warn d_near_px)" \
  "d_far_px:=$(board_get_or_warn d_far_px)"

# ⚠️ **只等"节点注册"和"预热完成"，不等检测数据** —— 这正是本次重构的目的：
#    码放之前不一定看得见托盘/纸箱，等数据会把整条链卡死在启动阶段。
#    预热是确定的（各跑一帧），与"看不看得见"无关，所以可以等、也必须等到位。
echo "      等检测器节点注册（最多 25s；**不等检测数据**）..."
wait_detector_nodes || exit 1
echo "      两个检测器已并行启动，等待各自首帧预热完成..."
if wait_detector_warmup; then
  echo "      ✓ 两个检测器已就绪（预热完成，待命）"
  if [[ -n "${DETECTORS_READY_FILE}" ]]; then
    : >"${DETECTORS_READY_FILE}"
  fi
else
  echo "  ⚠️ 两个检测器尚未全部就绪，不创建就绪标记。" >&2
fi

# 相机话题：只**提示**不拦。先保存完整输出，避免 pipefail 将 grep 提前退出
# 导致的 SIGPIPE 误判为 rostopic 失败。
CAMERA_TOPICS="$(rostopic list 2>/dev/null || true)"
if ! grep -qx "/camera/color/image_raw" <<<"${CAMERA_TOPICS}"; then
  echo "  ⚠️  现在还没看到 /camera/color/image_raw —— 检测器会一直等它。"
  echo "     起相机后用 rostopic hz /camera/color/image_raw 确认；"
  echo "     日志里检测器的心跳会写「门控=关（预热中）」直到第一帧到达。"
fi

if [[ "${DETECTORS_ONLY}" == "1" ]]; then
  echo
  echo "[4/4] 仅检测器模式：不启动 pallet_servo_real_v1 行为树"
  echo "      两个检测器保持待命，由外部主流程提供 /pallet_servo/slot 服务并控制门控。"
  echo "      结束：Ctrl+C（检测器一起收）"
  while true; do
    for pid in "${PIDS[@]}"; do
      if ! kill -0 "${pid}" 2>/dev/null; then
        if wait "${pid}"; then
          detector_code=0
        else
          detector_code=$?
        fi
        echo "  ✗ 检测器进程 PID=${pid} 已退出（状态码 ${detector_code}）" >&2
        [[ "${detector_code}" != "0" ]] || detector_code=1
        exit "${detector_code}"
      fi
    done
    sleep 1
  done
fi

# --------------------------------------------------------------------------- #
# [4/4] 前台跑行为树
# --------------------------------------------------------------------------- #
echo "[4/4] 启动行为树 ..."
cat <<'TIPS'
      现场流程：
        1) 去抓箱子           —— 不用 call 服务（检测器待命）
        2) 到放置位置         —— rosservice call /pallet_servo/slot "slot: N"
        3) 放完               —— rosservice call /pallet_servo/slot "slot: 0"
        4) 回到 1；直到全部放完
      看数：  rostopic echo /pallet_servo/dis
      看门控：rostopic echo /pallet_servo/detect_mode   （0=全停 1=纸箱 2=托盘+纸箱）
      结束：  Ctrl+C（检测器一起收）
TIPS
echo
bash "${SCENARIO_DIR}/start_behavior_tree.sh"
