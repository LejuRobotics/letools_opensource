#!/usr/bin/env bash
# 一条命令起停整条托盘伺服链路：三个检测器 + 行为树。
#
# 参数从**同目录的 board.json** 读（唯一真值源）：检测器那部分当 roslaunch
# 命令行 arg 传下去，行为树那部分由行为树进程自己经黑板读（READ_BOARD）。
#
# **不含 roscore 与相机** —— roscore 由机器人 bringup 或操作员自己起，
# 脚本只检查它在不在。
#
# 用法：
#   bash start_all.sh               # 起三个检测器 → 等话题 → 前台跑行为树
#   bash start_all.sh --dry-run     # 只检查参数与 roscore，不起任何东西
#
# 退出时（正常结束或 Ctrl+C）三个检测器会被杀掉。
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../../.." && pwd)"
BOARD="${SCENARIO_DIR}/board.json"
TREE="${SCENARIO_DIR}/py_tree.json"

DRY_RUN=0
for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    -h|--help) sed -n '1,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "未知参数：$arg（只认 --dry-run）" >&2; exit 2 ;;
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
    "pallet_size_mm", "long_side_parallel", "normal", "deck_z_mm",
    "fit_normal", "normal_refit",
    "box_size_mm", "z_near_mm", "z_far_mm",
    "score_w_conf", "score_w_area", "score_w_depth",
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
  for key in pallet_size_mm ref_edges long_side_parallel normal deck_z_mm \
             fit_normal normal_refit \
             box_size_mm z_near_mm z_far_mm score_w_conf score_w_area \
             score_w_depth model_path conf; do
    printf '  %-20s %s\n' "${key}" "$(board_get_or_warn "${key}")"
  done
  exit 0
fi

# --------------------------------------------------------------------------- #
# [2/4] 清掉上一轮的残留 —— **杀掉重启，不沿用**
#
# 为什么不沿用：这三个检测器的参数**全部来自 board.json**，而脚本每次都是从头
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
  # 打印当前活着的、我们关心的那三个节点（一行一个，去重）。
  local all
  all="$(rosnode list 2>/dev/null)" || return 1        # master 不通 -> 非 0
  printf '%s\n' "${all}" \
    | grep -xE '/(pallet_detection|carton_box_detect|box_detection)' || true
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
  # 而脚本后面等话题会超时，报出来的错跟真正的原因差很远。
  LEFT="$(nodes_alive)"
  if [[ -n "${LEFT}" ]]; then
    echo "  ✗ 这些节点杀不掉也清不掉：" >&2
    printf '        %s\n' ${LEFT} >&2
    echo "    手工处理：rosnode info <名字> 看它在哪台机器上" >&2
    exit 2
  fi
  echo "      ✓ 残留已清空"
else
  echo "  ✓ 三个检测器都还没起"
fi

# --------------------------------------------------------------------------- #
# [3/4] 起三个检测器 + 等话题
# --------------------------------------------------------------------------- #
echo "[3/4] 启动三个检测器 ..."

# 先把**从板子读到的关键值**打出来 —— 参数是命令行 arg 传下去的，不看这一眼
# 就不知道实际生效的是什么。只打会影响行为的（尺寸/法向/z 带/权重），
# 不打 model_path 这类长路径（它在下面每个检测器的行里）。
show_board_params() {
  echo "      板子读到的值："
  printf '        %-18s %s\n' pallet_size_mm  "$(board_get_or_warn pallet_size_mm)"
  printf '        %-18s %s\n' long_side_parallel "$(board_get_or_warn long_side_parallel)"
  printf '        %-18s %s\n' deck_z_mm      "$(board_get_or_warn deck_z_mm)"
  printf '        %-18s %s\n' box_size_mm    "$(board_get_or_warn box_size_mm)"
}
show_board_params
echo
PIDS=()
cleanup() {
  local code=$?
  echo
  echo "收尾：停止三个检测器 ..."
  for pid in "${PIDS[@]:-}"; do
    [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  exit "${code}"
}
trap cleanup EXIT INT TERM

start_detector() {
  local name="$1"; shift
  roslaunch "$@" >"/tmp/start_all_${name}.log" 2>&1 &
  PIDS+=("$!")
  echo "      ${name}  pid=$!  日志 /tmp/start_all_${name}.log"
}

start_detector pallet_detection pallet_detection pallet_detection.launch \
  "target_mm:=$(board_get_or_warn pallet_size_mm)" \
  "long_side_parallel:=$(board_get_or_warn long_side_parallel)" \
  "normal:=$(board_get_or_warn normal)" \
  "deck_z_mm:=$(board_get_or_warn deck_z_mm)" \
  "fit_normal:=$(board_get_or_warn fit_normal)" \
  "normal_refit:=$(board_get_or_warn normal_refit)"

start_detector carton_box_yolo carton_box_yolo carton_box_yolo.launch \
  "model_path:=$(board_get_or_warn model_path)" \
  "conf:=$(board_get_or_warn conf)" \
  "z_near_mm:=$(board_get_or_warn z_near_mm)" \
  "z_far_mm:=$(board_get_or_warn z_far_mm)" \
  "score_w_conf:=$(board_get_or_warn score_w_conf)" \
  "score_w_area:=$(board_get_or_warn score_w_area)" \
  "score_w_depth:=$(board_get_or_warn score_w_depth)"

start_detector box_detection box_detection box_detection.launch \
  "target_mm:=$(board_get_or_warn box_size_mm)"

echo "      等三路话题出数（每路最多 60s）..."
wait_topic() {
  local topic="$1" label="$2" timeout=60 elapsed=0
  # ⚠️ **等的时候要打点**：三路各等 60s、失败前一个字都不打的话，现场看到的是
  # "卡住了"，分不清是"相机没起""话题名写错"还是"检测器在跑但检不出东西"。
  # 每 10 秒打一次已等时长 + 该检测器日志的最后一行 —— 那一行通常直接说明原因。
  local log="/tmp/start_all_${3:-}.log"
  while (( elapsed < timeout )); do
    if timeout 3 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
      echo "      ${label} ${topic}  OK（等了 ${elapsed}s）"
      return 0
    fi
    if (( elapsed % 10 == 0 )); then
      local tailmsg=""
      [[ -f "${log}" ]] && tailmsg="$(grep -vE '^\s*$' "${log}" | tail -1 | cut -c1-90)"
      printf '        … %s 已等 %ds%s\n' "${label}" "${elapsed}" \
             "${tailmsg:+  最后一行: ${tailmsg}}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  echo "  ✗ ${label} ${topic} 等了 ${timeout}s 一直没有数据" >&2
  [[ -f "${log}" ]] && { echo "    该检测器日志最后 5 行：" >&2; tail -5 "${log}" >&2; }
  return 1
}

if ! wait_topic /pallet/detection "托盘检测" pallet_detection \
   || ! wait_topic /box/yolo_box     "纸箱检测" carton_box_yolo \
   || ! wait_topic /box/detection    "箱子四角" box_detection; then
  echo >&2
  echo "三路没全出数。" >&2
  echo "  · 相机起了吗？      rostopic hz /camera/color/image_raw" >&2
  echo "  · 托盘一直不出数？   看日志里有没有 reject=…（检不出和起不来是两件事）" >&2
  echo "                       tail -20 /tmp/start_all_pallet_detection.log" >&2
  echo "  · 全部日志：        /tmp/start_all_*.log" >&2
  exit 1
fi

# --------------------------------------------------------------------------- #
# [4/4] 前台跑行为树
# --------------------------------------------------------------------------- #
echo "[4/4] 启动行为树 ..."
echo "      三个数：rostopic echo /pallet_servo/dis"
echo "      切箱位：rosservice call /pallet_servo/slot \"slot: 1\""
echo "      停止：  call slot: 0；结束整条链路按 Ctrl+C"
echo
bash "${SCENARIO_DIR}/start_behavior_tree.sh"
