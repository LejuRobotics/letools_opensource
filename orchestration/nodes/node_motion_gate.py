# -*- coding: utf-8 -*-
"""NodeMotionGate：视觉与底盘伺服之间的人工确认闸门（非阻塞）。

为什么需要这个节点
------------------
码垛视觉链路（`pallet_servo_real_v1`）和底盘伺服链路（`chassis_servo_*`）合并成
一棵树之后，"视觉出数"和"底盘开始动"之间不再有人工检查点：档位一激活、树一
tick，底盘就会按三个误差量开始走。实际调试时需要在中间停一下，让人看一眼
`/pallet_servo/dis` 上的三个数和叠加图，确认合理再放行。

为什么不复用 `WaitForEnter`
---------------------------
`skills/atomic/refactored_sdk/wait_for_enter.py` 在 `on_execute()` 里**同步**调
`input()`，会把行为树主循环（50 Hz）整个卡住。本树里视觉节点是靠 `update()`
持续发数、发叠加图、发心跳的（`NodePalletServo` 的 `servo_error_heartbeat_s`），
主循环一停，人看到的就是一屏**冻结的旧数据** —— 正好在确认一个早就过期的画面。

所以本节点把读 stdin 放到**后台线程**：`update()` 永远立刻返回，视觉照常出数，
人按回车只影响闸门自己。

三段，都以"看到有效观测"为终点（都可配）
----------------------------------------
**每个 tick 都会读黑板**，不再"先死等固定时长再看"（2026-10-21 真机踩过：
固定等待到点时后台激活刚好才发出 `rosservice call`，闸门早一步判了超时）。

1. 激活窗口 `settle_after_slot_s`：从闸门第一次 tick 起算的**第一段预算**，
   要盖住 `start_all.sh` 后台子 shell 的整段激活流程（等服务出现 ≤10s
   + 再等 `--settle-before-slot` 秒 + call 往返）。`start_all.sh` 会把它算成
   「`--settle-before-slot` + 12」通过 `VISION_GATE_SETTLE_S` 传进来。
2. 出数窗口 `valid_timeout_s`：第 1 段用尽后，再给这么多秒等**有效观测**。
3. 任何时刻出现有效观测 → 立刻打印三个误差量（第 3 段），然后无限期等回车
   （不设超时，见 Q12）。

两段都等不到才 `FAILURE`，报错里写明两个窗口的预算和三条排查命令 —— 档位没激活、
检测器没出数、配对窗一直 reject，都表现为这里等不到东西，而整棵树**必须停**，
不能带着空数据往下走。

超时口径与 `NodeServoErrorFromTopic` 的分工
------------------------------------------
上游节点**只把有效帧写进黑板**（invalid 帧直接跳过，见该节点 update 的闸门 1），
所以"黑板上有观测"本身就等价于"valid=true"。本节点仍然判一次 `valid`，
是为了兼容直接写 `ServoError`（没有 valid 字段，按 True 处理）的旧场景。

stdin 不是终端时（Q15C）
------------------------
`nohup`、`systemd`、`< /dev/null`、被别的脚本用管道拉起 —— 这些情况下一旦
进入闸门就再也不会有回车。**当场 FAILURE**，而不是挂在那里等。现场必须用终端
跑 `start_all.sh`。

黑板契约
--------
读 latest_servo_error   AlignObservation（`NodeServoErrorFromTopic` 写入）
"""

import os
import sys
import threading
import time

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from core.domain.servo_types import is_observation_like, observation_field
from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest

logger = get_logger(__name__)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

OBS_KEY_DEFAULT = "latest_servo_error"

# 脚本侧的 `--settle-before-slot` / `--settle-after-slot` 通过这两个环境变量
# 传进来；`start_all.sh` 是唯一会设它们的地方（`py_tree.json` 里是默认值）。
ENV_ACTIVATE = "VISION_GATE_SETTLE_S"
ENV_OBSERVE = "VISION_GATE_VALID_TIMEOUT_S"

DEFAULT_MESSAGE = "确认三个误差量合理后按 Enter 启动底盘伺服（Ctrl+C 中止）"

# 提示信息的刷新间隔：倒计时每 tick 都改 feedback 没意义，1 秒一次就够。
_FEEDBACK_REFRESH_S = 1.0


def _read_blackboard(client, key, default=None):
    """读黑板键；py_trees Client.__getattr__ 抛 KeyError，须显式 try。"""
    try:
        return getattr(client, key)
    except KeyError:
        return default


def _parse_bool(val, default: bool) -> bool:
    """布尔参数解析；ParamsWrapper 可能把 bool 存成字符串 'false'。

    ``bool("false")`` 是 ``True``，与工厂 `_convert_type` 同口径（同
    `node_enable_vel_control`）。
    """
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        s = val.strip().lower()
        if s in ("true", "1", "yes"):
            return True
        if s in ("false", "0", "no", ""):
            return False
    return bool(val) if val is not None else default


def _env_float(name: str, default: float) -> float:
    """环境变量优先的 float 参数；没设或解析失败都退回默认值。

    解析失败要留痕：拼错的环境变量会让现场以为"我明明调了等待时间"。
    """
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        logger.warning("[NodeMotionGate] 环境变量 %s=%r 不是数字，退回默认 %s",
                       name, raw, default)
        return default


@define_manifest(
    label="运动闸门（人工确认）",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="等视觉出有效观测 → 打印三个误差量 → 等回车 → 放行底盘伺服（非阻塞）",
    params=[
        {"name": "obs_key", "type": "string", "default": OBS_KEY_DEFAULT,
         "description": "读观测量的黑板键名（须与 NodeServoErrorFromTopic 一致）"},
        {"name": "settle_after_slot_s", "type": "float", "default": "22.0",
         "description": "激活窗口：从闸门第一 tick 起，等有效观测的第一段预算（要盖住后台"
                        " call slot 的整段流程 + 视觉出数）。环境变量 VISION_GATE_SETTLE_S 优先"},
        {"name": "valid_timeout_s", "type": "float", "default": "8.0",
         "description": "出数窗口：激活窗口用尽后，再给这么多秒等有效观测；仍没有则整树 "
                        "FAILURE。环境变量 VISION_GATE_VALID_TIMEOUT_S 优先"},
        {"name": "confirm", "type": "bool", "default": "True",
         "description": "是否真的等回车。False=观测有效就放行（离线用例 / 无人值守）"},
        {"name": "confirm_message", "type": "string", "default": DEFAULT_MESSAGE,
         "description": "按回车前的提示语"},
    ],
    inputs=[{"name": OBS_KEY_DEFAULT, "type": "object",
             "description": "AlignObservation：两路像素偏差 + 偏差角 + valid"}],
    outputs=[],
)
class NodeMotionGate(BaseAction):
    """人工确认闸门：等有效观测（激活 + 出数两窗口）→ 等回车（全程非阻塞）。"""

    # "这个键从未被写过"的哨兵（None 是合法的「写过但为空」）
    _MISSING = object()

    def __init__(self, name, label, namespace, params):
        super(NodeMotionGate, self).__init__(name, label, namespace, params)
        self._obs_key = str(self.params.get("obs_key", OBS_KEY_DEFAULT)).strip()
        self._activate_budget_s = _env_float(
            ENV_ACTIVATE, float(self.params.get("settle_after_slot_s", 22.0)))
        self._observe_budget_s = _env_float(
            ENV_OBSERVE, float(self.params.get("valid_timeout_s", 8.0)))
        self._confirm = _parse_bool(self.params.get("confirm", True), True)
        self._message = str(self.params.get("confirm_message", DEFAULT_MESSAGE))

        self._started_at = None
        self._prompted = False
        self._confirmed = False
        self._input_error = ""
        self._input_lock = threading.Lock()
        self._input_thread = None
        self._tty_ok = True
        self._last_feedback_at = 0.0

        self.global_blackboard.register_key(
            key=self._obs_key, access=py_trees.common.Access.READ)

    # ------------------------------------------------------------------ 生命周期
    def initialise(self):
        self._started_at = time.monotonic()
        self._prompted = False
        self._confirmed = False
        self._input_thread = None
        self._last_feedback_at = 0.0
        with self._input_lock:
            self._input_error = ""
        # Q15C：非终端直接判失败。放在这里而不是 update 里，是为了让"第一次 tick
        # 就 FAILURE"，而不是先干等 settle 秒再报。
        self._tty_ok = (not self._confirm) or _DRY_RUN or sys.stdin.isatty()
        if self._confirm and not _DRY_RUN and not self._tty_ok:
            logger.error(
                "[NodeMotionGate] stdin 不是终端（isatty=False）—— 闸门无法确认，"
                "整树将失败。请用终端直接跑 start_all.sh，不要 nohup / "
                "systemd / 重定向 stdin。")

    def update(self):
        if _DRY_RUN:
            self.feedback_message = "dry-run motion_gate"
            return Status.SUCCESS

        if self._confirm and not self._tty_ok:
            self.feedback_message = (
                "stdin 不是终端，无法按回车确认 —— 请用终端跑 start_all.sh")
            return Status.FAILURE

        elapsed = time.monotonic() - self._started_at

        # 每个 tick 都看黑板：有效观测一出现就立刻进入第 3 段，不再"先死等
        # 固定时长再看"。死等的问题是真机暴露的 —— 后台子 shell 的 call 恰好
        # 落在固定等待到期的那一刻，闸门早一步判了超时。
        obs = _read_blackboard(self.global_blackboard, self._obs_key, self._MISSING)

        if not self._has_valid_obs(obs):
            total = self._activate_budget_s + self._observe_budget_s
            if elapsed > total:
                self.feedback_message = (
                    f"从起树算起 {total:g}s 一直没有有效观测"
                    f"（激活窗口 {self._activate_budget_s:g}s + 出数窗口 "
                    f"{self._observe_budget_s:g}s 都用尽）。"
                    f"排查：① grep '\\[激活\\]' 共用日志，确认 rosservice call 返回 "
                    f"ok: True；② rostopic echo /pallet_servo/dis 看 valid 与 reject；"
                    f"③ rostopic echo /pallet_servo/detect_mode 确认门控已切到非 0")
                logger.error("[NodeMotionGate] %s", self.feedback_message)
                return Status.FAILURE
            if elapsed > self._activate_budget_s:
                self._set_feedback(
                    f"激活窗口已过，等视觉出数 "
                    f"{elapsed - self._activate_budget_s:.1f}/{self._observe_budget_s:g}s"
                    f"（{self._obs_key} 上仍没有有效观测）")
            else:
                self._set_feedback(
                    f"等激活 + 视觉出数 {elapsed:.1f}/{self._activate_budget_s:g}s"
                    f"（{self._obs_key} 上还没有）")
            return Status.RUNNING

        # 第 3 段：打印实际数值 + 等回车
        if not self._prompted:
            self._announce(obs)
            self._prompted = True
            if self._confirm:
                self._start_input_thread()

        if not self._confirm:
            return Status.SUCCESS

        with self._input_lock:
            err = self._input_error
        if err:
            self.feedback_message = err
            return Status.FAILURE

        if self._confirmed:
            self.feedback_message = "已确认，放行底盘伺服"
            logger.info("[NodeMotionGate] 操作员已确认，放行底盘伺服")
            return Status.SUCCESS

        self._set_feedback(f"等操作员按 Enter —— {self._message}")
        return Status.RUNNING

    def terminate(self, new_status):
        # 闸门没有硬件副作用，可改动的东西只有后台线程，而它是 daemon：
        # 进程退出时自己结束，不需要在这里 join（join 会把树收场卡住）。
        if self._input_thread is not None and self._input_thread.is_alive():
            logger.info("[NodeMotionGate] 闸门终止（后台等待回车的线程留在后台）")

    # ------------------------------------------------------------------ 内部
    def _has_valid_obs(self, obs) -> bool:
        """黑板上的观测是否可用。

        `NodeServoErrorFromTopic` 只写有效帧，所以"有值"通常就够了；这里再判一次
        `valid`，是为了兼容直接写 `ServoError`（没有该字段，按 True）的旧场景。
        """
        if obs is self._MISSING or obs is None:
            return False
        if not is_observation_like(obs):
            return False
        return bool(observation_field(obs, "valid", True))

    def _announce(self, obs):
        """把三个误差量打到**终端**（不只是日志文件）。

        `start_all.sh` 导出了 `LETOOLS_LOG_CONSOLE=0`，logger 只写文件；操作员
        眼前这个终端里能看到的东西只有 print。所以面向人的提示一律走 print。
        """
        e_bottom = float(observation_field(obs, "e_bottom_px", 0.0))
        e_right = float(observation_field(obs, "e_right_px", 0.0))
        theta = float(observation_field(obs, "theta_rad", 0.0))
        line = (f"e_bottom={e_bottom:.1f}px  e_right={e_right:.1f}px  "
                f"theta={theta:.4f}rad")
        print("\n" + "=" * 66, flush=True)
        print("  视觉已出有效观测：" + line, flush=True)
        print("  叠加图：rosrun rqt_image_view rqt_image_view /pallet_servo/overlay",
              flush=True)
        print("=" * 66, flush=True)
        logger.info("[NodeMotionGate] 视觉已出有效观测：%s", line)
        self.feedback_message = f"视觉有效（{line}），等回车"

    def _start_input_thread(self):
        """起后台线程读 stdin。**绝不能**在主线程/`update()` 里调 `input()`。"""
        if self._input_thread is not None:
            return
        self._input_thread = threading.Thread(
            target=self._read_input, name="motion_gate_input", daemon=True)
        self._input_thread.start()

    def _read_input(self):
        try:
            # 提示语走 stdout（终端可见），见 `_announce` 的说明
            input(f"\n{self._message} ... ")
            self._confirmed = True
        except EOFError:
            with self._input_lock:
                self._input_error = "读 stdin 得到 EOF —— 终端已关闭，无法确认"
        except Exception as exc:                     # noqa: BLE001
            with self._input_lock:
                self._input_error = f"读 stdin 异常：{exc}"

    def _set_feedback(self, text: str):
        """限流写 feedback：每 tick 都改值会刷爆日志，1 秒一次足够。"""
        now = time.monotonic()
        if now - self._last_feedback_at < _FEEDBACK_REFRESH_S:
            return
        self._last_feedback_at = now
        self.feedback_message = text
