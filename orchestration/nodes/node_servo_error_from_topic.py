# -*- coding: utf-8 -*-
"""NodeServoErrorFromTopic：订阅 `/pallet_servo/dis` → 写黑板 `latest_servo_error`。

`NodePalletServo` 的**对外接口是话题，不是黑板** —— `PalletServoError.msg` 的头注释
把这条写死了：黑板是 py_trees 的进程内单例，**树外的进程根本看不见它**。所以"第二条
树要用同一份视觉结果"的唯一正路是订那条话题，而不是在新树里再跑一遍视觉链路。

这正是本节点存在的理由：`chassis_servo_real_v1`（底盘闭环）要和
`pallet_servo_real_v1`（托盘伺服）**在同一台机器上同时跑**，而两个树各起一套
`pallet_detection` / `box_detection` / `carton_box_yolo` 会互踢 —— 同名节点后起的把
先起的顶掉（现场就是 `new node registered with same name` 那一串 shutdown），何况
相机只有一台、检测器只该有一份。分工因此定成：**pallet 树独占视觉链路、把三个数发
到话题上；chassis 树只订话题**（`NodePalletObs` / `NodeBoxObs` / `NodePalletServo`
在 chassis 场景里一个都不起）。

## 黑板契约

    写  latest_servo_error / latest_servo_error_version      AlignObservation

写的是 `core/domain/servo_types.py` 的 `AlignObservation`（与 `NodeInjectAlignObs`
同一个类），于是下游 `NodeChassisServoCalc` / `NodeChassisServoJudge` **一行都不用
改**：它们用 `is_observation_like` 判鸭子类型（只要 e_bottom_px / e_right_px /
theta_rad 这三个字段），`valid` / `reject` 用 `observation_field` 宽容读。这**不是
巧合** —— 那个三被控量契约本来就是照着 dev 视觉链路的 `ServoError` 定的，两种观测源
因此不用互相转换就能接。

## `valid == false` 的帧：**一个字都不写**（本节点最关键的一条）

话题上的无效帧**三个量是 NaN**（`PalletServoError.msg` 的原话：「`valid == false` 时
三个都是 NaN，不是 0.0」，理由是不判 `valid` 的消费者会把 `0.0` 当成"已经对准"），
而且发布方保证**任何时刻都有一条不超过 1 秒的消息**（`servo_error_heartbeat_s`）——
所以停摆、或还没出第一帧时，话题上是**持续**的 `valid=false`。

两种处理都"能跑起来"，但只有一种是安全的：

* **原样搬进黑板** → 下游 calc / judge 判到 `valid=false` **直接返回 FAILURE**
  （见那两处的 `if not observation_field(obs, "valid", True)`）。而契约里
  `等输入（托盘 缺，箱子 有）` 这种**是正常状态、不是错误**（现场就是"箱子还没推进
  视野"）。一条正常状态把整棵树判死，这个场景就根本用不了。
* **不写**（本节点的做法）→ 版本号不动，下游表现为「**等观测**」：calc 不重算、
  exec 保持不动、judge 不判决，整条树安静地等下一帧有效观测。与 `NodePalletServo`
  对退化帧的处理是**同一口径**（那一支的注释原话：「退化帧：不写黑板、版本不自增」）。
  也正因此本节点**故意不留 `require_valid` 开关**（`NodeBoxObs` 有那个开关）：这里的
  无效帧本来就没有可用的数，没有一种配置是"要它"的。

代价是"上游为什么不出数"在黑板上看不见，所以本节点把 `reject` **原文**（带实际值与
阈值，如 `pair_dt: dt=0.081s > max_dt_s=0.05s`）在**内容变化时**喊一次，并配一条
"多久没有新的**有效**观测了"的告警（`stale_after_sec`）。**静默地等**和**喊出来再
等**，对现场是完全不同的两件事。

## 为什么按字段名取、不 import 消息类

订阅仍然需要一个消息类，所以 `msg_type` 参数给**全名**，运行期用
`roslib.message.get_message_class()` 解析 —— 换话题/换类型只改参数。
但解析那层是**纯函数**（`parse_servo_error_message`），只按字段名取、不 import 任何
消息类型，于是测试里喂普通假对象就能跑、能进 CI（`pytest -m unit`）；本仓库的
`pallet_servo_msgs` 是子目录里的一个 catkin 包，这条写法也让它**不成为 LeTools 的
构建依赖**。

节点持续返回 RUNNING（源节点口径，与 `NodePercep` / `NodeBoxObs` /
`NodeInjectAlignObs` 一致）：返回 SUCCESS 会让 py_trees 每 tick 重进
`initialise()`，于是变成"反复建/析构订阅"。
"""
from __future__ import annotations

import math
import re
import threading
import time
from typing import Optional

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from core.domain.servo_types import AlignObservation
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.utils.blackboard import (
    bump_version,
    is_dry_run,
    num_or_none,
    read_version,
)
from skills.atomic.perception.pallet_servo.algorithm import parse_bool_param

logger = get_logger(__name__)

# 默认话题与消息类型。**换话题只改 `topic`**；换了消息形状（字段名不一样）才动
# `msg_type`，并且要在 `parse_servo_error_message` 里补一条分支。
TOPIC_DEFAULT = "/pallet_servo/dis"
MSG_TYPE_DEFAULT = "pallet_servo_msgs/PalletServoError"

# 下面两条**不是业务阈值**（业务阈值都在下游：servo_stop_threshold_px 等），只是
# "明显不是这个单位"的探测器，挡的是那种**算得出来、也不报错**的输入：
#   像素量级：真图 640x480，两个偏差不可能到十万像素；归一化的 0~1 或"米"不像像素。
#   角度量级：`theta_rad` 正常 ±45°（msg 注释），把 45.0（角度）当弧度发是最典型的
#             一种静默错 —— 数值看着像模像样，乘上 gain 之后底盘一次性转过头。
_MAX_PLAUSIBLE_PX = 1.0e5
_MAX_PLAUSIBLE_RAD = 2.0 * math.pi

# 告警去重用的"读数"模式（见 `_stable_text_key`）。
_DIGITS_RE = re.compile(r"[0-9]+(?:\.[0-9]+)?")

_MISSING = object()


def _finite(value) -> Optional[float]:
    """宽容取一个**有限**数；取不到 / NaN / ±inf 都给 `None`。

    `num_or_none` 已经挡了 NaN，但 **±inf 能过它** —— 而 `inf` 进下游会算出 `inf`
    指令（再被 `max_step_m` 夹住），看着"还能跑"。

    这里的 NaN 更不是"碰巧"：它是 `PalletServoError` 契约里**无效帧的正式取值**，
    所以这一层不是防御性编程，而是接口判据的一部分。
    """
    f = num_or_none(value)
    if f is None or not math.isfinite(f):
        return None
    return f


def _numbers_problem(e_bottom, e_right, theta) -> str:
    """三个量能不能用；能用返回空串，不能用返回**一句人话**（进日志与 feedback）。

    分两级：先"取不取得到有限数"，再"像不像这个单位"（见 `_MAX_PLAUSIBLE_*`）。
    顺序不能反 —— `45.0` 是有限数，它是不是"角度当弧度"要看它跟 2π 比。
    """
    for name, value in (("e_bottom_px", e_bottom), ("e_right_px", e_right),
                        ("theta_rad", theta)):
        if value is None:
            return (f"`{name}` 取不到 / 不是有限数（NaN、inf 或字段缺失）—— "
                    f"上游要么在发无效帧，要么话题上的消息形状变了")
    if abs(e_bottom) > _MAX_PLAUSIBLE_PX or abs(e_right) > _MAX_PLAUSIBLE_PX:
        return (f"像素偏差 {e_bottom!r} / {e_right!r} 超过 {_MAX_PLAUSIBLE_PX:g}px "
                f"—— 明显不像像素（归一化？米？），这一帧不往黑板上写")
    if abs(theta) > _MAX_PLAUSIBLE_RAD:
        return (f"`theta_rad={theta!r}` 超过 {_MAX_PLAUSIBLE_RAD!r}（2π）—— 是不是把"
                f"**角度**当**弧度**发了？（±45° 该写 {math.radians(45.0):.4f}）")
    return ""


def _stamp_sec(msg) -> float:
    """`header.stamp` → 秒；取不到 / 非有限 → **哨兵 `0.0`**，**绝不退 `time.time()`**。

    与 `NodeBoxObs` / `NodePalletObs` 同一口径。本节点不配对（配对在上游
    `NodePalletServo` 里按图像时间戳做完了），这个数只进"同一条观测不重写"的键与
    日志，代价有限 —— 但口径不该因为"代价小"就改一次，下次就有人把它当时间用。
    """
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    to_sec = getattr(stamp, "to_sec", None)
    if callable(to_sec):
        return _finite(to_sec()) or 0.0
    return _finite(stamp) or 0.0


def _stable_text_key(text: str) -> str:
    """告警去重键：把文案里的**读数**换成 `#`，其余原样（打出来的仍是原文）。

    `reject` 里嵌着 `已经 8.2s` 这种**每帧都在长**的量（`PalletServoError.msg` 的
    注释里就点了这一条），拿原文做键恒不相等 —— 去重形同虚设，实测就是"每秒一条、
    内容只差秒数"。同一个坑、同一个做法见 `node_pallet_servo._reject_dedup_key`，
    只是那边只砍秒数、这里要处理任意文案。
    """
    return _DIGITS_RE.sub("#", text)


# --------------------------------------------------------------------------- #
# 解析：**纯函数，不碰 rospy** —— 测试因此能进 CI
# --------------------------------------------------------------------------- #
def parse_servo_error_message(msg) -> Optional[dict]:
    """ROS 消息 → `{"e_bottom_px", "e_right_px", "theta_rad", "valid", ...}`。

    认不出来返回 `None`（调用方只记 WARNING，不抛、不 FAILURE）。**纯函数**：只按
    字段名取，不 import 任何消息类型，所以测试里喂普通假对象就能跑。

    字段名就是 `PalletServoError.msg` 的字段名（**本仓库自己的消息**），所以这里没有
    `NodeBoxObs.parse_box_message` 那一串备选布局 —— 名字对不上就在**这里**加一条
    分支，别去下游打补丁。

    返回的字典：

        e_bottom_px / e_right_px / theta_rad   float 或 None（None = 这一帧取不到）
        valid                                  bool；**没有 `valid` 字段时按 True**，
                                               与 `NodeBoxObs` 同口径："没有这个说法"
                                               不等于 "valid=false"，后者会把本来能用
                                               的帧全拒掉
        has_valid_field                        bool，调用方据此知道上面的 True 是不是默认
        reject                                 str，原文（无效帧才非空）
        stamp                                  float，见 `_stamp_sec`
        numbers_ok                             bool，三个量能不能用
        why                                    str，不能用时的一句话（能用时空串）

    ⚠️ **无效帧不在这里变成 `None` 返回**：无效帧的三个量就是 NaN，那是**正常输入**，
    要原样交给调用方去判 `valid` 并告警。在这里 `return None` 会让"上游在发无效帧"
    被误报成"消息形状不对" —— 同一件事分两层说，日志才对得上现场。

    `header.frame_id` **不读也不判**：三个量是**像素**偏差，像素坐标做不了 TF，那个
    frame_id 是标注性质（`PalletServoError.msg` 里写明了，与 `PalletDetection.msg`
    的用法一致）。
    """
    if msg is None:
        return None

    raw = {name: getattr(msg, name, _MISSING)
           for name in ("e_bottom_px", "e_right_px", "theta_rad")}
    if all(value is _MISSING for value in raw.values()):
        # 三个字段一个都对不上 —— 这不是这条话题的消息（话题名/类型配错了）。
        return None

    e_bottom = _finite(raw["e_bottom_px"])
    e_right = _finite(raw["e_right_px"])
    theta = _finite(raw["theta_rad"])
    problem = _numbers_problem(e_bottom, e_right, theta)

    valid = getattr(msg, "valid", None)
    return {
        "e_bottom_px": e_bottom,
        "e_right_px": e_right,
        "theta_rad": theta,
        "valid": True if valid is None else bool(valid),
        "has_valid_field": valid is not None,
        "reject": str(getattr(msg, "reject", "") or ""),
        "stamp": _stamp_sec(msg),
        "numbers_ok": not problem,
        "why": problem,
    }


# --------------------------------------------------------------------------- #
# 节点
# --------------------------------------------------------------------------- #
class NodeServoErrorFromTopic(BaseAction):
    """订阅伺服误差话题 → 写黑板 `latest_servo_error` + `_version`。

    params:
        topic           订阅的话题，默认 ``/pallet_servo/dis``。**换话题只改这一个**
        msg_type        消息类型全名，默认 ``pallet_servo_msgs/PalletServoError``；
                        运行期用 ``roslib.message.get_message_class()`` 解析
        obs_key         写黑板的键，默认 ``latest_servo_error``（要与下游 calc /
                        judge 的 ``obs_key`` 对得上）
        stale_after_sec 默认 0（不查）。>0 时，超过这么久没有**新的有效观测**就告警
                        一次，**仍然 RUNNING** —— "等观测"是正常状态，不是错误
        queue_size      订阅队列，默认 1（只要最新帧，别堆积）
        enabled         默认 true；false 时一个字都不写并返回 SUCCESS（不占着树）。
                        ⚠️ 这个口径与 `NodeBoxObs` / `NodeInjectAlignObs` 一致，但它
                        **摆在 `Parallel(success_on_one)` 下会让整棵树立刻收口** ——
                        `success_on_one` 只要有**一个**子节点 SUCCESS 就收口，而
                        "关掉"返回的正是 SUCCESS。`chassis_servo_real_v1` 删掉模拟源
                        的根因就是这个，见那个场景的 README。
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._topic = str(self.params.get("topic", TOPIC_DEFAULT)).strip()
        self._msg_type = str(self.params.get("msg_type", MSG_TYPE_DEFAULT)).strip()
        self._obs_key = str(self.params.get("obs_key", "latest_servo_error")).strip()
        # 布尔参数**不许用 `bool()` 顶替**：`bool("false") is True`，而场景 JSON 的
        # READ_BOARD 分支真会传字符串（`parse_bool_param` 的 docstring 把这个坑写全了）。
        try:
            self._enabled = parse_bool_param(self.params.get("enabled", True), "enabled")
            self._enabled_error = ""
        except ValueError as exc:
            self._enabled = False
            self._enabled_error = str(exc)
            logger.error("NodeServoErrorFromTopic 参数错误：%s —— 按 enabled=false 处理", exc)
        self._stale_after = float(self.params.get("stale_after_sec", 0.0) or 0.0)
        self._queue_size = max(1, int(self.params.get("queue_size", 1)))

        # 配置错误（话题名空）也走 FAILURE：别静默订阅一个空话题。
        self._config_err = "" if self._topic else "topic 不能为空"
        if not self._msg_type:
            self._config_err = self._config_err or "msg_type 不能为空"

        self._lock = threading.Lock()
        self._subscriber = None
        self._pending: Optional[dict] = None        # 回调只往这儿放，绝不碰黑板
        self._received_monotonic = 0.0              # 最后一帧消息（不论有效与否）的到达时刻
        self._last_valid_monotonic = 0.0            # 最后一帧**有效**观测的到达时刻
        self._last_written_key = None               # 同一条观测不重写（见 update()）
        self._ticks = 0
        self._written = 0
        self._setup_error = ""
        # ★ 告警**两个槽位分开**：跳过某一帧与"多久没出数"是两件事，共用一格会让
        #   它们每 tick 互相顶掉、双双刷屏（同 `NodePalletObs._warn_stamp_once`
        #   为什么要自己一格的理由）。
        self._last_skip_key: Optional[str] = None
        self._last_stale_key: Optional[str] = None
        self._last_type_warn: Optional[str] = None

        # ---- 黑板：注册 + **同时**置初值 -------------------------------------
        # `register_key` 在 py_trees 2.x 里**只注册权限、不创建值**，漏了 set 就是
        # 下游的 `KeyError`（`NodePercep` 踩过并写在注释里的坑）。
        # **两个键必须一起写**：只写值不写 `_version`，下游 calc / judge 的版本门禁
        # 会停在 (0, 0)、只算一次然后**永远不再重算**。同 `node_box_obs.py` 的写法。
        for key in (self._obs_key, f"{self._obs_key}_version"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE)
        self.global_blackboard.set(self._obs_key, None)
        setattr(self.global_blackboard, f"{self._obs_key}_version", 0)

    # ------------------------------------------------------------------ 生命周期
    def initialise(self):
        self._setup_error = ""
        with self._lock:
            self._pending = None
            self._received_monotonic = 0.0
        self._last_valid_monotonic = 0.0
        self._last_skip_key = None
        self._last_stale_key = None

        if self._config_err or self._enabled_error:
            self.feedback_message = self._config_err or self._enabled_error
            return
        if is_dry_run() or not self._enabled:
            self.feedback_message = "dry-run / enabled=false：不订阅话题"
            return

        # ★ 幂等闸。py_trees 的 `Behaviour.tick()` 对"状态不是 RUNNING"的行为
        #   **每 tick 重进 `initialise()`**（见 `node_pallet_servo.py` 里那段完整
        #   来龙去脉）；本节点在正常路径上永远 RUNNING，所以正常情况下进不来第二次
        #   —— 但父节点重新初始化这一支时会。没有这一句就是"反复建/析构订阅"。
        if self._subscriber is not None:
            return

        try:
            import rospy
            from roslib.message import get_message_class

            if not rospy.core.is_initialized():
                raise RuntimeError("ROS 节点尚未初始化（没有 master？）")
            msg_class = get_message_class(self._msg_type)
            if msg_class is None:
                raise RuntimeError(
                    f"找不到消息类型 {self._msg_type!r} —— 它所在的包要能被 import "
                    f"（catkin 包先 catkin_make 再 source devel/setup.bash，并把 "
                    f"devel/lib/python*/dist-packages 放进 PYTHONPATH）")
            self._subscriber = rospy.Subscriber(
                self._topic, msg_class, self._on_message,
                queue_size=self._queue_size, tcp_nodelay=True)
            logger.info("NodeServoErrorFromTopic 订阅 %s（%s）→ 写 %s(+_version)；"
                        "stale_after_sec=%g",
                        self._topic, self._msg_type, self._obs_key, self._stale_after)
        except Exception as exc:                     # noqa: BLE001
            # **绝不把异常抛出 `initialise()`**：py_trees 的 `tick()` 不接这个钩子
            # 抛出的异常，抛出去就是整棵树连每帧日志一起没。同 inject 节点的纪律。
            self._setup_error = str(exc)
            self._cleanup_subscriber()
            self.feedback_message = f"订阅 {self._topic} 失败：{exc}"

    def update(self):
        if self._config_err or self._enabled_error:
            return Status.FAILURE
        if is_dry_run() or not self._enabled:
            # 不占着树：与 `NodeInjectServoInput` / `NodeBoxObs` 同口径
            # （`enabled: false` 时返回 SUCCESS，否则"真机上关掉它"会把整条分支
            # 永远卡住）。
            return Status.SUCCESS
        if self._setup_error:
            self.feedback_message = f"NodeServoErrorFromTopic 初始化失败：{self._setup_error}"
            return Status.FAILURE

        self._ticks += 1
        with self._lock:
            parsed = self._pending
            received = self._received_monotonic

        if parsed is None:
            # **"还没收到消息"是正常状态**，不是异常：RUNNING + feedback，不崩。
            self.feedback_message = f"等 {self._topic} 的消息"
            if self._ticks % 50 == 1:
                logger.debug("NodeServoErrorFromTopic 等首条消息：%s", self._topic)
            return Status.RUNNING

        # ---- 闸门 1：无效帧（契约：三个量是 NaN）**一个字都不写** --------------
        if parsed["has_valid_field"] and not parsed["valid"]:
            self._skip(f"这一帧 valid=false，**不写黑板**（下游继续用上一帧有效观测）",
                       f"reject={parsed['reject'] or '（上游没给原因）'}")
            self._warn_stale()
            return Status.RUNNING

        # ---- 闸门 2：三个量本身不可用（NaN / inf / 不像这个单位）--------------
        if not parsed["numbers_ok"]:
            self._skip("这一帧的三个量不可用，**不写黑板**", parsed["why"])
            self._warn_stale()
            return Status.RUNNING

        # 走到这里这一帧是**有效的**：记下它的**到达时刻**（不是本 tick 的时刻）——
        # 口径见 `_warn_stale`。恢复有效也要把告警槽位清掉：下一次失效重新喊一遍。
        self._last_valid_monotonic = received
        self._last_skip_key = None
        self._last_stale_key = None

        # **同一条观测只写一次。** 回调来一条就存一条，而 `update()` 每 tick 都会被
        # 调用 —— 不加这一句就是"10 Hz 相机、10 Hz tick，每 tick 把同一帧重写一遍"：
        # 版本号一直涨，下游版本门禁每次放行，calc 拿同一帧白算一步。
        # 键里带 `stamp`：**新的一帧**（新的图像）即使三个数一模一样，也算一次
        # "量过了、还是没对上"，该让下游重算 —— 所以它必须能区分"同一帧重播"与
        # "新帧"（上游 `stamp` 缺省时是 0.0，那时退化成按三个数去重，见 `_stamp_sec`）。
        key = (parsed["stamp"], parsed["e_bottom_px"], parsed["e_right_px"],
               parsed["theta_rad"])
        if key == self._last_written_key:
            self.feedback_message = "伺服误差没变（同一帧），不重写"
            return Status.RUNNING
        self._last_written_key = key

        obs = AlignObservation(
            e_bottom_px=parsed["e_bottom_px"], e_right_px=parsed["e_right_px"],
            theta_rad=parsed["theta_rad"], valid=True, reject="")
        setattr(self.global_blackboard, self._obs_key, obs)
        bump_version(self.global_blackboard, self._obs_key)
        self._written += 1
        self.feedback_message = (
            f"e_bottom={parsed['e_bottom_px']:.1f}px "
            f"e_right={parsed['e_right_px']:.1f}px "
            f"theta={parsed['theta_rad']:.4f}rad（stamp={parsed['stamp']:.3f}）")
        if self._ticks % 30 == 1:
            logger.debug("NodeServoErrorFromTopic 第 %d 次写入 %s v%d：%s",
                         self._written, self._obs_key,
                         read_version(self.global_blackboard, f"{self._obs_key}_version"),
                         self.feedback_message)
        # ★ **永远 RUNNING**（源节点口径）。返回 SUCCESS/FAILURE 会让 py_trees
        #   每 tick 重进 `initialise()`。
        return Status.RUNNING

    def terminate(self, new_status):
        self._cleanup_subscriber()

    # ------------------------------------------------------------------ 回调（ROS 线程）
    def _on_message(self, msg) -> None:
        """只做「消息 → 解析结果」存内存。**黑板一个字都不碰**（只能在 tick 线程写）。"""
        parsed = parse_servo_error_message(msg)
        if parsed is None:
            # 认不出来的消息：**只喊一次**（按实际类型去重），点出真实类型方便排查；
            # 绝不抛、绝不 FAILURE —— 话题指错/上游换了写法不该把整棵树弄死。
            actual = ""
            header = getattr(msg, "_connection_header", None)
            if isinstance(header, dict):
                actual = str(header.get("type", ""))
            if actual != self._last_type_warn:
                self._last_type_warn = actual
                logger.warning(
                    "NodeServoErrorFromTopic 认不出 %s 上的消息（实际类型 %r，配置的 "
                    "msg_type 是 %r）—— 这一帧不写黑板。把消息的字段名贴回来，在 "
                    "parse_servo_error_message() 里加一条分支即可（它是纯函数，"
                    "进 CI 很容易）", self._topic, actual or "未知", self._msg_type)
            return
        with self._lock:
            self._pending = parsed
            self._received_monotonic = time.monotonic()

    # ------------------------------------------------------------------ 工具
    def _skip(self, *parts: str) -> None:
        """这一帧不写黑板时的统一出口：feedback 永远更新，**日志按稳定键去重**。

        去重口径见 `_stable_text_key`。**feedback 不做去重** —— 它是 py_trees 状态
        视图看的"现在为什么不动"，每次都该是当前那一帧的真话。
        """
        message = "，".join(part for part in parts if part)
        key = _stable_text_key(message)
        if key != self._last_skip_key:
            self._last_skip_key = key
            logger.warning("NodeServoErrorFromTopic %s（%s）", message, self._topic)
        self.feedback_message = message

    def _warn_stale(self) -> None:
        """"多久没有**新的有效观测**了" —— 自己的槽位，与 `_skip` 互不干扰。

        ⚠️ **量的是"最后一帧有效观测的到达时刻"，不是"最后一次收到消息"**：停摆时
        上游仍按 `servo_error_heartbeat_s`（1 s）发无效帧，用"收到消息"当口径**永远
        不响** —— 而现场最需要被告知的恰恰是"话题上是活的、但全是无效帧"。反过来，
        上游彻底没了（节点死了）时那个时刻也不再前进，同样会响。

        那个时刻**只在有效帧上更新**：所以"一帧相同的观测停在黑板上不动"也会响 ——
        对伺服来说那是正常信息：**没有新的视觉结果**，不是"看过了没问题"。
        """
        if self._stale_after <= 0.0 or self._last_valid_monotonic <= 0.0:
            return
        age = time.monotonic() - self._last_valid_monotonic
        if age < self._stale_after:
            return
        message = (f"{self._topic} 已经 {age:.1f}s 没有新的**有效**观测了 —— 黑板上的 "
                   f"{self._obs_key} 还是 {self._written} 帧前那一份，下游会一直停在"
                   f"这一步等")
        key = _stable_text_key(message)
        if key != self._last_stale_key:
            self._last_stale_key = key
            logger.warning("NodeServoErrorFromTopic %s", message)
        self.feedback_message = message

    def _cleanup_subscriber(self) -> None:
        """换出句柄再注销：`unregister()` 抛异常也不留野句柄。"""
        subscriber, self._subscriber = self._subscriber, None
        if subscriber is not None:
            try:
                subscriber.unregister()
            except Exception:                        # noqa: BLE001
                pass
