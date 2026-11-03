# -*- coding: utf-8 -*-
"""NodePalletServo：读黑板上的托盘位姿与箱子观测 → 算伺服误差 → 写回黑板。

上游是 `NodePalletPose`（写 `latest_pallet`）与 YOLO 适配器（写
`latest_box_obs`）。本节点在它们下游补上「位姿 → 图像空间误差」这一环。

**与 NodePalletPose 的一个区别：本节点也不访问硬件**，但它需要 TF（把
base_link 系的托盘位姿换到相机系）。TF 那部分单独放在 `TfCamBaseResolver`
里，查询器是注入的，所以本文件可以脱离 ROS 单测。

黑板契约（照抄 NodePercep / NodePalletPose 的写法）
    读  latest_pallet / latest_pallet_version          Pose6D
    读  latest_box_obs / latest_box_obs_version        BoxObservation
    写  latest_servo_error / latest_servo_error_version  ServoError

节点持续返回 RUNNING，并且在两个版本号都没变时不重复计算 —— 与
`NodePalletPose.update()` 用的是同一套版本门禁。
"""
from __future__ import annotations

import json
import math          # 参数校验里的 isfinite（角度**不在这里转换**，见 HUD 那段）
import os
import re
import threading     # 底图回调线程与 tick 线程之间的锁
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
import py_trees
import yaml
from py_trees.common import Status

from core.common.logger import get_logger
from core.common.transform import pose6d_to_matrix
from core.common.transform import matrix_to_pose6d
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.utils.blackboard import (
    is_dry_run,
    read_blackboard,
)
from skills.atomic.perception.pallet_frame import (
    PalletFrameWindow,
    PalletObservation,
    PairReject,
)
from skills.atomic.perception.pallet_servo import neighbor as neighbor_algo
from skills.atomic.perception.pallet_servo.algorithm import (
    BoxObservation,
    Reject,
    ServoError,
    box_corners,
    box_edges,
    edge_offset,
    inward_normal,
    parse_bool_param,
    parse_pair_param,
    parse_ref_edges,
)
from skills.atomic.perception.pallet_servo.skill import (
    PalletServoParams,
    PalletServoSkill,
)

logger = get_logger(__name__)

# TF 连续失败多少帧才降级到下一级。先硬编码，后续可做成参数。
TF_FAIL_LIMIT = 5
# 伺服按帧跑，一帧等 1 秒会卡死环路（pallet_calibrate.py 的阻塞查询是给
# 离线标定用的，不能照搬到这儿）
TF_TIMEOUT_SEC = 0.05

# dump 的**节流**：两份 dump 之间至少隔这么久（秒）。
#
# 为什么必须有：伺服按 10 Hz 跑，而"整帧被拒"可以是**持续**状态（托盘太远或太侧
# 时 `edge_too_short` 一直成立，箱子观测却每帧都在更新 → 每帧都是新的版本对）。
# 不节流的话一帧一份 dump（≈1.5 KB / 份）加上三条 WARNING：10 Hz 下 ≈15 KB/s、
# 50 MB/h、30 行/秒，正好把要调试的上下文冲掉。诊断设施不该淹掉它自己的现场。
DUMP_MIN_INTERVAL_SEC = 1.0

# dump 目录**保留最近几份**（轮转）。节流只能压住"写得多快"，压不住"写多久" ——
# 而本任务新增的 `pair_none` / `pair_dt` 在"检测器不填 header.stamp"这类配置错
# 下是**稳态**（不是瞬时）：1 份/秒 × 7×24 = ≈8.6 万个 inode / ≈130 MB 一天，
# 目录确实在 .gitignore 里、不污染 git，但没有清理就是**写爆磁盘**。
#
# 为什么是 200：按默认节流 1 份/秒算是 ≈3 分钟的现场 —— 够把"刚刚发生了什么"
# 完整存下来（异常帧本身 ≤1.5 KB，200 份 ≈300 KB），又不至于无上限。`dump_on:
# "all"` 的调试跑法（10 Hz）下是 20 秒，同样够看；真要长期留档就把 dump_dir
# 指到别处、并把 dump_min_interval_sec 调大，而不是把这个数调大。
DUMP_KEEP = 200

# 本类写出的文件名形状：`%Y%m%d-%H%M%S-000123.json`。轮转**只删匹配它的文件** ——
# dump_dir 是操作员给的，可能指到一个还有别的东西的目录里，删别人的文件不是
# 这个类该干的事。
_DUMP_NAME_RE = re.compile(r"^\d{8}-\d{6}-\d{6}\.json$")

# 读相机内参的话题与超时。**话题名写死、不走 `camera` 参数**：那套
# `/camera/color/...` 的命名是 `CameraAdapter` 的约定，本节点不该依赖它
# （本节点不访问硬件，见 `_read_camera_info` 的 docstring）。
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
# 0.5 秒：`wait_for_message` 在**没有 master** 时也精确按这个时长返回，所以
# 它不会拖住 CI；而真机上话题一直在发，第一帧通常在毫秒级就到。
CAMERA_INFO_TIMEOUT_SEC = 0.5

# 读相机内参**失败**之后隔多久再试一次（秒）。
#
# 为什么要有重试：**相机在树启动那一刻还没起来是真机上很常见的一件事**
# （相机驱动先起、SDK 慢、`roslaunch` 竞态）。从前这条路上失败就**永久**缓存，
# 于是节点恒 FAILURE、每 tick 一条 ERROR、**不可能自愈**，只能重启整棵树 ——
# 而"重进 `initialise()` 会重读一次配置"正是 `_resolve_size()` 保留下来、
# 为的就是"配置修好即自愈"的那条口径，两处不该不一致。
#
# 为什么必须**节流**：`_read_camera_info()` 读不到时要等满
# `CAMERA_INFO_TIMEOUT_SEC`（0.5s），每个 tick 都试就是把 tick 拖成 0.5s 的卡死
# （那正是当初加缓存要解决的事）。5 秒一试把代价钉在"每 50 个 tick 堵 0.5 秒"，
# 而该路本来就恒 FAILURE、不是正常工况；相机起来后**至多 5 秒**自愈。
CAMERA_INFO_RETRY_SEC = 5.0

# 可视化告警的**周期提醒**间隔（tick）。沿用本文件既有的节流口径
# （`_log_throttled` / `NodePalletObs._VALID_FALSE_REMIND_TICKS` 都是
# `self._ticks % N` 这一套），**不新造机制**。
#
# 为什么必须有周期提醒：`_warn_overlay` 从前只按"键变了"去重，于是**长期不过**
# 的自检（托盘贴着画面边缘、探针被别的图层文字盖住）在操作员眼前**彻底安静** ——
# 日志里只有开头一条 WARNING，`/pallet_servo/overlay` 一条消息都不发。而
# "话题一条消息都没有"与"rqt 里话题名写错了"**长得一模一样**，操作员的第一反应
# 恰恰就是后者。检测器停摆专门加了 `live_timeout_s` 防静默，可视化这条路同样
# 不能没有存活口径。
_OVERLAY_REMIND_TICKS = 30


def _make_tf_lookup(timeout_sec: float = TF_TIMEOUT_SEC):
    """起一个**非阻塞**的 TF 查询器：`lookup(target, source) -> 4x4 | None`。

    形态沿用标定工具中的 TF 查询实现，只把 `rospy.Duration(1.0)` 换成短超时
    —— 标定是离线的，等一秒无所谓；伺服是每帧的，等一秒就把环路卡死了。

    没有 ROS（或起了监听但连不上 master）时返回 None，调用方据此走参数/单位阵
    回退。**构造这一整段都要包在 try 里**：单测环境里没有 master，
    `TransformListener(...)` 会抛 `ROSInitException`，那会让一个本该"优雅降级"
    的路径变成节点初始化直接崩掉。
    """
    try:
        import rospy
        import tf2_ros

        buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(buffer)        # 保持引用，否则被回收
        timeout = rospy.Duration(timeout_sec)
    except Exception as exc:                     # noqa: BLE001  # pragma: no cover
        logger.warning("起不了 TF 监听（%s: %s），T_cam_base 只能走参数或单位阵",
                       type(exc).__name__, exc)
        return None

    def lookup(target_frame: str, source_frame: str) -> Optional[np.ndarray]:
        try:
            tf = buffer.lookup_transform(target_frame, source_frame,
                                         rospy.Time(0), timeout)
        except Exception as exc:                 # noqa: BLE001
            logger.debug("TF %s <- %s 查不到: %s", target_frame, source_frame, exc)
            return None
        from scipy.spatial.transform import Rotation as R
        t, q = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    return lookup


class TfCamBaseResolver:
    """解析 `T_cam_base`：TF → 显式参数 → 单位阵，逐级回退。

    伺服要的是 `T_cam_pallet = T_cam_base @ T_base_pallet`，而黑板上的
    `latest_pallet` 约定是 base_link 系的位姿。

    **与先例的偏离（有意）**：`pallet_calibrate.py` 的 `make_tf_lookup()` 用
    `rospy.Duration(1.0)` 阻塞查询 —— 离线标定没问题，伺服按帧跑会卡死环路。
    这里用的是**短超时 + 最近一次成功值缓存**：查不到就沿用上一帧的
    `T_cam_base`，连续 `fail_limit` 帧失败（且没有缓存可用）才降级到下一级，
    降级时置一次 WARNING。TF 一旦恢复就立刻回到 `"tf"` 并清零计数，不是
    永久降级。

    `src` 的四个取值与设计文档 §5.4 的 `t_cam_base_src` 一一对应。
    """

    def __init__(self, lookup, camera_frame: str, base_frames: Sequence[str],
                 param=None, fail_limit: int = TF_FAIL_LIMIT) -> None:
        self._lookup = lookup
        self._camera_frame = str(camera_frame)
        self._base_frames = [str(f) for f in base_frames]
        self._fail_limit = max(1, int(fail_limit))
        self._param: Optional[np.ndarray] = None
        if param is not None:
            arr = np.asarray(param, np.float64)
            if arr.shape != (4, 4):
                raise ValueError(f"T_cam_base 参数形状必须是 4x4，实际 {arr.shape}")
            self._param = arr
        self._cache: Optional[np.ndarray] = None
        self._fails = 0
        self._degraded = False

    @property
    def fail_count(self) -> int:
        """连续失败帧数。给日志和测试看。"""
        return self._fails

    def resolve(self) -> Tuple[np.ndarray, str]:
        """返回 `(T_cam_base, src)`，`src ∈ {"tf", "cache", "param", "identity"}`。"""
        T = self._try_tf()
        if T is not None:
            if self._degraded:
                logger.warning("T_cam_base 的 TF 恢复了，回到 tf（之前连续失败 %d 帧）",
                               self._fails)
            self._cache = T
            self._fails = 0
            self._degraded = False
            return T, "tf"

        self._fails += 1
        if self._cache is not None and self._fails < self._fail_limit:
            return self._cache, "cache"

        # 走到这儿有两种情况：连续失败够多了，或者压根没有缓存可用
        if not self._degraded:
            self._degraded = True
            logger.warning(
                "T_cam_base 降级：TF 连续失败 %d 帧"
                "（相机帧 %s，基座候选 %s），改用 %s",
                self._fails, self._camera_frame, self._base_frames,
                "显式参数" if self._param is not None else "单位阵")
        if self._param is not None:
            return self._param, "param"
        return np.eye(4), "identity"

    def _try_tf(self) -> Optional[np.ndarray]:
        if self._lookup is None:
            return None
        for base in self._base_frames:
            try:
                T = self._lookup(self._camera_frame, base)
            except Exception as exc:             # noqa: BLE001
                logger.debug("TF 查询异常（%s <- %s）: %s", self._camera_frame,
                             base, exc)
                continue
            if T is not None:
                return np.asarray(T, np.float64)
        return None


# 基于源码位置解析，不依赖进程工作目录（框架里 lifecycle_mixin 也是这么做的）
_DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "pallet_tag.yaml"


# 写 dump 时 numpy 数组要变成 list 才塞得进 JSON
def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def _ros_node_initialized(rospy_module) -> bool:
    """`rospy.init_node()` 起过没有。**判不出来（假 rospy / 老版本）当"没有"。**

    为什么建发布器之前要先问这一句（与 `NodePalletObs.initialise()` 同一句先例）：
    `rospy.Publisher(...)` 在没有 `init_node()` 时**自己就会抛**
    （`ROSException: ROS node has not been initialized yet`），而且建出来也发不出去
    —— 那不是"这个话题起不来"，是"现在还没有 ROS"（离线跑 / 单元测试）。

    不判的话，凡是没 `init_node()` 的进程每次建树都会多一条
    「伺服误差话题起不来」的 WARNING，而**真正的**"`pallet_servo_msgs` 没编译"
    就淹进同一类噪声里 —— 与 `live_timeout_s` 那条 F1 是同一个病。
    """
    core = getattr(rospy_module, "core", None)
    is_initialized = getattr(core, "is_initialized", None)
    try:
        return bool(is_initialized()) if callable(is_initialized) else False
    except Exception:                                # noqa: BLE001
        return False


class DumpWriter:
    """异常帧的**完整原始输入**落盘。

    **日志给人读，dump 给机器复现。** 4×4 矩阵塞进日志行没法看，塞进 JSON
    可以直接喂回 `algorithm.servo_error`（它是纯函数、零依赖）重跑——这是
    design §6.1 第二条硬要求（"dump 必须完整到能重跑"）的落地。

    触发条件可配：`dump_on` 默认 `["reject", "warn"]`（拒绝帧、带 warn 的帧
    都写），也接受 `"all"`（**调试期用**——伺服按 10 Hz 跑，全量落盘仍会
    持续写盘）。`dump_dir` 为 None 或 `dump_on` 为空则整体关闭。

    **限流（`min_interval_sec`，默认 `DUMP_MIN_INTERVAL_SEC`）**：两份 dump 之间
    至少隔这么久，且与上一份**逐字相同**的直接不写。理由见那个常量的注释 ——
    默认配置下"每帧都被拒"可以持续很久（托盘太远/太侧），不拦的话是一帧一份
    文件、每份一条 WARNING。被跳过的份数不是悄悄丢掉：**算在下一份的日志里**
    （逐字相同几份、离得太近几份）。要"每帧一份"就把 `dump_min_interval_sec`
    设成 0（`dump_on: "all"` 的调试跑法就这么用）。

    **轮转（`keep`，默认 `DUMP_KEEP`）**：目录里**只留最近 `keep` 份**，多出来的
    从最旧的开始删。节流管的是"写得多快"，轮转管的是"写多久" —— 上面那个 1 份/秒
    的口径本身没有终点，7×24 就是写爆。只删**本类写出的文件名**（`_DUMP_NAME_RE`），
    dump_dir 里别的东西一个不动；删不掉（目录被删/权限不对）只记 DEBUG，绝不抛。

    不认识的写法（少一层方括号的 `"reject"`、把 `"all"` 放进列表里）**不会
    静默关闭落盘**，而是记一条点名了实际值与可选值的 WARNING（§7）；目录建
    不出来（`dump_dir` 指到文件底下）同样只 WARNING 并关掉落盘，不抛。

    ⚠️ **一个已知的洞**：某帧既没被拒也没 warn、但数值悄悄偏了，那一帧就
    没有 dump。它靠两件事兜：节流日志里每帧都有三个量的值（偏了看得出来），
    以及离线阶段输入本来就来自 `pallet_servo_sim.json`（复现是免费的）。
    dump 主要是给**真机**用的——那里没有可重放的输入。
    """

    # `dump_on` 的**列表**取值。字符串形式另有 `"all"`。§7：不认识的写法要
    # 点名声张，不能静默变成"永不落盘"。
    TOKENS = ("reject", "warn")

    # 写失败后隔多少次再喊一声（第一次必喊，之后只计数 —— 与 `_make_dir` 的
    # "一次后关掉"同一个口径：盘满时逐帧 WARNING 会把上下文冲掉）
    FAIL_REPORT_EVERY = 100

    def __init__(self, dump_dir, dump_on,
                 min_interval_sec: float = DUMP_MIN_INTERVAL_SEC,
                 keep: int = DUMP_KEEP) -> None:
        self._dir = Path(str(dump_dir)) if dump_dir else None
        self._seq = 0
        self._mode = ""
        self._on = frozenset()
        self._min_interval = max(0.0, float(min_interval_sec))
        try:
            self._keep = max(1, int(keep))
        except (TypeError, ValueError):
            # 轮转份数是个可选的构造参数（目前由 `DUMP_KEEP` 给死，不经节点参数），
            # 照样不抛：一个数字写错不该让诊断设施的构造失败。
            logger.warning("dump keep 不是整数（%r）—— 按默认值 %d 处理", keep,
                           DUMP_KEEP)
            self._keep = DUMP_KEEP
        self._last_write_at: Optional[float] = None
        self._last_text: Optional[str] = None
        self._skipped_same = 0
        self._skipped_rate = 0
        self._fail_count = 0
        self._parse_dump_on(dump_on)
        self._enabled = self._dir is not None and (
            self._mode == "all" or bool(self._on))
        if self._enabled:
            self._make_dir()

    def _parse_dump_on(self, dump_on) -> None:
        """解析 `dump_on`：认识的值照用，不认识的**点名 WARNING**（§7）。

        两条老路都会**静默**地把落盘关掉：`dump_on: "reject"`（少写了一层
        方括号）与 `dump_on: ["all"]`（`"all"` 只认字符串形式）。真出问题要
        dump 的那一刻，现场只会看到"怎么一个 dump 都没有" —— 配置错误必须
        当场说出来。
        """
        if isinstance(dump_on, str):
            token = dump_on.strip().lower()
            if token == "all":
                self._mode = "all"
            elif token:          # 空串 = 关闭，是文档里写明的写法，不吵
                logger.warning(
                    "dump_on 不认识：%r —— 字符串形式只认 \"all\"（全量落盘）；"
                    "要按帧选就写成列表，取值来自 %s。**本次不落盘**。",
                    dump_on, list(self.TOKENS))
            return
        try:
            tokens = [str(t).strip().lower() for t in (dump_on or [])]
        except TypeError:
            logger.warning("dump_on 既不是字符串也不是列表：%r —— 只认 \"all\" "
                           "或 %s 组成的列表。**本次不落盘**。",
                           dump_on, list(self.TOKENS))
            return
        unknown = sorted({t for t in tokens if t not in self.TOKENS})
        if unknown:
            logger.warning("dump_on 里有不认识的值：%s —— 只认字符串 \"all\" "
                           "或 %s 组成的列表；这几项忽略，其余照常。",
                           unknown, list(self.TOKENS))
        self._on = frozenset(t for t in tokens if t in self.TOKENS)

    def _make_dir(self) -> None:
        """建 dump 目录；建不出来就**关掉落盘**，但绝不抛。

        `dump_dir` 指到一个文件底下（`mkdir` 直接 NotADirectoryError）之类的
        配置错误，如果抛出去，爆的是 `NodePalletServo.__init__` —— 一个诊断
        设施把节点构造搞崩，正好和它的用途相反。
        """
        try:
            self._dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self._enabled = False
            logger.warning("dump 目录建不出来（%s）：%s: %s —— 本次运行不落盘，"
                           "伺服照常跑", self._dir, type(exc).__name__, exc)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def should_write(self, *, rejected: bool, warn: Sequence[str]) -> bool:
        if not self._enabled:
            return False
        if self._mode == "all":
            return True
        if rejected and "reject" in self._on:
            return True
        return bool(warn) and "warn" in self._on

    def write(self, payload: dict) -> Optional[Path]:
        """写一个 dump。文件名按时间排序，同一秒内靠序号保证不撞。

        返回 None 有四种情况，**都不是异常**：落盘关着、与上一份逐字相同、
        离上一份不足 `min_interval_sec`、真写失败了。

        **写不进去也不抛**：盘满、目录被删、权限不对，都只记一条带路径与异常
        的 WARNING 然后返回 None —— dump 是诊断设施，它的失败不该掀翻伺服环路。

        宽接 `Exception` 而不是只接 `OSError`：`json.dumps` 遇到序列化不了的
        东西同样会抛，那也同样是"dump 没写成"，同样不该掀翻环路（与
        `_make_tf_lookup` 的宽接同一套理由）。

        写失败会**逐帧重试**（失败不占额度：没有文件就没有体积问题），但日志
        去重：第一次 WARNING 带完整原因，之后只记账，每 `FAIL_REPORT_EVERY`
        次再喊一声（帧率下盘满时不再是 10 条/秒）。
        """
        if not self._enabled:
            return None
        try:
            text = json.dumps(_jsonable(payload), indent=1,
                              ensure_ascii=False, default=str)
        except Exception as exc:                 # noqa: BLE001
            self._report_failure(None, exc, "序列化")
            return None

        # 与上一份逐字相同：同一个原因、同一组输入反复触发时（版本门禁放在
        # dump 之后，箱子不动就只有它重复）不必再写一份同样的文件
        if text == self._last_text:
            self._skipped_same += 1
            logger.debug("托盘伺服 dump 与上一份逐字相同，跳过（累计跳过 %d 份）",
                         self._skipped_same)
            return None

        now = time.monotonic()
        if (self._last_write_at is not None
                and now - self._last_write_at < self._min_interval):
            self._skipped_rate += 1
            logger.debug("托盘伺服 dump 离上一份不足 %.1f s，跳过（累计跳过 %d 份）",
                         self._min_interval, self._skipped_rate)
            return None

        name = (time.strftime("%Y%m%d-%H%M%S")
                + f"-{self._seq:06d}.json")
        self._seq += 1
        path = self._dir / name
        try:
            path.write_text(text, encoding="utf-8")
        except Exception as exc:                 # noqa: BLE001
            self._report_failure(path, exc, "写失败")
            return None

        self._last_write_at = now
        self._last_text = text
        self._fail_count = 0
        skipped = self._skipped_same + self._skipped_rate
        if skipped:
            logger.warning("托盘伺服写下 dump：%s（此前跳过 %d 份：逐字相同 %d、"
                           "距上一份不足 %.1f s 的 %d；这两类的跳过量都封顶在"
                           "默认行为里，要看每一帧就把 dump_min_interval_sec 设成 0）",
                           path, skipped, self._skipped_same, self._min_interval,
                           self._skipped_rate)
        else:
            logger.warning("托盘伺服写下 dump：%s", path)
        self._skipped_same = 0
        self._skipped_rate = 0
        self._prune()
        return path

    def _prune(self) -> None:
        """只留最近 `keep` 份，多出来的**从最旧的开始删**（按文件名，它按时间排序）。

        三条边界，都是"这不该有能力把伺服弄死"那一条的延伸：

        * **只删本类写出的文件名**（`_DUMP_NAME_RE` 精确匹配）。`dump_dir` 是操作员
          给的，可能指到一个还有别的东西的目录里 —— 轮转是给 dump 自己收尸，不是
          替人清目录。
        * **删不掉不算失败**：目录被删、权限不对、文件正被打开，都只记 DEBUG
          （每次写都轮转，WARNING 会按帧率刷屏 —— 与 `_report_failure` 的"第一次
          必喊"不同，这里没有任何一次值得喊：留下的文件数不对，看一眼 `ls` 就知道）。
        * **绝不抛**：`listdir` / `unlink` 的任何异常都吞掉，理由同上。
        """
        if self._dir is None:
            return
        try:
            names = sorted(n for n in os.listdir(self._dir)
                           if _DUMP_NAME_RE.match(n))
        except Exception as exc:                     # noqa: BLE001
            logger.debug("托盘伺服 dump 轮转列目录失败（%s）：%s", self._dir, exc)
            return
        for name in names[:-self._keep]:             # 少于 keep 份时切片本就是空的
            try:
                (self._dir / name).unlink()
            except Exception as exc:                 # noqa: BLE001
                logger.debug("托盘伺服 dump 轮转删旧份失败（%s）：%s", name, exc)

    def _report_failure(self, path, exc, what: str) -> None:
        """dump 没写成：第一次（以及每 `FAIL_REPORT_EVERY` 次）WARNING，其余只计数。"""
        self._fail_count += 1
        count = self._fail_count
        where = "" if path is None else f"（{path}）"
        if count == 1 or count % self.FAIL_REPORT_EVERY == 0:
            logger.warning("托盘伺服 dump %s%s：%s: %s —— 这一帧照常输出，只是没"
                           "留下 dump（本次运行第 %d 次）",
                           what, where, type(exc).__name__, exc, count)
        else:
            logger.debug("托盘伺服 dump %s 第 %d 次：%s", what, count, exc)


class NodePalletServo(BaseAction):
    """算图像空间的三个伺服误差量，写到黑板。

    params:
        pallet_key        读托盘位姿的键，默认 latest_pallet
        box_key           读箱子观测的键，默认 latest_box_obs
        key               写伺服误差的键，默认 latest_servo_error
        ref_edges         参考边**槽位表**。两种形状都认：
                            * `["y=0","x=W"]` —— 只有一组（单箱位，等价于老写法）
                            * `[["y=0","x=1200"], ["y=800","x=1200"]]` —— N 组，
                              服务里第 N 个箱子用第 N 组
                          每组两条边，第 1 个是底边、第 2 个是右边。
                          ⚠️ **档位总数 = `ref_edges` 组数与 `servo_slots` 长度
                          的较大者**（2026-10-17）。比 `servo_slots` **短**是允许
                          的（后面几档按 `pallet` 兜底，那些档照样有参考边）—— 但
                          那时**档位数不受它限制**，多出来的邻居档照常能 call。
                          **启动时是"未激活"**（`_active_slot == 0`）：一帧有效数都不
                          出，话题上发 `valid=false` 的「伺服未启动」—— 要
                          `rosservice call /pallet_servo/slot "slot: N"` 才出数，
                          `slot: 0` 结束本次伺服。⚠️ **服务从 1 数、数组从 0 数**
                          选错边不会有任何报错（三个数照样算得出来，只是基准是
                          隔壁箱位的）
        servo_slots        **每个箱位走哪一套伺服**（2026-10-17 加），形状
                          `[{"slot": 1, "mode": "pallet"},
                            {"slot": 3, "mode": "neighbor", "side": "right"}, ...]`。
                          `mode` 只认 `pallet` / `neighbor`，`side` 只认
                          `left` / `right` / `none`（**画面口径**），写错**启动时**
                          报配置错误（不会静默按 pallet 跑 —— 那会让本该走邻居的
                          档拿"托盘看不见"的位姿去算，而照样算得出三个数）。
                          **不配 / 配成空数组 = 全部按 `pallet`**，行为与加这个键
                          之前完全一致。表比 `ref_edges` 短是合法的（后面几档都走
                          老路线）。**比它长也是合法的 —— 只要多出来那几档是
                          `neighbor`**（邻居档不读参考边；`ref_edges` 只是"档位
                          总数"的定义源之一，见 `initialise()` 里逐档判那一段）。
                          多出来那档若是 `pallet` 则**启动时报配置错误**。
                          可选 `"e_right_bias_px"` / `"e_bottom_bias_px"`：该档
                          最终的 `e_right` / `e_bottom` 加对应偏置
                          （`输出 = 原始值 + bias`，服务/话题/叠加图用的都是
                          加过的量）。默认都是 0。
        candidates_key    邻居路线读的**候选列表**键，默认 latest_box_candidates
                          （`NodeBoxObs` 写）。只有 `mode == "neighbor"` 的档用
                          它；托盘路线一个字都不看
        pallet_size_mm    台面尺寸 [W, H] 毫米。**运行时优先用上游检测器上报的
                          `{pallet_key}_size_mm`**；这个参数只是上游还没出结果时
                          的兜底（再退到 config_path）。主流程已不配它，pallet 档
                          会等上游尺寸；neighbor 档不需要尺寸
        config_path       标定结果路径，默认 config/pallet_tag.yaml
        K                 相机内参 3x3（**必须是嵌套的 3 行 x 3 列**；平铺 9 个数
                          不算，形状不对在启动时就被拒）。
                          **不给就自动从 `/camera/color/camera_info` 读**
                          （见 camera 参数）—— 真机上不用手抄，抄错一个 `cx`
                          会让所有像素量整体偏而三个数照样算得出来
        D                 畸变系数；不给时也先问相机（camera_info 里有就采信），
                          相机也没有才表示无畸变
        use_distortion    投影要不要带畸变，默认 true。**字符串布尔也认**
                          （true/false、1/0、yes/no）——行为树工厂的 READ_BOARD
                          分支不做类型转换，`"false"` 是真实会到的写法；解释不了
                          的值按默认 true 处理并记 WARNING
        image_size        [w, h]；给了就顺带查"整条边跑到图外"。
                          形状不对在**启动时**就报配置错误。
                          **不给就自动从 camera_info 的 width/height 取**
        camera            向哪个相机要内参，默认 camera（Orbbec 头部的命名空间）。
                          只在 K / D / image_size **没给全**的时候才会去问
        camera_frame      相机光学帧名，默认 camera_color_optical_frame
        pallet_frame      黑板上的托盘位姿按哪个坐标系解释，默认 "camera"。
                          "camera" = 相机系（视觉托盘检测链路，**不查 TF**、
                          **走配对**）；"base_link" = 老路径（NodePalletPose，
                          查 TF、**绕开配对**，与改动前逐位一致）。
                          **认不出来的值退回 "camera"** 并记 WARNING
        base_frames       基座帧候选，按顺序试，默认三个
        T_cam_base_param  显式 4x4，TF 查不到且缓存也失效时用
        max_dt_s          配对允许的最大时间戳差（秒），默认 0.05（30fps 下一帧
                          33ms，留一帧半的余量）。**只认正有限值**，否则退回
                          默认值并记 WARNING
        pair_cache        每侧留几个观测供配对，默认 5。**不是窗长** —— 与
                          `window` 是两个独立旋钮（addendum A2）。
                          ⚠️ **回溯窗口 ≈ `pair_cache` × 伺服 tick 周期**：push 是
                          按本节点的 tick 发生的（版本变了才推），10 Hz 下 `5`
                          ≈ **500 ms**。**必须大于托盘检测器的端到端延迟**，否则
                          慢的那一路交出来的帧早被挤出去了，配对只会一直报
                          `pair_none` / `pair_dt`
        window            成对序列上的窗长，默认 3（1 = 不平滑）
        agg               多对的聚合方式，**只认 "mean" / "median"**，默认 "mean"；
                          拼错退回 "mean" 并记 WARNING（可见性归本层，A3）
        stale_s           配对成功后这一对有多旧算陈旧（0 = 不查），默认 0.0。
                          ⚠️ **这是"延迟闸"不是"存活闸"** —— 它判的是"配上的
                          这一对"有多旧（判在去重之后），某一路停摆**永远轮不到
                          它**；存活判据是 `live_timeout_s`，两者别混
        live_timeout_s    **存活闸**：某一侧多久没有新的观测就记一次 WARNING
                          （写清"多久没出数"），默认 5.0（0 = 不查）。
                          **只认非负有限值**，否则退回默认值并记 WARNING ——
                          0 是写明的"不查"，负值若也按 0 处理就是**静默关掉**
                          唯一的检测器存活告警。同一条告警**按停摆侧别去重**，
                          不是每 tick 一遍。它补的是 `resolve()` 返回 `None` 那两种
                          含义**不可区分**的那一格（addendum A4）
        log_every_n       正常帧的节流日志间隔，默认 30
        dump_on           异常帧落盘触发条件，默认 ["reject", "warn"]；"all" = 全量；
                          不认识的值记 WARNING 并关掉落盘（不会静默）
        dump_dir          落盘目录，默认 log/pallet_servo_dump；给空则关闭
        dump_min_interval_sec  两份 dump 之间至少隔多久，默认 1.0 s（见
                          `DUMP_MIN_INTERVAL_SEC`）；要每帧一份就设 0
        overlay_out       叠加图发到哪个话题，默认 /pallet_servo/overlay；
                          **给空串 = 完全不发图**（省 CPU、省带宽）。
                          ⚠️ **不是"算出三个数"才发**：托盘与纸箱**各画各的**，
                          缺哪一路就不画哪一层（含未激活与被拒的帧）——
                          只发全套的那种做法会让"只有一边有结果"的帧整张图消失，
                          而现场分不出那是**这一路检测没出东西**还是
                          **叠加图这条链坏了**。HUD 上写明缺在哪一步
        overlay_period_s  发图的降频周期，默认 0.2 s。伺服 10 Hz，图 5 Hz 就够看，
                          而画图 + 编码不便宜
        image_color       底图话题，默认 /camera/color/image_raw
        servo_error_out   伺服误差发到哪个话题，默认 /pallet_servo/dis；
                          **给空串 = 完全不发**（连发布器都不建）。它是
                          **功能性输出**（下游控制器要用），默认**开**
        servo_error_heartbeat_s  无效帧的重发周期，默认 1.0 s。`0` = 只在内容
                          改变时发 —— 但要接受"停摆了就彻底没消息"，那与
                          "话题名写错了"分不开

        ⚠️ 从前还有一个 `box_uv_in`（默认 `/box/yolo_box`）用来在叠加图上画灰框，
        **2026-10-15 连同那条话题一起删了**：它发的是箱子四角的轴对齐包围盒，而
        黑板上的 `BoxObservation.u1/v1/u2/v2` 本来就是同一个数（`node_box_obs` 用
        `min/max(corners)` 填的）。现在灰框直接从手上那个 `box` 算，少一路订阅。
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._pallet_key = str(self.params.get("pallet_key", "latest_pallet"))
        self._box_key = str(self.params.get("box_key", "latest_box_obs"))
        self._key = str(self.params.get("key", "latest_servo_error"))
        # 参考边**槽位表**：第 N 组服务里第 N 个箱子用。形状归一在这里做，
        # 合法性（`parse_ref_edges`：写法 + 不平行）在 `initialise()` 里做。
        self._slots = _normalize_slots(self.params.get("ref_edges",
                                                       ["y=0", "x=W"]))
        # ★ **每个箱位走哪一套伺服**（2026-10-17 加）：托盘路线还是邻居路线。
        #   形状归一在这里做，合法性（`mode` / `side` 的白名单）**也在这里判** ——
        #   它不依赖尺寸，没有"等 `initialise()` 才有参照物"这回事（与
        #   `ref_edges` 的绝对写法不同），所以当场就能报。
        #   ⚠️ **缺省 = 全部按 `pallet`**：行为与加这个键之前**逐字节一致** ——
        #      没配这个键的板子（全仓除了本场景之外一个都没有）一行都不用动。
        self._slot_modes, mode_err = _normalize_servo_slots(
            self.params.get("servo_slots"))
        # ⚠️ **不在这里写 `self._config_err`**：它要到下面（K/D/image_size 那一段）
        #    才被置成 `None` 再逐项折进来，写在这里会被那一句擦掉。所以先存着，
        #    由那一段一次性 `or` 进去。
        self._servo_slots_err = mode_err
        # ★ **档位总数 = 两者的较大者**（2026-10-17 加）。`ref_edges` 从前是
        #   档位数量的**唯一**定义源（服务、叠加图、日志都按它数），于是
        #   `servo_slots` 比它长时多出来的那几档**永远 call 不到** —— 真机实测
        #   4 组 ref_edges + 5 档表 → 第 5 档被拒成"表里只有 4 组"。邻居档
        #   根本不读 `ref_edges`，所以"表更长"本身完全合法。
        #   ⚠️ `servo_slots` **短**时取 `len(self._slots)`：后面几档没写就按
        #      `pallet` 兜底（`_slot_mode_of` 的既有行为，那个兜底要求有参考边
        #       —— `initialise()` 里有逐档校验兜着）。
        #   这里只算长度，**合法性的判定在 `initialise()`**（那里才报得出配置
        #   错误）：`servo_slots` 比 `ref_edges` 长、且多出来那档是 `pallet`，
        #   是真错（pallet 档必须有基准）。
        self._total_slots = max(len(self._slots), len(self._slot_modes))
        # 候选列表的键：邻居路线要用它（`node_box_obs` 写、本节点读）。
        # 键名可配，与生产者那边的 `candidates_key` 是**同一串默认值**。
        self._candidates_key = str(self.params.get(
            "candidates_key", "latest_box_candidates")).strip()
        # **启动时未激活**（0）：没 call 过服务之前一帧有效数据都不出。
        # 隐式假设"默认第 1 组"会在板子加了箱位、或换场景从第 2 组开始时
        # **静默用错** —— 那是最难查的一类错。
        self._active_slot = 0
        self._slot_srv = None
        # 槽位**版本号**：`_activate_slot` 每次生效就 +1（服务回调线程写），
        # `update()` 读到版本变了就把配对窗的水位线清掉重算（tick 线程读）。
        # 服务那侧因此只做"校验 + 赋一个数"，不碰配对窗（§5.6 的纪律）。
        self._slot_version = 0
        self._slot_seen = 0
        self._size_param = self.params.get("pallet_size_mm")
        # 上游报的尺寸写在 `{pallet_key}_size_mm`（`NodePalletObs` 写、本节点读）。
        # ⚠️ 后缀**必须与那边逐字一致** —— 对不上不会报错，只会永远读不到、
        # 静默退回配置（那正是这次要修的那个静默）。
        self._size_key = f"{self.params.get('pallet_key', 'latest_pallet')}_size_mm"
        self._config_path = Path(str(self.params.get("config_path",
                                                     _DEFAULT_CONFIG)))
        # 布尔参数**不许用 `bool()` 顶替**：`bool("false") is True`，而工厂的
        # READ_BOARD 分支真会传字符串 bool（`parse_bool_param` 的 docstring 把
        # 这个坑写全了）。解释不了的值退回**本参数的默认值**（true）并点名报出
        # 实际值 —— 这里绝不抛：`initialise()` 抛出会掀翻整棵树，而它只是
        # 一个"说不清是开还是关"的参数。
        try:
            self._use_distortion = parse_bool_param(
                self.params.get("use_distortion", True), "use_distortion")
        except ValueError as exc:
            self._use_distortion = True
            logger.warning("NodePalletServo 参数 use_distortion 解释不了：%s —— "
                           "按默认值 true（带畸变）处理。**它与算像素的那一侧"
                           "不一致会让三个量静默算错**，要关就写 false/0/no",
                           exc)
        self._camera_frame = str(self.params.get(
            "camera_frame", "camera_color_optical_frame"))

        # `pallet_frame`：黑板上的托盘位姿是哪个坐标系。
        #
        #   "camera"（**默认**）—— 相机系。本仓库的视觉托盘检测链路走这条：
        #       `NodePalletObs` 写的就是相机系位姿，`T_cam_base` 是**声明过的**
        #       单位阵，`TfCamBaseResolver` 整个不构造。**这条路走配对**（见下面
        #       `_paired_inputs`：两个检测器的耗时差一个量级，必须按同一帧图像配对）。
        #   "base_link"         —— 老路径（`NodePalletPose` 写 base_link 系位姿），
        #       行为与改动前逐位一致。**这条路绕开配对**（addendum A10）：配对存在
        #       的理由是两个视觉检测器的耗时差，而老路径的生产者**不写
        #       `latest_pallet_stamp`**，配对在它上面只会产出空 —— 而 spec §7.1
        #       要求它逐位一致，两者不可兼得，以回归要求为准。
        #
        # **为什么默认改成 camera**：TF 断掉时 `TfCamBaseResolver` 会**静默**退回
        # 单位阵（只有一条 WARNING），而 base 系的位姿被当成相机系直接投影，
        # 三个数看着像模像样。声明成 "camera" 之后这条回退路径**根本不存在**。
        #
        # ⚠️ **认不出来的值退回 camera**（不是 base_link）：退回 camera 不查 TF、
        # 没有静默回退路径，方向是安全的；退回 base_link 会凭空多一条静默路。
        raw_frame = str(self.params.get("pallet_frame", "camera")).strip().lower()
        if raw_frame in ("camera", "base_link"):
            self._pallet_frame = raw_frame
        else:
            self._pallet_frame = "camera"
            logger.warning(
                "NodePalletServo 参数 pallet_frame 解释不了：%r —— 按默认值 "
                "camera（相机系）处理。**它决定黑板上的位姿按哪个系解释**，"
                "认错会让参考边整体偏掉而三个数照样算得出来；要老路径就写 "
                "base_link", raw_frame)

        # ---- 配对 + 平滑 ----------------------------------------------------
        # 托盘与箱子的检测耗时可能差一个量级（箱子实测 ~60ms），两者必须来自
        # **同一帧图像**才能相减 —— 否则误差里混着相机运动，而三个数照样算得出来。
        #
        # 配对的依据是 `header.stamp`（**输入图像的采集时刻**，不是处理完成
        # 时刻）。一旦按图像时刻配对，耗时差自动消失：两个检测器看的是同一帧
        # 图像，只是算完的时刻差了几百毫秒。
        #
        # **配不上就 reject，不猜、不外推、不用旧的。** 托盘检测慢的时候输出率
        # 被它卡住 —— 这是刻意的：宁可少出数，不出错数。
        #
        # ⚠️ 参数校验（addendum A3）：坏值**退回有意义的默认值 + 点名 WARNING**，
        # 绝不依赖"0 会在下层被拒"来暴露配置缺失 —— 那要等到第一帧才发现。
        self._max_dt_s = _positive_float_param(
            self.params.get("max_dt_s", 0.05), 0.05, "max_dt_s")
        self._pair_cache = _int_at_least(
            self.params.get("pair_cache", 5), 1, 5, "pair_cache")
        self._window = _int_at_least(self.params.get("window", 3), 1, 3, "window")
        # `agg` 的**白名单规范化**（addendum A3）：`average_pairs` 对拼错的 `agg`
        # 静默退回 `mean`（那是有意的 —— 纯函数、零状态、不 log），所以**可见性
        # 归本层**：这里读的是 ROS rosparam / 场景 JSON，认不出来就在这里说。
        raw_agg = str(self.params.get("agg", "mean")).strip()
        if raw_agg in ("mean", "median"):
            self._agg = raw_agg
        else:
            self._agg = "mean"
            logger.warning(
                "NodePalletServo 参数 agg 认不出来：%r —— 退回 mean（只认 "
                "\"mean\" / \"median\"）。**它只影响箱子四角的聚合方式**，"
                "托盘位姿一律走刚体平均；拼错不会出错数，但会让你以为在用中位数",
                raw_agg)
        # ⚠️ `stale_s` 是**延迟闸**，不是**存活闸**（addendum A4）：它管的是
        # 「**配对之后**这一对有多旧」，判在去重**之后** —— 某一路停摆的对根本
        # 走不到它，那时 `resolve()` 返回的是 `None`。而且 `max_dt_s` 判在它之前：
        # 一个 500ms 的托盘延迟会**先**被判 `pair_dt`，`stale` 轮不到。
        # 现场要配它之前先确认它大于该检测器的正常端到端延迟，否则会误报。
        self._stale_s = max(0.0, _float_or(self.params.get("stale_s"), 0.0, "stale_s"))
        self._pairing = PalletFrameWindow(
            max_dt_s=self._max_dt_s, pair_cache=self._pair_cache,
            window=self._window, agg=self._agg, stale_s=self._stale_s)
        # 存活闸：某一侧多久没有**新的观测**就记一次 WARNING（见 `_check_liveness`）。
        # **不要拿 `stale_s` 顶替它** —— 两者判的是完全不同的东西（见上）。
        #
        # ⚠️ **负值要点名 + 退默认**，不能像早先那样 `max(0.0, ...)` 钳成 0：
        # 0 在 `_check_liveness` 里是"不查"，于是 `-1` 这种手滑会**静默废掉**
        # 唯一的检测器存活告警（A3：坏值必须点名）。0 本身是文档里写明的"不查"，
        # 照收不吵。
        # ⚠️ **内联默认值不能省**（`self.params.get(键, 5.0)`）：本仓库**没有任何
        # 配置写过这个键**，`get()` 不带默认值时键不存在就是 `None`，而
        # `_nonnegative_float_param` 只认数字 —— 于是每次建树都喊一条
        # "不是数字（None）"的**假 WARNING**。真写了个 `-1` 时操作员看到的是
        # 同一类噪声，M-3 要的"坏值点名"就此被淹没。同批的 `max_dt_s` /
        # `pair_cache` / `window` / `agg` 都带了内联默认值，只有这个漏了。
        #
        # 默认值 5.0 是**操作员定的**（2026-09-24，原为 3.0）：
        # 「timeout 增加到 5s」。比托盘检测器单帧 ~0.9s 长一个量级，
        # 不会被正常的检测抖动误报。
        self._live_timeout_s = _nonnegative_float_param(
            self.params.get("live_timeout_s", 5.0), 5.0, "live_timeout_s")
        # 上一次推给窗的版本号 —— 判"有没有新的观测"
        self._last_pallet_version = -1
        self._last_box_version = -1
        # ★ **邻居路线的输入去重键** `(箱子版本, 候选版本)`。行为树主循环固定
        #   50Hz，邻居路线不走配对窗 —— 不去重就是每个 tick 重算同一个框
        #   （实测话题上 50Hz，而相机只有 30Hz、检测器还更慢）。
        #   `None` = 还没算过（换档位 / 复位时也要清回 None，否则新档位第一帧
        #   会拿"上一档算过的那一对版本"当自己的、白等一拍）。
        self._last_neighbor_key = None
        # 存活闸的两个时钟（`time.monotonic()`，与 stamp 的时钟无关）
        self._last_pallet_seen = 0.0
        self._last_box_seen = 0.0
        # 上一次喊过的**停摆侧别集合**（不是文案 —— 文案里的时长每 tick 都在变，
        # 按原文去重会每 tick 一条。见 `_check_liveness`）
        self._last_live_warn: Optional[Tuple[str, ...]] = None

        self._base_frames = list(self.params.get(
            "base_frames", ["base_link", "base_link_lb", "base_footprint"]))
        self._log_every_n = max(1, int(self.params.get("log_every_n", 30)))
        self._dump = DumpWriter(self.params.get("dump_dir",
                                                "log/pallet_servo_dump"),
                                self.params.get("dump_on", ["reject", "warn"]),
                                _float_or(self.params.get(
                                    "dump_min_interval_sec"), DUMP_MIN_INTERVAL_SEC,
                                    "dump_min_interval_sec"))

        # ---- 可视化话题 ------------------------------------------------------
        # `overlay_out` 给空串 = 完全不发图（省 CPU、省带宽）。
        # 发图**降频**：伺服 10Hz，图 5Hz 就够看，而画图 + 编码不便宜。
        self._overlay_out = str(self.params.get(
            "overlay_out", "/pallet_servo/overlay")).strip()
        # ⚠️ 用 `_float_or`（不抛 + 点名 + 退默认），**不要用裸 `float()`**：那会
        # 让 `overlay_period_s: "abc"` 在 `__init__` 里抛出来、掀翻建树 —— 与本提交
        # 为 `agg`/`window`/`pair_cache`/`max_dt_s` 立的那条纪律正好相反。
        # 负数由 `_float_or` 钳成 0 = 不降频（每帧都发，比"少发"更容易被发现）。
        self._overlay_period_s = _float_or(
            self.params.get("overlay_period_s"), 0.2, "overlay_period_s")
        self._image_color_topic = str(self.params.get(
            "image_color", "/camera/color/image_raw"))
        # **只为画 YOLO 框**。缺了不影响出数 —— 只是图上少一层灰框。
        self._overlay_pub = None
        self._color_sub = None
        # ---- 检测门控（2026-10-18）------------------------------------------
        # 见 `initialise()` 里建发布器那一段。`_last_detect_mode` 用来"变了立刻发、
        # 没变每秒补一条"（补的那条防检测器重启后收不到状态）。
        self._detect_mode_topic = str(self.params.get(
            "detect_mode_topic", "/pallet_servo/detect_mode")).strip()
        self._detect_mode_pub = None
        self._detect_mode_msg = None
        self._last_detect_mode = None
        self._last_detect_mode_t = 0.0
        self._last_color = None            # 回调线程写、update() 读
        self._overlay_lock = threading.Lock()
        self._last_T_cam_pallet = None      # render 要的相机系托盘位姿
        self._last_overlay_t = 0.0
        self._n_overlay = 0
        self._n_overlay_skip = 0
        self._n_overlay_selfcheck_fail = 0
        self._last_overlay_warn = None
        # 上一次喊过的**可视化告警键**与当时是第几个 tick（周期提醒按它算，
        # 见 `_warn_overlay`）。去重键=稳定键，提醒口径=本文件既有的"每 N tick"。
        self._last_overlay_warn_tick = 0
        # `latest_pallet_stamp` 缺 / 非正那条告警的一次性槽位（M7：**必须**在
        # `initialise()` 里复位 —— 它从前是懒建属性，换场景/重树之后**不会再响**）
        self._stamp_warned = False

        # ---- 伺服误差话题（对外接口）-----------------------------------------
        # 它是**功能性输出**（下游控制器要用），不是诊断设施 —— 所以默认**开**。
        # 给空串 = 完全不发（连发布器都不建）。
        #
        # ⚠️ 与黑板的职责划分：黑板上的 `latest_servo_error` 是**树内**用的实时值，
        #    本话题是**对外**的接口。两者同源同帧，但**只有话题是承诺过的接口** ——
        #    黑板是 py_trees 的进程内单例，树外的进程根本看不见它。
        self._servo_error_out = str(self.params.get(
            "servo_error_out", "/pallet_servo/dis")).strip()
        # 无效帧的重发周期（秒）。`0` = 只在内容变化时发 —— 但那会让"停摆了"
        # 退化成"没有新消息"，与"话题名写错了"分不开。默认 1.0 保证消费者
        # **任何时刻都能在 1 秒内**收到一条说明当前状态的消息。
        # 负数由 `_float_or` 钳成 0（与 `overlay_period_s` 同一口径）。
        self._servo_error_heartbeat_s = _float_or(
            self.params.get("servo_error_heartbeat_s"), 1.0,
            "servo_error_heartbeat_s")
        self._servo_error_pub = None
        # 心跳节流的状态。
        # ⚠️ **这两个槽位不许在 `initialise()` 里重置**：py_trees 对"状态不是
        #    RUNNING"的行为**每 tick 重进 `initialise()`**，而配置错误那条路恒
        #    FAILURE —— 一重置就变成每秒几十条一样的话题消息。放 `__init__` 里
        #    （整个实例的生命周期只初始化一次）即可。
        self._last_error_pub_key = None
        self._last_error_pub_t = 0.0
        self._last_pair_stamp = 0.0
        # ⚠️ **"检测停了但还没到告警阈值"那段静默期**（2026-10-10 加）。
        # 检测一停，`PalletFrameWindow.resolve()` 就返回 `None`，而**"正常去重"
        # 与"某一侧停摆"共用这个返回值** —— 于是那一段我们**什么都不发**，下游
        # 拿到的还是最后一条 `valid=true` 的消息，就照它一直走。操作员原话：
        # "一旦托盘识别停止…就会出现一直以之前的误差值伺服的情况，导致机器人
        # 一直朝一个方向移动。"
        #
        # `_check_liveness` 用的是 `live_timeout_s`（board 默认 **5.0s**），
        # 那是**告警**阈值、也是发 invalid 的起点 —— 5 秒对控制器太长。
        # 这里另加一条**只判"多久没出数"**的阈值：超过它就发 `valid=false`，
        # 但**不**去动 `live_timeout_s`（那会让停摆告警变吵）。
        # 取 0.5s = 5 个图像周期（10Hz），正常去重（相邻 tick 同一对）不会触发。
        self._servo_error_stale_s = _float_or(
            self.params.get("servo_error_stale_s"), 0.5,
            "servo_error_stale_s")
        # 上一次**成功出数**（`valid=True`）的 monotonic 时刻。`None` = 还没出过数
        # （那种情况由配置错误/配对失败那几条路各自发，不归这条管）。
        self._last_ok_mono = None
        # 本 tick 该发的无效帧文案（`_paired_inputs` 在内部设）。
        # ⚠️ **必须在 `update()` 每 tick 开头重置成 `None`** —— 否则上一 tick 的
        #    拒绝文案会粘住，这一 tick 明明没拒绝却照发一条无效帧。
        self._pending_reject = None

        # `ref_edges` 的**整表**校验放在 `initialise()`（那里能一次报全）：
        # 只查写法与"两条不平行"；绝对写法**允许落在台面外**（参考边是操作员
        # 想对齐的位置，2026-10-26 操作员裁定）。
        self._config_err = None

        # ---------------------------------------------------------------- 内参
        # `K` / `D` / `image_size` 三样**以实时的相机话题为准，配置文件里的
        # 写死值只是兜底**。
        #
        # 三样都在 `/camera/color/camera_info` 里，而 `CameraAdapter` 本来就订着
        # 它（`camera_adapter.py:406`；`ICamera.get_camera_info()` 就是它的读口）。
        # 抓帧那条路一直是自动读的（`pick_servo_inputs.py:482` → `meta.json`
        # → `--capture-dir`），**只有本节点要人在场景 JSON 里手抄** —— 抄错一个
        # `cx` 会让所有像素量整体偏，而三个数照样算得出来、看不出来。
        #
        # **为什么是"话题优先"而不是"参数优先"**：写死的值会过期。换了分辨率、
        # 换了一台机器人、相机换了个型号，场景 JSON 里的 K 不会自己跟着变，而
        # 那是**静默**的错；反过来，相机没起来时话题拿不到，那时才轮到写死值，
        # 而"相机没起来"这件事本身是看得见的。
        #
        # 参数**仍然有用**：离线回放（没有相机）、单测、以及相机话题因为任何原因
        # 取不到时的显式兜底。优先级由 `initialise()` 里的 `_resolve_intrinsics()`
        # 落地，来源写进日志的"内参来源="，永远看得出这一版用的是哪个。
        #
        # **没有 `camera` 参数了**：读话题的那条路用写死的话题名
        # （`CAMERA_INFO_TOPIC`），因为 `camera` 那套命名是 `CameraAdapter` 的
        # 约定，而本节点不访问硬件、不该依赖它。收着一个用不到的参数只会误导人。
        # 形状错误仍然是**立即**的配置错误（与有没有相机无关，那是场景写错了）
        self._K_param, k_err = _matrix_or_none(self.params.get("K"), (3, 3), "K")
        self._D_param, d_err = _matrix_or_none(self.params.get("D"), None, "D")
        self._image_size_param, s_err = parse_pair_param(
            self.params.get("image_size"), "image_size")
        self._config_err = (self._config_err or k_err or d_err or s_err
                            or self._servo_slots_err)

        self._K = None
        self._D = None
        # 读一次相机话题的结果，**每实例一次**（见 `_camera_info_once`）：
        # `initialise()` 会被 py_trees 每 tick 重进，"读不到"这条路上不缓存
        # 就是每 tick 白等一次超时。
        self._camera_info_cache = None
        # 上一次**问相机**的时刻 + 重试间隔（`_camera_info_once` 用，见那个常量的
        # 注释：失败要能自愈，但每个 tick 都试会把 tick 拖成 0.5s）
        self._camera_info_retry_after = 0.0
        # 配置错误那条 ERROR 的去重槽位（`_log_config_error`）。**故意不按运行重置**：
        # 重置了就等于没去重 —— 而那正是它要治的病。同一条错误只在**文案变了**
        # （换了配置 / 换了阶段）时再落一条。
        self._last_config_err: Optional[str] = None
        # 本次运行用的是哪个内参：`_resolve_intrinsics` 填，只进日志 ——
        # 现场第一件要核对的就是它。
        self._intrinsics_err = ""
        self._intrinsics_src = "（还没解析）"
        self._intrinsics_from_camera = False
        T_param, t_err = _matrix_or_none(self.params.get("T_cam_base_param"),
                                         (4, 4), "T_cam_base_param")
        self._config_err = self._config_err or t_err
        # `image_size` 的坏形状也在这里就查掉（`_image_size_param` 那两行）：不查的
        # 话坏值会活到第一帧的 `on_execute` 里（`ref_edge_px` 取 image_size[1] 抛
        # IndexError），被技能层当异常接住 → **当成"这一帧被拒"**：每帧一条
        # 看着像几何退化的 WARNING，外带每帧一份 dump。配置错误就该在启动时说。
        self._image_size = self._image_size_param

        self._resolver = None
        self._T_cam_base_param = T_param
        self._skill = None
        self._versions_seen = None
        self._size = None
        # 上游报的尺寸坏掉时只喊一次；键是**错误的文本**，换个坏法再喊一次
        self._size_upstream_bad = None
        # 这一 tick 的尺寸是上游报的还是配置兜底的（HUD 上要写出来，见下）
        self._size_from_upstream = False
        self._frames = 0        # 真正算过的帧数（日志里用它编号）
        self._ticks = 0         # update() 被调了多少次（节流用它，包括没输入的帧）
        self._version_warned = set()   # 已就"版本键从未写过"喊过的键名
        self._dump_payload_fails = 0   # dump 取字段失败了几次（日志按它去重）

        # ⚠️ `latest_pallet_stamp` **必须在 READ 注册里**：配对要读它，而读黑板
        # 用的是 `read_blackboard()`（它接的是 `KeyError`）—— 没注册就是 `KeyError`
        # 一路抛穿 `update()`（addendum A8）。
        for key in (self._pallet_key, f"{self._pallet_key}_version",
                    f"{self._pallet_key}_stamp",
                    # 上游报的尺寸（`NodePalletObs` 写）—— 每 tick 在
                    # `_resolve_size()` 里读；漏了注册就是 KeyError 掀翻整棵树
                    self._size_key,
                    self._box_key, f"{self._box_key}_version",
                    # ★ 候选列表（`NodeBoxObs` 写）—— 邻居路线专用。与老键一样
                    #   必须进 READ 注册：`read_blackboard()` 只接 `KeyError`，
                    #   没注册就是 **AttributeError 一路抛穿 `update()`**，而
                    #   py_trees 不接那个异常 → 整棵树连每帧日志一起没。
                    self._candidates_key, f"{self._candidates_key}_version"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.READ)
        for key in (self._key, f"{self._key}_version"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE)
        # 先置初值，免得其他节点 getattr 时 KeyError（register_key 只注册权限，
        # 不创建值）—— 这是 NodePercep 踩过并写在注释里的坑
        self.global_blackboard.set(self._key, None)
        setattr(self.global_blackboard, f"{self._key}_version", 0)

    # ---------------------------------------------------------------- 启动
    @staticmethod
    def _degenerate_k_reason(K, where):
        """`K` 退化就说清原因，否则返回 `None`。`where` 只用在提示里（来自哪）。"""
        fx, fy = float(K[0, 0]), float(K[1, 1])
        if fx > 0 and fy > 0:
            return None
        return (f"{where} 退化 —— fx={fx:g} fy={fy:g}，像素焦距必须 > 0"
                f"（全 0 的占位符会一路静默算出错误的角度）")

    def _resolve_intrinsics(self) -> str:
        """定下本次运行的 K / D / image_size。**话题优先，写死值兜底。**

        返回空串表示成了；非空是失败原因（调用方据此报配置错误）。

        优先级：**实时的相机话题优先，读不到才用场景里写死的参数。** 写死的值
        **会过期** —— 换了分辨率、换台机器人、换个相机型号，场景 JSON 里的 `K`
        不会跟着变，而那是**静默**的错（三个量照算，只是全偏）；反过来"相机没起来"
        是看得见的。所以真机上别写 `K`，让它自己读。

        ## ⚠️ 测试必须**挡掉这次读话题**

        这也是第一版把 CI 拖挂的那条路：CI 上 rosmaster 是活的、相机在跑，
        所以"读话题"会**成功**，测试拿到的 K 就**取决于跑在哪台机器上** ——
        本机没相机、CI 有相机，同一份断言两边结果不同。**测试一律把
        `_read_camera_info()` 换掉**（见 `test_node_pallet_servo.py` 的
        `_no_camera` fixture），别让它碰真话题。
        """
        reason = ""
        if is_dry_run():
            reason = "dry-run 不碰硬件，没去问相机"
        elif self._K_param is not None:
            # **写了 K 就一个话题都不碰。** 两个理由：
            # ① 确定性 —— CI 上 rosmaster 是活的、相机在跑，若"给了 K 还去读
            #    话题"，同一个断言在本地（没相机）和 CI（有相机）结果就不同；
            # ② 显式即钉死 —— 写 K 的人是在说"就用这个"。
            # 真机上**别写 K**（让它自己读）；离线复现/测试**写 K**（不依赖机器）。
            reason = "场景里写了 K，按显式钉死处理，没去问相机"
        else:
            cam_K, cam_D, cam_size, reason = self._camera_info_once()
            if cam_K is not None:
                bad = self._degenerate_k_reason(cam_K, "相机报的 K")
                if bad:
                    # 相机报了个退化的 K（fx/fy <= 0）不是"没数据"，是相机那边有
                    # 问题。**不退回写死值**：那样会把一个真问题盖掉，而三个量照算。
                    return bad + "（相机还没标定？）"
                self._K, self._D = cam_K, cam_D
                self._image_size = (cam_size if cam_size is not None
                                    else self._image_size_param)
                self._intrinsics_src = CAMERA_INFO_TOPIC
                self._intrinsics_from_camera = True
                return ""

        # 话题没给（或压根没问）—— 退回场景参数
        self._K = self._K_param if self._K is None else self._K
        self._D = self._D_param if self._D is None else self._D
        self._image_size = (self._image_size_param if self._image_size is None
                            else self._image_size)
        if self._K is None:
            return (f"没有内参 —— 相机话题那边要不到（{reason or '（没去问）'}），"
                    f"场景里也没写 `K`。两条路选一条：① 把相机起起来（本节点会"
                    f"自己读 {CAMERA_INFO_TOPIC}）；② 在场景 JSON 里显式给 "
                    f"`K`（嵌套 3×3）。")
        bad = self._degenerate_k_reason(self._K, "场景里的 K")
        if bad:
            return bad
        self._intrinsics_src = f"场景参数（{reason}）"
        return ""

    def _camera_info_once(self):
        """`_read_camera_info()` 的**节流缓存**。返回 `(K, D, size, 原因)`。

        为什么必须缓存：`initialise()` 在 `update()` 返回非 RUNNING 时会被 py_trees
        **每 tick 重进**。没有缓存的话，"相机读不到"这条路上每个 tick 都要等满一次
        `CAMERA_INFO_TIMEOUT_SEC` —— 50 Hz 的 tick 配上 0.5 秒的等待，现场就是
        彻底卡死。缓存把代价钉在"每 `CAMERA_INFO_RETRY_SEC` 一次"。

        ⚠️ **失败要能重试，但必须节流**（这一轮的修正）：从前是"失败也缓存、
        每实例只读一次"，而"相机在树启动那一刻还没起来"是真机上很常见的一件事
        —— 一旦第一帧读不到，节点就**永久** FAILURE、每 tick 一条 ERROR、
        **不可能自愈**。现在：**成功过一次就永远用那一次**（同一版运行里 `K`
        不该变），**失败**则每 `CAMERA_INFO_RETRY_SEC` 再问一次。两者一起才
        既自愈又不卡死，理由见那个常量的注释。
        """
        cached = self._camera_info_cache
        if cached is not None and cached[0] is not None:
            return cached                          # 成功过：一次就够，不再问
        now = time.monotonic()
        if cached is None or now >= self._camera_info_retry_after:
            cached = self._read_camera_info()
            self._camera_info_cache = cached
            self._camera_info_retry_after = now + CAMERA_INFO_RETRY_SEC
        return cached

    def _read_camera_info(self):
        """**直接读话题**要 `(K, D, image_size)`；要不到返回 `(None, None, None, 原因)`。

        ## 为什么是 `rospy.wait_for_message` 而不是 `get_shared_hardware()`

        本节点的契约是「**不访问硬件**」（见模块 docstring）。第一版这里图省事走了
        `get_shared_hardware()` → `CameraAdapter.get_camera_info()`，后果是：

        * 那个函数会 `HardwareFactory.create_hardware()` + `initialize()`，里面
          **无条件 `rospy.init_node()`**（`lifecycle_mixin.py:34`），还会起相机
          `roslaunch`、连 SDK、起状态管理器 —— 为了读**一条消息**。
        * 在**没有 ROS master** 的机器上，`init_node()` 会打印
          `Unable to register with master node ... Will keep trying` 然后**无限重试**。
          CI runner 正是这种情况：它把整个 job 拖到 1 小时超时被强杀
          （`ERROR: Job failed: execution took longer than 1h0m0s`）。
        * 本机之所以没暴露：11311 端口没人监听时 `init_node` 会**当场抛**
          `ConnectionRefusedError`，被 `except` 接住、静默退回写死值 ——
          **环境差异把 bug 盖住了**。

        `wait_for_message` 不需要 master、不需要 `init_node`，实测无 master 时
        **精确按 timeout 返回**（0.1/0.5/2.0 秒各测得 0.103/0.508/2.006 秒）。
        仓库里也有先例：`boxcarry.py:202`、`basket_place_after_nav_move.py:114`。

        用**默认的 `camera_info` 话题名**，不走 `camera` 前缀 —— 那套
        `/camera/color/...` 的命名是 `CameraAdapter` 的约定，本节点不该依赖它。

        **绝不抛**：这里在 `initialise()` 里，py_trees 不接它抛的异常。
        """
        try:
            import rospy
            from sensor_msgs.msg import CameraInfo
        except Exception as exc:                     # noqa: BLE001
            return None, None, None, f"导不进 rospy/sensor_msgs（{exc}）"
        try:
            info = rospy.wait_for_message(CAMERA_INFO_TOPIC, CameraInfo,
                                          timeout=CAMERA_INFO_TIMEOUT_SEC)
        except Exception as exc:                     # noqa: BLE001
            # 没 master / 话题没人发 / 超时 —— 都是"没有数据"，不是异常
            return None, None, None, (
                f"读不到 {CAMERA_INFO_TOPIC}（{type(exc).__name__}: "
                f"{str(exc)[:80]}）")
        k = getattr(info, "K", None)
        if not k or len(k) < 9:
            # 只判"有没有"，**不判 fx/fy 是否为正** —— 那是
            # `_degenerate_k_reason` 的活，由它统一报"退化"（含 fx=0 这种
            # "相机没标定"的情况）。在这里先拦一道会让报错文案指不到点子上。
            return None, None, None, f"{CAMERA_INFO_TOPIC} 里的 K 是空的"

        K = np.array([[k[0], 0.0, k[2]],
                      [0.0, k[4], k[5]],
                      [0.0, 0.0, 1.0]], np.float64)
        D = (np.asarray(list(info.D), np.float64)
             if getattr(info, "D", None) else None)
        size = ([int(info.width), int(info.height)]
                if getattr(info, "width", 0) and getattr(info, "height", 0)
                else None)
        return K, D, size, ""

    def initialise(self):
        self._skill = None
        self._versions_seen = None
        self._frames = 0
        self._version_warned = set()     # "版本键从未写过"的一次性警告，按运行重整
        self._dump_payload_fails = 0     # dump 取字段失败的计数，也按运行重置

        # ★ **时钟回跳的唯一出路**（addendum A4，`PalletFrameWindow.reset()` 的
        # docstring 把来龙去脉写全了）：`_last_key` 是"已经消费掉的观测"的水位线，
        # bag 循环播放 / 换时间源 / 重进 `initialise()` 之后新观测的 stamp 全都
        # **小于**它，`resolve()` 于是**永久返回 `None`** —— 伺服黑板停在旧值上，
        # 一个错都不报。
        #
        # ⚠️ **水位线与两路"上次推过的版本号"必须一起清**：只清窗、不清版本号的话
        # 版本没变就一个观测都推不进去 —— 那还是永久静默，只是换了个地方。
        self._pairing.reset()
        self._last_pallet_version = -1
        self._last_box_version = -1
        self._last_neighbor_key = None
        # 存活闸的两个时钟也按运行重整（否则重进之后立刻误报一次"停摆"）
        _mono = time.monotonic()
        self._last_pallet_seen = _mono
        self._last_box_seen = _mono
        self._last_live_warn = None
        # ★ M7：`_stamp_warned` 必须**并进这份复位清单**。它从前是懒建的
        #   （`_warn_missing_stamp_once` 里 `getattr(self, "_stamp_warned", False)`
        #   然后置 True），于是**只在本实例上响一次** —— 换场景 / 重树 / 重进
        #   `initialise()` 之后，"上游没填 `header.stamp`"这条**再也不会响**，
        #   而它正是"配上一对不同帧的图像"那类静默错的第一线索。
        #   与上面那几个"上次见过什么"的槽位同一处、同一口径。
        self._stamp_warned = False
        # 可视化那条周期提醒的计时也按运行重整：重进之后不该立刻补一条提醒
        self._last_overlay_warn_tick = self._ticks

        if self._config_err:
            # 设计文档 §6.2：配置错误是 ERROR，自动进 error 日志文件。
            # ⚠️ **去重**（`_log_config_error`）：py_trees 对"状态不是 RUNNING"的
            # 行为**每 tick 重进 `initialise()`**，而这条路恒 FAILURE —— 不去重
            # 就是每秒几十条一模一样的 ERROR。
            self._log_config_error(self._config_err)
            self.feedback_message = f"配置错误 —— {self._config_err}"
            return

        # 内参：**话题优先，写死值兜底**。形状类的错误已经在 `__init__` 里拦过了
        # （那跟相机在不在无关，是场景写错了），这里只剩"值退化了"和"两边都没有"。
        #
        # ⚠️ **这里**不把 `_intrinsics_err` 折进 `self._config_err`（从前折了）：
        #    折进去就等于把它永久钉死 —— "相机在树启动那一刻还没起来"是**可以**
        #    自愈的一件事（相机起来后 `_camera_info_once` 会重试，见那里的说明），
        #    而 `_config_err` 一旦写上就再也不会被清掉，自愈的路就被自己堵死了。
        #    判据同时看两者，见 `update()`。
        self._intrinsics_err = self._resolve_intrinsics()
        if self._intrinsics_err:
            self._log_config_error(self._intrinsics_err)
            self.feedback_message = f"配置错误 —— {self._intrinsics_err}"
            return

        self._size = self._resolve_size()
        if self._size is None:
            # 2026-10-26 操作员裁定：主流程的 `place_servo_pallet_size_mm` 已删，
            # 尺寸只从**检测器上报**的 `latest_pallet_size_mm` 来。启动时还没有
            # 检测结果，尺寸为 None 是**正常状态**，不能当配置错误把节点卡死：
            #   * `pallet` 档：`update()` 里等上游尺寸，等到之后照常算；
            #   * `neighbor` 档：本来就不用尺寸，照常出数。
            logger.info("台面尺寸暂未取到（检测器尚未上报 %s）；"
                        "pallet 档会等它，neighbor 档不受影响", self._size_key)

        # ★ 参考边**整表**校验：只查写法与"两条不平行"（绝对写法允许落在
        #   台面外 —— 参考边是操作员想对齐的位置，不是物理托盘约束）。
        #   **不留到轮到它才发现** —— 那时箱子已经在托盘上了，前一个白搬。
        #   `_config_err` 按本文件既有口径**不带前缀**（`k_err` / `d_err` /
        #   `s_err` 都不带），前缀由下面那个唯一的报出点统一加。
        if self._config_err is None:
            for index, slot in enumerate(self._slots, start=1):
                parsed_slot = parse_ref_edges(slot)
                if isinstance(parsed_slot, Reject):
                    self._config_err = (f"ref_edges 第 {index} 组 {slot!r} "
                                        f"不合法：{parsed_slot}")
                    break
        if self._config_err:
            # ⚠️ **同一次 `initialise()` 里就报出去**（与开头那段同一套：ERROR +
            #    去重 + feedback + 不往下走）。只写 `_config_err` 而继续跑到函数
            #    末尾的话，末尾那句"就绪…"会把 `feedback_message` 覆盖掉 ——
            #    现场看到的是"就绪"，而 `update()` 又因 `_config_err` 恒 FAILURE，
            #    两边对不上（既有用例 `test_node_startup_rejections_are_failures`
            #    正是钉这一条的）。开头那段是**下一次** `initialise()` 的入口，
            #    两条路报的是同一句话（`_config_err` 没变 → 去重命中）。
            self._log_config_error(self._config_err)
            self.feedback_message = f"配置错误 —— {self._config_err}"
            return

        # ★ **档位总数 = `ref_edges` 与 `servo_slots` 的较大者**，而**每一档
        #   都要说得清用什么基准**（2026-10-17 修，起因是真机实测）。
        #
        #   上一版这里是"`servo_slots` 更长就把多出来的**截掉** + 一条 WARNING"。
        #   那是个**功能死角**：`_activate_slot` 与 `self._slots[...]` 两处都按
        #   `len(ref_edges)` 算档位总数，于是 `ref_edges` 只有 4 组时
        #   `rosservice call ... "slot: 5"` **永远切不过去**（真机报"表里只有 4
        #   组"），而板子上第 5 档明明写着 `neighbor`。截断还让"本该走邻居的那几
        #   档悄悄走了托盘路线"，托盘被挡死时**一声不响**。
        #
        #   正确的判据**不是"两边必须一样长"**，而是分开看每一档：
        #     * **`pallet` 档必须有 `ref_edges`** —— 托盘路线的基准就是它，
        #       缺了就只能报错（那种帧静默算得出三个数，最难查）；
        #     * **`neighbor` 档不需要 `ref_edges`** —— 它的基准是别的箱子的边。
        #       所以 `servo_slots` 比 `ref_edges` 长时，**只要多出来那几档都是
        #       neighbor 就合法**（4 组 ref_edges + 5 档表正属此列）。
        #     * `servo_slots` **短**是合法的（保持原设计）：后面几档没写就按
        #       `pallet` 兜底，而那些档有 `ref_edges`（上面第一条保证）。
        #
        #   ⚠️ **`ref_edges` 不是"只有 1、2 箱才用"**：邻居档虽然不读它，但它是
        #   **档位总数的定义源**（服务与叠加图都按它数）。这就是"为什么 5 箱也会
        #   出问题"—— 档位数被它卡在 4。
        total_slots = self._total_slots
        for index in range(1, total_slots + 1):
            mode, _side = _slot_mode_of(self._slot_modes, index - 1)
            if mode == "pallet" and index > len(self._slots):
                self._config_err = (
                    f"servo_slots 第 {index} 档是 pallet，但 ref_edges 只有 "
                    f"{len(self._slots)} 组 —— **pallet 档必须有参考边**。"
                    f"补一组 ref_edges（可以照抄上一组占位），"
                    f"或者把这一档改成 neighbor")
                break
        if self._config_err:
            self._log_config_error(self._config_err)
            self.feedback_message = f"配置错误 —— {self._config_err}"
            return

        # 档位偏置在启动时说清楚：它不改变三个数的算法，只改**输出**，现场
        # "配了没生效"只能从机械臂停的位置倒推 —— 先让日志给出这个数。
        bias_notes = []
        for index in range(1, total_slots + 1):
            right_bias = _slot_bias_of(self._slot_modes, index - 1, "right")
            bottom_bias = _slot_bias_of(self._slot_modes, index - 1, "bottom")
            parts = []
            if right_bias:
                parts.append(f"e_right {right_bias:+g}px")
            if bottom_bias:
                parts.append(f"e_bottom {bottom_bias:+g}px")
            if parts:
                bias_notes.append(f"第 {index} 档 " + "/".join(parts))
        if bias_notes:
            logger.info("档位输出偏置（new = old + bias）：%s",
                        "，".join(bias_notes))

        # ⚠️ **TF 监听必须在尺寸检查之后才起。** py_trees 的 `Behaviour.tick()`
        # 对"状态不是 RUNNING"的行为**每 tick 重进 `initialise()`
        # （`py_trees/behaviour.py:307-308`）**，而本节点在"没有台面尺寸"这条路上
        # 返回 FAILURE —— 构造留在检查之前，一台装了 ROS 的机器上就是**每秒 50 次
        # 构造/析构 `/tf` 与 `/tf_static` 的订阅**（没装 ROS 时是 100 行/秒的
        # WARNING + 每帧重读一遍 YAML）。而 `pallet_size_mm` 是新增的键，**现存的
        # `config/pallet_tag.yaml` 一份都没有它** → 第一次接真机走的就是这条路。
        # 配置问题该是启动时**一条**清楚的 ERROR，不是一场资源风暴。
        #
        # `is None` 那半句是第二道闸：真被重进也只起一个监听（监听器本身不便宜，
        # 而且重建会丢掉 TF 缓存与降级计数）。
        # `pallet_frame == "camera"` 时**根本不构造**解析器：没有 TF 查询、
        # 没有缓存、没有静默回退，少一个失败源。
        if (self._pallet_frame == "base_link" and self._resolver is None
                and not is_dry_run()):
            self._resolver = TfCamBaseResolver(
                _make_tf_lookup(), self._camera_frame, self._base_frames,
                param=self._T_cam_base_param)

        # ---- 可视化：订阅底图 + 建发布器 ------------------------------------
        # **订阅走回调存最新帧，`update()` 里取** —— 绝不在 ROS 回调线程里画图
        # （画图 + 编码几十毫秒，会拖垮回调队列，进而让伺服拿到的图像变旧）。
        if self._overlay_out and not is_dry_run() and self._overlay_pub is None:
            try:
                import rospy
                from sensor_msgs.msg import Image

                self._overlay_pub = rospy.Publisher(
                    self._overlay_out, Image, queue_size=1)
                self._color_sub = rospy.Subscriber(
                    self._image_color_topic, Image, self._on_color,
                    queue_size=1, buff_size=2 ** 24, tcp_nodelay=True)
                logger.info("NodePalletServo 可视化 → %s（底图 %s，每 %.2fs 一帧）",
                            self._overlay_out, self._image_color_topic,
                            self._overlay_period_s)
            except Exception as exc:                     # noqa: BLE001
                # **绝不抛**：可视化是诊断设施，不该有能力把节点弄死。
                logger.warning("NodePalletServo 可视化起不来（%s）—— 不出图，"
                               "伺服结果不受影响", exc)
                self._overlay_pub = None

        # ---- 伺服误差话题：建发布器 ------------------------------------------
        # ⚠️ **整段包 try**：发布是输出设施，它不该有能力把节点弄死
        # （与可视化同一条纪律）。起不来就只写黑板，并**说清后果**。
        if (self._servo_error_out and not is_dry_run()
                and self._servo_error_pub is None):
            try:
                import rospy

                # 没有 `init_node()` 就没有 ROS（离线跑 / 单测）：**不建、不吵**。
                # 不判这一句的话，真正的"`pallet_servo_msgs` 没编译"会淹进
                # "话题起不来"这一类噪声里（与 `live_timeout_s` 那条 F1 同一个病）。
                if _ros_node_initialized(rospy):
                    from pallet_servo_msgs.msg import PalletServoError

                    self._servo_error_pub = rospy.Publisher(
                        self._servo_error_out, PalletServoError, queue_size=1)
                    logger.info("NodePalletServo 伺服误差话题 → %s（心跳 %.2fs）",
                                self._servo_error_out,
                                self._servo_error_heartbeat_s)
            except Exception as exc:                     # noqa: BLE001
                logger.warning(
                    "NodePalletServo 伺服误差话题起不来（%s）—— **只写黑板，"
                    "行为树之外的控制器拿不到误差**。多半是 pallet_servo_msgs "
                    "没编译（catkin_make --pkg pallet_servo_msgs）",
                    exc)
                self._servo_error_pub = None

        # ---- 检测门控：告诉两个检测器"现在要不要出数" -----------------------
        #
        # 2026-10-18 加。现场流程是"先抓箱、到位后才 call slot"，而码放之前
        # **不一定看得见托盘和纸箱** —— 检测器一直跑只会白烧 GPU、把日志灌满
        # 拒绝帧，启动脚本也没法"等到有数据"。
        #
        # 语义（`std_msgs/Int32`，**latched**：后启动的检测器也能立刻拿到当前值）:
        #   0 = 两个都停（未激活）
        #   1 = 只要纸箱（neighbor 档）
        #   2 = 托盘 + 纸箱（pallet 档）
        # 检测器那边的门控与预热见各自的节点（`~gated`）。
        if self._detect_mode_pub is None and not is_dry_run():
            # ⚠️ "没有 rospy / 没有 init_node" 是**离线或单测**，不是故障 ——
            #    按 DEBUG 处理，不占 WARNING（否则每次离线跑都多一条假告警，
            #    把真正的"消息包没编译"淹掉：与 `live_timeout_s` 那条 F1 同一个病）。
            try:
                import rospy
            except Exception:                            # noqa: BLE001
                rospy = None
            if rospy is None or not _ros_node_initialized(rospy):
                logger.debug("NodePalletServo 检测门控：没有 rospy / 没 init_node，"
                             "不建发布器（离线或单测）")
            else:
                try:
                    from std_msgs.msg import Int32

                    self._detect_mode_pub = rospy.Publisher(
                        self._detect_mode_topic, Int32, queue_size=1,
                        latch=True)
                    self._detect_mode_msg = Int32
                    logger.info("NodePalletServo 检测门控 → %s"
                                "（0=全停 / 1=纸箱 / 2=托盘+纸箱）",
                                self._detect_mode_topic)
                except Exception as exc:                 # noqa: BLE001
                    # 门控起不来 = 检测器按 `~gated=false` 的默认值一直算
                    # （fail-open）。不影响伺服正确性，只是白烧 GPU。
                    logger.warning(
                        "NodePalletServo 检测门控话题起不来（%s）—— 检测器会一直跑"
                        "（功能正常，只是白烧 GPU、日志更吵）", exc)
                    self._detect_mode_pub = None

        # ---- 开关服务：切箱位 / 停止 ----------------------------------------
        # 三重纪律与发布器那段一致：`_ros_node_initialized` 守卫（没有
        # `init_node()` 就没有 ROS，离线跑 / 单测**不建、不吵**）+ 整段包 try
        # + **绝不抛**（服务是接口设施，不该有能力把节点弄死）。
        #
        # ⚠️ 服务起不来的后果是**永远停在未激活**（不出有效数），不是崩溃。
        #    日志必须说清这一点，否则现场看到"话题上全是 valid=false"会去
        #    查检测器，而真正的原因是这个包没编译。
        if (self._slot_srv is None and not is_dry_run()):
            try:
                import rospy

                if _ros_node_initialized(rospy):
                    from pallet_servo_msgs.srv import (SetServoSlot,
                                                       SetServoSlotResponse)

                    def _handle_set_slot(req):
                        ok, message = self._activate_slot(int(req.slot))
                        response = SetServoSlotResponse()
                        response.ok = ok
                        response.message = message
                        return response

                    self._slot_srv = rospy.Service(
                        "/pallet_servo/slot", SetServoSlot, _handle_set_slot)
                    # ⚠️ **档位数说 `_total_slots`，不说 `len(self._slots)`**
                    #    （2026-10-17 改）。两个数不等时（`servo_slots` 比
                    #    `ref_edges` 长、多出来的是 neighbor），这条 INFO 从前
                    #    写的是参考边组数 —— 现场照着它判断"能 call 到几档"，
                    #    于是以为第 5 档不存在。参考边组数单独写出来仍有用
                    #    （它决定前几档的基准），所以两个数**都**报。
                    logger.info("NodePalletServo 箱位服务 → /pallet_servo/slot"
                                "（%d 档，其中 %d 档有参考边；"
                                "启动时未激活 —— 要 call 一次）",
                                self._total_slots, len(self._slots))
            except Exception as exc:                     # noqa: BLE001
                logger.warning(
                    "NodePalletServo 箱位服务起不来（%s）—— **节点会一直停在"
                    "未激活，话题上全是 valid=false**。多半是 pallet_servo_msgs "
                    "没编译（catkin_make --pkg pallet_servo_msgs）", exc)
                self._slot_srv = None

        # §6.2「初始化 INFO」：把决定口径的东西全打出来。现场出问题时，
        # 第一件要核对的就是这里的 K/ref_edges 有没有和离线点点时用的一致。
        logger.info(
            "NodePalletServo 初始化：keys=(%s, %s) -> %s 档位=%d档(参考边=%d组) "
            "第1组=%s "
            "pallet_size_mm=%s use_distortion=%s K=%s D=%s image_size=%s "
            "内参来源=%s camera_frame=%s base_frames=%s dump=%s "
            "pallet_frame=%s 配对=%s 存活闸=%ss overlay=%s 伺服误差=%s(心跳 %ss)",
            self._pallet_key, self._box_key, self._key,
            self._total_slots, len(self._slots), self._slots[0],
            self._size, self._use_distortion, np.round(self._K, 3).tolist(),
            None if self._D is None else np.round(self._D, 6).tolist(),
            self._image_size, self._intrinsics_src,
            self._camera_frame, self._base_frames,
            self._dump.enabled,
            self._pallet_frame,
            ("绕开（base_link 老路径）" if self._pallet_frame == "base_link"
             else f"max_dt_s={self._max_dt_s:g}s window={self._window} "
                  f"pair_cache={self._pair_cache} agg={self._agg} "
                  f"stale_s={self._stale_s:g}"),
            self._live_timeout_s,
            self._overlay_out or "不订",
            self._servo_error_out or "不发",
            f"{self._servo_error_heartbeat_s:g}")
        # ★ 报的是**档位数**（不只参考边组数）：现场靠这句话判断"能 call 到几
        #   档"，而两者不等时（5 档表 + 4 组参考边）从前那句话报的是 4。
        self.feedback_message = (f"就绪：{self._total_slots} 档"
                                 f"（参考边 {len(self._slots)} 组）"
                                 f" 尺寸={self._size}")

    def _log_config_error(self, message: str) -> None:
        """配置错误的 ERROR **去重** —— 同一句在一次进程里只落一条。

        为什么必须去重：py_trees 的 `Behaviour.tick()` 对"状态不是 RUNNING"的
        行为**每 tick 重进 `initialise()`**，而配置错误这条路**恒返回 FAILURE**
        —— 不去重就是**每 tick 一条 ERROR**（10 Hz ≈ 600 条/分钟），几百条一模
        一样的行把现场日志冲掉，而它要传达的只有第一行。

        **去重键取整条文案**（不是"有没有错过"这种布尔）：文案里嵌着实际值
        （尺寸、内参来源、缺的是哪个键），换了配置文案自己就变，新错误照样落盘。
        槽位**故意不按运行重置**（重置了就等于没去重 —— 那正是它要治的病）：
        它与 `_warn_*` 那几个"每运行一次"的槽位不是一回事。

        与 `_check_liveness` / `_VALID_FALSE_REMIND_TICKS` 的关系：那两处的键是
        **分类**（因为文案里带着每 tick 都会变的量：停摆时长、拒绝分数），这里
        的文案是**稳定**的，所以整条文案就是合格的键，不必再抽分类。
        """
        if message == self._last_config_err:
            return
        self._last_config_err = message
        logger.error("NodePalletServo 配置错误：%s", message)

    def _resolve_size(self):
        """台面尺寸：**上游报告优先**，其次 `pallet_size_mm` 参数，最后标定产物。

        ⚠️ **顺序 2026-09-30 改了，上游排到了第一位。** 原先是"参数优先"，
        理由是"伺服的投影基准是配置值"。现场推翻了它：`long_side_parallel`
        决定检测器**把哪条边配成 W/H**，而配置里那份是操作员写的**固定次序**
        —— `long_side_parallel=false` 时检测器报 `[1000,1200]`、配置是
        `[1200,1000]`，**两者的 `max/min` 相同，所以检测器自己没错**；但下游
        拿配置那份去 `project_pallet_points`，`(W,0,0)` 会沿 E1（短边方向）
        走 1200mm —— **比台面多出 200mm**，叠加图上就是"青框比托盘大一圈、
        方向还拧着"。它**不报错**，只能靠人眼在叠加图上看出来。

        检测器自己报的 `size_mm` 与它的 `T_cam_pallet`（`E1`/`E2`）是
        **同一帧、同一套几何**给出的，两者必然自洽 —— 那才是正确的基准。

        ⚠️ **配置那份不能删**：上游没给（检测器还没出结果 / `size_mm` 取不出数）
        时全靠它，而且它是**操作员意图**的唯一记录（`[长,短]`）。它只是不再
        决定"哪条边是 W"。
        """
        self._size_from_upstream = False
        # ① 上游这一帧报的（与它的 E1/E2 同源自洽）
        reported = read_blackboard(self.global_blackboard, self._size_key)
        if reported:
            size, err = parse_pair_param(reported, self._size_key)
            if err:
                # 上游写了个坏的 —— **不静默退回配置**，喊一次并说明退到了哪
                if self._size_upstream_bad != str(err):
                    self._size_upstream_bad = str(err)
                    logger.warning(
                        "%s 上游报的尺寸读不出来：%s（实际 %r）—— "
                        "**退回配置的 pallet_size_mm=%s**",
                        self._size_key, err, reported, self._size_param)
            else:
                self._size_from_upstream = True
                return size
        # ② 配置参数
        value = self._size_param
        if not value:
            try:
                with open(self._config_path, "r", encoding="utf-8") as stream:
                    value = (yaml.safe_load(stream) or {}).get("pallet_size_mm")
            except OSError:
                value = None
        if not value:
            return None
        size, err = parse_pair_param(value, "pallet_size_mm")
        if err:
            logger.error("台面尺寸读不出来：%s（实际 %r）", err, value)
            return None
        return size

    # ---------------------------------------------------------------- 每帧
    def update(self):
        if is_dry_run():
            return Status.SUCCESS
        # 上一 tick 的无效帧文案不许粘到这一 tick（见 `__init__` 里那个槽位的注释）
        self._pending_reject = None
        # 本 tick 的单调时钟。停发判定（`servo_error_stale_s`）要用它 ——
        # 与 stamp 的时钟无关（stamp 可能是仿真/回放时间，拿它算"多久没出数"会胡说）。
        self._mono = time.monotonic()
        # 本 tick 的槽位快照。服务回调跑在 ROS 线程、随时可能改 `_active_slot`，
        # 所以**只在函数开头读一次**；本函数里"这一帧用哪组参考边"的地方全用
        # 这个局部量。否则同一帧可能配对窗按旧基准复位、参考边按新基准算，出
        # 来的三个数看着正常、基准却是混的（下一 tick 才自愈）。
        slot = self._active_slot

        # ★ 配置错误**排在未激活之前**：配置错误是**静态的、永不自愈**（配置
        #   不会自己变好），而"未激活"是一种**正常的运行状态**（在等操作员 call
        #   服务）。顺序反了的话，配置错时现场只看得到"伺服未启动"，照做 call
        #   完服务才发现是配置错 —— 陷入"反复 call 却什么都没变"。
        #
        # ⚠️ **尺寸为 None 不再算配置错误**（2026-10-26）：主流程的
        #    `place_servo_pallet_size_mm` 已删，尺寸只从检测器上报的
        #    `latest_pallet_size_mm` 来。启动时/托盘未检出时它本来就是 None，
        #    pallet 档在下面等它，neighbor 档根本不用它。
        if self._config_err or self._intrinsics_err:
            # `initialise()` 已经把详细文案写进 `feedback_message` 了。
            # 发布器的**心跳承诺不能断**，所以这一支也要发无效帧。
            # ⚠️ **这一支不出图**：配置错误是**静态的启动问题**，`initialise()`
            #    里已经按 `_log_config_error` 报了（那条 ERROR 比图上多一行字
            #    清楚得多）；而这条路的 `_ticks` 不增（它在 `update()` 靠后），
            #    出图会把 `_log_overlay_counts` 的节流分母钉死在 0 上 → 每 tick
            #    一条 INFO，反而把现场淹掉。
            self._publish_invalid(self.feedback_message or "配置错误")
            return Status.FAILURE

        # ★ 尺寸**每 tick 取一次**：上游报的优先（`NodePalletObs` 写在
        #   `{pallet_key}_size_mm`），配置那份退成兜底（见 `_resolve_size`）。
        #
        # 为什么必须每 tick 取、不能像别的那样在 `initialise()` 里定：
        # 上游是**逐帧**上报的，而黑板那个键在检测器还没出结果时是 `None`。
        # 在启动时取会把 `None` 固化下来 —— 那就等于"永远用配置"，白改。
        self._size = self._resolve_size()

        # ★ **检测门控**（2026-10-18）：按当前档位告诉两个检测器要不要出数。
        #   必须放在**未激活早退之前** —— 否则 `slot: 0` 之后"全停"那条永远
        #   发不出去，检测器会一直跑（白烧 GPU、日志灌满拒绝帧）。
        _slot_mode_now, _ = (
            _slot_mode_of(self._slot_modes, slot - 1) if slot > 0
            else ("pallet", "none"))
        self._publish_detect_mode(slot, _slot_mode_now)

        # ★ 未激活：**什么都不算**（配对、投影全跳过），但**照发**一条无效帧。
        #   不发的话下游分不清「伺服结束了」与「行为树挂了」—— 而这两种的
        #   处置完全不同。早退点在 `_check_liveness` **之前**：没在算，就谈不上
        #   "检测器停摆"。
        #
        # ⚠️ `_ticks` 在这里也要 +1：它是节流的分母（`_ticks % log_every_n`），
        #    不加的话未激活期间它恒为 0，那条节流日志会**每 tick 一条**。
        if slot <= 0:
            self._ticks += 1
            self.feedback_message = "伺服未启动（call /pallet_servo/slot N）"
            self._log_throttled("未激活：%s", self.feedback_message)
            self._publish_invalid(self.feedback_message)
            # ★ 未激活也出图（诊断设施就该在"什么都没动"时也能看）：
            #   `slot=0` 时 `ref_edges=None`，于是不画参考边，但托盘台面、
            #   箱子四角该有还是有 —— 正是"到底哪一路没出东西"要看的。
            self._maybe_publish_overlay(
                None, slot,
                pose=read_blackboard(self.global_blackboard, self._pallet_key),
                box=read_blackboard(self.global_blackboard, self._box_key),
                reason=self.feedback_message)
            return Status.RUNNING

        # ★ 槽位换了 → 把配对窗的水位线清掉重算。
        #
        # 为什么必须清（而**不是**指望"下一对新观测自然到"）：`PalletFrameWindow`
        # 的 `_last_key` 是"已经消费掉的那一对"的水位线，切换基准时**同一批
        # 观测**还没被消费过也没有更新的观测可推 → `resolve()` 一路返回 `None`
        # （"没有新的配对可报"），话题上继续是**上一个箱位**的基准，**一个错都
        # 不报**。`reset()` 正是为"换场景/换基准"准备的（见它的 docstring）。
        #
        # 清在 `update()` 里而不是服务回调里：回调跑在 ROS 线程，只做赋值
        # （§5.6 的纪律）；而版本号 +1 是原子的，这里比较一下就够。
        # ⚠️ 版本号**不在这里清零** —— 那个槽位跨线程，节点层不做跨线程写。
        if self._slot_version != self._slot_seen:
            self._slot_seen = self._slot_version
            self._pairing.reset()
            self._last_pallet_version = -1
            self._last_box_version = -1
            # ★ 档位换了：邻居路线的输入去重键也要清 —— 不清的话新档位的第一帧
            #   会因为"版本号没变"被当成重复帧跳过（白等一拍）。
            self._last_neighbor_key = None
            self._versions_seen = None

        self._ticks += 1
        # 走 `read_blackboard()` 而不是 `getattr(..., None)`：后者接不住 py_trees
        # 在"键从未被写过"时抛的 `KeyError`，会把一个正常的"等输入"变成异常
        pose = read_blackboard(self.global_blackboard, self._pallet_key)
        pose_version = self._version_of(self._pallet_key, pose)
        box = read_blackboard(self.global_blackboard, self._box_key)
        box_version = self._version_of(self._box_key, box)
        # 邻居路线还要看**候选列表**的版本：被选中的那个箱子可能没动，而邻居动了
        # （`node_box_obs` 已经把它们一起放进自己的去重键里）。只在邻居档用得上，
        # 每 tick 多读一个键，代价可以忽略。
        # ⚠️ 把**值**一起传给 `_version_of`：生产者"写了候选却没写 _version"时
        #    要靠它喊一声 —— 否则版本恒为 0，邻居路线第一次之后**永远不再重算**
        #    （正是那个"静默过期"的老病）。
        cands_bb = read_blackboard(self.global_blackboard, self._candidates_key,
                                   None)
        cand_version = self._version_of(self._candidates_key, cands_bb)

        # ★ **这一帧走哪条路线**（2026-10-17 加）。按 `servo_slots` 查表，
        #   `slot` 是函数开头拍下的快照（服务线程随时可能改 `_active_slot`）。
        #   ⚠️ **`slot == 0`（未激活）时给 `("pallet", "none")`**：那一条出口在上面
        #   已经返回了，这里给什么都不影响行为 —— 但**不能**让它越界（`_slot_mode_of`
        #   本来就有兜底，这里显式写出来是为了读代码的人一眼看见"未激活与路线无关"）。
        slot_mode, slot_side = (
            _slot_mode_of(self._slot_modes, slot - 1) if slot > 0
            else ("pallet", "none"))

        if _servo_needs_inputs(slot_mode, pose, box):
            # **这不是异常**，是正常状态：上游还没给出东西（设计文档 §6.2）。
            # ⚠️ **这一道闸按路线区分**（见 `_servo_needs_inputs`）：
            #     * `pallet` 档：托盘位姿与箱子观测**都**要有 —— 与改动前逐字一致；
            #     * `neighbor` 档：**只要求箱子观测**。邻居路线本来就不用托盘 ——
            #       它存在的理由恰恰是"托盘被手里这个箱子挡死、`latest_pallet`
            #       是 `None`"。照老判据早退的话，邻居档会**永远**停在这句
            #       "等输入"上，连一次 `neighbor_servo` 都走不到（现场表现是
            #       "call 了服务、话题上一直是等输入"，而那看起来完全像个正常的
            #       上游没出数）。
            self.feedback_message = (
                _servo_wait_message(slot_mode, pose, box))
            self._log_throttled("等输入：%s", self.feedback_message)
            # 也要发 —— "链路通了但还没数据"与"话题断了"必须在消息层面分得开
            self._publish_invalid(self.feedback_message)
            # ★ 只有一路有结果时**照样出图**（这正是要看的帧）：四角/台面照画，
            #   缺的那一路不画，HUD 写清缺的是哪个。从前这一支什么都不画，
            #   现场于是分不出"纸箱检测没出东西"与"叠加图这条链坏了"。
            self._maybe_publish_overlay(None, slot, pose=pose, box=box,
                                        reason=self.feedback_message)
            return Status.RUNNING

        # ★ pallet 档需要台面尺寸来投影参考边；尺寸现在只来自检测器上报的
        #   `latest_pallet_size_mm`（主流程的配置键已删，2026-10-26）。
        #   **neighbor 档不用尺寸**，不能因为它一直等下去。
        if slot_mode != "neighbor" and self._size is None:
            self.feedback_message = (
                f"等上游托盘尺寸（{self._size_key} 还没有值；"
                f"pallet 档需要它投影参考边）")
            self._log_throttled("等尺寸：%s", self.feedback_message)
            self._publish_invalid(self.feedback_message)
            self._maybe_publish_overlay(None, slot, pose=pose, box=box,
                                        reason=self.feedback_message)
            return Status.RUNNING

        # ===================================================================== #
        # ★ 分流点：邻居档**不走** PalletServoSkill，也不走配对窗
        #   （2026-10-17 加；`neighbor_merge_spec.md` §5 / §8.2(3)）。
        #
        # 为什么整段**排在配对之前**：分两个理由，缺哪一条都走不到邻居路线。
        #
        #   ① 配对窗的输入是**托盘位姿与箱子观测**，而它存在的理由是"**两个视觉
        #      检测器**耗时差一个量级、必须按同一帧图像配起来"。邻居路线不用托盘
        #      位姿（托盘被挡死正是它的场景），而托盘侧没有位姿时 `_paired_inputs`
        #      会一路报 `pair_none` 并**每帧发一条无效帧** —— 邻居档一个数都出不来。
        #   ② 配对还会**就地覆盖 `pose` / `box`**（换成窗平均的那一对）。邻居路线
        #      要的是**这一帧原始**的候选与手里箱，被换掉之后算的就是另一帧的几何。
        #
        # ⚠️ **`pallet` 档一个字节都不许变**：下面那一整段（版本门禁 /
        #    `_paired_inputs` / `PalletServoSkill`）与改动前逐字相同。
        # ===================================================================== #
        if slot_mode == "neighbor":
            return self._run_neighbor_frame(slot, slot_side, pose, box,
                                            box_version=box_version,
                                            cand_version=cand_version)

        seen = None
        if self._pallet_frame == "base_link":
            # ---- 老路径：**绕开配对**，逐位复刻改动前的版本门禁直算 ------------
            # addendum A10（有意偏离 brief Step 10 的"两种模式都走配对"）：
            # 配对存在的理由是「**两个视觉检测器**的耗时差一个量级」；`base_link`
            # 那条是 Apriltag 老路径，它的生产者（`NodePalletPose`）**全文没有
            # `stamp`** —— 配对在它上面只会产出空（托盘侧没有可用的图像时刻），
            # 而 spec §7.1 要求它与改动前**逐位一致**。两者不可兼得，以回归要求
            # 为准。所以这一支一个字不动。
            seen = (int(pose_version), int(box_version))
            if seen == self._versions_seen:
                return Status.RUNNING
        else:
            # ---- camera：按**输入图像的采集时刻**配对 --------------------------
            # **不按版本号门禁**：版本变了只说明有新观测，配不配得上要按时间戳判。
            # 同一对重复 tick 由 `PalletFrameWindow.resolve()` 返回 `None` 挡掉。
            got = self._paired_inputs(pose, pose_version, box, box_version, slot)
            if got is None:
                # `_pending_reject` 由 `_paired_inputs` 在内部设好：
                #   配不成对 / 窗算不出来 / 配对结果用不了 / 停摆 → 有值 → 发
                #   正常去重（同一对已报过）                      → None → **不发**
                if self._pending_reject is not None:
                    self._publish_invalid(self._pending_reject)
                    # ★ 配不成对时**照样出图**：用的是**窗里那一对**观测
                    #   （`_paired_inputs` 就地覆盖过 `pose`/`box`），所以图上
                    #   画的就是"伺服真正看到的那两帧" —— 比看黑板最新值更贴题
                    #   （配不上多半正是因为两路各自的时刻对不上）。
                    #   正常去重（返回 None）那支**不发图**：那一 tick 什么都没
                    #   变，按 `overlay_period_s` 发的图与上一张逐像素一样。
                    self._maybe_publish_overlay(None, slot, pose=pose, box=box,
                                                reason=self._pending_reject)
                return Status.RUNNING
            pose, box, seen = got

        # T_cam_base 与 pose 的乘法**在技能层做**（`PalletServoSkill.on_execute`
        # 里的 `self._T_cam_base @ T_base_pallet`）。节点只负责把 T_cam_base
        # 解析出来传进参数 —— 几何只在一个地方算，回归测试才有唯一的真相。
        T_cam_base, src = self._resolve_t_cam_base()

        if self._skill is None:
            self._skill = PalletServoSkill(log_every_n=self._log_every_n)

        # 帧号由**节点**算，传给技能：技能自己也数，但它的计数器会被每帧的
        # `initialize()` 清零（旧写法让技能的每一条 WARNING 都写"第 1 帧"，
        # 而节点自己那条写"第 N 帧" —— 同一个事件两个号，而这两条都是
        # WARNING，默认级别下**都会落盘**）。节点这一份是权威编号。
        self._frames += 1

        params = PalletServoParams(
            pallet_pose=pose,
            box_obs=box,
            pallet_size_mm=self._size,
            ref_edges=self._slots[slot - 1],
            K=self._K,
            D=self._D,
            T_cam_base=T_cam_base,
            image_size=self._image_size,
            use_distortion=self._use_distortion,
            frame_no=self._frames)

        # 每帧都 initialize：位姿与观测都是这一帧的。技能层的 `_done` 会在
        # `on_initialize` 里清零，所以同一个技能实例能一直复用（与
        # `NodePalletPose` 每帧新建一个不同，那边没有可复用的状态）。
        init = self._skill.initialize(params)
        if not init.success:
            logger.error("托盘伺服初始化失败（本不该发生，配置已在 initialise "
                         "判过）：%s", init.message)
            self.feedback_message = init.message
            return Status.FAILURE

        result = self._skill.execute()

        if not result.success:
            # 退化帧：不写黑板、版本不自增，但留下**带实际值与阈值**的反馈
            self.feedback_message = result.message
            logger.warning("托盘伺服第 %d 帧被拒：%s", self._frames, result.message)
            self._maybe_dump(None, T_cam_base, src, pose, box, seen,
                             reason=result.message, slot=slot)
            self._versions_seen = seen      # 同一帧不重复刷屏
            self._publish_invalid(result.message)
            # ★ 被拒的帧更要出图：几何退化（参考边投不出来、箱子四角自交…）
            #   正是要靠图看的东西。这里用的 `pose` / `box` 是**窗平均那一对**
            #   （`_paired_inputs` 返回的），所以图上画的就是"伺服真正看到的两帧"
            #   —— 比看黑板最新值更贴题（这一帧正是它被拒的）。
            #   ⚠️ **不自己乘 `T_cam_pallet`**：`pose` 被上游写成 list/dict 时
            #   `pose6d_to_matrix()` 会直接抛（见
            #   `test_node_survives_upstream_objects_of_the_wrong_type`），而这条路
            #   正是坏输入常走的那条 —— 乘在 `_partial_T_cam_pallet()` 里做，它包了 try。
            self._maybe_publish_overlay(None, slot, pose=pose, box=box,
                                        reason=f"这一帧被拒 —— {result.message}")
            return Status.RUNNING

        error = result.data["error"]
        error.pallet_version = seen[0]
        error.box_version = seen[1]
        error.t_cam_base_src = src
        error.stamp = time.time()
        # 档位偏置（`servo_slots[*].e_right_bias_px` / `e_bottom_bias_px`，
        # board.json 可配）。**必须在这里**：写黑板 / 发话题 / 出叠加图都在下面，
        # 它们看到的要是同一个加过偏置的量（图上黄线也是按 `e_right_px` 画的）。
        right_bias = _slot_bias_of(self._slot_modes, slot - 1, "right")
        bottom_bias = _slot_bias_of(self._slot_modes, slot - 1, "bottom")
        if right_bias:
            error.e_right_px = float(error.e_right_px) + right_bias
        if bottom_bias:
            error.e_bottom_px = float(error.e_bottom_px) + bottom_bias

        self.global_blackboard.set(self._key, error)
        current = read_blackboard(self.global_blackboard,
                                   f"{self._key}_version", 0)
        setattr(self.global_blackboard, f"{self._key}_version", current + 1)
        self._versions_seen = seen
        self.feedback_message = error.to_log_line()
        self._log_throttled("托盘伺服第 %d 帧：%s", self._frames,
                            error.to_log_line())
        # 发图要在 dump 之前：dump 可能很慢（写盘），图是实时的。
        # `_last_T_cam_pallet` 给 render 用 —— 它要的是**相机系**的托盘位姿，
        # 而技能层算好的那个在 `error` 里没有，这里自己乘一遍。
        self._last_T_cam_pallet = T_cam_base @ pose6d_to_matrix(pose)
        # 话题要在 dump 之前：dump 可能很慢（写盘），话题是实时的。
        self._last_ok_mono = time.monotonic()
        self._publish_error(
            valid=True, reject="",
            e_bottom=error.e_bottom_px, e_right=error.e_right_px,
            theta=error.theta_rad, stamp=self._last_pair_stamp)
        self._maybe_publish_overlay(error, slot)
        self._maybe_dump(error, T_cam_base, src, pose, box, seen, slot=slot)
        return Status.RUNNING

    # ------------------------------------------------------------ 邻居路线
    def _run_neighbor_frame(self, slot, side, pose, box, *, box_version=0,
                            cand_version=0):
        """邻居档的一帧（`slot_mode == "neighbor"`）。**不走托盘路线那一整套。**

        与上面托盘那一段的区别（**只保证三个被控量同名同单位**）：

        * **不配对、不用托盘位姿。** 基准是别的箱子的边，输入是
          `node_box_obs` 写的**候选列表**（`latest_box_candidates`）。
        * **按输入版本去重**（2026-10-17 加）：行为树主循环固定 **50Hz**，而相机
          30Hz、检测器还要再降频。托盘路线靠配对窗天然去重，邻居路线从前是
          **每个 tick 都重算同一个框**（实测 50Hz 空转）。这里用 `(箱子版本,
          候选版本)` 当门禁：没有新输入就**不算、不发布** —— 帧率自然降到
          "检测器出多少算多少"，且不多于 30Hz。
          ⚠️ 跳过时**仍要跑停摆闸**：输入停了以后最后一条 `valid=true` 会一直
          留在话题上，下游会照着旧数继续走（`servo_error_stale_s` 那套的初衷）。
          所以跳过那一支会检查 `_last_ok_mono`，超时照样发无效帧。
        * 邻居路线失败 → 直接 `_publish_invalid`，**不降级**回托盘路线
          （操作员 2026-10-17 裁定；见 `neighbor_merge_spec.md` §5）。
        * 出的 `ServoError` 里，托盘专有的诊断量（`pallet_center_px` 等）
          **留空数组**，`warn` 里带 `source=neighbor` —— 叠加图据此区分这一帧是
          哪套算的（托盘被挡死的帧上那些量本来就没有意义）。
        """
        # ★ 输入没变 -> 这一 tick 不算。见 docstring 的"按输入版本去重"。
        key = (int(box_version), int(cand_version))
        if key == self._last_neighbor_key:
            mono = time.monotonic()
            if (self._servo_error_stale_s > 0.0
                    and self._last_ok_mono is not None
                    and mono - self._last_ok_mono >= self._servo_error_stale_s):
                self._publish_invalid(
                    f"邻居路线：{mono - self._last_ok_mono:.1f}s 没有新的箱子观测"
                    f"（最后一条 valid=true 已过期，下游不要照它走）")
            return Status.RUNNING
        self._last_neighbor_key = key

        # 候选列表：空 / 解析不出来都是空列表（生产者那边就是这么写的）。
        cands = read_blackboard(self.global_blackboard, self._candidates_key, [])
        if not isinstance(cands, (list, tuple)) or not cands:
            # `neighbor_servo` 本来就拒（`neighbor_no_up`），这里提前拦一句是为了
            # **把原因说成人话**：候选列表整条空着最常见的原因就是这一路检测
            # 还没出东西（`node_box_obs` 还没写过这个键），而 `neighbor_no_up`
            # 里那串"逐候选 []"读起来像是几何退化。
            reject = Reject(
                "neighbor_no_candidates",
                f"黑板 {self._candidates_key} 是空的 —— 邻居路线要的是"
                f"**全部候选**（NMS 之后、选箱之前），邻居就从里面挑。"
                f"多半是这一路箱子检测还没出东西")
            self.feedback_message = f"{reject}"
            self._publish_invalid(str(reject))
            self._maybe_publish_overlay(None, slot, pose=pose, box=box,
                                        reason=str(reject))
            return Status.RUNNING

        # ★ `held` 必须是**候选里那个同一个对象**（`neighbor.py` 靠 `is` 跳过它）。
        #   但黑板上 `latest_box_obs` 与候选列表里对应的那个是**两次独立构造的
        #   两个对象**（`node_box_obs` 分别造的，见接入契约 §2/§5）——
        #   `held is cands[i]` 恒为 False。所以按**四角**找：选箱结果本来就是候选
        #   之一，四角最接近的那个就是它。找不到就退回用 `held` 本身：
        #   `neighbor.py` 有 `dup_of_held` 兜底（中心重合的候选会被跳过）。
        held = self._nearest_candidate(cands, box)
        # 帧号由**节点**算（与托盘路线同一条纪律：技能的计数器每帧被清零，
        # 节点这一份才是权威编号）。这一句放在"等输入/空候选"两道闸**之后** ——
        # 与托盘路线里 `self._frames += 1` 在配对成功之后是同一个口径：
        # 它数的是"真的算过的帧"。
        self._frames += 1
        slot_cfg = neighbor_algo.NeighborSlot(side=side)
        # `image_size` 传进去是为了让算法能判"参照框是不是被画面边缘截断" ——
        # 截断的框中心和边都偏，不告警的话现场会以为"算错了"（真机实测过）。
        res = neighbor_algo.neighbor_servo(cands, held, slot_cfg,
                                           image_size=self._image_size)

        # ⚠️ **成功返回 `NeighborError` / 失败返回 `Reject`**（与托盘路线的
        #    `servo_error` 同一套约定）—— 判据是它有没有 `e_bottom_px`。
        if not hasattr(res, "e_bottom_px"):
            # **不降级**（操作员裁定）：拒了就拒了，不拿托盘路线再试一次 ——
            # 那会把"托盘被挡死"这个前提悄悄换成另一套基准，而三个数看着
            # 一样像模像样。走现成的 `_publish_invalid`，reason 用 Reject 原文。
            self.feedback_message = str(res)
            logger.warning("邻居伺服第 %d 帧被拒：%s", self._frames, res)
            self._publish_invalid(str(res))
            self._maybe_publish_overlay(None, slot, pose=pose, box=box,
                                        reason=f"这一帧被拒 —— {res}")
            return Status.RUNNING

        error = _neighbor_servo_error(res, held)
        error.stamp = time.time()
        # 与托盘路线同一句：偏置在写黑板 / 发话题 / 出图之前加（new = old + bias）。
        right_bias = _slot_bias_of(self._slot_modes, slot - 1, "right")
        bottom_bias = _slot_bias_of(self._slot_modes, slot - 1, "bottom")
        if right_bias:
            error.e_right_px = float(error.e_right_px) + right_bias
        if bottom_bias:
            error.e_bottom_px = float(error.e_bottom_px) + bottom_bias

        self.global_blackboard.set(self._key, error)
        current = read_blackboard(self.global_blackboard,
                                  f"{self._key}_version", 0)
        setattr(self.global_blackboard, f"{self._key}_version", current + 1)
        self.feedback_message = error.to_log_line()
        self._log_throttled("邻居伺服：%s", error.to_log_line())
        # 邻居路线没有"配对的图像采集时刻"这回事（不走配对窗）：话题的 header
        # 用出数时刻。⚠️ **不假装它是配对时刻** —— 那会让下游以为这条消息与
        # 图像对齐，而它没有。
        self._last_ok_mono = time.monotonic()
        self._publish_error(
            valid=True, reject="",
            e_bottom=error.e_bottom_px, e_right=error.e_right_px,
            theta=error.theta_rad, stamp=None)
        self._maybe_publish_overlay(error, slot)
        self._maybe_dump(error, np.eye(4), "邻居路线（无托盘位姿）", pose, box,
                         (-1, -1), slot=slot,
                         neighbor_diag=getattr(res, "diag", None))
        return Status.RUNNING

    def _nearest_candidate(self, cands, box):
        """在 `cands` 里找**四角与 `box` 最接近**的那个（同一个物理箱子）。

        `neighbor.py` 靠 `is` 认"手里箱"，而黑板上那两个键是**两个独立构造的
        对象**（`node_box_obs` 分别写的）—— 拿 `is` 比对恒为假，手里的箱子于是
        会被当成它自己的邻居。选箱结果本来就是候选之一，所以按四角找回来即可。

        **四角取不出来 / 找不到接近的 → 原样返回 `box`**（`neighbor.py` 的
        `dup_of_held` 兜底；它按"中心几乎重合"跳过副本，中心差得远就说明这不是
        同一个箱子，那时把它当候选用反而是对的）。**绝不猜**：这里只做"最近"，
        不设阈值 —— 阈值会把"选箱结果与候选差了几个像素"这种正常情形判成不匹配。
        """
        if box is None:
            return box
        try:
            q = box_corners(box)
        except Exception:                                    # noqa: BLE001
            return box
        if isinstance(q, Reject):
            return box
        best, best_d = box, None
        for cand in cands:
            try:
                cq = box_corners(cand)
            except Exception:                                # noqa: BLE001
                continue
            if isinstance(cq, Reject):
                continue
            d = float(np.abs(np.asarray(cq, np.float64)
                             - np.asarray(q, np.float64)).max())
            if best_d is None or d < best_d:
                best, best_d = cand, d
        return best

    # "这个键从未被写过"的哨兵。不能拿 `None` 或 0 代替：`None` 是"写过、值是
    # None"，0 是"写过、值是 0" —— 而这条判断要的正是"**从来没写过**"。
    _MISSING = object()

    def _version_of(self, key, value) -> int:
        """读 `key_version`；**从未写过**与"写了个 0"是两件事。

        `read_blackboard(..., default=0)` 把两者混成一个 0，于是"生产者只写了
        值、没写 `_version`"变成**静默停摆**：版本门禁第一次就停在 (0, 0)，节点
        算一次之后**永远不再算**，而 `feedback_message` 一直停在那一帧健康的
        `to_log_line()` 上、日志一声不响 —— 伺服于是持续追一个过期目标。这是
        本项目最忌的那类"静默过期"，所以这里分开判并**一次性 WARNING** 点出
        键名（每个键每次运行只喊一次，不是每帧一遍）。

        版本值不是整数时按 0 处理（与 `utils.blackboard.read_version` 同一口径）：旧代码在这里 `int()` 直接抛穿 `update()`，那会掀翻整棵树。
        """
        raw = read_blackboard(self.global_blackboard, f"{key}_version",
                               self._MISSING)
        if raw is self._MISSING:
            if value is not None and key not in self._version_warned:
                self._version_warned.add(key)
                logger.warning(
                    "黑板上 %s 有值，但 %s_version **从未被写过** —— 版本门禁会停在"
                    "(0, 0)，本节点只算一次然后**永远不再重算**，feedback 会一直"
                    "停在那一帧的读数上（看着完全正常）。生产者必须两个键一起写"
                    "（写法见 node_inject_servo_input）", key, key)
            return 0
        try:
            return int(raw)
        except (TypeError, ValueError):
            return 0

    def _log_throttled(self, fmt, *args):
        """正常帧的节流日志。用 `_ticks` 而不是 `_frames` 计数——"等输入"的帧
        不增加 `_frames`，拿它当模数会让第 0 帧那条每 tick 都打一遍。"""
        if self._ticks % self._log_every_n == 0:
            logger.debug(fmt, *args)

    # ------------------------------------------------------------ 坐标系 / 配对
    def _now_sec(self) -> float:
        """配对用的"现在"：**与 `header.stamp` 同一个时钟**（addendum A4）。

        `stale_s` 比的是"配上的这一对有多旧"（`now - pallet.stamp`），而这个差值
        只有在两个数**来自同一个时钟**时才有意义 —— 所以口径必须定死在这里，
        而不是随手写一个 `time.time()`。

        ROS 起着就用 `rospy.Time.now()`：它正是 `header.stamp` 的来源，而且
        `use_sim_time` 打开时它给的就是**仿真时间**（bag 回放 / 仿真里 stamp 是从
        0 开始的小数）。没有 ROS（单测 / 离线）退回 `time.time()`。

        ⚠️ **拿墙上时刻去比仿真时间戳 = 永远陈旧**：`stale_s > 0` 一旦配上
        `use_sim_time`，`now - stamp` 会是**整个 epoch** 那么大，每一帧都判 `stale`，
        伺服一个数都出不来。这正是 A4 要求先定死口径的原因。
        """
        try:
            import rospy
            return float(rospy.Time.now().to_sec())
        except Exception:                            # noqa: BLE001
            return time.time()

    def _partial_T_cam_pallet(self, pose):
        """没有三个数时，自己把托盘位姿乘到相机系；`pose` 缺或乘不动返回 `None`。

        与成功那一段的 `self._last_T_cam_pallet = T_cam_base @ pose6d_to_matrix(pose)`
        **是同一个定义** —— 叠加图上的青框要跟 `/pallet/detection` 的位置对得上，
        两处不能各算各的。

        `base_link` 模式下 `T_cam_base` 要查 TF，而这条路可能正是"没配对成功"，
        查不到就返回 `None`：**宁可不画台面那一层，也不拿单位阵顶替** ——
        单位阵拼出来的框会画在**别的地方**，而图上看着是个规规矩矩的矩形。
        """
        if pose is None:
            return None
        try:
            T_cam_base, _src = self._resolve_t_cam_base()
            return T_cam_base @ pose6d_to_matrix(pose)
        except Exception:                                    # noqa: BLE001
            return None

    def _resolve_t_cam_base(self):
        """返回 `(T_cam_base, src)`。**`pallet_frame == "camera"` 时是声明过的单位阵。**

        与 `TfCamBaseResolver.resolve()` 的区别全在 `src` 的写法上：
        这里的 `"identity(相机系位姿)"` 是**声明**，不是 TF 断掉之后的静默回退 ——
        两者在日志和图上必须区分得开，后者是已知的坑。
        """
        if self._pallet_frame == "camera":
            return np.eye(4), "identity(相机系位姿)"
        return self._resolver.resolve()

    def _paired_inputs(self, pose, pose_version, box, box_version, slot):
        """把两路观测按**输入图像的采集时刻**配对，返回 `(pose, box, seen)`。

        `pose` / `box` 是**就地覆盖**过的：配上的窗平均位姿 + 窗平均四角。这样
        调用方下游那三段（`PalletServoParams` / dump / 发图）一行都不用改。

        `slot` 是调用方在 `update()` 开头拍下的槽位快照，只用在 dump 里 —— 见
        `_maybe_dump`。

        返回 `None` 表示**这一 tick 不出数**（同一对已报过 / 配不成对 / 坏输入），
        此时 `feedback_message` 已经写好，调用方直接 `RUNNING` 即可。
        """
        now = self._now_sec()
        mono = time.monotonic()

        # ⚠️ 托盘的时间戳**从黑板的 `latest_pallet_stamp` 读**，不是从 `pose` 上
        # 取 —— `Pose6D` 只有六个数（x/y/z/yaw/pitch/roll），**装不下时间戳**。
        if pose_version != self._last_pallet_version:
            self._last_pallet_version = int(pose_version)
            self._push_pallet(pose, mono)
        if box_version != self._last_box_version:
            self._last_box_version = int(box_version)
            self._push_box(box, mono)

        self._mono = mono
        try:
            paired = self._pairing.resolve(now=now)
        except Exception as exc:                     # noqa: BLE001
            # 配对窗是纯算法，走到这儿抛只可能是**缓存里有个坏对象**（比如
            # `latest_box_obs` 是 dict、`quad` 里是字符串）。坏输入不许抛穿
            # `update()`（py_trees 不接 —— 那会连每帧的日志一起没）。
            self._pending_reject = (
                f"配对窗算不出来（{type(exc).__name__}: {exc}）"
                f" —— 上游给的类型不对时每帧都会这样")
            self._warn_pairing(f"配对窗算不出来（{type(exc).__name__}: {exc}）"
                               f" —— 这一帧不出数。**上游给的类型不对**时每帧"
                               f"都会这样")
            # ⚠️ **这一处不接 `_check_liveness` 的返回值**（addendum §3）：
            # 这一路已经有更具体的原因（`配对窗算不出来（…）`），停摆描述会把
            # 那个更具体的原因挤掉 —— 把真正的故障藏起来。
            self._check_liveness(mono)
            return None

        if paired is None:
            # **两种含义共用一个返回值**（addendum A4）：① 同一对已经报过
            # （正常去重）② 某一侧没有新观测（**可能是检测器停摆了**）。
            # 窗分不清这两件事，节点分得清 —— 见 `_check_liveness`。
            # 返回 `None` = 正常去重 → `_pending_reject` 留 `None` → **不发**。
            self._pending_reject = self._check_liveness(mono)
            return None

        if isinstance(paired, PairReject):
            self.feedback_message = f"配不成对 —— {paired}"
            self._pending_reject = str(paired)
            self._log_throttled("托盘伺服第 %d tick 配不成对：%s",
                                self._ticks, paired)
            # 配对失败也是"异常帧"（`dump_on` 默认含 `"reject"`）。
            # ⚠️ 这一段在 A 段（`T_cam_base` 解析）**之前**，那两个名字还没定义 ——
            # 用 `np.eye(4)` 与 "配对失败" 占位即可：dump 里的 `T_cam_base` 字段
            # 这时没有意义，`reason` 才是要看的东西。
            # `seen` 用**真实版本号**（Minor M-4）：复现"配不上"时最有用的两个数
            # 就是"是哪两个版本配不上"，`(0, 0)` 会把它们丢掉。
            self._maybe_dump(None, np.eye(4), "配对失败", pose, box,
                             (int(pose_version), int(box_version)),
                             reason=str(paired), slot=slot)
            return None

        # 配对成功：用**窗上的平均**而不是单帧。`paired.box_quad` 的顺序与
        # `BoxObservation.quad` 的契约一致（右下→左下→左上→右上），直接填，
        # **不要重排**（addendum A1）。
        #
        # ⚠️ **`seen` 里的版本号与真实喂给技能的东西不是一回事**（Minor M-5）：
        # camera 模式下 `pose` / `box` 是**窗上的平均**（可能来自 `window` 帧之前
        # 的观测），而 `seen` 写的是**黑板此刻最新收到的版本号**。它俩于是不同步
        # —— 存进 `ServoError.pallet_version` 的是"最新收到的"，不是"这一对的"。
        # 目前全仓只有 `to_log_line` 在用它（给人看的出处），没有消费者拿它做
        # 门禁/去重，所以不改行为；要改的话得先把"这一对是哪两帧"的出处从
        # `PairedFrame` 里取出来，那是接口变更，不在本轮范围。
        try:
            box = BoxObservation(
                quad=[[float(u), float(v)] for u, v in paired.box_quad],
                label=f"paired(n={paired.n_pairs})", confidence=1.0,
                stamp=float(paired.box_stamp))
            # 配对后的位姿**已经在相机系**，直接转成 Pose6D 交给技能层。
            # `T_cam_base` 在 camera 模式下是单位阵，乘不乘都一样。
            pose = matrix_to_pose6d(paired.T_cam_pallet)
        except Exception as exc:                     # noqa: BLE001
            self._pending_reject = (
                f"配对结果用不了（{type(exc).__name__}: {exc}）")
            self._warn_pairing(f"配对结果用不了（{type(exc).__name__}: {exc}）"
                               f" —— 这一帧不出数")
            return None
        # 有效帧的 header.stamp 要**配对的图像采集时刻**（不是出数时刻）——
        # 与 /pallet/detection、/box/detection 同一基准，录 bag 回放时能和图像对齐
        self._last_pair_stamp = float(paired.pallet_stamp)
        # 出数了 = 停摆结束：存活告警的槽位复位，下一次停摆要重新喊
        self._last_live_warn = None
        return pose, box, (int(pose_version), int(box_version))

    def _push_pallet(self, pose, mono) -> None:
        """把这一帧托盘观测推进配对窗。**坏输入不抛，只不出数。**

        `stamp` 的两种口径（addendum A7，**评审后改过**）：

        * **读不到这个键**（`None`）→ 这是**老生产者**（`NodePalletPose` 那类，
          契约里根本没有这个键）→ 退回 tick 时刻 + **告警一次**。**不能因此
          `return`** —— 那会让整条老路径彻底哑掉，而"哑掉"是没有任何报错的。
        * **读到了但 ≤ 0** → 这是**我们自己的** `NodePalletObs` 按契约写的
          （上游没填 `header.stamp` 时它写原始值 `0.0`）→ **不 push** + 告警一次
          （与 Task 4 同一口径：宁可少出数，不出错数）。写 `0.0` 会让配对侧显式
          reject（`pair_stamp_zero`），而那正是想要的失败模式。

        判据是「**生产者的行为**」（有没有这个键），不是 `pallet_frame` 的取值 ——
        这样不引入模式相关的分支。
        """
        try:
            stamp = read_blackboard(self.global_blackboard,
                                     f"{self._pallet_key}_stamp", None)
            if stamp is None:
                self._warn_missing_stamp_once(missing=True)
                # "tick 时刻"用的是**本节点那个时钟**（`_now_sec()`，ROS 起着就是
                # ROS 时间）—— 与箱子那边的 stamp 同源，配对的差值才有意义。
                stamp = self._now_sec()
            elif float(stamp) <= 0.0:
                self._warn_missing_stamp_once(missing=False)
                return
            else:
                stamp = float(stamp)
            self._pairing.push_pallet(PalletObservation(
                T_cam_pallet=pose6d_to_matrix(pose),
                stamp=stamp,
                # ⚠️ 这里**恒为空串**，不是漏填：黑板上的 `latest_pallet` 是
                # `Pose6D`（只有六个数），没有 `label`。检测器的 `source` 只进
                # `NodePalletObs` 的节点日志、**不进黑板**（见
                # `PalletObservation.source` 的 docstring）。留着这个字段是因为
                # 配对层的契约里有它；要真的填上得先让生产者往黑板上写一个位子。
                source=str(getattr(pose, "label", "") or "")))
            self._last_pallet_seen = mono
        except Exception as exc:                     # noqa: BLE001
            self._warn_pairing(
                f"托盘观测推不进配对窗（{type(exc).__name__}: {exc}）—— 这一帧"
                f"不出数。**上游给的类型不对**（比如 latest_pallet 是 6 个数的 "
                f"list、不是 Pose6D）时每帧都会这样")

    def _push_box(self, box, mono) -> None:
        """把这一帧箱子观测推进配对窗。`stamp` 就在这里先解一次（坏对象不抛）。"""
        try:
            # **先验后推**：坏类型（dict / 字符串）不许进缓存 —— 进了的话
            # `resolve()` 里每个 tick 都会炸，而它能待满 `pair_cache` 次推入。
            float(box.stamp)
        except Exception as exc:                     # noqa: BLE001
            self._warn_pairing(
                f"箱子观测推不进配对窗（{type(exc).__name__}: {exc}）—— 这一帧"
                f"不出数。**上游给的类型不对**（比如 latest_box_obs 是 dict、"
                f"不是 BoxObservation）时每帧都会这样")
            return
        self._pairing.push_box(box)
        self._last_box_seen = mono

    def _activate_slot(self, slot: int) -> Tuple[bool, str]:
        """切到第 `slot` 组参考边；`slot=0` 停止。返回 `(ok, 人话原因)`。

        ⚠️ **越界时绝不改状态**：下游手滑发个 `3` 不该把正在跑的伺服停掉。

        ⚠️ 服务从 **1** 数，`self._slots` 从 **0** 数 —— `slot=2` 用
        `self._slots[1]`。这个差一位选错了**不会有任何报错**（三个数照样算得
        出来，只是基准是隔壁箱位的），所以 srv 注释、板子 remark、这条消息
        里都要把它说清楚。
        """
        # ⚠️ **总数用 `self._total_slots`**（2026-10-17 修）。从前这里按
        #    `len(self._slots)` 算，`ref_edges` 比 `servo_slots` 短时**邻居档
        #    永远切不过去**（真机实测：5 档的表只能 call 到 2，第 5 档被拒成
        #    "表里只有 4 组"）。那个数是 `__init__` 里算好的（两者取大）。
        total = self._total_slots
        try:
            slot = int(slot)
        except (TypeError, ValueError):
            return False, f"slot 不是整数：{slot!r}"
        if slot < 0 or slot > total:
            # ⚠️ **文案说"档"不说"组"**（2026-10-17）：档位数与 `ref_edges` 组数
            #    不再相等（邻居档不读参考边），说"组"会把人引去数错那张表 ——
            #    真机那次误判的起因正是这句话里的两个数被当成了同一个。
            return False, (f"第 {slot} 档不存在 —— 表里只有 {total} 档"
                           f"（1..{total}，0 = 停止）。当前仍是 "
                           f"{self._active_slot}（越界调用不改状态）")
        was, self._active_slot = self._active_slot, slot
        # 版本号 +1 让 `update()` 知道**该重算了**。不 +1 的话切换之后那一对
        # 早被配过、水位线也还在它上面，`resolve()` 会一路返回 `None` —— 表现
        # 是"call 完服务之后还是老基准的数"，而这**不会报任何错**（三个数照样
        # 算得出来，只是基准是上一个箱位的）。
        self._slot_version += 1
        if slot == 0:
            return True, f"本箱位伺服已结束（之前是第 {was} 档）"
        mode, side = _slot_mode_of(self._slot_modes, slot - 1)
        if slot - 1 < len(self._slots):
            # ⚠️ `self._slots` **从 0 数**、服务**从 1 数** —— 这个差一位选错了
            #    不会有任何报错（三个数照样算得出来，只是基准是隔壁箱位的）。
            detail = f"参考边 {self._slots[slot - 1]}"
        else:
            # 邻居档不读参考边，所以"这一档没有 ref_edges"是**正常**的，不是错
            # （pallet 档缺参考边在启动时就报配置错误了）。**如实说**，不要
            # `IndexError`、也不要编一个"参考边"出来。
            detail = (f"这一档走邻居路线，不用参考边"
                      f"（ref_edges 只有 {len(self._slots)} 组）")
        return True, (f"切到第 {slot}/{total} 档（{mode}"
                      + (f"/{side}" if mode == "neighbor" else "")
                      + f"），{detail}")

    def _check_liveness(self, mono) -> Optional[str]:
        """存活闸（addendum A4）：某一侧多久没有**新的观测** → 记一次 WARNING。

        **为什么不在 `PalletFrameWindow` 里判**：`resolve()` 返回 `None` 有**两种
        含义且不可区分** —— ① 同一对已经报过（正常去重）② 某一侧没有新观测
        （**可能是检测器停摆了**）。窗只看得见 stamp，分不清这两件事；**节点分得清**
        —— 它自己知道这一 tick 有没有往某一侧推过东西（`_last_pallet_seen` /
        `_last_box_seen`）。所以存活判据做在这一层。

        ⚠️ **不要用 `stale_s` 冒充它**：`stale_s` 是**延迟闸**（判"配上的这一对
        有多旧"，判在去重之后），某一路停摆**永远轮不到它**。两者判的是完全不同
        的东西，`live_timeout_s` 是本节点自己的旋钮。

        `mono` 是 `time.monotonic()`（与 stamp 的时钟无关 —— stamp 可能是仿真时间
        或来自 bag，拿它算"多久没出数"会在时钟回跳时胡说）。

        ⚠️ **告警的去重键是"停摆侧别集合"，不是文案**（与 Task 4 修掉的是同一类病）：
        文案里嵌着 `{worst:.1f}s`，而 `worst = mono - _last_*_seen` **每 tick 都在
        长** —— 10 Hz 下 `{:.1f}` 每 tick 变一次，按整条文案比对**恒不相等**，
        去重形同虚设（实测 10 个 tick 10 条 WARNING）。停摆是**持续**状态，而这条
        告警恰好在"最需要看日志"的时刻每秒刷 10 行长文案。
        操作员要行动的信息是**哪一路停了**，时长在真机上没有判别力 —— 所以键取
        `("托盘",)` / `("箱子",)` / `("托盘", "箱子")`，`worst` 只进正文与
        `feedback_message`（后者每 tick 照常刷新，HUD 上仍看得见当前时长）。

        ★ **返回停摆描述**：`update()` 把它当作无效帧的 `reject` 发到话题上。
        返回 `None` = **没有停摆可报**（不查 / 这一 tick 没停摆）—— 调用方据此
        区分"停摆"与"正常去重"。`feedback_message` 照旧设，HUD 上那个行为一个字
        不变。
        """
        if self._live_timeout_s <= 0.0:
            return None                      # 不查 = 没有停摆可报
        ages = (("托盘", mono - self._last_pallet_seen),
                ("箱子", mono - self._last_box_seen))
        stalled = [(name, age) for name, age in ages
                   if age > self._live_timeout_s]
        if not stalled:
            self._last_live_warn = None      # 恢复了：下一次停摆要重新喊
            return None
        worst = max(age for _, age in stalled)
        message = (
            f"检测器停摆？ {'、'.join(n for n, _ in stalled)} 已经 {worst:.1f}s "
            f"没有**新的观测**（阈值 {self._live_timeout_s:g}s）—— 黑板停在旧值上，"
            f"三个数不再更新。`resolve()` 返回 `None` 把'正常去重'与'某路停摆'"
            f"压成同一个返回值，这条告警补的就是那一格（⚠️ 判存活**不用** stale_s，"
            f"那是延迟闸）")
        key: Tuple[str, ...] = tuple(name for name, _ in stalled)
        if key != self._last_live_warn:
            self._last_live_warn = key
            logger.warning("NodePalletServo %s", message)
        self.feedback_message = message
        return message

    def _warn_missing_stamp_once(self, missing: bool) -> None:
        """黑板上的 `latest_pallet_stamp` 读不到 / ≤ 0 时喊一次。

        **这是静默错的源头**：拿不到图像采集时刻，配对只能退回 tick 时刻，于是
        两个检测器的**耗时差直接变成配对的时间偏差** —— 配上的那一对可能不是同一
        帧图像，而伺服照样出数。

        两个分支的处置**不同**（addendum A7）：
          * `missing=True`（键读不到）—— 老生产者，退回 tick 时刻**照常 push**；
          * `missing=False`（读到 0.0）—— 我们自己的检测器没填 `header.stamp`，
            **不 push** 这一帧。

        ⚠️ 一次性槽位 `_stamp_warned` 在 `__init__` 里建、在 `initialise()` 里
        **复位**（M7）—— 从前是懒建的，换场景 / 重树之后这条告警**不会再响**。
        """
        if self._stamp_warned:
            return
        self._stamp_warned = True
        if missing:
            logger.warning(
                "黑板上没有 %s_stamp —— **老生产者**（NodePalletPose 那类不写这个"
                "键），配对退回 tick 时刻。⚠️ 托盘与箱子的**耗时差会直接变成配对"
                "的时间偏差**，配上的可能不是同一帧图像：要按图像时刻配对就得让"
                "生产者把图像采集时刻写进这个键", self._pallet_key)
        else:
            logger.warning(
                "%s_stamp 写的是 0.0（**上游检测器没填 header.stamp**）—— 这一帧"
                "托盘观测**不 push**，配对会失败（pair_stamp_zero）。去修检测器的"
                " header.stamp；**本节点不编造时刻**：编一个出来就可能配上一对"
                "不同帧的图像，而三个数照样算得出来", self._pallet_key)

    def _warn_pairing(self, message: str) -> None:
        """配对路上的坏输入：按原文去重（坏帧是按 tick 频率发生的事）。"""
        if message != getattr(self, "_last_pairing_warn", None):
            self._last_pairing_warn = message
            logger.warning("NodePalletServo %s", message)

    # ------------------------------------------------------------------ 可视化
    def _on_color(self, msg) -> None:
        """存最新彩色帧（**统一转成 BGR**）。**只存，不画。**

        ⚠️ **必须看 `msg.encoding`**（2026-10-26 现场）：相机发的是 **`rgb8`**，
        而这里原来把裸缓冲直接当 BGR 存，叠加图就红蓝对调 —— 与相机原图颜色
        不一致。这里手工解包（不用 `cv_bridge`，避免 boost_python），口径与
        `carton_obb_detect._arr_to_bgr` / `pallet_detection_node._arr_to_bgr` 一致。
        """
        try:
            import numpy as np
            row = msg.step // np.dtype(np.uint8).itemsize
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, row)
            encoding = str(getattr(msg, "encoding", "") or "bgr8").strip().lower()
            if encoding in ("bgr8", "8uc3", "rgb8"):
                channels = 3
            elif encoding in ("mono8", "8uc1"):
                channels = 1
            else:
                raise ValueError(
                    f"不支持的彩色编码 {encoding!r}（要 bgr8 / rgb8 / mono8）")
            n = msg.width * channels
            if n > row:
                raise ValueError(f"一行只有 {row} 个元素，装不下 "
                                 f"{msg.width}×{channels}")
            arr = a[:, :n].reshape(msg.height, msg.width, channels)
            if encoding == "rgb8":
                bgr = arr[:, :, ::-1]                    # RGB -> BGR
            elif channels == 1:
                bgr = np.repeat(arr, 3, axis=2)          # mono -> BGR
            else:
                bgr = arr
            bgr = np.ascontiguousarray(bgr).copy()
        except Exception as exc:                         # noqa: BLE001
            self._warn_overlay(f"彩色帧解不开：{exc}")
            return
        with self._overlay_lock:
            self._last_color = bgr

    def _publish_error(self, *, valid, reject, e_bottom=float("nan"),
                       e_right=float("nan"), theta=float("nan"),
                       stamp=None) -> None:
        """发一帧伺服误差。**整段包 try/except，失败只记 WARNING，绝不抛。**

        `stamp` 给 `None` 时用 `rospy.Time.now()`（只用于无效帧 —— 有效帧必须
        传**配对的图像采集时刻**，见 `_publish_error` 的调用点）。两种含义的
        契约在 `PalletServoError.msg` 里。

        ## 节流

        * **有效帧（`valid=True`）：每帧都发。** 出数本来就有帧级去重
          （同一对不会重复算），不需要第二层。
        * **无效帧（`valid=False`）：`(valid, reject)` 变了就立刻发**；
          没变则每 `servo_error_heartbeat_s` 重发一次。

        节流的理由是**不让消费者陷入沉默**：无效帧的内容在一次停摆里是
        **逐字相同**的，不去重就是每 tick 一条（10 Hz 下一小时两万条一样的消息）；
        而完全不发又会让"停摆"与"话题名写错"分不开。心跳是这两者之间唯一
        说得通的中间态：**内容变了立刻说，没变也每秒说一次**。

        ⚠️ **去重键取 `(valid, reject)` 而不是整条消息** —— 整条消息里带着
        `e_bottom_px` 等逐帧在变的量，拿它做键等于没去重（本项目反复踩的
        "告警去重按稳定键，不按整条文案"，见 `_check_liveness` / `_warn_overlay`）。

        ⚠️ **`reject` 本身也必须先去读数**（`_reject_dedup_key`）：停摆那条文案里
        嵌着 `已经 8.2s` 这种**每 tick 都在长**的量，拿原文做键同样恒不相等 ——
        实测真机跑出来的效果就是**每个 tick 一条**（10 Hz），而设计要的是
        「内容变了立刻说、没变每秒一次」。**发出去的 `msg.reject` 仍是原文**
        （读数是现场信息），被砍掉的只是**去重用的键**。
        """
        if self._servo_error_pub is None:
            return
        now = time.time()
        # ★ **"检测停了"必须在 `live_timeout_s`（5s 告警阈值）之前就说出去**
        # （2026-10-10）。检测一停，`resolve()` 就返回 `None`，而"正常去重"与
        # "某侧停摆"共用那个返回值 —— 那一段我们什么都不发，下游拿的还是最后
        # 一条 `valid=true`，就照它一直走。这里补的正是那一格：**只要距上次
        # 成功出数超过 `servo_error_stale_s`，就把当前这条（无效）原样当
        # 停发帧发出去**，绕开下面那套去重/心跳。
        #
        # ⚠️ **不改 `live_timeout_s`**：那是**告警**阈值，调小会让停摆告警变吵；
        #    两者判的东西不同（一个管"下游怎么走"，一个管"日志怎么喊"）。
        if ((not valid) and self._servo_error_stale_s > 0.0
                and self._last_ok_mono is not None):
            mono = getattr(self, "_mono", None)
            if mono is not None and (mono - self._last_ok_mono
                                     >= self._servo_error_stale_s):
                self._last_error_pub_key = None      # 不污染下一帧的去重
                self._last_error_pub_t = now
                self._publish_error_now(valid=False, reject=reject)
                return
        key = (bool(valid), _reject_dedup_key(str(reject)))
        if valid:
            self._last_error_pub_key = key
            self._last_error_pub_t = now
        else:
            if key == self._last_error_pub_key:
                # ⚠️ **心跳 0 = 只在内容变化时发**：`now - t < 0` 恒不成立，
                # 少了这一句 `heartbeat_s=0` 就退化成"每次调用都发"。
                if self._servo_error_heartbeat_s <= 0.0:
                    return
                if now - self._last_error_pub_t < self._servo_error_heartbeat_s:
                    return
            self._last_error_pub_key = key
            self._last_error_pub_t = now
        self._publish_error_now(valid=valid, reject=reject, e_bottom=e_bottom,
                                e_right=e_right, theta=theta, stamp=stamp)

    def _publish_error_now(self, *, valid, reject, e_bottom=float("nan"),
                           e_right=float("nan"), theta=float("nan"),
                           stamp=None) -> None:
        """立刻发一帧（**不走节流**）。**整段包 try/except，绝不抛。**

        两条路共用：① 停发帧（`servo_error_stale_s` 那条，见 `_publish_error`）；
        ② `_publish_error` 自己节流通过之后的实际发送。
        """
        if self._servo_error_pub is None:
            return
        try:
            import rospy
            from pallet_servo_msgs.msg import PalletServoError

            msg = PalletServoError()
            msg.valid = bool(valid)
            msg.reject = str(reject)
            # ⚠️ **无效帧的三个量是 NaN，不是 0.0**（`0.0` 恰好是"误差为零 =
            # 完全对准"的意思，漏判 `valid` 的消费者会以为箱子已经到位）。
            msg.e_bottom_px = float(e_bottom)
            msg.e_right_px = float(e_right)
            msg.theta_rad = float(theta)
            msg.header.frame_id = self._camera_frame
            if stamp is None:
                msg.header.stamp = rospy.Time.now()
            else:
                msg.header.stamp = rospy.Time.from_sec(float(stamp))
            self._servo_error_pub.publish(msg)
        except Exception as exc:                         # noqa: BLE001
            logger.warning("NodePalletServo 发伺服误差失败（%s）—— 黑板不受"
                           "影响，这一帧的话题消息丢了", exc)

    def _publish_invalid(self, reject: str) -> None:
        """无效帧的便捷入口：三个量填 NaN、`stamp` 用当前时刻。"""
        self._publish_error(valid=False, reject=reject)

    def _publish_detect_mode(self, slot, slot_mode) -> None:
        """告诉两个检测器这一档要不要出数。**纯输出设施，绝不抛。**

        取值与理由见 `initialise()` 里建发布器那一段。发送口径：
          * **变了立刻发**（call 服务之后要马上生效）；
          * 没变每 1s 补一条（检测器后启动 / 重启时能拿到当前状态；发布器
            本身是 latched 的，这一条是第二道保险）。
        """
        if self._detect_mode_pub is None or self._detect_mode_msg is None:
            return
        want = 0 if slot <= 0 else (2 if slot_mode == "pallet" else 1)
        now = time.monotonic()
        if want == self._last_detect_mode and now - self._last_detect_mode_t < 1.0:
            return
        changed = (want != self._last_detect_mode)
        self._last_detect_mode = want
        self._last_detect_mode_t = now
        try:
            self._detect_mode_pub.publish(self._detect_mode_msg(data=int(want)))
        except Exception as exc:                         # noqa: BLE001
            self._log_throttled("检测门控发布失败：%s", exc)
            return
        if changed:
            logger.info("检测门控 → %d（%s）", want,
                        {0: "全停", 1: "只要纸箱", 2: "托盘+纸箱"}.get(want, "?"))

    def _maybe_publish_overlay(self, error, slot, *, pose=None, box=None,
                               reason="") -> None:
        """按 `overlay_period_s` 降频发一张叠加图。

        ★ **不是"三个数算出来了"才画。** 三个数要托盘与纸箱**两路检测都出了
        结果、而且配得上对**才算得出来，而排查时最需要的恰恰是"只有一边有
        结果"的那些帧 —— 那时整张图都没有的话，操作员分不出是**这一路检测
        没出东西**还是**叠加图这条链坏了**。所以：

          * `error` 在（这一 tick 真算出了三个数）→ 画全套，与从前一样；
          * `error` 不在 → 用黑板上的**原始观测**画：有托盘位姿就画台面矩形 +
            参考边，有箱子四角就画箱子四条边，有外接框就画灰框。
            **哪一样没有就不画哪一样**（`render_servo_overlay` 里每一层都是
            可选的），HUD 上写清**为什么没有三个数**（`reason`）。

        调用方传给它的 `pose` / `box` 就是**它手上那两帧观测** —— 没配成对/被拒
        那几支传的是黑板原始值（配不上对时那两个局部量还没被覆盖），被拒那一支
        传的则是窗平均后的那一对。两条都是"伺服真正在看的输入"，正是排查要看的东西。

        **整段包在 try 里：可视化是诊断设施，它不该有能力把节点弄死。**
        出图前先跑 `numeric_self_check`；不过就**不发图**并记一次 WARNING ——
        人眼看到叠加图不对时，第一嫌疑应该是画图代码，不是数据。

        `slot` 是调用方在 `update()` 开头拍下的槽位快照：参考边的标签要用
        **这一帧**那一组（`self._slots[slot - 1]`）的原文，而 `self._active_slot`
        可能已经被服务回调改掉了 —— 标签指向的边与实际算出来的数就分家了。
        `slot <= 0`（未激活）时**不画参考边**（没有"这一帧用哪一组"可言），
        台面矩形照画。

        ⚠️ **这条链路必须自己可观察**：它对操作员来说是**唯一**的人工判读手段，
        而"一条消息都没有"与"rqt 里话题名写错了"长得一模一样。所以：
          * 发不出去的两条路（没底图 / 自检不过）各有计数器，并按节流进日志
            （`_log_overlay_counts`）；
          * 自检不过那条告警**按周期提醒**，不是只喊开头一次（`_warn_overlay`）。
        """
        if self._overlay_pub is None:
            return
        now = time.time()
        if self._overlay_period_s > 0.0 and \
                now - self._last_overlay_t < self._overlay_period_s:
            return
        with self._overlay_lock:
            color = self._last_color
        # ⚠️ **灰框不再来自 `/box/yolo_box`**（那条话题 2026-10-15 删了）：它发的就是
        # 箱子四角的**轴对齐包围盒**，而这里手上这个 `box`（黑板上的 `BoxObservation`）
        # 的 `u1/v1/u2/v2` **本来就是同一个数** —— `node_box_obs` 是用
        # `min/max(corners)` 填的。所以删掉那条话题零信息损失，也**不用加订阅**。
        box_uv = None
        if box is not None and getattr(box, "u2", None) is not None:
            u1, v1 = float(getattr(box, "u1", 0.0)), float(getattr(box, "v1", 0.0))
            u2, v2 = float(box.u2), float(box.v2)
            box_uv = (min(u1, u2), min(v1, v2), max(u1, u2), max(v1, v2))
        if color is None:
            self._n_overlay_skip += 1
            self._log_overlay_counts("底图还没来")
            return
        self._last_overlay_t = now
        try:
            from skills.atomic.perception.pallet_servo.render import (
                numeric_self_check, render_neighbor_overlay,
                render_servo_overlay)

            # 槽位那一组参考边：`slot` 从 1 数，`self._slots` 从 0 数。
            # 未激活（`slot <= 0`）时给 `None` —— 不画参考边，也不写它的标签。
            # ★ **超过 `ref_edges` 组数的档位同样给 `None`**（2026-10-17）：那些
            #   档是 `neighbor`（比 `ref_edges` 长且是 pallet 的话启动就报配置
            #   错误了），本来就没有参考边。给 `None` 正是不画这一层 —— 不许
            #   `IndexError`，也不许拿别的档那一组顶着画（那会画出一个与这一帧
            #   毫无关系的基准，而它看着规规矩矩）。
            ref_edges = (self._slots[slot - 1]
                         if 1 <= slot <= len(self._slots) else None)

            # ★ **这一帧走的是不是邻居路线**（2026-10-17 操作员裁定：邻居档的
            #   HUD 只留三个数，不写托盘尺寸 / 上游尺寸 / 配对版本号）。
            #   判据取**档位表**而不是 `error.box_source`：被拒的帧 `error is None`，
            #   那样判不出来 —— 而被拒的邻居帧同样不该出现托盘尺寸那一行。
            slot_mode, _slot_side = (
                _slot_mode_of(self._slot_modes, slot - 1) if slot > 0
                else ("pallet", "none"))
            is_neighbor = (slot_mode == "neighbor")

            if error is not None:
                # 相机系的托盘位姿在成功那一段里算好了（`error` 里没有它）。
                # ★ **邻居路线的帧不走这一格**（2026-10-17）：那条路线根本没看见
                #   托盘，`self._last_T_cam_pallet` 会停在**上一帧托盘路线**留下的
                #   值上 —— 拿它画青框的画，图上会出现一个与这一帧毫无关系的台面
                #   矩形，而它看着规规矩矩（正是本项目最忌的那类静默错）。所以
                #   按这一帧黑板上的位姿现算：有位姿就画（说明托盘其实还看得见，
                #   那是现场信息），没有就不画这一层。
                if getattr(error, "box_source", "") == "neighbor":
                    T_cam_pallet = self._partial_T_cam_pallet(pose)
                else:
                    T_cam_pallet = self._last_T_cam_pallet
                # ⚠️ **这一支不传 `box_quad_uv`**：绿线画的是 `error` 里那两条
                # **伺服真的用了的**边（底/右），再叠一整圈四角会多出两条会与
                # 别的图层撞的线，而"整圈框歪不歪"在没有三个数的那一支已经画了
                # —— `render` 那边本来也只在没有 `error` 时才看这个参数。
                box_quad = None
                # ⚠️ **角度用弧度**（2026-10-08 操作员裁定）：与 `/pallet_servo/dis`
                # 的 `theta_rad` **同单位**。从前这里显示度（`math.degrees(...)`），
                # 于是拿 `rostopic echo /pallet_servo/dis` 的读数去核图上那行字，
                # 两个数对不上（差 57.3 倍）。叠加图是**唯一**的人工判读手段，
                # 它的读数必须与对外话题同源同单位。
                hud = [f"e_bottom={error.e_bottom_px:+.1f}px  "
                       f"e_right={error.e_right_px:+.1f}px  "
                       f"theta={error.theta_rad:+.6f}rad"]
                if is_neighbor:
                    # ⚠️ **邻居档不写托盘专有的那行**（2026-10-17 操作员裁定）。
                    #    `pallet_v` / `box_v` 恒为 -1（邻居路线不配对）、
                    #    `t_cam_base` 是"邻居路线（无托盘位姿）"这种占位说明 ——
                    #    它们对判读这一帧**没有任何作用**，只会把有用的三行挤下去。
                    pass
                else:
                    hud.append(
                        # ★ `pallet_v` / `box_v` 是**配对那一套**的版本号；
                        #   邻居路线不配对，那两个字段恒是 `-1`。
                        f"pallet_v={error.pallet_version} "
                        f"box_v={error.box_version} "
                        f"t_cam_base={error.t_cam_base_src} "
                        f"box={error.box_source}")
                if error.warn:
                    hud.append("warn: " + "; ".join(error.warn))
            else:
                # ★ 没有三个数：把**手里有什么**画出来，HUD 写清缺在哪一步。
                # 托盘位姿自己乘一遍（与成功那一段同一套定义），三样不齐就
                # 不画托盘那两层 —— `render` 那边会各自判。
                T_cam_pallet = self._partial_T_cam_pallet(pose)
                box_quad = _box_quad_of(box)
                # ⚠️ **`ref_edges is None` 有两种来路**，HUD 上要分得开
                #    （2026-10-17）：`slot <= 0` 是**未激活**，而 `slot` 在
                #    `ref_edges` 组数之外是**那一档本来就没有参考边**（邻居档）
                #    —— 写成同一个"（未激活）"会让现场以为服务没 call 上。
                if slot <= 0:
                    slot_ref = "（未激活）"
                elif ref_edges is None:
                    slot_ref = (f"（这一档没有参考边，只有 {len(self._slots)} 组）")
                else:
                    slot_ref = ref_edges
                hud = [f"⚠ 没有伺服差：{reason or '（未说明）'}",
                       f"托盘 {'有' if pose is not None else '缺'} / "
                       f"纸箱 {'有' if box is not None else '缺'}"
                       + ("" if box_quad is None
                          else f"（四角 {len(box_quad)} 点）"),
                       f"slot={slot}/{self._total_slots} 参考边={slot_ref}"]

            # ★ **尺寸从哪来**（2026-09-30 加）：上游报的 vs 配置兜底的。
            # 现场那个"青框比托盘大一圈"就是两者不同源造成的，而图上**看不出来**
            # 用的是哪一份 —— 只能靠这行字。`from_upstream=0` 时去看
            # `latest_pallet_size_mm` 有没有值（检测器没出结果时它就是 None）。
            # ⚠️ `self._size` 可能是 `None`（配置错误那条路就是"尺寸没定下来"），
            #    **照实写出来**，不要拿 0 顶替 —— 0 看着像个正常读数。
            # ⚠️ **邻居档不写这一行**（2026-10-17 操作员裁定）：那条路线根本不看
            #    台面尺寸、也没有"上游报的尺寸"这回事（托盘没被检测到）。留着它
            #    只会让人以为这个数是这一帧的判据之一。
            if not is_neighbor:
                hud.append(_size_hud_line(self._size, self._size_from_upstream))

            if error is not None and getattr(error, "neighbor_quads", None):
                # ---- 邻居路线：**另一套画法**（2026-10-17）----
                # 与托盘路线分家的理由见 `render_neighbor_overlay` 的 docstring：
                # 那张图要回答的是"我到位了没有、邻居认对了没有"，几何形状也不同
                # （无参考边、横向的目标是个点）。**不设探针、不做自检** ——
                # 可视化是给人看的（操作员裁定），不许因为自检拦图或耗时。
                canvas = render_neighbor_overlay(
                    color, error=error, neighbors=error.neighbor_quads,
                    hud_lines=hud)
                probes = []
                problems = []
            else:
                canvas, probes = render_servo_overlay(
                    color, error=error, T_cam_pallet=T_cam_pallet,
                    size_mm=self._size, K=self._K,
                    D=self._D if self._use_distortion else None,
                    ref_edges=ref_edges, box_aabb_uv=box_uv, box_quad_uv=box_quad,
                    hud_lines=hud, ref_edge_labels=ref_edges)
                problems = numeric_self_check(canvas, probes)
            if problems:
                self._n_overlay_selfcheck_fail += 1
                shown = problems[:3]
                # ⚠️ **去重键不能是这条文案**：`problems` 里每条都带探针像素坐标
                # （`render.py:603` 的 `f"{name}: 探针落在画布外 ({ui}, {vi})…"`），
                # 托盘一动 `(ui, vi)` 每帧都变 —— 键取每条"坐标之前的那一段"
                # （探针名 + 失败种类），不带读数。文案照抄原文（坐标是现场读数）。
                self._warn_overlay(
                    "叠加图没通过数值自检，**这一帧不发图**（多半是画图代码的"
                    "问题，不是数据）：" + "; ".join(shown),
                    key="; ".join(_overlay_dedup_key(p) for p in shown))
                self._log_overlay_counts("自检不过")
                return

            import rospy
            from sensor_msgs.msg import Image as _Image
            out = _Image()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self._camera_frame
            out.height, out.width = canvas.shape[:2]
            out.encoding = "bgr8"
            out.is_bigendian = 0
            out.step = int(canvas.shape[1] * 3)
            out.data = np.ascontiguousarray(canvas).tobytes()
            self._overlay_pub.publish(out)
            self._n_overlay += 1
        except Exception as exc:                         # noqa: BLE001
            self._warn_overlay(f"发叠加图失败：{exc}")

    def _log_overlay_counts(self, why: str) -> None:
        """**发不出图**时，把三个计数器按节流打一条 INFO —— "图发不出去"要可观察。

        计数器从前**只写不读**：一个发图链路长期不出图时，日志里除了开头那条
        WARNING 之外**什么都没有**，操作员无从判断"到底发出去过没有"。

        ⚠️ **级别是 INFO，不是 `_log_throttled`（DEBUG）**：随仓库提供的
        `config/log_config.yaml` 全局级别就是 INFO，打 DEBUG 等于没打 —— 而这条
        日志的全部目的就是"在默认配置下看得见"。

        ⚠️ **只在发不出去的两条路上打，不在出图成功时打**：正常出图不需要每 3 秒
        一条计数器（那正是本项目最忌的"噪声淹掉现场"），而"发不出去"恰恰是最需要
        有人说话的时候。三个数一起报是因为"发图 0 张"本身分不出是**底图没来**还是
        **自检不过** —— 那是两条完全不同的排查路，而两个计数器正好各指一条。

        节流口径沿用本文件既有的 `self._ticks % self._log_every_n`。
        """
        if self._ticks % self._log_every_n:
            return
        logger.info(
            "托盘伺服可视化（%s）：发图 %d 张、因无底图跳过 %d 次、自检不过 %d 次 —— "
            "话题 %s。⚠️ 长期一条消息都没有时，先看这三个数，再怀疑 rqt / 话题名",
            why, self._n_overlay, self._n_overlay_skip,
            self._n_overlay_selfcheck_fail, self._overlay_out or "（已关闭）")

    def _warn_overlay(self, message: str, key: Optional[str] = None) -> None:
        """按**稳定的键**去重，并且**同一原因每 `_OVERLAY_REMIND_TICKS` tick 再喊一次**。

        ⚠️ **键不能取整条文案**（与 `_check_liveness` 是同一类病）：
        `numeric_self_check` 报出来的问题里嵌着**探针像素坐标**，托盘一动这些数
        **每帧都变** —— 按原文去重于是恒不相等，退化成每次发图都刷一条（5 Hz）。
        "数据相关的自检失败"（比如托盘临到画面边缘）正会让坐标每帧漂，那时就是
        5 Hz 刷屏。所以键取**坐标之前的那一段**（探针名 + 失败种类）。

        `message` **照抄原文**：坐标是现场读数，要留给人看；被压掉的只是**重复**。
        `key=None`（自由文案，比如异常的 `str(exc)`）时退回"文案里坐标之前的那
        一段"；没有括号就是整条文案，与从前一样。

        ★ **周期提醒不能省**（与 `NodePalletObs._warn_valid_false` 同一条纪律）：
        只在"键变了"时喊，会让一个**长期不过**的自检在操作员眼前**彻底安静** ——
        而那正是这条路的主场（托盘贴画面边缘、探针被别的图层文字盖住）。操作员
        的第一反应会是"rqt / 话题名不对"，可这条链路的**唯一**人工判读手段就是
        这个话题。检测器停摆专门加了 `live_timeout_s` 防"静默"，可视化这条路
        同样不能只有"开头一条 WARNING"。

        周期按 `_last_overlay_warn_tick`（**上次喊这个键的 tick**）算，不是全局
        相位：全局相位下"首次喊发生在哪个 tick"会决定相位，键恰好在那时变化时
        首次喊与周期提醒会在**同一 tick** 触发、后者把前者吞掉 —— 这条与
        `NodePalletObs._warn_valid_false` 的处置逐字同源。
        """
        if key is None:
            key = _overlay_dedup_key(message)
        if key != self._last_overlay_warn:
            self._last_overlay_warn = key
            self._last_overlay_warn_tick = self._ticks
            logger.warning("NodePalletServo 可视化 %s", message)
            return
        if self._ticks - self._last_overlay_warn_tick >= _OVERLAY_REMIND_TICKS:
            self._last_overlay_warn_tick = self._ticks
            logger.warning(
                "NodePalletServo 可视化 **仍在**报同一个问题（每 %d tick 提醒一次，"
                "免得长期不过时彻底安静）：%s", _OVERLAY_REMIND_TICKS, message)

    def _maybe_dump(self, error, T_cam_base, src, pose, box, seen, reason=None,
                    *, slot, neighbor_diag=None):
        """按 `dump_on` 决定要不要把这一帧的**完整原始输入**落盘。

        **整段包在 try 里：dump 是诊断设施，它不该有能力把节点弄死。**

        这条路径上要把上游给的对象**重新解一遍**（`pose.to_list()`、
        `pose6d_to_matrix(pose)`、`getattr(box, f)`），而上游给的东西**可能不是
        我们的类型**：`latest_box_obs` 写成普通 dict（离线工具那份 JSON 正是这个
        形状）、`latest_pallet` 写成 6 个数的 list，都会让 `on_execute` 抛异常 ——
        技能层接住它（`skill_base.py:40-44`）判成"这一帧被拒"，而**默认的
        `dump_on` 里就有 `"reject"`**，于是这里拿同一批坏对象再解一次、再抛一次。
        区别在于：这一次是在技能层那个 `try` **之外**，异常会一路抛穿 `update()`，
        而 py_trees **不接** `update()` 抛出的异常 → **整棵树连每帧的日志一起没**。
        （兄弟节点 `node_inject_servo_input` 早就立了这条纪律：坏输入只许是
        FAILURE + feedback，不许抛。这里对齐。）

        失败只记 WARNING（按原文去重，不是每帧一条），**绝不抛**；这一帧的伺服
        结果不受影响 —— 少的只是一份诊断文件。

        `slot` 是调用方在 `update()` 开头拍下的槽位快照：dump 要用**这一帧**
        那一组参考边，而 `self._active_slot` 可能已经被服务回调改掉了。
        **keyword-only 必填**：默认 `None` 会让新加的调用点静默写出
        `"active_slot": null`，而 dump 的契约是"可重跑"（`replay_dump` 会对它
        `int()` 直接崩）—— 漏传就该在调用点当场 `TypeError`。

        `pose` **可以是 `None`**（邻居档就是"没有托盘位姿"那一档）：那时
        `T_cam_pallet` 与 `pallet_pose` 都写 `null`，其余字段照写 ——
        **不猜一个位姿出来**（单位阵拼出来的 `T_cam_pallet` 在 dump 里看着
        像个正常读数，而它会把离线重跑引到另一组数上去）。

        `neighbor_diag` 是**邻居路线**独有的那一块（`NeighborError.diag`）：
        它认了谁当上/侧邻居、`t1`/`t2` 各是多少、用的是哪两条边。托盘路线传
        `None`（它的 `ref_edges` + `pallet_pose` 已经够重跑了）。
        """
        try:
            warn = [] if error is None else error.warn
            if not self._dump.should_write(rejected=error is None, warn=warn):
                return
            T_cam_pallet = (None if pose is None
                            else T_cam_base @ pose6d_to_matrix(pose))
            payload = {
                "reason": reason or ("warn: " + "; ".join(warn)),
                "T_cam_pallet": T_cam_pallet,
                "T_cam_base": T_cam_base,
                "t_cam_base_src": src,
                "pallet_pose": None if pose is None else pose.to_list(),
                "box": {f: getattr(box, f) for f in
                        ("u1", "v1", "u2", "v2", "quad", "label", "confidence",
                         "stamp")},
                "ref_edges": [list(s) for s in self._slots],
                "active_slot": slot,
                "pallet_size_mm": list(self._size),
                "K": self._K,
                "D": self._D,
                "use_distortion": self._use_distortion,
                "image_size": self._image_size,
                "pallet_version": seen[0],
                "box_version": seen[1],
                "stamp": time.time(),
            }
            # ★ **邻居路线的诊断量单独一块**（2026-10-17 加）。托盘路线的 dump 里
            #   有 `ref_edges` / `pallet_pose` 可以重跑，邻居路线要重跑还得知道
            #   **它认了谁当邻居、用的是哪条边** —— 不落盘的话，"e_bottom 看着
            #   不对"就只能在现场盯着图猜（真机第一轮就只能猜）。
            #   `diag` 里已经有 `t1_bottom` / `t2_bottom` / `bottom_px` / `top_px` /
            #   `u_far` 与逐候选的几何，够把这一帧完整复现。
            if neighbor_diag is not None:
                payload["neighbor_diag"] = neighbor_diag
        except Exception as exc:                 # noqa: BLE001
            self._dump_payload_fails += 1
            if (self._dump_payload_fails == 1
                    or self._dump_payload_fails % DumpWriter.FAIL_REPORT_EVERY == 0):
                logger.warning(
                    "托盘伺服这一帧的 dump 有个字段取不出来（%s: %s）—— 跳过这一帧"
                    "的 dump，伺服结果不受影响。**上游给的类型不对**（比如 "
                    "latest_box_obs 是 dict、latest_pallet 是 list）时每帧都会这样，"
                    "本次运行已第 %d 次", type(exc).__name__, exc,
                    self._dump_payload_fails)
            else:
                logger.debug("托盘伺服 dump 取字段又失败一次（第 %d 次）：%s",
                             self._dump_payload_fails, exc)
            return
        try:
            self._dump.write(payload)
        except Exception as exc:                 # noqa: BLE001
            # `write()` 的契约是"绝不抛"，这里是**第二道闸**（上面那段的同一个
            # 理由，只是这次连 payload 都构造出来了）。写这条兜底是因为这层
            # 保险本来就在"谁都不许把节点弄死"的第一现场：`write()` 里任何一个
            # 疏漏（比如改常量时漏改一处引用）都会直接掀翻整棵树。
            logger.warning("托盘伺服 dump 没落盘（%s: %s）—— 这一帧照常输出，"
                           "伺服结果不受影响", type(exc).__name__, exc)

    def terminate(self, new_status):
        """收订阅。**可视化那一路也要收** —— 不收就是换树之后还在占着图像话题。"""
        for sub in (self._color_sub,):
            if sub is not None:
                try:
                    sub.unregister()
                except Exception:                        # noqa: BLE001
                    pass
        self._color_sub = None
        self._overlay_pub = None


def _int_at_least(value, minimum, default, name):
    """整数参数（`window` / `pair_cache`）：解释不了退回默认值，太小就**钳到下限**。

    两者都**不抛**（与 `_float_or` 同一套理由：一个旋钮写错不该把节点构造搞崩），
    但都**点名 WARNING** —— 参数是从 ROS rosparam / 场景 JSON 进来的，静默改语义
    就是本项目最忌的那类错（addendum A3：`agg` 拼错在纯函数层静默退回 mean，
    可见性归节点层负责，同一条纪律）。

    钳位而不是退默认值：`PalletFrameWindow` 自己也是 `max(1, int(window))`，
    `window: 0` 的意图显然是"不平滑"（=1），退成默认值会让**语义反向**。
    """
    try:
        n = int(value)
    except (TypeError, ValueError):
        logger.warning("%s 不是整数（%r）—— 按默认值 %d 处理", name, value, default)
        return int(default)
    if n < minimum:
        logger.warning("%s 至少要 %d（实际 %r）—— 按 %d 处理", name, minimum,
                       value, minimum)
        return int(minimum)
    return n


def _positive_float_param(value, default, name):
    """正有限浮点参数（`max_dt_s` / `live_timeout_s`）：解释不了/不合法退回默认值。

    `max_dt_s` **必须**是正有限值：`nearest_pair` 会显式拒（`pair_threshold`），
    但那是"配对时才发现"，而参数层的错该在**启动时**就说出来 —— 而且默认值
    必须是有意义的（0/NaN/inf 会让门禁静默失效或全拒，见 `nearest_pair` 的
    docstring）。addendum A3。
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        logger.warning("%s 不是数字（%r）—— 按默认值 %s 处理", name, value, default)
        return float(default)
    if not math.isfinite(v) or v <= 0.0:
        logger.warning(
            "%s 必须是正有限值（实际 %r）—— 按默认值 %s 处理。"
            "0/负数会让**任何**有微小时间差的一对全被拒（等于把功能关掉），"
            "NaN/inf 会让这道门禁**静默失效**（dt > nan 恒为 False，任何一对都放行）",
            name, value, default)
        return float(default)
    return v


def _nonnegative_float_param(value, default, name):
    """**非负**浮点参数（`live_timeout_s`）：坏值退回默认值并**点名**。

    与 `_float_or` 的区别只在**负数**：`_float_or` 把负数静默钳成 0，而 0 在
    `live_timeout_s` 上是"**不查**" —— `-1` 这种手滑于是会**静默废掉唯一的
    检测器存活告警**（正是本项目最忌的那类：配置写错、行为反了、一声不响）。
    所以这里负数与"解释不了"同等对待：点名 + 退回默认值（A3）。

    0 本身照收不吵：它是文档里写明的"不查"。默认值必须是正数，否则这条纪律
    自己就把告警关掉了。

    ⚠️ **调用方必须把内联默认值写进 `self.params.get(键, 默认)`**（与同批的
    `max_dt_s` / `pair_cache` / `window` 同一写法）。本函数**不认** `None`：它只
    认数字，`None` 走的是"不是数字"那一支。键不存在时 `get()` 返回的正是
    `None`，少写内联默认值 = **每次建树一条假 WARNING**（这是修 M-3 那一轮真
    踩过的回归）。与 `_float_or` 的差别不是疏忽：那一路的参数（`stale_s` /
    `dump_min_interval_sec` / `overlay_period_s`）本来就把 `None` / `""` 当
    "没给"静默退默认，而本函数的纪律是"**给了个解释不了的值**才点名"——
    `get()` 带不带默认值，决定了"没给"会不会变成"给了个 None"。
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        logger.warning("%s 不是数字（%r）—— 按默认值 %s 处理", name, value, default)
        return float(default)
    if not math.isfinite(v) or v < 0.0:
        logger.warning(
            "%s 必须是非负有限值（实际 %r）—— 按默认值 %s 处理。"
            "**0 就是'不查'**，负值若也按 0 处理就等于静默关掉这条告警",
            name, value, default)
        return float(default)
    return v


def _matrix_or_none(value, shape, name):
    """把参数里的嵌套 list 拧成 ndarray；`value` 为空返回 `(None, None)`。

    `shape=None` 表示不校验具体形状（畸变系数是变长的）。
    """
    if value is None or (isinstance(value, (list, tuple)) and len(value) == 0):
        return None, None
    try:
        arr = np.asarray(value, np.float64)
    except (TypeError, ValueError) as exc:
        return None, f"{name} 不是数值：{exc}"
    if shape is not None and arr.shape != shape:
        return None, f"{name} 形状必须是 {shape}，实际 {arr.shape}"
    if not np.all(np.isfinite(arr)):
        return None, f"{name} 里有 NaN/inf"
    return arr, None


def _normalize_slots(raw) -> List[List[str]]:
    """`ref_edges` 归一成 `[[边, 边], ...]`。

    两种形状都认：

      ["y=0", "x=W"]                   → 一组（单箱位，等价于老写法）
      [["y=0","x=W"], ["y=H","x=W"]]   → N 组，服务里第 N 个箱子用第 N 组

    ⚠️ **不在这里校验合法性** —— 校验（写法 + 不平行）统一在 `initialise()` 里
    用 `parse_ref_edges` 做。这里只做形状归一，认不出来的原样塞进去，
    让校验去报（它报得出来，因为 `parse_edge_spec` 对任何输入都给得出答案）。
    """
    if not isinstance(raw, (list, tuple)) or not raw:
        return [["y=0", "x=W"]]
    first = raw[0]
    if isinstance(first, (list, tuple)):
        return [list(slot) for slot in raw]
    return [list(raw)]


# 邻居路线只认这两侧（画面口径），与 `neighbor.NeighborSlot` 的白名单一致。
# ⚠️ **在这里再写一份而不是从 `neighbor` 里 import**：这一份要在**启动时**把
#    配置错误说清楚（"第 N 档的 side 是 xxx，只认 left/right/none"），而
#    `NeighborSlot` 那边抛的 `ValueError` 里没有"第几档"。两处的**取值集合**
#    必须一致 —— 有单测同时钉住两边（`test_servo_slots_side_whitelist_matches`）。
_NEIGHBOR_SIDES = ("left", "right", "none")
_SERVO_MODES = ("pallet", "neighbor")
#: `servo_slots` 每项可选的输出偏置，**按下标顺序**进归一化后的元组。
_BIAS_KEYS = ("e_right_bias_px", "e_bottom_bias_px")


def _parse_slot_bias(item, key: str, index: int):
    """`servo_slots` 第 `index` 项的数值偏置 → `(值, 错误说明)`。缺省 0。"""
    raw = item.get(key, 0.0)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None, f"servo_slots 第 {index} 项的 {key}={raw!r} 不是数值"
    if not math.isfinite(value):
        return None, f"servo_slots 第 {index} 项的 {key}={raw!r} 不是有限数"
    return value, None


def _normalize_servo_slots(raw):
    """`servo_slots` → `([(mode, side, e_right_bias_px), ...], 错误说明)`。

    形状（见 `board.json` 的 `servo_slots`）：:

        [{"slot": 1, "mode": "pallet"},
         {"slot": 3, "mode": "neighbor", "side": "right",
          "e_right_bias_px": -8.0}, ...]

    * 返回的列表**按下标 0 数**，与 `self._slots`（`ref_edges`）**同一个下标**
      —— 服务里 call 第 N 个箱子用的是两者的第 N-1 项。这一条与 `ref_edges`
      的"服务从 1 数、数组从 0 数"是同一句纪律，**别改**。
    * **缺省 / 空数组 → 全部 `("pallet", "none")`**（行为与加这个键之前一致）。
      长度按 `ref_edges` 对齐由调用方做 —— 这里不知道那边有几组。
    * `mode` 只认 `pallet` / `neighbor`，`side` 只认 `left` / `right` / `none`，
      **别的值一律返回错误说明**（第二项非 None）：这属于**启动时的配置错误** ——
      静默按 pallet 跑的话，本该走邻居的那几档会拿"托盘看不见"的位姿去算，
      而且它**照样算得出三个数**（没有任何报错），是最难查的那一类。
    * `side` 只有 `mode == "neighbor"` 时才有意义，但**每个表项都照样校验它**
      （含 `pallet` 档）：板子上把两套的字段填全是一种常见写法，而"这一项
      拼错了"与"这一项用不上"是两件事 —— 只校验用得上的那几项，会让一个拼错的
      `side` 静静躺在那儿，等谁把那一档改成 `neighbor` 时才发作。
    * `e_right_bias_px` / `e_bottom_bias_px`（可选，默认 0）是这个档位对**最终
      `e_right` / `e_bottom`** 的固定偏置：`输出 = 原始值 + bias`。服务与话题
      拿到的就是这个加过偏置的量（控制器会把它驱到 0，所以 `e_right_bias_px =
      -8` 的含义是"停在实际差 +8px 的位置"）。
      非法值（非数值 / NaN / inf）返回错误说明 —— 静默按 0 跑会让偏置"配了没
      生效"，而现场只能从机械臂停的位置倒推。

    **`slot` 号本身不在这里校验**：表项的顺序就是档位，像 `ref_edges` 那样按
    出现顺序取。写了 `slot` 但不按顺序排的话，两边会错位 —— 所以这里**按
    `slot` 排序**（都写了就按它排；一个没写才按原顺序），错位这件事不许靠运气。
    """
    if not isinstance(raw, (list, tuple)) or not raw:
        return [("pallet", "none", 0.0, 0.0)], None
    rows = []
    for index, item in enumerate(raw, start=1):
        if not isinstance(item, dict):
            return [], (f"servo_slots 第 {index} 项不是对象（{item!r}）—— "
                        f"每项形如 "
                        f'{{"slot": 1, "mode": "pallet"}}')
        mode = str(item.get("mode", "pallet")).strip()
        if mode not in _SERVO_MODES:
            return [], (f"servo_slots 第 {index} 项的 mode={item.get('mode')!r} "
                        f"不认识 —— 只认 {'/'.join(_SERVO_MODES)}")
        side = item.get("side", "none")
        # ⚠️ **大小写不宽容**（与 `NeighborSlot.__post_init__` 的白名单同一口径）：
        #    那边 `'Right'` 是构造即抛的，这边放宽的话两处判据就分叉了 ——
        #    板子上写 `"Right"` 会在这里静默变成 `right`，而所有人都会以为
        #    两个校验器说的是同一件事。**更多一层宽容 = 少一层保护。**
        side = "none" if side is None else str(side).strip()
        if side not in _NEIGHBOR_SIDES:
            return [], (f"servo_slots 第 {index} 项的 side={item.get('side')!r} "
                        f"不认识 —— 只认 {'/'.join(_NEIGHBOR_SIDES)}"
                        f"（画面口径）")
        slot = item.get("slot", index)
        try:
            slot = int(slot)
        except (TypeError, ValueError):
            return [], f"servo_slots 第 {index} 项的 slot={slot!r} 不是整数"
        bias_values = []
        for key in _BIAS_KEYS:
            value, bias_err = _parse_slot_bias(item, key, index)
            if bias_err:
                return [], bias_err
            bias_values.append(value)
        rows.append((slot, mode, side, *bias_values))
    # 有 `slot` 就按它排（重复的排完还是相邻，长度对得上就行）；没有就按原顺序。
    rows.sort(key=lambda r: r[0])
    return [(mode, side, bias_r, bias_b) for _slot, mode, side, bias_r, bias_b
            in rows], None


def _slot_mode_of(slot_modes, index: int) -> Tuple[str, str]:
    """第 `index` 档（**0 数**）的 `(mode, side)`；表不够长时按 `pallet` 兜底。

    兜底而不是报错：`servo_slots` 比 `ref_edges` 短是**合法**配置（"后面几档
    都是老路线"），报文里那句"全部按 pallet"正是这个意思。

    ⚠️ **多出来的项不再被截掉**（2026-10-17 改）：`servo_slots` 比 `ref_edges`
    长、且多出来那几档都是 `neighbor` —— 那是**合法**的（邻居档不读参考边）。
    只有"多出来那档是 `pallet`"才是配置错误，由 `initialise()` 逐档判并报出来。
    从前的"截掉 + WARNING"会让末档**永远切不过去**（真机实测）。

    表项是 `_normalize_servo_slots` 给的三元组 `(mode, side, bias)`；这里只取
    前两项（偏置由 `_slot_bias_of` 读）。手工传二元组也兼容。
    """
    if 0 <= index < len(slot_modes):
        return slot_modes[index][0], slot_modes[index][1]
    return "pallet", "none"


def _slot_bias_of(slot_modes, index: int, axis: str = "right") -> float:
    """第 `index` 档（**0 数**）的偏置；未配置 / 越界 = 0。

    `axis` ∈ `{"right", "bottom"}`，对应 `e_right_bias_px` / `e_bottom_bias_px`
    （归一化元组的下标 2 / 3）。

    越界也返回 0（与 `_slot_mode_of` 的兜底同一口径）：`servo_slots` 短是
    合法配置，缺省档位不该凭空多出一个偏置。
    """
    if axis not in ("right", "bottom"):
        raise ValueError(f"axis={axis!r} 只认 right/bottom")
    offset = 2 if axis == "right" else 3
    if 0 <= index < len(slot_modes) and len(slot_modes[index]) > offset:
        return float(slot_modes[index][offset])
    return 0.0


def _servo_needs_inputs(mode: str, pose, box) -> bool:
    """这一帧是不是"还在等输入"（应按 `RUNNING` 早退）。

    ★ **按路线区分，这是邻居接入的那个卡点**（`neighbor_merge_spec.md` §8.1）：

    * `pallet` —— 托盘位姿与箱子观测**都**要有：托盘路线的三个数全部由"托盘台面
      参考边的投影"给出，缺位姿连参考边都投不出来。与改动前**逐字一致**。
    * `neighbor` —— **只要求箱子观测**。邻居路线的基准是**别的箱子的边**，
      根本不用托盘位姿；而"托盘被手里这个箱子挡死 → `latest_pallet` 是 `None`"
      正是它被做出来的**那个场景**。照老判据早退的话，邻居档永远走不到分流。
    """
    if box is None:
        return True
    return pose is None if mode == "pallet" else False


def _servo_wait_message(mode: str, pose, box) -> str:
    """「等输入」那句人话。`pallet` 档的文案与改动前**逐字相同**。

    ⚠️ 邻居档**不能**照抄那句 —— 它会写"托盘 缺"，而邻居路线**不需要**托盘，
    这句话落到 `feedback_message` / 话题 / 叠加图 HUD 上会让人去查托盘检测器
    （那条路本来就没坏）。邻居档把"缺的是候选列表"写清楚。
    """
    if mode == "pallet":
        return (f"等输入（托盘 {'有' if pose is not None else '缺'}，"
                f"箱子 {'有' if box is not None else '缺'}）")
    if box is None:
        return "等输入（邻居路线：箱子 缺）"
    return "等输入（邻居路线：候选 缺，箱子 有）"


def _neighbor_servo_error(res, held):
    """邻居路线的 `NeighborError` → 托盘路线那套 `ServoError`（**同名同单位**）。

    托盘路线与邻居路线是两套算法（`servo_error` / `neighbor_servo`），但**对外
    只有一种输出结构**：同一个 `ServoError`、同一个 `_publish_error`、同一条
    叠加图链路。这个函数就是那道转换 —— 三个被控量直接搬，诊断量按下面的口径
    补，缺的一律**留空数组**而不是编一个数出来。

    诊断量怎么填（`neighbor_merge_spec.md` §8.2(4)）：

    * `box_bottom_px` / `box_right_px` —— **手里箱自己的**底边与右边（`box_edges`
      的同一套定义，叠加图上的绿线）。
    * `ref_bottom_px` —— **上邻居的底边**（它朝手里箱的那条）。
    * `ref_right_px` —— **侧邻居的近边**（它朝手里箱的那条）；这一档没有侧邻居
      （`side="none"` 或那一侧没检出）时是**空数组 `np.zeros((0, 2))`** ——
      `render` 跳过空数组（见那里的 `_is_blank`），图上的黄线因此只有一条。
    * `pallet_center_px` —— **上邻居的中心**。`render` 拿它判参考边的内法向
      （哪个方向是"内部"）；邻居路线上"内部"就是手里箱那一侧。
      ⚠️ `render` 现在并不读这个字段（它只在 `error` 缺席时自己投影），填它是
      为了结构完整与离线工具好用。
    * `bottom_inward_px` / `right_inward_px` —— **从参考边指向手里箱**的单位
      向量（`inward_normal` 的同一套：拿手里箱中心去点乘，`ref_right` 空时给空）。
    * `*_foot_px` —— 手里箱那条边中点在参考边所在直线上的垂足（`edge_offset`
      的同一套算法），叠加图的红色垂距线要用。参考边不存在时给空。

    ⚠️ **三个被控量是这套结构里唯一承诺"与托盘路线同口径"的东西**，其余都是
    诊断量。所以 `warn` 里必须带 `source=neighbor`：图上那两个数与托盘路线算出来
    的不是一回事，现场判读时要能一眼分出来。
    """
    corners = box_corners(held)
    if isinstance(corners, Reject):
        # 走到这里说明 `neighbor_servo` 已经拿同一个 `held` 取过四角且成功了
        # （它先判 `neighbor_no_held`），所以这一支实际到不了 —— 留着只是让
        # "取不出四角"这件事有个不抛的出口，**不猜一个框出来**。
        edges = {"bottom": np.zeros((0, 2)), "right": np.zeros((0, 2))}
    else:
        edges = box_edges(corners)

    held_center = np.asarray(res.held_quad, np.float64).reshape(4, 2).mean(axis=0)
    # 上邻居一定在（成功路径上 `up` 非空）；它的"底边" = 朝手里箱的那条边。
    # ⚠️ 直接用 `neighbor.py` 里那个函数而不是再写一份：**同一条边必须只有一个
    #    定义**（两份实现一旦分叉，图上的黄线与算法用的参照边就不是同一条了）。
    up_bottom = neighbor_algo._edge_facing(res.up_quad, res.held_quad)
    up_center = np.asarray(res.up_quad, np.float64).reshape(4, 2).mean(axis=0)
    bottom_inward = inward_normal(up_bottom, held_center)
    bottom_foot = edge_offset(up_bottom, edges["bottom"], bottom_inward).foot_px

    empty2 = np.zeros((0, 2))
    if res.side_quad is not None:
        side_near = neighbor_algo._edge_facing(res.side_quad, res.held_quad)
        right_inward = inward_normal(side_near, held_center)
        right_foot = edge_offset(side_near, edges["right"],
                                 right_inward).foot_px
    else:
        side_near, right_inward, right_foot = empty2, empty2, np.zeros((0,))
    return ServoError(
        e_bottom_px=res.e_bottom_px,
        e_right_px=res.e_right_px,
        theta_rad=res.theta_rad,
        ref_bottom_px=up_bottom,
        ref_right_px=side_near,
        box_bottom_px=edges["bottom"],
        box_right_px=edges["right"],
        pallet_center_px=up_center,
        bottom_foot_px=bottom_foot,
        right_foot_px=right_foot,
        bottom_inward_px=bottom_inward,
        right_inward_px=right_inward,
        box_source="neighbor",
        # ★ **邻居的两个框整圈** —— 只给叠加图用（`render_servo_overlay` 的
        #   `neighbor_quads`）。没有它，图上只有"参与伺服的那两条边"，
        #   **判不出认的是哪两个箱子**（上邻居认成侧邻居、或者压根认到别的箱子
        #   上，黄线照样在那里）。挂在这个对象上而不是另开一条传递路径：它随
        #   `error` 一起流到出图那一段，零新接口。
        #   ⚠️ 托盘路线的 `ServoError` **没有**这两个字段 → 出图那边用
        #   `getattr(error, "neighbor_quads", None)` 取，取不到就是 `None`。
        neighbor_quads=(np.asarray(res.up_quad, np.float64).reshape(4, 2),
                        None if res.side_quad is None
                        else np.asarray(res.side_quad, np.float64).reshape(4, 2)),
        # 出图三件套（见 `render_neighbor_overlay` 的 docstring）：
        #   held_quad_px   手里箱四角（灰框）——「现在在哪」
        #   target_bottom  上邻居朝手里箱的那条边（黄线）——「底边该压到哪」
        #   target_center  上邻居中心（黄十字）——「中心该对齐到哪」
        # ⚠️ **一律由 `neighbor.py` 交出来，节点不自己推**：曾经在这里用
        #    `box_edges()` 画"手里箱底边"，而算法用的是 `_frame_of()` 推出来的
        #    那条 —— 相机翻 180° 时两者不是同一条，图上出现"一个选上一个选下"。
        held_quad_px=np.asarray(res.held_quad, np.float64).reshape(4, 2),
        held_bottom_px=np.asarray(res.held_bottom_px, np.float64).reshape(2, 2),
        held_side_px=(None if res.held_side_px is None
                      else np.asarray(res.held_side_px, np.float64).reshape(2, 2)),
        # 底边黄线的平移方向用 `u_bottom`（参考边法向的加权平均）—— **不是
        # `u_far`**：`u_far` 带横向/透视偏移（实测与竖直差 27.5°），沿它平移会
        # 把黄线斜着推出去、且垂距归不了零（2026-10-17 真机踩）。
        u_bottom=np.asarray(res.u_bottom, np.float64).reshape(2),
        u_side_shift=np.asarray(res.u_side_shift, np.float64).reshape(2),
        u_far=np.asarray(res.diag['u_far'], np.float64).reshape(2),
        u_side=np.asarray(res.diag['u_side'], np.float64).reshape(2),
        warn=["source=neighbor"] + list(res.warn))
    return [list(raw)]


def _float_or(value, default, name):
    """可选浮点参数：没给就用默认值；解释不了也退回默认值并记 WARNING（不抛）。

    与 `parse_bool_param` 的"解释不了就抛"不同，这里**不抛**：`dump_min_interval_sec`
    只是个节流旋钮，为它把节点构造搞崩（进而掀翻整棵树）不值得。负数一律按 0
    处理 —— 那个旋钮的 0 就是"不节流"。
    """
    if value is None or value == "":
        return float(default)
    try:
        return max(0.0, float(value))
    except (TypeError, ValueError):
        logger.warning("%s 不是数字（%r），按默认值 %s 处理", name, value, default)
        return float(default)


def _size_hud_line(size, from_upstream) -> str:
    """叠加图 HUD 上那行台面尺寸。**尺寸没定下来时照实写 `（未定）`**。

    拿 0 顶替会让图上那行字看着像个正常读数（`size=0x0`），而现场要判断的
    恰恰是"这一帧到底用哪一份尺寸" —— 配置错误那条路本来就走不到算出三个数，
    但叠加图现在也画那种帧，HUD 得说得清。
    """
    if size is None:
        return "size=（未定）"
    return (f"size={float(size[0]):.0f}x{float(size[1]):.0f} "
            f"from_upstream={int(bool(from_upstream))}")


def _box_quad_of(box):
    """黑板上的箱子观测 → 契约序四角 `[[u, v], ...]`；拿不到返回 `None`。

    **不抛**：黑板上那个键可能是普通 dict / list（离线工具写的就是这个形状），
    而本函数跑在**出图**这条诊断路上 —— 解析不了就不画那一层，绝不能因为
    可视化把节点弄死。

    ⚠️ **不重排、不排序**：`quad` 的契约序就是"右下 → 左下 → 左上 → 右上"，
    与 `render_servo_overlay` 要的一致。顺序错了边就取错了，而图上看着仍是个
    四边形 —— 那正是要避免的静默错。
    """
    if box is None:
        return None
    try:
        quad = getattr(box, "quad", None)
        if quad is None and isinstance(box, dict):
            quad = box.get("quad")
        if quad is None:
            return None
        q = [[float(p[0]), float(p[1])] for p in quad]
    except Exception:                                     # noqa: BLE001
        return None
    return q if len(q) == 4 else None


def _overlay_dedup_key(message: str) -> str:
    """可视化告警的**稳定去重键**：砍掉文案里的逐帧读数。

    `numeric_self_check` 的每条问题都是
    `f"{name}: 探针落在画布外 ({ui}, {vi})…"` / `f"{name}: ({ui}, {vi}) 附近…"`
    —— **坐标是逐帧在变的量**（托盘一动就变），拿整条文案当去重键等于没去重。
    所以键取**第一个 `(` 之前的那一段**：探针名 + （落画布外那支的）失败种类，
    两者都只随"哪条探针、怎么坏的"变，不随读数变。

    没写坐标的文案（异常那一路）原样返回 —— 与"按原文去重"一致。

    注意用的是**半角** `(`：自检文案里的坐标对是半角，而包装句里的
    "（多半是画图代码的问题）"是全角，不会被切掉。
    """
    return message.partition("(")[0]


# 停摆文案里**每 tick 都在长**的那一段：`已经 8.2s`。
# 与 `_overlay_dedup_key` 是同一类病的第二个实例 —— 那一个砍坐标，这一个砍秒数。
_REJECT_SECONDS_RE = re.compile(r"已经\s*[\d.]+\s*s")


def _reject_dedup_key(reject: str) -> str:
    """话题无效帧的**稳定去重键**：砍掉文案里逐帧在变的读数。

    停摆那条 `reject` 里嵌着 `已经 8.2s`，而那个数**每 tick 都在长** —— 拿原文
    做键恒不相等，去重形同虚设，实测真机上是**每个 tick 一条**（10 Hz），而设计
    要的是「内容变了立刻说、没变每秒一次」。现场表现是消费者被一样（只差秒数）
    的消息刷屏。

    砍掉的只有秒数：`托盘` / `箱子` / `没有**新的观测**` 这些**稳定**部分全部留下，
    所以"哪一路停了"仍然能区分（两侧同时停 vs 只停托盘 → 两个不同的键）。

    ⚠️ **只用于去重键**，`msg.reject` 发的仍是原文 —— 秒数是现场读数，要留给人看。
    """
    return _REJECT_SECONDS_RE.sub("已经 <t>s", reject)
