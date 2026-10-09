# -*- coding: utf-8 -*-
"""NodePalletObs：订阅视觉托盘检测话题 → 转 `Pose6D` → 写黑板。

这是 `latest_pallet` 的**第二个真生产者**（第一个是 `NodePalletPose`，
走 AprilTag）。两者**互斥**：同一条链路上只能有一个在写，否则伺服读到哪一份
取决于 tick 顺序，**没有任何报错**。

黑板契约（照抄 NodePercep / NodePalletPose / NodeBoxObs 的写法）
    写  latest_pallet / latest_pallet_version      Pose6D

## 上游话题：`pallet_detection_msgs/PalletDetection`

默认话题 `/pallet/detection`，发布者是**视觉托盘检测器**（本仓库之外）。
契约见 `infrastructure/ros_packages/src/ros_vision/pallet_detection_msgs/`。

⚠️ **`latest_pallet` 的坐标系是相机系**（不是 base_link）。这是本节点与
`NodePalletPose` 的关键区别：后者给的是 base_link 系位姿，要 `NodePalletServo`
乘 `T_cam_base` 换回来；本节点给的就是相机系，伺服那边 `pallet_frame: "camera"`
直接跳过 TF。**配错会让参考边整体偏掉，而三个数照样算得出来。**

## 三道闸门

1. `valid == false` → 一个字都不写（占位值不是观测）
2. `det` 不在 +1 侧 → 不写。**`det` 由本节点自己算**，不采信上游自报
3. 同一条观测只写一次（否则每 tick 重写、版本号一直涨、下游白算）

## `latest_pallet_stamp` 的取值口径（**与 `NodeBoxObs` 不同，是有意的**）

`header.stamp` **取不到**（`None` / 非数）时写 `0.0`；**取到了但非正**
（`0.0` / 负数）时**原样写上游给的那个数** —— 负数**不折成 `0.0`**（"不编造"
的语义是保留上游给的实际值，负数也是上游给的）。两种情况都**不退回本节点的处理
时刻**。编造一个"看起来像时间"的数会让配对侧照单全收 —— 托盘侧用 tick 时刻、
箱子侧用图像时刻，箱子那 ~60ms 的检测耗时整个变成 `pair_dt`：要么超阈值永远配不上，
要么配上一对**不同帧的图像**，两种都不报错。写 `0.0` 则配对侧**显式 reject**
（`pair_stamp_zero`），失败模式一眼可见。理由详见 `parse_pallet_message()` 里
`"stamp"` 那段注释。

## 为什么是"按字段名取"而不是 import 那个消息类

与 `NodeBoxObs` 同一理由：`pallet_detection_msgs` 是 **catkin 包**，要单独放进
ROS 工作区编译。按字段名取则 ① LeTools 不多一条构建依赖；② 换个"形状一样、
包名不一样"的消息照样能用；③ 测试里可以用假的消息对象，不必装 ROS ——
解析函数因此能进 CI。

节点持续返回 RUNNING（源节点口径，与 NodePercep / NodePalletPose / NodeBoxObs 一致）。
"""
from __future__ import annotations

import threading
from typing import Optional

import numpy as np
import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from core.common.transform import matrix_to_pose6d
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.utils.blackboard import (
    bump_version,
    is_dry_run,
    num_or_none,
    read_version,
)
from skills.atomic.perception.pallet_servo.algorithm import (
    handedness_problem,
    parse_bool_param,
    parse_pair_param,
)

logger = get_logger(__name__)

TOPIC_DEFAULT = "/pallet/detection"
MSG_TYPE_DEFAULT = "pallet_detection_msgs/PalletDetection"

# 上游自报的 det 与本节点算出来的差超过这个值就记一次 WARNING（只在日志里留痕，
# 不拦 —— 拦的判据是**本节点算出来的那个**）。
DET_MISMATCH_TOL = 0.1

# 上游自报的 `size_mm` 与本节点配置的 `pallet_size_mm` 的**最大逐维相对差**超过
# 它就记一次 WARNING（同样是只留痕、不拦 —— 几何一律用配置值）。
#
# ## 这个阈值是怎么定的（**不是拍的**）
#
# * **正常情况下差多少**：`.msg` 与设计文档都写明这个字段**不是独立测量** ——
#   按已知尺寸搜索的检测器**必然**报出接近配置的值（自我实现）。所以正常运行
#   时的相对差只反映检测器内部的像素级噪声，量级是**几个百分点**。
# * **要抓的错有多大**：设计点名的现场是 `pallet_size_mm` 少写一位
#   （1200 → 120，相对差 0.91）。而"差一倍"（相对差 0.5）是**量级错的下界** ——
#   阈值必须**严格小于 0.5**，否则 2× 这种最典型的量级错会被放过。
# * **取 0.2 的理由**：夹在"噪声上限（几个百分点）"与"量级错下界（0.5）"之间 ——
#   对 5% 的噪声留了 4 倍余量（真机不会假报），对 2× 的量级错留了 2.5 倍余量。
#   宁可松一点：这条诊断**没有拦阻能力**（几何用的是配置值），它的全部价值就是
#   在"三个数照样算得出来"的时候出个声；假报会把这个声的价值耗掉。
# * **已知抓不到的**：W/H 写反且两个数相近时（1200×1000 写成 1000×1200，
#   逐维相对差 0.17）不会响 —— 那是"两个数各自核对"的另一类检查，不在本条。
#
# 相对差的分母取 `max(|报的|, |配置的|)`：对称（谁作基准结果一样），且某一维
# 接近 0 时不会把相对差放大到无意义的数。
SIZE_MISMATCH_REL_TOL = 0.2


def _size_rel_gap(reported, configured) -> Optional[float]:
    """两个 `[W, H]` 的**最大逐维相对差**；算不出来返回 `None`。

    分母取 `max(|报的|, |配置的|)`，理由见 `SIZE_MISMATCH_REL_TOL` 那段。
    两维都是 0（按契约那是 `valid=false` 的占位值）时该维不判 —— 别把"没数据"
    算成"差 100%"。
    """
    if reported is None or configured is None:
        return None
    try:
        gaps = []
        for a, b in zip(reported, configured):
            a, b = float(a), float(b)
            denom = max(abs(a), abs(b))
            if denom <= 0.0:
                continue
            gaps.append(abs(a - b) / denom)
    except (TypeError, ValueError):
        return None
    return max(gaps) if gaps else None


# --------------------------------------------------------------------------- #
# 解析：**纯函数，不碰 rospy** —— 测试因此能进 CI
# --------------------------------------------------------------------------- #
def _matrix16(raw) -> Optional[np.ndarray]:
    """16 个数 → 4×4，**行主序**。取不出来返回 None。

    行主序 vs 列主序搞错会让位姿整体转置 —— 静默错，所以这里只有一个约定，
    且测试里钉住它。
    """
    if raw is None:
        return None
    try:
        vals = [num_or_none(v) for v in raw]
    except TypeError:
        return None
    if len(vals) != 16 or any(v is None for v in vals):
        return None
    return np.asarray(vals, np.float64).reshape(4, 4)


def parse_pallet_message(msg) -> Optional[dict]:
    """ROS 消息 → `{"T_cam_pallet", "det", "valid", "source", ...}`。

    认不出来返回 `None`（调用方只记 WARNING，不抛、不 FAILURE）。**纯函数**：
    只按字段名取，不 import 任何消息类型。

    `det` **由本函数自己算**（`np.linalg.det` 于旋转块），不采信消息里的
    `det` 字段 —— 那是上游的自报。消息里有这个字段时**交叉核对**，
    差超过 `DET_MISMATCH_TOL` 就在返回值里置 `det_mismatch`，调用方记一次
    WARNING。**判据用的是本地算的那个。**
    """
    if msg is None:
        return None

    T = _matrix16(getattr(msg, "T_cam_pallet", None))
    if T is None:
        # 换个说法也认（契约是我们的，这里只是留条后路）
        for field in ("transform", "T", "matrix"):
            T = _matrix16(getattr(msg, field, None))
            if T is not None:
                break
    if T is None:
        return None
    if not np.all(np.isfinite(T)):
        return None

    det_local = float(np.linalg.det(T[:3, :3]))
    det_raw = getattr(msg, "det", None)
    det_field = num_or_none(det_raw)
    mismatch = (det_field is not None
                and abs(det_field - det_local) > DET_MISMATCH_TOL)

    valid = getattr(msg, "valid", None)

    stamp = 0.0
    header = getattr(msg, "header", None)
    stamp_obj = getattr(header, "stamp", None)
    to_sec = getattr(stamp_obj, "to_sec", None)
    if callable(to_sec):
        stamp = num_or_none(to_sec()) or 0.0
    else:
        stamp = num_or_none(stamp_obj) or 0.0

    size = None
    try:
        size = [num_or_none(v) for v in (getattr(msg, "size_mm", None) or [])]
        size = None if len(size) != 2 or any(v is None for v in size) else size
    except TypeError:
        size = None

    return {
        "T_cam_pallet": T,
        "det": det_local,
        "det_field": det_field,
        "det_mismatch": bool(mismatch),
        "has_det_field": det_raw is not None,
        # 没有 `valid` 字段的消息**不能当 False 处理** —— 那会把本来能用的输入
        # 全拒掉。它只是"没有这个说法"，按 True 放行。
        "valid": True if valid is None else bool(valid),
        "has_valid_field": valid is not None,
        "source": str(getattr(msg, "source", "") or ""),
        "size_mm": size,
        "diag": str(getattr(msg, "diag", "") or ""),
        "rejects": str(getattr(msg, "rejects", "") or ""),
        "mode": "T_cam_pallet",
        # ⚠️ **`header.stamp` 取不到（`None` / 非数）时写 `0.0`；取到了但非正
        #    （`≤ 0`，含负数）时**原样写那个值** —— 负数不折成 `0.0`。两种情况都
        #    绝不退回 `time.time()`。** 这一条**与 `NodeBoxObs` 的口径不同，是有意的**：
        #
        #    * box 的 stamp 来自**本仓库自己的** `box_detection` 包（`WORKLOG` §17），
        #      那个生产者本来就可靠，"没有就退 tick 时刻"是"有总比没有强"。
        #    * 托盘这边是**仓库之外**的上游检测器。给它编一个"看起来像时间"的数，
        #      配对侧会**照单全收**：托盘用 tick 时刻、箱子用图像时刻，箱子那
        #      ~60ms 的检测耗时整个变成 `pair_dt` —— 要么超阈值永远配不上，要么
        #      配上一对**不同帧的图像**。两种都是静默错。
        #    * 写 `0.0` 则配对侧会**显式 reject**（`pair_stamp_zero`），失败模式
        #      一眼可见，操作员该去修的是检测器的 `header.stamp`。
        #
        #    另外 `NodePalletServo` 那边的 `_warn_missing_stamp_once()` 只在
        #    `None` / `≤ 0` 时告警 —— 编造出来的时刻**永远非零**，那道告警
        #    就**永远不会响**，两个任务的错误处置并不 compose。
        #    本节点因此自己告警一次（见 `update()` 里 `stamp <= 0.0` 那一段）。
        "stamp": stamp,
    }


# ⚠️ **手性判据不在这里重写，直接复用 `pallet_servo.algorithm.handedness_problem`。**
#
# 那个函数的 docstring 自己写着：「这段判据在同一个文件上已经回归过两次
# （容差误拒正常点击、`det≈0` 与镜面共用一句话）」。再抄一份就是第三次。
#
# 而且它就在 `skills/atomic/perception/pallet_servo/algorithm.py` 里 ——
# **本文件已经 import 了同一个模块的 `parse_bool_param`**，把它加进那个 import
# 列表即可，一行新 import 都不用。
def handedness_bad(T: np.ndarray) -> Optional[str]:
    """`T` 的旋转块是不是不在 +1 一侧。通过返回 `None`，否则返回说明。

    薄封装，判据全在 `handedness_problem` 里。**不要在这里加任何判断逻辑** ——
    它只负责处理"不是有限数"这一种 `handedness_problem` 不查的情况。
    """
    R = np.asarray(T, np.float64)[:3, :3]
    if not np.all(np.isfinite(R)):
        return f"旋转块里有 NaN/inf：{R.tolist()}"
    return handedness_problem(T)


# `valid=false` 告警的**去重桶上限**。合法的 `reject=<code>` 只有 7 种
# （`no_wood` / `no_deck` / `mask_too_large` / `no_theta_ref` / `ambiguous_theta` /
# `too_few_edges` / `degenerate`），正常永远到不了这个数。能堆到这里的只有
# `_reject_bucket()` 的**兜底路径**（`rejects` 里根本没有 `reject=` 段）—— 那种串
# 按设计逐帧在变，桶会无限涨。加个上限、超了清空重来：语义不变（每个桶仍各喊一次
# 完整文案），只是不会一直吃内存。
_REJECT_BUCKET_MAX = 16

# 同一类 `valid=false` 拒绝的**周期提醒**间隔（tick）。沿用本节点既有的节流口径
# （`update()` 里写成功那条日志用的 `self._ticks % 30 == 1`），不新造机制。
_VALID_FALSE_REMIND_TICKS = 30


def _reject_bucket(rejects: str) -> str:
    """从 `rejects` 里取**稳定的分类**做去重键 —— **不是整条文案**。

    `.msg` 契约**要求** `rejects` 写**实际值与阈值**（`score=0.3000<threshold=0.35`），
    所以它**逐帧在变** —— 拿整条文案做去重键等于**没有去重**，每 tick 重喊一遍
    （评审实测：真实生产者 + 颜色/深度交替 + 持续 `valid=false`，6 tick 出 6 条）。
    `source` 同理（检测器交替报 `color` / `depth`）。**"把易变量一个个从文案里删掉"
    这条路走不通：下一个易变量还会来** —— 所以键要换成**分类**，不是文案本身。

    取 `reject=<code>` 那一段（生产者的 `format_rejects()` 第一段就是它）。

    取不到就**退回整串** —— 那会退化成"每 tick 重喊"。**这是有意的**：那种 `rejects`
    不符合契约（不写是哪一条拒绝），操作员**就该被打扰**；**宁可退化成刷屏，也不要
    静默**。
    """
    text = str(rejects or "")
    for part in text.split(","):
        part = part.strip()
        if part.startswith("reject=") and part != "reject=":
            return part
    return text


# --------------------------------------------------------------------------- #
# 节点
# --------------------------------------------------------------------------- #
class NodePalletObs(BaseAction):
    """订阅视觉托盘检测话题 → 写黑板 `latest_pallet` + `_version`。

    params:
        topic               订阅的话题，默认 ``/pallet/detection``
        msg_type            消息类型全名，默认
                            ``pallet_detection_msgs/PalletDetection``；
                            运行期用 ``roslib.message.get_message_class()`` 解析
        pallet_key          写黑板的键，默认 ``latest_pallet``（要与伺服节点的
                            ``pallet_key`` 对得上）
        require_valid       默认 true：``valid == false`` 的帧**一个字都不写**
        require_handedness  默认 true：``det`` 不在 +1 侧时**不写**
        pallet_size_mm      ``[W, H]`` 毫米，**只用于 `size_mm` 那条交叉核对**
                            （诊断，不参与任何几何）。**必须与 `NodePalletServo`
                            上配的那个一致** —— 两者是同一个台面尺寸的两份配置，
                            详见 `_crosscheck_upstream_size()`。不给 = 整条核对
                            静默跳过（没给不算坏参数）
        enabled             默认 true；false 时一个字都不写并返回 SUCCESS
        queue_size          订阅队列，默认 1（只要最新帧，别堆积）
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._topic = str(self.params.get("topic", TOPIC_DEFAULT)).strip()
        self._msg_type = str(self.params.get("msg_type", MSG_TYPE_DEFAULT)).strip()
        self._key = str(self.params.get("pallet_key", "latest_pallet")).strip()
        # 布尔参数**不许用 `bool()` 顶替**：`bool("false") is True`，而场景 JSON
        # 的 READ_BOARD 分支真会传字符串。
        self._require_valid = parse_bool_param(
            self.params.get("require_valid", True), "require_valid")
        self._require_handedness = parse_bool_param(
            self.params.get("require_handedness", True), "require_handedness")
        try:
            self._enabled = parse_bool_param(self.params.get("enabled", True),
                                             "enabled")
            self._enabled_error = ""
        except ValueError as exc:
            self._enabled = False
            self._enabled_error = str(exc)
            logger.error("NodePalletObs 参数错误：%s —— 按 enabled=false 处理", exc)
        self._queue_size = max(1, int(self.params.get("queue_size", 1)))

        self._config_err = "" if self._topic else "topic 不能为空"
        if not self._msg_type:
            self._config_err = self._config_err or "msg_type 不能为空"

        # ★ `size_mm` 交叉核对的**基准**。它是**本节点自己**的参数，因为它没有
        #   别的来源：
        #   * 黑板上的 `latest_pallet` 是 `Pose6D`（只有六个数），装不下尺寸；
        #   * 配置的 `pallet_size_mm` 在 `NodePalletServo` **那一边**（伺服拿它
        #     当投影基准），而本节点看不到那边的参数。
        #   所以这条核对只能在**本层**做，基准只能是这份 —— 两处必须一致，
        #   见 `_crosscheck_upstream_size()` 的说明。
        #
        # 「**没给**不算坏参数」：不给 → `_size_ref is None` → 整条核对**静默跳过**
        # （它是诊断、不是闸门），既有场景一行都不用改。给了个解释不了的才是坏
        # 参数 —— **点名**（写清实际值）之后照样跳过，绝不抛。
        size_param = self.params.get("pallet_size_mm")
        self._size_ref = None
        self._size_param_err = ""
        if size_param not in (None, "", []):
            self._size_ref, self._size_param_err = parse_pair_param(
                size_param, "pallet_size_mm")
            if self._size_param_err:
                logger.warning(
                    "NodePalletObs 参数 pallet_size_mm 解释不了：%s（实际 %r）—— "
                    "**只影响 size_mm 那条交叉核对**（整条跳过），位姿链路不受影响",
                    self._size_param_err, size_param)

        self._lock = threading.Lock()
        self._subscriber = None
        self._pending: Optional[dict] = None
        self._last_written_key = None
        self._ticks = 0
        self._written = 0
        self._setup_error = ""
        self._last_warn: Optional[str] = None
        self._last_type_warn: Optional[str] = None
        self._last_det_warn: Optional[str] = None
        # ★ **新加的两条告警各有各的槽位。** 与既有的 `_last_det_warn` 同一做法。
        #
        # 为什么必须分槽：上游每帧报垃圾 `det` **且** `header.stamp` 缺失
        # （这两个恰好是同一种「上游没按契约填字段」的现场，会同时发生）时，
        # 共用 `_last_warn` 的两条告警会**每 tick 互相覆盖** —— 于是两条都
        # 每次都响。实测 6 tick 出 12 条 WARNING。
        #
        # ⚠️ 其余 `_warn_once` 调用点（手性 / 转换失败）**保持共用**：它们是同一类
        #    「这一帧被闸门挡了」，本来就会互相覆盖，且不是这一轮引入的。改动面最小。
        #    （`valid=false` 那条**这一轮搬去了自己的槽位** —— 它的去重键换成了
        #    分类，与共用槽位的"整条文案"不是一回事，见 `_warn_valid_false`。）
        self._last_stamp_warn: Optional[str] = None
        self._last_det_unreadable_warn: Optional[str] = None
        # ★ `size_mm` 交叉核对的**自己的槽位**（同样不与 `_warn_once` 共用）：
        #   去重键取"上游报的那两个数**四舍五入到整毫米**"—— 检测器的分数/长度
        #   本来就在抖，按整条文案去重会在小数点后抖出无穷多条；取整毫米之后
        #   同一个持续的量级错只喊一次，而上游真报了个**明显不同**的尺寸时会再喊。
        self._last_size_warn: Optional[tuple] = None
        # ★ `valid=false` 告警的**分类**去重槽位（不是文案本身，见 `_reject_bucket`）：
        #   * `_seen_reject_buckets` —— 已经喊过完整文案的分类；**空集**表示下一个
        #     分类要喊完整文案，正是 `initialise()` 的**既有口径**（它不重置
        #     `_last_stamp_warn` 等槽位 —— 这一条与它们保持一致，**本轮不动**）。
        #   * `_last_reject_tick` —— 该分类上次喊的 tick；周期提醒按它算（见
        #     `_warn_valid_false`），**不用全局 `_ticks % 30`**：全局相位会在
        #     分类变化时把首次喊挤掉（"同一分类 30 tick 内至多一条"保证不了）。
        #   * 为什么这条告警要**自己的槽位**：它的去重键是分类，而共用槽位
        #     `_warn_once` 的键是整条文案 —— 混在一起会让周期提醒把共用槽位里
        #     "手性/转换失败"的文案覆盖掉。见 `_warn_once` 的说明。
        self._seen_reject_buckets: set = set()
        self._last_reject_tick: Optional[int] = None

        # ---- 黑板：注册 + **同时**置初值 -------------------------------------
        # `register_key` 只注册权限、不创建值，漏了 set 就是下游的 `KeyError`。
        #
        # **三个键必须一起写。**
        #   * 只写值不写 `_version` → 下游的版本门禁会停在 (0, 0)、只算一次
        #     然后**永远不再重算**
        #   * 只写值不写 `_stamp` → 配对退回 tick 时刻，两个检测器的耗时差
        #     直接变成配对的时间偏差，**而不报错**
        #
        # `_stamp` 是必须的，不是可选的诊断量：`Pose6D` 只有六个数
        # （x/y/z/yaw/pitch/roll），**装不下时间戳**，而配对恰恰要它。
        # 箱子那边没这个问题 —— `BoxObservation` 自带 `stamp` 字段。
        self._stamp_key = f"{self._key}_stamp"
        # ⚠️ `size_mm` 也要上黑板（2026-09-30 加）：伺服的投影基准是
        # `pallet_size_mm`，而**它不知道检测器把哪条边配成了 W**。现场
        # `long_side_parallel=false` 时检测器报 `[1000,1200]`、配置是
        # `[1200,1000]` —— 两者 `max/min` 相同所以检测器自己没错，但下游拿
        # 配置那份去投影会**沿 E1（短边）走 1200mm**，比台面多 200mm，
        # 叠加图上就是"青框比托盘大一圈、方向还拧着"。详见
        # `_crosscheck_upstream_size` 的 docstring（那里记录了为什么当时
        # 判断"不该上黑板"、以及现在为什么反过来）。
        self._size_key = f"{self._key}_size_mm"
        for key in (self._key, f"{self._key}_version", self._stamp_key,
                    self._size_key):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE)
        self.global_blackboard.set(self._key, None)
        setattr(self.global_blackboard, f"{self._key}_version", 0)
        setattr(self.global_blackboard, self._stamp_key, 0.0)
        setattr(self.global_blackboard, self._size_key, None)

    # ------------------------------------------------------------------ 生命周期
    def initialise(self):
        self._setup_error = ""
        with self._lock:
            self._pending = None

        if self._config_err or self._enabled_error:
            self.feedback_message = self._config_err or self._enabled_error
            return
        if is_dry_run() or not self._enabled:
            self.feedback_message = "dry-run / enabled=false：不订阅话题"
            return

        # ★ 幂等闸：`Behaviour.tick()` 对"状态不是 RUNNING"的行为每 tick 重进
        #   `initialise()`。没有这一句就是"反复建/析构订阅"。
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
                    f"找不到消息类型 {self._msg_type!r} —— 它所在的包要能被 import"
                    f"（catkin 包先 catkin_make 再 source devel/setup.bash，并把 "
                    f"devel/lib/python*/dist-packages 放进 PYTHONPATH）")
            self._subscriber = rospy.Subscriber(
                self._topic, msg_class, self._on_message,
                queue_size=self._queue_size, tcp_nodelay=True)
            logger.info("NodePalletObs 订阅 %s（%s）→ 写 %s(+_version)；"
                        "require_valid=%s require_handedness=%s",
                        self._topic, self._msg_type, self._key,
                        self._require_valid, self._require_handedness)
        except Exception as exc:                     # noqa: BLE001
            # **绝不把异常抛出 `initialise()`**：py_trees 的 `tick()` 不接这个
            # 钩子抛出的异常，抛出去就是整棵树连每帧日志一起没。
            self._setup_error = str(exc)
            self._cleanup_subscriber()
            self.feedback_message = f"订阅 {self._topic} 失败：{exc}"

    def update(self):
        if self._config_err or self._enabled_error:
            return Status.FAILURE
        if is_dry_run() or not self._enabled:
            return Status.SUCCESS
        if self._setup_error:
            self.feedback_message = f"NodePalletObs 初始化失败：{self._setup_error}"
            return Status.FAILURE

        self._ticks += 1
        with self._lock:
            parsed = self._pending

        if parsed is None:
            # **"还没收到消息"是正常状态**，不是异常
            self.feedback_message = f"等 {self._topic} 的消息"
            if self._ticks % 50 == 1:
                logger.debug("NodePalletObs 等首条消息：%s", self._topic)
            return Status.RUNNING

        # ---- 逐条闸门：过不去就**一个字都不写**，让下游继续用上一帧 ----------
        if parsed["has_valid_field"] and self._require_valid and not parsed["valid"]:
            self.feedback_message = (
                f"跳过：这一帧 valid=false（source={parsed['source'] or '?'}），"
                f"位姿是占位值")
            # ⚠️ **去重键是分类，不是文案**（`_warn_valid_false` 里有完整的来龙去脉）：
            #    文案里**可以**带 `source` / `rejects` —— 它们逐帧在变，但那**不再
            #    影响去重**，因为键是 `_reject_bucket()` 取出来的 `reject=<code>`。
            #    第三轮把 `source` 从文案里拿掉，评审却在**真实生产者**上复现出这个病
            #    还活着 —— 生产者的 `format_rejects()` 把 `source=` 又拼了回去，而且
            #    按契约**必须**写实际值与阈值（`score=0.3000<threshold=0.35`）。
            #    所以这条路是"删易变量"治不好的，得换键；换了键之后信息量**全还回来**。
            #    （`feedback_message` 里那个 `source` 不进日志、也不参与去重，
            #    与这条告警是两回事，保持原样。）
            self._warn_valid_false(parsed["rejects"], parsed["source"])
            return Status.RUNNING

        # ---- 交叉核对上游自报的 `det`：**纯诊断，必须在闸门 2 之前** ----------
        # 这一段不写黑板、不影响任何判据，唯一的作用是**指出上游那一步有 bug**。
        # 排在闸门 2 之后的话，「镜面帧 + 上游自报 det=+1」这一帧会被闸门 2
        # **提前 return** —— 而那正是最该看到它的场景：上游自报的与本地算的
        # 对不上 ⇒ 上游坏了（而"镜面"本身往往就是那个 bug 的后果）。
        # ⚠️ **闸门本身的顺序语义不变**（闸门 1 最前、闸门 2 居中、闸门 3 最后），
        #    前移的只是这个诊断动作。
        self._crosscheck_upstream_det(parsed)
        # 与 `det` 那条同一个形状、同一个位置（都在闸门 2 之前）：这样"镜面帧 +
        # 上游尺寸也对不上"这一帧两条诊断都看得到，而不是被闸门 2 提前 return 掉。
        self._crosscheck_upstream_size(parsed)

        if self._require_handedness:
            bad = handedness_bad(parsed["T_cam_pallet"])
            if bad is not None:
                self.feedback_message = f"跳过：{bad}"
                self._warn_once(
                    f"{self._topic} 这一帧的位姿不能用：{bad} —— **不写黑板**。"
                    f"镜面/退化的位姿经 `matrix_to_pose6d()` 会被**静默投影**成"
                    f"最近的旋转，参考边跑到托盘外面去而三个数照样算得出来")
                return Status.RUNNING

        # `header.stamp` 非正 → **本节点自己告警一次**（与
        # `NodePalletServo._warn_missing_stamp_once()` 同一件事，但那一层只在
        # `None` / `≤ 0` 时响 —— 而本节点**不编造**时间戳，所以这边才是第一时间
        # 能喊出来的地方）。
        if parsed["stamp"] <= 0.0:
            self._warn_stamp_once(parsed["stamp"])

        # **同一条观测只写一次。**
        key = (parsed["stamp"], parsed["source"],
               tuple(np.round(parsed["T_cam_pallet"], 9).reshape(-1).tolist()))
        if key == self._last_written_key:
            self.feedback_message = (f"托盘观测没变（source="
                                     f"{parsed['source'] or '-'}），不重写")
            return Status.RUNNING

        try:
            pose = matrix_to_pose6d(parsed["T_cam_pallet"])
        except Exception as exc:                     # noqa: BLE001
            # 闸门 2 已经挡掉了镜面/退化，走到这儿还抛说明是别的问题
            self.feedback_message = f"位姿转换失败：{exc}"
            self._warn_once(f"{self._topic} 的位姿转换失败：{exc} —— 不写黑板")
            return Status.RUNNING

        # ⚠️ `_last_written_key` **必须等转换成功之后再赋值**（与黑板写入同一个
        #    原子块）：提前赋值的话，转换抛异常这一帧也已被标记"写过"，后续**同
        #    stamp 同矩阵**的帧会被"没变"那一道闸静默跳过 —— 黑板永远停在旧值，
        #    而每一帧都返回 RUNNING、日志里只有一句"没变"。那是最难查的一种。
        self._last_written_key = key

        setattr(self.global_blackboard, self._key, pose)
        bump_version(self.global_blackboard, self._key)
        # `_stamp` 与值、版本号**一起写** —— 它是配对的依据，漏了就会静默退回
        # tick 时刻（见 `__init__` 里那段）。
        setattr(self.global_blackboard, self._stamp_key, float(parsed["stamp"]))
        # ⚠️ 与位姿**在同一个"只写一次"的块里**：漏了的话，同 stamp 同矩阵
        # 的后续帧会在上面那道闸就返回，这块永远停在旧值 —— 而那正是
        # `_last_written_key` 注释里点名的那类静默。`size_mm` 取不出数时写
        # `None`（不写 `[0,0]`）：读数方按"没有"处理，而不是按"尺寸是 0"。
        setattr(self.global_blackboard, self._size_key,
                None if parsed["size_mm"] is None else list(parsed["size_mm"]))
        self._written += 1
        self.feedback_message = (
            f"托盘位姿 x={pose.x:+.3f} y={pose.y:+.3f} z={pose.z:+.3f} "
            f"（{parsed['mode']}，source={parsed['source'] or '-'}，"
            f"det={parsed['det']:+.4f}，stamp={parsed['stamp']:.3f}）")
        if self._ticks % 30 == 1:
            logger.debug("NodePalletObs 第 %d 次写入 %s v%d：%s", self._written,
                         self._key,
                         read_version(self.global_blackboard,
                                       f"{self._key}_version"),
                         self.feedback_message)
        # ★ **永远 RUNNING**（源节点口径）
        return Status.RUNNING

    def terminate(self, new_status):
        self._cleanup_subscriber()

    # ------------------------------------------------------------------ 回调（ROS 线程）
    def _on_message(self, msg) -> None:
        """只做「消息 → 解析结果」存内存。**黑板一个字都不碰**（只能在 tick 线程写）。"""
        parsed = parse_pallet_message(msg)
        if parsed is None:
            actual = ""
            header = getattr(msg, "_connection_header", None)
            if isinstance(header, dict):
                actual = str(header.get("type", ""))
            if actual != self._last_type_warn:
                self._last_type_warn = actual
                logger.warning(
                    "NodePalletObs 认不出 %s 上的消息（实际类型 %r，配置的 "
                    "msg_type 是 %r）—— 这一帧不写黑板。把消息的字段名贴回来，"
                    "在 parse_pallet_message() 里加一条分支即可（它是纯函数，"
                    "进 CI 很容易）", self._topic, actual or "未知", self._msg_type)
            return
        with self._lock:
            self._pending = parsed

    # ------------------------------------------------------------------ 工具
    def _cleanup_subscriber(self) -> None:
        subscriber, self._subscriber = self._subscriber, None
        if subscriber is not None:
            try:
                subscriber.unregister()
            except Exception:                        # noqa: BLE001
                pass

    def _warn_once(self, message: str) -> None:
        """**共用**槽位：闸门挡下这一帧时喊一句，同一条不重喊。

        ⚠️ 它只有**一个**槽位，所以几条不同的告警会互相覆盖。这对「这一帧被
        闸门挡了」那几条是**可接受的**（每帧只会走到其中一条，且它们不是这一轮
        引入的）。**新增的告警不要往这里塞** —— 用一个独立的槽位（见
        `_warn_stamp_once` / `_warn_det_unreadable`），否则两条告警会每 tick
        互相覆盖、双双刷屏。
        """
        if message != self._last_warn:
            self._last_warn = message
            logger.warning("NodePalletObs %s", message)

    def _warn_valid_false(self, rejects: str, source: str) -> None:
        """`valid=false` 这一帧被闸门 1 挡下 → 按**分类**去重地喊（自己的槽位）。

        ## 为什么不能拿文案当去重键

        `_warn_once` 的键是**整条文案**，所以文案里只要有**任何一个逐帧在变的量**
        就等于**没有去重**。这条告警里有两个：

          * `source` —— 检测器交替报 `color` / `depth`（第三轮已从文案里拿掉）
          * `rejects` —— `.msg` 契约**要求**它写**实际值与阈值**
            （`score=0.3000<threshold=0.35`），**不写就排查不了**，所以它逐帧不同；
            而且真实生产者的 `format_rejects()` 还会把 `source=` 再拼回来

        第三轮"删易变量"的修法因此在真实生产者上**完全失效**（评审实测 6 tick 6 条）。
        **下一个易变量还会来** —— 所以键换成**稳定的分类**（`_reject_bucket`），
        而不是继续删文案。

        ## 两档输出

        * **新的分类** → 立刻喊**完整文案**（带 `source` 与 `rejects`）。
          第一次喊是唯一的、不会刷屏，**信息量全部还回来**（第三轮为了去重把
          `source` 删掉、代价却白付了 —— 它又随 `rejects=` 回到了同一句里）。
        * **同一分类再来** → **每 30 tick 提醒一次**，用**短文案**（只报分类，
          不带易变量）。

        ⚠️ **周期提醒不能省。** 只在"分类变化"时喊，会让一个**持续失败**的检测器
        在操作员眼前**彻底安静** —— 那比刷屏更糟（刷屏至少还看得见）。
        节流口径沿用本节点既有的 `self._ticks % 30 == 1` 那个 30。

        ⚠️ **周期按"上次喊这个分类的 tick"算，不是全局相位。** 用 `self._ticks % 30`
        的话，首次喊发生在哪个 tick 就决定了相位 —— 分类恰好在 tick 30 才变化时，
        首次喊（完整文案）与周期提醒会在**同一 tick** 触发，后者把前者**直接吞掉**。
        按 `_last_reject_tick` 算则"同一分类 30 tick 内至多一条"，与相位无关。

        ⚠️ **不用共用槽位 `_warn_once`**：它的键是整条文案，而这里两个键不是一回事。
        混在一起会让周期提醒把共用槽位里"手性 / 转换失败"的文案覆盖掉（反之亦然）。
        """
        bucket = _reject_bucket(rejects)
        if bucket not in self._seen_reject_buckets:
            if len(self._seen_reject_buckets) >= _REJECT_BUCKET_MAX:
                # 兜底路径的 `rejects` 逐帧在变 → 桶会无限涨。清空重来：
                # 语义不变（每个桶仍各喊一次完整文案），只是不吃内存。
                self._seen_reject_buckets.clear()
            self._seen_reject_buckets.add(bucket)
            self._last_reject_tick = self._ticks
            # ★ 完整文案：`source` 与 `rejects` 都**还回来**
            logger.warning(
                "NodePalletObs %s 这一帧 valid=false（rejects=%s，source=%s）"
                "—— 位姿是**占位值**，**不写黑板**，下游继续用上一帧；"
                "之后同一类拒绝每 %d tick 只提醒一次",
                self._topic, rejects or "无", source or "?", _VALID_FALSE_REMIND_TICKS)
            return
        if self._ticks - self._last_reject_tick >= _VALID_FALSE_REMIND_TICKS:
            self._last_reject_tick = self._ticks
            # ★ 短文案：只报分类，不带易变量（那正是会让去重失效的东西）
            logger.warning(
                "NodePalletObs %s 仍在报 valid=false（%s）—— 上游还没修，"
                "不写黑板（同一类拒绝每 %d tick 提醒一次）",
                self._topic, bucket, _VALID_FALSE_REMIND_TICKS)

    def _warn_stamp_once(self, stamp) -> None:
        """`header.stamp` 非正 → 喊一次（**自己的槽位**，不与 `_warn_once` 共用）。

        **去重键不含 `source`**：检测器交替报 `color` / `depth`、或隔帧
        `valid=false` 时，`source` 每帧都在变 —— 把它放进去重键（无论放在文案里
        比较，还是单独比较）就等于**没有去重**，每 tick 重喊一遍（实测 5 tick 5 条）。
        文案里也不再提它：这条告警要操作员去修的是**上游检测器的 `header.stamp`
        没填**，`source` 那点信息在这一句里没有用处，而它恰恰是那个会让去重失效的
        易变量。**稳定性优先于信息量。**
        """
        # ⚠️ 「黑板写的是什么」必须报**实际写进去的那个值**（`update()` 里
        #    `setattr(..., float(parsed["stamp"]))`，传进来的 `stamp` 就是它）：
        #    非正这一支里 `0.0` 与**负数**都会走到这儿，写死 `0.0` 就是让日志
        #    在说假话（`header.stamp = -5.0` 的帧，操作员读到"写的是原始值 0.0"，
        #    而黑板上是 `-5.0`）。
        msg = (f"{self._topic} 这一帧的 `header.stamp` 取不到 / 非正（取到 "
               f"{stamp!r}）—— 黑板上的 `{self._stamp_key}` 写的是"
               f"**原始值 {stamp!r}**，"
               f"不编造处理时刻。**配对会失败**（下游按 pair_stamp_zero 显式 "
               f"reject，这比配上一对不同帧的图像好）—— 去修上游检测器：它的 "
               f"`header.stamp` 没填。")
        if msg != self._last_stamp_warn:
            self._last_stamp_warn = msg
            logger.warning("NodePalletObs %s", msg)

    def _crosscheck_upstream_det(self, parsed: dict) -> None:
        """交叉核对上游自报的 `det` —— **纯诊断**，不写黑板、不影响任何判据。

        ⚠️ **必须在闸门 2 之前调用。** 排在它后面的话，「镜面帧 + 上游自报
        `det=+1`」这一帧会被闸门 2 提前 `return`，于是 `det_mismatch` 那条
        （唯一能指出「上游那一步有 bug」的线索）在最该出现的场景里**永远不响**。

        上游**报了** `det` 但那个值是垃圾（字符串/NaN 之类）时，`_num` 返回
        `None` → 核对**静默跳过**。这是**有意的**：核对只是诊断，判据用的是本
        节点自己算的 `det`，不该因为上游把字段填成字符串就拒掉这一帧。但要留
        一句日志，别让"核对没跑"与"核对跑了且一致"在日志里长得一模一样。
        """
        if parsed["det_mismatch"]:
            msg = (f"上游自报的 det={parsed['det_field']:+.6f} 与本地算出来的 "
                   f"{parsed['det']:+.6f} 对不上（容差 {DET_MISMATCH_TOL:g}）—— "
                   f"**判据用的是本地算的那个**，上游那一步多半有 bug")
            if msg != self._last_det_warn:
                self._last_det_warn = msg
                logger.warning("NodePalletObs %s", msg)
        elif parsed["has_det_field"] and parsed["det_field"] is None:
            self._warn_det_unreadable(parsed["diag"], parsed["rejects"])

    def _crosscheck_upstream_size(self, parsed: dict) -> None:
        """交叉核对上游自报的 `size_mm` 与本节点配置的 `pallet_size_mm`。

        **纯诊断，只告警不拦** —— 判据与几何一律用**配置值**（见 `SIZE_MISMATCH_
        REL_TOL` 那段）。它补的是 `_crosscheck_upstream_det` 留下的一个空洞：
        `det` 对不上说明上游**手性**那一步坏了，而 `size_mm` 对不上说明上游
        「把量出来的长度配成已知尺寸」那一步坏了 —— 两者同属"三个数照样算得
        出来、现场只有人眼在叠加图上看得出不对"那一类。

        ## 为什么这一条在**本节点**做，而不是在 `NodePalletServo`

        要核对的另一边是**配置的 `pallet_size_mm`**，而那份配置在伺服节点上。
        两条路各有利弊，这里选了"本节点加一个只管诊断的参数"：

        * **伺服那条路要先把 `size_mm` 送上黑板** —— 那是新增一条跨节点接口，
          而且会把一个设计上**明文禁止**当输入的量（`.msg`：伺服的投影基准是
          配置值，`size_mm` 只是诊断）摆到伺服手边，等于给它留一条被误用的路。
        * **本节点这条路的问题是"同一个尺寸配了两份"**，但它的代价是**可见的**：
          两处不一致时这条 WARNING 会响，而文案里**点名**了这一点（见下），
          操作员先核对场景 JSON 的两处即可。`pallet_key` / `box_key` 两侧必须
          对得上也是同一类契约。

        ## ⚠️ 上面这条推理在 2026-09-30 被现场推翻了 —— `size_mm` 已经上黑板

        当时的前提是"伺服的投影基准是配置值，`size_mm` 只是诊断"。现场实测：
        检测器在 `long_side_parallel=false` 下报 `[1000,1200]`（短边横向——
        这是操作员要的语义），而配置 `pallet_size_mm` 是 `[1200,1000]`。
        **两个数的 `max/min` 相同，所以检测器自己没错**；但下游拿配置那份去
        `project_pallet_points` 时，`(W,0,0)` 会**沿 E1（短边方向）走 1200mm**
        —— 比台面多出 200mm，叠加图上就是"青框比托盘大一圈、比例也不对"。

        它**不会报错**（三个数照样算得出来），而那正是最难查的一类。
        所以现在 `NodePalletObs` 把上游自报的 `size_mm` 写到
        `{pallet_key}_size_mm`，`NodePalletServo` 优先用它、配置那份退成兜底
        （见那边的 `_resolve_size`）。**两边仍然是同一个尺寸的两份配置，
        但顺序不再影响几何** —— 检测器说什么就是什么。

        ⚠️ `pallet_size_mm` 参数**仍然要配**：本节点用它做范围校验的兜底、
        上游没给时伺服也用它。但**不要再指望"两份必须一致"**：不一致是正常
        工况（`long_side_parallel` 一翻就不同），这条 WARNING 的文案也跟着改了。

        ## 跳过（**静默**）的三种情况

        * 本节点**没给** `pallet_size_mm`（`_size_ref is None`）—— 没给不算坏参数；
        * 上游这一帧**没有** `size_mm`（`[]` / 取不出数）—— 核对只是诊断，
          不该因为上游少填一个可选字段就拒帧，也不该为此吵；
        * `valid == false` 的**占位帧** —— 按契约那种帧下面全是占位值
          （`size_mm` 填 0），拿它比会每帧都喊，而那正是这一条最该避开的刷屏。

        ⚠️ **判据一个字都不用这条告警**：`size_mm` 从来不是输入。
        """
        if self._size_ref is None or parsed["size_mm"] is None:
            return
        if not parsed["valid"]:
            # 占位帧（`valid=false`）：下面的字段按契约都是占位值，不判。
            # 注意 `parse_pallet_message` 把"**没有** valid 字段"归成 True ——
            # 那是"没有这个说法"，不是"说不能用"，所以这里只看显式的 false。
            return
        gap = _size_rel_gap(parsed["size_mm"], self._size_ref)
        if gap is None or gap <= SIZE_MISMATCH_REL_TOL:
            return
        # 去重键：上游报的那两个数**四舍五入到整毫米**（理由见 `_last_size_warn`）
        key = tuple(round(float(v)) for v in parsed["size_mm"])
        if key == self._last_size_warn:
            return
        self._last_size_warn = key
        logger.warning(
            "NodePalletObs %s 上游自报的 size_mm=%s 与本节点配置的 pallet_size_mm="
            "%s 差得远（最大逐维相对差 %.0f%%，容差 %.0f%%）。⚠️ **先看是不是"
            "`long_side_parallel` 翻过** —— 那个键只换 W/H 的次序、不换 max/min，"
            "而本节点这条是按**逐维**比的，所以它一翻这条就会响（`[1000,1200]` vs "
            "`[1200,1000]` 相对差 17%%）。**几何不受影响**：伺服现在用上游报的"
            "`size_mm`（黑板 `%s`），配置那份只在没给时兜底。真正要查的是"
            "「两个数连 max/min 都对不上」（量级错，比如少写一位）",
            self._topic, [round(float(v), 1) for v in parsed["size_mm"]],
            [round(float(v), 1) for v in self._size_ref], gap * 100.0,
            SIZE_MISMATCH_REL_TOL * 100.0, self._size_key)

    def _warn_det_unreadable(self, diag: str, rejects: str) -> None:
        """上游**报了** `det` 但那个数取不出来（字符串/NaN…）时留一句。

        交叉核对因此**静默跳过** —— 这是**有意的**：核对只是诊断，判据用的是
        本节点自己算的 `det`，不该因为上游把字段填成字符串就拒掉这一帧。
        但没有这句话，"核对没跑"与"核对跑了且一致"在日志里长得一模一样。

        **自己的槽位**（`_last_det_unreadable_warn`），不与 `_warn_once` 共用 ——
        见 `_warn_once` 的说明。
        """
        msg = (f"{self._topic} 上游的 `det` 字段取不出数（不是 float/int/数字字符串）"
               f"—— **交叉核对跳过**（判据用的是本节点自己算的 det，这一帧照常处理）。"
               f"diag={diag or '-'} rejects={rejects or '-'}")
        if msg != self._last_det_unreadable_warn:
            self._last_det_unreadable_warn = msg
            logger.warning("NodePalletObs %s", msg)
