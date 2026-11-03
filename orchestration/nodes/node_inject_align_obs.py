# -*- coding: utf-8 -*-
"""NodeInjectAlignObs：把离线模拟的 AlignObservation 写进黑板，冒充视觉感知模块。

**只为离线联调存在。** 真机上 ``latest_servo_error`` 由上游视觉感知模块
（carton_vision 侧）写，这个节点不参与——用 ``enabled: false`` 把它关掉即可。

它写的是**真实链路里的那个键**（``latest_servo_error`` + ``_version``），所以伺服
节点一行都不用改；这就是设计文档要的"接缝留好，真机接入时下游代码一行不改"。

输入文件是一个 JSON 对象，``observations`` 是一个列表，逐 tick 播放一条、版本号
递增——这样离线就能看到伺服误差随观测变化而"实时"变化。每条观测只需给核心偏差
字段（``e_bottom_px`` / ``e_right_px`` / ``theta_rad`` / ``valid``），
其余字段本节点补 0。

本节点是个**源节点**，所以有东西可写时持续返回 **RUNNING**（与 ``NodePercep`` /
``NodePalletPose`` / ``NodeInjectServoInput`` 同口径：自己决定写什么，何时收由
父节点定），播完也保持 RUNNING 但一个字都不写。**不能返回 SUCCESS**：
py_trees 的 ``Behaviour.tick()`` 在 ``status != RUNNING`` 时**每 tick 重进
``initialise()``**，而 ``initialise()`` 会把播放游标归零 —— 于是列表从头再播一遍，
"播完停手"根本没发生过。同理，坏输入是 FAILURE（终点，父节点能反应），
``enabled: false`` 与 dry-run 则是 SUCCESS（不占着树）。

与 ``node_inject_servo_input.py`` 同构，只是 ``AlignObservation`` 字段比
``BoxObservation`` 简单，解析也跟着简化。
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from core.domain.servo_types import AlignObservation
from orchestration.nodes.base_node import BaseAction

logger = get_logger(__name__)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _read_blackboard(client, key, default=None):
    """读黑板键；**键从未被写过时也返回 `default`**。

    py_trees 的 `Client.__getattr__` 在键不存在时抛的是 **`KeyError`**，
    `getattr(client, key, None)` 接不住它 —— 与 `node_inject_servo_input`.
    `_read_blackboard` 同一个坑、同一份兜底。
    """
    try:
        return getattr(client, key)
    except KeyError:
        return default


def _read_version(client, key) -> int:
    """黑板上的版本号；没写过 / 不是数字都当 0（调用方随后 +1 写回）。"""
    try:
        return int(_read_blackboard(client, key, 0))
    except (TypeError, ValueError):
        return 0


def _read_idle(client, key) -> bool:
    """读节拍门禁键；从未写过（执行节点还没构造）当作空闲，别把树卡死。"""
    return bool(_read_blackboard(client, key, True))


def _parse_bool(val, default: bool) -> bool:
    """认 JSON 布尔值和 true/false/1/0/yes/no 字符串；别的值按 default 走。"""
    if isinstance(val, bool):
        return val
    if isinstance(val, (int, float)):
        return bool(val)
    if isinstance(val, str):
        s = val.strip().lower()
        if s in ("true", "1", "yes"):
            return True
        if s in ("false", "0", "no"):
            return False
    return default


# AlignObservation 里模拟数据常省略的字段 → 默认值（与 dev 视觉链路 ServoError 字段对齐）
_OBS_DEFAULTS = {
    "valid": True, "reject": "",
}


def _parse_obs(entry: Dict[str, Any], index: int) -> AlignObservation:
    """一条观测 dict → AlignObservation。缺字段用默认值补，不抛。

    只强制要 ``e_bottom_px`` / ``e_right_px`` / ``theta_rad`` 三个核心偏差字段
    （没有它们伺服没有可算的东西），其余可省略。
    """
    where = f"observations[{index}]"
    if not isinstance(entry, dict):
        raise ValueError(f"{where} 不是对象，是 {type(entry).__name__}")
    fields: Dict[str, Any] = {}
    for key, default in _OBS_DEFAULTS.items():
        fields[key] = entry.get(key, default)
    for required in ("e_bottom_px", "e_right_px", "theta_rad"):
        if required not in entry:
            raise ValueError(f"{where} 缺 `{required}` —— 每条观测都得有 "
                             f"e_bottom_px/e_right_px/theta_rad")
    fields["e_bottom_px"] = float(entry["e_bottom_px"])
    fields["e_right_px"] = float(entry["e_right_px"])
    fields["theta_rad"] = float(entry["theta_rad"])
    fields["valid"] = _parse_bool(entry.get("valid", True), True)
    fields["reject"] = str(entry.get("reject", ""))
    return AlignObservation(**fields)


def _parse_payload(payload) -> List[AlignObservation]:
    """把 JSON payload 拧成 AlignObservation 列表。任何字段不对都抛 ValueError。"""
    if not isinstance(payload, dict):
        raise ValueError(f"顶层不是 JSON 对象，是 {type(payload).__name__}")
    if "observations" not in payload:
        raise ValueError("没有 `observations` 字段（观测列表）")
    raw = payload["observations"]
    if raw is None:
        raise ValueError("`observations` 是 null —— 要一个列表")
    if not isinstance(raw, list):
        raise ValueError(f"`observations` 不是列表，是 {type(raw).__name__}")
    if not raw:
        raise ValueError("`observations` 是空列表 —— 至少要一条观测")
    return [_parse_obs(e, i) for i, e in enumerate(raw)]


class NodeInjectAlignObs(BaseAction):
    """把一份模拟 AlignObservation 列表逐 tick 写到黑板上。

    状态：有东西可写 / 播完了都返回 **RUNNING**（源节点口径，见模块 docstring），
    坏输入返回 FAILURE，``enabled: false`` 与 dry-run 返回 SUCCESS。

    params:
        sim_path        模拟观测 JSON 文件路径（必填）
        obs_key          写观测的黑板键名，默认 latest_servo_error
        enabled          false 时什么都不写，默认 true（真机上用它关掉本节点）
        idle_key         节拍门禁键名，默认 servo_exec_idle；读到 False（底盘有动作
                         在途）就暂停播放，等执行节点走完这一步再播下一帧
        hold_ticks       每条观测保持几个 tick（默认 1 = 每 tick 换一条，行为与改动前
                         完全一致）。真机上主循环 50Hz、每 tick 才 20ms，一条观测只
                         活一帧的话速度模式连 0.1s 都跑不到、底盘看不出动静；把这值
                         调大就能按需拉长每条观测的有效时长。

                         ⚠️ 必须满足 `hold_ticks * 0.02 < exec 的 obs_timeout_s`
                         （默认 0.5s ⇒ hold_ticks ≤ 24）。本节点是**每换一条观测才**
                         涨版本号，而 exec 的速度看门狗是**按版本号是否变化**判观测
                         死活的 —— 保持时间一旦越过看门狗阈值，就会在自己主动保持
                         的那段里被误判成「视觉断了」而发零速、判 FAILURE。
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._sim_path = Path(str(self.params.get("sim_path", "")))
        self._obs_key = str(self.params.get("obs_key", "latest_servo_error"))
        self._idle_key = str(self.params.get("idle_key", "servo_exec_idle"))
        self._hold_ticks = max(1, int(float(self.params.get("hold_ticks", 1) or 1)))
        try:
            self._enabled = _parse_bool(self.params.get("enabled", True), True)
            self._enabled_error = ""
        except (TypeError, ValueError) as exc:  # _parse_bool 不抛，兜底
            self._enabled = False
            self._enabled_error = str(exc)

        # 只注册自己会写的键
        for key in (self._obs_key, f"{self._obs_key}_version"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE)
        self.global_blackboard.register_key(
            key=self._idle_key, access=py_trees.common.Access.READ)
        # 先置初值，免得别的节点 getattr 时 KeyError
        self.global_blackboard.set(self._obs_key, None)
        setattr(self.global_blackboard, f"{self._obs_key}_version", 0)

        self._observations: List[AlignObservation] = []
        self._cursor = 0
        self._obs_version = 0
        # 当前这条观测还剩几个 tick 的保持额度（见 hold_ticks 文档）
        self._hold_left = 0
        # 上一次喊过的坏输入原文（去重 ERROR，见 node_inject_servo_input._reject）
        self._last_reject: Optional[str] = None

    def _reject(self, message):
        """坏输入的统一出口：feedback 每 tick 更新，ERROR 只在原文变了时喊。"""
        self.feedback_message = message
        if message != self._last_reject:
            self._last_reject = message
            logger.error("%s", message)

    def initialise(self):
        self._observations = []
        self._cursor = 0
        self._hold_left = 0
        # 版本号接着黑板上的当前值涨（真生产者 NodePalletPose.update 这么写）。
        # 归零会让重播时版本从 3 掉回 1 —— 换成单调门禁会把它当陈旧数据跳过。
        self._obs_version = _read_version(
            self.global_blackboard, f"{self._obs_key}_version")
        if self._enabled_error:
            self._reject(f"enabled 参数不对（{self._enabled_error}）—— 已按"
                         f" enabled=false 关掉，一个键都不写")
            return
        if not self._enabled:
            self.feedback_message = "enabled=false，不写任何东西"
            return
        try:
            payload = json.loads(self._sim_path.read_text(encoding="utf-8"))
        except OSError as exc:
            self._reject(f"读不到模拟输入 {self._sim_path}: {exc}")
            return
        except ValueError as exc:  # JSONDecodeError 是 ValueError
            self._reject(f"模拟输入 {self._sim_path} 不是合法 JSON：{exc}")
            return
        try:
            self._observations = _parse_payload(payload)
        except (ValueError, KeyError, TypeError) as exc:
            # 解析不了的输入只许是 FAILURE（update 里返回）：py_trees 的
            # Behaviour.tick() 不接 initialise()/update() 抛出的异常，抛出去
            # 就是整棵树一起没，比 FAILURE 糟得多。
            self._reject(f"模拟输入 {self._sim_path} 字段不对：{exc}")
            return
        self._last_reject = None
        logger.info("模拟观测就绪：%s，%d 条 AlignObservation",
                    self._sim_path, len(self._observations))
        self.feedback_message = f"已装载 {self._sim_path.name}：{len(self._observations)} 条观测"

    def update(self):
        if self._enabled_error:
            return Status.FAILURE
        if _DRY_RUN:
            # 与 NodePercep / NodePalletPose / NodeInjectServoInput 同口径：
            # dry-run 一律 SUCCESS（不碰硬件，--tick-once 才收得了场）。
            return Status.SUCCESS
        if not self._enabled:
            # 关掉 = 不占着树：SUCCESS 让父节点照常往下走。
            return Status.SUCCESS
        if not self._observations:
            # 坏输入（initialise 已喊过）：FAILURE 是终点，父节点看得见。
            return Status.FAILURE

        # 节拍门禁：底盘还在走上一步就先把帧攒着。模拟源每 tick 播一帧，而底盘走完
        # 一步要几百毫秒 —— 不拦的话几帧观测会在几十毫秒内烧光，calc/judge 跟着空转，
        # 看起来「跑完了」其实底盘一步没走。
        if not _read_idle(self.global_blackboard, self._idle_key):
            self.feedback_message = (f"底盘在途，暂停播放（停在第 "
                                     f"{self._cursor}/{len(self._observations)} 条）")
            return Status.RUNNING

        # 播完就停手，不要拿最后一条反复写（每写一次版本号涨，下游版本门禁跟着
        # 重算同一个数，白算）。但不能返回 SUCCESS（见模块 docstring），继续
        # RUNNING、一个字都不写，最后一帧留在黑板上给伺服节点一直用。
        if self._cursor >= len(self._observations):
            self.feedback_message = f"播放完毕（{len(self._observations)} 条）"
            return Status.RUNNING

        # 保持期（hold_ticks）：当前这条观测还没到换的时候，一个字都不写。
        # 不写 = 版本号不涨 = 下游版本门禁不重算、exec 的观测看门狗也认得出这是
        # 「本节点主动保持」而非「视觉断流」，前提是保持时长不越过 obs_timeout_s。
        if self._hold_left > 0:
            self._hold_left -= 1
            self.feedback_message = (f"保持第 {self._cursor}/{len(self._observations)} 条"
                                     f"观测（还剩 {self._hold_left} tick）")
            return Status.RUNNING

        obs = self._observations[self._cursor]
        setattr(self.global_blackboard, self._obs_key, obs)
        self._obs_version = self._bump_version(self._obs_key)
        self._cursor += 1
        self._hold_left = self._hold_ticks - 1
        self.feedback_message = (f"播放第 {self._cursor}/{len(self._observations)} 条"
                                 f"观测（e_bottom={obs.e_bottom_px}, "
                                 f"e_right={obs.e_right_px}, "
                                 f"theta={obs.theta_rad}）")
        logger.debug("模拟源写入：%s v%d（%s）",
                     self._obs_key, self._obs_version, self.feedback_message)
        # RUNNING：源节点继续持有这一支，initialise() 因此不会被重进
        # （游标保住，列表才会一条条往下播）。
        return Status.RUNNING

    def _bump_version(self, key) -> int:
        """把 `key_version` 加一写回，返回写进去的那个数。

        读黑板上的当前值再加，与 NodePalletPose.update 一模一样：版本号
        因此单调，重进 initialise() 也不会掉回去。
        """
        version = _read_version(self.global_blackboard, f"{key}_version") + 1
        setattr(self.global_blackboard, f"{key}_version", version)
        return version
