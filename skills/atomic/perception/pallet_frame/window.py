# -*- coding: utf-8 -*-
"""`PalletFrameWindow`：两路观测的缓存 + 配对 + 成对序列维护。

有状态，**不 import ROS**。

## 为什么平滑放在这里、不放在检测器里

「托盘与箱子同窗长同聚合」如果靠两边各自实现，就是两份独立的滞后，谁改一边就
静默失配。放在一处，才能与箱子共用同一份配置、并用一条测试钉住。

而且因为**每一对都是同一帧图像**，配对之后再做窗，窗内两边的相位**天然对齐** ——
「同窗长同聚合」变成自动成立：同一个窗、同一批对，没有第二份实现可以走偏。

## 一次 resolve 的生命周期

    push_pallet(...)  ─┐
                       ├→ resolve() → PairedFrame / PairReject / None
    push_box(...)     ─┘

`resolve()` 返回 `None` 表示**没有新的配对可报**。节点每 tick 都调它，不加这道闸
就是"同一对反复出数、版本号一直涨"。

⚠️ 但 `None` 是**两种**含义共用的返回值（"同一对已报过" / "某一侧没有新观测"，
后者= 检测器停摆），**在返回类型上不可区分** —— 所以 **liveness 判据不在本层**，
由 Task 5 的节点负责。详见 ③ 与 `resolve()` 的 docstring。

## 两处"照抄会错"的地方（都不是照抄题）

### ① "配出哪一对"不能只看缓存里的最小值（`nearest_pair` 的 tie-break）

`nearest_pair` 并列最小 dt 时**先遇到者胜**（下标小的赢）。稳态下每一对的 dt 都是
0（两个检测器看同一帧），于是**缓存里最早的那一对永远赢**。这会同时坏掉两件事：

* **去重**：报过 (1.0,1.0) 之后，下一帧选出来的还是它 —— 直接返回 `None` 会让窗
  在报出第一对之后**永久沉默**（直到旧对被 `pair_cache` 挤出去，才吐出一个已经
  过期好几帧的位姿）。伺服以 1/5 的输出率拿到 165ms 前的位姿 —— 静默错。
* **出的是哪一帧**：就算靠"和 `_last_key` 比对"挡掉了重报，挡掉的也只是"完全
  相同的那一对"；(1.0,1.0) 与 (1.1,1.1) 并列时仍旧选 (1.0,1.0)（**更旧**），
  于是 `_last_key` 反而被回写成旧值，下一帧继续选中旧对 —— 一样是沉默，只是
  死法不同。

所以 `resolve()` **不看全量缓存的最小值**，只在**比上次报出去的那一对更新的
观测**里挑（`_newer_than_last()`）。这样挑出来的对必然严格新于上次那一对，
"每一帧报一次、且报的是最新那一帧"就是结构性成立的，不是靠事后比对挡下来的。
还没报过任何一对时（`_last_key is None`）才走全量缓存，此时 `nearest_pair`
自己的判据与顺序**原样不变**。

### ② 一个坏观测不该挤掉好观测（`pair_stamp_zero` 判在"选出最小 dt 之后"）

`nearest_pair` 的 `pair_stamp_zero` 判在**选出全局最小 dt 之后**：坏对的 dt 往往
就是 0（两侧都停在 0.0 上），天然是"最小"的那一个，于是整次调用直接 reject。可
那个 reject 的本意是"**这一对**不能配"（两路时间戳差着整个 epoch），不是"**这一
窗**作废"。坏观测要在 deque 里待满 `pair_cache` 次推入才会被挤出去，这期间每个
tick 都返回同一个 reject，**好观测一直配不上、伺服一个数都出不来**。

本类的处置：先按原判据判一次（这样"两侧都忘了填 header"仍然报 `pair_stamp_zero`
而不是被静默吞成 `pair_none`），只在**选中的那一对确实两侧同时 ≤ 0** 时，剔掉
`stamp <= 0` 的观测再挑一次；**只在重试真的配上时才采信**，重试失败就原样返回
`pair_stamp_zero`。理由与边界见 `_best_pair()`。
（真正的理由是"**坏对必须有自己的名字**"；"预先剔会废掉 `NodePalletObs` 写
0.0 的设计意图"那条在集成层站不住，见 `_best_pair()` 里的措辞更正。）

## ③ 时钟回跳必须 `reset()`（给 Task 5 的调用方看）

`_last_key` 是"已经消费掉的观测"的水位线。**时钟整体回跳**（bag 循环播放、换
时间源、重进 `initialise()`）之后，新观测的 stamp 全都小于它，`resolve()` 于是
**永久返回 `None`** —— 伺服黑板停在旧值，一个错都不报。本层只看得见 stamp，
分不清"时间倒退了"与"检测器没出数"（两者都长成 `None`），所以**发现回跳是调用
方的责任**：`NodePalletServo.initialise()` / 重进流程里必须调 `reset()`（调用方
**已经接了** —— `node_pallet_servo.py:962` 就在 `initialise()` 里）。这条前提仍然
写在这儿，是因为它对**下一个调用方**同样成立。细节见 `reset()` 与 `resolve()` 的
docstring。
"""
from __future__ import annotations

from collections import deque
from typing import Deque, List, Optional, Tuple, Union

from .algorithm import (
    PairedFrame,
    PalletObservation,
    PairReject,
    average_pairs,
    nearest_pair,
)


class PalletFrameWindow:
    """两路观测的配对窗。

    params:
        max_dt_s    配对允许的最大时间戳差（秒）。30fps 下一帧 = 33ms，
                    默认 0.05 留一帧半的余量。
        pair_cache  两边各留多少个观测供配对。**不是窗长** —— 配对要往回看
                    几个，因为有延迟的检测器交出来的可能是几帧前的。
                    ⚠️ **它的量纲：回溯窗口 ≈ `pair_cache` × 伺服 tick 周期。**
                    push 是**按伺服 tick** 发生的（版本变了才推，不是检测器一
                    出帧就推），所以 10 Hz 下 `pair_cache=5` 就是 **500 ms** 的
                    回溯。这个窗口**必须大于托盘检测器的端到端延迟** —— 小了的话
                    "慢的那个检测器"交出来的帧早就被挤出去了，配对只能一直报
                    `pair_none`/`pair_dt`；大了则坏观测多留几个 tick。换 tick
                    频率（或换一个更慢的检测器）时**这个数要跟着改**。
        window      成对序列上的窗长（1 = 不平滑）。与 `pair_cache` **互相独立**：
                    前者是"成对序列上取几对做平均"，后者是"每一侧留几个观测供
                    配对"，两个旋钮各管各的，谁都不封顶谁。想平滑 8 帧就写
                    `window=8`，**不必**连带把 `pair_cache` 也开大 —— 开大
                    `pair_cache` 只会加深配对回溯、让坏观测多留几个 tick
                    （模块 docstring ② 列的代价），与平滑几帧无关。
                    构造函数只保证 `window >= 1`、`pair_cache >= 1`。
        agg         多对聚合：`mean` / `median`。**只作用在箱子四角上**，托盘一律
                    刚体平均。**拼错（或大小写不对）静默按 `mean`** —— 合法值只有
                    这两个，理由见 `average_pairs` 的 docstring；Task 5 若想让
                    拼错可见，在节点侧读参数时白名单校验一次。
        stale_s     托盘观测超过这个时长算陈旧（0 = 不查）。**这是"延迟闸"不是
                    "存活闸"** —— 见 `resolve()` 里 `stale` 那段。
    """

    def __init__(self, *, max_dt_s: float = 0.05, pair_cache: int = 5,
                 window: int = 5, agg: str = "mean",
                 stale_s: float = 0.0) -> None:
        self._max_dt_s = float(max_dt_s)
        self._cache = max(1, int(pair_cache))
        # 两个旋钮各自独立，**不要**互相钳：`pair_cache` 是"每一侧留几个观测供
        # 配对"，`window` 是"成对序列上取几对做平均"—— 两件事，没有谁封顶谁。
        # `_pairs` 是持强引用的 deque，已配好的对不会因为观测被 `_pallets` 挤出去
        # 而消失；配对侧只需要"比水位线更新的**一个**观测"，每 tick 推一个就够。
        # 所以 `pair_cache=5, window=8` 就是老老实实平滑 8 帧。想抬 `pair_cache`
        # 的人要付的代价是配对回溯更深、坏观测多留几个 tick（模块 docstring ②），
        # 别为了 `window` 去动它。
        self._window = max(1, int(window))
        self._agg = str(agg)
        self._stale_s = float(stale_s)

        self._pallets: Deque[PalletObservation] = deque(maxlen=self._cache)
        self._boxes: Deque[object] = deque(maxlen=self._cache)
        self._pairs: Deque[Tuple[PalletObservation, object]] = deque(
            maxlen=self._window)
        # 上一次报出去的配对，用来去重。用**两个 stamp** 做 key：
        # 托盘和箱子各自的 stamp 一起定死了"是哪一对"；它同时是"已经消费掉的
        # 观测"的水位线（见 `_newer_than_last()`）。
        self._last_key: Optional[Tuple[float, float]] = None

    # ------------------------------------------------------------------ 推
    def push_pallet(self, obs: PalletObservation) -> None:
        self._pallets.append(obs)

    def push_box(self, obs) -> None:
        self._boxes.append(obs)

    # ------------------------------------------------------------------ 配对
    def _best_pair(self, pals: List[PalletObservation],
                   boxes: List[object]):
        """在给定的两份观测里挑一对。返回 `(found, 托盘表, 箱子表)`。

        后两项必须一起返回：`found` 是**这两份列表**的下标，而下面重试时剔掉了
        元素，下标已经不是原 deque 的下标了。

        `pair_stamp_zero` 的重试（模块 docstring ②）：`nearest_pair` 判它判在
        "选出全局最小 dt 之后"，坏对的 dt 往往是 0，于是它天然是"最小"的那一个，
        整次调用直接 reject —— 缓存里同时有坏对和好对时，**好对被坏对挤掉**，
        而且坏观测要等 `pair_cache` 次推入才出得去，这期间每个 tick 都是同一个
        reject。上层节点每 tick 都调 `resolve()`，代价被放大到 `pair_cache` 个 tick。

        三条约束让它只赚不亏：

        * **先按原判据判一次**，不预先剔 —— 预先剔的话"两侧都还没填 header"这种
          真实故障会被静默吞成 `pair_none`（剔完两边都空），操作员看到的是"没有
          观测"而不是"时间戳没填"。坏对必须有自己的名字。
          （**措辞更正（第二轮）**：早先写的理由是"预先剔会废掉 `NodePalletObs`
          写 0.0 的设计意图"，那条在**集成层站不住** —— 两路的生产者现在都**不
          编造时刻**（`NodePalletObs` 写 `0.0`，`NodeBoxObs` 也写哨兵 `0.0`），
          而 `NodePalletServo._push_pallet()` 读到 `stamp <= 0` **根本不 push**，
          所以"两路都是 0.0"在**节点链路**上根本到不了这一层，只对直接调窗 API
          成立。真正的理由就是上面这条：**坏对要有自己的名字**，而这条处置本身
          仍然更好，不必回退。）
        * **只在重试真的配上时才采信**。重试失败（`pair_none` / `pair_dt` / 又出
          `pair_stamp_zero`）就原样返回第一次的 `pair_stamp_zero`，诊断信息一个字
          不少 —— 最坏情况与不重试完全一样。
        * **只剔 `stamp <= 0` 的观测**，`> 0` 的一个不动。重试配出来的对因此必然
          满足"两侧都 > 0"，比原判据更严，不可能靠重试偷偷放进一个坏对。
        * 单侧为 0 的合法情形（仿真时间下第一帧）**根本走不到这里**：那时最小 dt
          的那一对不是"两侧 ≤ 0"。
        """
        found = nearest_pair([o.stamp for o in pals],
                             [b.stamp for b in boxes],
                             self._max_dt_s)
        if not (isinstance(found, PairReject)
                and found.code == "pair_stamp_zero"):
            return found, pals, boxes

        kept_pals = [o for o in pals if float(o.stamp) > 0.0]
        kept_boxes = [b for b in boxes if float(b.stamp) > 0.0]
        if len(kept_pals) == len(pals) and len(kept_boxes) == len(boxes):
            # 剔不掉任何东西（两侧各只有一个 0.0 之类）→ 不存在"好观测被挤掉"
            # 这回事，原样报出去。
            return found, pals, boxes

        retry = nearest_pair([o.stamp for o in kept_pals],
                             [b.stamp for b in kept_boxes],
                             self._max_dt_s)
        if isinstance(retry, tuple):
            return retry, kept_pals, kept_boxes
        return found, pals, boxes

    def _newer_than_last(self):
        """只留**比上次报出去的那一对更新的**观测（严格 `>`，同 stamp 也算旧的）。

        "更新"按各侧的 stamp 分别判：两路的时间戳各自单调时，`> 上次那一对的
        托盘 stamp` 就是"还没被消费掉的托盘观测"，箱子侧同理。这正是"每一帧只
        报一次、且总是报最新的那一帧"的落地方式 —— 见模块 docstring ①。
        """
        last_pallet, last_box = self._last_key
        return ([o for o in self._pallets if float(o.stamp) > last_pallet],
                [b for b in self._boxes if float(b.stamp) > last_box])

    # ------------------------------------------------------------------ 取
    def resolve(self, now: float = 0.0) -> Union[PairedFrame, PairReject, None]:
        """找最接近的一对；配上就把它推进窗并返回窗上的平均。

        返回：
          * `PairedFrame` —— 成对成功（含平滑结果）
          * `PairReject`  —— 配不上，`code` 是 `pair_threshold` / `pair_none` /
            `pair_nan` / `pair_stamp_zero` / `pair_dt` / `stale`
          * `None`        —— **没有新的配对可报**

        ⚠️ **`None` 的两种含义在返回类型上不可区分**（首对报出之后）：

          1. **同一对已经报过**，缓存里还没有更新的观测 —— 正常，每 tick 都调。
          2. **某一侧没有新观测**（托盘检测停了 / 箱子检测停了 / 那一路挂了）。

        情形 2 是**真的**：首对成功之后，任一侧停摆都会让 `_newer_than_last()`
        那一侧为空 → `_best_pair` 得到 `pair_none` → 这里返回 `None`，而且
        **永远**如此（`_last_key` 不回退，停摆侧再也不会给出比水位线更新的观测）。
        两条路各自的用例在 `tests/test_pairing.py` 里钉住了这个语义。

        **后果必须让调用方知道**：伺服节点每 tick 拿到 `None` 就静默 `RUNNING`，
        黑板停在旧值 —— **操作员看不到"检测器挂了"**。

        **所以 liveness（"检测器还活着吗"）判据不在本层。** 本层只有"有没有新的
        配对可报"这一个信息，`None` 把"正常去重"和"某路停摆"压成同一个返回值，
        这是**刻意的**：`None` 是 Task 5 依赖的契约（见 task-5-brief Step 10），
        本轮不改返回类型、也不为区分二者新造返回值（那是接口变更）。

        责任划分：

        * **本层**只保证"每一帧报一次、且报的是最新的那一帧"，并把两种含义
          写在这里。
        * **`PalletFrameWindow` 的调用方（Task 5 的 `NodePalletServo`）** 负责
          liveness：拿"距上次出数过了多久"（或上游观测的版本号/时间戳）自己判
          超时并告警 —— 那是**节点层**的信息，本层看不到"操作员想不想被告警"。
        * `stale_s` **顶不上这个用**：它判的是"配上的这一对有多旧"，判在去重
          **之后**，已消费的对根本走不到它 —— 见下面 `stale` 那段与
          `PalletFrameWindow` 的 `stale_s` 参数说明。

        时钟**整体回跳**（bag 循环 / 换时间源）后，`_last_key` 变成未来值，
        此后所有观测都"不更新"→ 本函数**永久返回 `None`**。唯一出路是 `reset()`
        （见它的 docstring：时钟回跳必须 reset）。
        """
        if self._last_key is not None:
            # 主判据：**只看"比上次报出去的那一对更新的观测"**。
            #
            # 为什么不先在全量缓存上挑一次：`nearest_pair` 并列 dt 时"先遇到者
            # 胜"，而缓存里越旧的观测下标越小 —— 稳态下（每一对的 dt 都是 0）
            # 全量挑出来的**永远是缓存里最早的那一对**，报过之后每一帧都还是
            # 它。要么返回 `None` 让窗永久沉默，要么每个 tick 重报同一对（版本号
            # 一直涨）。这两种都是上一轮 `pair_stamp_zero` 那条同族 bug 的翻版。
            #
            # 换成"只看更新的观测"之后，这条也自动成立：`_best_pair` 在这里
            # 挑出的对必然**严格新于**上次那一对，所以"同一对不重复报"是结构性
            # 保证的，不是靠事后比对挡下来的。
            new_pals, new_boxes = self._newer_than_last()
            found, pals, boxes = self._best_pair(new_pals, new_boxes)
            if isinstance(found, PairReject):
                if found.code == "pair_none":
                    # 只有一边推进了（托盘检测慢的时候箱子会一直领先几百毫秒），
                    # 这是**稳态**而不是故障 —— 报 `pair_none` 只会每个 tick 刷
                    # 一条"托盘 0 个观测"（还会把缓存里真实的观测数说成 0）。
                    # 没有比已报的那一对更新的配对 = 没有新闻。
                    return None
                # `pair_dt` 之类是真的对不上（时间戳跳了/两路失步），如实报，
                # 不能吞 —— 吞掉就是"永远沉默"。
                return found
        else:
            # 还没报过任何一对：在**全量缓存**上按原判据挑（`nearest_pair` 自己
            # 的判据不变、顺序不变）。空窗报 `pair_none` 会把"还没开始"说成
            # "配不成对"，这里返回 `None`；只有一边有观测时 `pair_none` 照常
            # 报出来（那是可指认的故障）。
            if not self._pallets and not self._boxes:
                return None
            found, pals, boxes = self._best_pair(list(self._pallets),
                                                 list(self._boxes))
            if isinstance(found, PairReject):
                return found
        i, j = found
        pallet, box = pals[i], boxes[j]
        key = (float(pallet.stamp), float(box.stamp))

        if self._stale_s > 0.0 and now > 0.0:
            # ⚠️ `stale_s` 是**延迟闸**，不是**存活闸**。它问的是"**配上的这一对**
            # 有多旧"（`now - pallet.stamp`），不是"检测器还活着吗"：
            #
            # * 它判在**去重之后** —— 已经报过的那一对在 `_newer_than_last()` 那
            #   一层就被挡掉了，根本走不到这里，所以"某路停摆"永远不会变成
            #   `stale`，而是变成 `None`（见 `resolve()` 的 docstring）。
            # * `max_dt_s` 判在**它之前**：`max_dt_s=0.05` + 500ms 的托盘延迟，
            #   那一对会先被判 `pair_dt`（两路差 0.5s 远超 0.05），`stale` 轮不到。
            #
            # 所以配在"慢但健康"的检测器上时它**会误报**，而"检测器死了"它又
            # 一个都不报。Task 5 配它之前先把 `now` 的口径定死（`now` 必须是
            # **托盘 stamp 的同一个时钟**，且 `stale_s` 要大于该检测器的正常
            # 端到端延迟），并把 liveness 告警做在节点层（本层给不出这个信息）。
            age = float(now) - float(pallet.stamp)
            if age > self._stale_s:
                return PairReject(
                    "stale",
                    f"托盘观测已经 {age:.3f}s 没更新（阈值 {self._stale_s:g}s）"
                    f" —— 检查检测器是不是卡了/挂了。**不要靠外推救**")

        self._pairs.append((pallet, box))
        self._last_key = key
        return average_pairs(list(self._pairs), self._agg)

    # ------------------------------------------------------------------ 状态
    @property
    def n_pairs(self) -> int:
        """窗里现在有几对。上限是构造时给的 `window`（与 `pair_cache` **互相
        独立**，见类 docstring），所以 `window=8, pair_cache=5` 时这里能到 8。"""
        return len(self._pairs)

    def reset(self) -> None:
        """清空两路缓存与成对序列（换场景/重进 `initialise()` 时用）。

        ⚠️ **时钟整体回跳（bag 循环播放 / 换时间源 / 重进 `initialise()`）之后
        必须调它**，否则本窗会**永久静默**：`_last_key` 是"已经消费掉的观测"的
        水位线（见 `_newer_than_last()`），回跳之后新来的观测 stamp 全都**小于**
        它，于是 `resolve()` 永远返回 `None`（"没有新的配对可报"），伺服黑板停在
        回跳前那一帧的值上，**没有任何报错**。回跳不是本层能自己发现的事 ——
        本层只看得见 stamp，分不清"时间倒退了"与"检测器没出数"，两者都长成
        `None`（见 `resolve()` 的 docstring）。

        因此这条前提是**给调用方（`NodePalletServo`）看的**：
        `initialise()` / 重进流程时必须 `reset()`。**调用方已经接了** ——
        `NodePalletServo.initialise()` 里就是 `self._pairing.reset()`
        （`node_pallet_servo.py:962`）；本层只是把这条前提写下来，
        换一个调用方时同样成立。

        不调 `reset()` 的**唯一**出路就是等 `_last_key` 被自然回写，而它只会被
        "严格更新的观测"回写 —— 回跳之后这种观测不存在，所以是死局，只能 reset。
        """
        self._pallets.clear()
        self._boxes.clear()
        self._pairs.clear()
        self._last_key = None
