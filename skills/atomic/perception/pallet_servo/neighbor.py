#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""邻居箱子伺服：托盘被遮死时，拿上/侧邻居当参照算同一套三个被控量。

`algorithm.py` 管**托盘路线**（托盘台面还看得见），本模块管**邻居路线**（托盘被
手里那个箱子挡死）。两条路线出的都是图像空间的这三个量：

    e_bottom_px  底边方向上的带符号差（像素）
    e_right_px   排方向上的带符号差（像素）
    theta_rad    手里箱底边角 − 参照边角，折进 (-pi/2, pi/2]（与托盘路线同口径）

与 `algorithm.py` 同一套约定：**纯函数、零框架依赖、不记日志**。不 import ROS /
onnxruntime / 模型，`Reject` 与诊断量一律作为**返回值**给出去，由上层决定怎么记
（这正是 `Reject` 存在的理由）。四角一律走 `algorithm.box_corners()` —— 托盘路线
和邻居路线对"哪四个点、什么顺序"必须只有一份实现，各带一份必然分叉。

场景
====
5 个箱子，码放顺序 1→5。托盘在画面里（**这一张是机器人在托盘下方时看的**）::

        +--------+--------+
        |   5    |        |
        +--------+   4    |
        |   2    |        |
        +--------+--------+
        |   3    |   1    |
        +--------+--------+

* **托盘还看得见**时 -> 走**托盘路线**（`algorithm.servo_error`）。
* **托盘被手里这个箱子挡死**时 -> 走**邻居路线**（本模块）。
  ⚠️ **哪一档走哪条路只看 `servo_slots`**（当前板子 2/4/5 档是 `neighbor`）；
  本模块只认 `mode` / `side`，不认档位号。
* **换到托盘另一侧时**（本布局在放图上标 4 的箱子之前），相机跟着转 180°，
  画面上下左右全反。所以图上写的左右只对换侧前的视角成立。本模块一律按
  **画面**说话，`servo_slots` 里换侧后那几档的 `side` 要按翻转后的画面填
  （那张表在 `board.json`，不在本层）。

⚠️ 下面的图上编号与「邻居关系」表都是**物理位置编号**，不是服务档位号 ——
   当前板子按操作员要求把服务档位 2/3 对调了（call 2 走邻居 `right`，
   call 3 走托盘），服务号与物理号不要混用。

邻居关系（操作员 2026-10-17 确认）::

    手里箱  上邻居  侧邻居
      3       2      1(右)
      4       1      无       <- 2 与它不对位；那一刻 5 还没放
      5       2      4(左)    <- 上邻居是 2 不是 3（不同排，中间隔着 2）

三个被控量（**只保证这三个量与托盘路线一致**）
============================================
诊断量两条路线各是各的：托盘路线的 `ref_bottom_px` / `pallet_center_px` 之类，
邻居路线给不出来（托盘本来就没看见），所以那些字段留空，`warn` 里注明
`source=neighbor`，叠加图据此区分这一帧是哪套算的。

参照边按"靠机器人一侧"定义：

* **底边** = 手里箱四条边里**背向**上邻居的那条（离机器人近）
* **顶边** = 朝向上邻居的那条
* **近边** = 邻居**朝着手里箱**的那条边

::

    e_bottom_px = 加权平均(t1, t2)
        t1 = (侧邻居.底边 − 手里箱.底边) 沿**侧邻居底边法向**的垂直距离
        t2 = (上邻居.底边 − 手里箱.顶边) 沿**上邻居底边法向**的垂直距离
    e_right_px  = 加权平均(t1', t2')
        t1' = (手里箱.近边中点 - 侧邻居.近边中点) . u_side
        t2' = (手里箱.顶边中点 - 上邻居.底边中点) 沿**接触边方向**（朝画面右）
    theta_rad   = 加权平均( fold(角上邻居底边 - 角手里箱底边),
                            fold(角侧邻居底边 - 角手里箱底边) )

⚠️ **横向比的是"接触边中点"，不是"框中心"**（2026-10-17 真机第二踩）：手里箱在
夹爪里、上邻居在垛上，两者在图像里沿"列方向"天然错开几十像素；拿中心直接相减
会把这段深度差（外加透视/截断造成的旋转差）当成横向误差 —— 实测 slot4 一帧
中心差 −43.9px，真正的横向只有 −13px。换成两个**接触边中点**之后，深度差沿法向，
不进横向那一项。

⚠️ **底边两项是"该法向的垂直距离"，不是"中点差投影到 u_far"**（2026-10-17 真机
踩过）：`u_far` 带着横向偏移与透视，实测与竖直差 **27.5°**，投影会把"手里箱偏右
100px"漏进底边误差，把 62px 真值算成 165px —— 叠加图上目标底边高得离谱。
用参考边自己的法向后，横向分量不进这一项；同一帧两个邻居给出 61.9 / 55.6，对得上。
底边两项是「参照 − 手里」，横向两项是「手里 − 参照」—— 对应托盘路线两条**内法向**
的朝向不同（底边朝里 = `+u_far`；右边朝里 = 画面**左** = `−u_side`），不是笔误。
见下面的「方向与符号」。

权重 = 该邻居的**检测置信度**；配置那一侧检不到 -> 只用上邻居那一项。

`t1`/`t2` 为什么能平均（公式成立的关键）：手里箱沿 `u_far` 挪动 eps 后**两项都变
−eps、同号**。因为 `t2` 参照的是上邻居的**底边**（靠手里箱那条），不是顶边 ——
用顶边的话理想值是"一个行距"这个常量、符号还相反。⚠️ **手里的横向移动不改变
这两项**（垂直距离对横向平移不敏感），所以它们量的确实是同一件事。

⚠️ 但这条公式还依赖一个**布局前提**：同排的两个箱子必须从垛的**同一端**起算，
它们的底边才共线（理想值 0）。本布局是靠"视角翻转"凑上的，换布局时这条会立刻失效。

方向与符号（2026-10-17 操作员现场裁定）
======================================
* `u_far` = 手里箱中心**指向上邻居**的方向 = **远离机器人**（往里走）。
  ⚠️ **机器人换到托盘对面时它不翻**：相机跟着转 180°，画面左右反了、上下没反 ——
  "远"在两种视角下都还是画面里的"上"。所以两条路线、两种视角都用**同一套符号**，
  换视角不需要任何翻转（现场实测：以为要翻、结果反了）。
* `u_side` = `(1, 0)`，画面 +u（向右），与外侧在哪一侧无关。

**符号约定与托盘路线逐字一致**（`algorithm.servo_error` 那一套）::

    e_bottom = (参照底边 − 手里箱底边) 沿**参考边法向**的垂直距离
    e_right  = (手里箱     − 参照     ) · u_side

* `e_bottom > 0` = 手里箱**还没到位**（底边在参照线靠机器人这一侧）→ **前进**；
  推过头 → 负 → **后退**。判据与托盘路线相同：**底边在伺服边下方时发正、超过时发负**。
* `e_right > 0` = 手里箱**偏画面右** → 往左走。
* `theta_rad` = 手里箱底边角 − 参照边角，折进 `(-pi/2, pi/2]`。

⚠️ **两个量的减数不一样，这不是笔误**：它们对应托盘路线的两条**内法向**
（`n_bottom` 指向托盘内部 = 远离机器人；`n_right` 指向托盘内部 = 画面**左**）。
托盘路线是 `e = (参照 − 手里)·n`，把两条内法向代进去就是上面这两行。

两条判据（生产判据不含箱子的长宽比）
====================================
* **「相邻」**：一侧间隙 ≤ `max_gap` × **手里箱**该方向的跨度（默认 0.60）。
* **「对位」**：沿排方向与**手里箱**的重叠比例 ≥ `min_overlap`（默认 0.40）。
  分母是**手里箱**的跨度，不是取两者较小 —— 反例 B（拿 4 时 2 只重叠 100px，
  相对 4 的 310px = 0.32 < 0.40）就是靠这一条挡掉的。

⚠️ **两个阈值都按"伺服还没收掉误差时的真机画面"定**（2026-10-18 现场）：
- 手里箱离目标位还差 0.4 个箱深，**这 0.4 正是伺服要收掉的量**，间隙闸门
  必须大于它 —— 否则误差越大越找不到邻居，等于在需要邻居的时候把邻居关掉。
  实测这一帧 gy = 0.40× 箱深，0.35 会拒；隔一整排的箱子间隙 ≈ 1.0× 箱深，
  0.60 仍分得开。
- 同理，没到位时同排邻居沿列方向也被拉开，重叠只有 0.45；阈值为 0.50 时
  "找不到右邻居"。0.40 与反例 B 的 0.32 之间还有余量。

⚠️ 两条都是**相对**判据，**没有任何一条用到箱子的长宽比**。真机检测时相机斜看、
箱子常常只露出一部分，框的长宽比到不了真实比例；合成自检场景才用真实尺寸
（下面「真实尺寸」一节），那是为了让"谁挨着谁、谁对位、同排从哪端起算"贴合真实
托盘，判据本身不碰它。

* **tie-break 一律用中心距离最近**（侧邻居、上邻居都是），不用"重叠最多"：
  重叠比例相近时谁大谁小由几十个像素的抖动决定，实测出现过 0.60 vs 0.63 选错
  一次。
* **手里箱的副本**（四角与它重合的对象，多半是调用方又造了一个内容相同的
  `BoxObservation`）跳过并记 `dup_of_held`：不跳的话它会成为自己的邻居，中心
  重合、方向算不出来，静默给出一个"好好看着"的退化帧。

真实尺寸（**只在合成的自检场景里用**）
=====================================
箱子 **530 × 350 mm**（长 × 宽，操作员 2026-10-17 给的实际尺寸），比例 1.514。
**两列的摆法是转 90° 的**：

    左列（5 / 2 / 3）  长边**横**放 -> 350 深 × 530 宽  -> 宽高比 1.51
    右列（4 / 1）      长边**竖**放 -> 530 深 × 350 宽  -> 宽高比 0.66

两列的总深因此都是 ~1050mm（3×350 / 2×530），与图上两列底边齐平吻合。

⚠️ **这个长宽比只能用在合成数据里，生产判据绝不碰它**（理由见上一节）。

⚠️ 两列总深不等（3×350=1050 vs 2×530=1060），所以理想摆放不是严格 0：在**翻转后**
的画面里两者都改从垛的**远端**起算，差只剩 **10mm** -> 底边那一项带 **+6.3px** 的
固定偏置，`e_bottom` 上是 **+3.15px**（加权平均的一半）。**操作员 2026-10-17 裁定：
这点误差忽略不计，不做标定抵消。**

用法::

    from skills.atomic.perception.pallet_servo.neighbor import (
        NeighborSlot, NeighborError, neighbor_servo)

    r = neighbor_servo(candidates, held, NeighborSlot(side='right'))
    if isinstance(r, Reject):        # 这一帧算不出来，原因在 r.code / r.detail
        ...
    else:
        r.e_bottom_px, r.e_right_px, r.theta_rad

    # 自检（不需要模型、不需要 ROS）：
    python3 -m pytest skills/atomic/perception/pallet_servo/tests/test_neighbor.py -q
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np

from .algorithm import (
    BoxObservation,
    Reject,
    box_corners,
    edge_offset,
    fold_line_angle,
    line_angle,
)

# 手里箱某条边短于此像素数就该几何量退化。与托盘路线的 `MIN_EDGE_PX` 同一个用意：
# 兜的是"明显退化"，不是标定出来的阈值。
DEGENERATE_SPAN_PX = 8.0

_SIDES = ("left", "right", "none")


# --------------------------------------------------------------------------- #
# 档位
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class NeighborSlot:
    """一次邻居伺服调用用哪个档位：侧邻居在哪一侧、两条判据的松紧。

    `side` 只认 `left` / `right` / `none`（操作员 2026-10-17 裁定："只能配置左、
    右、无三者之一"）。`none` 时只留上邻居那一项（`e_right`/`theta` 的侧邻居项
    整个消失，见模块 docstring 的加权平均）。**非法值在构造时就抛** —— 这是启动
    时的配置错误，不该拖到第一帧变成每帧一次的几何退化。

    `side` 是**画面口径**：放 4、5 那两帧相机已翻 180°，谁在左谁在右按翻转后的
    画面填（理由见模块 docstring 的「场景」）。
    """

    side: str = "none"
    # 对位闸门：沿排方向与**手里箱**的重叠比例下限。
    # ⚠️ **0.40 而不是 0.50**（2026-10-18 真机）：手里箱还没到位时，它沿列方向
    # 差出去多少、这一侧的重叠就少多少 —— 真机实测 e_bottom 还没收掉 0.4 个箱深
    # 时，同排侧邻居的重叠只有 **0.45**，被 0.50 挡在门外（"找不到右邻居"）。
    # 操作员点名的反例（拿 4 时 2 不对位）实测是 **0.32**，0.40 仍然挡得住。
    min_overlap: float = 0.40
    # 相邻闸门：两框沿"贴边方向"的间隙上限 = 该值 × 手里箱该方向的跨度。
    # ⚠️ **0.60 而不是 0.35**（2026-10-18 真机）：这个间隙**正是伺服要收掉的
    # 前向误差**，开始时本来就有几十像素 —— 真机实测 0.40 个箱深被 0.35 拒掉。
    # 隔一整排的那个箱子间隙 ≈ **1.0 个箱深**，0.60 离它还有 0.4 的余量。
    max_gap: float = 0.60

    def __post_init__(self) -> None:
        if self.side not in _SIDES:
            raise ValueError(
                f"side 只能是 {'/'.join(_SIDES)}，收到 {self.side!r}")


# --------------------------------------------------------------------------- #
# 返回值
# --------------------------------------------------------------------------- #
@dataclass
class NeighborError:
    """一次邻居伺服差的完整结果：三个被控量 + 出图/排查要的诊断量。

    前三个是**被控量**（与托盘路线同名同单位）；`*_quad` 是三个箱子按
    `box_corners` 契约序的四角，给叠加图用；`warn` / `diag` 是人看的与排查用的
    —— 算法层不记日志，全部从这里带出去。
    """

    # —— 三个被控量 ——
    e_bottom_px: float
    e_right_px: float
    theta_rad: float
    # —— 诊断：三个箱子的四角（契约序 [右下, 左下, 左上, 右上]）——
    up_quad: np.ndarray
    side_quad: Optional[np.ndarray]
    held_quad: np.ndarray
    # —— 诊断：出图要用的三样（2026-10-17）——
    # `held_bottom_px` / `held_side_px` = 手里箱**参与计算的那两条边**。出图把它
    # 画成绿线，再沿 `e_*` 平移出黄线（"该到的位置"）—— 绿黄重合就是到位了。
    #
    # ⚠️ **必须由这里交出去，出图不许自己推**：节点曾经用 `algorithm.box_edges()`
    # （写死的 `corners[0:2]` / `corners[3],corners[0]`）去画"手里箱的底边"，而
    # 算法用的是 `_frame_of()` 推出来的那条（背向上邻居 / 朝向侧邻居）——
    # 相机翻 180° 时两者**不是同一条**，图上出现"一个选上一个选下"（实测踩过）。
    # **同一条边只能有一个定义**，所以这里把它交出去。
    held_bottom_px: np.ndarray
    # 侧邻居存在时才有。**没有时横向那一项是"中心对中心"**，手里箱那一侧没有边
    # —— 出图会自己拿一条穿过中心的短线当它的"边"（见 `render_neighbor_overlay`）。
    held_side_px: Optional[np.ndarray]
    # 出图把绿线（`held_bottom_px`）平移成"目标位置"用的方向：**参考边法向的
    # 加权平均**（指向"往里走"）。⚠️ **不是 `u_far`** —— `u_far` 带横向/透视偏移
    # （实测与竖直差 27.5°），沿它平移会把黄线斜着推出去、且垂距归不了零。
    u_bottom: np.ndarray
    # 横向黄线的平移方向：参与那一项（或两项）的**正方向**的加权平均 —
    # 侧邻居项是近边法向（朝右），上邻居项是接触边方向（朝右）。同样**不是
    # `u_side`**：画面一斜（真机 27°），沿图像 +u 平移会歪。
    u_side_shift: np.ndarray
    warn: List[str]
    diag: dict


# --------------------------------------------------------------------------- #
# 基础几何
# --------------------------------------------------------------------------- #
def _edges_of(corners) -> List[np.ndarray]:
    """四条边，第 i 条 = 角 i -> 角 (i+1)%4。"""
    q = np.asarray(corners, np.float64).reshape(4, 2)
    return [np.array([q[i], q[(i + 1) % 4]], np.float64) for i in range(4)]


def _center_of(corners) -> np.ndarray:
    return np.asarray(corners, np.float64).reshape(4, 2).mean(axis=0)


def _mid(edge) -> np.ndarray:
    return np.asarray(edge, np.float64).mean(axis=0)


def _span(edge) -> float:
    e = np.asarray(edge, np.float64)
    return float(np.linalg.norm(e[1] - e[0]))


def _unit(v) -> Optional[np.ndarray]:
    v = np.asarray(v, np.float64)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else None


def _angle_of(edge) -> float:
    """这条边的方向角（无向，折半圈）。复用 `algorithm.line_angle` 的口径。"""
    e = np.asarray(edge, np.float64)
    return line_angle(e[0], e[1])


def _wavg_angle(a: float, b: float, wa: float, wb: float) -> float:
    """两个无向直线角的**加权**平均（各自先折半圈，再按权重插值）。"""
    d = fold_line_angle(a - b)
    t = wa / (wa + wb) if (wa + wb) > 1e-9 else 0.5
    return fold_line_angle(b + t * d)


def _away_normal(edge, along) -> Optional[np.ndarray]:
    """参考边所在直线的单位法向，方向取**与 `along` 同向**。

    两个用处（同一个函数，别各写一份）：
      * 底边：`along = u_far`（往里走）—— 定 `e_bottom` 的正方向和黄线平移方向；
      * 近边：`along = u_side`（画面右）—— 定 `e_right` 的正方向和黄线平移方向。

    ⚠️ **必须用边自己的法向，不能直接拿 `u_far` 当法向**（2026-10-17 真机踩）。
    `u_far` 是"手里箱中心 → 上邻居中心"，它带着**横向偏移 + 透视**：实测那一帧
    `u_far` 与竖直差 **27.5°**（上邻居在画面左边 112px）。拿它当投影轴的话，
    「手里箱偏右」这 100px 横向误差会**漏进 e_bottom**，把底边误差放大到两倍多
    （实测 t1：真值 62px 被算成 165px）—— 叠加图上就是"目标底边高得离谱"。
    改用参考边自己的法向后，横向偏移在法向上的投影≈0，两项也才对得上
    （同一帧 t1=61.9 / t2=55.6，两个独立邻居给出同一个物理偏移）。

    ⚠️ 横向那一项以前是"中点差点乘 `u_side`"，同样会把沿边方向的深度差漏进来；
    刚性转 30° 的合成场景里它从 0 变成 −28px。改用**近边法向的垂距**后与旋转无关。
    """
    e = np.asarray(edge, np.float64)
    d = _unit(e[1] - e[0])
    if d is None:
        return None
    n = np.array([-d[1], d[0]], np.float64)
    return -n if float(n @ np.asarray(along, np.float64)) < 0.0 else n


def _edge_gap(ref_edge, held_edge, u_far) -> Optional[float]:
    """`(参照边 − 手里边)` 沿**参考边法向**的像素距离，正 = 手里箱还没到位。

    只取垂直分量：`edge_offset` 把手里边中点投到参考边所在直线上得到垂足，
    两个中点之差再点乘"往里走"的法向。横向偏多少都不影响这个数 —— 那正是
    `e_bottom` 该量的东西。
    """
    n = _away_normal(ref_edge, u_far)
    if n is None:
        return None
    return -float(edge_offset(ref_edge, held_edge, n).e_px)


def _conf(obs) -> float:
    """候选置信度（`BoxObservation.confidence`；取不到或不有限时按 1.0 等权）。

    只是**权重**，不是判据 —— 拿不到就当等权，别让它毁掉一帧。
    """
    try:
        c = float(obs.confidence)
    except (TypeError, ValueError, AttributeError):
        return 1.0
    return c if math.isfinite(c) else 1.0


def _gap(a, b) -> float:
    """两个区间的间隙（重叠时为负）。"""
    if a[1] < b[0]:
        return b[0] - a[1]
    if b[1] < a[0]:
        return a[0] - b[1]
    return -min(a[1] - b[0], b[1] - a[0])


def _overlap(a, b) -> float:
    """两个区间的重叠长度（无重叠为 0）。"""
    return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))


def _extent(q, axis) -> Tuple[float, float]:
    """四角在 `axis` 方向上的投影区间（min, max）。

    ⚠️ **不要用图像轴的 AABB 判"相邻/对位"**（2026-10-17 排查）：相机是斜看的
    （真机 `u_far` 与竖直差 9.8~27.5°），AABB 会把两个相邻框的投影**撑大**，
    于是"间隙"和"重叠"都变了味。合成场景刚性转到 40° 时，AABB 判据直接认错邻居
    （slot4 把 5 当成上邻居）。改成**沿手里箱自己的两条边方向**投影，判据就与
    画面怎么转无关 —— 物理上的"同一排/同一列"就是这两条轴。
    """
    vals = np.asarray(q, np.float64).reshape(4, 2) @ np.asarray(axis, np.float64)
    return float(vals.min()), float(vals.max())


# --------------------------------------------------------------------------- #
# 邻居指认
# --------------------------------------------------------------------------- #
@dataclass
class _Neighbors:
    """一次邻居指认的结果（模块内部）：选中的邻居 + 它们的四角 + 诊断量。"""

    up: Optional[BoxObservation]
    side: Optional[BoxObservation]
    held_q: np.ndarray
    up_q: Optional[np.ndarray]
    side_q: Optional[np.ndarray]
    diag: dict


def _find_neighbors(candidates: Sequence[BoxObservation], held,
                    slot: NeighborSlot) -> Union[_Neighbors, Reject]:
    """按"挨着" + "配置的那一侧"找邻居。

    备选 = 与手里箱**贴边**（一侧间隙小）且**对位**（另一侧与手里箱重叠过半）
    的框。侧邻居 = 备选里中心落在 `slot.side` 一侧的那个；上邻居 = 剩下的那个。

    操作员点名的两个反例都由"对位"这条挡掉：

    * 拿 5 时 3 虽然同列更远，但与 5 不对位（中间隔着 2）-> 不入选
    * 拿 4 时 2 与它只部分相接（不对位）-> 不入选

    ⚠️ **正常调用下 `held` 就是 `candidates` 里的那一个**（同一个对象）。但这里
    **不**拿"`held` 在不在候选里"当判据：黑板上的 `latest_box_obs` 与
    `latest_box_candidates` 里对应的那个是两条写入路径，是两个**内容相同的对象**
    （见接入契约 §2/§5）。所以真正的兜底是下面那条**中心重合**的判据：与手里箱
    中心几乎重合的候选直接跳过并记 `dup_of_held`，免得它变成自己的邻居（中心
    重合 -> 方向算不出来）时静默给出一个退化帧。

    四角一律走 `box_corners()`：`quad` 与非 `quad` 两条路径在这里没有分支。
    某个候选的四角取不出来（`Reject`）时**跳过它并计数**，不毁整帧。
    """
    hq = box_corners(held)
    if isinstance(hq, Reject):
        return Reject("neighbor_no_held",
                      f"手里箱的四角取不出来（{hq}）；手里箱必须是能过 "
                      f"box_corners 的 BoxObservation")
    hc = _center_of(hq)
    # ★ **判"相邻/对位"用手里箱自己的两条边方向**，不用图像轴 AABB
    #   （2026-10-17 排查）：相机斜看时 AABB 会把相邻框的投影撑大，刚性转到
    #   40° 就认错邻居。契约序里第 0、1 条边一定相邻，直接拿它们当两条轴。
    _e = _edges_of(hq)
    a_axis = _unit(_e[0][1] - _e[0][0])       # 排方向（宽）
    b_axis = _unit(_e[1][1] - _e[1][0])       # 列方向（深）
    if a_axis is None or b_axis is None:
        return Reject("neighbor_no_held",
                      "手里箱有一条边退化（长度 < 1e-9px），定不出它的两条轴")
    hx = _extent(hq, a_axis)
    hy = _extent(hq, b_axis)
    hspan_x = max(1e-9, hx[1] - hx[0])
    hspan_y = max(1e-9, hy[1] - hy[0])

    dup = 0
    bad_codes: List[str] = []
    cands = []
    for d in candidates:
        if d is held:
            continue
        q = box_corners(d)
        if isinstance(q, Reject):
            bad_codes.append(q.code)
            continue
        if float(np.linalg.norm(_center_of(q) - hc)) < 1e-6:
            dup += 1                       # 手里箱的副本，不是邻居
            continue
        x, y = _extent(q, a_axis), _extent(q, b_axis)
        gx, gy = _gap(hx, x), _gap(hy, y)
        # 对位判据一律相对**手里箱**的跨度（不是取两者较小）：
        # 4 与 2 只重叠 100px，相对 4 的 310px = 0.32 < 0.5 -> 挡住
        ox_frac = _overlap(hx, x) / hspan_x
        oy_frac = _overlap(hy, y) / hspan_y
        side_ok = (gx <= slot.max_gap * hspan_x
                   and oy_frac >= slot.min_overlap)
        up_ok = (gy <= slot.max_gap * hspan_y
                 and ox_frac >= slot.min_overlap)
        # 中心仍用**图像坐标**（`side` 是画面口径、`dcent` 是图像距离）
        cc = _center_of(q)
        cxc, cyc = float(cc[0]), float(cc[1])
        cands.append({'det': d, 'q': q, 'cx': cxc, 'cy': cyc,
                      'dx': cxc - float(hc[0]),
                      'dcent': float(np.hypot(cxc - float(hc[0]),
                                              cyc - float(hc[1]))),
                      'gx': gx, 'gy': gy, 'ox': ox_frac, 'oy': oy_frac,
                      'side_ok': side_ok, 'up_ok': up_ok})

    diag = {'n_cand': len(cands), 'dup_of_held': dup,
            'n_bad_corners': len(bad_codes), 'bad_codes': bad_codes,
            'held_span': (round(hspan_x, 2), round(hspan_y, 2)),
            'cands': [{k: v for k, v in c.items() if k not in ('det', 'q')}
                      for c in cands]}

    side_c = None
    if slot.side != 'none':
        want_right = (slot.side == 'right')
        pool = [c for c in cands if c['side_ok'] and abs(c['dx']) > 1e-6
                and (c['dx'] > 0) == want_right]
        if pool:
            # **中心距离最近**的那个（不能按"重叠最多"排 —— 重叠比例相近时
            # 谁大谁小由几十个像素的抖动决定，实测出现过 0.63 vs 0.60 选错的）。
            side_c = min(pool, key=lambda c: c['dcent'])

    up_c = None
    up_pool = [c for c in cands if c is not side_c and c['up_ok']]
    if up_pool:
        # **中心距离最近**的那个 = 紧挨着的那一排。
        # 不能按"重叠最多"排：垛是一列时上面可能还压着更远的箱子，它们的重叠
        # 比例同样够高（实测 0.60 vs 0.63），按重叠排会被抖动决定胜负。
        up_c = min(up_pool, key=lambda c: c['dcent'])

    diag['side_found'] = side_c is not None
    diag['up_found'] = up_c is not None
    if up_c is not None:
        diag['up_center'] = _center_of(up_c['q']).round(2).tolist()
    if side_c is not None:
        diag['side_center'] = _center_of(side_c['q']).round(2).tolist()
    if slot.side != 'none' and side_c is None:
        diag['side_missing'] = f'配置 {slot.side} 侧没找到对位的邻居，降级为只用上邻居'
    return _Neighbors(up=(None if up_c is None else up_c['det']),
                      side=(None if side_c is None else side_c['det']),
                      held_q=hq,
                      up_q=(None if up_c is None else up_c['q']),
                      side_q=(None if side_c is None else side_c['q']),
                      diag=diag)


# --------------------------------------------------------------------------- #
# 按邻居定参照边
# --------------------------------------------------------------------------- #
def _edge_facing(q_from, q_to) -> np.ndarray:
    """`q_from` 四条边里**中点离 `q_to` 中心最近**的那条（= 朝向对方的边）。"""
    tc = _center_of(q_to)
    return min(_edges_of(q_from), key=lambda e: float(np.linalg.norm(_mid(e) - tc)))


def _edge_matching(q_src, q_ref, edge_ref, u) -> np.ndarray:
    """`q_src` 里那条**与 `edge_ref` 平行、且沿 `u` 偏在同样一侧**的边。

    用途：侧邻居的"底边"。侧邻居与手里箱同排，它的底边应当与手里箱底边共线，
    所以按"垂直于 u_far + 偏置同号"去挑，而不是靠四条边的远近（远近会把**近边**
    挑出来，那是另一条边）。
    """
    c_src, c_ref = _center_of(q_src), _center_of(q_ref)
    u = np.asarray(u, np.float64)
    ref_off = float((_mid(edge_ref) - c_ref) @ u)
    ref_dir = _unit(np.asarray(edge_ref, np.float64)[1]
                    - np.asarray(edge_ref, np.float64)[0])
    best, best_score = None, None
    for e in _edges_of(q_src):
        d = _unit(e[1] - e[0])
        if d is None or ref_dir is None:
            continue
        par = abs(float(d @ ref_dir))                 # 与参考边平行的程度
        off = float((_mid(e) - c_src) @ u)
        same_side = (off * ref_off) >= 0.0
        score = par - (0.0 if same_side else 1.0) - 0.5 * abs(off - ref_off) / (
            abs(ref_off) + 1e-9)
        if best_score is None or score > best_score:
            best, best_score = e, score
    return best


def _frame_of(held_q, up_q, side_q) -> dict:
    """由邻居推出手里箱的关键边与方向。

    * `u_far` = 手里箱中心 -> 上邻居中心（单位向量）
    * `top`   = 朝向上邻居的那条边；`bottom` = 它的对面
    * `near_side` = 朝向侧邻居的那条边

    只吃四角的**几何**，与四条边按哪个角起头无关（下面几处都是遍历全部四条边
    挑的），所以契约序只需保证环是凸的、不自交的 —— `box_corners` 已经查过。
    """
    out: dict = {'u_side': np.array([1.0, 0.0])}
    if up_q is None:
        return out
    u_far = _unit(_center_of(up_q) - _center_of(held_q))
    if u_far is None:
        return out
    out['u_far'] = u_far
    top = _edge_facing(held_q, up_q)
    # 底边 = 手里箱里与顶边**近似平行**、沿 u_far 偏置**反号**的那条
    c = _center_of(held_q)
    ref_off = float((_mid(top) - c) @ u_far)
    top_dir = _unit(np.asarray(top, np.float64)[1] - np.asarray(top, np.float64)[0])
    cand = []
    for e in _edges_of(held_q):
        if e is top:
            continue
        d = _unit(e[1] - e[0])
        if d is None or top_dir is None:
            continue
        if abs(float(d @ top_dir)) < 0.7:
            continue
        if float((_mid(e) - c) @ u_far) * ref_off >= 0.0:
            continue
        cand.append(e)
    out['top'] = top
    out['bottom'] = (max(cand, key=lambda e: abs(float((_mid(e) - c) @ u_far)))
                     if cand
                     else min(_edges_of(held_q),
                              key=lambda e: float((_mid(e) - c) @ u_far)))
    if side_q is not None:
        out['near_side'] = _edge_facing(held_q, side_q)
    return out


# --------------------------------------------------------------------------- #
# 主入口
# --------------------------------------------------------------------------- #
def _clipped(q, image_size) -> bool:
    """四角是不是伸到画面外（检测框被边缘截断）。

    ⚠️ **截断的框，中心和边都是偏的**（2026-10-17 真机实测：slot4 的上邻居四角
    y 一直到 −27，检测出来 209x203；按比例真实应有约 230x286）。拿它当参照，
    误差会偏几十像素 —— 那正是"看着对齐了还显示几十像素"的来源之一。
    这里只**告警**（仍然出数）：视野就是这么摆的，拒了就没数可用了。
    """
    if image_size is None:
        return False
    try:
        w, h = float(image_size[0]), float(image_size[1])
    except (TypeError, ValueError, IndexError):
        return False
    p = np.asarray(q, np.float64).reshape(4, 2)
    return bool(p[:, 0].min() < 0.0 or p[:, 0].max() > w
                or p[:, 1].min() < 0.0 or p[:, 1].max() > h)


def neighbor_servo(candidates: Sequence[BoxObservation], held,
                   slot: NeighborSlot,
                   image_size=None) -> Union[NeighborError, Reject]:
    """算邻居路线的一帧伺服差。**不抛异常**：算不出来就返回 `Reject`。

    `candidates` 是 NMS 之后、选箱**之前**的全部候选（含手里那个），`held` 是
    手里那个箱子。判据（相邻/对位/tie-break）全部只在四角的几何上做，见
    `_find_neighbors` 与模块 docstring。

    `image_size=(W, H)` 可选：给了就检查**手里箱 / 上邻居 / 侧邻居**有没有伸出
    画面，伸出去的往 `warn` 里记一条（**不拒帧**）。截断的框中心与边都是偏的，
    误差会跟着偏几十像素 —— 不告警的话现场会以为"算错了"。

    `Reject.code` 四个取值：`neighbor_no_held`（手里箱的四角取不出来）/
    `neighbor_no_up`（没有合格的上邻居，**不乱挑一个凑数**）/
    `neighbor_degenerate`（上邻居与手里箱中心重合之类，参照方向定不出来）/
    `neighbor_edge_too_short`（手里箱的顶边或底边退化，量不出误差）。
    `detail` 里一律带**实际值与阈值** —— 只写一句"失败了"，到了上层就没法排查。
    """
    nb = _find_neighbors(candidates, held, slot)
    if isinstance(nb, Reject):
        return nb

    warn: List[str] = []
    diag = dict(nb.diag)
    diag['side_cfg'] = slot.side
    if diag['n_bad_corners']:
        warn.append(f"{diag['n_bad_corners']} 个候选的四角取不出来"
                    f"（{', '.join(diag['bad_codes'])}），已跳过；"
                    f"邻居是从剩下的候选里挑的")
    # 画面边缘截断：框的中心/角度/边长都会偏，参照边也就不可靠了。
    # ⚠️ **只报邻居、不报手里箱**：手里箱在夹爪上、离相机近，它的**下边**经常
    #    压在画面底边（实测 y 到 486、画布高 480）—— 那是正常取景，而底边只在
    #    有侧邻居时才参与计算。每帧都报一条会把真正要紧的告警淹掉。
    for _name, _q in (('上邻居', nb.up_q), ('侧邻居', nb.side_q)):
        if _q is not None and _clipped(_q, image_size):
            warn.append(f'{_name}被画面边缘截断 —— 它的中心/边可能有偏，'
                        f'这一帧的误差可信度下降')

    # ⚠️ 报的必须是**判据真正用的那个跨度**（沿手里箱列方向），不是图像 y 跨度 ——
    #    两者在斜视角下不等，消息里写错了会让人按错的数去核对。
    hspan_y = max(1e-9, float(diag['held_span'][1]))
    if nb.up is None:
        gap_thr = slot.max_gap * hspan_y
        # ⚠️ **两个判据的值都要报**（2026-10-18 真机踩）：上邻居看的是
        # `gy`（列方向间隙）+ `ox`（排方向重叠），而从前这里打的是
        # `gy/oy` —— `oy` 是**侧邻居**那条判据用的，现场照着它排查会得出
        # "重叠明明够了"的错误结论。两组四个数一起打，多几个字符而已。
        per = ', '.join(
            f"gy={c['gy']:.0f}px({c['gy'] / hspan_y:+.2f}×) ox={c['ox']:.2f}"
            f" | gx={c['gx']:.0f}px({c['gx'] / max(1e-9, float(diag['held_span'][0])):+.2f}×)"
            f" oy={c['oy']:.2f}"
            for c in diag['cands']) or '没有候选'
        return Reject(
            "neighbor_no_up",
            f"没有上邻居：{diag['n_cand']} 个候选"
            f"（dup_of_held={diag['dup_of_held']}）里没有一个同时满足"
            f"列方向间隙 ≤ {slot.max_gap:g}×手里箱在列方向的跨度 "
            f"{hspan_y:.1f}px = {gap_thr:.1f}px "
            f"且沿排方向与手里箱重叠 ≥ {slot.min_overlap:g}"
            f"；逐候选 [gy/ox 是上邻居判据，gx/oy 是侧邻居判据] [{per}]")

    hq, uq = nb.held_q, nb.up_q
    sq = nb.side_q
    fr = _frame_of(hq, uq, sq)
    bottom, top, u_far, u_side = (fr.get('bottom'), fr.get('top'),
                                  fr.get('u_far'), fr['u_side'])
    if bottom is None or top is None or u_far is None:
        return Reject(
            "neighbor_degenerate",
            f"由上邻居推不出参照系：u_far={'有' if u_far is not None else '没有'}"
            f"、顶边={'有' if top is not None else '没有'}"
            f"、底边={'有' if bottom is not None else '没有'}"
            f"（上邻居中心与手里箱中心重合时 u_far 无定义，那正是 dup_of_held "
            f"那条判据要拦的情况）")
    for name, e in (('底边', bottom), ('顶边', top)):
        if _span(e) < DEGENERATE_SPAN_PX:
            return Reject(
                "neighbor_edge_too_short",
                f"手里箱的{name}只有 {_span(e):.1f}px（阈值 "
                f"{DEGENERATE_SPAN_PX:.0f}px）—— 这条边退化，量不出误差")
    d_ang = abs(fold_line_angle(_angle_of(bottom) - _angle_of(top)))
    if d_ang > math.radians(30.0):
        warn.append(f'底边与顶边不平行（差 {math.degrees(d_ang):.1f}°），'
                    f'手里箱的四角可能有误')

    # ---- 上邻居那两项（永远参与）----
    w_u = _conf(nb.up)
    u_bottom = _edge_facing(uq, hq)                # 上邻居朝手里箱的边 = 它的底边
    # ⚠️ **取"边到边的垂直距离"，不是"中点差投影到 u_far"**（2026-10-17 真机踩）。
    #    u_far 带横向/透视偏移（实测与竖直差 27.5°），投影会把横向误差漏进来，
    #    把 62px 的底边误差算成 165px。法向由**参考边自己**定，见 `_away_normal`。
    #    符号（2026-10-17 操作员裁定，与托盘路线同一套）：手里箱还没到位 -> **正**
    #    -> 前进；推过头 -> 负 -> 后退。
    t2 = _edge_gap(u_bottom, top, u_far)
    if t2 is None:
        return Reject(
            "neighbor_degenerate",
            "上邻居的底边退化（长度 < 1e-9px），定不出法向 —— 量不出底边误差")
    n_up = _away_normal(u_bottom, u_far)
    n_side = None
    n_near = None
    # 横向**不翻**：`u_side` 是画面 +u（右），而托盘路线的右内法向指向画面**左**，
    # 所以 `e_right = (手里 − 参照)·u_side` 才是同一套约定（正 = 偏右）。
    #
    # ⚠️ **比「接触边中点」而不是「框中心」**（2026-10-17 真机第二踩）。
    #    拿两个**不同深度**的框中心直接相减：手里箱在夹爪里、上邻居在垛上，图像里
    #    两者沿"列方向"错开几十像素；再叠上透视/截断带来的旋转差，横向分量里会混进
    #    一大堆"不是横向误差"的东西。实测 slot4 那一帧：中心差 −43.9px，
    #    真正横向只有 −13px（其中 29px 是"上邻居中心 vs 上邻居底边中点"的几何差）。
    #    改用**手里箱顶边中点 − 上邻居底边中点**（这两个点才是"贴上"时要重合的），
    #    再取**沿接触边方向**的分量 —— 深度差沿法向，不进这一项。
    d_top = _unit(top[1] - top[0])
    if d_top is not None and float(d_top @ u_side) < 0.0:
        d_top = -d_top                    # 统一朝画面右，正号 = 偏右
    t2r = (None if d_top is None
           else float((_mid(top) - _mid(u_bottom)) @ d_top))
    if t2r is None:
        return Reject(
            "neighbor_degenerate",
            "手里箱的顶边退化（长度 < 1e-9px），定不出接触边方向 —— 量不出横向误差")
    # theta 与托盘路线同一套（2026-10-20 翻正）：**手里箱 − 参照**。
    th2 = fold_line_angle(_angle_of(bottom) - _angle_of(u_bottom))
    w_s = 0.0
    t1 = t1r = None
    th1 = None

    if sq is not None:
        w_s = _conf(nb.side)
        s_near = _edge_facing(sq, hq)              # 侧邻居朝手里箱的边
        s_bottom = _edge_matching(sq, hq, bottom, u_far)
        t1 = _edge_gap(s_bottom, bottom, u_far)
        if t1 is None:
            warn.append('侧邻居的底边退化（长度 < 1e-9px），'
                        '底边误差降级为只用上邻居那一项')
        else:
            n_side = _away_normal(s_bottom, u_far)
        # ⚠️ **横向也是"边到边的垂直距离"**（2026-10-17 真机第二踩）：两条近边
        #    在到位时应当**重合**，所以差就是它们之间的垂距。从前写成
        #    `(手里近边中点 − 侧邻居近边中点)·u_side` —— 两个中点沿**边方向**错开
        #    一整个"深度差"，画面一斜（实测真机 27°）这段就漏进横向：
        #    合成场景刚性转 30° 时，$t1'$ 从 0 变成 −28px。
        n_near = _away_normal(s_near, u_side)      # 垂直近边、朝画面右
        t1r = (None if n_near is None
               else float(edge_offset(s_near, fr['near_side'], n_near).e_px))
        th1 = fold_line_angle(_angle_of(bottom) - _angle_of(s_bottom))

    if t1 is None or t1r is None:                   # 只有上邻居（或侧邻居那条边退化）
        if t1r is None and sq is not None:
            warn.append('侧邻居的近边退化（长度 < 1e-9px），'
                        '横向误差降级为只用上邻居那一项')
        e_b, e_r, th = t2, t2r, th2
    else:
        wsum = w_s + w_u
        if wsum > 1e-9:
            e_b = (w_s * t1 + w_u * t2) / wsum
            e_r = (w_s * t1r + w_u * t2r) / wsum
        else:
            e_b, e_r = (t1 + t2) / 2.0, (t1r + t2r) / 2.0
        th = _wavg_angle(th1, th2, w_s, w_u)

    # "目标位置"的平移方向 = 参与那一项（或两项）的**参考边法向**的加权平均。
    # 两项都在时两个法向只差几度；只用上邻居时就是它自己的法向。
    if n_side is not None and (w_s + w_u) > 1e-9:
        u_bottom = _unit(w_u * n_up + w_s * n_side)
    else:
        u_bottom = _unit(n_up)
    if u_bottom is None:            # 两个法向正好反向 —— 几何已经不对劲了
        u_bottom = _unit(u_far)

    # 横向黄线的平移方向同理：侧邻居项用近边法向（朝右），上邻居项用接触边
    # 方向（朝右），按同样的权重平均。`u_side`（图像 +u）只在都拿不到时兜底。
    dirs = []
    if t1r is not None and n_near is not None:
        dirs.append((w_s, n_near))
    if t2r is not None and d_top is not None:
        dirs.append((w_u, d_top))
    if dirs:
        acc = sum((wd * np.asarray(dd, np.float64) for wd, dd in dirs),
                  np.zeros(2, np.float64))
        u_side_shift = _unit(acc)
    else:
        u_side_shift = None
    if u_side_shift is None:
        u_side_shift = _unit(u_side)

    for nm, e in (('底边', bottom), ('顶边', top)):
        if _span(e) < 3.0 * DEGENERATE_SPAN_PX:
            warn.append(f'{nm}只有 {_span(e):.0f}px，偏短')

    diag.update({'t1_bottom': None if t1 is None else round(t1, 3),
                 't2_bottom': round(t2, 3),
                 't1_right': None if t1r is None else round(t1r, 3),
                 't2_right': round(t2r, 3),
                 'w_side': w_s, 'w_up': w_u,
                 'bottom_px': bottom, 'top_px': top,
                 'near_side_px': fr.get('near_side'),
                 'u_far': u_far, 'u_side': u_side, 'u_bottom': u_bottom,
                 'u_side_shift': u_side_shift})
    if sq is None:
        diag.setdefault('side_missing', '本档位配置就是 none（无形变项）')
    return NeighborError(e_bottom_px=e_b, e_right_px=e_r, theta_rad=th,
                         up_quad=uq, side_quad=sq, held_quad=hq,
                         held_bottom_px=bottom,
                         held_side_px=None if sq is None else fr['near_side'],
                         u_bottom=u_bottom, u_side_shift=u_side_shift,
                         warn=warn, diag=diag)
