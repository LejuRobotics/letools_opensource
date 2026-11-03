#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""细化器的**建议替换版**（相对 `obbval/refine.py` 只改两处，见下）。

用法与 `refine.refine` 完全同构：

    from pallet_detect.refine_ridge import refine2
    f5, diag = refine2(f4, color, k, gmag=grad_mag(color))

## 与 `refine.py` 的差别（只有两处，都有全量 601 帧实测）

1. **量测规则：软规则代替"取最近"**。

       score(tt) = mag(tt) * exp(-tt²/(2σ²)),   σ = 4.5 px

   "取最近"（`argmin|tt|`）在初值离得远时会撞上木纹/板条缝；"取最强"又会跳到
   30px 外的背景边。指数项是"初值到真值边有多远"的先验，把两者折中。
   单独只换这一条规则、其余全不变：**11.23px -> 11.01px**（+IRLS 剔点后）。
   再叠上第 2 条（2 条边也解 + 小窗口）到 **8.05px**。

2. **允许只量到 2 条边也解**（`MIN_LINES = 2`，原来硬性要求 3 条），
   并把 2 条边时的**搜索窗口收到 ±8mm / ±1°**（`MOVE2_MM` / `ROT2_DEG`）。
   两条边只给 4 个约束解 6 自由度，没被约束的两维会乱跑（不加窗口实测能跑出
   47px）；窗口一收，解变成"贴着这两条边、其余自由度基本不动"。
   实测：**11.01px -> 8.05px**，逐帧 好 335 / 差 111 / 平 155。

## 不加什么（试过，都是负收益，别再走）

* **没有 IRLS 剔点**：软规则下每个采样点只取一个峰，板条缝造成的假峰已经在
  `exp(-tt²/2σ²)` 那一步被压低；再加一轮 |r|<2px 剔点会把"离初值最远但正确"的
  那批点先剔掉（实测 11.01px -> 11.66px，更差）。
* **梯度方向门控**（要求梯度方向与边法向 cos>0.7/0.85）：13.3px，更差。
* **着色/木色通道的梯度**（Lab-a、R-max(G,B)、三通道取最大）：都 ≤ 11.06px，无意义改善。
* **只用"源边"或子集边 + 标量约束**：秩亏，22~190px。框的四条边**等权**是对的。
* **用框区域深度拟合台面平面给法向**：法向反而更差（4.13° 中位 / 25.9° p90，
  而 meta nrm 是 3.81° / 7.17°），端到端 16.51px。

## 已知的不足（如实记账）

* **430/601 帧量不到 2 条边**（贴边、纹理不共线），这些帧保持 4-DoF 初值不变，
  误差停在 13.9px。这是整条链路的**真正瓶颈**：开火的 174 帧是 4.97px，
  没开火的 427 帧是 13.93px，而全体中位 11.23 就是这两群的混合。
* **σ=4.5 与窗口 8/1 是在全部 601 帧上选的**。按包留一（LOBO，`E59c.py`：
  每个包都不参与自己的参数选择）复现：**基线 11.24 → 8.07px**，
  逐帧 好 335 / 差 110 / 平 155；13 个包里 12 个都选中 `σ=3.5 / 8mm / 1°`。
  **收益不是调出来的**，但部署前建议用部署那台机位复核一次 σ。
* 时间：本机（4090 + `OMP_NUM_THREADS=1`）约 38ms/帧，比原 refine 的 21ms 多 17ms
  （`exp()` 在 Python 内层循环里，且 2 条边也解会把迭代跑满）。Orin 上需要实测；
  实在要省可以把内层改成查表或 numpy 向量化。
* **最差帧没改善**：p95 从 26.9 到 27.6，max 从 47.9 到 52.4（同一批难帧，
  贴边 3 个角 + 纹理不共线）。中位改善主要来自 p10~p50 那段。
"""
from __future__ import annotations

import numpy as np

from .geom import _corners_mm, _project_px
from .solve_obb import PALLET_SIZE_MM

import cv2

BLUR_SIGMA = 3.0
SEARCH_HALF_PX = 30
SAMPLE_STEP_PX = 8.0
EDGE_MARGIN = 0.12
GRAD_MIN = 40.0
FIT_MAX_RESID_PX = 2.5
MIN_PTS = 6
MAX_ITER = 2
RESID_RMS_MM = 60.0
MAX_MOVE_MM = 1500.0
MAX_ROT_DEG = 90.0

SIGMA_PX = 4.5      # 软规则的尺度：允许"离初值这么远"的峰与"更近的弱峰"竞争
MIN_LINES = 2       # 原来要求 3 条边；2 条也解（huber + 小步长抗欠定）
# 2 条边只给 4 个约束解 6 自由度：没被约束的两维会乱跑（实测能跑出 47px）。
# 所以少边时把搜索窗口收紧。实测扫描（601 帧中位）：1500mm/90° -> 8.84px、
# 60/5 -> 8.74、15/2 -> 8.61、**8/1 -> 8.05**、1/0.1 -> 8.74（太紧又回不去）。
MOVE2_MM = 8.0
ROT2_DEG = 1.0


# ⚠️ 不要整条丢弃任何边。托盘右边**绝大多数帧是好的**（极性一致性 98%），
# 丢了它等于把 2 个约束白送，全量中位从 4.63 退到 7.44（实测）。这里只保留一个
# 开关，默认空，供做对照实验用。
DROP_EDGES = ()

# 极性闸门：一条边上"往外变暗"（极性正确）的采样列少于这个比例时，**整条边不用**
# （= 你说的"没有极性对的边就直接忽略"）。None = 关闭，量测退回纯软规则。
# 动机：托盘上摆的箱子侧边就在托盘边附近、极性相反；`2026-10-10-11-44-12_00140`
# 的右边**一个极性正确的峰都没有**（真值右边在 x=639，贴在 640 宽的图像边界上），
# 而软规则把 20px 内侧的箱侧边当成右边，量出的线偏内 24.7px，整块解随之缩水 7%。
POL_GATE = None

# --- 新量测（默认打开）------------------------------------------------------
# 逐列：① 只保留极性正确的峰（沿外法向往外变暗），② 只保留落在几何预测位置
# `|tt| <= POLAR_APERTURE_PX` 内的峰；两条都不满足 -> 这一列不投点。
# 整条边有效列占比 < POLAR_NEED -> 这条边交回 None（= 认为不可见，弃用）。
MEASURE_MODE = 'polar_aperture'
POLAR_APERTURE_PX = 20.0
POLAR_NEED = 0.4


def drop_edges(lines, drop=None):
    """把 `drop` 里的边整条置空。`drop=None` 时用模块级 `DROP_EDGES`（可在运行时改）。"""
    d = DROP_EDGES if drop is None else drop
    return [None if i in d else l for i, l in enumerate(lines)]


def signed_grad(color, blur=BLUR_SIGMA):
    """有符号梯度（gx, gy）。极性选择要用它，`grad_mag` 只有幅值不够。"""
    g = cv2.GaussianBlur(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), (0, 0), blur)
    return (cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3).astype(np.float32),
            cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3).astype(np.float32))


def _bil(A, u, v):
    h, w = A.shape
    if not (1 <= u < w - 1 and 1 <= v < h - 1):
        return None
    i, j = int(u), int(v)
    fu, fv = u - i, v - j
    return float(A[j, i] * (1 - fu) * (1 - fv) + A[j, i + 1] * fu * (1 - fv)
                 + A[j + 1, i] * (1 - fu) * fv + A[j + 1, i + 1] * fu * fv)


def _bil_grid(A, U, V, w, h):
    """`_bil` 的整块版：`U`/`V` 是 (n, m) 的浮点采样坐标，返回同形状的双线性插值。

    ⚠️ **索引必须夹进 `[0, w-2]`/`[0, h-2]`**（写 `w-1` 会越界读下一行）。
    越界位置由调用方的 `ok` 掩码丢弃，所以夹过界的那几个数不会被用到 —— 但**夹**
    这一步不能省，`U` 有 ±aperture 的外扩，必然越界。

    与 `_bil` 的关系：在 `1 <= u < w-1`、`1 <= v < h-1` 的范围内两者结果**逐位相同**
    （`int()` 对正数就是 `floor()`）。范围外的行为不同 —— `_bil` 返回 `None`，
    这里返回夹过界的插值 —— 所以调用方**必须**自己用 `ok` 挡掉。
    """
    i = np.clip(np.floor(U).astype(np.int64), 0, w - 2)
    j = np.clip(np.floor(V).astype(np.int64), 0, h - 2)
    fu, fv = U - i, V - j
    return (A[j, i] * (1 - fu) * (1 - fv) + A[j, i + 1] * fu * (1 - fv)
            + A[j + 1, i] * (1 - fu) * fv + A[j + 1, i + 1] * fu * fv)


def measure_soft_polar(frame, gmag, k, gxy=None, sigma=SIGMA_PX, half=SEARCH_HALF_PX,
                       gmin=GRAD_MIN, maxrms=FIT_MAX_RESID_PX, need=0.5):
    """与 `measure_soft` 同，但**优先取"梯度方向正确"的那个峰**。

    物理规律（E94 实测，13 个包全部一致）：台面比它外面的地面亮，所以沿**外法向**
    梯度应当为**负**（往外走变暗）。现场托盘上摆的**纸箱**侧边就在托盘边附近、
    极性相反 —— 实测 `2026-10-10-11-44-12_00130`：真值右边 "往外变暗" 占 100%，
    箱侧边那条占 0%。

    做法：每个采样列沿法向扫一遍，同时记住
      * `best_any` —— 软规则选出的峰（= 现状，离预测最近）
      * `best_ok`  —— 其中**梯度方向正确**（往外变暗）的那个峰
    若某一列有 `best_ok` 就用它（这就是"丢掉极性反的后，再去找极性对的那条"），
    没有才退回 `best_any`。整条边要求至少有 `need` 比例的列用了 `best_ok`，
    否则交回 `None`（宁可不用，也不要混着用）。
    """
    h, w = gmag.shape
    px = _project_px(_corners_mm(frame, frame['W'], frame['H']), k)
    gx = gy = None
    if gxy is not None:
        gx, gy = gxy
    lines = []
    for i in range(4):
        a, b = px[i], px[(i + 1) % 4]
        L = float(np.linalg.norm(b - a))
        if L < 40.0:
            lines.append(None); continue
        d = (b - a) / L
        nv = np.array([-d[1], d[0]])
        if float(np.dot((a + b) / 2 - px.mean(axis=0), nv)) < 0:
            nv = -nv                                    # 外法向
        ts = np.linspace(EDGE_MARGIN, 1 - EDGE_MARGIN,
                         max(3, int(L * (1 - 2 * EDGE_MARGIN) / SAMPLE_STEP_PX)))
        P = np.array([a + t * (b - a) for t in ts])
        keep = (P[:, 0] > 2) & (P[:, 0] < w - 3) & (P[:, 1] > 2) & (P[:, 1] < h - 3)
        P = P[keep]
        if len(P) < MIN_PTS:
            lines.append(None); continue
        pts, used_ok, n_col = [], 0, 0
        for p in P:
            n_col += 1
            b_any = None; b_ok = None
            for tt in range(-int(half), int(half) + 1):
                u, v = p[0] + tt * nv[0], p[1] + tt * nv[1]
                if not (1 <= u < w - 1 and 1 <= v < h - 1):
                    continue
                val = float(gmag[int(round(v)), int(round(u))])
                if val < gmin:
                    continue
                sc = val * np.exp(-(tt * tt) / (2.0 * sigma * sigma))
                if b_any is None or sc > b_any[0]:
                    b_any = (sc, tt)
                if gx is not None:
                    vx = _bil(gx, u, v); vy = _bil(gy, u, v)
                    if vx is not None and (vx * nv[0] + vy * nv[1]) < 0.0:
                        if b_ok is None or sc > b_ok[0]:
                            b_ok = (sc, tt)
            pick = b_ok if b_ok is not None else b_any
            if pick is None:
                continue
            if b_ok is not None:
                used_ok += 1
            pts.append([p[0] + pick[1] * nv[0], p[1] + pick[1] * nv[1]])
        if POL_GATE is not None:
            # 只改"用不用这条边"这一个决策；量到的线仍是软规则那条（不做逐列硬筛，
            # E116 实测逐列硬筛会把好边也改坏：中位 4.72 -> 5.91）。
            if n_col == 0 or used_ok / n_col < POL_GATE:
                lines.append(None); continue
        elif n_col and used_ok / n_col < need:
            lines.append(None); continue
        if len(pts) < MIN_PTS:
            lines.append(None); continue
        arr = np.asarray(pts, np.float32)
        m = np.ones(len(arr), bool)
        for _ in range(4):
            if m.sum() < 3:
                break
            vx_, vy_, x0, y0 = cv2.fitLine(np.ascontiguousarray(arr[m]),
                                           cv2.DIST_L2, 0, 0.01, 0.01).ravel()
            n2 = np.array([-vy_, vx_], float); n2 /= np.linalg.norm(n2)
            m = np.abs((arr - np.array([x0, y0])) @ n2) < 2.0
        if m.sum() < MIN_PTS:
            lines.append(None); continue
        arr = arr[m]
        vx_, vy_, x0, y0 = cv2.fitLine(np.ascontiguousarray(arr),
                                       cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        n2 = np.array([-vy_, vx_], float)
        if float(np.sqrt(np.mean(((arr - np.array([x0, y0])) @ n2) ** 2))) > maxrms:
            lines.append(None); continue
        lines.append((float(n2[0]), float(n2[1]), -float(n2[0] * x0 + n2[1] * y0)))
    return lines


def measure_polar_aperture(frame, gmag, k, gxy, aperture=12.0, need=0.5,
                           sigma=SIGMA_PX, half=SEARCH_HALF_PX,
                           gmin=GRAD_MIN, maxrms=FIT_MAX_RESID_PX):
    """**按极性避错边，再在几何预测位置附近找极性对的边；找不到就整条边不用。**

    与 `measure_soft_polar` 的区别（这是关键），逐列三步：

    1. 只保留**极性正确**（沿外法向往外变暗）的峰 —— 这一步把"箱子侧边 / 地面接缝
       这类极性相反的假边"挡在外面，不再靠"离预测最近"去猜。
    2. 在保留下来的峰里，只接受落在**几何预测位置** `|tt| <= aperture` 内的 ——
       满足"符合几何尺寸"。这一步挡住"极性对但在 30px 外的背景边"。
    3. 本列两个条件都不满足 -> **这一列不投点**（不退回任何极性错的峰）。

    整条边若有效列占比 < `need`，交回 `None`（= 干脆忽略这条边）。
    阈值取 `need=0.5` 是实测：`2026-10-10-11-44-12_00140` 的右边
    **一个极性正确的峰都没有**（真值右边 x=639 正好压在 640 宽的图像边界上，
    外面根本没有像素），有效列 0% -> 这条边被丢掉；而左上两条边接近 100%。
    """
    h, w = gmag.shape
    gx, gy = gxy
    px = _project_px(_corners_mm(frame, frame['W'], frame['H']), k)
    lines = []
    for i in range(4):
        a, b = px[i], px[(i + 1) % 4]
        L = float(np.linalg.norm(b - a))
        if L < 40.0:
            lines.append(None); continue
        d = (b - a) / L
        nv = np.array([-d[1], d[0]])
        if float(np.dot((a + b) / 2 - px.mean(axis=0), nv)) < 0:
            nv = -nv
        ts = np.linspace(EDGE_MARGIN, 1 - EDGE_MARGIN,
                         max(3, int(L * (1 - 2 * EDGE_MARGIN) / SAMPLE_STEP_PX)))
        P = np.array([a + t * (b - a) for t in ts])
        keep = (P[:, 0] > 2) & (P[:, 0] < w - 3) & (P[:, 1] > 2) & (P[:, 1] < h - 3)
        P = P[keep]
        if len(P) < MIN_PTS:
            lines.append(None); continue
        lim = int(np.ceil(aperture))
        # ⚠️ **整条边一次算完，不要退回逐点 Python 循环。** 原版是
        # `for p in P: for tt in range(-lim, lim+1)` 的三重循环，每格一次标量索引
        # 加两次 `_bil`；实测这是 refine 里最贵的一段（Orin AGX 上 refine 325ms
        # 里它占 ~133ms）。改成 (n_col, n_tt) 的整块后**结果逐位相同**
        # （601 帧 + 40 帧实机数据，最大分量差 0.000e+00），Orin 上 refine 325→151ms。
        tts = np.arange(-lim, lim + 1, dtype=np.float64)
        U = P[:, 0][:, None] + tts[None, :] * nv[0]
        V = P[:, 1][:, None] + tts[None, :] * nv[1]
        ok = (U >= 1) & (U < w - 1) & (V >= 1) & (V < h - 1)
        val = np.where(ok, gmag[np.round(np.clip(V, 0, h - 1)).astype(np.int64),
                                np.round(np.clip(U, 0, w - 1)).astype(np.int64)], 0.0)
        # 极性：`_bil` 原版在范围内恒不返回 None（判据与上面的 `ok` 逐字相同），
        # 所以这里不需要再判一次 None。
        vx = _bil_grid(gx, U, V, w, h)
        vy = _bil_grid(gy, U, V, w, h)
        good = ok & (val >= gmin) & ((vx * nv[0] + vy * nv[1]) < 0.0)
        sc = np.where(good,
                      val * np.exp(-(tts ** 2) / (2.0 * sigma * sigma))[None, :],
                      -np.inf)
        # `argmax` 取**第一个**最大值，与原版 `sc > best[0]` 的严格大于同口径。
        pick = sc.argmax(axis=1)
        has = np.isfinite(sc[np.arange(len(P)), pick])
        n_ok = int(has.sum())
        if not P.size or n_ok / len(P) < need or n_ok < MIN_PTS:
            lines.append(None); continue
        picked = tts[pick[has]]
        pts = np.stack([P[has, 0] + picked * nv[0],
                        P[has, 1] + picked * nv[1]], axis=1)
        arr = np.asarray(pts, np.float32)
        m = np.ones(len(arr), bool)
        for _ in range(4):
            if m.sum() < 3:
                break
            vx_, vy_, x0, y0 = cv2.fitLine(np.ascontiguousarray(arr[m]),
                                           cv2.DIST_L2, 0, 0.01, 0.01).ravel()
            n2 = np.array([-vy_, vx_], float); n2 /= np.linalg.norm(n2)
            m = np.abs((arr - np.array([x0, y0])) @ n2) < 2.0
        if m.sum() < MIN_PTS:
            lines.append(None); continue
        arr = arr[m]
        vx_, vy_, x0, y0 = cv2.fitLine(np.ascontiguousarray(arr),
                                       cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        n2 = np.array([-vy_, vx_], float)
        if float(np.sqrt(np.mean(((arr - np.array([x0, y0])) @ n2) ** 2))) > maxrms:
            lines.append(None); continue
        lines.append((float(n2[0]), float(n2[1]), -float(n2[0] * x0 + n2[1] * y0)))
    return lines


# --------------------------------------------------------------------------- #
# 帧内记忆化：同一帧上重复调用的量测原语
# --------------------------------------------------------------------------- #
# `measure_lines` / `support` 是**纯函数**——结果只取决于「台面四角投影到哪几个像素」
# 与两张梯度图。而 `refine3` 的循环 + `_corner_stage` + `refine4` 的 `_depth_stage`
# 会在**同一帧**上反复调它们：实测每帧 `measure_lines` 调 5 次里只有 2 次是不同帧、
# `support` 调 15 次里只有 7 次不同。重复的那些白算。
#
# ⚠️ **命中的判据是"四角投影逐位相同 + `gmag`/`gxy` 是同一对对象"**，不是"值差不多"：
#   * 四角用 `tobytes()` 逐位比，不做 round —— 宁可少命中，也不要因取整把两帧混起来；
#   * 梯度图比**对象身份**（`is`），不比内容 —— 每帧的 `gmag`/`gxy` 只算一次、整帧共用，
#     身份不同就一定是另一帧，直接不命中。缓存项里存着对象引用，所以存活的 id 不会被
#     回收复用而误命中。
#
# 每帧开头 `begin_frame()` 清一次，**内存有界**。
_MEMO: dict = {}


def begin_frame() -> None:
    """开一帧：清掉上一帧的记忆化。**`refine_pallet_frame` 开头会自己调。**"""
    _MEMO.clear()


def _memo_key(frame, k):
    """四角投影的逐位指纹。取不到四角（frame 不完整）时返回 `None`（= 不缓存）。"""
    try:
        P = _project_px(_corners_mm(frame, frame['W'], frame['H']), k)
    except Exception:                                # noqa: BLE001
        return None
    return P.tobytes()


def measure_lines(frame, gmag, k, gxy=None):
    """当前量测入口。`MEASURE_MODE` 切换（'soft' = 只软规则给对照用）。

    带**帧内记忆化**（见上面 `_MEMO` 的注释）：同一帧重复调用直接返回上次结果。
    """
    key = _memo_key(frame, k)
    if key is not None:
        hit = _MEMO.get(('ml', key))
        if hit is not None and hit[0] is gmag and hit[1] is gxy:
            return hit[2]
    if MEASURE_MODE == 'polar_aperture' and gxy is not None:
        out = measure_polar_aperture(frame, gmag, k, gxy,
                                     aperture=POLAR_APERTURE_PX, need=POLAR_NEED)
    else:
        out = measure_soft_polar(frame, gmag, k, gxy=gxy)
    if key is not None:
        _MEMO[('ml', key)] = (gmag, gxy, out)
    return out


def grad_mag(color):
    gray = cv2.GaussianBlur(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), (0, 0), BLUR_SIGMA)
    return np.hypot(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3),
                    cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3))


def measure_soft(frame, gmag, k, sigma=SIGMA_PX, half=SEARCH_HALF_PX,
                 gmin=GRAD_MIN, maxrms=FIT_MAX_RESID_PX):
    """每条边沿法向取 max over tt of  mag(tt)·exp(-tt²/2σ²)，拟成一条直线。"""
    h, w = gmag.shape
    px = _project_px(_corners_mm(frame, frame['W'], frame['H']), k)
    lines = []
    for i in range(4):
        a, b = px[i], px[(i + 1) % 4]
        L = float(np.linalg.norm(b - a))
        if L < 40.0:
            lines.append(None)
            continue
        d = (b - a) / L
        nv = np.array([-d[1], d[0]])
        if float(np.dot((a + b) / 2 - px.mean(axis=0), nv)) < 0:
            nv = -nv
        ts = np.linspace(EDGE_MARGIN, 1 - EDGE_MARGIN,
                         max(3, int(L * (1 - 2 * EDGE_MARGIN) / SAMPLE_STEP_PX)))
        P = np.array([a + t * (b - a) for t in ts])
        keep = (P[:, 0] > 2) & (P[:, 0] < w - 3) & (P[:, 1] > 2) & (P[:, 1] < h - 3)
        P = P[keep]
        if len(P) < MIN_PTS:
            lines.append(None)
            continue
        pts = []
        for p in P:
            best = None
            for tt in range(-int(half), int(half) + 1):
                u, v = p[0] + tt * nv[0], p[1] + tt * nv[1]
                if not (1 <= u < w - 1 and 1 <= v < h - 1):
                    continue
                val = float(gmag[int(round(v)), int(round(u))])
                if val < gmin:
                    continue
                s = val * np.exp(-(tt * tt) / (2.0 * sigma * sigma))
                if best is None or s > best[0]:
                    best = (s, tt)
            if best is not None:
                pts.append([p[0] + best[1] * nv[0], p[1] + best[1] * nv[1]])
        if len(pts) < MIN_PTS:
            lines.append(None)
            continue
        arr = np.asarray(pts, np.float32)
        m = np.ones(len(arr), bool)
        for _ in range(4):                       # IRLS：先按 |r|<2px 剔点再拟合
            if m.sum() < 3:
                break
            vx, vy, x0, y0 = cv2.fitLine(np.ascontiguousarray(arr[m]),
                                         cv2.DIST_L2, 0, 0.01, 0.01).ravel()
            n2 = np.array([-vy, vx], float)
            n2 /= np.linalg.norm(n2)
            m = np.abs((arr - np.array([x0, y0])) @ n2) < 2.0
        if m.sum() < MIN_PTS:
            lines.append(None)
            continue
        arr = arr[m]
        vx, vy, x0, y0 = cv2.fitLine(np.ascontiguousarray(arr),
                                     cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        n2 = np.array([-vy, vx], float)
        rms = float(np.sqrt(np.mean(((arr - np.array([x0, y0])) @ n2) ** 2)))
        if rms > maxrms:
            lines.append(None)
            continue
        lines.append((float(n2[0]), float(n2[1]), -float(n2[0] * x0 + n2[1] * y0)))
    return lines


def support(frame, gmag, k):
    """四条边的脊线支持度（有脊线的采样列占比）。**带帧内记忆化。**

    它只取决于「四角投影 + `gmag`」，而 refine 每帧要调它 ~15 次、其中只有 ~7 次
    是不同帧 —— 重复的那些直接返回上次结果（判据与 `measure_lines` 同，
    见 `_MEMO` 的注释）。
    """
    key = _memo_key(frame, k)
    if key is not None:
        hit = _MEMO.get(('sup', key))
        if hit is not None and hit[0] is gmag:
            return hit[1]
    out = _support_impl(frame, gmag, k)
    if key is not None:
        _MEMO[('sup', key)] = (gmag, out)
    return out


def _support_impl(frame, gmag, k):
    """四条边的脊线支持度（有脊线的采样列占比）。与原版一致。

    ⚠️ **整条边一次算完。** 原版是 `for p in P: for tt in range(-12, 13)` 的逐点
    扫描、命中即 `break`；向量化后同一判据写成"沿 tt 取最大值再看是否 >= 30"——
    两者**等价**（因为原版的 `break` 只影响速度，不影响"有没有命中"这个布尔结论），
    实测逐位相同。它是 refine 里调用次数最多的原语（每帧 ~15 次），
    Orin AGX 上曾占 100ms/帧。
    """
    h, w = gmag.shape
    px = _project_px(_corners_mm(frame, frame['W'], frame['H']), k)
    tts = np.arange(-12, 13, dtype=np.float64)
    hits = []
    for i in range(4):
        a, b = px[i], px[(i + 1) % 4]
        L = float(np.linalg.norm(b - a))
        if L < 40.0:
            hits.append(0.0)
            continue
        d = (b - a) / L
        nv = np.array([-d[1], d[0]])
        ts = np.linspace(0.15, 0.85, max(3, int(L * 0.7 / 6.0)))
        P = np.array([a + t * (b - a) for t in ts])
        keep = (P[:, 0] > 2) & (P[:, 0] < w - 3) & (P[:, 1] > 2) & (P[:, 1] < h - 3)
        P = P[keep]
        if len(P) < 4:
            hits.append(0.0)
            continue
        U = P[:, 0][:, None] + tts[None, :] * nv[0]
        V = P[:, 1][:, None] + tts[None, :] * nv[1]
        ok = (U >= 1) & (U < w - 1) & (V >= 1) & (V < h - 1)
        Ui = np.round(np.clip(U, 0, w - 1)).astype(np.int64)
        Vi = np.round(np.clip(V, 0, h - 1)).astype(np.int64)
        val = np.where(ok, gmag[Vi, Ui], -np.inf)   # 越界处 -inf，不参与 max
        hits.append(float((val.max(axis=1) >= 30.0).mean()))
    return float(np.mean(hits)), hits


def _planes(lines, k):
    out = []
    for l in lines:
        if l is None:
            out.append(None)
            continue
        a, b, c = l
        n3 = np.array([a * k.fx, b * k.fy, a * k.cx + b * k.cy + c], float)
        nn = float(np.linalg.norm(n3))
        out.append(n3 / nn if nn > 1e-12 else None)
    return out


def refine2(frame0, color, k, gmag=None, size_mm=PALLET_SIZE_MM):
    """4 自由度初值 -> 像素脊线细化。返回 `(frame', diag)`；不收就交回 `frame0`。"""
    from scipy.optimize import least_squares

    diag = {'stage': 'init', 'rounds': []}
    W, H = float(frame0['W']), float(frame0['H'])
    if gmag is None:
        gmag = grad_mag(color)
    t0 = np.asarray(frame0['origin'], float)
    R0 = np.stack([frame0['E1'], frame0['E2'], frame0['nrm']], axis=1)

    def frame_of(p):
        rv = np.asarray(p[3:], float)
        a = float(np.linalg.norm(rv))
        if a > 1e-12:
            ax = rv / a
            Kx = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            dR = np.eye(3) + np.sin(a) * Kx + (1 - np.cos(a)) * (Kx @ Kx)
        else:
            dR = np.eye(3)
        R = dR @ R0
        f = dict(frame0)
        f['E1'], f['E2'], f['nrm'] = R[:, 0], R[:, 1], R[:, 2]
        f['origin'] = t0 + np.asarray(p[:3], float)
        return f

    cur, rel = frame0, np.zeros(6)
    best_score, _ = support(cur, gmag, k)
    init_score = best_score
    diag['init_score'] = round(best_score, 4)
    for it in range(MAX_ITER):
        lines = measure_soft(cur, gmag, k)
        n_ok = sum(1 for l in lines if l is not None)
        diag['rounds'].append({'iter': it, 'n_lines': n_ok})
        if n_ok < MIN_LINES:
            diag['rounds'][-1]['stop'] = 'n_lines'
            break
        planes = _planes(lines, k)

        def residuals(p):
            C = _corners_mm(frame_of(p), W, H)
            res = []
            for i, pl in enumerate(planes):
                if pl is None:
                    continue
                for v in (C[i], C[(i + 1) % 4]):
                    res.append(float(pl @ v))
            return np.asarray(res)

        mv = MOVE2_MM if n_ok < 3 else MAX_MOVE_MM
        rt = ROT2_DEG if n_ok < 3 else MAX_ROT_DEG
        try:
            sol = least_squares(residuals, rel, loss='huber', f_scale=5.0, max_nfev=200,
                                bounds=([-mv] * 3 + [-np.radians(rt)] * 3,
                                        [mv] * 3 + [np.radians(rt)] * 3),
                                x_scale=[60.0, 60.0, 60.0, 0.02, 0.02, 0.02])
        except Exception as exc:                              # noqa: BLE001
            diag['rounds'][-1]['lsq_error'] = type(exc).__name__
            break
        rms_mm = float(np.sqrt(np.mean(sol.fun ** 2)))
        diag['rounds'][-1]['rms_mm'] = round(rms_mm, 2)
        if rms_mm > RESID_RMS_MM:
            diag['rounds'][-1]['stop'] = 'rms_mm'
            break
        cand_rel = rel + sol.x
        if float(np.linalg.norm(cand_rel[:3])) > MAX_MOVE_MM or \
                float(np.degrees(np.linalg.norm(cand_rel[3:]))) > MAX_ROT_DEG:
            diag['rounds'][-1]['stop'] = 'guard'
            break
        cand = frame_of(cand_rel)
        sc, _ = support(cand, gmag, k)
        diag['rounds'][-1]['score'] = round(sc, 4)
        if sc <= best_score + 1e-9:
            diag['rounds'][-1]['stop'] = 'no_gain'
            break
        best_score, rel, cur = sc, cand_rel, cand
    diag['final_score'] = round(best_score, 4)
    diag['stage'] = 'refined' if best_score > init_score + 1e-9 else 'kept_init'
    return (cur if diag['stage'] == 'refined' else frame0), diag
