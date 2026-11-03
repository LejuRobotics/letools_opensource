#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""细化的**最终建议版**：`refine2` 的脊线级 + 一级"边线求交角点"。

用法与 `refine.refine` 同构：

    from pallet_detect.refine_corner import refine3
    f5, diag = refine3(f4, color, k, gmag=grad_mag(color))

## 三级，逐级只在"有把握"时才收

**第 1 级 · 脊线级**（同 `refine2`）：沿每条边用软规则 `argmax mag·exp(-tt²/2σ²)`
量一条直线，2 条边以上就解 6 自由度（少边时窗口收到 ±8mm/±1°）。
贡献：14.62 → 8.05px。

**第 2 级 · 角点级**（本文件新增）：把相邻两条量到的**边线求交**得到角点，
用 2~3 个角点投影当量测解 4 自由度（锁法向 + 已知尺寸）。

为什么角点要这么来：Harris/Shi-Tomasi 在木托盘角上中位差 **13.4px**（角上是板条
+ 深度不连续，角点响应糊成一片）；而**两条边线的交点**中位只差 **4.29px**。
边线的位置误差虽然中位 2~4px，但两条线一交，误差不累积。

为什么不是"1 个角 + 2 条边方向"：那个配方**秩亏** —— 锁法向的 4 自由度里，
深度方向近乎不可观测（尺度无关条件数 >200，深度扫 0.6~1.6× 残差只从 2.3 变到
5.4px）。必须**两个角点**（4 个标量约束）才定得下来。用真值角做上界：两角 0.00px。

贡献：8.05 → **6.13px**，逐帧 好 123 / 差 12 / 平 466。

## 采纳规则

每级解完都要求**脊线支持度不降**（与 `refine.py`/`refine2.py` 同一条闸门）。
所以逐级只会变好：实测角点级 12 帧变差（都是 ≤1px 量级的抖动）。

## 已知不足

* 189/601 帧在角点级"求不出 2 个角"（相邻边线至少有一条没量到）。这些帧保持
  脊线级的结果，不再改善。
* 最差帧（p90 25.8）没动。改善集中在中位数这段。
* 耗时：本机（4090 + OMP_NUM_THREADS=1）约 57ms/帧。**Orin 上必须实测**。
"""
from __future__ import annotations

import numpy as np

from .geom import _corners_mm, _project_px
from .solve_obb import PALLET_SIZE_MM
from . import solve_obb as SO

from scipy.optimize import least_squares

from . import dsearch
from . import edge_trust
from .dsearch import D_SEARCH_GAIN, D_SEARCH_N, D_SEARCH_ROUNDS
from .refine_ridge import (BLUR_SIGMA, EDGE_MARGIN, FIT_MAX_RESID_PX, GRAD_MIN, MAX_ITER,
                           MAX_MOVE_MM, MAX_ROT_DEG, MIN_LINES, MIN_PTS, MOVE2_MM,
                           RESID_RMS_MM, ROT2_DEG, SAMPLE_STEP_PX, SEARCH_HALF_PX, SIGMA_PX,
                           _planes, grad_mag, drop_edges, measure_lines, signed_grad, support)

# 退化（只剩两条**相邻**边、没有任何一对对边）时是否锁死深度。
# 实测：`2026-10-10-11-44-12` 整包与 `14-22-38` 的部分帧靠它从"框小一圈"救回来；
# 设 False 可做 A/B 对照。
LOCK_DEGEN = True

# 给定了 D 范围时，退化帧怎么用这个范围：
#   'clamp'（默认）—— 退化帧的 D 就用 **OBB 初值的 d0**，夹进给定范围，不搜索。
#                      实测这是这批数据上最稳的来源：深度图在这类帧上稳定偏小
#                      （`01730` 深度 656 vs 真值 ~900），而 d0 通常只差几个百分点。
#   'search'        —— 在范围内扫一遍，按"没量到的边的脊线支持度"投票挑 D。
#                      ⚠️ **不要用**：实测支持度与正确性反相关（`01730` 搜到 1092、
#                      真值最优 900；`14-24-40_00200` 反向命中一次）。±12px 窗口
#                      太松，框一放大就蹭到旁边的木纹/箱边。留着只为复核。
D_MODE = 'clamp'

# 退化帧挑 D 时，框内深度**主峰占比**到这个值就用深度测量值，否则用 OBB 初值的 d0。
# ⚠️ 这是"主峰占比"（`solve_obb._plane_dist_mm` 的第二个返回值），
# 与旧版那个"内点率"（迭代收紧后的占比，中位 0.594）**不是同一个量**、数值不可比。
#
# 2026-10-19：`solve_obb.D_LOCK_INLIER` 改成 0（真机 881 帧：该锁没锁贡献 89% 大跳），
# 本别名跟着变 0 —— 即**退化帧总是用深度 D**。真机上带 refine 的全链路实测为净胜
# （max 747 → 465mm、>100mm 101 → 90、角点距 24.62 → 23.27px），见
# `maduo/debug/jitter/WORKLOG_jitter.md` §12。
#
# 唯一真相源在 `solve_obb.D_LOCK_INLIER`（`refine4.D_INLIER_MIN` 同样是它的别名）。
D_DEPTH_INLIER = edge_trust.D_INLIER
# 退化帧上，"贴合量到的边"的残差超过这个值（毫米）就认为这个 D 不对。
# ⚠️ 别把它当"质量门"用：只有 2 条边时，残差**主要是量测噪声**（边线本身有
# 几像素抖动 -> 十几毫米），跟 D 对不对关系不大。实测设 12mm 会把正确解一起
# 挡掉（`14-22-38_01730` 21.0 -> 20.7px 无改善）。这个门只用于**挡住 D 让解
# 根本贴不上**的极端情况，量级要放到 30mm 以上才有意义。
D_SOLVE_RMS_MM = 1e9

# 平面距离 D 的**搜索范围**（毫米）。None = 不搜（退化时锁 OBB 初值的 d0）。
# 给定时，退化帧会在 [lo, hi] 内按"四边脊线支持度"挑最自洽的 D（见 dsearch.py）。
# 建议在 refine4 那一层设（`refine4(..., d_range=(lo, hi))`）或直接设
# `dsearch.D_RANGE_MM`，两处等价。
D_RANGE_MM = None


MAX_INT_OFF = 40.0    # 交点离预测角超过这么多 -> 认成别的角，丢掉
MIN_CORNERS = 2       # 4 自由度至少要 2 个角（4 个标量约束）
MAX_CORNERS = 3       # 最多用 3 个（再多是超定，留一点冗余但不贪）


def _frame_of(f0, p):
    """6 自由度参数化（3 平移 + 3 旋转向量，绕 f0 的朝向）。"""
    t0 = np.asarray(f0['origin'], float)
    R0 = np.stack([f0['E1'], f0['E2'], f0['nrm']], axis=1)
    rv = np.asarray(p[3:], float)
    a = float(np.linalg.norm(rv))
    if a > 1e-12:
        ax = rv / a
        Kx = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        dR = np.eye(3) + np.sin(a) * Kx + (1 - np.cos(a)) * (Kx @ Kx)
    else:
        dR = np.eye(3)
    R = dR @ R0
    f = dict(f0)
    f['E1'], f['E2'], f['nrm'] = R[:, 0], R[:, 1], R[:, 2]
    f['origin'] = t0 + np.asarray(p[:3], float)
    return f


def _ridge_stage(f0, k, gmag, gxy=None, depth=None, obb_box=None):
    """第 1 级：脊线级，**4 自由度**（平移 3 + 面内转角 1），法向锁死。

    ⚠️ 不锁法向的代价（实测 E87，601 帧）：脊线级是 6 自由度时，只量到 3 条边
    的帧里**一条错边就能把法向拧走** —— 158/601 帧相对 meta 法向偏转 >5°、
    46 帧 >15°、最大 **27.4°**，而且相邻帧能从 0.73° 跳到 20.9°（`..._14-22-38`
    的 00770 -> 00780）。法向被拧歪之后，靠它算出来的平面距离 D 也跟着失去意义
    （`..._11-44-12_00110`：法向差 23.6°，D 634mm vs 真值 887mm）。

    锁上法向后只剩 3+1=4 个自由度，4 条边（8 个约束）有冗余，错边的影响被平均掉。
    """
    W, H = float(f0['W']), float(f0['H'])
    n0 = np.asarray(f0['nrm'], float)
    n0 = n0 / np.linalg.norm(n0)
    if float(n0 @ np.array([0., 0., 1.])) < 0:
        n0 = -n0
    R0 = np.stack([f0['E1'], f0['E2'], f0['nrm']], axis=1)
    e1 = np.array(R0[:, 0], float)
    e2 = np.array(R0[:, 1], float)
    t0 = np.asarray(f0['origin'], float)
    d0, a0, b0 = np.linalg.solve(np.stack([n0, e1, e2], axis=1), t0)
    E1_0 = np.array(f0['E1'], float)
    th0 = float(np.arctan2(E1_0 @ e2, E1_0 @ e1))

    def frame_of(p):
        d, a, b, th = p
        c, s = np.cos(th), np.sin(th)
        return dict(origin=d * n0 + a * e1 + b * e2, E1=c * e1 + s * e2,
                    E2=-s * e1 + c * e2, nrm=n0, W=W, H=H)

    def score(fr_, got_):
        """脊线支持度，**只统计量到的边**。

        ⚠️ 必须按量到的边打分。退化帧（只剩两条相邻边）里没量到的那几条边
        本来就在画面外，它们的支持度恒为 0；拿全边平均当门槛，正确解永远
        超不过初值，细化会被一律丢弃、退回粗糙的 OBB 初值 —— 表现就是
        "D 和尺寸好像没起作用"（实测 `11-44-12_00290`：全边 0.50 恒定，
        但只算量到的两条边是 0.92）。
        """
        _, hits = support(fr_, gmag, k)
        if not got_:
            return float(np.mean(hits))
        return float(np.mean([hits[i] for i in got_]))

    cur = f0
    _got0 = [i for i, l in enumerate(measure_lines(f0, gmag, k, gxy=gxy))
             if l is not None]
    best = score(cur, _got0)
    init = best
    best_rms = np.inf
    ridge_D = None
    dsearch_log = []
    _ridge_stage.last_lock_d = None
    _ridge_stage.last_D = None
    _ridge_stage.last_search = []
    for _ in range(MAX_ITER):
        lines = drop_edges(measure_lines(cur, gmag, k, gxy=gxy))
        n_ok = sum(1 for l in lines if l is not None)
        if n_ok < MIN_LINES:
            break
        # ⚠️ 锁法向后是 **4 自由度**，2 条边（4 个端点约束）= 4 个方程，
        # **正好定死**，不需要 refine2 那个 ±8mm/±1° 的小窗口。那个小窗口是为
        # 6 自由度（2 条边只有 4 个约束 < 6）设的；在这里套用会把解困在初值附近
        # （实测 2026-10-10-11-44-12_00140：丢错边后只剩 2 条边，31.02 -> 37.20，
        #  就是因为被小窗口挡住了）。
        mv = MAX_MOVE_MM
        rt = MAX_ROT_DEG
        planes = _planes(lines, k)

        # ⚠️ **2 条相邻边 + 4 自由度 = 深度 d 完全不可辨**（实测：Jacobian 最小
        # 奇异值恰为 0）。此时解出来的"框的大小"等于**初值那个大小**，边贴得再准
        # 也没用 —— 表现就是"绿框比托盘小一圈"：`2026-10-10-11-44-12` 整包 26~38
        # 帧都只有两条相邻边（托盘一半出画），尺寸比恒在 0.93；`14-22-38_00990`
        # 只剩"下+左"时更是缩成 0.65（273px vs 真值 420px）。
        # 判据：量到的边里**没有任何一对对边**、且总数 < 3 -> d 不可辨，锁死。
        got = [i for i, l in enumerate(lines) if l is not None]
        det = (any((i + 2) % 4 in got for i in got) or len(got) >= 3)
        lock_d = None if (det or not LOCK_DEGEN) else max(d0, 300.0)
        _ridge_stage.last_lock_d = lock_d
        rng = D_RANGE_MM if D_RANGE_MM is not None else dsearch.D_RANGE_MM
        def solve_at(Dval):
            """固定 D = `Dval`（None 表示 D 自由），解面内 (a,b,θ) 贴合量到的边线。

            返回 `(frame, rms_mm)`。⚠️ 初值**每轮从当前解重算**（否则第二轮会从
            原始初值再来一次、原地打转），并**按 D/dc 缩放**平移量 —— 平面坐标与
            深度近似成正比，不缩放的话 D 差得多时最小二乘要从很远处爬回来。
            """
            tc = np.asarray(cur['origin'], float)
            dc, ac, bc = np.linalg.solve(np.stack([n0, e1, e2], axis=1), tc)
            E1c = np.asarray(cur['E1'], float)
            thc = float(np.arctan2(E1c @ e2, E1c @ e1))

            def fd_of(p):
                if Dval is not None:
                    a, b, th = p
                    d_use = Dval
                else:
                    d_use, a, b, th = p
                return dict(origin=d_use * n0 + a * e1 + b * e2,
                            E1=np.cos(th) * e1 + np.sin(th) * e2,
                            E2=-np.sin(th) * e1 + np.cos(th) * e2, nrm=n0, W=W, H=H)

            def res(p):
                C = _corners_mm(fd_of(p), W, H)
                r = []
                for i, pl in enumerate(planes):
                    if pl is None:
                        continue
                    for v in (C[i], C[(i + 1) % 4]):
                        r.append(float(pl @ v))
                return np.asarray(r)

            if Dval is not None:
                s_ = Dval / max(dc, 1.0)
                x0 = np.array([ac * s_, bc * s_, thc])
                lo_ = [-mv, -mv, thc - np.radians(rt)]
                hi_ = [mv, mv, thc + np.radians(rt)]
                xs_ = [60.0, 60.0, 0.02]
            else:
                x0 = np.array([max(dc, 300.0), ac, bc, thc])
                lo_ = [300.0, -mv, -mv, thc - np.radians(rt)]
                hi_ = [4000.0, mv, mv, thc + np.radians(rt)]
                xs_ = [300.0, 60.0, 60.0, 0.02]
            try:
                sol = least_squares(res, x0, loss='huber', f_scale=5.0, max_nfev=200,
                                    bounds=(lo_, hi_), x_scale=xs_)
            except Exception:                               # noqa: BLE001
                return None, np.inf
            if float(np.sqrt(np.mean(sol.fun ** 2))) > RESID_RMS_MM:
                return None, np.inf
            f = fd_of(sol.x)
            P = _project_px(_corners_mm(f, W, H), k)
            if not np.all(np.isfinite(P)):
                return None, np.inf
            rms = float(np.sqrt(np.mean(sol.fun ** 2)))
            # ⚠️ 退化帧（只有两条相邻边）里，**"贴合量到的边"对任何 D 都成立** ——
            # 4 个端点约束定 3 个未知量，残差恒等于初始失配。所以拟合这一步永远
            # "成功"，D 选错也照样通过。真正要卡的是**留下的失配有多大**：
            # 紧紧贴住所量边的解，残差应当很小（实测正确解 3~8mm，量测本身有噪声
            # 所以不会是 0）；残差还停在几十毫米，说明这个 D 让解根本贴不上
            # （`11-44-12_00290`：初值残差 100mm+，换 D 也只降到 7.8mm 就卡住不动）。
            if rms > D_SOLVE_RMS_MM:
                return None, rms
            return f, rms

        # ---- D 的处理：自由 / 范围搜索 / 锁初值 ----
        # 判据（见上面那段注释）：量到的边里没有任何一对对边、且总数 < 3
        # -> 深度不可辨，D 必须由外部定。
        if lock_d is None:
            cand, _rms = solve_at(None)
            if cand is None or                     float(np.sqrt(np.mean(_rms ** 2)) if np.ndim(_rms) else _rms) > RESID_RMS_MM:
                cand = None
        elif rng is not None and D_MODE == 'search':
            # 在**用户给的范围**里挑 D：D 不可辨时"贴合量到的边"对任何 D 都成立
            # （残差≈0），真正有分辨力的是**没量到的那几条边的脊线支持度**。
            # ⚠️ 搜索只有"存在没量到、但**在画面里**的边"时才有分辨力。
            # 若没量到的边全出画（`01730` 只剩两条边且对边都在画外），打分函数
            # 处处为 0，argmax 退化成抽签、会挑到范围端点，结果比不搜更差
            # （实测 01730 IoU 0.91 -> 0.67，14-34-24_00000 0.91 -> 0.78）。
            # 所以：**先算一个基准分**（D 取 d0），只有当某个 D 的支持度明显
            # 胜过基准时才采纳；否则退回"夹 d0"。
            D0 = dsearch.clamp_D(float(lock_d), rng)
            _base_f, _ = solve_at(float(D0))
            base = support(_base_f, gmag, k)[0] if _base_f is not None else -1.0
            bestD, cand, _bs = float(D0), _base_f, base
            for D in np.linspace(float(rng[0]), float(rng[1]), D_SEARCH_N):
                f_, rms = solve_at(float(D))
                if f_ is None:
                    continue
                scD, _ = support(f_, gmag, k)
                if scD > _bs:
                    bestD, cand, _bs = float(D), f_, scD
            for _round in range(D_SEARCH_ROUNDS):
                step = (float(rng[1]) - float(rng[0])) / (D_SEARCH_N - 1) / (3 ** (_round + 1))
                for D in np.linspace(max(rng[0], bestD - step),
                                     min(rng[1], bestD + step), D_SEARCH_N):
                    f_, rms = solve_at(float(D))
                    if f_ is None:
                        continue
                    scD, _ = support(f_, gmag, k)
                    if scD > _bs:
                        bestD, cand, _bs = float(D), f_, scD
            if _bs < base + D_SEARCH_GAIN:
                bestD, cand = float(D0), _base_f     # 没找到更好的 -> 别搜
            _ridge_stage.last_search_spread = (round(base, 3), round(_bs, 3))
            ridge_D = bestD
            _ridge_stage.last_lock_d = bestD
        else:
            # 'clamp'：退化帧的 d 不可辨 -> 从两个来源里挑：
            #   框内深度**主峰占比** >= D_DEPTH_INLIER 时用**深度测量值**（它在这类帧上
            #   比 d0 准），否则用 **OBB 初值的 d0**。
            _Dfix = float(lock_d)
            if depth is not None and obb_box is not None:
                _Dm, _pk = SO._plane_dist_mm(n0, depth, obb_box, k)
                if _Dm and _pk >= D_DEPTH_INLIER:
                    _Dfix = float(_Dm)
            if rng is not None:
                _Dfix = dsearch.clamp_D(_Dfix, rng)
            cand, _rms = solve_at(float(_Dfix))
            ridge_D = float(_Dfix)
        if cand is None:
            break
        sc = score(cand, got)
        # ⚠️ 接受判据不能只看支持度。锁 D 的退化帧里，解会沿**边线方向**滑动
        # （那正是平面约束的零空间）—— 投影出来的边仍压在同样的脊线上，支持度
        # **完全相等**，但平面残差实实在在降下来了。用 `sc <= best` 一刀切会把
        # 这种解当成"没进展"丢掉，于是永远退回 OBB 初值：表现就是"D 和尺寸
        # 好像没起作用"（`11-44-12_00290`：残差 27.2mm 卡住不动，解其实已把
        # 残差降到 7.8mm）。
        # 所以：支持度更高 -> 收；支持度打平但残差更小 -> 也收。
        if sc > best + 1e-9 or _rms < best_rms - 5.0:
            best, cur, best_rms = sc, cand, _rms
            continue
        break
    ok = best > init + 1e-9
    _ridge_stage.last_D = ridge_D
    _ridge_stage.last_search = dsearch_log
    return (cur, True) if ok else (f0, False)


def _intersect(l1, l2):
    A = np.array([[l1[0], l1[1]], [l2[0], l2[1]]], float)
    b = np.array([-l1[2], -l2[2]], float)
    if abs(np.linalg.det(A)) < 1e-9:
        return None
    return np.linalg.solve(A, b)


def _corner_stage(f0, k, gmag, gxy=None):
    """第 2 级：边线求交得角点 -> 4 自由度（锁法向修正量 + 尺寸不变）。"""
    W, H = float(f0['W']), float(f0['H'])
    lines = drop_edges(measure_lines(f0, gmag, k, gxy=gxy))
    if sum(1 for l in lines if l is not None) < 2:
        return f0, False, None, None
    P0 = _project_px(_corners_mm(f0, W, H), k)
    h, w = gmag.shape
    corners = {}
    for i in range(4):
        l1, l2 = lines[(i - 1) % 4], lines[i]
        if l1 is None or l2 is None:
            continue
        p = _intersect(l1, l2)
        if p is None or not (0 <= p[0] < w and 0 <= p[1] < h):
            continue
        if float(np.linalg.norm(p - P0[i])) > MAX_INT_OFF:
            continue
        corners[i] = p
    if len(corners) < MIN_CORNERS:
        return f0, False, None, None
    idxs = sorted(corners)[:MAX_CORNERS]

    R0 = np.stack([f0['E1'], f0['E2'], f0['nrm']], axis=1)
    n0 = np.array(R0[:, 2], float)
    t0 = np.asarray(f0['origin'], float)
    if float(t0 @ n0) < 0:
        n0 = -n0
    e1 = np.array(R0[:, 0], float)
    e2 = np.array(R0[:, 1], float)
    d0, a0, b0 = np.linalg.solve(np.stack([n0, e1, e2], axis=1), t0)
    E1_0 = np.array(f0['E1'], float)
    th0 = float(np.arctan2(E1_0 @ e2, E1_0 @ e1))

    def frame4(p):
        d, a, b, th = p
        c, s = np.cos(th), np.sin(th)
        return dict(origin=d * n0 + a * e1 + b * e2,
                    E1=c * e1 + s * e2, E2=-s * e1 + c * e2, nrm=n0, W=W, H=H)

    def res(p):
        C = _project_px(_corners_mm(frame4(p), W, H), k)
        r = []
        for i in idxs:
            r.append(float(C[i][0] - corners[i][0]))
            r.append(float(C[i][1] - corners[i][1]))
        return np.asarray(r)

    # ⚠️ θ 的搜索窗口必须**以初值自己的 θ 为中心**（±`SO.INPLANE_LIMIT_DEG`）。
    # 锁死窗口而种子写 0，会把已经偏出窗口的帧解不回来（实测真值输入下能从
    # 0.00px 崩到 141px）。
    #
    # ⚠️ **跟着 `solve_obb.INPLANE_LIMIT_DEG` 走，不要写死** —— 2026-10-15 把那个
    # 常数从 25° 放到 40°，这里写死就意味着"detect 放开了、这一级还夹着"：
    # 细化会把 detect 已经解对的角度**重新拉回旧窗口内**，而且两处日志都正常。
    lim = np.radians(SO.INPLANE_LIMIT_DEG)
    best = None
    for t in (th0, 0.0):
        try:
            s = least_squares(res, np.array([max(d0, 300.0), a0, b0, t]),
                              loss='soft_l1', f_scale=3.0, max_nfev=300,
                              bounds=([300.0, -1500.0, -1500.0, t - lim],
                                      [4000.0, 1500.0, 1500.0, t + lim]),
                              x_scale=[300.0, 200.0, 200.0, 0.05])
        except Exception:                                   # noqa: BLE001
            continue
        if best is None or s.cost < best.cost:
            best = s
    if best is None:
        return f0, False, None, None
    cand = frame4(best.x)
    P = _project_px(_corners_mm(cand, W, H), k)
    if not np.all(np.isfinite(P)):
        return f0, False, None, None
    return cand, True, idxs, corners


def refine3(frame0, color, k, gmag=None, size_mm=PALLET_SIZE_MM,
            depth=None, obb_box=None):
    """4 自由度初值 -> 脊线级 + 角点级。返回 `(frame', diag)`。

    `depth` / `obb_box` 只用于**退化帧**（只剩两条相邻边、深度不可辨）时挑 D：
    框内深度内点率够高就用深度测量值，否则退回 OBB 初值的 d0。
    """
    diag = {'stage': 'init', 'rounds': []}
    if gmag is None:
        gmag = grad_mag(color)
    gxy = signed_grad(color)
    s0, _ = support(frame0, gmag, k)
    best, stage1 = _ridge_stage(frame0, k, gmag, gxy=gxy,
                                depth=depth, obb_box=obb_box)
    s1, _ = support(best, gmag, k)
    if s1 > s0 + 1e-9:
        cur, diag['ridge'] = best, 'refined'
    else:
        cur, diag['ridge'] = frame0, 'kept_init'
    cand, ok, idxs, cdict = _corner_stage(cur, k, gmag, gxy=gxy)
    if ok:
        s2, _ = support(cand, gmag, k)
        diag['corner'] = {'n_corners': len(idxs), 'score': round(s2, 4),
                          'idx': list(idxs),
                          'px': {k: [float(cdict[k][0]), float(cdict[k][1])]
                                 for k in idxs if cdict and k in cdict}}
        if s2 > s1 + 1e-9:
            cur, diag['stage'] = cand, 'corner_refined'
        else:
            diag['stage'] = 'ridge_only'
    else:
        diag['corner'] = None
        diag['stage'] = 'ridge_only' if cur is not frame0 else 'kept_init'
    diag['init_score'] = round(s0, 4)
    diag['final_score'] = round(support(cur, gmag, k)[0], 4)
    return cur, diag
