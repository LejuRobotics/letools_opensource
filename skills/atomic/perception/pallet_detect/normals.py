# -*- coding: utf-8 -*-
"""台面法向：TF 先验 + **当前帧深度自检**。纯函数、零状态、不 import ROS。

法向是**两级**的（用户 2026-08 确认的现场事实）：

1. **TF 链** `base_link` 的 z 轴转进相机系 = **先验法向** —— 它可能有
   **0~10°** 的偏差（TF 是通的、但 base_link 的 z 与托盘实际台面法向不重合）。
2. **深度拟合**在先验附近把它纠正到实际法向（`fit_floor_normal`）。

所以"TF 通了"不等于"法向对了"：只信 TF，那 0~10° 会**直接进位姿**，而
`frame_from_obb` 把法向**锁死**成先验（自由度 6→4），下游没有任何一步会把它
纠回来 —— 偏差就这么静默地留在结果里。必须靠这一步自检纠偏。

    fit_floor_normal(depth, k, prior) -> (normal, n_inliers) | (None, 0)

⚠️ 本模块**只做"法向"这一件事**（从深度拟合最大的那个平面）。OBB 路线其余部分
（`frame_from_obb` 的锁法向 4 自由度初值）在 `solve_obb.py`。
"""
from __future__ import annotations

import numpy as np

from .geom import CameraIntrinsics


def _backproject(us: np.ndarray, vs: np.ndarray, zs: np.ndarray,
                 k: CameraIntrinsics) -> np.ndarray:
    return np.stack([(us - k.cx) * zs / k.fx, (vs - k.cy) * zs / k.fy, zs], axis=-1)



# --------------------------------------------------------------------------- #
# T10：法向**从当前帧深度自检**（TF 只当搜索初值）
# --------------------------------------------------------------------------- #
# ⚠️ **为什么要有这个**：TF 链给出的法向**会静默偏掉**，而 detect 对法向极其敏感
# （见模块 docstring 第 1 条）。2026-09-29 现场实测（test_jia4，37 帧）：
#
#     地面平面（纯深度 RANSAC，8.1 万内点）  [0.035, -0.344, -0.938]
#     托盘台面平面（木色∧高度近台面，2.2 万点）[0.037, -0.339, -0.940]
#     TF 链给出的法向                        [0.003, -0.135, -0.991]
#     地面 vs 托盘 **1.14°**（同一批帧最大 1.53°）
#     TF   vs 地面 **12.52°**（最大 12.72°）
#
# **37 帧里 TF 法向一个数都没变**（逐帧最大角度差 0.000°），相机原点 z 也恒为
# 1117.3mm —— 整个 bag 里 TF 是静止的，所以这不是抖动，是**静态偏了 12.4°**。
# 偏 12.4° 的后果是 `too_few_edges` **37/37 全拒**（支撑边 0/4）；把法向换成
# 拟合值、其它一个参数都不动，**37/37 全过**（支撑边 3~4/4，四角逐帧标准差
# 0.4~1.3px）。
#
# ⚠️ **拟合的是"占画面最大的那个平面"，不一定是台面** —— 现场那帧最大的面是
# **地面**（9.5 万点），台面只有 2.2 万点。这是**刻意**的：地面与台面本来就
# 近似平行（实测差 1.14°），而 detect 的临界角是 5~7°（见 `DECK_BAND_MM`），
# 差这一个量级不影响；反过来，按颜色挑台面会重新引入"浅色通路把地面算成木色"
# 的问题，而那正是当前掩码最大的坑。**用最大的面 = 最稳的面。**
#
# ⚠️ **初值 `prior` 的职责是"别锁到墙上去"，不是"给出答案"**：现场画面里蓝色
# 墙面占 36.9%，它的法向与地面差 80°+，靠 `FIT_NORMAL_WIN_DEG` 的夹角窗直接
# 排除。窗内挑最大的面，所以初值歪 12° 完全够用（窗 ±25° 有 2 倍余量）。
FIT_NORMAL_SUB = 1500             # 抽样子样点数：实测 253k 点 -> 1500 抽样，耗时 5.9ms
FIT_NORMAL_ITERS = 120            # RANSAC 迭代次数
FIT_NORMAL_TOL_MM = 15.0          # 内点阈值（点到假设平面的距离）
FIT_NORMAL_WIN_DEG = 25.0         # 与初值的夹角上限；超出直接丢弃该假设
FIT_NORMAL_MIN_INLIERS = 400      # 内点下限，少于这个判"拟合失败"
FIT_NORMAL_SEED = 0               # **固定种子**：同一帧给同一结果（可复现，便于回归）


def _backproject_cloud(depth: np.ndarray, k) -> np.ndarray:
    """深度图 -> `(N, 3)` 点云，**单位 mm**（相机光学系）。无有效点时返回 `(0, 3)`。

    与 `_rasterise` 内部那次反投影走的是同一个 `_backproject`，只是这里不过体素
    化 —— `fit_floor_normal` 要的是原始点，体素化会先把平面内的结构磨掉。
    """
    vy, ux = np.nonzero(depth > 0)
    if vy.size == 0:
        return np.zeros((0, 3), np.float64)
    return _backproject(ux.astype(np.float64), vy.astype(np.float64),
                        depth[vy, ux].astype(np.float64), k)


def fit_floor_normal(depth: np.ndarray, k, prior: np.ndarray, *,
                     n_sub: int = FIT_NORMAL_SUB, iters: int = FIT_NORMAL_ITERS,
                     tol_mm: float = FIT_NORMAL_TOL_MM,
                     win_deg: float = FIT_NORMAL_WIN_DEG,
                     min_inliers: int = FIT_NORMAL_MIN_INLIERS,
                     seed: int = FIT_NORMAL_SEED,
                     return_plane: bool = False):
    """从**当前帧**深度拟合"占画面最大的那个平面"的法向。返回 `(normal, n_inliers)`。

    失败返回 `(None, 0)` —— **调用方据此刻意回退到 `prior`**（见下）。

    `return_plane=True` 时多返回第三项 `plane_mm`：**相机原点到该拟合平面的距离**
    （沿法向，毫米，取内点质心）。调用方用它做合理性自检
    （`normal_is_plausible`）—— 只看法向方向会被"TF 初值本身歪了"骗过去
    （2026-10-22 现场：初值侧向、拟合与初值只差 0.05°，D 却是 241mm）。
    失败时第三项是 `None`，元组长度保持不变。

    `prior`（必需）：**只当搜索初值用**，有两个职责：
      1. `win_deg` 夹角窗 —— 丢弃与它差太远的假设。现场画面里蓝色墙面占 36.9%、
         法向与地面差 80°+，没有这个窗 RANSAC 会有一半概率锁到墙上。
      2. 定符号 —— 平面法向 ±n 都合法，拟合完按 `n·prior > 0` 翻正，保证与调用
         方后续用 `prior` 时的约定一致（相机系里"朝上"的那一侧）。

    ⚠️ **它不保证拟合出来的是台面**：现场那帧最大的面是地面（9.5 万点 vs 台面
    2.2 万点）。这是刻意的 —— 地面与台面实测只差 1.14°，而 detect 的临界角是
    5~7°，差这个量级不影响；按颜色挑台面反而会重新引入浅色通路的问题。
    见 `FIT_NORMAL_*` 常量上那段论证。

    ⚠️ **耗时**：253k 点 -> 抽 1500 -> 120 次迭代，**实测 9.6ms（中位）/12.2ms（最大）**
    37 帧现场数据。这是**首帧**成本；调用方通常只算一次就缓存（伺服时机器人
    只做平面内平移，相机相对 base 的朝向不变）。

    ⚠️ **固定种子**：同一帧永远给同一个结果，回归测试才能钉住它。RANSAC 本来
    就有随机性，不固定种子会让"这次改动了没有"变成一个统计问题。
    """
    def _fail():
        return (None, 0, None) if return_plane else (None, 0)

    P = _backproject_cloud(depth, k)
    if P.shape[0] < min_inliers:
        return _fail()
    n0 = np.asarray(prior, float)
    norm0 = float(np.linalg.norm(n0))
    if not np.isfinite(norm0) or norm0 < 1e-9:
        return _fail()
    n0 = n0 / norm0

    rng = np.random.RandomState(seed)
    Q = P[rng.choice(P.shape[0], min(n_sub, P.shape[0]), replace=False)]
    cos_win = float(np.cos(np.radians(win_deg)))
    best_count, best_n, best_p0 = 0, None, None
    for _ in range(int(iters)):
        a, b, c = Q[rng.choice(Q.shape[0], 3, replace=False)]
        n = np.cross(b - a, c - a)
        nl = float(np.linalg.norm(n))
        if nl < 1e-9:                    # 三点共线
            continue
        n = n / nl
        if abs(float(n @ n0)) < cos_win:  # 与初值差太远 -> 多半是墙
            continue
        count = int((np.abs((Q - a) @ n) < tol_mm).sum())
        if count > best_count:
            best_count, best_n, best_p0 = count, n, a
    if best_n is None or best_count < min_inliers:
        return _fail()

    # 用全部内点做一次 SVD 精修（RANSAC 那个三点解本身是有偏的）
    inl = Q[np.abs((Q - best_p0) @ best_n) < tol_mm]
    if inl.shape[0] < min_inliers:
        return _fail()
    centre = inl.mean(axis=0)
    n = np.linalg.svd(inl - centre, full_matrices=False)[2][2]
    if n[2] > 0.0:                       # 与 `_fit_plane` 同一约定：法向朝 -z 侧
        n = -n
    if float(n @ n0) < 0.0:              # 再按初值定符号
        n = -n
    n = n / np.linalg.norm(n)
    if return_plane:
        # 相机原点在 (0,0,0)，原点到平面的距离 = |centre·n|（mm）
        return n, int(inl.shape[0]), float(abs(float(centre @ n)))
    return n, int(inl.shape[0])


def normal_is_plausible(normal, *, max_tilt_deg: float = 60.0,
                        plane_mm=None, d_ref_mm=None,
                        d_tol: float = 0.35) -> tuple:
    """一次深度拟合结果是否**物理上说得通** -> `(ok, 原因)`。

    两道互相独立的闸（都可关），**只用于拒绝"明显不合理"的拟合**，不是发布判据
    （发布判据是节点的 D 闸门，`d_tol=10%`）：

    1. **与相机光轴夹角**：台面是俯视相机的"地面/桌面"类平面，法向应与光轴大致
       平行。2026-10-22 现场那次首帧拟到了**侧向**（法向几乎 `-Y`，与光轴
       **69.8°**），内点 487、"与 TF 初值差 0.05°" 都看着很自信 —— 因为
       **TF 初值本身就歪了**，相对初值的校验拦不住它。`max_tilt_deg <= 0` = 关。
    2. **拟合平面的距离**：相机原点到拟合平面的距离若与 `d_ref_mm` 差太远，
       说明拟到的不是托盘/地面这一层（那次是 **241mm vs 990mm**）。
       用**拟合平面的距离**而不是最终位姿 D（后者要检测之后才有），容差必须
       比发布闸门宽：拟合的常常是地面，地面与台面实测差 ~15%。
       `d_tol <= 0` / `d_ref_mm is None` / `plane_mm is None` = 关。

    返回的原因串只用于日志，不要拿它做分支。
    """
    n = np.asarray(normal, float)
    if n.shape != (3,) or not np.all(np.isfinite(n)):
        return False, '法向非有限值'
    norm = float(np.linalg.norm(n))
    if norm < 1e-9:
        return False, '法向长度为零'
    n = n / norm
    if max_tilt_deg > 0.0:
        tilt = float(np.degrees(np.arccos(np.clip(abs(float(n[2])), 0.0, 1.0))))
        if tilt > max_tilt_deg:
            return False, (f'与光轴夹角 {tilt:.1f}° > {max_tilt_deg:.1f}°'
                           f'（法向侧向，多半拟到墙/立面）')
    if (plane_mm is not None and d_ref_mm is not None
            and float(d_ref_mm) > 0.0 and d_tol > 0.0):
        plane_mm = float(plane_mm)
        if not np.isfinite(plane_mm):
            return False, '拟合平面距离非有限'
        dev = abs(plane_mm / float(d_ref_mm) - 1.0)
        if dev > d_tol:
            return False, (f'拟合平面 {plane_mm:.0f}mm vs 参考 '
                           f'{float(d_ref_mm):.0f}mm（{dev * 100:.1f}% > '
                           f'{d_tol * 100:.0f}%）')
    return True, ''


def fit_floor_normal_checked(depth: np.ndarray, k, prior: np.ndarray, *,
                             max_tilt_deg: float = 60.0,
                             d_ref_mm=None, d_tol: float = 0.35,
                             **fit_kwargs):
    """`fit_floor_normal` + **合理性自检** + 一次**光轴初值重试**。

    返回 `(normal | None, n_inliers, plane_mm | None, 说明)`。

    为什么要重试：现场 17:44 那次 TF 初值本身侧向，第一次拟合在这个歪初值的
    ±25° 窗里必然还是侧向；只用同一初值重拟合解决不了。第二次用**相机光轴**
    `-Z` 当初值 —— 托盘台面必在俯视相机的光轴附近，这是与 TF 无关的物理先验。

    两次都不合格 -> 返回 `(None, 0, None, 原因)`，由调用方决定回退到哪个先验；
    **绝不在不合格时缓存**（那正是 2026-10-22 那次"一帧拟歪、全程被 D 闸门
    拒绝"的根因）。
    """
    fitted, n_inl, plane_mm = fit_floor_normal(
        depth, k, prior, return_plane=True, **fit_kwargs)
    ok, why = (False, '拟合失败') if fitted is None else normal_is_plausible(
        fitted, max_tilt_deg=max_tilt_deg, plane_mm=plane_mm,
        d_ref_mm=d_ref_mm, d_tol=d_tol)
    if ok:
        return fitted, n_inl, plane_mm, ''
    first_why = why

    # 第二级：光轴初值。只有第一级不合格才付这一次拟合的钱（~10ms）。
    axis_prior = np.array([0.0, 0.0, -1.0])
    fitted2, n_inl2, plane_mm2 = fit_floor_normal(
        depth, k, axis_prior, return_plane=True, **fit_kwargs)
    if fitted2 is not None:
        ok2, why2 = normal_is_plausible(
            fitted2, max_tilt_deg=max_tilt_deg, plane_mm=plane_mm2,
            d_ref_mm=d_ref_mm, d_tol=d_tol)
        if ok2:
            return fitted2, n_inl2, plane_mm2, f'首次不合格（{first_why}），改用光轴初值'
        return None, 0, None, (f'两次都不合格：首次 {first_why}；'
                               f'光轴初值 {why2}')
    return None, 0, None, f'两次都不合格：首次 {first_why}；光轴初值拟合失败'

