# -*- coding: utf-8 -*-
"""把有漂移的托盘台面坐标系，靠已知尺寸 + 台面外轮廓的可见边细化到准确值。
**纯函数、零状态、不 import ROS。**

输入是 `pallet_detect` 那种 frame（`origin`/`E1`/`E2`/`nrm`/`W`/`H`，相机系 mm），
输出是同一个结构的细化结果，外加一份诊断。**本模块是伺服链路上的第二步**：
`pallet_detect.detect_pallet_frame()` 给初值，本模块把它细化到伺服能用的精度。

    输入：初值 frame + 彩色图 + 深度图（16UC1，mm）+ 内参
    输出：细化后的 frame + 诊断（`reject` / `updated` / `held` / `plane` / …）

## 两条必须知道的接口约定

1. **`plane=True` 是部署路径的必需项，而本模块的默认是 `False`。**
   平面细化（SVD 拟合台面，法向 2 + 高度 1 自己解）默认关，理由是可测的：
   april_test7 五帧静止场景里，用深度重拟合台面平面时**有一帧法向动了 3.28°**，
   把跨帧一致性从"两两距离最大 37.6mm"顶到 99.4mm。但**关掉它，5.6° 的法向偏差
   会让朝向停在 5.7°**（伺服拿去会歪）。所以调用方**必须显式传 `opts={'plane': True}`**
   —— ROS 壳里那个参数默认就是 `True`，与这里的默认值不同是刻意的。
2. **哪条边配长边是按量出来的长度定的，不是假设的。** 见 `refine_pallet_frame`
   里那段注释：把 `target_mm[0]` 直接当 E1 的边长是错的，5_test 上会翻车。

设计文档与开发记录在 maduo 仓库（`docs/superpowers/specs/2026-09-20-pallet-frame-refine-design.md`、
`WORKLOG_3test.md`），**不在本仓库**。
"""
from __future__ import annotations

import cv2
import numpy as np
from scipy.optimize import least_squares

# ---- 内联的依赖 ----------------------------------------------------------
# 下面这四个原先在 maduo 的 `ground_detector` 里。与 `pallet_detect` /
# `box_frame` 同一做法**逐字抠出来内联**，函数体一行没动。
# ⚠️ 与 `pallet_detect/algorithm.py` 里那份是**同一份代码的两份拷贝** ——
# 两个模块各自要能单独 import（不互相依赖），所以宁可重复。改一处要改两处。

def _backproject(us: np.ndarray, vs: np.ndarray, zs: np.ndarray,
                 k: CameraIntrinsics) -> np.ndarray:
    return np.stack([(us - k.cx) * zs / k.fx, (vs - k.cy) * zs / k.fy, zs], axis=-1)


def _project_px(points: np.ndarray, k: CameraIntrinsics) -> np.ndarray:
    z = np.where(np.abs(points[..., 2]) < 1e-6, 1e-6, points[..., 2])
    return np.stack([k.cx + k.fx * points[..., 0] / z,
                     k.cy + k.fy * points[..., 1] / z], axis=-1)


def _fit_plane(points: np.ndarray, tol: float, iters: int = 6):
    """SVD 拟合平面 + 内点迭代。返回 `(法向, 质心, 内点掩码)`。"""
    c = np.median(points, axis=0)
    q = points - c
    inl = np.ones(len(points), bool)
    n = None
    for _ in range(iters + 1):
        _, _, vt = np.linalg.svd(q[inl], full_matrices=False)
        n = vt[-1]
        new = np.abs(q @ n) <= tol
        if np.array_equal(new, inl):
            break
        inl = new
    return n, c, inl


def _corners_mm(frame: dict, W: float, H: float) -> np.ndarray:
    """台面矩形四个角，相机系 mm。顺序 (0,0) -> (W,0) -> (W,H) -> (0,H)。

    ⚠️ 写成一次向量化（原来是 4 次带 Python 列表推导的逐角相加）。这个函数
    一帧被调 70+ 次（`_edges_px` 里调它，`_residuals` 里每次迭代又调 `_edges_px`），
    值得省掉列表推导那点固定开销。
    """
    o, E1, E2 = frame['origin'], frame['E1'], frame['E2']
    ab = np.array([[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]], np.float64)
    return (o[None, :] + ab[:, 0:1] * E1[None, :] + ab[:, 1:2] * E2[None, :])



# --------------------------------------------------------------------------- #
# 常量 —— 凡是标「拍的」的都没有数据支撑，等有数据了再定
# --------------------------------------------------------------------------- #
DEFAULT_TARGET_MM = (1200.0, 1000.0)   # 操作员给的：托盘 120cm × 100cm（长,短）

SAMPLE_STEP_PX = 8.0        # 拍的：沿边采样间隔
SEARCH_HALF_PX = 40.0       # 拍的：法向搜索半窗，必须覆盖预期漂移
GRAD_MIN = 60.0             # 拍的：Sobel 幅值下限（ksize=3 的 8 位灰度，强边约 200~500）
MIN_EDGE_SAMPLES_ABS = 8    # 拍的：一条边至少这么多内点才算可用
MIN_EDGE_SAMPLES_FRAC = 0.25  # 拍的：且不低于该边采样数的这个比例

OBS_MAX_RESIDUAL_PX = 3.0   # 拍的：拟合后单个采样点离它的边超过这么多就丢掉
MIN_EDGE_KEEP = 4           # 拍的：丢完离群点后一条边至少还剩这么多才留着

# 观测点里至少要有这么大比例落在稳健拟合的直线附近（1.5px 内），这条边才算
# 被观测到。**这个才是真正的判据**：`_edge_straightness` 算出来的 p90 只进
# 诊断字段 `straight_p90_px`，不参与任何判定（p90 会被 10% 的离群点毙掉整条
# 好边 —— 5_test 的近边就是这么被误杀的，而支持率不会）。
# 拍的，没有系统性标定：真边的支持率应当在 0.8 以上，纯噪声在 0.1 以下。
EDGE_SUPPORT_MIN = 0.5

# 台面平面沿法向的搜索。初值可以是**整个平面偏低**的（5_test 实测低了约 125mm：
# 操作员按二维码反算的偏移情况故意点在低 125mm 的那层上），此时任何单纯"在平面内
# 平动"的细化都救不回来 —— 必须先把平面自己抬上去。
# 判据：托盘台面的外轮廓必须落在台面平面上（h≈0）。平面沿法向移动时，预测的矩形
# 会因视差在图像里平移，所以"哪个抬升量能让四条边同时落在 h≈0 的连贯线上"是
# 一个一维搜索。上限取得足够宽，覆盖一个托盘的常见厚度量级。
PLANE_SEARCH_MAX_MM = 320.0
PLANE_SEARCH_STEP_MM = 10.0
# 观测点到台面平面的容许高度。操作员给的实物约束：**箱子至少 200mm 高**，
# 所以 ±40mm 这道带能把"箱子/货"和"台面自己"干净分开，也不会误伤台面
# （台面是一块板，本身厚 20~25mm）。
PLANE_H_TOL_MM = 40.0

# 平面细化（Step 1）的守卫
PLANE_MARGIN_MM = 150.0     # 足迹外扩，取台面附近的深度点
PLANE_H_BAND_MM = 60.0      # 只取离初值台面这么高的点
PLANE_MIN_POINTS = 3000     # 拍的：点太少就不拟合
PLANE_MAX_TILT_DEG = 8.0    # 拍的：法向移动超过这个角度就判定拟合锁到了别的东西上

# 四条边：名字、在台面系里的位置、所属「族」。
# 族决定它约束哪个平移分量 —— 台面系里四条边都是轴对齐的，所以这是确定的：
#   b = 0 / b = H 这两条沿 E1，法向是 E2，约束 t_e2（ty）
#   a = 0 / a = W 这两条沿 E2，法向是 E1，约束 t_e1（tx）
_EDGE_SPEC = [
    ('y=0', 0.0, 0.0, 1.0, 0.0, 'y'),
    ('x=W', 1.0, 0.0, 1.0, 1.0, 'x'),
    ('y=H', 1.0, 1.0, 0.0, 1.0, 'y'),
    ('x=0', 0.0, 1.0, 0.0, 0.0, 'x'),
]
FREE_FOR_FAMILIES = {
    frozenset({'x', 'y'}): (0, 1, 2),   # 两个方向都有观测 -> 全解
    frozenset({'y'}): (1,),             # 只有沿 E1 的边 -> 只更新 t_e2
    frozenset({'x'}): (0,),             # 只有沿 E2 的边 -> 只更新 t_e1
}


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def _move(frame: dict, tx: float, ty: float, theta: float) -> dict:
    """把台面系整体在自身平面内平移 (tx, ty) 并绕法向转 theta。"""
    E1, E2, nrm = frame['E1'], frame['E2'], frame['nrm']
    c, s = float(np.cos(theta)), float(np.sin(theta))
    out = dict(frame)
    out['E1'] = c * E1 + s * E2
    out['E2'] = -s * E1 + c * E2
    out['nrm'] = nrm
    out['origin'] = frame['origin'] + tx * E1 + ty * E2
    return out


def _corners_mm(frame: dict, W: float, H: float) -> np.ndarray:
    """台面矩形四个角，相机系 mm。顺序 (0,0) -> (W,0) -> (W,H) -> (0,H)。

    ⚠️ 写成一次向量化（原来是 4 次带 Python 列表推导的逐角相加）。这个函数
    一帧被调 70+ 次（`_edges_px` 里调它，`_residuals` 里每次迭代又调 `_edges_px`），
    值得省掉列表推导那点固定开销。
    """
    o, E1, E2 = frame['origin'], frame['E1'], frame['E2']
    ab = np.array([[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]], np.float64)
    return (o[None, :] + ab[:, 0:1] * E1[None, :] + ab[:, 1:2] * E2[None, :])


def _edges_px(frame: dict, W: float, H: float, k) -> list[dict]:
    """四条边投影到图像。端点在相机后方或出画的边标记为不可观测。"""
    corners = _corners_mm(frame, W, H)
    if np.any(corners[:, 2] <= 1.0):
        return [dict(name=n, family=f, p0=None, p1=None, ok=False) for n, _, _, _, _, f in _EDGE_SPEC]
    px = _project_px(corners, k)
    out = []
    for i, (name, a0, b0, a1, b1, fam) in enumerate(_EDGE_SPEC):
        p0, p1 = px[i], px[(i + 1) % 4]
        out.append(dict(name=name, family=fam, p0=p0, p1=p1, ok=True))
    return out


# --------------------------------------------------------------------------- #
# Step 1 — 平面细化
# --------------------------------------------------------------------------- #
def _refine_plane(frame: dict, depth: np.ndarray, k, W: float, H: float) -> tuple[dict, dict]:
    """用深度把台面平面（法向 2 + 高度 1）重拟合一遍。

    守卫是这段的要害：台面被货压住时，足迹里的深度**是货不是台面**，
    硬拟合会把平面锁到货上、而且是悄悄锁上。所以只要点数不够或法向动太多，
    就退回初值平面并在诊断里标出来。
    """
    ys, xs = np.nonzero(depth > 0)
    if ys.size < PLANE_MIN_POINTS:
        return frame, dict(refined=False, note='not_enough_depth_pixels')
    P = _backproject(xs.astype(np.float64), ys.astype(np.float64),
                     depth[ys, xs].astype(np.float64), k)
    r = P - frame['origin']
    a, b, h = r @ frame['E1'], r @ frame['E2'], r @ frame['nrm']
    near = ((a >= -PLANE_MARGIN_MM) & (a <= W + PLANE_MARGIN_MM) &
            (b >= -PLANE_MARGIN_MM) & (b <= H + PLANE_MARGIN_MM) &
            (np.abs(h) <= PLANE_H_BAND_MM))
    if int(near.sum()) < PLANE_MIN_POINTS:
        return frame, dict(refined=False, note='not_enough_points_near_deck')

    pts = P[near]
    # `_fit_plane` 是 SVD + 迭代剔离群，返回 (法向, 中位点, 内点掩码)。
    # 中位点落在平面上，投影 `origin` 时用它当平面上的参考点就够。
    nrm, centroid, inl = _fit_plane(pts, tol=25.0)
    if nrm is None or int(inl.sum()) < PLANE_MIN_POINTS:
        return frame, dict(refined=False, note='plane_fit_failed')
    nrm = _unit(np.asarray(nrm, np.float64))
    if nrm @ frame['nrm'] < 0:
        nrm = -nrm
    tilt = float(np.degrees(np.arccos(np.clip(nrm @ frame['nrm'], -1.0, 1.0))))
    if tilt > PLANE_MAX_TILT_DEG:
        return frame, dict(refined=False, note=f'tilt_{tilt:.1f}deg_over_guard')

    # 把旧的 E1/E2 投到新平面上再正交化，保持参数化的连续性（不要跳）
    E1 = _unit(frame['E1'] - (frame['E1'] @ nrm) * nrm)
    E2 = _unit(np.cross(nrm, E1))
    if E2 @ frame['E2'] < 0:
        E2 = -E2
    origin = frame['origin'] - ((frame['origin'] - centroid) @ nrm) * nrm
    out = dict(frame)
    out.update(origin=origin, E1=E1, E2=E2, nrm=nrm)
    dh = float((origin - frame['origin']) @ frame['nrm'])
    return out, dict(refined=True, note=None, tilt_deg=tilt, height_mm=dh)


# --------------------------------------------------------------------------- #
# Step 3 — 单条边的梯度吸附
# --------------------------------------------------------------------------- #
def _lift(frame: dict, h0: float) -> dict:
    """把台面平面沿法向平移 h0 毫米（正 = 抬离相机看到的更低的那层）。"""
    out = dict(frame)
    out['origin'] = frame['origin'] + h0 * frame['nrm']
    return out


def _obs_height(frame: dict, uv, depth: np.ndarray, k, half: int = 2) -> float | None:
    """一个图像观测点，反投影后离台面平面的高度（mm）。

    用一个小窗的中位深度，单像素深度太噪。这是**唯一一条能把"箱子/货"和
    "台面自己"分开的判据**：它们在图像里可能都是连续的强边，但在 3D 高度上
    差着至少一个箱子高（操作员给的实物约束：箱子 ≥200mm）。
    """
    u, v = int(round(uv[0])), int(round(uv[1]))
    h_img, w_img = depth.shape
    u0, u1 = max(0, u - half), min(w_img, u + half + 1)
    v0, v1 = max(0, v - half), min(h_img, v + half + 1)
    if u1 <= u0 or v1 <= v0:
        return None
    w = depth[v0:v1, u0:u1]
    w = w[w > 0]
    if w.size == 0:
        return None
    z = float(np.median(w))
    P = _backproject(np.array([float(u)]), np.array([float(v)]), np.array([z]), k)[0]
    return float((P - frame['origin']) @ frame['nrm'])


def _search_plane_offset(gx, gy, frame, depth, k, W, H, step_px, half_px, grad_min,
                         max_mm: float = PLANE_SEARCH_MAX_MM,
                         step_mm: float = PLANE_SEARCH_STEP_MM):
    """沿法向找台面平面的抬升量：哪一档能让四条边同时落在 h≈0 的连贯线上。

    返回 (最佳 h0, 逐档得分表)。得分 = 有几条边同时满足
    "点数够 + 直线支持率够 + 观测点的中位高度落在 ±PLANE_H_TOL_MM 内"。

    为什么要这一步：初值可以是**整个平面偏低**的（5_test 实测低 125mm）。
    那时"在平面内做平移"永远对不齐 —— 边在图像里的位置有一部分视差是由
    平面的高低决定的，平动补不回来。先抬平面，再谈平动。
    """
    table = []
    best_h, best_score = 0.0, -1
    mag = cv2.magnitude(gx, gy)     # 算一次，循环里反复用（见 `_snap_edge` 的 mag 参数）
    for h0 in np.arange(0.0, max_mm + 1e-6, step_mm):
        fr = _lift(frame, float(h0))
        score = 0
        for seg in _edges_px(fr, W, H, k):
            if not seg['ok']:
                continue
            found, n_samples = _snap_edge(gx, gy, seg['p0'], seg['p1'],
                                          step_px, half_px, grad_min, mag)
            need = max(MIN_EDGE_SAMPLES_ABS,
                       int(round(MIN_EDGE_SAMPLES_FRAC * max(n_samples, 1))))
            if len(found) < need:
                continue
            sup, _ = _edge_line_support(found, step_px)
            if sup < EDGE_SUPPORT_MIN:
                continue
            hs = [x for x in (_obs_height(fr, uv, depth, k) for uv, _ in found)
                  if x is not None]
            if hs and abs(float(np.median(hs))) <= PLANE_H_TOL_MM:
                score += 1
        table.append((float(h0), score))
        if score > best_score:
            best_h, best_score = float(h0), score
    return best_h, best_score, table


def _edge_line_support(found, step_px: float, tol_px: float = 1.5):
    """观测点有多大比例落在**稳健拟合出的直线**附近，返回 (支持率, 直线)。

    这是三条判据试下来的第三条：

      * 残差 RMS —— 几个离群点就能把一条好边的 RMS 顶到 10px 以上（实测过）。
      * MAD / 中位数 —— 又太钝：被模糊掉的边 MAD 只给 0.48px，看着完美。
        稳健统计量的设计目的就是"忽略离群点"，而这里要抓的恰恰是
        "相当一部分点散开了"，用它方向是反的。
      * **支持率** —— 先把直线用迭代重加权（Cauchy 权重）稳健地拟合出来，
        再数有多少点落在它 1.5px 以内。10% 的离群点既带不偏这条线，
        也不会被忽略掉：5_test 的近边实测 90% 的点中位偏 −1.8px、
        另有 10% 跑到 −38px，支持率仍然很高；而噪声边是均匀铺开的，
        支持率会掉到几个百分点。

    用 `_edge_straightness` 的 p90 当判据就是因为这一条被毙掉的 —— 判据跑在
    离群点剔除**之前**，而它对 10% 的尾巴太敏感。
    """
    if len(found) < MIN_EDGE_KEEP:
        return 0.0, None
    t = np.arange(len(found), dtype=np.float64) * step_px
    off = np.array([o for _, o in found], np.float64)
    A = np.stack([np.ones_like(t), t], axis=1)
    coef, *_ = np.linalg.lstsq(A, off, rcond=None)
    for _ in range(3):                       # 迭代重加权
        r = off - A @ coef
        w = 1.0 / (1.0 + (r / (2.0 * tol_px)) ** 2)
        coef, *_ = np.linalg.lstsq(A * w[:, None], off * w, rcond=None)
    r = np.abs(off - A @ coef)
    return float((r <= tol_px).mean()), coef


def _edge_straightness(found, step_px: float) -> float | None:
    """观测点离「最佳直线」的 |残差| 的 90 分位（px）。

    真边给出的偏移量沿边是**光滑**变化的（初值位姿有误差，所以整条边的偏移
    随位置线性漂移），离那条最佳直线很近。被挡住、或只剩零散噪声的边，
    点就散了。实测：四条真边 0.23~0.64px，被模糊掉的边 32.7~33.0px —— 差 50 倍。

    ⚠️ **别用 MAD 或中位数**：MAD 的设计目的就是"忽略离群点"，而这里要抓的
    恰恰是"相当一部分点散了"。同一批数据 MAD 给出 0.48px（看着完美），
    |r|p90 给出 32.7px（一眼看穿）。稳健统计量在这里是反的。

    这个判据决定了「哪几条边算被观测到」，进而决定 §4.2 里哪些自由度被放开 ——
    判错一个族，另一个方向就没人约束、会跑飞（合成测试里实测跑飞 68mm）。
    """
    if len(found) < MIN_EDGE_KEEP:
        return None
    t = np.arange(len(found), dtype=np.float64) * step_px
    off = np.array([o for _, o in found], np.float64)
    A = np.stack([np.ones_like(t), t], axis=1)
    coef, *_ = np.linalg.lstsq(A, off, rcond=None)
    r = off - A @ coef
    return float(np.percentile(np.abs(r), 90.0))


def _snap_edge(gx: np.ndarray, gy: np.ndarray, p0, p1,
               step_px: float, half_px: float, grad_min: float,
               mag: np.ndarray | None = None):
    """沿一条预测边找梯度峰，返回 [(图像点, 该点), ...]。

    每个采样点**只沿边的法向**扫，扫不到过阈值的峰就**不产生观测** ——
    遮挡、弱边、出画都会自然落进"没有观测"，不需要额外的遮挡判断。

    `mag` = 预计算好的幅值图，**强烈建议传**：这个函数一帧要被调 160 次，
    每次自己算一遍整张 `cv2.magnitude` 是纯浪费（实测占单帧约 8%）。
    不传就照旧自己算，行为不变。
    """
    d = np.asarray(p1, np.float64) - np.asarray(p0, np.float64)
    length = float(np.hypot(*d))
    if length < 2.0:
        return [], 0
    u = d / length
    nvec = np.array([-u[1], u[0]], np.float64)
    if mag is None:
        mag = cv2.magnitude(gx, gy)
    h_img, w_img = mag.shape

    n_samples = max(1, int(length // step_px))
    offs = np.arange(-half_px, half_px + 1.0, 1.0)
    # ⚠️ **整段向量化**（2026-09-21）：原来是 for 每个采样点一套 numpy 调用
    # （一帧 160 次 `_snap_edge` × ~44 个采样点 × ~10 次调用 ≈ 7 万次），
    # 全是 3 元素小数组的固定开销。这里把「采样点 × 窗口偏移」拉成一个
    # (n_samples, n_off) 的矩阵一次算完 —— 尺寸约 44×81，内存可以忽略。
    # 判据（局部极大 + 就近取 + 抛物线插值）逐条保持原样。
    base = np.asarray(p0, np.float64)[None, :] + ((np.arange(n_samples) + 0.5)
                                                 * step_px)[:, None] * u[None, :]
    pts = base[:, None, :] + offs[None, :, None] * nvec[None, None, :]
    ui = np.rint(pts[:, :, 0]).astype(np.int64)
    vi = np.rint(pts[:, :, 1]).astype(np.int64)
    ok = (ui >= 0) & (ui < w_img) & (vi >= 0) & (vi < h_img)
    enough = ok.sum(axis=1) >= 5                  # 原：`if ok.sum() < 5: continue`
    prof = np.where(ok, mag[np.clip(vi, 0, h_img - 1), np.clip(ui, 0, w_img - 1)],
                    -1.0)
    # 取**最近的**过阈值局部极大，而不是最强的那个。托盘有厚度，搜索窗里
    # 常常同时压着好几条平行线（台面外轮廓、侧面棱线、底面轮廓），最强的
    # 那条约不齐是要的那条；而初值的承诺本来就是"离得不远"，所以就着预测
    # 位置就近取。april_test7 那批掠视数据上，取最强会稳定地吸到 32px 外的
    # 另一条线上（偏移中位数 +31px 而真边只该差十几毫米）。
    cand = ((prof[:, 1:-1] >= grad_min)
            & (prof[:, 1:-1] >= prof[:, :-2])
            & (prof[:, 1:-1] >= prof[:, 2:])
            & enough[:, None])
    # 「离 0 最近」= 偏移绝对值最小；`offs` 关于中心对称，故取绝对值最小者。
    # 并列时（中心两侧 ±1）原 `min(key=...)` 取**下标小**的那个，`argmin` 也是。
    cost = np.where(cand, np.abs(offs[1:-1])[None, :], np.inf)
    j = np.argmin(cost, axis=1)
    has = np.isfinite(cost[np.arange(n_samples), j])
    if not np.any(has):
        return [], n_samples
    jj = j[has] + 1                                # 换算回 prof 的下标
    p_hit = prof[has]
    # 抛物线插值到亚像素
    y0, y1, y2 = p_hit[np.arange(len(jj)), jj - 1], p_hit[np.arange(len(jj)), jj], \
        p_hit[np.arange(len(jj)), jj + 1]
    denom = (y0 - 2.0 * y1 + y2)
    delta = np.where(np.abs(denom) > 1e-9, 0.5 * (y0 - y2) / np.where(
        np.abs(denom) > 1e-9, denom, 1.0), 0.0)
    delta = np.clip(delta, -1.0, 1.0)
    off = offs[jj] + delta
    found = [(base_row + o * nvec, float(o))
             for base_row, o in zip(base[has], off)]
    return found, n_samples


# --------------------------------------------------------------------------- #
# Step 4 — 已知尺寸矩形拟合（矩形性是参数化的硬约束）
# --------------------------------------------------------------------------- #
def _residuals(free_values, free_idx, p_full, frame0, W, H, k, observations):
    p = list(p_full)
    for slot, idx in enumerate(free_idx):
        p[idx] = free_values[slot]
    fr = _move(frame0, p[0], p[1], p[2])
    segs = _edges_px(fr, W, H, k)
    # ⚠️ **按边分组批量算**（2026-09-21）：`least_squares` 一帧调这个函数 65 次，
    # 原来每个观测点单独走一遍（`np.asarray` ×3、`np.hypot`、`np.array`、点积），
    # 一帧 8,411 次 `np.asarray` 就是这么来的 —— 在**任何机器上都是纯浪费**。
    # 观测点本来就只落在 4 条边上，按边分组后每组一次算完。
    # 顺序、每条边的数值、坏边给 SEARCH_HALF_PX 的规则都保持不变。
    res = np.full(len(observations), SEARCH_HALF_PX, np.float64)
    if not observations:
        return res
    obs_e = np.asarray([e for e, _ in observations], np.int64)
    obs_uv = np.asarray([uv for _, uv in observations], np.float64)
    for e in np.unique(obs_e):
        seg = segs[int(e)]
        m = (obs_e == e)
        if not seg['ok']:
            continue                        # 解跑到相机后方：保持大残差把它推回来
        A = np.asarray(seg['p0'], np.float64)
        d = np.asarray(seg['p1'], np.float64) - A
        L = float(np.hypot(*d))
        if L < 1e-6:
            continue
        d = d / L
        res[m] = (obs_uv[m] - A[None, :]) @ np.array([-d[1], d[0]])
    return res


# --------------------------------------------------------------------------- #
# 主入口
# --------------------------------------------------------------------------- #

def refine_pallet_frame(frame: dict, color: np.ndarray, depth: np.ndarray, k, *,
                        target_mm=DEFAULT_TARGET_MM, opts: dict | None = None
                        ) -> tuple[dict | None, dict]:
    """把初值 frame 细化。返回 (细化后的 frame | None, 诊断 dict)。"""
    opts = dict(opts or {})
    step_px = float(opts.get('sample_step_px', SAMPLE_STEP_PX))
    half_px = float(opts.get('search_half_px', SEARCH_HALF_PX))
    grad_min = float(opts.get('grad_min', GRAD_MIN))
    # 平面细化默认关。理由是可测的：april_test7 五帧是静止场景，用深度重拟合
    # 台面平面时四帧法向只动 0.30~0.44 度，**有一帧动了 3.28 度**，而那个离群点
    # 把跨帧一致性从"两两距离最大 37.6mm / 法向散布 10.7mm"顶到
    # "99.4mm / 33.7mm"。关掉之后一致性与初值持平（E2 那项还好 20%）。
    # 掠视时台面附近的深度点大部分落在侧面和别的东西上，单帧拟合本来就不稳。
    # 需要时用 --plane 显式打开；真要用它，得先解决"怎么知道这一帧的平面可信"。
    use_plane = bool(opts.get('plane', False))

    _ta, _tb = float(target_mm[0]), float(target_mm[1])
    long_mm, short_mm = max(_ta, _tb), min(_ta, _tb)
    # 哪条边配长边，**不能假设**。`ground_frame` 的 E1 是"点击的 0->1 那条边"，
    # 跟长短毫无关系；把 `target_mm[0]` 直接当 E1 的边长是错的，只是 april_test7
    # 上恰好成立（量出 1208 对 1200）所以没暴露。5_test 上一眼就翻车：点击量出
    # 1158×1340，而实物是 1000×1200 —— 差一点把长短边配反、整个解扭过去。
    #
    # 用**量出来的长度排序**去配：量得长的那条边就是长边。透视会让两条边一起
    # 缩/放，但不会调换它们的大小顺序（实测 1158<1340 对 1000<1200；
    # april_test7 是 910<1208 对 860<1200，顺序都对得上）。配出来的结果会写进
    # 诊断，别让它悄悄发生。
    e1_is_long = frame['W'] >= frame['H']
    W, H = (long_mm, short_mm) if e1_is_long else (short_mm, long_mm)
    diag: dict = {'target_mm': [W, H], 'target_input_mm': [float(target_mm[0]), float(target_mm[1])],
                  'e1_along_mm': W, 'e2_along_mm': H, 'e1_is_long_side': bool(e1_is_long),
                  'measured_wh_mm': [float(frame['W']), float(frame['H'])],
                  'edges': [], 'reject': None, 'updated': [], 'held': []}

    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    # 先高斯模糊再求梯度。这不是为了"去噪"这种泛泛的理由：木托盘面上有大量
    # 细尺度强纹理（板条缝、木纹、阴影），而台面外轮廓是**粗尺度、低对比**的
    # 过渡。不模糊时，搜索窗里的峰几乎全是纹理，RANSAC 也找不到连贯的线 ——
    # 5_test 实测：支持率 8/20/12/9%（各边）-> 模糊后 30/66/35/22%。
    # 模糊把细尺度纹理压掉，轮廓就露出来了。
    blur_sigma = float(opts.get('blur_sigma', 3.0))
    if blur_sigma > 0:
        gray = cv2.GaussianBlur(gray, (0, 0), sigmaX=blur_sigma)
    gray = gray.astype(np.float32)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)

    frame0 = dict(frame)
    frame0['W'], frame0['H'] = W, H
    if use_plane:
        frame0, plane_diag = _refine_plane(frame0, depth, k, W, H)
    else:
        plane_diag = dict(refined=False, note='disabled')
    diag['plane'] = plane_diag

    # 平面沿法向的一维搜索。这一步是"初值整个平面偏低"唯一的解药：
    # 平面不动的话，光在平面里平移永远对不齐（边在图像里的位置有一部分视差
    # 由平面的高低决定）。5_test 实测初值低了约 125mm。
    if opts.get('search_plane_offset', True):
        h0, n_ok, table = _search_plane_offset(gx, gy, frame0, depth, k, W, H,
                                               step_px, half_px, grad_min)
        diag['plane_offset_mm'] = h0
        diag['plane_offset_edges_ok'] = n_ok
        diag['plane_offset_table'] = [[round(v, 1), s] for v, s in table]
        if h0 > 0:
            frame0 = _lift(frame0, h0)
    else:
        diag['plane_offset_mm'] = 0.0

    segs0 = _edges_px(frame0, W, H, k)
    mag = cv2.magnitude(gx, gy)     # 同上：整条边循环共用一张幅值图
    observations: list[tuple[int, np.ndarray]] = []
    per_edge = []
    for i, seg in enumerate(segs0):
        if not seg['ok']:
            per_edge.append(dict(name=seg['name'], family=seg['family'],
                                 n_inliers=0, rms_px=None, used=False, note='unprojectable'))
            continue
        found, n_samples = _snap_edge(gx, gy, seg['p0'], seg['p1'], step_px, half_px, grad_min, mag)
        # 高度过滤：只在台面平面 ±PLANE_H_TOL_MM 之内的观测才算数。
        # 这是**唯一能把"箱子/货"和"台面自己"分开的判据** —— 两者在图像里
        # 都可能是又长又直的强边（5_test 实测：下面那条边吸到的点全在
        # +596mm 的箱子上，支持率还高达 57%），只有 3D 高度分得开。
        # 操作员给的实物约束：箱子 ≥200mm，所以这道 ±40mm 的带不会误伤。
        # 深度取不到时**放行**，不判死刑：判不了就不该当判过。合成测试（没有
        # 深度图）全靠这一条才不会被整个过滤掉。真正的箱子点深度是有的，
        # 照样会被滤掉。
        n_before = len(found)
        heights = [_obs_height(frame0, uv, depth, k) for uv, _ in found]
        found = [fo for fo, hh in zip(found, heights)
                 if hh is None or abs(hh) <= PLANE_H_TOL_MM]
        n_dropped_by_height = n_before - len(found)
        need = max(MIN_EDGE_SAMPLES_ABS, int(round(MIN_EDGE_SAMPLES_FRAC * max(n_samples, 1))))
        straight = _edge_straightness(found, step_px)
        support, _line = _edge_line_support(found, step_px)
        # 两条都过才算"这条边被观测到"：点数够，**并且**这些点确实排成一条线。
        # 只数点数是不够的 —— 遮挡掉一条边后剩下的零散噪声照样能凑够条数。
        used = (len(found) >= need and support >= EDGE_SUPPORT_MIN)
        if len(found) < need:
            note = f'only_{len(found)}_of_{n_samples}'
        elif not used:
            note = f'scattered_support_{support:.0%}'
        else:
            note = None
        per_edge.append(dict(name=seg['name'], family=seg['family'],
                             n_inliers=len(found), n_samples=n_samples,
                             straight_p90_px=straight, line_support=support,
                             n_dropped_by_height=n_dropped_by_height,
                             rms_px=None, used=used, note=note))
        if used:
            observations.extend((i, uv) for uv, _ in found)
    diag['edges'] = per_edge

    families = frozenset(e['family'] for e in per_edge if e['used'])
    free_idx = FREE_FOR_FAMILIES.get(families)
    if not observations or free_idx is None:
        diag['reject'] = 'no_edges'
        return None, diag

    # 平面内参数按台面系自己的轴缩放，免得优化器在一个量纲差 2500 倍的空间里走
    step_scale = {0: 50.0, 1: 50.0, 2: 0.02}

    def solve(obs, idx):
        p_full = [0.0, 0.0, 0.0]
        x0 = np.array([p_full[i] for i in idx], np.float64)
        sol = least_squares(_residuals, x0,
                            args=(idx, p_full, frame0, W, H, k, obs),
                            loss='huber', f_scale=2.0,
                            x_scale=[step_scale[i] for i in idx])
        for slot, i in enumerate(idx):
            p_full[i] = float(sol.x[slot])
        return p_full, sol

    p_full, sol = solve(observations, free_idx)

    # 拟合后复检。两件事，都按"每个采样点"而不是"整条边"来做 —— 部分遮挡的
    # 语义本来就是同一条边上有些点好、有些点坏，丢整条边太粗：
    #
    #  1. 丢掉离它那条边太远的个别采样点（搜索窗扫到了别的边、角点附近两条边
    #     互相串扰，都会产生这种点）。用 RMS 判整条边是不行的：少数几个离群点
    #     就能把一条好边的 RMS 顶到 10px，而它的中位残差其实只有 0.4px。
    #  2. 丢完之后再数一遍，一条边剩下的点太少就整条不用。
    def _residual_of(edge_i, uv, p):
        seg = _edges_px(_move(frame0, p[0], p[1], p[2]), W, H, k)[edge_i]
        if not seg['ok']:
            return None
        A = np.asarray(seg['p0'], np.float64)
        d = np.asarray(seg['p1'], np.float64) - A
        L = float(np.hypot(*d))
        if L < 1e-6:
            return None
        d = d / L
        return float((np.asarray(uv, np.float64) - A) @ np.array([-d[1], d[0]]))

    for _ in range(2):
        keep = []
        for edge_i, uv in observations:
            r = _residual_of(edge_i, uv, p_full)
            if r is None or abs(r) > OBS_MAX_RESIDUAL_PX:
                continue
            keep.append((edge_i, uv))
        if len(keep) == len(observations):
            break
        if not keep:
            break
        observations = keep
        counts = {i: sum(1 for e, _ in observations if e == i) for i in range(len(_EDGE_SPEC))}
        for i in range(len(_EDGE_SPEC)):
            if per_edge[i]['used'] and counts.get(i, 0) < MIN_EDGE_KEEP:
                per_edge[i]['used'] = False
                per_edge[i]['note'] = f'only_{counts.get(i, 0)}_after_outlier_drop'
        families = frozenset(e['family'] for e in per_edge if e['used'])
        free_idx = FREE_FOR_FAMILIES.get(families)
        if free_idx is None:
            diag['reject'] = 'no_edges'
            return None, diag
        p_full, sol = solve(observations, free_idx)

    def edge_rms(edge_i, p):
        pts = [uv for i, uv in observations if i == edge_i]
        rs = [_residual_of(edge_i, uv, p) for uv in pts]
        rs = [r for r in rs if r is not None]
        if not rs:
            return None
        return float(np.sqrt(np.mean(np.square(rs))))

    for i in range(len(_EDGE_SPEC)):
        n_keep = sum(1 for e, _ in observations if e == i)
        per_edge[i]['n_inliers'] = n_keep

    refined = _move(frame0, p_full[0], p_full[1], p_full[2])
    refined['W'], refined['H'] = W, H

    names = ('t_e1', 't_e2', 'theta')
    diag['updated'] = [names[i] for i in free_idx]
    diag['held'] = [names[i] for i in range(3) if i not in free_idx]
    diag['params'] = dict(tx_mm=p_full[0], ty_mm=p_full[1], theta_deg=float(np.degrees(p_full[2])))
    diag['move_mm'] = float(np.linalg.norm(refined['origin'] - frame0['origin']))
    diag['turn_deg'] = float(np.degrees(np.arccos(np.clip(refined['E1'] @ frame0['E1'], -1, 1))))
    res = _residuals(sol.x, free_idx, p_full, frame0, W, H, k, observations)
    diag['rms_px'] = float(np.sqrt(np.mean(np.square(res))))
    diag['n_observations'] = len(observations)
    for i in range(len(_EDGE_SPEC)):
        if any(o[0] == i for o in observations):
            r = edge_rms(i, p_full)
            per_edge[i]['rms_px'] = r
    diag['_initial'] = frame0
    diag['_observations'] = observations
    return refined, diag


# --------------------------------------------------------------------------- #
# 可视化（出图前先跑数值自检，见设计 §1.1）
