"""从检测框拟合箱子顶面完整四角（全自动，零人工）。

输入 **YOLO 给的轴对齐外接框**（框住箱子被遮挡后的可见部分），输出**箱子顶面
完整外轮廓的四个角**（像素坐标，顺序固定 **右下 → 左下 → 左上 → 右上**）。
箱子已知 530×350mm。**要做的事就是「恢复被 YOLO 丢掉的那个旋转角」** ——
只想要外接矩形的话根本不需要这个模块。

**这是算法核心**：零交互、零状态、纯函数。**不 import 框架任何东西**
（不 import `core.*`、不 import ROS、不 log），也不依赖任何外部仓库。
分层：本文件 → `window.py`（有状态的时间窗）→ `ros/carton_box_detect.py`。

⚠️ **不要 import `pupil_apriltags`**（比如通过 maduo 的 `calibrate_pallet_tag`）：
同进程里它和 PIL 一起加载会在解释器析构时 abort（实测 2026-09-20）。
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import time

import cv2
import numpy as np

# ---- 内联的依赖 ----------------------------------------------------------
# 下面这些原先散在 maduo 的 ground_detector / rgbd_detector / refine_pallet_frame
# 里。合并进 LeTools 时**逐字抠出来内联**，好让本模块自成一体（见上）。
# 抠出来的内容与 maduo 那边**逐位一致** —— 20 帧比对四角差 0.000e+00px。
# 名字保留前导下划线，是为了**不改动正文** —— 正文里对它们的调用一处没动。

SAMPLE_STEP_PX = 8.0        # 拍的：沿边采样间隔
SEARCH_HALF_PX = 40.0       # 拍的：法向搜索半窗，必须覆盖预期漂移
MIN_EDGE_SAMPLES_FRAC = 0.25  # 拍的：且不低于该边采样数的这个比例
OBS_MAX_RESIDUAL_PX = 3.0   # 拍的：拟合后单个采样点离它的边超过这么多就丢掉
MIN_EDGE_KEEP = 4           # 拍的：丢完离群点后一条边至少还剩这么多才留着
EDGE_SUPPORT_MIN = 0.5
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

def _backproject(us: np.ndarray, vs: np.ndarray, zs: np.ndarray, k: CameraIntrinsics) -> np.ndarray:
    return np.stack([(us - k.cx) * zs / k.fx, (vs - k.cy) * zs / k.fy, zs], axis=-1)


def _project_px(points: np.ndarray, k: CameraIntrinsics) -> np.ndarray:
    z = np.where(np.abs(points[..., 2]) < 1e-6, 1e-6, points[..., 2])
    return np.stack([k.cx + k.fx * points[..., 0] / z, k.cy + k.fy * points[..., 1] / z], axis=-1)


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def read_image(path: str | Path, flags: int) -> np.ndarray | None:
    'Use imdecode so Windows OpenCV can read paths containing Chinese characters.'
    try:
        encoded = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return cv2.imdecode(encoded, flags)


def load_intrinsics(sequence_dir: str | Path) -> CameraIntrinsics | None:
    sequence = Path(sequence_dir)
    path = sequence / 'intrinsics_depth_aligned.txt'
    if not path.exists():
        path = sequence / 'intrinsics.txt'
    if not path.exists():
        return None
    matrix = np.loadtxt(path, dtype=np.float64)
    return CameraIntrinsics(float(matrix[0, 0]), float(matrix[1, 1]), float(matrix[0, 2]), float(matrix[1, 2]))


def pair_frames(sequence_dir: str | Path, tolerance_ms: float) -> list[tuple[Path, Path]]:
    sequence = Path(sequence_dir)
    color_paths = sorted((sequence / 'color_frames').glob('*.png'), key=lambda item: float(item.stem))
    depth_paths = sorted((sequence / 'depth_frames').glob('*.png'), key=lambda item: float(item.stem))
    if not color_paths or not depth_paths:
        raise FileNotFoundError(f'Expected PNG frames under {sequence}')
    depth_times = np.array([float(path.stem) for path in depth_paths], dtype=np.float64)
    pairs: list[tuple[Path, Path]] = []
    for color_path in color_paths:
        color_time = float(color_path.stem)
        position = int(np.searchsorted(depth_times, color_time))
        candidates = [index for index in (position - 1, position) if 0 <= index < len(depth_paths)]
        best_index = min(candidates, key=lambda index: abs(depth_times[index] - color_time))
        if abs(depth_times[best_index] - color_time) <= tolerance_ms / 1000.0:
            pairs.append((color_path, depth_paths[best_index]))
    return pairs


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


# --------------------------------------------------------------------------- #
# 常量 —— 凡标「拍的」的都没有数据支撑，等实测再定
# --------------------------------------------------------------------------- #
DEFAULT_TARGET_MM = (530.0, 350.0)   # 操作员给的：箱子 53cm × 35cm（长,短）

MIN_PLANE_POINTS = 800               # 拍的：≈ 0.4×(530×350)mm² 在实测距离下的像素量级
BOX_INBOX_PAD_MM = 15.0              # 取「框内点」时向外放宽的余量，**按物理尺寸算**。
                                     # 它要容忍的是「YOLO 框比可见部分小一点」，与框在
                                     # 图像里多大无关 —— 所以不能取像素比例：框会随箱子
                                     # 旋转而涨大（6_test 实测 274px→312px），按比例取
                                     # 余量会跟着涨、多吞邻箱的点，旋转 10° 以上四个
                                     # 候选全被撑爆（no_fitting_orientation）。
PLANE_INLIER_MM = 8.0                # 拍的：点到平面的内点容差
PLANE_FACING_MIN_COS = 0.5           # 拍的：法向与视线夹角 < 60° 才算「正面朝向相机」
PLANE_H_TOL_MM = 30.0                # 拍的：mask 取「到该平面距离 ≤ 这么多」的像素
PLANE_Z_RANGE_PAD_MM = 20.0          # flat 模式下深度带的余量（顶面点 2%~98% 分位之外再放宽）
PLANE_SIZE_TOL_FRAC = 0.2            # 平面外接矩形比目标箱子大出这个比例就判它不是箱子。
                                     # 拍 0.6 时在 6_test 上翻车：选中了 span 678x461（比箱子
                                     # 大 28%，是托盘上别的东西或箱子顶面+侧面的混合），
                                     # size_ok 放行、把真正的箱子（330x297，score 只有它一半）
                                     # 压掉，五帧全 reject=no_fitting_orientation。
                                     # 取 0.2（530x1.2=636 / 350x1.2=420）能同时排除
                                     # 1200x1000 托盘、678x461 混杂物；**偏小一律放行**（遮挡）。

GRAD_MIN_MM_PER_PX = 50.0            # 拍的：深度阶跃下限。箱子比周围高 100mm+，
                                     # 真实边界在 1~2px 内完成跳变，木纹起伏达不到

OUTER_WALK_PX = 40.0                 # 判「外侧是不是手」时往外找多远（px）。
                                     # 与 `SEARCH_HALF_PX` 同量级 —— 那是深度峰的
                                     # 搜索半窗，两个数描述的是同一件事（越过箱沿
                                     # 多远算「外面」）。不是按场景试出来的常数。
MIN_HAND_RUN = 2                     # 连续这么多个点被判成「手」才算真被挡住。
                                     # 2 就是「至少两个相邻采样」，不是调出来的：
                                     # 机械手是连续物体，单点必是噪声，见
                                     # `_drop_short_runs`。

LINE_INLIER_MM = 8.0                 # 拍的：2D 点到直线的内点容差
LINE_MIN_SPAN_MM = 100.0             # 拍的：一段直线至少这么长才算数
LINE_EDGE_BAND_FRAC = 0.12           # 拍的：每侧取这么多比例的外缘点去拟合边线
_LINE_DIR_HALF_PX = 400.0            # 把图像直线反投影成平面方向时，基线取多长。
                                     # 平面是平的，基线长短不影响方向，只要够长
                                     # 盖过像素量化就行 —— 400px 是这个视野的
                                     # 大半幅，不是按场景试出来的。
SIZE_TOL_MM = 80.0                   # 候选矩形「装得下点集」的容差。
                                     # 不能拍小了：边线由**外缘带的质心**拟合，
                                     # 带整体比真实边界内缩约半个带宽
                                     # （0.12 × 530 / 2 ≈ 32mm），两条边的交点
                                     # 因此比真实角点往里缩，点集会「探出」交点。
                                     # 实测挖 0% 时 a.min()=-26mm —— 拍 25 会整批
                                     # 候选被毙。同时它远小于朝向猜错时的溢出量
                                     # （530 vs 350 差 180mm），区分度不受影响。

SIZE_OUTLIER_PCT = 1.0               # 「装得下点集」用这个分位数判，不用极值。
                                     # 极值判据一个离群点就够把候选全毙掉 ——
                                     # 实测 6_test 旋转 10° 时外接框从 274x182 涨到
                                     # 302x227、inbox 多吞邻箱的点，b 的极值冲到
                                     # 633.9mm（箱子长边才 530），四个候选全灭
                                     # （no_fitting_orientation）。而朝向猜错时
                                     # 是一**大批**点溢出，分位数照样区分得开。

DEGRADED_RATIO = 0.9                 # 拍的：朝向歧义两解分数比值高于此 -> degraded

# --------------------------------------------------------------------------- #
# 箱子朝向提示（操作员/上游服务给的先验）—— 专门用来掐掉 90° 转向
# --------------------------------------------------------------------------- #
# 语义**只有一个**：箱子那条**长边（530mm）**在**图像里**更接近哪根轴。
#   'horizontal' = 长边在图像里更接近水平（u 方向）—— 即「箱子横着」
#   'vertical'   = 长边在图像里更接近垂直（v 方向）—— 即「箱子竖着」
# 判据是「投影后的长边 |Δu| 与 |Δv| 谁大」（`candidate_long_axis`），
# 所以**与平面基、与相机怎么装都不相干**，只看图像里那一眼看到的方向。
# 前提是长边偏离图像轴不超过 45° —— 超了「横/竖」这句话本身就没意义了。
#
# 为什么需要它：候选生成会枚举「530 配哪条轴」两种假设（`long_along_e1`），
# 正常情况下靠「矩形要装得下可见点集」把错的那种筛掉；但箱子被挡得只剩一小条
# 时可见区域近似各向同性，两种假设都能装下，就轮到边支持率去选 —— 而手臂的
# 深度边缘会给错误候选凭空送分（见 `_extrap_mm`）。此时选错就是**整框转 90°**，
# 而输出四角照样是个规规矩矩的矩形，不比对真值根本看不出来。

# `/infer_carton_pose` 的 `tape_orientation_deg` → 朝向提示。
# ⚠️ 这里假定**封箱胶带沿箱子的长边贴**（标准开槽纸箱的封条就是沿长边那道缝）。
# 若实物是横着贴（垂直长边），把下面那个 True 改成 False 即可 —— 别改别处。
TAPE_ALONG_LONG_EDGE = True

THETA_FIT_TOL_PX = 3.0               # theta 拟合时，观测点到直线的垂距超过它就当外点。
                                     # 观测点本身的噪声约 2.6px（见 `_lock_theta`），
                                     # 留一倍余量；实际阈值取 max(本值, 3σ_MAD)。

RANSAC_CONFIDENCE = 0.999            # 平面 RANSAC 想以多大概率找到「最大那块」。
                                     # 这是**统计置信度**，不是场景量：找到一块内点率
                                     # 为 w 的平面后，所需的采样次数是
                                     # ln(1-p)/ln(1-w³)；w 大（大块平面）时几十次就够。
                                     # 换成别的数字只是改「漏掉的概率」，换场景不用动。
RANSAC_MAX_ITERS = 500               # 采样次数上限（原来的固定值，现在只是封顶）。
                                     # 小块平面 w 小、公式会给出很大的 N，就直接跑满。
RANSAC_SEARCH_POINTS = 2000          # 假设搜索用的抽稀点数。这是**计算预算**不是场景量：
                                     # 抽稀不改内点率 w，所以每次采样成功的概率与用全量
                                     # 点完全一样，只影响每次采样扫多少点。
                                     # 取 2000：最小认可的平面（`MIN_PLANE_POINTS` = 800 点，
                                     # 占 ROI 的 0.86%）在这个预算下仍能分到 ~17 个采样点、
                                     # 几百个候选三点组合；实测再往下调不提速（119~122ms
                                     # 是平台），所以停在留够余量的位置。
RANSAC_CHUNK = 128                   # 一批评估多少组假设。**计算预算**，不是场景量：
                                     # 大批省「每次 numpy 调用的固定开销」，小批省内存
                                     # （一批要开 chunk × 搜索点数的距离矩阵）。
                                     # 128 × 4000 × 8B ≈ 4MB，正合适。

BOUNDARY_ITER = 2000                 # RANSAC 采样次数（模型是 y = a·x + b，两点定
                                     # 线）。与场景无关的通用量：2000 次对「离群点
                                     # 不超过一半」的两点模型，失败概率可忽略。
BOUNDARY_MAX_SLOPE = 1.0             # 包络线 |dy/dx| 超过它就不算上下边（>45°）。
                                     # 这是算法**输入前提**的表述（箱子相对相机大致
                                     # 轴对齐），不是可以按场景调的阈值。
BOUNDARY_MIN_COLS_FRAC = 0.25        # 一条边界至少要占可见列数的这个比例才算找到了。
                                     # 用**比例**不用绝对列数：绝对列数会随箱子在
                                     # 图像里的大小变，换个机位/距离就得重调。
BOUNDARY_MIN_COLS_ABS = 6            # 比例之外的绝对下限：几个点的「直线」没有意义。
                                     # 这是数学下限（两点定线 + 几个余量），不是场景量。


def load_frame(sequence: str | Path, stem: str
               ) -> tuple[np.ndarray, np.ndarray, CameraIntrinsics]:
    """读一帧 RGB-D 和内参。深度保持 uint16 mm（0 = 无效）。"""
    sequence = Path(sequence)
    color = read_image(sequence / 'color_frames' / f'{stem}.png', cv2.IMREAD_COLOR)
    depth = read_image(sequence / 'depth_frames' / f'{stem}.png', cv2.IMREAD_UNCHANGED)
    if color is None:
        raise FileNotFoundError(f'读不到彩色帧：{sequence}/color_frames/{stem}.png')
    if depth is None:
        raise FileNotFoundError(f'读不到深度帧：{sequence}/depth_frames/{stem}.png')
    k = load_intrinsics(sequence)
    if k is None:
        raise FileNotFoundError(f'读不到内参：{sequence}')
    return color, depth.astype(np.uint16), k


def roi_from_box(box_uv, shape: tuple[int, int]) -> tuple[int, int, int, int]:
    """检测框 -> ROI (x0, y0, x1, y1)，裁剪到图像内。

    **不做外扩**（2026-09-22，操作员裁定）。原来外扩 25%，理由是「框只框住可见
    部分，真实箱子会超出框」。实测这个前提不成立：

      * **平面查找结果完全一样** —— 5/6/8/9_test 四个序列，外扩 0.25 与 0 选中的
        是同一个箱顶面，span 差 < 2mm、`z_ref` 差 < 0.3mm。外扩只让 RANSAC 多看
        到一圈框外点（`n_planes` 5→3/4），多出来的平面一个都没被选中。
      * **有真值的两个序列上精度一模一样** —— 6_test 四角最大误差两档差 ≤ 0.23px，
        5_test 下边两角完全相同（15.0 / 4.7px）。外扩没带来任何精度收益。
      * **外扩反而稀释平面打分** —— 打分是 `点数 × 落在框内的比例`，框外点只拉低
        比例。8_test 外扩 0.25 时选中平面 43545 点但只有 64% 在框内；不外扩时
        27939 点、99.8% 在框内。

    操作员原话：「哪怕我的 bbox 稍小，框住的也是我的正常平面。」
    """
    u1, v1, u2, v2 = (float(t) for t in box_uv)
    x0, x1 = min(u1, u2), max(u1, u2)
    y0, y1 = min(v1, v2), max(v1, v2)
    h, w = shape
    return (max(0, int(np.floor(x0))), max(0, int(np.floor(y0))),
            min(w, int(np.ceil(x1))), min(h, int(np.ceil(y1))))


def valid_depth_points(depth: np.ndarray, k: CameraIntrinsics,
                       roi: tuple[int, int, int, int]) -> np.ndarray:
    """ROI 内有效深度像素 -> 相机系 3D 点 (N,3)，单位 mm。"""
    x0, y0, x1, y1 = roi
    sub = depth[y0:y1, x0:x1]
    vs, us = np.nonzero(sub > 0)
    if us.size == 0:
        return np.zeros((0, 3), np.float64)
    z = sub[vs, us].astype(np.float64)
    return _backproject((us + x0).astype(np.float64), (vs + y0).astype(np.float64), z, k)


# --------------------------------------------------------------------------- #
# 深度不连续图 —— 本脚本的主要观测源
# --------------------------------------------------------------------------- #
def depth_discontinuity(depth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """深度不连续图：|∇depth|，单位 mm/px。

    实测箱子和托盘颜色接近，RGB 梯度不可靠；而箱子比周围高 100mm+，
    深度阶跃与颜色无关。这是把 refine_pallet_frame 的观测源换掉的那一步 ——
    `_snap_edge` 只把传进去的 gx/gy 当梯度图用，所以只需换这里。

    两处必须做的事：
      1. 无效值（0）先填掉。0 和 1200mm 之间是 1200 的假阶跃，不填满图都是假边。
      2. 中值滤波。深度图噪声在 1~2px 尺度上能造出几十 mm/px 的假梯度。
    """
    d = depth.astype(np.float32)
    invalid = d <= 0.0
    if invalid.any():
        # 用有效值的中值填 —— 不追求物理正确，只是不让 0 造出假阶跃，
        # 且填充后是平坦区域，梯度为 0，不会产生观测。
        fill = float(np.median(d[~invalid])) if (~invalid).any() else 0.0
        d[invalid] = fill
    d = cv2.medianBlur(d, 5)
    gx = cv2.Sobel(d, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(d, cv2.CV_32F, 0, 1, ksize=3)
    return gx, gy


# --------------------------------------------------------------------------- #
# 顶面平面分割与平面几何
# --------------------------------------------------------------------------- #
def _fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
    """最小二乘平面。返回 (单位法向, 质心)；点共线时返回 (None, None)。"""
    centroid = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points - centroid, full_matrices=False)
    if singular[0] <= 1e-12 or singular[1] / singular[0] < 1e-6:
        return None, None
    return vt[-1], centroid


def _thin(points: np.ndarray, budget: int = RANSAC_SEARCH_POINTS) -> np.ndarray:
    """等间隔抽稀到不超过 `budget` 点 —— **只给 RANSAC 的「假设搜索」用**。

    为什么抽稀是**无损**的：RANSAC 每次采样成功（三点共面）的概率只取决于
    **内点率 w**，而等间隔抽稀不改变 w（按同样的比例抽内点/外点）。所以
    「500 次采样能找到某块平面的概率」和用全量点时**完全一样**。

    真正会变的只有两件事，都不影响结果：
      * 每次采样要扫的点数（← 这才是省下来的时间）；
      * 落在某块平面上的**具体**是哪几个点（但平面本来就是从点集里拟合出来的，
        抽稀后的子集同样在平面上）。

    ⚠️ **判内点和最终 `_fit_plane` 一律回全量点**，绝不拿抽稀后的点当结果 ——
    平面质心就是 `flat_plane` 的原点，是整套流程最敏感的量：实测原点被带偏 90mm
    会让矩形整体挪远、投影缩小 10.5%、近边抬高 38px（见 WORKLOG §12.1）。

    用等差抽稀而不是随机抽：`points[::k]` 是 O(1) 的切片，不用洗牌；点云是
    ROI 的栅格序，等间隔采样天然铺满整个 ROI。也**不用体素**：体素要
    `np.unique` 排序（一帧 5 次 ≈ 6ms），而且把点挪到体素均值上会轻微改动几何。
    """
    n = len(points)
    if n <= budget:
        return points
    return points[:: int(np.ceil(n / float(budget)))]


def _ransac_plane(points: np.ndarray, tol_mm: float, min_points: int,
                  rng: np.random.Generator,
                  iters: int = RANSAC_MAX_ITERS) -> tuple:
    """RANSAC 找**最大的一块平面**。

    不能只用「全点最小二乘 + 剔离群」：ROI 里同时有箱子顶面和托盘台面时，
    两者平行却相距几百 mm，一次最小二乘会拟合出夹在中间的平面，
    然后两边都落在容差外 —— 内点数归零，一个平面都找不到（2026-09-20 实测踩过，
    合成数据 z=800/z=1100 两片平行面直接返回空列表）。
    改为先随机采 3 点定平面、数内点，取内点最多的那个。

    **速度**（2026-09-21，原来固定跑 500 轮、每轮扫全量 9.3 万点，单帧 864ms 里
    它占 87.5%）：
      * **抽稀**只用于假设搜索（见 `_thin`）—— 不改内点率，采样成功率不变；
      * **自适应迭代数**：找到一块之后按 `N = ln(1-p)/ln(1-w³)` 收紧所需次数
        （`w` = 当前最好内点率）。大块平面 w 大，几十次就够；只有小块才跑满上限。
        这不是「写死一个小的 iters」—— 概率保证 `RANSAC_CONFIDENCE` 还在；
      * `rng.integers` 代替 `rng.choice(..., replace=False)`（后者内部要洗牌，
        贵几十倍）；
      * **批量**采样+评估（`RANSAC_CHUNK`）—— 逐次做的话一组假设要发十几个
        numpy 调用，2500 组的固定开销就 75ms。
    """
    P = _thin(points)
    n_s = len(P)
    best_p0, best_nv, best_n = None, None, 0
    need = float(iters)
    done = 0
    while done < iters:
        # **批量**采一批假设再一起评估：逐次做的话，一组假设要发十几个 numpy
        # 调用（每次 ~5µs 开销），2500 组就是 30µs×2500 ≈ 75ms —— 实测抽稀到
        # 多少点都省不掉这部分（预算 4000→500 只快 3ms），因为瓶颈根本不在
        # 点数上。批量之后变成每批十几次调用。
        k = min(RANSAC_CHUNK, iters - done)
        idx = rng.integers(0, n_s, size=(k, 3))
        ok = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
        idx = idx[ok]
        done += k
        if len(idx) == 0:
            continue
        A = P[idx[:, 0]]
        U = P[idx[:, 1]] - A
        V = P[idx[:, 2]] - A
        NV = np.cross(U, V)
        L = np.linalg.norm(NV, axis=1)            # 批量归约，比逐个 norm 便宜得多
        good = L > 1e-9
        if not np.any(good):
            continue
        A, NV, L = A[good], NV[good], L[good][:, None]
        AN = A * (NV / L)
        # D[k, j] = |(P[j] - A[k])·n[k]|
        D = np.abs(P @ (NV / L).T - AN.sum(axis=1)[None, :])
        cnt = (D <= tol_mm).sum(axis=0)
        j = int(np.argmax(cnt))
        if int(cnt[j]) > best_n:
            best_n = int(cnt[j])
            best_p0 = A[j]
            best_nv = NV[j] / L[j, 0]
            w = best_n / float(n_s)                   # 内点率
            if w >= 0.999:                            # 完美平面，不用再找
                break
            denom = float(np.log1p(-min(w ** 3, 1.0 - 1e-12)))
            if denom < 0.0:
                need = min(float(iters),
                           float(np.log1p(-RANSAC_CONFIDENCE)) / denom)
        if done >= need:
            break
    if best_nv is None:
        return None, None, None
    # —— 回**全量点**重判内点、重拟合（抽稀只用来选假设）——
    inl = np.abs((points - best_p0) @ best_nv) <= tol_mm
    if int(inl.sum()) < min_points:
        return None, None, None
    nrm, origin = _fit_plane(points[inl])
    return nrm, origin, inl


def split_planes(pts3d: np.ndarray, *, tol_mm: float = PLANE_INLIER_MM,
                 min_points: int = MIN_PLANE_POINTS, rounds: int = 5,
                 iters: int = 500) -> list[dict]:
    """迭代平面分割：每轮找最大的一块平面、剔掉内点、再来一轮。

    返回按内点数从多到少排序的平面列表，每个含 {'nrm','origin','n_points','points'}。
    内点少于 min_points 的直接丢弃。
    """
    remaining = pts3d
    out: list[dict] = []
    rng = np.random.default_rng(0)
    for _ in range(rounds):
        if len(remaining) < min_points:
            break
        nrm, origin, inl = _ransac_plane(remaining, tol_mm, min_points, rng, iters)
        if nrm is None:
            break
        for _ in range(3):                      # 用内点最小二乘精修
            inl2 = np.abs((remaining - origin) @ nrm) <= tol_mm
            if inl2.sum() < 3:
                break
            n2, o2 = _fit_plane(remaining[inl2])
            if n2 is None:
                break
            nrm, origin, inl = n2, o2, inl2
        if inl.sum() < min_points:
            break
        out.append({'nrm': nrm, 'origin': origin, 'n_points': int(inl.sum()),
                    'points': remaining[inl]})
        remaining = remaining[~inl]
    out.sort(key=lambda p: -p['n_points'])
    return out


def plane_basis(nrm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """给定法向，造一组平面内的正交单位基 (E1, E2)。"""
    nrm = nrm / np.linalg.norm(nrm)
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(float(tmp @ nrm)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    e1 = np.cross(nrm, tmp)
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(nrm, e1)
    return e1, e2 / np.linalg.norm(e2)


def project_to_plane(pts3d: np.ndarray, nrm: np.ndarray,
                     origin: np.ndarray) -> np.ndarray:
    """把 3D 点投到平面上，返回平面内 2D 坐标 (N,2)，单位 mm。"""
    e1, e2 = plane_basis(nrm)
    rel = pts3d - origin
    return np.c_[rel @ e1, rel @ e2]


def unproject_from_plane(pts2d: np.ndarray, nrm: np.ndarray,
                         origin: np.ndarray) -> np.ndarray:
    """`project_to_plane` 的逆。"""
    e1, e2 = plane_basis(nrm)
    a = np.asarray(pts2d, np.float64).reshape(-1, 2)
    return origin[None, :] + a[:, 0:1] * e1[None, :] + a[:, 1:2] * e2[None, :]


def _plane_size_ok(points: np.ndarray, plane: dict, target_mm,
                   tol_frac: float = PLANE_SIZE_TOL_FRAC
                   ) -> tuple[bool, list[float]]:
    """这个平面有没有可能是一块 530x350 的箱子顶面？

    判据是「外接矩形不能明显偏大」—— **偏小是允许的**（被手臂遮掉一截），
    偏大则说明它根本不是箱子。5_test 实测：整图大框时，1200x1000 的托盘台面
    （53102 点）以 27650 分击败了 27197 点的箱子顶面（21754 分），
    加上这条之后托盘被筛掉、箱子留下。
    """
    if len(points) < 20:                    # 框内点太少 -> 无从判断，放行交给后面的步骤
        return True, [0.0, 0.0]
    p2 = project_to_plane(points, plane['nrm'], plane['origin'])
    lo = np.percentile(p2, 2.0, axis=0)
    hi = np.percentile(p2, 98.0, axis=0)
    span = np.sort(hi - lo)[::-1]              # 长边在前
    ok = (span[0] <= target_mm[0] * (1.0 + tol_frac)
          and span[1] <= target_mm[1] * (1.0 + tol_frac))
    return bool(ok), [round(float(s), 1) for s in span]


def segment_top_plane(depth: np.ndarray, k: CameraIntrinsics,
                      roi: tuple[int, int, int, int], box_uv,
                      opts: dict | None = None,
                      target_mm: tuple[float, float] = DEFAULT_TARGET_MM
                      ) -> tuple[dict | None, np.ndarray | None, dict]:
    """在 ROI 内找出「箱子顶面」平面。

    硬筛：点数够、正面朝向相机、**外接矩形不明显大于目标箱子**。
    打分 = 内点数 × 落在原始框内的比例 —— 框是「可见部分」，落在框内的点
    属于箱子的把握最大。
    """
    opts = opts or {}
    pts = valid_depth_points(depth, k, roi)
    diag: dict = {'n_points_roi': int(len(pts))}
    if len(pts) < MIN_PLANE_POINTS:
        diag['reject'] = 'roi_too_few_points'
        return None, None, diag

    planes = split_planes(pts, min_points=int(opts.get('min_plane_points',
                                                       MIN_PLANE_POINTS)))
    diag['n_planes'] = len(planes)
    if not planes:
        diag['reject'] = 'no_plane'
        return None, None, diag

    bu0, bv0 = min(box_uv[0], box_uv[2]), min(box_uv[1], box_uv[3])
    bu1, bv1 = max(box_uv[0], box_uv[2]), max(box_uv[1], box_uv[3])

    best, best_score, best_det = None, -1.0, {}
    for pl in planes:
        px = _project_px(pl['points'], k)
        view = -pl['origin'] / (float(np.linalg.norm(pl['origin'])) + 1e-9)
        facing = abs(float(pl['nrm'] @ view))
        in_box = ((px[:, 0] >= bu0) & (px[:, 0] <= bu1)
                  & (px[:, 1] >= bv0) & (px[:, 1] <= bv1))
        frac = float(in_box.mean()) if len(px) else 0.0
        # 尺寸判据**只用落在框内的点**量：ROI 可能比框大（诊断脚本可以显式外扩），
        # 6_test 里右边挨着另一个已摆好的箱子，外扩把它圈了进来，两个箱子的顶面在深度上
        # 连成一整块（实测 678x461mm），尺寸判据就把它当成箱子、把真正的箱子
        # （330x297，只有它一半点数）压掉 → 五帧全 reject。
        # 框内只有目标箱子，所以只按框内部分判尺寸。
        ok_size, span = _plane_size_ok(pl['points'][in_box], pl, target_mm)
        sc = (float(pl['n_points']) * frac
              * (1.0 if (facing >= PLANE_FACING_MIN_COS and ok_size) else 0.0))
        det = {'n_points': pl['n_points'], 'frac_in_box': round(frac, 3),
               'facing': round(facing, 3), 'span_mm': span, 'size_ok': ok_size,
               'bbox_uv': [round(float(px[:, 0].min()), 1),
                           round(float(px[:, 1].min()), 1),
                           round(float(px[:, 0].max()), 1),
                           round(float(px[:, 1].max()), 1)],
               'score': round(sc, 1)}
        diag.setdefault('planes', []).append(det)
        if sc > best_score:
            best, best_score, best_det = pl, sc, det
    if best is None or best_score <= 0.0:
        diag['reject'] = 'no_facing_plane'
        return None, None, diag

    # 全图掩码：落在该平面上、且到平面距离在容差内的像素
    h, w = depth.shape
    mask = np.zeros((h, w), bool)
    z = depth.astype(np.float64)
    vs, us = np.nonzero(z > 0)
    P = _backproject(us.astype(np.float64), vs.astype(np.float64), z[vs, us], k)
    keep = np.abs((P - best['origin']) @ best['nrm']) <= PLANE_H_TOL_MM
    mask[vs[keep], us[keep]] = True
    diag['selected'] = best_det
    return best, mask, diag


# --------------------------------------------------------------------------- #
# 2D 点集 -> 直线段 -> 候选矩形
# --------------------------------------------------------------------------- #
def extract_line_segments(pts2d: np.ndarray, *, inlier_mm: float = LINE_INLIER_MM,
                          min_span_mm: float = LINE_MIN_SPAN_MM,
                          band_frac: float = LINE_EDGE_BAND_FRAC) -> list[dict]:
    """找出点集外轮廓上的四条边线，按长度降序返回。

    三次踩坑换来的做法（都在这一个函数里）：

      * **不用凸包顶点**：凸包只留十几个顶点，530×350 实测 16 个，长边上不足
        4 个，凑不齐一条边（实测只提出 1 段、跨度 334mm）。
      * **轮廓点 + RANSAC 也不行**：栅格化的轮廓是锯齿状的，RANSAC 会吸到
        阶梯的对角线上（实测 6 段、最长两段平行，都不是箱子的边）。
      * **`approxPolyDP` 也不行**：锯齿幅度超过简化容差，实测仍留 37 个顶点。

    最终做法：`cv2.minAreaRect`（旋转卡壳，对凸包操作，抗噪）定主方向，
    再在该方向的**四个侧面各取一段外缘带**（各 12% 的点）拟合直线。
    得到的是「贴着可见点集边界」的四条线 —— 哪些是箱子的真边、哪些是被
    手臂截断留下的假边界，交给后面的深度不连续观测判定。
    """
    pts = np.asarray(pts2d, np.float64).reshape(-1, 2)
    if len(pts) < 20:
        return []
    rect = cv2.minAreaRect(pts.astype(np.float32))
    box = cv2.boxPoints(rect)                       # (4,2)，顺序确定
    e1 = np.asarray(box[1], np.float64) - np.asarray(box[0], np.float64)
    L = float(np.hypot(*e1))
    if L < 1e-9:
        return []
    e1 = e1 / L
    e2 = np.array([-e1[1], e1[0]])
    a = pts @ e1
    b = pts @ e2
    if (b.max() - b.min()) > (a.max() - a.min()):
        e1, e2 = e2, -e1                            # 让 e1 指向跨度大的一维
        a, b = b, -a

    out: list[dict] = []
    # ⚠️ 减号侧的外推方向必须是**负**的：`u-` 是坐标最小的一侧，向外是 -e1。
    # 传成 +e1 会让它朝**内**推半个带宽（实测 0.12×505.5/2 = 30.3mm ≈ 15px@z750），
    # 整个矩形跟着偏移 —— 这正是「左边在 u=268 而可见区域左边在 253」的来源。
    for side, coord, edir in (('u+', a, e1), ('u-', a, -e1),
                              ('v+', b, e2), ('v-', b, -e2)):
        frac = [1.0 - band_frac, 1.0] if side.endswith('+') else [0.0, band_frac]
        lo, hi = np.percentile(coord, [f * 100.0 for f in frac])
        m = (coord >= lo) & (coord <= hi)
        if int(m.sum()) < 10:
            continue
        band = pts[m]
        c = band.mean(axis=0)
        _, _, vt = np.linalg.svd(band - c, full_matrices=False)
        dirv = vt[0] / np.linalg.norm(vt[0])
        if float(dirv @ edir) < 0:                  # 统一符号，side 才有意义
            dirv = -dirv
        # 把边线沿**向外**的方向推半个带宽。外缘带的质心比真实边界内缩约
        # 半个带宽（band_frac × 该维跨度 / 2），不补这一下两条边线的交点
        # （= 初始角点）会整体内缩，矩形对边被推远，±40px 的观测搜索窗够不着 ——
        # 实测真实数据上「经过角点的两条边」有观测、「对边」一条都没有，就是这个。
        band_mm = band_frac * float(coord.max() - coord.min())
        c = c + edir * (band_mm / 2.0)
        t = (band - c) @ dirv
        span = float(t.max() - t.min())
        if span >= min_span_mm:
            out.append({'p0': c + t.min() * dirv, 'p1': c + t.max() * dirv,
                        'dir': dirv, 'span_mm': span, 'n_inlier': int(m.sum()),
                        'side': side})
    out.sort(key=lambda s: -s['span_mm'])
    return out


def _line_intersection(sa: dict, sb: dict):
    """两条直线（由段给出）的交点；平行时返回 None。"""
    p, r = np.asarray(sa['p0'], np.float64), np.asarray(sa['dir'], np.float64)
    q, s = np.asarray(sb['p0'], np.float64), np.asarray(sb['dir'], np.float64)
    denom = r[0] * s[1] - r[1] * s[0]
    if abs(denom) < 1e-9:
        return None
    t = ((q[0] - p[0]) * s[1] - (q[1] - p[1]) * s[0]) / denom
    return p + t * r


def _inward_dir(d: np.ndarray, o: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """从角点 o 出发，沿 d 还是 -d 才是走进点集？"""
    t = (pts - o[None, :]) @ d
    return d if abs(float(t.max())) >= abs(float(t.min())) else -d


def _extrap_mm(seg: dict, o) -> float:
    """角点 o 要沿这条边线**外推多远**才够到 —— 落在可见段内就是 0。

    为什么要这个量：候选角是「两条边线求交点」。如果交点落在两条边各自
    **被实际点支撑的那一段**上，这个角就是边长亲眼看到的；如果得把边线
    凭空延长一大截才交得上，那个角是**猜**出来的 —— 边线自己 2~3° 的
    拟合误差会被放大量级地放大（实测帧2 的 `u-/v-` 外推 208mm，而它自己
    只有 216mm 长，最终偏 14px）。
    """
    d = np.asarray(seg['dir'], np.float64)
    t = float((np.asarray(o, np.float64).reshape(-1) - np.asarray(seg['p0'], np.float64)) @ d)
    return max(0.0, -t, t - float(seg['span_mm']))


def normalize_orientation(value) -> str | None:
    """把各种写法的朝向提示归一成 `'horizontal'` / `'vertical'` / `None`。

    接受（大小写不敏感）：`'horizontal'` / `'h'` / `'横'` / `'长边水平'`、
    `'vertical'` / `'v'` / `'竖'` / `'长边竖直'`；以及**度数的字面量**
    `0` / `90` / `'0'` / `'90'`（`/infer_carton_pose` 的 `tape_orientation_deg`
    就是这个口径：0=horizontal、90=vertical、-1=unknown）。
    `None` / `'auto'` / `'unknown'` / `-1` / 空串 → `None`（不提示，按原逻辑自动选）。

    只认这些，别的一律**报错**而不是默默当没有 —— 提示是上游给的，写错了要当场知道，
    否则「我以为用了提示」和「其实没用」在结果上分不出来。
    """
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError(f'朝向提示不接受布尔值：{value!r}')
    if isinstance(value, (int, float, np.integer, np.floating)):
        return tape_orientation_to_hint(int(value))
    s = str(value).strip().lower()
    if s in ('', 'auto', 'none', 'unknown', 'any', '-1'):
        return None
    if s in ('horizontal', 'h', 'horiz', '横', '横着', '横放', '长边水平', '0'):
        return 'horizontal'
    if s in ('vertical', 'v', 'vert', '竖', '竖着', '竖放', '长边竖直', '90'):
        return 'vertical'
    raise ValueError(f'认不出的朝向提示 {value!r}；'
                     f"要么 'horizontal'，要么 'vertical'，要么 None/-1（不提示）")


def tape_orientation_to_hint(deg: int) -> str | None:
    """`/infer_carton_pose` 的 `tape_orientation_deg` → 长边朝向。

    0 = horizontal、90 = vertical、-1（或其它）= unknown → `None`。
    折算见 `TAPE_ALONG_LONG_EDGE`：胶带沿长边贴时两者同向，横着贴时差 90°。
    """
    if deg not in (0, 90):
        return None
    horiz = (deg == 0)
    if not TAPE_ALONG_LONG_EDGE:
        horiz = not horiz
    return 'horizontal' if horiz else 'vertical'


def frame_long_axis(frame: dict, k: CameraIntrinsics) -> str | None:
    """一个已定型的矩形 frame，它的**长边**投影到图像后更接近水平还是垂直？

    判据就是「投影后 |Δu| 与 |Δv| 谁大」—— 没有任何阈值：45° 是两者相等的那个点，
    也是「横」和「竖」这两句话本身的分界，不是可以按场景调的量。

    投影不合法（角点在相机后方）返回 `None`，调用方按「判不了」处理。
    """
    W, H = float(frame['W']), float(frame['H'])
    c4 = _corners_mm(frame, W, H)
    # 角点顺序 (0,0)->(W,0)->(W,H)->(0,H)：第 0 条边沿 E1（长 W），第 1 条沿 E2（长 H）。
    i0, i1 = (0, 1) if W >= H else (0, 3)
    if np.any(c4[[i0, i1], 2] <= 1.0):
        return None
    px = _project_px(c4[[i0, i1]], k)
    du = float(px[1, 0] - px[0, 0])
    dv = float(px[1, 1] - px[0, 1])
    if du == 0.0 and dv == 0.0:
        return None
    return 'horizontal' if abs(du) >= abs(dv) else 'vertical'


def candidate_long_axis(cand: dict, plane: dict,
                        k: CameraIntrinsics) -> str | None:
    """候选矩形升成 frame 后看长边朝向（见 `frame_long_axis`）。"""
    return frame_long_axis(candidate_to_frame(cand, plane), k)


def filter_by_orientation(cands: list[dict], plane: dict, k: CameraIntrinsics,
                          hint: str | None) -> tuple[list[dict], dict]:
    """按朝向提示把候选筛一遍。返回 (留下的候选, 诊断)。

    诊断里 `applied` 说明这次到底用没用上提示 —— 操作员给的先验属于「人工输入」，
    项目规矩是**用了必须留痕**。

    ⚠️ **提示与数据冲突时不回退，直接判失败**（操作员 2026-09-22 定）：
    「宁可没有结果走 yolo 框回归这条 fallback，也不能接受错误的结果」。
    原来是「没一个候选对得上就退回全部候选、只标 conflict」—— 那等于**明知
    先验和数据打架，还拿一个先验说不通的朝向去输出**。上游既然给了朝向，
    它说不对就是对不上；调用方（`fit_box_frame`）据此 reject，窗口层自然会
    退回 YOLO 框，并把这个冲突一路带到 payload 的 `orientation='conflict'` 上。
    """
    diag: dict = {'hint': hint, 'applied': False, 'conflict': False,
                  'n_in': len(cands)}
    if hint is None or not cands:
        diag['n_out'] = len(cands)
        return list(cands), diag
    match, axes = [], []
    for c in cands:
        ax = candidate_long_axis(c, plane, k)
        axes.append(ax)
        if ax == hint:
            match.append(c)
    diag['axes'] = axes
    if not match:
        diag['conflict'] = True
        diag['n_out'] = 0
        diag['reject'] = 'orientation_conflict'
        return [], diag
    diag['applied'] = True
    diag['n_out'] = len(match)
    return match, diag


def initial_frame_candidates(segs: list[dict], pts2d: np.ndarray,
                             target_mm: tuple[float, float],
                             e1_hint: np.ndarray | None = None
                             ) -> tuple[list[dict] | None, dict]:
    """从四条边线 + 尺寸约束造候选矩形（不做选择，选择交给支持率）。

    枚举「哪两条边围出了那个角」（u± × v± 共 4 个组合 —— 相邻的两条边必
    不平行，其交点就是箱子的一个角），每个角最多 1 个候选。

    ⚠️ **不再枚举「530 配哪条轴」**（2026-09-22 删，见循环里的长注释）：
    e1 取自 `u±`，而 `u±` 按构造就是点集短边那一对，所以 530 必然沿 e2。

    **e1（矩形短轴）的方向**：给了 `e1_hint` 就用它 —— 那是 mask 上/下边界
    融合出来的（见 `_boundary_axis`，§24）。没给才退回「u± 里 span_mm 最大的
    那条边」，那条路在 9_test 上偏 −11°、会把候选整体转歪 12~18°。
    退回不是「编个数」，是「没有额外信息时用原来的信息」。

    被手臂截断的那一侧，边线是「贴着可见点集」的假边界；用「矩形必须装得下
    **所有**可见点」这条把用假边界当角的路筛掉 —— 朝向或角猜错时点集必然溢出。
    """
    diag: dict = {'n_segments': len(segs)}
    pts = np.asarray(pts2d, np.float64).reshape(-1, 2)
    by_side = {s['side']: s for s in segs if 'side' in s}
    diag['sides'] = sorted(by_side)
    if len(by_side) < 2:
        diag['reject'] = 'no_edges'
        return None, diag

    # 水平方向（e1）用**两条 u 边里更完整的那条**定，而不是循环里碰巧轮到的 sa。
    # u± 就是上下边、即操作员关心的「下边方向」的来源；而 u+ 恰恰经常被手臂
    # 挡住（实测 234mm vs u- 的 339mm），拿被挡的那条定方向会让角度在 ±3° 之间
    # 乱跳 —— 同一静止场景五帧实测 -3.59° ~ +2.10°。用 span 大的那条之后波动收敛。
    # 也不要改用长边（v±，竖直）转 90°：实测那会系统性偏 -4°。
    # 更**不要用 SVD 正交化**：ia 与 ib 近似正交时两个奇异值几乎相等，
    # SVD 给出的基是任意的（实测直接返回单位基，四个角一个都凑不出来）。
    #
    # ⚠️ **这一条仍然不稳**（2026-09-22 记录，没修）：`span_mm` 是「这条边**露
    # 出来**多长」而不是「这条边有多长」，被手挡掉一半就变。9_test 的 u± 外缘带
    # 沿边有 30~107mm 空隙、残差 p90 48~70mm（其他三个序列 23~31mm），e1 方向在
    # 静止场景五帧里摆动 48°（108/113/123/78/75°；6_test 2.3°、5_test 1.2°、
    # 8_test 0.4°）。后果是点集在 a 方向的投影跨度虚增（9_test 真实 365mm，
    # 量出 420~456mm），`a_hi <= 430` 这条就把**正确朝向**筛掉、只留错的那支 ——
    # 905753/906941 零候选、874568 输出竖框，都是这个。见 WORKLOG §23。
    u_cands = [by_side[k] for k in ('u+', 'u-') if k in by_side]
    u_best = max(u_cands, key=lambda s: s['span_mm']) if u_cands else None
    # e1 优先用调用方给的（mask 边界算出来的，见 `_boundary_axis`）；没给才退回
    # `u_best` 的方向。下面循环里**只算一次**，四个角共用同一个 e1 —— 否则四个
    # 候选会各转一个角度，「选哪个角」和「选哪个朝向」就缠在一起了。
    e1_fixed = None
    if e1_hint is not None:
        _h = np.asarray(e1_hint, np.float64)
        _n = float(np.linalg.norm(_h))
        if _n > 1e-9:
            e1_fixed = _h / _n

    cands = []
    for su in ('u+', 'u-'):
        for sv in ('v+', 'v-'):
            sa, sb = by_side.get(su), by_side.get(sv)
            if sa is None or sb is None:
                continue
            o = _line_intersection(sa, sb)
            if o is None:
                continue
            # 这个角是「亲眼看到的」还是「延长线够出来的」—— 后者位置不可信，
            # 交给 choose_candidate 折减 support（见 `choose_candidate`）。
            ex = max(_extrap_mm(sa, o), _extrap_mm(sb, o))
            ib = _inward_dir(np.asarray(sb['dir'], np.float64), o, pts)
            if e1_fixed is not None:
                # 边界给的方向只有「一条直线」，符号是任意的；从角点出发要指向
                # **点集内部**才对（否则 a 的跨度全在负半轴，装得下的判据必挂）。
                e1 = _inward_dir(e1_fixed, o, pts)
                e1 = e1 / np.linalg.norm(e1)
            else:
                ia = _inward_dir(np.asarray(u_best['dir'], np.float64), o, pts)
                e1 = ia / np.linalg.norm(ia)
            e2 = np.array([-e1[1], e1[0]])
            if float(e2 @ ib) < 0:
                e2 = -e2
            rel = pts - o[None, :]
            a, b = rel @ e1, rel @ e2
            lo_p, hi_p = SIZE_OUTLIER_PCT, 100.0 - SIZE_OUTLIER_PCT
            a_lo, a_hi = np.percentile(a, [lo_p, hi_p])
            b_lo, b_hi = np.percentile(b, [lo_p, hi_p])
            # ⚠️ **只枚举「530 配 e2」这一支**（2026-09-22，操作员裁定）。
            # e1 取自 `u±`，而 `u±` 按构造就是**点集短边那一对** ——
            # `extract_line_segments` 里 `e1,e2 = e2,-e1` 那一步把 e1 强制指向
            # 点集跨度大的一维，于是 `u±`（沿 e1 的两条边）永远垂直于长边。
            # 四个序列 × 五帧实测全部如此（9_test u± 194~373mm / v± 523~570mm，
            # 6_test 211~244 / 553~573，5_test 234~339 / 506~518，
            # 8_test 269~404 / 527~551）。所以 `long_along_e1=True` 这一支
            # **一次都没对过**，它存在的唯一作用，是给「整框转 90°」那种错
            # 朝向留一条容差更大的旁路（`a_hi <= 530+80` 比 `a_hi <= 350+80`
            # 松 180mm）—— 9_test 874568 正是靠它挤进来的。
            # 删掉它，错朝向就没有出口了。
            W, H = float(target_mm[1]), float(target_mm[0])
            if (a_hi <= W + SIZE_TOL_MM and b_hi <= H + SIZE_TOL_MM
                    and a_lo >= -SIZE_TOL_MM
                    and b_lo >= -SIZE_TOL_MM):
                cands.append({'origin2d': o, 'e1': e1, 'e2': e2,
                              'W': float(W), 'H': float(H),
                              'long_along_e1': False,
                              'corner': f'{su}/{sv}',
                              'extrap_mm': float(ex)})
    diag['n_candidates'] = len(cands)
    diag['corners'] = [c['corner'] for c in cands]
    if not cands:
        # 失败时把每个角点算出来的 a/b 范围留下来：区分「交点算错了」和
        # 「确实装不下」（可见区残缺太多，530x350 的矩形真容不下点集）
        # ⚠️ e1 必须用**循环里那个 u_best**，不能就近用 sa_ —— 两者可能不是
        # 同一条边，用 sa_ 报出来的数会和真正的判据对不上。
        dbg = []
        for su in ('u+', 'u-'):
            for sv in ('v+', 'v-'):
                sa_, sb_ = by_side.get(su), by_side.get(sv)
                if sa_ is None or sb_ is None:
                    continue
                o_ = _line_intersection(sa_, sb_)
                if o_ is None:
                    dbg.append({'corner': f'{su}/{sv}', 'note': 'no_intersection'})
                    continue
                if e1_fixed is not None:
                    ia_ = _inward_dir(e1_fixed, o_, pts)
                else:
                    ia_ = _inward_dir(np.asarray(u_best['dir'], np.float64), o_, pts)
                ib_ = _inward_dir(np.asarray(sb_['dir'], np.float64), o_, pts)
                e1_ = ia_ / np.linalg.norm(ia_)
                e2_ = np.array([-e1_[1], e1_[0]])
                if float(e2_ @ ib_) < 0:
                    e2_ = -e2_
                rel_ = pts - o_[None, :]
                aa_, bb_ = rel_ @ e1_, rel_ @ e2_
                dbg.append({'corner': f'{su}/{sv}',
                            'a': [round(float(aa_.min()), 1), round(float(aa_.max()), 1)],
                            'b': [round(float(bb_.min()), 1), round(float(bb_.max()), 1)]})
        diag['reject_spans'] = dbg
        diag['reject'] = 'no_fitting_orientation'
        return None, diag
    return cands, diag


# --------------------------------------------------------------------------- #
# 候选升 3D + 边观测（深度不连续版）
# --------------------------------------------------------------------------- #
def candidate_to_frame(cand: dict, plane: dict) -> dict:
    """2D 候选 -> 3D frame，直接架在已拟合好的平面基上。

    不需要重新估深度：候选是**平面内**的 2D 几何，平面本身已由点云确定。
    """
    E1, E2 = plane_basis(plane['nrm'])
    o = np.asarray(cand['origin2d'], np.float64)
    origin = plane['origin'] + o[0] * E1 + o[1] * E2
    e1 = cand['e1'][0] * E1 + cand['e1'][1] * E2
    e2 = cand['e2'][0] * E1 + cand['e2'][1] * E2
    return {'origin': origin, 'E1': e1 / np.linalg.norm(e1),
            'E2': e2 / np.linalg.norm(e2), 'nrm': plane['nrm'],
            'W': float(cand['W']), 'H': float(cand['H']),
            'long_along_e1': cand['long_along_e1']}


def _ray_plane_z(uv, nrm: np.ndarray, origin: np.ndarray,
                 k: CameraIntrinsics) -> float | None:
    """像素射线与平面交点的 z（相机系 mm）。平行或交点在背后时返回 None。"""
    d = np.array([(float(uv[0]) - k.cx) / k.fx,
                  (float(uv[1]) - k.cy) / k.fy, 1.0])
    denom = float(d @ nrm)
    if abs(denom) < 1e-9:
        return None
    t = float(origin @ nrm) / denom
    return t if t > 1.0 else None          # d[2] == 1，交点 z 就是 t


def _depth_in_range(uv, edge_dir: np.ndarray, depth: np.ndarray,
                    z_range: tuple[float, float], half: int = 4) -> bool:
    """观测点附近的深度有没有落在顶面深度带内。

    平面被摆正（法向 = 光轴）之后，「射线与平面的交点深度」退化成**一个常数**
    （≈质心深度），而真实顶面深度跨 673~865mm —— 用交点深度当判据的话，上下边
    的观测一进门就全被判超差（实测四条边全灭、reject=no_supported_edge）。
    摆正之后「这个观测点属不属于顶面」的正确判据，是深度落在顶面的深度带里。
    """
    d = np.asarray(edge_dir, np.float64)
    L = float(np.hypot(*d))
    if L < 1e-9:
        return False
    d = d / L
    nvec = np.array([-d[1], d[0]])
    u, v = float(uv[0]), float(uv[1])
    ts = np.arange(-half, half + 1, dtype=np.float64)
    us = np.rint(u + ts * nvec[0]).astype(np.int64)
    vs = np.rint(v + ts * nvec[1]).astype(np.int64)
    h_img, w_img = depth.shape
    ok = (us >= 0) & (us < w_img) & (vs >= 0) & (vs < h_img)
    if not ok.any():
        return False
    zs = depth[vs[ok], us[ok]].astype(np.float64)
    zs = zs[zs > 0]
    if zs.size == 0:
        return False
    return bool(((zs >= z_range[0]) & (zs <= z_range[1])).any())


def _on_hand(uv, edge_dir: np.ndarray, depth: np.ndarray,
             z_range: tuple[float, float], centre_uv) -> bool:
    """这个观测点是不是落在**压在箱子上的手/吸盘**上？

    判据：沿边的法向往**外侧**走，遇到的第一个有效深度比箱顶还近。

    为什么是这个方向 —— 越过箱沿看到的一定是**更远**的东西（地面/托盘/背景），
    而机械手压在箱子**上面**，所以它的外侧一定是**更近**的金属件。这是纯物理
    符号，不含阈值（与 `_envelope_line` 的物理门同一个判据）。用 6_test 四角
    真值量过：四条边的「外侧 − 内侧」分别是下 +184mm、左 +197mm、上 +366mm、
    右 +278mm —— **四条边同一个方向**，不需要按左右分开判。

    外侧由几何定：箱子是凸的，离矩形中心投影更远的那一侧就是外侧。
    ⚠️ 这个「中心」必须是**矩形中心**（四角投影的均值）。原来传的是
    `frame['origin']` —— 那是矩形的**一个角**（`_corners_mm` 里 (0,0) 那个），
    拿角当中心会让近一半观测点的「外侧」指反，门等于没开（8_test 左边实测：
    用角时判据保留/丢弃与随机无异，用中心时才稳定挑出真边那一段）。

    ⚠️ **不能「遇到无效深度就停」**：吸盘是**悬空**的，它旁边就是大片无效深度，
    停在那里等于放行 —— 8_test 左边正是这么漏掉一半手点的（32 个观测里 24 个
    在手上，全部通过）。停不得，必须一路走到 `OUTER_WALK_PX`。
    代价是**可能**够到比箱顶更近的别的东西而误判，但那是「把真边当手丢掉」，
    下面按**连续段**取用就是为了兜住这种误判（丢掉几个点不至于毁掉整条边）。
    """
    d = np.asarray(edge_dir, np.float64)
    L = float(np.hypot(*d))
    if L < 1e-9:
        return False
    nv = np.array([-d[1], d[0]]) / L
    q = np.asarray(uv, np.float64)
    out = (nv if (np.hypot(*(q + 8.0 * nv - centre_uv))
                  > np.hypot(*(q - 8.0 * nv - centre_uv))) else -nv)
    h, w = depth.shape
    for t in np.arange(1.0, OUTER_WALK_PX + 0.5, 1.0):
        p = q + t * out
        ui, vi = int(round(p[0])), int(round(p[1]))
        if 0 <= ui < w and 0 <= vi < h and depth[vi, ui] > 0:
            return bool(float(depth[vi, ui]) < z_range[0])
    return False                      # 整条外法向都没深度 -> 判不了，不罚


def _drop_short_runs(flags: list[bool], min_len: int) -> list[bool]:
    """把长度不足 `min_len` 的 True 连段抹成 False。

    用来滤掉**孤立的**「手」判定：机械手是连续物体，它挡住的一定是连续一段；
    单点被判成手，只可能是深度噪声或某个恰好更近的杂物。实测 6_test 的
    x=0/y=H 各有 1~3 个孤立手点，不滤的话按手段切开会把一条完整的边剁成两截、
    丢掉一半（四角总误差 7.9 → 8.3px）。
    """
    out = list(flags)
    i, n = 0, len(flags)
    while i < n:
        if not flags[i]:
            i += 1
            continue
        j = i
        while j < n and flags[j]:
            j += 1
        if j - i < min_len:
            for k in range(i, j):
                out[k] = False
        i = j
    return out


def _best_clear_span(hand: list[bool], ok: list[bool]) -> tuple[int, int]:
    """按「手」把观测序列切段，取**段内有效点最多**的那一段，返回半开区间。

    切的分隔符是**手**（连续的 `hand=True`），不是 `ok` 的 False —— 这是这个
    函数唯一容易搞错的地方。`ok` 里的空洞有两种：手挡住的（长、连续）和深度
    噪声/无效的（短、零散）。按 `ok` 切会把一条完整的边在噪声处剁碎，然后
    「取最长段」丢掉另一半（实测 6_test 的 y=H 就这么被砍掉 5 个点、整条边
    失效）。按**手**切才符合物理：手是一个连续物体，它挡住的一定是连续的一段。
    """
    best = (0, 0)
    i, n = 0, len(hand)
    while i < n:
        if hand[i]:                      # 跳过一整段手
            i += 1
            continue
        j = i
        while j < n and not hand[j]:
            j += 1
        n_ok = sum(1 for k in range(i, j) if ok[k])
        if n_ok > best[1] - best[0]:
            best = (i, j)
        i = j
    return best


def collect_observations(frame: dict, gx: np.ndarray, gy: np.ndarray,
                         depth: np.ndarray, k: CameraIntrinsics,
                         opts: dict | None = None) -> tuple[list, dict]:
    """四条边各自找观测点，判有效性。返回 (observations, diag)。

    observations 是 `[(edge_i, uv), ...]`，与 `_residuals` 的约定一致。
    扫不到峰就不产生观测 —— 遮挡天然落进「没有观测」，不需要额外的遮挡判断。

    **手/真边混杂时取可用的那一段**（2026-08-22 操作员裁定，见 WORKLOG §22）：
    一条边被机械手挡掉大半时，手的边缘和真边会同时产生深度峰，混在一条边线
    里谁也说不清。做法是

      1. 逐点用 `_on_hand` 判「在不在手上」；
      2. 在剩下的点里取**最长连续段** —— 真边是一段连续的可见边沿，手是另一段，
         中间被 `_depth_in_range` 隔开；
      3. **只把这一段**当这条边的观测喂给拟合。

    不能因为手多就把整条边丢掉（真边那一段还有用），也不能把手的点混进去
    （矩形会整体移位十几像素）。用操作员点出的 8_test 左边真值量过：段内点到
    真值线的中位距离 4.1px（改之前是 8.6px），输出左下角误差 11.3 → 3.1px。

    `frac` 的**分母扣掉手点数**：手挡掉的那部分本来就不该算「这条边该有的采样」，
    不扣的话「真边只剩 7 个点」会被当成「这条边大半没扫到」而误判为无效。
    """
    opts = opts or {}
    grad_min = float(opts.get('grad_min', GRAD_MIN_MM_PER_PX))
    z_range = opts['z_range']              # 顶面点的深度带，由 fit_box_frame 摆正时算好
    W, H = float(frame['W']), float(frame['H'])
    segs = _edges_px(frame, W, H, k)
    # 幅值图**算一次就够**：`_snap_edge` 一帧被调上百次，每次都自己
    # `cv2.magnitude(gx, gy)` 重算整张图是纯浪费（实测占单帧约 8%）。
    mag = cv2.magnitude(gx, gy)
    # 「外侧」的参照点必须是**矩形中心**（四角投影的均值），不是 `frame['origin']`
    # ——后者是矩形的一个角，拿它当中心会让近一半观测点的外侧指反。见 `_on_hand`。
    centre_uv = np.mean([s['p0'] for s in segs if s['ok']], axis=0)
    observations: list[tuple[int, np.ndarray]] = []
    edges_diag = []
    for i, seg in enumerate(segs):
        if not seg['ok']:
            edges_diag.append({'name': seg['name'], 'family': seg['family'],
                               'used': False, 'support': 0.0, 'n_obs': 0,
                               'note': 'unprojectable'})
            continue
        found, n_samples = _snap_edge(gx, gy, seg['p0'], seg['p1'],
                                      SAMPLE_STEP_PX, SEARCH_HALF_PX, grad_min,
                                      mag)
        edge_dir = np.asarray(seg['p1'], np.float64) - np.asarray(seg['p0'], np.float64)
        # 两道过滤，判不了的一律放行（不罚）：
        #   ① 落在手/吸盘上 -> 不是箱子的边（纯物理符号判据）
        #   ② 深度不在顶面深度带里 -> 不是顶面上的点
        hand = [_on_hand(uv, edge_dir, depth, z_range, centre_uv) for uv, _ in found]
        hand = _drop_short_runs(hand, MIN_HAND_RUN)
        ok = [not hd and _depth_in_range(uv, edge_dir, depth, z_range)
              for (uv, _), hd in zip(found, hand)]
        # 按手切成段，取**段内有效点最多**的那一段；段内再逐点按 `ok` 过滤。
        # 一条边整条没被手碰过时只有一段，逐点过滤 = 加门之前的行为（逐位一致）。
        a, b = _best_clear_span(hand, ok)
        kept = [f for f, k in zip(found[a:b], ok[a:b]) if k]
        support, _ = _edge_line_support(kept, SAMPLE_STEP_PX)
        n_hand = sum(hand)
        frac = len(kept) / max(1, n_samples - n_hand)
        used = (support >= EDGE_SUPPORT_MIN and len(kept) >= MIN_EDGE_KEEP
                and frac >= MIN_EDGE_SAMPLES_FRAC)
        if used:
            observations.extend((i, uv) for uv, _ in kept)
        edges_diag.append({'name': seg['name'], 'family': seg['family'],
                           'used': bool(used), 'support': round(float(support), 3),
                           'n_obs': len(kept), 'n_samples': n_samples,
                           'frac': round(float(frac), 3), 'n_hand': n_hand})
    return observations, {'edges': edges_diag}


def choose_candidate(cands: list[dict], plane: dict, gx: np.ndarray, gy: np.ndarray,
                     depth: np.ndarray, k: CameraIntrinsics,
                     opts: dict | None = None) -> tuple[dict | None, dict]:
    """在朝向假设的候选之间用**边支持率**选优 —— 不猜，用证据选。

    分数接近时标 degraded，如实呈现，不硬选。
    """
    scored = []
    for cand in cands:
        frame = candidate_to_frame(cand, plane)
        obs, cdiag = collect_observations(frame, gx, gy, depth, k, opts)
        used = [e for e in cdiag['edges'] if e['used']]
        support = float(sum(e['support'] for e in used))
        # 角点是自己看到的还是延长线够出来的：按外推距离**折减** support。
        # 光看 support 分不出来 —— 左右边的深度峰是手臂的边缘，会给错误候选
        # 凭空送分（见 `_extrap_mm`）。
        # 用**乘法**不用「support − k×外推」：外推距离和 support 量纲不同，
        # 相加就必须定一个 k，而 k 只能靠真值试出来（原来那个 2.0 就是这么来的）
        # —— 换场景就得重调，且是「拿真值凑出来的好效果」。乘法里唯一的常数是
        # 那个 1，含义是「外推为 0 时不折减」，无量纲、无场景含义。
        pen = float(cand.get('extrap_mm', 0.0)) / max(min(cand['W'], cand['H']), 1e-9)
        scored.append({'cand': cand, 'frame': frame, 'obs': obs,
                       'edges': cdiag['edges'],
                       'support': support, 'penalty': pen,
                       'score': support / (1.0 + pen),
                       'families': frozenset(e['family'] for e in used)})
    if not scored:
        return None, {'reject': 'no_candidate'}
    scored.sort(key=lambda s: -s['score'])
    best = scored[0]
    diag = {'candidates': [{'corner': s['cand']['corner'],
                            'long_along_e1': s['cand']['long_along_e1'],
                            'support': round(s['support'], 3),
                            'extrap_mm': round(float(s['cand'].get('extrap_mm', 0.0)), 1),
                            'score': round(s['score'], 3),
                            'families': sorted(s['families'])} for s in scored]}
    if len(scored) > 1 and best['score'] > 0.0:
        ratio = scored[1]['score'] / max(best['score'], 1e-9)
        diag['orientation_ratio'] = round(float(ratio), 3)
        if ratio > DEGRADED_RATIO:
            diag['degraded'] = 'orientation_ambiguous'
    if best['score'] <= 0.0:
        diag['reject'] = 'no_supported_edge'
        return None, diag
    frame = best['frame']
    frame['_obs'] = best['obs']
    frame['_edges'] = best['edges']
    frame['_families'] = best['families']
    diag['edges'] = best['edges']
    diag['picked'] = best['cand']['corner']
    return frame, diag


# --------------------------------------------------------------------------- #
# 求解与分级更新
# --------------------------------------------------------------------------- #
def _wrap_half_turn(dth: float) -> float:
    """把直线方向的差折进 (−π/2, π/2] —— 直线方向有 180° 歧义。"""
    while dth >= np.pi / 2.0:
        dth -= np.pi
    while dth < -np.pi / 2.0:
        dth += np.pi
    return float(dth)


def inbox_pad_px(k: CameraIntrinsics, z_mm: float) -> float:
    """「框内点」的外扩余量（像素）—— 由物理尺寸 `BOX_INBOX_PAD_MM` 折算。

    按物理量而不是像素比例，是为了**与箱子在图像里的旋转角无关**：框会随箱子
    转过角而涨大，像素比例式的余量跟着涨，就会多吞框外邻箱的点。
    """
    return BOX_INBOX_PAD_MM * float(k.fx) / max(float(z_mm), 1.0)


def _clip_to_box(mask: np.ndarray, box_uv, pad_px: float) -> np.ndarray:
    """把 mask 裁到「输入框 + 一点余量」内。

    为什么必须裁：6_test 里右边挨着另一个已摆好的箱子，mask 里含**隔壁那个干扰箱**（顶面与它
    共面同高，RANSAC 分不开），而隔壁箱子的上沿比本箱子的上边更靠上 ——
    实测「mask 最上 10%」的 6077 个点**全在隔壁箱子上**（x∈[420,630]）。
    不裁的话 theta 量的是隔壁箱子的角度，0° 时恰好接近（差 0.08°）纯属
    **两个箱子摆得同角度的巧合**，一转起来就散了。
    """
    h, w = mask.shape
    m = np.zeros_like(mask)
    u0, u1 = int(max(0, box_uv[0] - pad_px)), int(min(w, box_uv[2] + pad_px + 1))
    v0, v1 = int(max(0, box_uv[1] - pad_px)), int(min(h, box_uv[3] + pad_px + 1))
    m[v0:v1, u0:u1] = mask[v0:v1, u0:u1]
    return m


def _envelope_line(mask: np.ndarray, side: str, *, depth: np.ndarray | None = None,
                   iters: int = BOUNDARY_ITER) -> dict | None:
    """mask 上/下边界线的稳健拟合：逐列取包络 → **物理门** → RANSAC 直线。

    逐列取包络（而不是按 y 分位取带）的理由 —— **与箱子在图像里转多少度无关**：
    矩形转过角之后，「图像最上/最下 10% 高度」里装的不再是那条边，而是**一个角
    附近的小三角**，采样短了还混进邻边，实测旋转 15° 时按 y 分位取带漂 7~8°，
    逐列包络只漂 0.6~1.3°。

    边界上混着四种点，按下面三道处理：

    1. **物理门**（给了 depth 时）：真边的外侧一定**更远** —— 越过箱沿看到的
       是地面/托盘/更远的背景。而被夹爪挡住的列，mask 是戛然而止在夹爪上的，
       外侧是**更近**的夹爪。判据是**纯符号比较，不含任何阈值**：往外找第一个
       有效深度，比箱面大就留、小就丢（找不到有效深度就留 —— 判不了就不罚）。
       实测 5_test 下边界 105 列真边 / 27 列夹爪，6_test 107 / 46，分得干净。
    2. **RANSAC**：隔壁箱子的边会在连续若干列上形成「另一条线」。6_test 隔壁
       箱子的上沿比本箱高 13px、占 30/270 列。RANSAC 只认能连成一条线的那段。
    3. **内点阈值由包络自己的量化步长估**：mask 边界是阶梯状的（实测每级 2~5px），
       写死一个像素阈值，换个相机/分辨率就失效。这里取相邻列 y 跳变的**中位数**
       当阈值 —— 阶梯的步长是多少，就容忍多少。

    返回 {'ang'(rad, 折进 (−π/2, π/2]), 'n_inlier', 'n_cols', 'n_dropped',
          'span_px', 'step_px', 'score'}，或 None。方向有 180° 歧义，由调用方归一化。
    """
    if mask is None:
        return None
    col_any = mask.any(axis=0)
    cols = np.nonzero(col_any)[0]
    if len(cols) == 0:
        return None
    h = mask.shape[0]
    # —— 逐列取包络（向量化）——
    # 原来的写法是 `for x: np.nonzero(mask[:, x])[0]`，150 列 × 480 行的 Python
    # 循环，再在里面再套一层「往外找第一个有效像素」的 Python 循环。实测占单帧
    # 22%（2026-09-21 profile）。改成两次整图累积：
    #   `argmax` 直接给出每列的首/末个 True；
    #   「往外第一个有效像素」用两张累积表（向下最小行号 / 向上最大行号）一次算好。
    if side == 'top':
        v0_all = np.argmax(mask, axis=0).astype(np.int64)
    else:
        v0_all = (h - 1 - np.argmax(mask[::-1], axis=0)).astype(np.int64)
    keep = col_any.copy()
    n_drop = 0
    if depth is not None:
        valid = depth > 0
        rows = np.arange(h, dtype=np.int64)[:, None]
        nxt = np.minimum.accumulate(np.where(valid, rows, h)[::-1], axis=0)[::-1]
        prv = np.maximum.accumulate(np.where(valid, rows, -1), axis=0)
        v0c = v0_all[cols]
        z0 = depth[v0c, cols].astype(np.float64)
        if side == 'bottom':
            rr = nxt[np.minimum(v0c + 1, h - 1), cols]
            has = (v0c + 1 < h) & (rr < h)
        else:
            rr = prv[np.maximum(v0c - 1, 0), cols]
            has = (v0c - 1 >= 0) & (rr >= 0)
        zo = np.where(has, depth[np.clip(rr, 0, h - 1), cols].astype(np.float64),
                      np.inf)
        # 外侧更近 = 遮挡，不是真边；找不到有效深度（has=False）就留
        drop = has & (z0 > 0.0) & (zo < z0)
        keep[cols[drop]] = False
        n_drop = int(drop.sum())
    sel = np.nonzero(keep)[0]
    n = len(sel)
    if n < max(BOUNDARY_MIN_COLS_ABS, int(BOUNDARY_MIN_COLS_FRAC * len(cols))):
        return None
    xs = sel.astype(np.float64)
    ys = v0_all[sel].astype(np.float64)

    # 内点阈值 = 包络自己的阶梯步长（相邻列 y 的非零跳变的中位数），至少 1px
    dy = np.abs(np.diff(ys))
    nz = dy[dy > 0.0]
    tol_px = max(1.0, float(np.median(nz)) if len(nz) else 1.0)
    min_cols = max(BOUNDARY_MIN_COLS_ABS, int(BOUNDARY_MIN_COLS_FRAC * n))

    # RANSAC：**一次采完 all 组假设、一次性评估**，替掉原来 2000 次小数组的
    # Python 循环（每次 numpy 调用约 5µs 开销，2000 次 ≈ 10ms × 每帧两次）。
    # 采样点本来就是随机的，批量与逐个在统计上等价。
    rng = np.random.RandomState(7)           # 固定种子：同输入必须同输出
    ij = rng.randint(0, n, size=(iters, 2))
    ok = ij[:, 0] != ij[:, 1]
    ij, dx = ij[ok], xs[ij[ok, 1]] - xs[ij[ok, 0]]
    good = np.abs(dx) >= 1.0
    ij, dx = ij[good], dx[good]
    a_all = (ys[ij[:, 1]] - ys[ij[:, 0]]) / dx
    good = np.abs(a_all) <= BOUNDARY_MAX_SLOPE
    ij, a_all = ij[good], a_all[good]
    if len(a_all) == 0:
        return None
    b_all = ys[ij[:, 0]] - a_all * xs[ij[:, 0]]
    k_all = (np.abs(ys[None, :] - (a_all[:, None] * xs[None, :]
                                   + b_all[:, None])) <= tol_px).sum(axis=1)
    j = int(np.argmax(k_all))
    if int(k_all[j]) < min_cols:
        return None
    best = (float(a_all[j]), float(b_all[j]))
    # 用内点最小二乘重拟合两轮（RANSAC 给的只是「哪一段」，斜率还得精修）
    for _ in range(2):
        a, b = best
        m = np.abs(ys - (a * xs + b)) <= tol_px
        if int(m.sum()) < min_cols:
            return None
        A = np.stack([xs[m], np.ones(int(m.sum()))], axis=1)
        a, b = np.linalg.lstsq(A, ys[m], rcond=None)[0]
        best = (float(a), float(b))
    a, b = best
    m = np.abs(ys - (a * xs + b)) <= tol_px
    span = float(xs[m].max() - xs[m].min()) if m.any() else 0.0
    n_in = int(m.sum())
    # `b` 一起返回：调用方（`_boundary_axis`）要把这条**图像直线**反投影到平面上
    # 取方向，只有斜率不够 —— 截距决定了它落在哪条线上。
    return {'ang': _wrap_half_turn(float(np.arctan(a))), 'b': float(b),
            'n_inlier': n_in, 'n_cols': n, 'n_dropped': n_drop,
            'span_px': span, 'step_px': tol_px,
            'score': float(n_in) * span}


def _boundary_axis(mask: np.ndarray, depth: np.ndarray, k: CameraIntrinsics,
                   plane: dict, box_uv, pts2: np.ndarray
                   ) -> tuple[np.ndarray | None, dict]:
    """箱顶 **mask 的上/下边界**给出的矩形短轴方向（平面 2D 单位向量）。

    这是 §24 的核心：候选矩形的朝向不再取自 `extract_line_segments` 的
    「u± 里 span 更大的那条」。

    **为什么必须换掉**：`span_mm` 量的是「这条边**露出来**多长」，不是「这条边
    有多长」—— 手一挡就变。9_test 的 u± 外缘带混着手，`u_best` 的方向偏离物理
    近边 **−11.05°**（6_test 只偏 0.3~1.0°），候选矩形整体跟着转 12~18°，
    于是长边的深度峰漂出 `_edge_line_support` 的 1.5px 容差、`support` 从 0.77
    掉到 0.17、族 x 判为无效 —— 只观测到族 y，`t_e1` 被 held，theta 又只能绕
    光轴转、补不回矩形的宽边方向，最后输出四角全歪。见 WORKLOG §24。

    **为什么用 mask 边界**：它是深度分割的直接产物，跨整个可见宽度、不依赖
    `_snap_edge` 在哪儿恰好采到峰，而且 `_envelope_line` 已经带了物理门
    （外侧更远才是真边）+ RANSAC，天然剔掉隔壁箱子和夹爪。

    **上边界和下边界一起用**（操作员 2026-09-22 要求）：真实顶面有约 15° 俯仰，
    投影出来上下边接近平行但**不严格平行**（实测差 0.36~6.64°），两条一起按
    `score`（内点数 × 跨度）加权融合，比只用一条稳 —— 只用下边界时 8_test
    有一帧（976874）整条边失效。

    **长边还是短边**（操作员 2026-09-22 要求）：箱子竖放时上下边界是**短边**对
    （e1 沿边界），横放时是**长边**对（e1 垂直边界）。判据是纯比较、无阈值 ——
    看点集沿边界方向的跨度大还是垂直方向的跨度大，谁大谁是长边。四个序列实测
    比值 0.68~1.57，分得干净。

    返回 (e1_2d, diag)；边界算不出来时 e1 为 None（调用方退回原路）。
    """
    diag: dict = {}
    if mask is None or depth is None or box_uv is None:
        diag['fail'] = 'no_inputs'
        return None, diag
    pad = inbox_pad_px(k, float(plane['origin'][2]))
    mc = _clip_to_box(mask, box_uv, pad)
    cols = np.nonzero(mc.any(axis=0))[0]
    if len(cols) < BOUNDARY_MIN_COLS_ABS * 2:
        diag['fail'] = 'too_few_cols'
        return None, diag
    u_mid = float(np.median(cols))
    got: dict = {}
    for side in ('bottom', 'top'):
        ln = _envelope_line(mc, side, depth=depth)
        if ln is None:
            continue
        d = _line_to_plane_dir(ln['ang'], ln['b'], u_mid, k, plane)
        if d is None:
            continue
        got[side] = {'dir': d, 'ang': float(ln['ang']), 'score': float(ln['score']),
                     'n_inlier': int(ln['n_inlier']), 'span_px': float(ln['span_px'])}
    diag['boundaries'] = {s: {'ang_deg': round(float(np.degrees(v['ang'])), 3),
                              'n_inlier': v['n_inlier']}
                          for s, v in got.items()}
    if not got:
        diag['fail'] = 'no_boundary'
        return None, diag

    # 方向按 2θ 圆周平均融合 —— 直线方向有 180° 歧义，直接线性平均会在 ±90°
    # 附近翻转。权重用 `score`（内点数 × 跨度），即「这条边界有多可信」。
    src = [got[s] for s in ('bottom', 'top') if s in got]
    ws = [max(s['score'], 1e-9) for s in src]
    angs = [np.arctan2(s['dir'][1], s['dir'][0]) for s in src]
    ss = float(sum(w * np.sin(2 * a) for w, a in zip(ws, angs)))
    cc = float(sum(w * np.cos(2 * a) for w, a in zip(ws, angs)))
    ang = 0.5 * float(np.arctan2(ss, cc))
    d_b = np.array([np.cos(ang), np.sin(ang)])
    if len(src) == 2:
        diag['parallel_deg'] = round(float(np.degrees(
            _wrap_half_turn(angs[0] - angs[1]))), 3)

    # 长边 / 短边：纯比较，无阈值。
    eu = float(np.ptp(pts2 @ d_b))
    ev = float(np.ptp(pts2 @ np.array([-d_b[1], d_b[0]])))
    diag['extent_along'] = round(eu, 1)
    diag['extent_perp'] = round(ev, 1)
    diag['boundary_is'] = 'long' if eu > ev else 'short'
    e1 = np.array([-d_b[1], d_b[0]]) if eu > ev else d_b
    return e1 / np.linalg.norm(e1), diag


def _line_to_plane_dir(ang: float, b: float, u_ref: float,
                       k: CameraIntrinsics, plane: dict) -> np.ndarray | None:
    """图像里的一条直线（v = tan(ang)·u + b）-> 它所在**平面**上的 2D 单位方向。

    在 `u_ref` 两侧各取一点、反投影到平面，两点之差就是方向。取多远不影响方向
    （平面是平的），±`_LINE_DIR_HALF_PX` 只是取个够长的基线。
    """
    a = float(np.tan(ang))
    nn = np.asarray(plane['nrm'], np.float64)
    oo = np.asarray(plane['origin'], np.float64)
    P = []
    for du in (-_LINE_DIR_HALF_PX, _LINE_DIR_HALF_PX):
        u = u_ref + du
        v = a * u + b
        d = np.array([(u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0])
        den = float(d @ nn)
        if abs(den) < 1e-12:
            return None
        t = float(oo @ nn) / den
        if t <= 1.0:                       # 交点在相机背后
            return None
        P.append(d * t)
    p2 = project_to_plane(np.asarray(P), nn, oo)
    dd = p2[1] - p2[0]
    n = float(np.linalg.norm(dd))
    return dd / n if n > 1e-9 else None


def _perspective_dp(frame0: dict, k: CameraIntrinsics, real_plane: dict) -> float:
    """候选矩形的「**近边 − 远边**」夹角（rad）—— 由几何算出，**不依赖真值**。

    为什么需要它：算法按 `flat_plane` 建模（法向 = 光轴），投影是相似变换，
    所以候选矩形的上下边**严格平行**（Δp 恒为 0）；而真实顶面有约 15° 俯仰，
    投影出来上下边差一个透视量。要把「上边界（远边）的测量」折算成「下边界
    （**近边**，也就是真值口径）」的，就得补上这个量。

    做法：把候选四角沿**真实平面**的法向抬到真实平面上再投影，量它自己近边
    与远边的夹角。远边/近边按图像 v 分（v 大的那两个角构成近边）。

    实测 6_test：本函数给 **+0.63°**，四角真值给 +0.62° —— 零真值输入。
    5_test 的远边恰好落在主点那一行附近（v≈195 vs cy=245.7），Δp 只有 0.04°。
    """
    cm = _corners_mm(frame0, float(frame0['W']), float(frame0['H']))
    n = np.asarray(real_plane['nrm'], np.float64)
    nrm_n = float(np.linalg.norm(n))
    if nrm_n < 1e-12:
        return 0.0
    n = n / nrm_n
    o = np.asarray(real_plane['origin'], np.float64)
    lift = cm - (((cm - o) @ n)[:, None]) * n[None, :]
    uv = _project_px(lift, k)

    def _ang(idx) -> float:
        d = uv[idx[1]] - uv[idx[0]]
        return float(np.arctan2(d[1], d[0]))

    order = np.argsort(uv[:, 1])
    far, near = order[:2], order[2:]
    far = far[np.argsort(uv[far, 0])]
    near = near[np.argsort(uv[near, 0])]
    return _wrap_half_turn(_ang(near) - _ang(far))


def _lock_theta(frame0: dict, k: CameraIntrinsics, segs2d: list, plane: dict,
                observations: list, mask: np.ndarray | None = None,
                info: dict | None = None, box_uv=None,
                real_plane: dict | None = None,
                depth: np.ndarray | None = None) -> tuple[float | None, int]:
    """用 mask 的**上下边界**定 theta 增量（不让 least_squares 自由解）。

    为什么不让自由解：四条边里左右边带着俯仰透视的汇聚，会把 theta 一会儿往上
    拽一会儿往下拽，同一静止场景五帧在 −3.59°~+2.10° 之间抖。

    **关于「哪条边跟哪条边平行」的一次纠正（2026-09-20，操作员指出）**：
    ⚠️ 本函数早期注释写过「顶面倾斜时上下边在图像里是梯形的两腰、**不平行**」——
    **这是错的**。用真值量（6_test，可见四角）：
        上边 +0.91° ∼ 下边 +1.53° → 差 **0.62°**（近乎平行）
        左边 −83.44° ∼ 右边 +87.18° → 差 **9.38°**（左右才是两腰）
    物理上也该如此：俯仰是绕**图像水平轴**转的，平行于该轴的边（上下边）投影后
    仍然平行；垂直方向的那两条（左右边）才有 Z 分量、才会汇聚。
    当年那个「差 3.14° / 1.13°」的测量是拿**隔壁箱子的点**和本箱子比出来的，
    不是上下边之差。**结论：上下边可以融合，见 ①。**

    **边线来源的沿革**（都踩过坑）：
      * 一开始用「mask 上下边界带 + 按 y 分位取带」—— 箱子一转角就崩：矩形转过
        15° 后「图像最上/最下 10% 高度」里装的是一个角附近的小三角，采样短还混进
        邻边，实测漂 7~8°。
      * 中间试过改用 `extract_line_segments` 的边线 —— 它是「四侧外缘带」拟合的，
        会被外缘带形状带偏，跟观测方向能差到 10°。
      * 现在：**逐列取包络 + RANSAC**（`_envelope_line`），与旋转无关、且天然剔掉
        隔壁箱子与夹爪。

    操作员给的场景先验（2026-09-20）：**机器人手只挡下边和左右的下半部分，
    上边线总是完整的**。实测确实如此，且反过来「下边被夹爪切掉一截」很常见 ——
    所以两条边都得能用，只能按每次测量的质量给权重（见 ①）。
    """
    if not segs2d or plane is None:
        if info is not None:
            info['fail'] = 'no_segs_or_plane'
        return None, 0

    # 找 frame0 里「图像上最靠上」的那条边，用**它自己的观测点**拟合方向。
    # ⚠️ 不要用 `extract_line_segments` 的边线：它是「外缘带」拟合出来的，
    # 实测（6_test 帧1）边线方向 +3.51° 而观测点拟合 +3.51°、最终输出 +9.49°，
    # 边线跟观测方向能差到 10° —— 因为它被外缘带的形状带偏了。观测点是
    # `_snap_edge` 在深度梯度上实测的，更可信。
    # 也不能按 span 最长选边：span 最大的是「沿长边」那条，长边是横是竖取决于
    # 箱子摆放（5_test 竖放时它是竖直边，拿它定方向偏 4.5°）。操作员给的先验是
    # 「手只挡下边和左右下半，**上边总是完整的**」，所以按图像位置选最稳。
    segs_px = _edges_px(frame0, float(frame0['W']), float(frame0['H']), k)
    ti, tv = None, 1e9
    for i, s_ in enumerate(segs_px):
        if not s_['ok']:
            continue
        v_ = 0.5 * (float(s_['p0'][1]) + float(s_['p1'][1]))
        if v_ < tv:
            tv, ti = v_, i
    if ti is None:
        if info is not None:
            info['fail'] = 'no_projectable_edge'
        return None, 0
    seg = segs_px[ti]
    e = np.asarray(seg['p1'], np.float64) - np.asarray(seg['p0'], np.float64)
    ang_seg = float(np.arctan2(e[1], e[0]))

    # ⚠️ **手性**：`_move` 绕 nrm 转 theta，但转出来在图像上是顺时针还是逆时针取决于
    # (E1,E2) 是不是右手系 —— 而 `initial_frame_candidates` 里的 e1 是用 `_inward_dir`
    # 按**角点**定出来的，换个候选角（`u-/v+` vs `u-/v-`）就会翻个个儿。
    # 实测 6_test 帧1（`u-/v+`）与其余四帧（`u-/v-`）手性相反，于是同一套 theta 在
    # 帧1 转反了 3.6°，而输出四角照样是规矩的矩形 —— 不比对真值根本看不出来。
    # `_EDGE_SPEC` 里第 0 条是 +E1（y=0 边）、第 1 条是 +E2（x=W 边），叉积定号。
    hand = 1.0
    if (segs_px[0]['ok'] and segs_px[1]['ok']):
        _a = np.asarray(segs_px[0]['p1'], np.float64) - np.asarray(segs_px[0]['p0'], np.float64)
        _b = np.asarray(segs_px[1]['p1'], np.float64) - np.asarray(segs_px[1]['p0'], np.float64)
        hand = 1.0 if (_a[0] * _b[1] - _a[1] * _b[0]) > 0.0 else -1.0

    # ① 首选：mask 的**下边界**（顶面近边）。
    #    它跨整个箱宽，比观测点长十倍，不依赖 `_snap_edge` 在哪儿恰好采到峰，
    #    而且经过物理门之后**只剩真边**（外侧更远的那些列）。
    #
    #    为什么是「下边界为主」而不是「上下加权融合」——
    #    口径上，输出要的是**近边**方向：
    #      * 下边界就是近边，**直接可量**；
    #      * 上边界是**远**边，必须靠 Δp 折算到近边，折算误差会**原样变成方向误差**。
    #    操作员 2026-09-21 裁定「以深度边界（=物理近边）为准」之后，这一点是决定性的：
    #    实测把上边界的权重从 1.0 降到 0 时，两个序列的输出角度**分别精确落在各自
    #    的物理深度边界上**（5_test −1.28°、6_test +1.02°），6_test 的四角总误差
    #    还从 7.6px 降到 7.4px。
    #    所以上边界只作**回退**：下边界被挡光（物理门之后列数不够）时才用它。
    if mask is not None:
        mc = (_clip_to_box(mask, box_uv, inbox_pad_px(k, float(frame0['origin'][2])))
              if box_uv is not None else mask)
        ln_b = _envelope_line(mc, 'bottom', depth=depth)
        if ln_b is not None:
            if info is not None:
                info.update(source='mask_bottom', n_points=ln_b['n_inlier'],
                            ref_edge=seg['name'], hand=hand,
                            ref_edge_deg=round(float(np.degrees(ang_seg)), 3),
                            bottom_deg=round(float(np.degrees(ln_b['ang'])), 3),
                            bottom_cols=ln_b['n_cols'],
                            bottom_dropped=ln_b['n_dropped'],
                            step_px=round(float(ln_b['step_px']), 2))
            return (_wrap_half_turn(hand * (ln_b['ang'] - ang_seg)),
                    ln_b['n_inlier'])
        # ①' 回退：上边界（**远**边）+ 透视量 Δp 折算到近边口径。
        #     Δp 由 `_perspective_dp` 按真实平面几何算出（6_test 实测 0.61° vs 四角
        #     真值 0.62°）。它自己**依赖 θ**（实测 dΔp/dθ ≈ 0.125），所以迭代两轮
        #     到自洽 —— 只算一轮的话 6_test 会把 Δp 算成 0.81°。
        ln_t = _envelope_line(mc, 'top', depth=depth)
        if ln_t is not None and real_plane is not None:
            dth, dp = 0.0, 0.0
            for _ in range(2):
                dp = _perspective_dp(_move(frame0, 0.0, 0.0, dth), k, real_plane)
                dth = _wrap_half_turn(hand * (_wrap_half_turn(ln_t['ang'] + dp)
                                              - ang_seg))
            if info is not None:
                info.update(source='mask_top_plus_dp', n_points=ln_t['n_inlier'],
                            ref_edge=seg['name'], hand=hand,
                            ref_edge_deg=round(float(np.degrees(ang_seg)), 3),
                            top_deg=round(float(np.degrees(ln_t['ang'])), 3),
                            dp_deg=round(float(np.degrees(dp)), 3),
                            top_cols=ln_t['n_cols'],
                            step_px=round(float(ln_t['step_px']), 2))
            return dth, ln_t['n_inlier']

    # ② 回退：这条边自己的观测点
    pts = np.asarray([uv for i, uv in observations if i == ti], np.float64)
    if len(pts) < 6:
        if info is not None:
            info['fail'] = 'too_few_observations'
            info['n_obs_raw'] = len(pts)
        return None, 0
    # ⚠️ **不能直接 SVD 全部观测点**：`_snap_edge` 找的是深度不连续峰，峰可能落
    # 在手臂或别的物体的边缘上，这些外点会把方向整个拽走。实测 5_test 同一个
    # 静止场景，那条完整的上边只取到 6~13 个观测点，直接 SVD 算出的 theta 在
    # −2.8°~+2.9° 之间抖了 5.7° —— 把五帧的定位带偏了十几 px。
    # 迭代剔点：每轮按到拟合直线的垂距剔掉 > max(下限, 3σ) 的点再重拟合。
    keep = np.ones(len(pts), bool)
    for _ in range(4):
        q = pts[keep]
        if len(q) < 6:
            break
        c = q.mean(axis=0)
        _, _, vt = np.linalg.svd(q - c, full_matrices=False)
        dv = vt[0] / np.linalg.norm(vt[0])
        r = np.abs((pts - c) @ np.array([-dv[1], dv[0]]))
        med = float(np.median(r[keep]))
        thr = max(THETA_FIT_TOL_PX, 3.0 * 1.4826 * med)
        new = r <= thr
        if int(new.sum()) < 6 or bool((new == keep).all()):
            break
        keep = new
    pts = pts[keep]
    if len(pts) < 6:
        return None, 0
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    dd = vt[0] / np.linalg.norm(vt[0])
    if abs(float(dd[0])) < 1e-9:
        return None, 0
    ang_obs = float(np.arctan2(dd[1], dd[0]))
    if info is not None:
        info.update(source='observations', n_points=len(pts),
                    ref_edge=seg['name'], hand=hand,
                    ref_edge_deg=round(float(np.degrees(ang_seg)), 3),
                    top_boundary_deg=round(float(np.degrees(ang_obs)), 3))
    return _wrap_half_turn(hand * (ang_obs - ang_seg)), len(pts)


def solve_frame(frame0: dict, observations: list, k: CameraIntrinsics,
                opts: dict | None = None) -> tuple[dict | None, dict]:
    """按分级放开自由度，用最小二乘精化。矩形性是参数化的硬约束。

    只更新被观测到的自由度，观测不到的保持初值 —— 不编数，但也不直接拒绝。
    `updated` + `held` 必须正好是 {t_e1, t_e2, theta} 的一个划分。
    """
    from scipy.optimize import least_squares

    opts = opts or {}
    W, H = float(frame0['W']), float(frame0['H'])
    fams = frozenset(frame0.get('_families', set()))
    diag: dict = {'n_observations': len(observations)}
    if not observations or not fams:
        diag['reject'] = 'no_edges'
        return None, diag

    free_idx = FREE_FOR_FAMILIES.get(fams)
    if free_idx is None:
        diag['reject'] = 'unsupported_family_combination'
        diag['families'] = sorted(fams)
        return None, diag

    p_full = [0.0, 0.0, 0.0]          # (tx, ty, theta) 相对 frame0 的增量
    x_scale = [50.0, 50.0, 0.05]
    theta_src = observations
    theta_locked = False
    # theta 用 mask 的上下边界**锁定**，不交给 least_squares 自由解 ——
    # 左右边带着俯仰透视的汇聚（实测左 −2.0° / 右 +2.7°），自由解会让角度在
    # ±3° 之间抖。这是既定前提，不是开关。
    #
    # ⚠️ **不要门控在 `2 in free_idx` 上**：theta 的信息源是 mask 的上下边界，
    # 它跟「哪几条边有观测」是两回事。实测 6_test 旋转 10° 时左右边全被手臂挡住、
    # `families={'y'}`、`free_idx=(1,)`，theta 因此没被锁、退化成候选矩形的初值
    # 朝向，输出角度偏了 4.79°。theta 该锁就锁，与 families 无关。
    tinfo: dict = {}
    theta_lock, n_th = _lock_theta(frame0, k, opts.get('_segs2d'),
                                   opts.get('_plane'), theta_src,
                                   opts.get('_mask'), tinfo,
                                   opts.get('_box_uv'),
                                   opts.get('_real_plane'),
                                   opts.get('_depth'))
    if tinfo:
        diag['theta_info'] = tinfo
    if theta_lock is not None:
        p_full[2] = float(theta_lock)
        free_idx = tuple(i for i in free_idx if i != 2)
        theta_locked = True
        diag['theta_deg'] = round(float(np.degrees(theta_lock)), 3)
        diag['theta_n_points'] = n_th

    def _solve(obs):
        return least_squares(
            _residuals, [p_full[i] for i in free_idx],
            args=(free_idx, p_full, frame0, W, H, k, obs),
            loss='huber', f_scale=2.0, x_scale=[x_scale[i] for i in free_idx])

    obs = list(observations)
    res = _solve(obs)

    # ⚠️ 试过「逐边离群检测」：一条边整条平移时，它每个点到预测边的**带符号**
    # 残差同号且都大（6_test 实测 y=H 整体偏 16.7px），而最小二乘只看无符号垂距、
    # 对这种整体平移不敏感。但**测出来也只能记录，不能剔** —— 试过「整条剔掉重解」：
    # 6_test 从 27.5 恶化到 38.6、5_test 左下从 6.1 退化到 14.2，因为被剔的恰好是
    # 竖直边，而「u 方向的位置」只能由竖直边约束，剔掉就漂了。
    # 既然不剔，就没有下游消费它 —— 整段已删，留这段结论免得再试一遍。

    for _ in range(2):                 # 逐观测点离群剔除后重解
        p = list(p_full)
        for slot, idx in enumerate(free_idx):
            p[idx] = float(res.x[slot])
        segs = _edges_px(_move(frame0, p[0], p[1], p[2]), W, H, k)
        keep = []
        for edge_i, uv in obs:
            seg = segs[edge_i]
            if not seg['ok']:
                continue
            A = np.asarray(seg['p0'], np.float64)
            B = np.asarray(seg['p1'], np.float64)
            d = B - A
            L = float(np.hypot(*d))
            if L < 1e-6:
                continue
            nvec = np.array([-d[1], d[0]]) / L
            if abs(float((np.asarray(uv, np.float64) - A) @ nvec)) <= OBS_MAX_RESIDUAL_PX:
                keep.append((edge_i, uv))
        if len(keep) == len(obs) or len(keep) < MIN_EDGE_KEEP:
            break
        obs = keep
        res = _solve(obs)

    p = list(p_full)
    for slot, idx in enumerate(free_idx):
        p[idx] = float(res.x[slot])
    frame = _move(frame0, p[0], p[1], p[2])
    frame['long_along_e1'] = frame0.get('long_along_e1')
    frame['W'], frame['H'] = W, H

    slot_of = {0: 't_e1', 1: 't_e2', 2: 'theta'}
    updated = [slot_of[i] for i in free_idx]
    if theta_locked:
        # 锁定的 theta **仍然算 updated**：它来自观测（mask 的上下边界），
        # 只是没交给 least_squares 去自由解。只有 `_lock_theta` 返回 None
        # （边界量不出来）时，它才真的停在初值、算 held。
        updated.append('theta')
    held = [slot_of[i] for i in (0, 1, 2) if slot_of[i] not in updated]
    diag.update({'updated': updated, 'held': held, 'n_observations': len(obs),
                 'families': sorted(fams),
                 'confidence': 'normal' if len(updated) == 3 else 'degraded'})
    return frame, diag


# --------------------------------------------------------------------------- #
# 四角排序与主入口
# --------------------------------------------------------------------------- #
def order_corners_uv(corners_uv) -> np.ndarray:
    """四角排序为：右下 → 左下 → 左上 → 右上。

    起点取 u+v 最大的角（图像上最靠右下），随后按屏幕顺时针。与输入顺序无关。
    """
    p = np.asarray(corners_uv, np.float64).reshape(-1, 2)
    if len(p) != 4:
        raise ValueError(f'要 4 个角，收到 {len(p)} 个')
    start = int(np.argmax(p[:, 0] + p[:, 1]))
    c = p.mean(axis=0)
    a0 = np.arctan2(p[start][1] - c[1], p[start][0] - c[0])
    rest = [i for i in range(4) if i != start]
    rest.sort(key=lambda i: (np.arctan2(p[i][1] - c[1], p[i][0] - c[0]) - a0)
              % (2 * np.pi))
    return p[[start] + rest]


def fit_box_frame(color: np.ndarray, depth: np.ndarray, k: CameraIntrinsics,
                  box_uv, *, target_mm: tuple[float, float] = DEFAULT_TARGET_MM,
                  opts: dict | None = None) -> tuple[dict | None, dict]:
    """主入口：零交互，纯函数。这就是最终要被 YOLO 调用的那个。

    返回 (result, diag)。失败时 result 为 None，diag['reject'] 写明原因。
    """
    opts = opts or {}
    diag: dict = {'ok': False, 'reject': None,
                  'confidence': 'normal', 'target_mm': tuple(target_mm)}
    roi = roi_from_box(box_uv, depth.shape)
    diag['roi'] = list(roi)

    plane, mask, pdiag = segment_top_plane(depth, k, roi, box_uv, opts, target_mm)
    diag['plane'] = pdiag
    # 留下**真实拟合到的**平面（下一段可能被 flat_plane 换成「法向=光轴」）。
    # theta 要用它算上下边的透视夹角 Δp —— 那是真实俯仰造成的，换掉就没了。
    real_plane = plane
    if plane is None:
        diag['reject'] = pdiag.get('reject', 'no_plane')
        return None, diag

    # ⚠️ **mask 必须裁到「本箱子的框 + 一点余量」内，这一步在 2026-09-21 才补上。**
    # `segment_top_plane` 给的 mask 是「**全图**里到该平面距离 ≤30mm 的像素」，
    # 而平面分割只在 ROI 里做过 —— 于是 ROI 之外任何与箱顶共面/近平行的面都会进来。
    # 实测 8_test：选中平面的 bbox 一直伸到 v=136（比框高 68px），mask 全图 106435 点、
    # **只有 28% 在框内**，框外那 7.7 万点深度 694~998mm。后果不是 theta，是**位置**：
    # `flat_plane` 拿「mask 点云质心」当平面原点，质心被拉到 849mm（箱面才 700~820），
    # 矩形整体放到 90mm 以外 → 投影缩小 760/849 = 0.895（实测 238/267 = 0.891）
    # → **近边抬高 38px**。5_test 之所以没暴露：它的 mask 本来 100% 就在框内。
    # 下游每一处（质心、深度带、theta 的边界）本来就都假定 mask 是「这只箱子的顶面」，
    # 这里把它变成真的。
    z_ref = float(real_plane['origin'][2])
    mask = _clip_to_box(mask, box_uv, inbox_pad_px(k, z_ref))

    vs, us = np.nonzero(mask)
    P3 = _backproject(us.astype(np.float64), vs.astype(np.float64),
                      depth[vs, us].astype(np.float64), k)

    # 操作员 2026-09-20：**忽略俯仰角、不考虑梯形关系** —— 后续几何一律把顶面
    # 当作正对相机的平面。mask 仍然用上面真实拟合出来的平面算（否则点都找不
    # 对：实测顶面相对光轴偏 15.35°），但从这里开始换成「法向 = 光轴」的平面，
    # 于是四角落在同一个深度上、投影出来是**矩形**而不是梯形。
    c = P3.mean(axis=0)
    plane = {'nrm': np.array([0.0, 0.0, -1.0]),
             'origin': np.array([c[0], c[1], c[2]]),
             'points': plane['points'], 'n_points': plane['n_points']}
    # 摆正之后高度判据失效，改用「顶面点的深度带」过滤观测（2%~98% 分位 + 余量）
    z_lo, z_hi = np.percentile(P3[:, 2], [2.0, 98.0])
    opts = dict(opts)
    opts['z_range'] = (float(z_lo) - PLANE_Z_RANGE_PAD_MM,
                       float(z_hi) + PLANE_Z_RANGE_PAD_MM)
    diag['flat_origin'] = [round(float(t), 1) for t in plane['origin']]
    diag['z_range'] = [round(opts['z_range'][0], 1), round(opts['z_range'][1], 1)]
    # 只留框内（+ 一点余量）的点：框外的高处点更可能是别的东西。
    # ⚠️ 余量要**小**：6_test 里右边挨着另一个已摆好的箱子，取框宽的 20%（55px）
    # 正好把它捞进来 —— 两个箱子顶面共面，mask 分不开，候选的 b 跨度被测成
    # 686mm（≈530+156），四条边全对不上、五帧全 reject。
    # 但余量也**不能按框的像素比例取**：框会随箱子旋转涨大，余量跟着涨就会
    # 多吞邻箱的点（旋转 10° 时四个候选全被撑爆）。按物理尺寸算是与旋转无关的。
    px = _project_px(P3, k)
    pad = inbox_pad_px(k, float(plane['origin'][2]))
    bu0, bv0 = min(box_uv[0], box_uv[2]), min(box_uv[1], box_uv[3])
    bu1, bv1 = max(box_uv[0], box_uv[2]), max(box_uv[1], box_uv[3])
    inbox = ((px[:, 0] >= bu0 - pad) & (px[:, 0] <= bu1 + pad)
             & (px[:, 1] >= bv0 - pad) & (px[:, 1] <= bv1 + pad))
    P3 = P3[inbox]
    diag['n_top_face_points'] = int(len(P3))
    if len(P3) < MIN_PLANE_POINTS:
        diag['reject'] = 'top_face_too_few_points'
        return None, diag

    pts2 = project_to_plane(P3, plane['nrm'], plane['origin'])
    segs = extract_line_segments(pts2)
    diag['segments'] = [{'side': s['side'], 'span_mm': round(s['span_mm'], 1),
                         'n_inlier': s['n_inlier']} for s in segs]
    # 候选矩形的**朝向**由 mask 上/下边界给出，不再由 `u±` 里 span 最大的那条边
    # 决定。`span_mm` 是「露出来多长」不是「这条边有多长」，被手一挡就变 ——
    # 9_test 上它偏了 −11°，候选整体转 12~18°，长边因此扫不到深度峰、族 x 判无效，
    # t_e1 被 held 就再也纠不回来。详见 `_boundary_axis` 与 WORKLOG §24。
    # 边界算不出来时 `e1_hint` 为 None，退回原来的取法（不编数）。
    e1_hint = None
    if mask is not None and depth is not None:
        e1_hint, bdiag = _boundary_axis(mask, depth, k, plane, box_uv, pts2=pts2)
        diag['boundary_axis'] = bdiag
    cands, cdiag = initial_frame_candidates(segs, pts2, target_mm,
                                            e1_hint=e1_hint)
    # 操作员/上游服务给的朝向先验：把「长边是横还是竖」说死，掐掉 90° 转向。
    # 放在这里而不是 `initial_frame_candidates` 里面：那边只有平面 2D 坐标，
    # 判「图像上是横是竖」必须投影，得同时有平面和相机内参。
    hint = normalize_orientation((opts or {}).get('orientation'))
    cands, odiag = filter_by_orientation(cands or [], plane, k, hint)
    cdiag = dict(cdiag, orientation=odiag)
    if not cands:
        cands = None
    diag['orientation_hint'] = odiag
    diag['initial'] = cdiag
    if cands is None:
        # 提示和数据打架 -> **直接判失败**（操作员 2026-09-22：宁可没有结果走
        # YOLO 框 fallback，也不能接受一个先验说不通的朝向）。
        # reject 先看朝向那一关：`orientation_conflict` 比笼统的
        # `no_initial_frame` 更准确地说明「为什么这一帧没了」。
        diag['reject'] = (odiag.get('reject') or cdiag.get('reject')
                          or 'no_initial_frame')
        return None, diag

    gx, gy = depth_discontinuity(depth)
    frame0, chdiag = choose_candidate(cands, plane, gx, gy, depth, k, opts)
    diag['candidate_choice'] = chdiag
    if frame0 is None:
        diag['reject'] = chdiag.get('reject', 'no_candidate')
        return None, diag

    # theta 用**可见 mask 的上下边界**锁定（加权融合），而不是 `_snap_edge` 的
    # 观测点：实测同一静止场景五帧，边界方向只波动 0.09~0.51°，而观测点算出来的
    # 角度抖 4.5°（观测点会被手的边缘带偏，且只有 6~13 个、基线 48~128px）。
    # 详见 `_lock_theta` 的 docstring（含「上下边近乎平行、左右才是两腰」的纠正）。
    opts = dict(opts)
    opts['_segs2d'] = segs
    opts['_plane'] = plane
    opts['_mask'] = mask
    opts['_box_uv'] = box_uv
    opts['_real_plane'] = real_plane
    opts['_depth'] = depth

    frame, sdiag = solve_frame(frame0, frame0['_obs'], k, opts)
    diag['solve'] = sdiag
    if frame is None:
        diag['reject'] = sdiag.get('reject', 'solve_failed')
        return None, diag

    updated, held = set(sdiag['updated']), set(sdiag['held'])
    if updated | held != {'t_e1', 't_e2', 'theta'} or (updated & held):
        raise AssertionError(f'updated/held 不是三自由度的划分：{updated} {held}')

    corners_mm = _corners_mm(frame, float(frame['W']), float(frame['H']))
    if np.any(corners_mm[:, 2] <= 1.0):
        diag['reject'] = 'unprojectable'
        return None, diag

    quad_uv = order_corners_uv(_project_px(corners_mm, k))

    # 最后一道闸：**输出**的朝向也必须和先验对得上（操作员 2026-09-22 要求）。
    # 前面 `filter_by_orientation` 筛的是**候选**；求解会动 (t_e1, t_e2, theta)，
    # 所以这里必须拿最终 frame 再对一次 —— 否则「筛的时候对、解完不对」会溜过去。
    # 对不上就 reject，上层（`window.BoxFrameWindow`）自然退回 YOLO 框。
    if hint is not None:
        got = frame_long_axis(frame, k)
        odiag['final'] = got
        if got != hint:
            odiag['conflict'] = True
            diag['reject'] = 'orientation_conflict'
            return None, diag

    diag.update({'ok': True, 'updated': sdiag['updated'], 'held': sdiag['held'],
                 'confidence': sdiag['confidence']})
    if chdiag.get('degraded'):
        diag['confidence'] = 'degraded'
        diag['degraded_reason'] = chdiag['degraded']
    return {'corners_uv': quad_uv, 'corners_mm': corners_mm, 'frame': frame,
            'plane': {'nrm': plane['nrm'], 'origin': plane['origin']}}, diag


# --------------------------------------------------------------------------- #
# 渲染与拼图
# --------------------------------------------------------------------------- #
def montage(tiles: list[np.ndarray], cols: int = 4, gap: int = 6) -> np.ndarray:
    """把多张图拼成网格（18 号灰底）。

    语义同 `calibrate_pallet_tag.montage`，但**不 import 那个模块** —— 它会连带
    加载 pupil_apriltags，跟交互壳的 PIL 同进程会 abort（见文件头的警告）。
    """
    h = max(t.shape[0] for t in tiles)
    w = max(t.shape[1] for t in tiles)
    rows = (len(tiles) + cols - 1) // cols
    out = np.full((rows * h + (rows - 1) * gap,
                   cols * w + (cols - 1) * gap, 3), 18, np.uint8)
    for i, t in enumerate(tiles):
        r, c = divmod(i, cols)
        y, x = r * (h + gap), c * (w + gap)
        out[y:y + t.shape[0], x:x + t.shape[1]] = t
    return out


# 颜色约定（BGR）：红实线 = 有观测支撑；蓝虚线 = held（按尺寸推的）
_COLOR_OBS = (0, 0, 255)
_COLOR_HELD = (255, 128, 0)
_COLOR_BOX = (0, 255, 255)


def _dashed_line(img: np.ndarray, p0, p1, color, dash_px: float = 12.0,
                 thick: int = 2) -> None:
    """虚线段。cv2 没有虚线，只能一段段画。"""
    p0 = np.asarray(p0, np.float64)
    p1 = np.asarray(p1, np.float64)
    d = p1 - p0
    L = float(np.hypot(*d))
    if L < 1e-6:
        return
    u = d / L
    t = 0.0
    while t < L:
        a = p0 + t * u
        b = p0 + min(t + dash_px, L) * u
        cv2.line(img, tuple(np.round(a).astype(int)),
                 tuple(np.round(b).astype(int)), color, thick)
        t += 2 * dash_px


def render_box_check(color: np.ndarray, result: dict | None, diag: dict,
                     k: CameraIntrinsics, box_uv, gt_uv=None) -> np.ndarray:
    """单帧渲染：原图 + 你点的框 + 四角/四边（实线=观测，虚线=held）。

    `result` 为 None（失败帧）时只画框和一行失败说明 —— 失败也要出图，
    否则拼图会缺格，操作员看不出哪一帧没做出来。
    """
    vis = color.copy()
    bu0 = int(round(min(box_uv[0], box_uv[2])))
    bv0 = int(round(min(box_uv[1], box_uv[3])))
    bu1 = int(round(max(box_uv[0], box_uv[2])))
    bv1 = int(round(max(box_uv[1], box_uv[3])))
    cv2.rectangle(vis, (bu0, bv0), (bu1, bv1), _COLOR_BOX, 1)

    if result is None:
        cv2.putText(vis, f"FAILED: {diag.get('reject', '?')}", (6, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return vis

    frame = result['frame']
    fams_with_obs = frame.get('_families', set())
    for seg in _edges_px(frame, float(frame['W']), float(frame['H']), k):
        if not seg['ok']:
            continue
        if seg['family'] in fams_with_obs:
            cv2.line(vis, tuple(np.round(seg['p0']).astype(int)),
                     tuple(np.round(seg['p1']).astype(int)), _COLOR_OBS, 2)
        else:
            _dashed_line(vis, seg['p0'], seg['p1'], _COLOR_HELD)

    # 人工真值（绿）—— 有标注时才画，方便一眼看出差在哪
    if gt_uv is not None:
        g = np.asarray(gt_uv, np.float64).reshape(-1, 2)
        if len(g) >= 3:
            cv2.polylines(vis, [np.round(g).astype(np.int32)], True, (0, 220, 0), 2)
        for p in g:
            cv2.circle(vis, (int(round(p[0])), int(round(p[1]))), 4, (0, 220, 0), -1)

    for i, (u, v) in enumerate(result['corners_uv']):
        cv2.circle(vis, (int(round(u)), int(round(v))), 5, _COLOR_OBS, -1)
        cv2.putText(vis, str(i + 1), (int(round(u)) + 8, int(round(v)) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    label = (f"updated={'+'.join(diag.get('updated', [])) or 'none'}  "
             f"held={'+'.join(diag.get('held', [])) or 'none'}  "
             f"{diag.get('confidence', '?')}")
    # 朝向先验属人工输入，用了必须看得见（项目规矩：人工/上游输入必须留痕）。
    # 三种情形必须在图上分得开：
    #   `applied`  -> 真用上了；
    #   `conflict` -> 上游说了朝向、数据里没一个候选对得上。**现在直接判失败**
    #                 （§23，宁可走 YOLO 框 fallback），所以正常走不到这里；
    #                 留这条分支是为了「万一有别的路径进来」时也能一眼看出来。
    od = diag.get('orientation_hint') or {}
    if od.get('applied'):
        label = f"*** ORIENT={od['hint']} ***  " + label
    elif od.get('conflict'):
        label = f"*** ORIENT={od['hint']} CONFLICT ***  " + label
    cv2.putText(vis, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (255, 255, 255), 2)
    return vis


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def parse_arguments() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description='从检测框拟合箱子顶面完整四角')
    ap.add_argument('--sequence', type=Path, required=True,
                    help='序列目录，如 test_data/5_test')
    ap.add_argument('--box-file', type=Path, default=None,
                    help='JSON：{"<stem>": [u1,v1,u2,v2]}，'
                         'tools/pick_box_tk.py 可以生成')
    ap.add_argument('--box', action='append', default=None,
                    help='u1,v1,u2,v2；按帧顺序重复给，与 --box-file 二选一')
    ap.add_argument('--out', type=Path, default=None,
                    help='输出目录，默认 box_out/<序列名>')
    ap.add_argument('--limit', type=int, default=None)
    ap.add_argument('--target-mm', type=float, nargs=2,
                    default=list(DEFAULT_TARGET_MM), metavar=('LONG', 'SHORT'))
    ap.add_argument('--gt', type=Path, default=None,
                    help='人工真值 JSON（{"stem": [[u,v]×4]}），画到图上作对照')
    ap.add_argument('--orientation', default='auto',
                    choices=['auto', 'horizontal', 'vertical'],
                    help='朝向先验：箱子长边在图像里是横的还是竖的。'
                         'auto（默认）= 不提示、按边支持率自动选。'
                         '用来掐掉「整框转 90°」那种错 —— 数据里看不出来，'
                         '得靠上游（/infer_carton_pose 的 tape_orientation_deg）')
    return ap.parse_args()


def main() -> None:
    import json

    args = parse_arguments()
    out_dir = (args.out if args.out is not None
               else Path('box_out') / args.sequence.name)
    out_dir.mkdir(parents=True, exist_ok=True)

    boxes: dict[str, list[float]] = {}
    if args.box_file is not None:
        raw = json.loads(args.box_file.read_text(encoding='utf-8'))
        # 下划线开头的键当作注释/元数据跳过（允许在 JSON 里写 _note）
        boxes = {k: [float(t) for t in v] for k, v in raw.items()
                 if not k.startswith('_')}

    frames = pair_frames(args.sequence, 20.0)
    if args.limit:
        frames = frames[:args.limit]
    if args.box:
        if len(args.box) != len(frames):
            raise SystemExit(f'--box 给了 {len(args.box)} 组，但有 {len(frames)} 帧')
        for (color_path, _), spec in zip(frames, args.box):
            boxes[color_path.stem] = [float(t) for t in spec.split(',')]
    missing = [c.stem for c, _ in frames if c.stem not in boxes]
    if missing:
        raise SystemExit(f'这些帧没有框：{missing}（用 --box 或 --box-file 给）')

    gt_map = ({k2: v for k2, v in
               json.loads(args.gt.read_text(encoding='utf-8')).items()
               if not k2.startswith('_')} if args.gt is not None else {})

    tiles = []
    times: list[float] = []
    for color_path, _ in frames:
        stem = color_path.stem
        color, depth, k = load_frame(args.sequence, stem)
        box_uv = tuple(boxes[stem])
        t0 = time.perf_counter()
        result, diag = fit_box_frame(color, depth, k, box_uv,
                                     target_mm=tuple(args.target_mm),
                                     opts={'orientation': args.orientation})
        ms = (time.perf_counter() - t0) * 1000.0
        times.append(ms)
        diag['stem'] = stem
        diag['box_uv'] = list(box_uv)
        (out_dir / f'{stem}_diag.json').write_text(
            json.dumps(diag, ensure_ascii=False, indent=2, default=str),
            encoding='utf-8')

        if result is None:
            print(f'{stem}: {ms:6.1f}ms  失败 reject={diag.get("reject")}')
            stale = out_dir / f'{stem}_box.json'
            if stale.exists():
                stale.unlink()      # 别让上一轮的旧结果冒充这一轮的成功
        else:
            (out_dir / f'{stem}_box.json').write_text(
                json.dumps({'corners_uv': [[round(float(u), 2), round(float(v), 2)]
                                           for u, v in result['corners_uv']],
                            'corners_mm': [[round(float(t), 2) for t in c]
                                           for c in result['corners_mm']],
                            'updated': diag.get('updated'),
                            'held': diag.get('held'),
                            'confidence': diag.get('confidence')},
                           ensure_ascii=False, indent=2), encoding='utf-8')
            print(f'{stem}: {ms:6.1f}ms  ok  updated={diag.get("updated")} '
                  f'held={diag.get("held")} conf={diag.get("confidence")}')

        tile = render_box_check(color, result, diag, k, box_uv,
                                (gt_map or {}).get(stem))
        cv2.imwrite(str(out_dir / f'{stem}_check.png'), tile)
        tiles.append(tile)

    if tiles:
        cv2.imwrite(str(out_dir / 'box_montage.png'), montage(tiles, cols=2, gap=6))
        if times:
            a = np.asarray(times)
            print(f'逐帧耗时: 中位 {np.median(a):.1f}ms  均值 {a.mean():.1f}ms  '
                  f'最大 {a.max():.1f}ms   (n={len(a)})')
            print('  ⚠️ 部署到 Orin 前先 export OMP_NUM_THREADS=1 '
                  'OPENBLAS_NUM_THREADS=1 —— 本项目的矩阵都太小，')
            print('     OpenBLAS 开多线程是纯开销，实测差 1.6x（见 WORKLOG §15.1）。')
    print(f'拼图写到 {out_dir / "box_montage.png"}')


if __name__ == '__main__':
    main()
