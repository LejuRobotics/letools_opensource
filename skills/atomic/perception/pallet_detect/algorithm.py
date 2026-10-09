# -*- coding: utf-8 -*-
"""无初值检测木托盘台面坐标系。**纯函数、零状态、不 import ROS。**

外部先验（台面法向）把地面系从 6 个自由度降到 3 个 —— 平面内平移 x2 + 绕法向
转角 x1，正好与 `pallet_servo` 的 `t_e1`/`t_e2`/`theta` 同构。所以本模块是
**给伺服造初值**：用已知尺寸的旋转矩形在木色台面掩码上做支撑率搜索。

    输入：彩色图 + 对齐后的深度图（16UC1，mm）+ 内参 + **台面法向**
    输出：`frame`（`origin`/`E1`/`E2`/`nrm`/`W`/`H`，相机系，mm）+ 诊断

分层：本文件 → ROS 壳
`infrastructure/ros_packages/src/ros_vision/pallet_detection/`。
本文件**不 import 框架任何东西**，也不 import 别的 skills 模块。

## 三条必须知道的接口约定

1. **法向是必需先验，且本模块不会细化它。** 法向就是投影平面本身，偏 ε 时真值
   台面在"高度"坐标里变成一条斜坡，跨度 ≈ `tan(ε) × 台面尺寸`；超过
   `DECK_BAND_MM` 台面就装不进那个高度带、掩码塌成月牙。**临界角随带宽变**：
   `DECK_BAND_MM=40` 时约 3~4°，**提到 85 之后约 5~7°** —— 现场实测 TF 法向偏
   6.83° 时 40mm 带过不了、85mm 带能过（见 `DECK_BAND_MM` 的注释）。
   ⚠️ **没有法向先验时它不会崩、不会报错** —— 给一个完全错误的法向（比如竖直
   向下）它只是换个 score 继续"如实拒绝"（`source` 从 color 变 depth）。所以
   调用方**必须**自己保证法向来自可信来源（TF 或操作员给定），缺了就别调。
   ⚠️ **偏得再多也不用怕**：`refine_pallet_frame` 的平面拟合会把法向修回来
   （现场实测 6.83° -> 1.48°）。detect 只要能出一帧位姿，下游就收敛得到 ——
   所以带宽要留够，别让它卡在 detect 这一步。
2. **θ 由图像空间的主路径给（2026-09-30 加），`long_side_parallel` 只对回退路径
   有效。** 原先 θ 只有 `_theta_ref`（栅格掩码的协方差主轴）一个来源，现场帧实测
   它给出 34.44°/124.44° 而托盘长边是 83.49° —— **两个候选轴都不是托盘的边**，
   `long_side_parallel` 只是在错答案里二选一，发布出来是转 90° 的位姿。
   现在改从图像空间取 θ（掩码稠密、单应反查成米制、`minAreaRect` 的长边方向），
   实测同 8 帧真值中位 6px。**回退路径仍然吃 `long_side_parallel`**，给错了照样偏
   90°（april_test7 实测偏 +67.6°、452mm），**默认 `False`（短边平行）**，
   因为实测 april_test7 / april_test6 / apriltag_test3 都是短边近平行，只有
   5_test 是长边。换场景必须显式确认。详见 README §3 第 2 条。
3. **`prior` 是可选的外部状态。** 给上一帧的结果就走先验跟踪路径（单帧快 3~7 倍）；
   给一个过时或来自别处的位姿**不会报错**，只会退化成"搜不到 -> 回退全搜索"。

## 与 `refine_pallet_frame` 的关系

`detect` 单独对法向敏感（5.6° -> 180mm），但下游 `refine_pallet_frame` 的
`_refine_plane` 是 SVD 拟合台面平面，法向 2 + 高度 1 自己解，能把终点拉回
13.2mm / dθ 0.1° —— **前提是显式传 `plane=True`**（它的默认是 `False`）。
本模块只负责给 refine 一个初值。

设计文档与开发记录都在 maduo 仓库（`docs/superpowers/specs/2026-09-22-…`、
`WORKLOG_3test.md`、`.sdd-detect/progress.md`），**不在本仓库**。
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


# ---- 内联的依赖 ----------------------------------------------------------
# 下面这些原先散在 maduo 的 `ground_detector` / `refine_pallet_frame` /
# `rgbd_detector` 里。合并进 LeTools 时**逐字抠出来内联**，好让本模块自成一体
# （与 `box_frame/algorithm.py` 同一做法）。抠出来的内容与 maduo 那边**逐位一致**
# —— 见 `tests/test_pallet_detect.py` 的逐格比对。
#
# 名字保留前导下划线、**一处没改**：正文里原先写作 `_wood_mask` 的调用点，
# 这里只去掉了 `gd.` 前缀，函数体一行没动。

# 木色（托盘台面 / 纸箱）与蓝色底座、黄色护栏的色域分界。
WOOD_HSV = (5, 8, 70), (45, 145, 255)
# ⚠️ **50 -> 8**（2026-09-29 现场实测）。这个下限原本是"挡住灰色地面与蓝色边框"，
# 但现场那批浅色木托盘**饱和度中位数只有 3~7**（S = (max-min)/max，托盘发白就是
# 分母大），连 `WOOD_HSV` 自带的下界 8 都够不到 —— 50 的门槛一个像素都放不进来。
# 降到 8 = 等于把这道附加门槛关掉；蓝色（H 88~132）与黄色（H 18~42）仍有
# `_wood_mask` 末尾的显式排除，实测蓝色地面**零泄漏**。
WOOD_SAT_MIN = 8
# 浅色通路：发白的木托盘进不了上面的色域（饱和度太低），但它**又亮又中性**。
# 与木色通路取并集，再走同一套蓝/黄排除。
# ⚠️ **这条是"低饱和 + 高亮"，不是"木色"** —— 它会一并放进浅灰的地面/白墙。
# 现场地面 S 中位 56，所以分得开；**换到白地板/强反光地面就会误检**。
# 真出问题就把这里调严（S 上限调小、V 下限调大），或者把 `& PALE_HSV` 去掉。
PALE_HSV = (20, 160)       # (S 上限, V 下限)
# `_deck_sat_min` 用的地板：高度带里木色的占比低于它就放宽饱和度下限。
# 见 `_deck_sat_min` 的 docstring —— **与 `_color_coverage` 的分母不一致是已知的**，
# 操作员 2026-09-23 裁决「先接受」。
WOOD_COVERAGE_MIN = 0.40
BLUE_HSV = (88, 55, 35), (132, 255, 255)
YELLOW_HSV = (18, 150, 80), (42, 255, 255)

# 地面系体素边长（mm）。每个有效深度像素反投影后在 (a, b, h) 上分箱，每个体素用
# 成员点的**质心**代表，后面的台面掩码/顶视栅格都在体素代表点上跑。
VOXEL_MM = 5.0             # 0.5 cm

# ⚠️ **40 -> 85**（2026-09-29 现场实测）。法向偏 ε 时真值台面在"高度"坐标里变成
# 一条斜坡、跨度 ≈ tan(ε) × 台面尺寸；现场那一帧的 TF 法向偏 **6.83°**，台面因此
# 摊开 **128mm** —— 40mm 的带只装得下 1/3，掩码塌成月牙，score 0.04 过不了。
# 85mm 留 3 倍余量（实测该帧 score 0.54，10/10 帧全过）。
#
# ⚠️ **硬上限 90：不能 >= `BOX_RAISE_MM`(90)。** `_deck_mask` 里有一条
# "扣货"判据用 `h - deck_h >= BOX_RAISE_MM`；`DECK_BAND_MM < BOX_RAISE_MM` 时
# 那条判据**恒不可达**（两个集合交集为空），现在的 85 仍是死代码。
# **调大前先看 `test_deck_mask_deadcode`** —— 它会 FAIL 提醒那行要复活了。
DECK_BAND_MM = 85.0        # deck pixels lie within this of the adaptive deck height


@dataclass(frozen=True)
class CameraIntrinsics:
    """针孔内参。与 `rgbd_detector.CameraIntrinsics` 同构（逐字段同名同序）。"""
    fx: float
    fy: float
    cx: float
    cy: float


def _backproject(us: np.ndarray, vs: np.ndarray, zs: np.ndarray,
                 k: CameraIntrinsics) -> np.ndarray:
    return np.stack([(us - k.cx) * zs / k.fx, (vs - k.cy) * zs / k.fy, zs], axis=-1)


def _project_px(points: np.ndarray, k: CameraIntrinsics) -> np.ndarray:
    z = np.where(np.abs(points[..., 2]) < 1e-6, 1e-6, points[..., 2])
    return np.stack([k.cx + k.fx * points[..., 0] / z,
                     k.cy + k.fy * points[..., 1] / z], axis=-1)


def _corners_mm(frame: dict, W: float, H: float) -> np.ndarray:
    """台面矩形四个角，相机系 mm。顺序 (0,0) -> (W,0) -> (W,H) -> (0,H)。

    ⚠️ 写成一次向量化（原来是 4 次带 Python 列表推导的逐角相加）。这个函数
    一帧被调 70+ 次（`_edges_px` 里调它，`_residuals` 里每次迭代又调 `_edges_px`），
    值得省掉列表推导那点固定开销。
    """
    o, E1, E2 = frame['origin'], frame['E1'], frame['E2']
    ab = np.array([[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]], np.float64)
    return (o[None, :] + ab[:, 0:1] * E1[None, :] + ab[:, 1:2] * E2[None, :])


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


def _voxel_centroids(a: np.ndarray, b: np.ndarray, h: np.ndarray,
                     color: np.ndarray, z: np.ndarray,
                     voxel_mm: float) -> tuple[np.ndarray, ...]:
    """Bin ground-frame points into `voxel_mm` cubes; return the per-voxel
    centroid position `(a, b, h)`, mean color and mean camera depth.

    The grid is anchored at the cloud's own minimum on each axis so the bin
    index stays a small non-negative integer before being packed into one
    int64 key for `np.unique`.
    """
    ia = np.floor((a - a.min()) / voxel_mm).astype(np.int64)
    ib = np.floor((b - b.min()) / voxel_mm).astype(np.int64)
    ih = np.floor((h - h.min()) / voxel_mm).astype(np.int64)
    key = (ia * (ib.max() + 1) + ib) * (ih.max() + 1) + ih
    _, inv = np.unique(key, return_inverse=True)
    inv = inv.reshape(-1)
    n = int(inv.max()) + 1
    count = np.bincount(inv, minlength=n).astype(np.float64)

    def mean(values: np.ndarray) -> np.ndarray:
        return np.bincount(inv, weights=values, minlength=n) / count

    color = np.stack([mean(color[:, c].astype(np.float64)) for c in range(3)], axis=-1)
    return mean(a), mean(b), mean(h), np.round(color).astype(np.uint8), mean(z)


def _ground_points(color: np.ndarray, depth: np.ndarray, k: CameraIntrinsics,
                   frame: dict[str, Any], voxel_mm: float = 0.0
                   ) -> dict[str, np.ndarray] | None:
    """Back-project valid depth pixels and (optionally) voxelise in the ground frame.

    Returns arrays `a, b, h` (mm, ground frame), `color` (BGR) and `z` (camera
    depth, used for the painter's-algorithm occlusion order).  With
    `voxel_mm <= 0` this is the plain per-pixel cloud.
    """
    origin, E1, E2, nrm = frame['origin'], frame['E1'], frame['E2'], frame['nrm']
    vy, ux = np.nonzero(depth > 0)
    if vy.size == 0:
        return None
    P = _backproject(ux.astype(np.float64), vy.astype(np.float64),
                     depth[vy, ux].astype(np.float64), k)
    r = P - origin
    a, b, h = r @ E1, r @ E2, r @ nrm
    cloud = dict(a=a, b=b, h=h, color=color[vy, ux].astype(np.uint8),
                 z=depth[vy, ux].astype(np.float64))
    if voxel_mm and voxel_mm > 0:
        cloud['a'], cloud['b'], cloud['h'], cloud['color'], cloud['z'] = _voxel_centroids(
            cloud['a'], cloud['b'], cloud['h'], cloud['color'], cloud['z'], voxel_mm)
    return cloud


def _paint(canvas: np.ndarray, height: np.ndarray,
           ip: np.ndarray, iq: np.ndarray, color: np.ndarray,
           hgt: np.ndarray, z: np.ndarray, radius: int) -> None:
    """Painter's algorithm: far points first, so the nearest surface wins.

    `radius` fills the voxel footprint (radius 0 = a single pixel).  Each offset
    is one flat scatter instead of a Python loop; numpy applies duplicate indices
    in order, so the last -- nearest -- point still wins, exactly as before.

    Caveat carried over from the loop: the offsets are applied in sequence, so a
    later offset overrides an earlier one regardless of depth.  `radius > 0` is
    therefore a footprint *fill*, not a strictly depth-correct splat.  Kept
    as-is -- every caller uses the default `radius == 0`.
    """
    offsets = [(di, dj) for di in range(-radius, radius + 1)
               for dj in range(-radius, radius + 1)]
    if not offsets:
        return
    order = np.argsort(-z)
    ip, iq = ip[order], iq[order]
    color, hgt = color[order], hgt[order]
    rows, cols = canvas.shape[:2]
    if len(offsets) > 1:
        ip = np.concatenate([ip + di for di, _ in offsets])
        iq = np.concatenate([iq + dj for _, dj in offsets])
        color = np.tile(color, (len(offsets), 1))
        hgt = np.tile(hgt, len(offsets))
    ok = (ip >= 0) & (ip < cols) & (iq >= 0) & (iq < rows)
    p, q = ip[ok], iq[ok]
    canvas[q, p] = color[ok]
    height[q, p] = hgt[ok]


def _wood_mask(top: np.ndarray, sat_min: float = WOOD_SAT_MIN) -> np.ndarray:
    """台面候选掩码 = **木色通路 ∪ 浅色通路**，再排除蓝 / 黄。

    ⚠️ **两条通路并列是刻意的**（2026-09-29 现场实测）：木色通路认的是"有色的木"，
    浅色通路认的是"发白但很亮的木" —— 后者进不了 `WOOD_HSV` 的色域（饱和度太低），
    只有并上才能覆盖浅色托盘。浅色通路的误检风险见 `PALE_HSV` 的注释。

    `sat_min` 只作用于**木色通路**（`_deck_sat_min` 会按帧放宽它）；浅色通路的
    阈值是常量 `PALE_HSV`，不随帧自适应 —— 放宽它没有对应的"该不该放宽"判据。
    """
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    wood = cv2.inRange(hsv, np.asarray(WOOD_HSV[0], np.uint8),
                       np.asarray(WOOD_HSV[1], np.uint8)) > 0
    blue = cv2.inRange(hsv, np.asarray(BLUE_HSV[0], np.uint8),
                       np.asarray(BLUE_HSV[1], np.uint8)) > 0
    yellow = cv2.inRange(hsv, np.asarray(YELLOW_HSV[0], np.uint8),
                         np.asarray(YELLOW_HSV[1], np.uint8)) > 0
    saturated = hsv[:, :, 1] >= sat_min
    pale = (hsv[:, :, 1] <= PALE_HSV[0]) & (hsv[:, :, 2] >= PALE_HSV[1])
    return ((wood & saturated) | pale) & ~blue & ~yellow


def _deck_sat_min(height: np.ndarray, wood: np.ndarray, frame: dict[str, Any],
                  deck_h: float) -> float:
    """Saturation floor to use for the deck mask of THIS frame.

    `WOOD_SAT_MIN` is an absolute floor, and saturation is brightness-relative
    (`S = (max - min) / max`), so it only means anything for the exposure it was
    calibrated on.  A view that renders the same wood darker drives the whole
    surface under it at once: 10_test looks at the stack from the short end,
    near-grazing, where the pallet's top boards blend with their own shadow --
    measured across the operator's GT the pallet's own saturation slides from 59
    at the near edge to 37 at the far one, straddling the 50 floor.  The mask
    then keeps only the patch nearest the camera (23% of the pallet), and
    `_dense_bounds` fits its inscribed rectangle to that patch instead of to the
    pallet -- 0.13 IoU against the operator's GT.

    The gate can be checked against its own job.  It exists to separate wood from
    the grey floor and the blue rim, so the deck band -- the surface at the
    adaptive deck height, which `_adaptive_deck_height` has already located from
    the pixels the gate DID accept -- ought to be mostly wood.  When it is not,
    the floor is what is wrong, not the scene, and dropping it to the lower edge
    of `WOOD_HSV` (which still caps S at 145, so the yellow rail stays excluded)
    recovers the surface.  On 10_test that is 0.956 / 0.928 / 0.929 IoU.

    Coverage on the golden frames is 89 / 78 / 55 / 91%, so they never trip the
    test and this is a no-op for them; the relaxed path is only reached on a
    frame whose gate has already failed.  The 40% cut sits between the two
    populations, with 3_test (55%) the nearer side -- if that frame's mask ever
    loosens, this is the constant that moved first.
    """
    H, W = frame['H'], frame['W']
    band = np.zeros(height.shape, bool)
    band[:H, :W] = True
    band &= (height > -1e3) & (np.abs(height - deck_h) <= DECK_BAND_MM)
    n_band = int(band.sum())
    if n_band < 200:
        return WOOD_SAT_MIN
    if int((band & wood).sum()) >= WOOD_COVERAGE_MIN * n_band:
        return WOOD_SAT_MIN
    return float(WOOD_HSV[0][1])


def _adaptive_deck_height(height: np.ndarray, wood: np.ndarray,
                          z_band: tuple[float, float] | None = None) -> float | None:
    """Robust median height of the dominant flat warm plane (the deck).

    Iteratively trims via median absolute deviation so cargo boxes (much
    higher) and any blue-rim / floor bleed (much lower) do not bias the deck.

    `z_band`（可选，`(lo, hi)`）：**只在这个高度带内选台面**，单位与 `height`
    一致。给它是为了排除"高度上根本不可能"的东西 —— 现场实测手里抱着的纸箱
    在 base_link 的 z ≈ +600mm，而托盘 ≈ +43mm、地面 ≈ −107mm；
    不带这个约束时，那 2.9 万个橙黄色箱子像素会把中位拉到两层之间
    （实测 `h_deck` 落在 615mm，掩码整个建在箱子上）。

    ⚠️ **默认 `None` = 完全保持原有行为**（`_rasterise` 出来的 `height` 是
    "沿法向、相对点云质心"的量，**本身没有绝对意义**，所以带内约束必须由调用方
    先把它换算到同一个基准上再传进来）。换算方式见 `detect_pallet_frame` 的
    `camera_z_mm` 参数。

    ⚠️ **带内木色格不足 500 时返回 `None`**（= 拒帧），**不回退到无约束的中位** ——
    回退就等于"约束不存在"，那正是要避免的静默失效。
    """
    if z_band is not None:
        lo, hi = float(z_band[0]), float(z_band[1])
        m = (wood > 0) & (height > -500) & (height >= lo) & (height <= hi)
        if int(m.sum()) < 500:
            return None
        hw = height[m]
    else:
        hw = height[(wood > 0) & (height > -500)]
    if hw.size < 500:
        return None
    med = float(np.median(hw))
    for _ in range(3):
        mad = float(np.median(np.abs(hw - med)))
        keep = np.abs(hw - med) <= 2.5 * max(mad, 8.0)
        if int(keep.sum()) < 200:
            break
        hw = hw[keep]
        med = float(np.median(hw))
    return med

# ---- 内联结束 ------------------------------------------------------------


# --------------------------------------------------------------------------- #
# 常量
# --------------------------------------------------------------------------- #
TARGET_MM = (1200.0, 1000.0)      # 操作员给：托盘 120cm x 100cm
LONG_SIDE_PARALLEL = False        # 操作员给：5_test 里与画面近平行的边是短边
VOXEL_MM = 5.0                    # 5.0，沿用 ground_detector（见本文件顶部的内联块）
PLACEHOLDER_MM = 4000.0           # 拍的：占位栅格边长，需 >= 托盘对角 1562mm 且留余量
COARSE_CELL_MM = 4.0              # 拍的：需 <= 收敛域 50mm 的 1/10
THETA_COARSE_STEP_DEG = 2.0       # 拍的
THETA_FINE_STEP_DEG = 0.25        # 拍的
THETA_FINE_HALF_DEG = 4.0         # 拍的：精搜在粗搜赢家附近的窗口
# T9：精搜的**位移**不再全画布穷举 —— 只在粗搜赢家周围 ±这个值（mm）里细化。
# ⚠️ 取 8 不是拍的，是实测出来的（6 帧真值数据，见 `_search_rect` 的 docstring）：
# 粗搜的 4mm 网格本身已经给出 ±4mm 的定位，窗口要 >= 那个误差才装得下真正的最优解。
# 实测 win=2 有 4/6 帧结果不同、win=4 有 3/6 帧不同、**win=8 起 6/6 帧逐位一致**。
FINE_SHIFT_WIN_MM = 8
# T9c：闭运算的核边长（**不能改**，见 `_deck_mask` 里那段论证）与它的局部化边距。
# ⚠️ `CLOSE_PAD_MM` 必须 >= `CLOSE_KERNEL`：闭运算对某格的影响半径是**两倍核半径**
# （先腐蚀会"看见" 2r 外的结构，膨胀再把它送出去），pad 不够就会切掉边界效应。
CLOSE_KERNEL = 25
CLOSE_PAD_MM = 26
THETA_HALF_DEG = 45.0             # 半象限，由规格 §4 Step 4「窗口 = 一个完整象限」反推

# --------------------------------------------------------------------------- #
# T9b：先验跟踪路径的窗口（操作员 2026-09-23 给的数据）
# --------------------------------------------------------------------------- #
# 操作员：「我的木托盘在实际中每一帧的移动其实很小」+ 明确答复两帧之间（10Hz）
# **最多 150mm / 5°**。真值实测（april_test7 相邻帧，dt=33.4ms）：
# Δorigin 最大 37.1mm、Δθ(E1) 最大 2.41° —— 150/5 留了约 4x / 2x 余量。
#
# ⚠️ **2026-09-29 现场实测后收窄到 60mm / 3°**。原值 150/5 是按"托盘可能被动
# 挪动"定的，但**伺服跟踪时托盘不动、动的是机器人**，相邻帧的相对位移小得多：
# 现场 10 帧实测 **位移 max 9.9mm、dθ max 0.47°**（dt=67ms）。60/3 相对实测值
# 仍有 6x 余量，却把先验搜索的 ROI 缩掉一大圈 —— 同一批帧 `search` 段
# 175~200ms -> **66~80ms**（`PRIOR_*` 是 ROI 尺寸的直接乘数，
# 见 `_search_rect_prior` 的 docstring）。
#
# ⚠️ 这两个数是**搜索窗**，不是"允许多少误差"：真超出去了不是给出错答案，
# 而是**退回全画布搜索**（见 `PRIOR_MIN_SCORE` 与 `_search_rect_prior`）。
# 所以收窄的代价只是"动得快时那一帧慢一点"，不会错 —— 但也**别收得比
# 实际位移还小**，那会让先验路径大部分帧都退回全局，等于没接。
# ⚠️ **一帧全局、一帧先验要轮流有结果**：全局那帧必须在 `PRIOR_THETA_HALF_DEG`
# 覆盖得到的地方初始化，所以窗收窄后**首帧/丢失后重捕的那一帧要多等一帧**。
PRIOR_THETA_HALF_DEG = 3.0        # 先验 θ 周围的细扫半窗
PRIOR_SHIFT_WIN_MM = 60.0         # 先验位置周围的位移窗
# 先验路径的 ROI 在「候选矩形并集」外再放几毫米，避免浮点/取整把真最优切掉
PRIOR_PAD_MM = 6.0
# 先验路径的结果低于这个 coverage 就**退回全画布搜索**（不静默接受一个坏结果）。
# 取 `COVERAGE_MIN` 而不是另一个拍的值：低于它本来就会被判 `ambiguous_theta`，
# 那还不如直接回退去拿全搜索的结果 —— 多花 2.7s，但不会跟丢。
# （定义在 `COVERAGE_MIN` 之后，见下方。）
# ⚠️ **这个不是 `refine.py` 里那个 `EDGE_SUPPORT_MIN`（0.5）**，两者量的是不同
# 东西，别去"统一"它们：
#   * 这里（`_edge_support`）量的是**掩码支撑率** —— 沿预测边采样，有多大比例
#     的点落在台面掩码的边界上。判据跑在**占位栅格**里，量的是"这条边在不在
#     分割出来的台面上"。结果进 `diag['edges'][i]['observed']`，进而决定
#     `n_observed_edges`（< 2 就 reject `too_few_edges`）。
#   * `refine.py` 那个量的是**直线支撑率** —— 已经采到的观测点有多大比例落在
#     稳健拟合出来的直线附近。判据跑在**图像**里，量的是"这些点排不排成一条线"。
# 两个 0.30 / 0.5 都是拍的，无数据支撑（规格 §6）。
EDGE_MASK_SUPPORT_MIN = 0.30
COVERAGE_MIN = 0.35               # 拍的：无数据支撑（规格 §6）
# 先验路径的结果低于这个 coverage 就**退回全画布搜索**（不静默接受一个坏结果）。
# 取 `COVERAGE_MIN` 而不是另一个拍的值：低于它本来就会被判 `ambiguous_theta`，
# 那还不如直接回退去拿全搜索的结果 —— 多花几秒，但不会跟丢。
PRIOR_MIN_SCORE = COVERAGE_MIN
MIN_WOOD_PX = 500                 # 拍的：木色像素下限，少于这个直接 no_wood
MIN_DEPTH_CELLS = 400             # 拍的：深度路线里一个高度带至少要有这么多格才算"面"
                                  # （沿用 OCCUPANCY_MIN_CELLS 的量级）

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

# T5c **新判据**（不是调参）：掩码面积 / 托盘面积 的**上限**，超过就拒绝。
#
# ⚠️ 它拦的是**另一个问题**，与 `COVERAGE_MIN` 分工不同，别以为一个判据能修两个：
#
#   - `COVERAGE_MIN` 拦的是"掩码太小/太稀，已知尺寸矩形框不满"（5_test 实测
#     掩码面积比 0.64、coverage 0.261 —— 它是被 `COVERAGE_MIN` 拦下的）；
#   - 本条拦的是"掩码**太大**"：深度路线取到**地面**那一带后，25x25 闭运算把
#     整张栅格填实，掩码变成"整块地面"。此时**任何**已知尺寸矩形都落在掩码内部，
#     `coverage` 恒等于 1.000 —— `COVERAGE_MIN` 是**下界**，对它完全失效
#     （实测合成场景：261978 格 -> 11 648 628 格，膨胀 45.3 倍，占栅格 58%）。
#     能分开"整块地面"与"台面"的只有**掩码的绝对大小**。
#
# 阈值 3.0 取在实测的可分区间的中间（全部为 voxel=2.0mm 的合成场景实测）：
#
#   | 用例                       | source | 掩码格数   | 面积比 | coverage | 该拒 |
#   |----------------------------|--------|-----------|--------|----------|------|
#   | 颜色路线（无遮挡）          | color |  1 195 789 | 0.997 | 0.993 | 否 |
#   | 深度路线 场景A（地面小）    | depth  |  1 217 996 | 1.015 | 0.998 | 否 |
#   | 深度路线 遮 3 边            | depth  | 11 648 628 | 9.707 | 1.000 | 是 |
#   | 深度路线 遮对边             | depth  | 11 648 628 | 9.707 | 1.000 | 是 |
#   | 5_test 真实数据             | depth  |    766 411 | 0.64  | 0.261 | 是*|
#
#   （* 5_test 是**被 `COVERAGE_MIN` 拦下的**，本条拦不住它 —— 0.64 < 3.0。
#     这两条是不同的问题，**不要**因为 5_test 失败就动这个阈值。）
#
# 该成功的一条 1.015、该拒绝的两条 9.707 —— 判据在合成场景上完全可分，3.0 两边
# 各留了 ~3 倍余量。**注意：遮 1 边 / 遮 2 边相邻也走深度路线、掩码与遮 3 边
# 逐格相同**（深度路线不做任何颜色过滤），所以它们同样会被这条拦下。
#
# ⚠️ **T5d 复核（换分母之后）**：上面 5 行里只有第 1 行是颜色路线，而它新旧定义下
# 都走颜色路线、掩码逐格不变；第 2 行（台面刷灰）与 3/4 行（`wood` 全灭）本来就是
# 深度路线，与 `_color_coverage` 无关。**所以这条判据的每一格数字都没变。**
# 唯一的语义变化是：遮 1 边 / 遮 2 边相邻**不再**走深度路线了（换分母后回到颜色
# 路线），所以**这条判据不再拦它们** —— 它们现在卡在 `COVERAGE_MIN` 上
# （实测 score 0.3140 / 0.1589 < 0.35，见 `_color_coverage` 与那条测试的 docstring）。
MASK_AREA_RATIO_MAX = 3.0         # T5c 新判据：阈值取在实测的 1.015 与 9.707 之间



# --------------------------------------------------------------------------- #
# 占位基
# --------------------------------------------------------------------------- #
def _basis_from_normal(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """平面内一组正交单位基，`u` 取平面内最接近相机系 +x 的方向。

    相机系里 +x 就是图像右，所以 `u` 一开始就"大致指向画面右"。后面搜索到的
    转角 θ 是在这个基上的**增量**，不是绝对朝向 —— 这样 θ 的搜索窗才小。
    """
    n = np.asarray(n, float)
    n = n / np.linalg.norm(n)
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(ref @ n)) > 0.99:            # 法向几乎沿 +x：换一个参考方向
        ref = np.array([0.0, 0.0, -1.0])
    u = ref - float(ref @ n) * n
    u = u / np.linalg.norm(u)
    v = np.cross(n, u)
    return u, v


# --------------------------------------------------------------------------- #
# 占位 frame + 台面掩码（颜色 / 深度双路）
# --------------------------------------------------------------------------- #
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
                     seed: int = FIT_NORMAL_SEED):
    """从**当前帧**深度拟合"占画面最大的那个平面"的法向。返回 `(normal, n_inliers)`。

    失败返回 `(None, 0)` —— **调用方据此刻意回退到 `prior`**（见下）。

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
    P = _backproject_cloud(depth, k)
    if P.shape[0] < min_inliers:
        return None, 0
    n0 = np.asarray(prior, float)
    norm0 = float(np.linalg.norm(n0))
    if not np.isfinite(norm0) or norm0 < 1e-9:
        return None, 0
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
        return None, 0

    # 用全部内点做一次 SVD 精修（RANSAC 那个三点解本身是有偏的）
    inl = Q[np.abs((Q - best_p0) @ best_n) < tol_mm]
    if inl.shape[0] < min_inliers:
        return None, 0
    centre = inl.mean(axis=0)
    n = np.linalg.svd(inl - centre, full_matrices=False)[2][2]
    if n[2] > 0.0:                       # 与 `_fit_plane` 同一约定：法向朝 -z 侧
        n = -n
    if float(n @ n0) < 0.0:              # 再按初值定符号
        n = -n
    return n / np.linalg.norm(n), int(inl.shape[0])


def _placeholder_frame(normal: np.ndarray, origin: np.ndarray | None = None,
                       w_h: float = PLACEHOLDER_MM) -> dict:
    """占位 frame：法向给定，平面内基任意，W/H 开得足够大。

    ⚠️ `origin` 不能留在相机原点。`_ground_points` 算出的 `h` 是**相对 origin**
    的高度，而 `_adaptive_deck_height` 里有一条 `height > -500` 的过滤 —— origin 在
    相机原点时整帧的 h 都在 -1800 上下，**木色点会被这条全滤掉**，
    `_adaptive_deck_height` 直接返回 None（实测过，这是 T1 报告里那个坑的同源问题）。

    因为法向已知，h 只差一个常数（规格 §2.2），origin 放在哪**不影响掩码的正确性**，
    只影响数值大小。这里取点云在平面上的投影质心，让 h 关于 0 大致对称。
    """
    u, v = _basis_from_normal(normal)
    return dict(origin=(np.zeros(3) if origin is None else np.asarray(origin, float)),
                E1=u, E2=v, nrm=np.asarray(normal, float),
                W=int(w_h), H=int(w_h))


def _rasterise(color: np.ndarray, depth: np.ndarray, k, normal: np.ndarray,
               opts: dict | None = None):
    """把点云栅格化成占位顶视图。返回 `(top, height, info)`；点云为空时 `(None, None, None)`。

    `info` 里的 `p0`/`q0` 是栅格原点在 (a, b) 上的偏移，`origin` 是占位平面原点的
    相机系坐标 —— 两者后面都要用来把真值/预测投到同一张栅格上，**别丢**。

    ⚠️ **T9c：第二趟反投影已省掉**（原来反投影两遍，第二遍纯属重复劳动）。
    `_voxel_centroids` 的分箱锚在 `a.min()`/`b.min()`/`h.min()` 上，所以把
    origin 平移一个常数后，**每个点落进哪个体素逐点相同**，只是 `a/b/h` 整体
    平移同一个常数。实测（`debug/3test/detect_synthetic_test.py::test_rasterise_single_pass`）
    两趟的逐点差 `<= 1e-12 mm`、重栅格化后高度图/彩色图**逐格相同**、
    `_wood_mask` 后的掩码**逐格相同**。所以第二趟换成"第一趟点云减一个常数"。
    省下的是一整趟反投影 + 体素化：实测 33ms（april_test7）~ 115ms（april_test6）。
    """
    opts = dict(opts or {})
    voxel_mm = float(opts.get('voxel_mm', VOXEL_MM))
    u, v = _basis_from_normal(normal)
    cloud = _ground_points(color, depth, k, _placeholder_frame(normal), voxel_mm)
    if cloud is None or cloud['a'].size == 0:
        return None, None, None
    # 第一趟只为求质心（见 `_placeholder_frame`），第二趟的 origin 就用它。
    ca = float(np.median(cloud['a']))
    cb = float(np.median(cloud['b']))
    ch = float(np.median(cloud['h']))
    origin = u * ca + v * cb + np.asarray(normal, float) * ch
    # 平移量（相机系）= 新 origin - 旧 origin。旧 origin 是零向量（`_placeholder_frame(normal)`）。
    # 点云的 (a, b, h) 是**相对 origin** 的，所以 origin 从 0 挪到 `origin` 后，
    # 每个点的 (a, b, h) 各减去 origin 在对应基上的投影。
    da, db, dh = float(origin @ u), float(origin @ v), float(origin @ np.asarray(normal, float))
    cloud = dict(a=cloud['a'] - da, b=cloud['b'] - db, h=cloud['h'] - dh,
                 color=cloud['color'], z=cloud['z'])
    frame = _placeholder_frame(normal, origin)

    p0 = float(cloud['a'].min()) - 2.0
    q0 = float(cloud['b'].min()) - 2.0
    W = int(np.floor(float(cloud['a'].max()) - p0)) + 4
    H = int(np.floor(float(cloud['b'].max()) - q0)) + 4
    top = np.zeros((H, W, 3), np.uint8)
    height = np.full((H, W), -1e3, np.float32)
    ip = np.clip(np.floor(cloud['a'] - p0).astype(np.int64), 0, W - 1)
    iq = np.clip(np.floor(cloud['b'] - q0).astype(np.int64), 0, H - 1)
    _paint(top, height, ip, iq, cloud['color'], cloud['h'].astype(np.float32),
              cloud['z'], 0)
    return top, height, dict(p0=p0, q0=q0, origin=frame['origin'].copy(),
                             voxel_mm=voxel_mm)


def _color_coverage(height: np.ndarray, wood: np.ndarray, deck_h: float) -> float | None:
    """**木色格有多大比例落在它们自己的那个高度层上**（T5d：分母已换，见下）。

    新定义（操作员 2026-09-23 裁决）：`coverage = |band ∩ wood| / |wood|`，
    其中 `band = 高度在 deck_h ± DECK_BAND_MM 内的格`。

    **这就是 §4 Step 2 那个选择器用的量** —— 它问的是"颜色门控有没有对上它该描述
    的那个面"。`_deck_sat_min` 内部问的是同一件事，但它只返回"要不要放宽"，
    这里要的是**数值本身**，所以单独算一遍，不反推。

    ⚠️ **为什么分母该是木色格而不是所有格**（T5d 的根因，别再改回去）：

    旧定义是 `|band ∩ wood| / |band|`（分母 = 高度带里的**所有**格）。
    它在"台面被遮挡"时会**误判** —— 因为合成场景（以及真实场景）里，盖住托盘
    一条边的遮挡物**落在与台面同一个高度带内**（`render()` 的遮挡块不抬高深度，
    深度上就是台面），于是这些格被算进分母、却不可能同时是木色：

      | 用例        | 木色格 | 旧(÷band) | 新(÷wood) |
      |-------------|--------|-----------|-----------|
      | 无遮挡      | 44 542 |     1.000 | **1.000** |
      | 遮 1 边     | 11 241 |    0.2524 | **1.000** |
      | 遮 2 边相邻 |  5 579 |    0.1253 | **1.000** |
      | 遮 3 边/对边|  **0** |      None |    None   |

    （上表是 `voxel_mm=2.0` 的合成场景实测；`voxel_mm=5.0` 那组同量级：
    40 270 / 11 003 / 5 535，旧值 0.273 / 0.137。）

    旧定义把"台面被盖住"读成了"台面不木色"，于是选择器**误切深度路线** ——
    而深度路线在这个场景里必然选到地面（见 `_dominant_band`），全盘皆错。
    **但遮挡只减少木色的数量，不改变它们的高度集中度** —— 所以问"木色自己的
    高度集中度"才是选择器该问的问题。新定义下遮挡用例恒为 1.000（实测）。

    ⚠️ **常量名 `WOOD_COVERAGE_MIN`（0.40）保持不变，但它的语义变了**：
    它现在比的是"木色格落在台面高度带内的比例"，不再是"高度带里木色的占比"。
    名字是历史遗留（T3 定的），改名会牵动 `ground_detector.py`（**不允许改**），
    所以在调用处与这里说明，不重命名。

    ⚠️⚠️ **已知的不一致：`_deck_sat_min` 用的仍是旧分母 —— 操作员 2026-09-23 裁决「先接受」**。
    T5d 只换了**这里**的分母（`÷ band` → `÷ wood`），**没有**换
    `ground_detector.py::_deck_sat_min` 里的那一处（它在禁改文件里）。
    于是**同一个常量 `WOOD_COVERAGE_MIN = 0.40` 在同一帧上会问出两个相反的答案**：

      | 函数 | 所在文件 | 分母 | 5_test 实测 | 结论 |
      |---|---|---|---|---|
      | `_color_coverage`（这里） | `detect_pallet_frame.py` | `÷ wood` | 0.6043 | **不放宽** |
      | `_deck_sat_min` | `ground_detector.py` | `÷ band` | 0.0775 | **放宽** |

    5_test 上两者都跑了：`_deck_sat_min` 判定"放宽"→ `sat_min` 从 50 降到 8
    （`diag['sat_min'] = 8.0`），`wood` 从 4435 涨到 15497；随后**这里**用新分母
    判"不放宽"→ `source = 'color'`。**两个判断都生效了，只是问的不是同一件事**，
    结果恰好是我们想要的（放宽让木色多一点，走颜色路线），但**这是巧合不是设计**。

    **为什么本轮不修**（操作员 2026-09-23 裁决「先接受，注释好」）：
      1. `_deck_sat_min` 在 `ground_detector.py` 里，动它要走 4 帧 golden 回归
         （`debug/3test/regress.sh` 对 `output/` 逐字比 JSON）；
      2. **那条回归对这条路径是盲的** —— `_deck_sat_min` 的放宽分支只在
         "覆盖率 < 0.40" 时触发，而 4 帧 golden 的覆盖率是 55~91%，
         **永远不会走到**。也就是说改了它，**回归给不出任何证据**，
         得另设计验证手段；
      3. 它**不是 5_test 失败的主因**：放宽后真值矩形内的木色点 recall 也只从
         0.08% 到 0.36%，掩码与真值矩形 IoU 仍只有 0.0589。
         门控放宽救不了"木色点结构性稀薄"。

    **将来要统一时，别只改一边**：`_deck_sat_min` 的 docstring 里那段论证
    （"deck band 应当大部分是木色"）是按 `÷ band` 写的，换分母要连论证一起重写，
    并且要**新增**一个能触发放宽分支的验证用例（合成场景里把台面刷成低饱和度即可）。

    ⚠️ **`wood` 为 0 格时返回 `None`**（现有语义：走深度路线）。注意这与
    "占比低"是**两件事**：`wood` 全灭时不是"木色不集中"，而是**一个木色像素都没有**
    （合成场景里遮 3 边/遮对边时遮挡带盖住了整块台面）。这种用例**走哪条路都是
    失败**，不是换分母能修的。
    """
    n_wood = int(wood.sum())
    if n_wood == 0:
        return None
    band = (height > -1e3) & (np.abs(height - deck_h) <= DECK_BAND_MM)
    return float((band & wood).sum()) / n_wood


def _dominant_band(height: np.ndarray, min_cells: int = MIN_DEPTH_CELLS
                   ) -> tuple[float | None, float]:
    """深度路线：取高度直方图里**格数最多**的那一带。返回 `(h_deck | None, 该带占比)`。

    ⚠️ 先按**单元支撑度**筛出真正的"面"再取最大，不要直接对逐点高度取直方图峰值 ——
    栅格噪声会造出假峰，而一个真正的水平面是成千上万个格共享同一个高度。
    （这与 `_height_clusters` 是同一个思路，只是那边用格数当门限、这边还要取最大。）

    ⚠️ **这条"最大一带 = 台面"的判据在真实数据上不成立**（2026-09-23 实测，5_test，
    法向用操作员点击反算的真值；参考面用**已由操作员目视确认在台面上**的 refine origin，
    图 `debug/3test/_5test_plane_question.jpg`）：
      - 台面在占位栅格里的高度 h = **+509mm**；
      - `_dominant_band` 选出的是 **h = −235mm** —— **离台面 −744mm，完全不是台面**；
        那一带 28752 格、**82.2% 是蓝色**（BGR 中位 (62,62,44)），与真值台面矩形
        **IoU = 0.000**；
      - 作为对照，真值台面那一带（h≈+509）有 32218 格、暖色 42.7%、
        BGR 中位 (162,166,161)，与真值矩形 IoU = 0.022。
    判据本身（"最大的那个面就是台面"）在真实场景里不成立。

    ⚠️ **本文档早先写过一版数字（"真值台面 379mm"、"最大带 56311 格"、
    "带内真值占比 0.000"、"bbox 归一化得 0.0345"），那些来自有缺陷的探针，已作废** ——
    缺陷是**把体素栅格当成了平面像素**（1 格 = 一个 5mm 体素 = 25 mm²，不是 1mm²）
    以及**没区分"在台面投影内/外"**。上面的数字是重测的。**具体错在哪一层、为什么，
    尚未查清**（操作员已裁决本轮接受失败，故未继续深挖）。
    详见 `.sdd-detect/task-3-report.md`。**保留原判据不动，不靠调阈值蒙过去。**
    """
    valid = height > -1e3
    n_valid = int(valid.sum())
    if n_valid < min_cells:
        return None, 0.0
    cell = float(VOXEL_MM)
    bins = np.round(height[valid] / cell) * cell
    uniq, cnt = np.unique(bins, return_counts=True)
    keep = cnt >= min_cells
    if not keep.any():
        return None, 0.0
    uniq, cnt = uniq[keep], cnt[keep]
    # 相邻带合并：一个面可能跨几个 cell 高度
    merged = [[float(uniq[0]), float(uniq[0]), int(cnt[0])]]
    for u, c in zip(uniq[1:], cnt[1:]):
        if float(u) - merged[-1][1] <= cell * 1.5:
            merged[-1][1] = float(u)
            merged[-1][2] += int(c)
        else:
            merged.append([float(u), float(u), int(c)])
    best = max(merged, key=lambda m: m[2])
    return (best[0] + best[1]) / 2.0, best[2] / float(n_valid)


def _deck_mask(color: np.ndarray, depth: np.ndarray, k, normal: np.ndarray,
               opts: dict | None = None) -> tuple[np.ndarray, np.ndarray, dict]:
    """台面掩码 + 高度图 + 占位栅格信息。**颜色 / 深度双路**（规格 §4 Step 2）。

    选择器用 `_color_coverage` 跟 `WOOD_COVERAGE_MIN` 比：

      - 覆盖率 >= `WOOD_COVERAGE_MIN` -> **颜色路线**（现有路径，1~4/10_test 验过）
      - 覆盖率 <  `WOOD_COVERAGE_MIN` -> **深度路线**（不做任何颜色过滤）

    两条路都如实写进 `info['source']`，**不静默切换**。

    ⚠️ **T5d**：`_color_coverage` 的分母已从"高度带里的所有格"换成"木色格"，
    所以 `WOOD_COVERAGE_MIN`（0.40，**名字不变**）现在问的是
    "木色格落在自己那个高度层上的比例"。理由与旧定义的失效模式见
    `_color_coverage` 的 docstring。**不要改回 `÷ band`。**
    """
    empty = (np.zeros((1, 1), bool), np.full((1, 1), -1e3, np.float32),
             dict(p0=0.0, q0=0.0, origin=np.zeros(3), voxel_mm=VOXEL_MM, h_deck=None,
                  sat_min=WOOD_SAT_MIN, n_wood=0, source='color',
                  color_coverage=None))
    top, height, info = _rasterise(color, depth, k, normal, opts)
    if top is None:
        return empty
    info.update(h_deck=None, sat_min=WOOD_SAT_MIN, n_wood=0,
                source='color', color_coverage=None)

    wood = _wood_mask(top)
    info['n_wood'] = int(wood.sum())

    # ---- 可选：把台面高度的选择约束在「base_link 的绝对高度带」里 ----
    # `height` 是「沿法向、相对点云质心」的量，**本身没有绝对意义**。调用方给了
    # `camera_z_mm`（相机原点在 base_link 里的 z，mm）之后，换算关系是：
    #     z_base = height + K,  K = camera_z_mm + (info['origin'] · normal)
    # （`info['origin']` 是占位平面原点，相机系 mm —— `_rasterise` 第二趟把它挪到了
    # 点云质心，`height` 就是相对它的）。所以带内约束的边界是 `(z_lo - K, z_hi - K)`。
    # ⚠️ 少加 `origin·normal` 这一项，整个带会平移一个 origin 的投影量（实测
    # jia1 上差 23.9mm，看着不大，但换帧就会飘）。
    z_band = None
    cz = opts.get('camera_z_mm') if opts else None
    zb = opts.get('deck_z_mm') if opts else None
    if cz is not None and zb is not None:
        K = float(cz) + float(np.asarray(info['origin'], float) @ np.asarray(normal, float))
        z_band = (float(zb[0]) - K, float(zb[1]) - K)
        info['deck_z_mm'] = [float(zb[0]), float(zb[1])]

    # --- 颜色路线：先拿到 deck_h，才能问"这个面上的木色占比" ---
    deck_h, coverage = None, None
    if info['n_wood'] >= MIN_WOOD_PX:
        deck_h = _adaptive_deck_height(height, wood, z_band)
        if deck_h is not None:
            # ⚠️ `_deck_sat_min` 用的是**旧分母**（`÷ band`），与下面
            # `_color_coverage` 的**新分母**（`÷ wood`）不一致，但两者共享
            # `WOOD_COVERAGE_MIN`。**操作员 2026-09-23 裁决「先接受」** ——
            # 完整说明、为什么回归基线证明不了它、以及将来怎么统一，
            # 都写在 `_color_coverage` 的 docstring 里（别只改这一边）。
            sat_min = _deck_sat_min(height, wood,
                                       _placeholder_frame(normal, info['origin'],
                                                          max(height.shape)),
                                       deck_h)
            if sat_min != WOOD_SAT_MIN:
                # 10_test 那次的修复：门控对不上时放宽饱和度下限重来一遍
                wood = _wood_mask(top, sat_min)
                deck_h = _adaptive_deck_height(height, wood, z_band)
            info['sat_min'] = sat_min
            if deck_h is not None:
                coverage = _color_coverage(height, wood, deck_h)
    info['color_coverage'] = None if coverage is None else round(float(coverage), 4)

    if coverage is not None and coverage >= WOOD_COVERAGE_MIN:
        info['source'] = 'color'
        # ⚠️ 这里**不要**再写 `deck &= ~box`。T3 实测（`test_deck_mask_deadcode` 钉住了）：
        # `deck` 已要求 `|h - deck_h| <= DECK_BAND_MM`(40)，而货的定义是
        # `h - deck_h >= BOX_RAISE_MM`(90)，两者**交集恒为空** —— 那一行永远不会
        # 改变任何东西。留着它有两个坏处：读代码的人以为货是靠它扣掉的
        # （实际靠的是颜色门控），而且哪天有人把 `DECK_BAND_MM` 调到 >= 90，
        # 它会**突然开始生效**、静默改变行为。
        deck = wood & (np.abs(height - deck_h) <= DECK_BAND_MM)
    else:
        # --- 深度路线：不做任何颜色过滤 ---
        info['source'] = 'depth'
        deck_h, frac = _dominant_band(height)
        if deck_h is None:
            return np.zeros(height.shape, bool), height, info
        deck = (height > -1e3) & (np.abs(height - deck_h) <= DECK_BAND_MM)

    info['h_deck'] = None if deck_h is None else float(deck_h)
    # 形态学闭运算。⚠️ 两条路的掩码**形态可能不同**，但**核是共用的**（2026-09-23，T5 实测）：
    #
    # * **深度路线**：`deck` 是"每个有效格"（合成场景里整块地面 261978 格，
    #   几乎逐格相邻），闭运算的开运算腿会把 25x25 圆盘里没被覆盖的格**填满** ——
    #   实测 261978 格 -> 11 648 628 格（把 3708x5412 的栅格填掉 58%），
    #   于是任何已知尺寸矩形都能拿到 0.593 的 coverage，
    #   `COVERAGE_MIN` 直接失去意义。**这不是调参能救的**。
    #   T5c 的做法是**加一条面积判据**（`MASK_AREA_RATIO_MAX`），让"整块地面"
    #   这类糊实的掩码被面积拦下 —— 不去动这里的核。
    #
    # * **颜色路线**：⚠️ **原注释（T5 时写的）说"颜色路线的 `deck` 本身连成片，
    #   闭运算只是补小洞"—— 这句话在 5_test 上被实测证伪了（T6，2026-09-23）。**
    #   5_test 的木色格是 **13885 个连通域、最大域只有 37 格**（栅格 1px = 1mm），
    #   根本不是"连成片"，而是一张**细点阵**。闭运算把它 **18.3x** 膨胀：
    #   11753 格 -> 214924 格，其中 **198516 格（92.4%）是无深度格** ——
    #   和深度路线是**同一个机理**，只是发生在颜色路线上。
    #   为什么 1~4/10_test 没暴露：那几帧木色密（单帧 4 万格量级），点阵本身就近乎连片。
    #
    #   **本轮不修**（操作员裁决"5_test 上接受失败"）：5_test 走颜色路线时
    #   `mask_area_ratio = 0.179 < MASK_AREA_RATIO_MAX`，面积判据**不开火**，
    #   最终由 `COVERAGE_MIN` 拒（`score` 0.1696 < 0.35）。失败是干净、可解释的。
    #   **但这意味着面积判据只兜住了"糊得很大"的那种糊**；5_test 是"糊得不大但糊错了"
    #   （掩码与真值矩形 IoU 0.0589，147 块碎片，最大域与真值零交集）。
    #
    # ⚠️ 核大小**不能动**（T5c 复核）：栅格 1px=1mm 但体素 5mm，原始 `deck` 是
    # **稀疏点阵**，对真值台面的 IoU 只有 0.033；闭运算把点阵之间填实才得到台面形状
    # （IoU 0.989）。**减到核 5 就退化成点阵了，反而错。**
    #
    # ⚠️ **T9c：只在内容 bbox 附近做闭运算**（原来在整张栅格上做）。
    # 这是**严格等价**的：闭运算（= 先腐蚀后膨胀）对某格的影响半径是
    # **两倍核半径**（腐蚀会"看见"2r 外的结构，膨胀再把它送出去），
    # 所以只要把内容 bbox 向外放 `2r` 就一格不差。核 25 -> r=12 -> pad 取
    # `CLOSE_PAD_MM = 26`（比严格界 24 留 2mm 余量）。
    # 收益：栅格 1px=1mm、体素 5mm，内容只占很小一块 —— april_test7 上
    # 子图 1.30M vs 全图 11.42M，**86.9ms -> 9.9ms**。
    deck = _close_near_content(deck)
    return deck > 0, height, info


def _close_near_content(deck: np.ndarray, kernel: int = CLOSE_KERNEL,
                        pad: int = CLOSE_PAD_MM) -> np.ndarray:
    """在**内容 bbox 外扩 `pad`** 的子图上做闭运算，其余位置保持原值（0）。

    ⚠️ `pad` 必须 >= 核直径 `kernel`（见调用点的论证：影响半径 = 2 × 核半径）。
    这里默认 `CLOSE_PAD_MM = 26 >= 25`，且断言住 —— 以后有人调小核或调小 pad
    会立刻炸，而不是静默切掉边界。
    """
    if pad < kernel:
        raise ValueError(f'_close_near_content: pad({pad}) 必须 >= kernel({kernel})，'
                         f'否则会切掉闭运算的边界效应（影响半径 = 2×核半径）')
    ys, xs = np.nonzero(deck)
    if ys.size == 0:
        return deck                                    # 全零：闭运算还是全零
    r0 = max(0, int(ys.min()) - pad)
    c0 = max(0, int(xs.min()) - pad)
    r1 = min(deck.shape[0], int(ys.max()) + pad + 1)
    c1 = min(deck.shape[1], int(xs.max()) + pad + 1)
    sub = cv2.morphologyEx(deck[r0:r1, c0:c1].astype(np.uint8) * 255, cv2.MORPH_CLOSE,
                           cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel, kernel)))
    out = np.zeros(deck.shape, bool)
    out[r0:r1, c0:c1] = sub > 0
    return out


# --------------------------------------------------------------------------- #
# 图像空间掩码 -> 单应反查 -> 稠密米制掩码（主路径）
# --------------------------------------------------------------------------- #
# 2026-09-30 加。背景：`_rasterise` 是**正向撒点** —— 每个深度像素翻一格，于是
# 1200x1000mm 的托盘在 1px=1mm 的栅格里只被盖到 **6.3%**，形状统计全部失真。
# 现场帧实测：
#   * 全画布掩码 839150 格，**91% 在托盘 ROI 之外**（`close(25)` 把地面一起填了）
#   * 它的 minAreaRect 是 1568x1565 的**方形**；核 5/9/13/17/21/25 全是方形
#   * 掩码协方差主轴 34.44°，而托盘长边 83.49° —— **两个候选轴都不是托盘的边**
# 于是 `_theta_ref` 给出错 40~50° 的参考角，而 `_search_rect` 的覆盖度目标又
# 分辨不出对错：同一掩码，θ 取 1.24/44.82/83.49/134.82°，score 分别是
# 0.667/0.700/0.700/0.683 —— **几乎一样**。发布出来就是整体转 90° 的位姿。
#
# 换成图像空间就全好了：图像掩码是稠密的，回投验证 **79.6% 落在真值四边形内**。
# 但矩形要在米制里定，所以 `_image_to_raster` 用**单应反查**把图像空间的场
# （掩码、深度脊线）铺回栅格：每一格算出对应的 3D 点、投回图像去采样。实测掩码
# 填充率 **6.3% -> 70.4%**，minAreaRect 给出 1026x1170mm @ 84.50°
# （真值 1200x1000 @ 83.49°）。
#
# ⚠️ **栅格必须与 `_rasterise` 完全一致**（同 `info['origin']`/`p0`/`q0`、同
# `_basis_from_normal(normal)`、1px=1mm）。`_edge_support` 的探针换算
# 都按栅格坐标算，坐标系不一致会**静默全错**。
IMG_OPEN_KERNEL = 25          # 图像空间开运算核（像素）
IMG_MIN_COMPONENT_PX = 3000   # 去小块：连通域小于这么多像素就丢
DENSE_MIN_FILL = 0.25         # 守卫：稠密掩码填充率低于它 -> 回退旧路径（太碎）
DENSE_MAX_FILL = 1.50         # 守卫：填充率高于它 -> 回退（掩码把地面也吃进来了）
# 守卫：`minAreaRect` 量出的长短比与 `target` 的相对差超过这个比例 -> 回退 `_theta_ref`。
# ⚠️ **0.25 -> 0.10**（2026-09-30 实测）。L 形掩码（遮 2 条相邻边）的主轴没有意义：
# 量出 1016x1000（应有的 1207x1018），偏 15.3%，主轴因此**转了 90°**，θ 一用就把位姿
# 带到 184mm 外。0.25 太松、放它过去了。实测像矩形的场景只偏 1~6%（真机 5.8%、
# 11-04-26 7.6%、遮 1 边 1.4%），L 形偏 15.3%、遮 3 边 15.6% —— 0.10 正好落在中间。
DENSE_ASPECT_TOL = 0.10
RIDGE_MIN_MM = 20.0           # 深度脊线阈值：相邻像素 3D 距离大于它才算"边"
RIDGE_RESIDUAL_MAX_PX = 2.5   # 边吸附质量门：拟合残差中位超过它就不采纳该边
RIDGE_MIN_POINTS = 8          # 边吸附质量门：脊点少于它就不采纳该边


def _image_wood_mask(color: np.ndarray, depth: np.ndarray, *,
                     open_kernel: int = IMG_OPEN_KERNEL,
                     min_component_px: int = IMG_MIN_COMPONENT_PX) -> np.ndarray:
    """**图像空间**的木色/浅色掩码 + 开运算 + 去小块。bool，与 `color` 同尺寸。

    与 `_wood_mask` 分工不同，**两个都要留着**：
      * `_wood_mask` 作用在 `_rasterise` 出来的**栅格化顶视图** `top` 上，是
        **旧路径**（`_deck_mask` -> `_theta_ref` -> `_search_rect`）的依赖；
      * 本函数作用在**原始彩色图**上，是主路径的输入。

    ⚠️ **形态学只能在这里做。** 栅格空间做不了：那里的掩码是 5mm 体素的**稀疏
    点阵**（托盘范围内 6.3% 覆盖率），开运算会把它整个开没 —— 实测（开 / 开闭 /
    开闭+去小块）三种都直接 `no_deck`。图像空间是稠密的，开运算(25) 去掉的正是
    漏到地面/护栏的那部分，即操作员在 `mask_morph_compare.png` 里看到的
    "橙色带子 vs 绿色/紫色"。

    实测（现场帧）：原始 115456px -> 开运算 88277 -> 去小块 85525。
    """
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    wood = cv2.inRange(hsv, np.asarray(WOOD_HSV[0], np.uint8),
                       np.asarray(WOOD_HSV[1], np.uint8)) > 0
    blue = cv2.inRange(hsv, np.asarray(BLUE_HSV[0], np.uint8),
                       np.asarray(BLUE_HSV[1], np.uint8)) > 0
    yellow = cv2.inRange(hsv, np.asarray(YELLOW_HSV[0], np.uint8),
                         np.asarray(YELLOW_HSV[1], np.uint8)) > 0
    pale = (hsv[:, :, 1] <= PALE_HSV[0]) & (hsv[:, :, 2] >= PALE_HSV[1])
    m = (((wood & (hsv[:, :, 1] >= WOOD_SAT_MIN)) | pale)
         & ~blue & ~yellow & (depth > 0)).astype(np.uint8)
    if open_kernel >= 3:
        m = cv2.morphologyEx(
            m, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_kernel, open_kernel)))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    out = np.zeros_like(m)
    for i in range(1, n):
        if int(stats[i, cv2.CC_STAT_AREA]) >= min_component_px:
            out[labels == i] = 1
    return out > 0


def _image_to_raster(field: np.ndarray, shape: tuple[int, int], k,
                     normal: np.ndarray, info: dict, h_mm: float, *,
                     chunk_rows: int = 256) -> np.ndarray:
    """把**图像空间**的场经单应反查铺到占位栅格上。float32，形状 `shape`。

    逐格算它对应的 3D 点、投回图像、采样 `field`；投到图像外的格填 0。

    ⚠️ **方向与 `_rasterise` 相反，这正是稠密化的来源。** `_rasterise` 是
    "像素 -> 格"（撒点，一个像素只占一格，于是 1.2M mm² 的矩形只被盖到 6.3%）；
    本函数是"格 -> 像素"（反查，把整个面填满，实测 70.4%）。同一个托盘、同一张
    栅格，只是方向反过来。

    ⚠️⚠️ **栅格点必须落在「台面」平面上，即 `info['origin'] + h_mm * normal` ——
    这就是 `h_mm` 存在的唯一理由，绝不能省。** 栅格的 (col, row) 是
    `(a - p0, b - q0)`，而 (a, b) 是**沿法向的正交投影**：不同深度上"投进同一个
    像素"的那些点，(a, b) 差得很远。若把栅格建在点云质心平面上而不是台面上，
    反查出来的区域就不是托盘的 (a, b) 足迹。
      实测：合成场景（地面沿法向下沉 900mm 并主导点云，质心离台面很远）上
      `dense_fill` 从应有的 ~0.7 变成 **2.74**、`minAreaRect` 量出
      **1999x1684mm**（标称 1200x1000），整帧被判 `too_few_edges`。
      真机 bag 上 `h_deck` 只有 **3.29mm**（台面几乎就在质心平面上），所以这个
      错误在那批数据上**看不出来** —— 别拿它当"已经验证过"。

    ⚠️ 分块是为了内存：全栅格 1758x2572 一次算要 4.5M x 3 的 float64（108MB），
    分块后峰值只有 `chunk_rows x rw`。
    """
    rh, rw = int(shape[0]), int(shape[1])
    ih, iw = field.shape
    u_b, v_b = _basis_from_normal(normal)
    org = np.asarray(info['origin'], float) + float(h_mm) * np.asarray(normal, float)
    p0, q0 = float(info['p0']), float(info['q0'])
    xs = np.arange(rw, dtype=np.float64) + p0
    out = np.zeros((rh, rw), np.float32)
    for r0 in range(0, rh, chunk_rows):
        r1 = min(rh, r0 + chunk_rows)
        ys = (np.arange(r0, r1, dtype=np.float64) + q0)[:, None]
        pts = (org[None, None, :]
               + xs[None, :, None] * u_b[None, None, :]
               + ys[:, :, None] * v_b[None, None, :])
        z = pts[..., 2]
        ok = z > 1.0
        u = k.fx * pts[..., 0] / np.where(ok, z, 1.0) + k.cx
        v = k.fy * pts[..., 1] / np.where(ok, z, 1.0) + k.cy
        ok &= (u >= 0) & (u < iw) & (v >= 0) & (v < ih)
        ui = np.clip(np.round(u), 0, iw - 1).astype(np.intp)
        vi = np.clip(np.round(v), 0, ih - 1).astype(np.intp)
        out[r0:r1] = np.where(ok, field[vi, ui], 0.0).astype(np.float32)
    return out


def _rect_from_dense(mask: np.ndarray, long_mm: float, short_mm: float):
    """稠密米制掩码 -> 已知尺寸的矩形四角。返回 `(rc, info)`；掩码太小给 `(None, None)`。

    `rc` 是 `(4, 2)` 的**栅格 (col, row)**，顺序与 `_search_rect` 的 `rc` 同构：
    `rc[0] -> rc[1]` 沿**长边**、`rc[1] -> rc[2]` 沿**短边** —— 下游
    `detect_pallet_frame` 正是这么读的（`d_long = plane[1] - plane[0]`）。

    ⚠️ **长轴由 `minAreaRect` 自己量出的尺寸定，不问 `long_side_parallel`。**
    这就是 90° 歧义消失的地方：`minAreaRect` 已经量出哪条边长（实测 1026 vs
    1170），把较长的那条配给 `long_mm` 就行。旧路径要靠"与画面近平行的边是不是
    长边"去猜象限，才有那个"给错就偏 90° 而 score 照样很高"的坑。
    """
    ys, xs = np.nonzero(mask)
    if xs.size < MIN_WOOD_PX:
        return None, None
    (cx, cy), (w, h), ang = cv2.minAreaRect(
        np.column_stack([xs, ys]).astype(np.float32))
    long_ang = np.radians(ang if w >= h else ang + 90.0)
    a1 = np.array([np.cos(long_ang), np.sin(long_ang)], float)      # 长边方向
    a2 = np.array([-a1[1], a1[0]], float)                            # 短边方向
    c = np.array([cx, cy], float)
    rc = np.array([c - a1 * long_mm / 2 - a2 * short_mm / 2,
                   c + a1 * long_mm / 2 - a2 * short_mm / 2,
                   c + a1 * long_mm / 2 + a2 * short_mm / 2,
                   c - a1 * long_mm / 2 + a2 * short_mm / 2])
    return rc, dict(center=c, long_axis=a1, short_axis=a2,
                    measured_mm=(float(max(w, h)), float(min(w, h))),
                    angle_deg=float(np.degrees(long_ang) % 180.0),
                    fill=float(mask.sum()) / max(long_mm * short_mm, 1.0))


def _snap_rect_to_ridge(rc: np.ndarray, ridge: np.ndarray, *,
                        min_mm: float = RIDGE_MIN_MM,
                        residual_max: float = RIDGE_RESIDUAL_MAX_PX,
                        min_points: int = RIDGE_MIN_POINTS,
                        half_mm: int = 45, n_samp: int = 30):
    """把矩形四条边吸附到深度脊线上。返回 `(rc_new, report)`。

    `ridge` 是**栅格坐标**下的"相邻像素 3D 距离"（1px = 1mm），由调用方用
    `_image_to_raster` 铺进来 —— 这样垂直步长就是米制，与矩形的边同一套量纲。

    质量门：一条边只有 **拟合残差中位 < `residual_max` 且脊点 >= `min_points`**
    才被采纳；其余边保留原位。

    ⚠️ **实测：在旧基线上有效，在主路径上是负收益 —— 所以默认不启用**
    （`opts['snap_edges']`，默认 `False`）。两次实测：
      * **图像空间 minAreaRect 基线**（33px，本身偏得厉害）：采纳 3 条
        （残差 0.68 / 0.62 / 1.66px）、弃 1 条（11.0px），**33px -> 19px**。
      * **主路径（稠密掩码 + minAreaRect，13px）**：8 帧真值实测
        **13px -> 14px**，逐帧 5 帧变差 / 1 帧改善 / 2 帧持平，而且角3 系统性变坏
        （27~28 -> 27~33px）。原因不难理解：主路径已经比脊线能给的位置更准了，
        再往上贴只是把噪声引进来。

    所以它是**留着给"主路径退化、而脊线恰好干净"的场景**的备用手段，不是默认步骤。

    ⚠️ **θ 只在两条长边都被采纳时才改。** 一条长边移动 + 一条对照边没动，若还让 θ
    跟着转，等于"用一条边的证据转整个矩形"，而没动的那些边的偏移量在新基下会被
    平移 —— 实测现场帧上这一条就把 12px 顶到 14px。两条长边都采纳时 θ 才是被
    两侧同时支持的。短边不参与定 θ（正交性已经把它们的朝向钉死了）。

    ⚠️ **四条线各自拟合会得到一般四边形，而下游 `_orient` 用
    `_corners_mm(frame, W, H)` 构造的是矩形** —— 四边形喂不进去。所以这里把结果
    **重新约束成矩形**：先定 θ，再把四条边各自的偏移量填回去。

    ⚠️ **只动位置与方向，尺寸语义不变**：`detect_pallet_frame` 后面拿 `rc[0]` 当
    矩形原点、用标称 `long_mm`/`short_mm` 重建四角（在 `_orient` 里），所以真正
    传下去的是位置与 θ。
    """
    c0 = rc.mean(axis=0)
    a1_0 = rc[1] - rc[0]
    a1_0 = a1_0 / max(float(np.linalg.norm(a1_0)), 1e-9)      # 长边方向
    # 边 i：rc[i] -> rc[i+1]。0/2 是长边（法向沿短轴），1/3 是短边（法向沿长轴）
    fitted = [None] * 4
    report: list = []
    for i in range(4):
        sv, ev = rc[i], rc[(i + 1) % 4]
        u = ev - sv
        ln = float(np.linalg.norm(u))
        if ln < 20.0:
            report.append({'edge': i, 'accepted': False, 'why': 'too_short'})
            continue
        u = u / ln
        nv = np.array([-u[1], u[0]], float)
        if float(nv @ (c0 - (sv + ev) / 2.0)) < 0:
            nv = -nv                                           # 指向矩形内部
        wins = np.arange(-half_mm, half_mm + 1, dtype=np.float64)
        pts = []
        for t in np.linspace(0.08, 0.92, n_samp):
            base = sv + t * ln * u
            prof = np.zeros(wins.size, float)
            for j, s in enumerate(wins):
                pt = base + s * nv
                col, row = int(round(pt[0])), int(round(pt[1]))
                if 0 <= row < ridge.shape[0] and 0 <= col < ridge.shape[1]:
                    prof[j] = ridge[row, col]
            if float(prof.max()) < min_mm:
                continue                                       # 这一列没有脊
            pk = int(np.argmax(prof))
            off = 0.0
            if 0 < pk < prof.size - 1:                         # 抛物线亚像素
                y0, y1, y2 = prof[pk - 1], prof[pk], prof[pk + 1]
                den = float(y0 - 2.0 * y1 + y2)
                if abs(den) > 1e-9:
                    off = float(np.clip(0.5 * (y0 - y2) / den, -1.0, 1.0))
            pts.append(base + (wins[pk] + off) * nv)
        if len(pts) < min_points:
            report.append({'edge': i, 'accepted': False, 'why': 'few_points',
                           'n': len(pts)})
            continue
        pts = np.asarray(pts, float)
        X = pts.copy()                                         # 迭代重加权直线拟合
        for _ in range(4):
            mu = X.mean(axis=0)
            _, _, vt = np.linalg.svd(X - mu, full_matrices=False)
            d = vt[0]
            res = np.abs((X - mu) @ np.array([-d[1], d[0]], float))
            sc = float(np.median(res)) * 1.4826
            if sc < 1e-6:
                break
            X = X[res <= 2.5 * max(sc, 0.5)]
            if len(X) < 6:
                break
        mu = X.mean(axis=0)
        _, _, vt = np.linalg.svd(X - mu, full_matrices=False)
        d = vt[0]
        resid = float(np.median(np.abs((pts - mu) @ np.array([-d[1], d[0]], float))))
        if resid > residual_max:
            report.append({'edge': i, 'accepted': False, 'why': 'residual',
                           'residual_px': round(resid, 2), 'n': len(pts)})
            continue
        if float(d @ u) < 0:
            d = -d
        fitted[i] = (mu, d)
        report.append({'edge': i, 'accepted': True, 'residual_px': round(resid, 2),
                       'n': len(pts)})

    longs = [fitted[i] for i in (0, 2)]
    if longs[0] is not None and longs[1] is not None:       # 两条长边都采纳才动 θ
        acc = np.zeros(2, float)
        for _, d in longs:
            acc += d if float(d @ a1_0) >= 0 else -d
        nrm = float(np.linalg.norm(acc))
        a1 = acc / nrm if nrm > 1e-6 else a1_0
    else:
        a1 = a1_0

    def _off(i: int) -> float:
        """边 i 在新基下的偏移（相对原中心 `c0`）；没采纳的边用原位。"""
        if fitted[i] is not None:
            mu = fitted[i][0]
        else:
            mu = (rc[i] + rc[(i + 1) % 4]) / 2.0
        axis = np.array([-a1[1], a1[0]], float) if i in (0, 2) else a1
        return float((mu - c0) @ axis)

    b1, b2 = np.array([-a1[1], a1[0]], float), a1
    o = [_off(i) for i in range(4)]
    s_lo, s_hi = (o[0], o[2]) if o[0] <= o[2] else (o[2], o[0])   # 长边 -> 短边尺寸
    l_lo, l_hi = (o[1], o[3]) if o[1] <= o[3] else (o[3], o[1])   # 短边 -> 长边尺寸
    long_new, short_new = l_hi - l_lo, s_hi - s_lo
    if not (200.0 < long_new < 4000.0 and 200.0 < short_new < 4000.0):
        report.append({'accepted': False, 'why': 'degenerate_size',
                       'long_mm': round(float(long_new), 1),
                       'short_mm': round(float(short_new), 1)})
        return rc, report
    center = c0 + b2 * ((l_hi + l_lo) / 2.0) + b1 * ((s_hi + s_lo) / 2.0)
    rc_new = np.array([center - b2 * long_new / 2 - b1 * short_new / 2,
                       center + b2 * long_new / 2 - b1 * short_new / 2,
                       center + b2 * long_new / 2 + b1 * short_new / 2,
                       center - b2 * long_new / 2 + b1 * short_new / 2])
    return rc_new, report


def _depth_ridge_mm(depth: np.ndarray, k) -> np.ndarray:
    """相邻像素的 **3D 距离**（mm）取大值 —— 深度图的边缘强度。

    ⚠️ 用 3D 距离而不是 `|dZ|`：相机是斜视的，一个**平行于光轴的竖直面**在同一
    深度上相邻像素的 `|dZ|` 是 0，可它明明是个台阶。现场帧实测 `|dZ|` 在托盘
    左边（真值边3）上几乎为零，而 3D 距离在那里给出 20.0mm 的脊线、位置正好落在
    边上。
    """
    cloud = _backproject_cloud(depth, k)
    h, w = depth.shape
    full = np.zeros((h, w, 3), np.float32)
    full[depth > 0] = cloud
    valid = depth > 0
    g = np.zeros((h, w), np.float32)
    for dv, du in ((0, 1), (1, 0), (1, 1), (1, -1)):
        sv = np.roll(valid, (dv, du), (0, 1))
        d = np.linalg.norm(full - np.roll(full, (dv, du), (0, 1)), axis=2)
        g = np.maximum(g, np.where(valid & sv, d, 0.0))
    return g


def _theta_from_image(color: np.ndarray, depth: np.ndarray, raster_shape: tuple[int, int],
                      k, normal: np.ndarray, info: dict,
                      long_mm: float, short_mm: float, long_side_parallel: bool,
                      opts: dict):
    # `long_side_parallel` **不参与计算**，只用于把"声明 vs 实测"写进 meta（见下）。
    """主路径入口：图像掩码 -> 稠密米制掩码 -> `minAreaRect` 的**长边方向**。

    返回 `(theta | None, meta)`。`theta` 是 `(E1, E2)` 基下的长边方向（弧度），
    与 `_theta_ref` 的返回值同构 —— 调用方把它喂给 `_search_rect`，其它什么都不用改。

    ⚠️ **只换 θ，不换搜索**（2026-09-30 实测后定的）。原先想让 `minAreaRect` 的
    矩形直接取代 `_search_rect`，8 帧真值上能到 12~14px，但：
      * 合成场景"遮 2 条相邻边"（规格 §5.1 要求输出位姿）会退化成 L 形掩码，
        `minAreaRect` 量出 1016x1000（应有的 1207x1018），整个矩形转错；
      * 而**只把 θ 交给旧搜索**，同一个 L 形场景里有 4/5 个用例反而正确
        （dE1/dE2 都在 8mm 内），真机 bag 上更是 **5px**（纯 minAreaRect 版 13px）。
    道理是：**θ 是形状统计量（掩码完整时才可信），位置是覆盖度搜索量**（搜索
    对遮挡本来就有韧性）。所以两者各取所长，而不是用一个取代另一个。

    ### `long_side_parallel`：只用来**复核**，不参与计算

    ⚠️ **`minAreaRect` 已经量出哪条边长了**（实测 1170 vs 1026），"哪条是长边"
    不需要先验 —— 这正是它比 `_theta_ref` 强的地方（后者拿主轴猜，给错就偏 90°）。
    所以 `long_side_parallel` 在这里**一次都不参与 θ、也不选轴**。

    **它只作为独立来源做一次交叉核对**（把结果写进 `meta`，不影响返回值）：

    | 声明 | 长边应指向 |
    |---|---|
    | `False`（短边平行）| 与画面近**垂直** |
    | `True`（长边平行）| 与画面近**平行** |

    实测两种都真实出现过：现场帧的**短边**横着（`True`），而 09-29 那批的**长边**
    横着（`False`）—— 所以这个朝向**不能写死**，只能按帧量。不一致时记进
    `diag['lsp_measured_parallel']`，由调用方决定要不要理会。

    **守卫**（不过就返回 `None`，调用方回退 `_theta_ref`）：稠密掩码的
    `minAreaRect` 长短比必须与 `target` 的接近 —— 形状不像矩形时主轴没有意义。
    实测：像矩形的场景偏 1~6%（真机 5.8%、11-04-26 7.6%），L 形偏 15.3%，
    遮 3 边偏 15.6%。阈值取 `DENSE_ASPECT_TOL = 0.10` 落在中间。
    """
    meta: dict = {'why': None, 'fill': None, 'measured_mm': None,
                  'angle_deg': None}
    img_mask = _image_wood_mask(color, depth)
    if int(img_mask.sum()) < IMG_MIN_COMPONENT_PX:
        meta['why'] = 'image_mask_small'
        return None, meta
    # ⚠️ `h_deck` 是栅格的平面锚点，不是可选的（见 `_image_to_raster` 的论证）。
    # 走到这里 `info['h_deck']` 一定非空 —— 上面 `no_deck` 那道闸门已经拦过。
    dense = _image_to_raster(img_mask.astype(np.float32), raster_shape,
                             k, normal, info, info['h_deck']) > 0.5
    fill = float(dense.sum()) / max(long_mm * short_mm, 1.0)
    meta['fill'] = round(fill, 4)
    if not (DENSE_MIN_FILL <= fill <= DENSE_MAX_FILL):
        meta['why'] = 'dense_thin' if fill < DENSE_MIN_FILL else 'dense_fat'
        return None, meta
    _, ri = _rect_from_dense(dense, long_mm, short_mm)
    if ri is None:
        meta['why'] = 'dense_no_rect'
        return None, meta
    meas_long, meas_short = ri['measured_mm']
    meta['measured_mm'] = [round(float(meas_long), 1), round(float(meas_short), 1)]
    aspect = meas_long / max(meas_short, 1e-9)
    want = long_mm / max(short_mm, 1e-9)
    if abs(aspect - want) / want > DENSE_ASPECT_TOL:
        meta['why'] = 'dense_aspect'
        return None, meta

    # θ 的**符号**：长边应指向画面右（x 分量 > 0）—— 一条直线差 180° 是同一个
    # 方向，而搜索窗（±45°）装不下两个，所以要选一个符号。**轴不选** ——
    # `minAreaRect` 已经量出哪条长了。
    axis = np.asarray(ri['long_axis'], float)
    if float(axis[0]) < 0.0:
        axis = -axis
    deg = float(np.degrees(np.arctan2(float(axis[1]), float(axis[0]))))
    meta['angle_deg'] = round(deg % 180.0, 2)
    return float(np.radians(deg)), meta


# --------------------------------------------------------------------------- #
# 参考角 θ_ref
# --------------------------------------------------------------------------- #
def _long_short_angle_diff(frame: dict, normal: np.ndarray,
                           theta_deg: float) -> float:
    """帧的**长边方向**与 `theta_deg`（图像空间量出的长边方向）的夹角，折到 [0, 90]。

    **不依赖真值的长短边哨兵。** `_orient` 的 docstring 记过一次静默失效：
    "检测自己算出的位姿是对的，但 `E1`/`W` 这一对是错配的 —— 任何'沿 E1 走 W 毫米'
    的使用者都会走到另一个方向上去"。那一类错误的**指纹就是差 90°**，而这里拿
    图像空间量出的 θ（与帧的构造完全独立）当参照，一比就出来。差得远 = 长短边反向。

    ⚠️ **帧的长边是哪条要看 `W`/`H`，不能想当然取 `E1`** —— `_orient` 会把 `W`/`H`
    跟着 `E2_new` 落在哪条物理边上走（2026-09-29 的修复），所以 `E1` 完全可能是短边。
    """
    u_b, v_b = _basis_from_normal(normal)
    long_vec = frame['E1'] if float(frame['W']) >= float(frame['H']) else frame['E2']
    deg = float(np.degrees(np.arctan2(float(long_vec @ v_b),
                                      float(long_vec @ u_b))))
    d = abs(deg % 180.0 - float(theta_deg) % 180.0) % 180.0
    return float(min(d, 180.0 - d))


def _project_axis(frame: dict, vec: np.ndarray, k) -> np.ndarray | None:
    """把平面内一个方向投到图像平面，返回**单位**投影方向（du, dv）。

    取该方向**两端各 500mm**的点投影后相减。相机系里 +x 是图像右、+y 是图像下，
    所以 (du, dv) 与图像坐标同向。

    ⚠️ **这里原先写的是 `p0 = o + 0.0 * vec`（2026-09-30 修）。** 与 docstring 的
    "两端各 500mm" 不符，而且后果是静默的：`o` 若为 `np.zeros(3)`（旧调用方
    `detect_pallet_frame` 传的就是相机光心），`p0` 正好落在光心上、`_project_px`
    把 z 夹到 1e-6 后把它投到主点，而 `p1 = o + 500*vec` 因为 `vec` 的 z 分量是负的
    （平面法向 ≈ −z，平面内的基向量几乎与光轴垂直）**落在相机后方**、投影整个翻掉。
    实测 E1 的方向因此从"画面右"变成 `[-1.0000, 0.0006]`（画面左）。

    ⚠️ **它没造成 90° 错误，但那是运气**：唯一的使用者 `_theta_ref` 对角度取了
    `abs()` 又折到 `[0, 90)`，方向的正负号被吃掉了 —— 把它修对之后 `angs` 从
    [34.4, 55.6] 变成 [29.07, 51.93]，**顺序不变、θ_ref 一模一样**。修它是因为
    它是错的，而且换个场景（主轴接近 45°、两条轴夹角接近相等）就可能翻掉
    `i_par`。新的主路径不走这里。
    """
    o = np.asarray(frame['origin'], float)
    p0 = o - 500.0 * np.asarray(vec, float)
    p1 = o + 500.0 * np.asarray(vec, float)
    a, b = _project_px(np.stack([p0, p1]), k)
    d = b - a
    n = float(np.hypot(*d))
    if n < 1e-6:
        return None
    return d / n


def _theta_ref(mask: np.ndarray, frame: dict, k,
               long_side_parallel: bool) -> float | None:
    """参考角：掩码主轴 + 先验选象限（规格 §4 Step 4）。

    步骤：
      1. 对掩码取二阶矩得**主轴**（两条，相差 90°）；
      2. 把两条主轴投到图像平面，各与**图像水平方向** (+u) 求夹角；
      3. 用 `long_side_parallel` 选：它说"与画面近平行的边是短边"，就选夹角更小的
         那条主轴当"平行边"，另一条（更接近垂直）就是**长边**方向。

    返回的是**长边方向**在 (frame['E1'], frame['E2']) 基上的角度。
    """
    ys, xs = np.nonzero(mask)
    if xs.size < MIN_WOOD_PX:
        return None
    cu, cv = float(xs.mean()), float(ys.mean())
    # 协方差主轴（栅格坐标 u/v，1px = 1mm）
    cov = np.cov(np.stack([xs - cu, ys - cv]).astype(np.float64))
    w, vecs = np.linalg.eigh(cov)
    if w[1] <= 1e-9 or w[0] / w[1] > 0.98:
        return None                       # 近乎各向同性，主轴没意义
    # eigh 按特征值升序：vecs[:, 1] 是主轴（长轴），vecs[:, 0] 是次轴
    axes = [vecs[:, 1], vecs[:, 0]]       # 长轴, 次轴（栅格坐标系）

    # 栅格 (u, v) -> 平面内 (a, b) -> 相机系方向
    def to_cam(axis_uv):
        return axis_uv[0] * frame['E1'] + axis_uv[1] * frame['E2']

    angs = []
    for axis in axes:
        proj = _project_axis(frame, to_cam(axis), k)
        if proj is None:
            return None
        angs.append(abs(np.degrees(np.arctan2(proj[1], proj[0]))))
    # angs[i] 是与图像水平方向的夹角，取 [0, 90)
    angs = [min(a, 180.0 - a) for a in angs]
    # "与画面近平行的边" = 夹角更小的那条
    i_par = 0 if angs[0] <= angs[1] else 1
    i_long = 1 - i_par if not long_side_parallel else i_par
    # 长边方向在占位基上的角度。注意 `to_cam` 用的是栅格轴，栅格 u 沿 E1、v 沿 E2，
    # 所以可以直接读出分量。
    long_uv = axes[i_long]
    return float(np.arctan2(long_uv[1], long_uv[0]))


# --------------------------------------------------------------------------- #
# 旋转矩形搜索
# --------------------------------------------------------------------------- #
def _integral(mask: np.ndarray) -> np.ndarray:
    """积分图，(H+1, W+1)，`_rect_sum` 用。

    ⚠️ 用 `cv2.integral`（C++，**0.023s**）而不是 `np.cumsum`（**0.335s**，
    实测 6565² 画布）—— 慢 14 倍，是 T5b 之前单帧 51s 的大头之一。
    输出是 **int32**：掩码元素只有 0/1，全图之和 <= 元素总数（本仓库 43M），
    远在 int32 范围内，累加**精确无舍入**（`np.cumsum` 的 float64 也是精确的，
    两者逐位相同）。`_rect_sum` 已经 `float(...)` 出来，调用方不受影响。
    """
    return cv2.integral(mask.astype(np.uint8))


def _rect_sum(ii: np.ndarray, x0: int, y0: int, w: int, h: int) -> float:
    """积分图上 O(1) 求 [x0, x0+w) x [y0, y0+h) 的和。"""
    return float(ii[y0 + h, x0 + w] - ii[y0, x0 + w] - ii[y0 + h, x0] + ii[y0, x0])


def _rotated(mask: np.ndarray, theta: float) -> tuple[np.ndarray, np.ndarray, int]:
    """把掩码绕原点旋转 -θ（栅格 mm 坐标）。

    返回 `(rot, M, big)`：`rot` 是旋转后的掩码，`M` 是栅格 -> 旋转坐标的 2x3 矩阵，
    `big` 是旋转后栅格的边长。旋转后矩形变成轴对齐的 `W_mm x H_mm` 框，
    平移搜索就能用积分图做。
    """
    h, w = mask.shape
    c, s = float(np.cos(theta)), float(np.sin(theta))
    cx, cy = w / 2.0, h / 2.0
    big = int(np.ceil(np.hypot(w, h))) + 4
    # ⚠️ **绕画布中心转，不是绕输入栅格中心转**（T5b 修的既有缺陷）。
    # 绕输入栅格中心转、画布却是 `[0, big]` 时，画布相对内容整体偏了
    # `(big/2 - cx, big/2 - cy)`；**非正方形栅格**上内容会被转出画布**静默裁掉**
    # （实测 900x700 栅格、内容偏左上、θ=17.5° 时丢了 11.5% 的非零格；
    # 3708x5412 的真实栅格上内容小且居中，所以一直没暴露）。
    # 画布中心转是严格更正确的做法，且与 `_rotated_crop` 的推导一致。
    M = np.array([[c, s, big / 2.0 - c * cx - s * cy],
                  [-s, c, big / 2.0 + s * cx - c * cy]], np.float64)
    rot = cv2.warpAffine(mask.astype(np.uint8) * 255, M, (big, big),
                         flags=cv2.INTER_NEAREST)
    return rot > 0, M, big


def _content_bbox(mask: np.ndarray, margin: int = 2
                  ) -> tuple[int, int, int, int] | None:
    """掩码非零内容的**轴对齐** bbox，`(r0, c0, r1, c1)`（**左闭右开**）。全零返回 None。

    ⚠️ **T9：这里不再按 θ 旋转**（T5b 那一版是按 θ 旋转后再取 bbox，已废弃）。
    理由不是"省一点时间"，而是**原来那一版在大半的角上根本裁不动**：

      T5b 的出发点是"栅格上的内容是一大块正方形，按原图 bbox 裁等于没裁"。
      但它换成"按旋转后的框裁"之后，当 θ 转到离内容主轴 45° 附近时，
      **旋转后的外接框本身就退化成整张栅格的对角线** —— 于是
      `_rotated_crop` 的兜底 `if big_w >= big or big_h >= big` 触发，
      **整段退回不裁剪**。实测（5_test / april_test7 两帧，各 46 个粗搜角）：

        | 帧 | 退回全画布的角度数 | 退回时的画布 | 裁剪成功时的画布 |
        |---|---|---|---|
        | 5_test | **19/46** | 3945² = 15.6M | 6~7M |
        | april_test7 | **31/46** | 5388² = 29.0M | — |

      即：T5b 的优化在一半的角上失效，而且是**静默**失效（结果仍正确，只是慢）。

    ⚠️ **为什么轴对齐的紧 bbox 也是严格的**：裁剪图只要**装得下全部内容**，
    `_rotated_crop` 的严格性论证就成立 —— 那条论证只用到"裁剪图 ⊇ 内容"这一点
    （旋转后的外接框是内容旋转后外接框的**超集**）。轴对齐紧 bbox 显然 ⊇ 内容，
    而且是**与 θ 无关**的：算一次，46 + 33 个角共用，不会因为 θ 而退化。

    ⚠️ **仍然会有帧裁不动**：内容本身细长、已经占满栅格对角线时（april_test7：
    内容 1635x4456，栅格 2359x4839，内容对角 4746 ≈ 栅格对角 5383），
    旋转后的窗口装不进 `big`，兜底照旧触发。这是**几何上无法避免**的 ——
    搜索矩形 1200x1000 加上去之后，窗口本来就要 ~5950 > 5388。
    那种帧的粗搜提速只能靠别的办法，不靠裁剪。

    实测收益见 `_rotated_crop` 的 docstring（T9 段）。
    """
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        return None
    return (max(0, int(ys.min()) - margin), max(0, int(xs.min()) - margin),
            min(mask.shape[0], int(ys.max()) + 1 + margin),
            min(mask.shape[1], int(xs.max()) + 1 + margin))


def _rotated_crop(mask: np.ndarray, theta: float, w_mm: float, h_mm: float,
                  step: int, crop: bool = True, margin: int = 2,
                  bbox: tuple[int, int, int, int] | None = None):
    """裁剪 + 旋转：返回 `(rot, M_warp, (big_h, big_w), M_out)`。

    ⚠️ **为什么要裁剪**：`_rotated` 把画布开成**整张栅格的对角线**
    （`ceil(hypot(H, W)) + 4`）。本仓库实测栅格 3708x5412 -> 画布 6565² = **43.1M
    元素**，而掩码的非零内容只占 1013x1204（全图的 1/16）。粗搜 46 + 精搜 33 =
    79 次搜索，每次都要 warp 一遍 + 建 345MB 的 float64 积分图，**单帧 47.5s**，
    端到端跑不完。绝大多数计算花在空白上。

    ⚠️ **严格性论证**（不是"实测这次没差"）：窗口取的是"**框内含有内容的矩形**
    的左上角可能出现的全部位置"，推导如下。
    设内容在**旋转后**的外接框为 `[X0, X1] x [Y0, Y1]`（由裁剪矩形四角解析算出，
    是旋转后非零格的**超集**，最多差 1px 取整）。把矩形按**框内有没有内容**分两类：

      - **甲类：框内没有内容格**，和恒为 0。这种位移不可能是最优解
        （只要内容能装进矩形 —— 掩码非零、`MIN_WOOD_PX` 已把关 —— 甲类的和 0
        就小于乙类的正和；若内容连矩形都装不下，两者都是 0，`np.argmax` 取
        扫描序里的**第一个**，仍是同一个格点）。
      - **乙类：框内至少含一个内容格 `p`**，则左上角必然落在
        `x0 ∈ [p_x - w, p_x] ⊆ [X0-w-1, X1+1]`、`y0 ∈ [Y0-h-1, Y1+1]`。
        这就是**内容的那一小块区域**，它**完全落在裁剪窗口内**。

    两类的最优解都不需要"框伸到内容外很远"的位置：乙类的位置由内容格直接钉住，
    甲类由扫描序兜住。裁剪窗口就是 `[X0-w-margin, X1+step+margin] x [Y0-h-margin,
    Y1+step+margin]`（比内容那一块多出整整一个矩形的尺寸），所以裁剪后
    `np.argmax` 选中的是**同一个格点**。

    ⚠️ **实测的偏差上限**（30 组「内容摆放 x θ0 x 步长」的裁剪/不裁剪对比，
    见 `debug/3test/detect_synthetic_test.py::test_search_crop_equivalence`）：
    `theta` **逐位相等**；框角差 **<= 1.7e-13 mm**；
    `score` 的**框内和最多差 1 个格**（`|Δscore| * w * h <= 1`）。
    最后这一条不是"裁剪引起的"，来源是 `cv2.warpAffine` 是**定点**实现 ——
    它对**源坐标**做定点舍入，所以矩阵平移一个整数**不严格保距**
    （实测 1145² 画布上平移后有 ~1e-4 比例的采样点落到相邻源像素）；
    像素边界上的采样点于是取到不同邻居。**同一个矩阵、只把画布开小一点也一样**
    （实测），所以它不是裁剪引入的。换成 `WARP_INVERSE_MAP`（对目标坐标取整，
    天然平移不变）实测位精确相等，但要重写整条旋转路径，为 1/60000 个格不值当。
    1 个格的差异远小于 `COVERAGE_MIN` 的判据间隔，不影响任何决定。

    窗口原点再向下取整到 `step` 的整数倍，保证抽样格点与不裁剪时**同相**
    （`_best_shift` 用 `S[::step, ::step]` 抽样，相位差一格就会静默挪结果）。

    ⚠️ 还有一条**保守兜底**：若窗口算出来比不裁剪的画布还大（内容很小而搜索矩形
    很大、栅格本身又不大时），直接退回不裁剪 —— 否则"优化"会变成负优化。
    两条路结果相同，退回只是省事。

    实测收益（合成场景，3708x5412 栅格）：画布 43.1M -> 7.2M 元素，
    `_search_rect` 47.5s -> 见 `.sdd-detect/task-5-report.md` 的「T5b」。

    ⚠️ **T9：裁剪矩形改用轴对齐的紧 bbox**（`bbox` 参数 / `_content_bbox`）。
    上面的严格性论证**一个字都不用改** —— 它只用到"裁剪图 ⊇ 内容"。换掉的是
    **怎么算**这个矩形：T5b 按 θ 旋转后取 bbox，θ 离内容主轴 45° 时那个框会
    退化成整张栅格，兜底触发、**整段退回不裁剪**（实测 5_test 19/46 个角、
    april_test7 31/46 个角退回）。轴对齐紧 bbox 与 θ 无关，46 + 33 个角共用。
    仍然会有内容已占满栅格对角线的帧裁不动（april_test7），那是几何限制。

    `M_warp` 把**裁剪图**坐标映射到旋转画布（直接喂 `cv2.warpAffine`），
    `M_out` 把**原栅格**坐标映射到旋转画布（返回给调用方，
    `_rot_corner_to_ab` 用它反算回原栅格 —— 所以裁剪偏移是**算进矩阵里**的，
    调用方完全不用改）。`crop=False` 时退回 `_rotated`，即改动前的行为。
    """
    if not crop:
        rot, M, big = _rotated(mask, theta)
        return rot, M, big, M
    h, w = mask.shape
    # 内容的**轴对齐**紧 bbox（T9：不再按 θ 旋转，理由见 `_content_bbox`）。
    # `bbox` 可由调用方传入：θ 无关，46 + 33 个角共用一次 `np.nonzero`。
    if bbox is None:
        bbox = _content_bbox(mask, margin)
    if bbox is None:                      # 全零掩码：没有内容可裁，退回不裁剪
        rot, M, big = _rotated(mask, theta)
        return rot, M, big, M
    r0, c0, r1, c1 = bbox
    crop_mask = mask[r0:r1, c0:c1]

    c, s = float(np.cos(theta)), float(np.sin(theta))
    cx, cy = w / 2.0, h / 2.0
    big = int(np.ceil(np.hypot(w, h))) + 4
    # 与 `_rotated` 同一套约定：**绕画布中心转**（画布 `[0, big]` 见方）。
    # 裁剪图与整图共用同一个旋转中心，所以裁剪偏移是一个**整数平移**。
    t_out = np.array([big / 2.0 - c * cx - s * cy,
                      big / 2.0 + s * cx - c * cy])
    # 旋转后内容的外接框（`_content_bbox` 给的 `[r0, r1] x [c0, c1]` 是**原图**坐标，
    # 这里再解析地转一次四角，拿旋转画布坐标下的框）
    corners = np.array([[c0, r0], [c1, r0], [c1, r1], [c0, r1]], np.float64)
    q = corners @ np.array([[c, -s], [s, c]]) + t_out
    X0, X1 = float(q[:, 0].min()), float(q[:, 0].max())
    Y0, Y1 = float(q[:, 1].min()), float(q[:, 1].max())
    # 窗口：让"框内含有内容的矩形"（含被左移/上移规整过的那个最优解）全部落进来
    wx0, wx1 = X0 - float(w_mm) - margin, X1 + float(step) + margin
    wy0, wy1 = Y0 - float(h_mm) - margin, Y1 + float(step) + margin
    dx = int(np.floor(wx0 / step) * step)      # 对齐到 step 的整数倍（同相抽样）
    dy = int(np.floor(wy0 / step) * step)
    big_w = int(np.ceil(wx1 - dx)) + 1
    big_h = int(np.ceil(wy1 - dy)) + 1
    # 裁剪画布必须装得下旋转后的**裁剪图**。裁剪图比"内容的外接框"每边最多多出
    # `margin`（≈2mm），转过去后每边最多多 `margin*sqrt(2)`（<3mm），
    # 而窗口每边本来就多出一个 `w_mm`/`h_mm`（几百 mm），所以窗口一定装得下；
    # 这里只兜住"裁剪没赚到"（窗口比全画布还大）的退化情形。
    if big_w >= big or big_h >= big:
        # 裁剪没赚到（或装不下）：退回不裁剪。两条路结果相同，退回只是省事
        rot, M, big = _rotated(mask, theta)
        return rot, M, big, M
    M_out = np.array([[c, s, t_out[0] - dx],
                      [-s, c, t_out[1] - dy]], np.float64)
    M_warp = np.array([[c, s, t_out[0] - dx + c * c0 + s * r0],
                       [-s, c, t_out[1] - dy - s * c0 + c * r0]], np.float64)
    rot = cv2.warpAffine(crop_mask.astype(np.uint8) * 255, M_warp, (big_w, big_h),
                         flags=cv2.INTER_NEAREST)
    return rot > 0, M_warp, (big_h, big_w), M_out


def _best_shift(rot: np.ndarray, w_mm: int, h_mm: int, step: int,
                near: tuple[float, float, float] | None = None
                ) -> tuple[float, int, int, bool]:
    """在旋转后的掩码上找框内和最大的轴对齐 `w_mm x h_mm` 框。

    ⚠️ 必须**向量化**。朴素的双重 Python 循环在这个栅格尺寸上不可行：旋转后的
    栅格是 `ceil(hypot(w,h))` 见方，5_test 上约 400×400，步长 4 就是 1e4 个位置
    × 46 个候选角 = 5e5 次 numpy 调用，跑一次要几分钟。滑动窗用积分图一次算完。

    ⚠️ **T9：`near=(y_c, x_c, win)` 把候选位移限制在 `(x_c, y_c)` 周围 ±win 内**。
    精搜专用：粗搜已经用 `COARSE_CELL_MM` 的网格把位置定下来了，精搜没有必要在
    整张画布上再穷举一遍 —— 那正是单帧 5.3s 的来源（5388² 画布 × 33 个角 ≈
    10M 个候选位移）。

    ⚠️ **给了 `near` 时，平局按"离 `(x_c, y_c)` 最近"破，不按扫描序。**
    这不是锦上添花，是 `near` 能成立的前提：掩码是稀疏点阵 + 闭运算出来的，
    矩形沿某些方向平移几像素**框内和一格都不变**（等值平台），扫描序 `argmax`
    在平台上挑的是**左上角最小**的那个点，可以离真最优很远（实测合成场景里
    离粗搜赢家 200mm 以上）。那时"粗搜赢家 ±8mm"这个前提根本不成立 ——
    窗口里**一个平局点都没有**，只能退回全画布，优化就白做了。
    按距离破平局把精搜拉回粗搜赢家附近，`near` 才真正生效。
    （实测：只加 `near` 不加这条破法，合成退化用例 9 个里 5 个 FAIL；
    加上之后 9/9 —— 但**这个对照不能证明"就近破平局本身更对"**，
    见下面那条警告。）

    ⚠️ **这里必须说清楚的一条**：在**真正的等值平台**上，"哪个位移更对"是
    **不可判的** —— 目标函数在该方向上没有极值。所以"就近破平局"只是把结果
    钉在**粗搜的结论附近**（一个可复现、可解释的选择），不是"找到了更优解"。
    它改变结果的地方，都是原本就没有信息的维度。
    粗搜（`near is None`）保持原样不动 —— 那条路是整个搜索的地基，
    改它会牵动全部下游判据。
    """
    ii = _integral(rot)
    H, W = rot.shape
    if w_mm >= W or h_mm >= H:
        return 0.0, 0, 0, False
    # 每个 (y0, x0) 处的框内和，一次算完
    # ⚠️ T5b：**只算要抽样的那些格点**（`np.ix_` 花式索引），不要先算满整张
    # `S` 再 `[::step, ::step]`。结果逐位相同（实测 `array_equal=True`），
    # 6565² 画布上 0.118s -> 0.032s，且省掉一张 ~43M float64 的中间数组。
    rows = np.arange(0, H - h_mm + 1, step)
    cols = np.arange(0, W - w_mm + 1, step)
    if near is not None:
        y_c, x_c, win = near
        # 画布装得下的左上角范围是 `[0, W-w] x `[0, H-h]`。粗搜赢家有可能落在
        # 它的边界外 —— 两种来源：① 精搜画布的窗口每边比粗搜窄
        # `coarse_step - fine_step`（窗口边距是 `+step`）；② **粗搜那一轮
        # `_rotated_crop` 兜底退回了不裁剪的全画布**（内容已占满栅格对角线时），
        # 于是粗搜赢家可以在全画布的任何位置，包括精搜裁剪窗口之外。
        # ② 在等值平台上很常见：`argmax` 挑的是扫描序第一个，不一定是真最优。
        x_c = float(np.clip(x_c, 0.0, max(0, W - w_mm)))
        y_c = float(np.clip(y_c, 0.0, max(0, H - h_mm)))
        rows = rows[(rows >= y_c - win) & (rows <= y_c + win)]
        cols = cols[(cols >= x_c - win) & (cols <= x_c + win)]
        if rows.size == 0 or cols.size == 0:
            rows = np.arange(0, H - h_mm + 1, step)
            cols = np.arange(0, W - w_mm + 1, step)
    Ss = (ii[np.ix_(rows + h_mm, cols + w_mm)] - ii[np.ix_(rows, cols + w_mm)]
          - ii[np.ix_(rows + h_mm, cols)] + ii[np.ix_(rows, cols)])
    if near is None:
        j = int(np.argmax(Ss))
        by, bx = divmod(j, Ss.shape[1])
        return float(Ss[by, bx]), int(bx * step), int(by * step), False
    # 平局按**离粗搜赢家的距离**破，不按扫描序
    best = float(Ss.max())
    ty = rows[:, None].astype(np.float64) - y_c
    tx = cols[None, :].astype(np.float64) - x_c
    d = ty * ty + tx * tx
    d = np.where(Ss >= best - 1e-9, d, np.inf)
    j = int(np.argmin(d))
    by, bx = divmod(j, Ss.shape[1])
    # ⚠️ **`sat`（顶到窗边）不是"这个解不好"，是"真最优可能在窗外"** ——
    # 先验路径靠它决定要不要退回全画布搜索。判据：选中的格点落在**实际搜过的
    # 那一片的边界**上，即 `bx/by` 在过滤后的 `cols/rows` 两端。
    #
    # ⚠️ **不能写成 `cols[bx] <= x_c - win`**（T9b 踩过的坑）：`cols` 是
    # `np.arange` 之后按窗口过滤的，`step=1` 时最小格是 `ceil(x_c-win)`，
    # **严格大于** `x_c - win`，那个比较永远为假、`sat` 永远是 False，
    # 于是漂移检测静默失效（实测先验偏 250mm 时 score 还有 0.917、位姿偏 48mm，
    # 却报 `prior_used=True`）。
    sat = (bx == 0 or bx == cols.size - 1 or by == 0 or by == rows.size - 1)
    return float(Ss[by, bx]), int(cols[bx]), int(rows[by]), bool(sat)


def _search_rect(mask: np.ndarray, w_mm: float, h_mm: float, theta_ref: float,
                 opts: dict | None = None) -> dict:
    """已知尺寸的旋转矩形搜索（规格 §4 Step 4）。

    形状是**硬约束**（矩形恒为 w_mm x h_mm），所以"缩成一小块拿满分"这个退化不存在。
    打分用 coverage = 框内掩码和 / 框面积 —— 对遮挡稳健（3/4 可见就该得 ~0.75）。

    返回 `{'theta', 'x0', 'y0', 'score', 'M', 'rot', 'big', 'crop'}`。
    ⚠️ **`M` 必须留在返回值里**（任务书 Step 5 的修正版）：调用方要用它把旋转后栅格里
    的框左上角还原回原栅格的 `(a, b)`。

    ⚠️ **T5b 提速**：每一轮都先按掩码内容的 bbox 裁剪再旋转（`_rotated_crop`），
    画布从"整张栅格的对角线"缩到"内容对角 + 搜索矩形对角"。裁剪偏移**已经算进
    `M` 里**，所以 `_rot_corner_to_ab(M, x0, y0)` 仍然给出**全图栅格坐标** ——
    调用方的用法一个字都不用改。`opts['crop_search']=False` 可退回不裁剪
    （`crop=False` 时 `M` 与改动前的 `_rotated` 完全一致），测试用它钉住两者等价。
    裁剪的严格性论证见 `_rotated_crop` 的 docstring。

    ⚠️ **T9 提速（两刀，都实测过等价性）**：

    **A. 裁剪矩形改用轴对齐紧 bbox**（`_content_bbox`，46 + 33 个角只算一次）。
    T5b 按 θ 旋转后取 bbox，θ 离内容主轴 45° 附近时那个框退化成整张栅格，
    兜底触发、**整段退回不裁剪** —— 实测 5_test 19/46 个角、april_test7
    31/46 个角在跑 15.6M / 29.0M 元素的画布。轴对齐紧 bbox 与 θ 无关。

    **B. 精搜的位移限制在粗搜赢家周围 ±`FINE_SHIFT_WIN_MM`**（`_best_shift(near=...)`）。
    粗搜用 `COARSE_CELL_MM`(4mm) 的网格已经把位置定下来了，精搜没有必要在
    整张画布上再穷举一遍 —— 实测那正是最大的一刀：april_test7 上精搜 33 个角
    × 5388² 画布 ≈ 10M 个候选位移，**单是 `argmax` 就 5347ms**。

    ⚠️ **坐标要换算，而且不能自己减平移项**：粗搜 `step=4`、精搜 `step=1`，
    `_rotated_crop` 的窗口原点对齐到各自的 step，两次的**画布坐标差一个整数平移**。
    换算走**调用方同款**的 `_rot_corner_to_ab(M, x0, y0)` 拿原栅格坐标
    （`M` 是 `M_out`，无论那一轮走的是裁剪还是兜底退回 `_rotated`，它都给全图
    栅格坐标），精搜里再用**本轮自己的** `M` 映回去。
    ⚠️ **别写成"加 `M_out[:,2] - M_warp[:,2]`"** —— 兜底退回那一路
    `M_warp is M_out`、两者之差恒为 0，算出来的"全局坐标"是错的。这个坑实测踩过：
    遮 1 边那组因此给出 (886, 1301) 这种越界坐标。

    ⚠️ **窗口大小是实测定的，不是拍的**（6 帧真值数据：april_test7 ×3、
    april_test6 ×3、5_test ×1，每帧与「全画布精搜」逐位对拍）：

      | 窗口 | 结果不一致的帧数 |
      |---|---|
      | ±2mm | 4/7 |
      | ±4mm | 3/7 |
      | **±8mm** | **0/7** |

    ±8 与粗搜 4mm 网格给出的 ±4mm 定位误差是同量级的（还要留一个 step 的余量），
    所以这个数字有物理依据，不是"调到恰好能过"。`opts['fine_shift_win_mm']`
    可覆盖。

    ⚠️ **但上面那张表不足以支撑 `near`**：那 7 帧上"全画布精搜"的赢家本来就离
    粗搜赢家很近，所以窗口再小也不影响结果 —— 那是**数据恰好如此**，不是窗口
    足够大的证明。真正的风险在**等值平台**上：此时全画布精搜挑的是扫描序第一个，
    可以离粗搜赢家 200mm 以上（合成场景实测），窗口根本装不下它。
    `_best_shift` 对这种情况的处理是"就近破平局"（见它的 docstring）——
    结果是**钉在粗搜结论附近**，不是复现全画布精搜。
    这是本任务里唯一一处**主动改变结果**的地方，且只发生在**目标函数没有极值**
    的维度上；`near=None`（粗搜）的行为一字未动。
    """
    opts = dict(opts or {})
    crop = bool(opts.get('crop_search', True))
    coarse_step = int(opts.get('coarse_cell_mm', COARSE_CELL_MM))
    fine_step = int(opts.get('fine_cell_mm', 1))
    win = float(opts.get('fine_shift_win_mm', FINE_SHIFT_WIN_MM))
    d_th_coarse = np.radians(float(opts.get('theta_coarse_step_deg', THETA_COARSE_STEP_DEG)))
    d_th_fine = np.radians(float(opts.get('theta_fine_step_deg', THETA_FINE_STEP_DEG)))
    half = np.radians(float(opts.get('theta_half_deg', THETA_HALF_DEG)))
    fine_half = np.radians(float(opts.get('theta_fine_half_deg', THETA_FINE_HALF_DEG)))
    area = float(w_mm * h_mm)
    # 内容 bbox 与 θ 无关：整个搜索共用一次 `np.nonzero`（T9/A）
    bbox = _content_bbox(mask) if crop else None

    # --- 粗搜 ---
    # `M`/`rot`/`big` 一起初始化，避免粗搜一次都没命中时 `best['M']` 不存在
    best = dict(theta=float(theta_ref), x0=0, y0=0, score=-1.0,
                M=None, rot=None, big=None, crop=crop)
    # 粗搜赢家在**原栅格**坐标下的位置（精搜的 `near` 要它，见 docstring）
    ab_x = ab_y = 0.0
    n_coarse = max(2, int(round(2 * half / d_th_coarse)) + 1)
    for k in range(n_coarse):
        th = theta_ref - half + k * d_th_coarse
        rot, M_warp, big, M = _rotated_crop(mask, th, w_mm, h_mm, coarse_step, crop,
                                            bbox=bbox)
        s, x0, y0, _ = _best_shift(rot, int(w_mm), int(h_mm), coarse_step)
        score = s / area
        if score > best['score']:
            best = dict(theta=th, x0=x0, y0=y0, score=score, M=M, rot=rot,
                        big=big, crop=crop)
            # 用**调用方同款**的换算拿原栅格坐标：`_rot_corner_to_ab` 吃的是
            # `M_out`，无论这一轮走的是裁剪还是兜底退回 `_rotated`，它都给
            # 全图栅格坐标。**别自己减 `M_warp` 的平移项** —— 兜底那一路
            # `M_warp is M_out`，两者之差恒为 0，算出来的"全局坐标"是错的
            # （实测：遮 1 边那组会因此落到窗外，报出 886/1301 这种越界值）。
            ab = _rot_corner_to_ab(M, x0, y0)
            ab_x, ab_y = float(ab[0]), float(ab[1])

    # --- 精搜 ---
    th_c = best['theta']
    n_fine = max(2, int(round(2 * fine_half / d_th_fine)) + 1)
    for k in range(n_fine):
        th = th_c - fine_half + k * d_th_fine
        rot, M_warp, big, M = _rotated_crop(mask, th, w_mm, h_mm, fine_step, crop,
                                            bbox=bbox)
        # 原栅格坐标 -> **这一轮自己的**旋转画布坐标（各轮画布的原点不同）
        near_x, near_y = (M @ np.array([ab_x, ab_y, 1.0]))[:2]
        s, x0, y0, _ = _best_shift(rot, int(w_mm), int(h_mm), fine_step,
                                   near=(float(near_y), float(near_x), win))
        score = s / area
        if score > best['score']:
            best = dict(theta=th, x0=x0, y0=y0, score=score, M=M, rot=rot,
                        big=big, crop=crop)

    return best          # 保留 M / rot / big：调用方要用 M 把框角还原回原栅格


def _rot_corner_to_ab(M: np.ndarray, x0: float, y0: float) -> np.ndarray:
    """把旋转后栅格里的框左上角，还原成原栅格 (a, b) 坐标。

    `M` 是**原栅格 -> 旋转画布**的矩阵（裁剪偏移已含在里面），所以这里不需要
    任何裁剪补偿 —— 换裁剪边距也不会改变它的输出。
    """
    Minv = cv2.invertAffineTransform(M)
    p = Minv @ np.array([x0, y0, 1.0])
    return p[:2]


# --------------------------------------------------------------------------- #
# T9b：先验跟踪路径
# --------------------------------------------------------------------------- #
def _theta_from_frame(frame: dict, normal: np.ndarray) -> float:
    """把 frame 的 **E1** 化成 `_search_rect` 那套 θ（占位基上的角度）。

    `_theta_ref` 返回的是"长边方向"在 `(_basis_from_normal(normal))` 基上的角度，
    而搜索时矩形以**长边为 x 轴**。所以先验 θ 必须由调用方按**同一个约定**给出：
    `frame['E1']` 是长边时就是 `atan2(E1·E2基, E1·E1基)`；
    若 `E1` 是短边（`W < H`），要再加 90°。
    """
    u, v = _basis_from_normal(normal)
    e1 = np.asarray(frame['E1'], float)
    th = float(np.arctan2(float(e1 @ v), float(e1 @ u)))
    if float(frame.get('W', 0.0)) < float(frame.get('H', 0.0)):
        th += np.pi / 2.0
    return th


def _prior_from_frame(frame: dict, info: dict, normal: np.ndarray) -> tuple[float, tuple[float, float]]:
    """上一帧的 frame -> 这一帧栅格上的先验 `(theta, (col_c, row_c))`。

    ⚠️ **约定要对齐，否则先验是错的、而且错得很隐蔽**：

      - `_search_rect` 的 θ 是**长边**方向（调用方传的是 `w=long, h=short`），
        而 `_theta_ref` 返回的就是长边在 `_basis_from_normal(normal)` 基上的角度。
        所以先验 θ 取 `frame['E1']` 的角度 —— **但前提是 E1 就是长边**。
        `detect_pallet_frame` 返回的 frame 里 `W` 是**沿 E1 的边长**、`H` 是沿 E2 的，
        所以 `W < H` 时 E1 是短边，θ 要加 90°。
      - 位置用**矩形中心**，不用角点：角点在"旋转画布坐标"里是哪个角会随 θ 变，
        中心才是 θ 无关的。

    返回的 `(col_c, row_c)` 是**栅格坐标**（`col = a - p0`，`row = b - q0`），
    与 `_rotated_crop` 吃的那套一致。
    """
    u, v = _basis_from_normal(normal)
    o = np.asarray(frame['origin'], float)
    c = o + 0.5 * float(frame['W']) * np.asarray(frame['E1'], float) \
          + 0.5 * float(frame['H']) * np.asarray(frame['E2'], float)
    r = c - np.asarray(info['origin'], float)
    col = float(r @ u) - float(info['p0'])
    row = float(r @ v) - float(info['q0'])
    return _theta_from_frame(frame, normal), (col, row)


def _search_rect_prior(mask: np.ndarray, w_mm: float, h_mm: float,
                       prior_theta: float, prior_ab: tuple[float, float],
                       opts: dict | None = None) -> dict:
    """**先验跟踪路径**：θ 只在先验附近细扫，位移只在先验附近小窗里找。

    这一条路是为**实时**准备的（操作员 2026-09-23）：托盘逐帧移动很小
    （真值实测相邻帧 ≤37mm / ≤2.4°，见 `PRIOR_*` 常量的注释），所以上一帧的位姿
    就是这一帧极强的先验，不需要再全象限搜索。

    ⚠️ **为什么必须换裁剪窗口**：`_rotated_crop` 的窗口是按**整块掩码内容**算的
    （`[X0-w, X1+w] x [Y0-h, Y1+h]`），内容铺满栅格时窗口就接近全画布，
    画布仍有 29M 元素 —— 实测那样跑 25 个角要 **970ms**，一点没省。
    这里改成按**候选矩形的并集**裁剪（先验位置 ± 位移窗，先验 θ ± 细扫窗），
    实测同一帧 **384ms**（画布 5.8M vs 29M）。

    ⚠️ **`bbox` 要按角度逐轮重算**：θ 细扫 ±5° 时先验矩形的**轴对齐**外接框
    会随之变大，用先验 θ 那一个固定 bbox 会在两端切掉角点。

    ⚠️ **θ 的相位**：细扫网格锚定在**先验 θ** 上（`prior ± half`，步长
    `fine_cell`），不锚在 `theta_ref` 上 —— 先验路径下 `theta_ref` 本身也可能是
    错的，那正是要避开的东西。

    `prior_ab` 是**矩形中心**的栅格坐标（见 `_prior_from_frame`）。
    返回结构与 `_search_rect` 一致，额外多两个字段：

      - `'prior': True`；
      - `'window_saturated'`：**最优解顶到了位移窗的边界上**。这是"真最优可能在
        窗外"的信号（先验偏了、或者托盘真的动了超过 `PRIOR_SHIFT_WIN_MM`），
        调用方应当**退回全画布搜索**。

    ⚠️ **为什么必须有 `window_saturated`**：`score` 拦不住先验漂移 —— 掩码是一整块
    台面，矩形落在台面里任何位置 coverage 都高。合成场景实测：先验偏 400mm 时
    score 仍有 0.789（远高于 `COVERAGE_MIN` 0.35），但位姿已经偏 254mm。
    "顶到窗边"才是真正可观测的漂移信号。实测先验偏 120mm -> 不顶边、结果精确；
    偏 160mm（刚超出 ±150 窗）-> 顶边、偏 14.5mm；**先验在窗内时结果与全局搜索
    逐位相同**。
    """
    opts = dict(opts or {})
    fine_step = int(opts.get('fine_cell_mm', 1))
    d_th = np.radians(float(opts.get('theta_fine_step_deg', THETA_FINE_STEP_DEG)))
    half = np.radians(float(opts.get('prior_theta_half_deg', PRIOR_THETA_HALF_DEG)))
    win = float(opts.get('prior_shift_win_mm', PRIOR_SHIFT_WIN_MM))
    pad = float(opts.get('prior_pad_mm', PRIOR_PAD_MM))
    area = float(w_mm * h_mm)
    ax, ay = float(prior_ab[0]), float(prior_ab[1])
    H, W = mask.shape
    # 画布坐标下的矩形半尺寸（长边沿画布 x）
    hw, hh = w_mm / 2.0, h_mm / 2.0
    loc = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]])

    best = dict(theta=float(prior_theta), x0=0, y0=0, score=-1.0,
                M=None, rot=None, big=None, crop=True, prior=True,
                window_saturated=False)
    n = max(2, int(round(2 * half / d_th)) + 1)
    for k in range(n):
        th = prior_theta - half + k * d_th
        c, s = float(np.cos(th)), float(np.sin(th))
        # 画布偏移 -> 栅格偏移：canvas = R @ raster，R = [[c, s], [-s, c]]，
        # 所以 raster_offset = R^T @ canvas_offset = (loc @ R) 的行向量形式。
        rc = loc @ np.array([[c, s], [-s, c]], np.float64) + np.array([ax, ay])
        # ⚠️ **必须把位移窗 `win` 也算进 ROI**（T9b 踩过的坑）：ROI 是"裁剪图 ⊇
        # 内容"里的**内容**，而这里的内容是**搜索会走到的那一片** = 先验矩形
        # 再往各方向挪 ±win。只放先验矩形本身（哪怕加几毫米 pad）会把真最优
        # **裁掉**，搜索只能在剩下的部分里挑次优解 —— 而且**不报错**，
        # 只是 score 悄悄低一点（合成场景实测：先验偏 22mm 时 score 0.9934 ->
        # 0.9884，位姿差 10mm）。位移是画布坐标下的 ±win，换到栅格每个轴的
        # 分量绝对值 <= win，所以**每个轴各放 win** 是安全的（含对角方向）。
        grow = win + pad
        bbox = (max(0, int(np.floor(rc[:, 1].min() - grow))),
                max(0, int(np.floor(rc[:, 0].min() - grow))),
                min(H, int(np.ceil(rc[:, 1].max() + grow)) + 1),
                min(W, int(np.ceil(rc[:, 0].max() + grow)) + 1))
        if bbox[2] - bbox[0] < 4 or bbox[3] - bbox[1] < 4:
            continue                       # 先验整个落在栅格外：这个角没法搜
        rot, M_warp, big, M = _rotated_crop(mask, th, w_mm, h_mm, fine_step, True,
                                            bbox=bbox)
        # 先验矩形**中心**在本轮画布里的位置；画布里矩形轴对齐，
        # 所以左上角 = 中心 - (w/2, h/2)
        cx, cy = (M @ np.array([ax, ay, 1.0]))[:2]
        s_, x0, y0, sat = _best_shift(rot, int(w_mm), int(h_mm), fine_step,
                                      near=(float(cy) - hh, float(cx) - hw, win))
        score = s_ / area
        # θ 顶到细扫窗的**两端**也是漂移信号（与位移顶边同理）：真 θ 在窗外时
        # 搜索只能在窗边挑一个，误差 = 先验误差 - 窗宽。实测先验 θ 偏 8°、
        # 窗 ±5° 时，结果 dθ = 3.00°（正好是 8-5），score 却还有 0.975。
        th_sat = (k == 0 or k == n - 1)
        if score > best['score']:
            best = dict(theta=th, x0=x0, y0=y0, score=score, M=M, rot=rot,
                        big=big, crop=True, prior=True,
                        window_saturated=bool(sat or th_sat))
        elif score == best['score']:
            best['window_saturated'] = (best['window_saturated']
                                        or bool(sat) or bool(th_sat))
    return best


# --------------------------------------------------------------------------- #
# 定向：按规格 §3 定 origin / E1 / E2
# --------------------------------------------------------------------------- #
def _orient(frame: dict, W: float, H: float, k) -> dict:
    """把四个角投回图像，定出 origin 和 E1/E2 的朝向（规格 §3）。

    ⚠️ 规格 §3 的两条约定都要**算出来**，不能拍：
      - E2 取"投影到图像里更向上（v 更小）"的那条边方向；
      - E1 = E2 x nrm（右手系）。相机系里图像右就是 +x，所以只要 E1[0] < 0
        就把 E1、E2 同时取反（E2 仍指向上）。

    ⚠️ **任务书 Step 4 的草稿在选 origin 那一步是错的，这里是修正版**（T5 实测）：
    草稿先取反 E1/E2，再用 `o + a*E1_new + b*E2_new` **重新生成**四角去挑
    "u 小、v 大"的那个。可是把 E1、E2 同时取反**并不会保持矩形不动** ——
    矩形从 origin 沿 +E1/+E2 方向展开，两个方向都取反后矩形整体翻到了另一边。
    于是候选四角里只剩 origin 自己还落在真实矩形上，挑出来的 origin 会偏
    **一整个对角线**（合成场景实测：偏了 (W, H) = (1200, 1000)mm，
    `dE1=+1199mm dE2=+1000mm`，随后 `_edge_support` 判出 0 条边 → `too_few_edges`）。

    正确做法：**物理矩形是固定的**，四条边与四个角都从输入 frame 直接算，
    朝向只用来定 E1/E2 的符号；origin 则从**真实的四个角**里挑"投影 u 小、v 大"的
    那个。这样 E1/E2 取不取反都不影响矩形本身。

    ⚠️ **2026-09-30 起，`W`/`H` 由调用方按 `long_side_parallel` 定，本函数不再
    自作主张翻转它们。** 下面那段 W/H 换位是 2026-09-29 的修复，现在**不该再触发**
    —— `detect_pallet_frame` 会把"横向的那条边"放进 `rough['E1']`，于是
    `E2_new`（指向上的一条）恒等于 `rough['E2']`，点积判据不成立。留着它是兜底：
    万一 `E2_new` 落到另一条边上，宁可在这里换位，也不能让 `E1`/`W` 错配。

    ⚠️ **（历史）W/H 曾经要跟着"E2_new 实际落在哪条物理边上"走**（2026-09-29 现场实测）。
    入参 `W`/`H` 是**沿输入 frame 的 E1/E2 的边长**，而 `E2_new` 是从**四条边里**
    挑出来的一条 —— 它可能原本是沿 E1 的那条（长度 W）。旧代码直接把入参 `W`/`H`
    原样带出去，于是当 `E2_new` 恰好沿 E1 时，返回的 `E1/E2` 与 `W/H` **差一个
    90°**：`E1` 指短边、`W` 却是长边的长度。

    后果**在下游才显形，而且看着完全正常**：检测自己算出的位姿是对的（四角与
    操作员点击真值差 6~21px），但 `E1`/`W` 这一对是错配的 ——
    任何"沿 E1 走 W 毫米"的使用者都会走到**另一个方向**上去。实测现场帧：
    发布出来的四角整体转了 90°（角间距 62/64px），而 `size_mm`、
    `_theta_from_frame`（1.31° vs 搜索的 91.31°，正好差 90°）全都跟着错。

    ⚠️ **判定只能用 3D 基向量，不能用投影边长。** 旧代码在后面拿
    `_project_px` 量 e1_px/e2_px 来定长边 —— 透视缩短会把顺序颠倒：现场帧上
    **长边（1200mm）投影出 403px、短边（1000mm）294px**，看着还是长的长，
    但此前那次 1200mm 只投出 322px 而 1000mm 投出 495px，于是判反。
    `E2_new` 与入参 `E1`/`E2` 的点积是**平面内的精确量**，没有透视问题。
    """
    o, E1, E2, nrm = frame['origin'], frame['E1'], frame['E2'], frame['nrm']
    corners = _corners_mm(frame, W, H)            # 真实的四个角，顺序 (0,0)(W,0)(W,H)(0,H)
    px = _project_px(corners, k)
    # 四条边：i -> i+1。边 0 沿 E1（长度 W），边 1 沿 E2（长度 H）
    up = []                       # (dv, 沿哪个基)
    for i, vec in ((0, E1), (1, E2), (2, -E1), (3, -E2)):
        d = px[(i + 1) % 4] - px[i]
        up.append((float(d[1]), vec))
    up.sort(key=lambda t: t[0])
    E2_new = np.asarray(up[0][1], float)          # v 更小 = 更向上
    E2_new = E2_new / np.linalg.norm(E2_new)
    E1_new = np.cross(E2_new, nrm)
    E1_new = E1_new / np.linalg.norm(E1_new)
    if E1_new[0] < 0:                             # 保证指向画面右
        E1_new, E2_new = -E1_new, -E2_new

    # W/H 跟着 E2_new 落在哪条物理边上（见 docstring：不能用投影边长判）
    e1u, e2u = E1 / np.linalg.norm(E1), E2 / np.linalg.norm(E2)
    W_new, H_new = (H, W) if abs(float(E2_new @ e1u)) > abs(float(E2_new @ e2u)) else (W, H)

    # 左下角 = 投影里 (u 小, v 大) 的那个角。**在真实四角里挑**
    score = px[:, 0] - px[:, 1]                   # u 小、v 大 -> 分数小
    origin = corners[int(np.argmin(score))]
    return dict(origin=origin, E1=E1_new, E2=E2_new, nrm=nrm,
                W=W_new, H=H_new)


# --------------------------------------------------------------------------- #
# 边的支撑率
# --------------------------------------------------------------------------- #
def _edge_support(frame: dict, W: float, H: float, mask: np.ndarray, k,
                  info: dict, opts: dict) -> list[dict]:
    """每条预测边的**支撑率**：这条边上有多大比例的点落在掩码边界上（规格 §5.1）。

    ⚠️ **必须在占位栅格坐标里做，不能用图像像素索引 `mask`**（任务书 Step 5 的
    草稿在这里是"沿图像法向探 `mask[row, col]`"，那需要 `mask` 是**图像**掩码；
    而 `_deck_mask` 返回的是**占位栅格**掩码，5_test 上两者尺寸差一个量级，
    直接索引会探到完全无关的地方）。做法：

      1. 预测四角（相机系 mm）减 `info['origin']`，再投到占位基上得 `(a, b)`，
         减 `info['p0']`/`info['q0']` 得栅格像素 —— 与 `truth_mask_on_raster` 同一套换算；
      2. 沿每条边采样，每个采样点沿边的**外法向**在 ±`probe_px`（栅格 1px = 1mm）
         内找掩码的"有 -> 无"跳变；找到就算支撑。

    这与 refine 的梯度吸附是同一个几何，只是这里用的是二值掩码而不是图像梯度。
    """
    probe = int(opts.get('edge_probe_px', 25))
    n_samp = int(opts.get('edge_samples', 40))
    u_b, v_b = _basis_from_normal(frame['nrm'])
    r = _corners_mm(frame, W, H) - np.asarray(info['origin'], float)
    px = np.stack([r @ u_b - float(info['p0']), r @ v_b - float(info['q0'])], -1)
    hh, ww = mask.shape
    out = []
    for i in range(4):
        p0, p1 = px[i], px[(i + 1) % 4]
        d = p1 - p0
        L = float(np.hypot(*d))
        if L < 20:
            out.append(dict(name=f'edge{i}', support=0.0, observed=False,
                            note='too_short'))
            continue
        u = d / L
        n = np.array([u[1], -u[0]])          # 栅格里的一个法向
        hit = 0
        tot = 0
        for t in np.linspace(0.1, 0.9, n_samp):
            base = p0 + t * L * u
            found = False
            for sgn in (1.0, -1.0):
                for s in range(1, probe + 1):
                    pt = base + sgn * s * n
                    col = int(round(pt[0])); row = int(round(pt[1]))
                    if not (0 <= row < hh and 0 <= col < ww):
                        break
                    if mask[row, col]:
                        found = True
                        break
                if found:
                    break
            tot += 1
            hit += int(found)
        support = hit / max(tot, 1)
        out.append(dict(name=f'edge{i}', support=round(support, 3),
                        observed=support >= EDGE_MASK_SUPPORT_MIN, note=None))
    return out


# --------------------------------------------------------------------------- #
# 主入口
# --------------------------------------------------------------------------- #
def detect_pallet_frame(color: np.ndarray, depth: np.ndarray, k, *,
                        normal, target_mm=TARGET_MM,
                        long_side_parallel: bool = LONG_SIDE_PARALLEL,
                        prior: dict | None = None,
                        camera_z_mm: float | None = None,
                        deck_z_mm: tuple[float, float] | None = None,
                        opts: dict | None = None) -> tuple[dict | None, dict]:
    """无初值检测木托盘台面坐标系。返回 (frame | None, 诊断 dict)。

    返回的 frame 字段与 `pallet_from_ground_quad.ground_frame()` 同构，
    **直接可喂给 `refine_pallet_frame`**。

    `prior`（T9b，可选）：**上一帧的检测结果**（就是本函数的 `found`）。
    给了它就走**先验跟踪路径**（`_search_rect_prior`）：θ 与位移都只在上一帧
    位姿附近搜，单帧快 3~7 倍。**先验结果低于 `PRIOR_MIN_SCORE` 时自动退回
    全画布搜索**（那一帧慢，但不会跟丢），并在 `diag['prior_reason']` 里如实
    写明为什么没用先验。`diag['prior_used']` 说这一帧实际走了哪条路。

    ⚠️ 先验是**外部状态**：调用方负责保证它是**上一帧、同一相机**的结果。
    给一个过时或来自别处的位姿不会报错，只会退化成"搜不到 -> 回退全搜索"。
    """
    t_start = time.perf_counter()
    opts = dict(opts or {})
    if prior is not None:
        opts.setdefault('prior', prior)
    # `camera_z_mm` / `deck_z_mm` 也可以从 opts 给（历史调用方习惯），显式参数优先。
    if camera_z_mm is not None:
        opts['camera_z_mm'] = float(camera_z_mm)
    if deck_z_mm is not None:
        opts['deck_z_mm'] = (float(deck_z_mm[0]), float(deck_z_mm[1]))
    normal = np.asarray(normal, float)
    normal = normal / np.linalg.norm(normal)
    _ta, _tb = float(target_mm[0]), float(target_mm[1])
    diag: dict = {'target_mm': [_ta, _tb], 'long_side_parallel': bool(long_side_parallel),
                  'reject': None, 'theta_ref_deg': None, 'theta_alt_deg': None,
                  'score': None, 'source': None, 'color_coverage': None,
                  'edges': [], 'n_observed_edges': 0, 'origin_visible': None,
                  'deck_h_rel_mm': None, 'sat_min': None, 'n_wood': 0,
                  'W_mm': None, 'H_mm': None,
                  'mask_area_ratio': None, 'mask_cells': None,
                  'prior_used': False, 'prior_reason': None, 'prior_theta_deg': None,
                  # 主路径（`_theta_from_image`）专属：θ 是不是图像空间给的、
                  # 没给成的原因、稠密掩码的填充率与 minAreaRect 量出的尺寸/角度
                  'rect_source': None, 'rect_why': None,
                  'dense_fill': None, 'dense_measured_mm': None,
                  'dense_angle_deg': None, 'snap_edges': None,
                  'lsp_measured_parallel': None, 'lsp_agrees': None,
                  'theta_search_deg': None,
                  'long_short_diff_deg': None, 'long_short_mismatch': None,
                  'timing_ms': {'deck_mask': None, 'theta_image': None, 'snap': None,
                                'theta_ref': None, 'search': None,
                                'orient': None, 'edges': None, 'total': None}}

    def _tick(key: str, t0: float) -> float:
        now = time.perf_counter()
        diag['timing_ms'][key] = round((now - t0) * 1000.0, 2)
        return now

    t = time.perf_counter()
    mask, height, info = _deck_mask(color, depth, k, normal, opts)
    _tick('deck_mask', t)
    diag['n_wood'] = info['n_wood']
    diag['sat_min'] = info['sat_min']
    diag['deck_h_rel_mm'] = info['h_deck']
    # 台面高度的**绝对**读数（只在调用方给了 z 带时有意义）
    if info.get('deck_z_mm') is not None:
        diag['deck_z_mm'] = info['deck_z_mm']
    diag['source'] = info.get('source')
    diag['color_coverage'] = info.get('color_coverage')
    if info['h_deck'] is None or int(mask.sum()) < MIN_WOOD_PX:
        diag['reject'] = 'no_wood' if info['n_wood'] < MIN_WOOD_PX else 'no_deck'
        _tick('total', t_start)
        return None, diag

    # --- T5c：掩码**面积过大** -> 拒绝（新判据，见 MASK_AREA_RATIO_MAX 的注释）---
    # 掩码面积比托盘标称面积。`target_mm` 是**操作员给的已知尺寸**，所以这个比值
    # 是"掩码有几倍托盘大"，与掩码自身的膨胀无关，也不需要知道栅格/体素的换算。
    # 放在搜索**之前**：既省掉后面 15s 的矩形搜索，也让诊断一眼能看出拒绝的原因。
    _mask_cells = int(mask.sum())
    _mask_area_ratio = _mask_cells / max(_ta * _tb, 1.0)
    diag['mask_cells'] = _mask_cells
    diag['mask_area_ratio'] = round(float(_mask_area_ratio), 4)
    if _mask_area_ratio > MASK_AREA_RATIO_MAX:
        diag['reject'] = 'mask_too_large'
        _tick('total', t_start)
        return None, diag

    # 栅格坐标 (row, col) = (b - q0, a - p0)，所以 E1 沿 col、E2 沿 row。
    # ⚠️ `origin` 取 `info['origin']`（**平面上的点**），不是 `np.zeros(3)` ——
    # 后者是**相机光心**，`_theta_ref` 拿它去 `_project_axis` 时两端点会一个落在
    # 光心、一个落到相机后方，投影方向整个翻掉（这是 2026-09-30 查出来的真 bug，
    # 见 `_project_axis`）。它只被 `_theta_ref` 读，改这里不影响别处。
    frame_ph = dict(origin=np.asarray(info['origin'], float),
                    E1=_basis_from_normal(normal)[0],
                    E2=_basis_from_normal(normal)[1], nrm=normal)
    # 哪条边配 target_mm 的长边：**按量出来的长度**配，不假设 E1 是长边（规格 §3）。
    # 主路径把它交给 `minAreaRect`（谁长谁配 `long_mm`）；旧路径靠掩码主轴选象限。
    long_mm, short_mm = max(_ta, _tb), min(_ta, _tb)
    # θ_ref 是**长边方向**。搜索时矩形以长边为 x 轴，所以 w=long、h=short。
    w_mm, h_mm = long_mm, short_mm

    prior = opts.get('prior')
    theta_ref = None
    theta_img = None            # 主路径给的 θ（只有无先验时算），哨兵也要用
    best = None
    rc = None
    diag['prior_used'] = False
    diag['prior_reason'] = None

    # --- 主路径：图像掩码 -> 稠密米制掩码 -> **只取 θ**（2026-09-30）---
    # 只在**没有先验**时跑（首帧 / 跟丢后的重捕帧）：有先验那一帧的矩形由
    # `_search_rect_prior` 给，用不上它，省掉 `_image_to_raster` 那 ~100ms。
    #
    # ⚠️ **它替换的是 `_theta_ref`，不是 `_search_rect`**（见 `_theta_from_image`
    # 里那段实测：只换 θ 时真机 5px，连矩形一起换反而 13px、还会把合成的
    # "遮 2 条相邻边"用例弄坏）。所以下面搜索那一段**一行没动**。
    #
    # ⚠️ **它必须排在 `_theta_ref` 前面。** 若先算 `_theta_ref` 并在它为 `None` 时
    # 直接 `return no_theta_ref`，就会多出一道"θ_ref 解不出来 => 整帧拒"的关卡，
    # 而主路径本来能出结果。顺序：主路径 -> 不通过才去算 `_theta_ref`。
    if prior is None:
        t = time.perf_counter()
        theta_img, tmeta = _theta_from_image(color, depth, mask.shape, k, normal,
                                             info, long_mm, short_mm,
                                             long_side_parallel, opts)
        _tick('theta_image', t)
        diag['rect_source'] = 'image_theta' if theta_img is not None else None
        diag['rect_why'] = tmeta['why']
        diag['dense_fill'] = tmeta['fill']
        diag['dense_measured_mm'] = tmeta['measured_mm']
        diag['dense_angle_deg'] = tmeta['angle_deg']
        # 声明 vs 实测的**交叉核对**（只记录，不改变上面任何东西，见
        # `_theta_from_image` 的 docstring）。θ 折到 [-90,90) 后 |θ|<45 = 长边近平行。
        if tmeta['angle_deg'] is not None:
            diag['lsp_measured_parallel'] = bool(
                abs(float(tmeta['angle_deg'] + 90.0) % 180.0 - 90.0) < 45.0)
            diag['lsp_agrees'] = bool(diag['lsp_measured_parallel']
                                      == bool(long_side_parallel))
        if theta_img is not None:
            theta_ref = theta_img

    # --- T9b 先验跟踪路径（可选）---
    # 上一帧的 frame 给出来 -> 只在它附近搜。**两种情况退回全画布搜索**：
    #   ① 先验路径的 score 低于 `PRIOR_MIN_SCORE`；
    #   ② 最优解**顶到了位移窗的边界**（`window_saturated`）—— 这是"真最优可能
    #      在窗外"的信号，`score` 拦不住它（掩码是一整块台面，矩形落在台面里
    #      任何位置 coverage 都高：实测先验偏 400mm 时 score 仍有 0.789）。
    # 退回那一帧慢（几秒），但**能自愈** —— 拿全搜索的结果当下一帧的先验就回来了。
    # 操作员 2026-09-23 明确选了这条（"自动退回全画布搜索"）。
    t = time.perf_counter()
    if prior is not None:
        try:
            p_th, p_ab = _prior_from_frame(prior, info, normal)
            bp = _search_rect_prior(mask, w_mm, h_mm, p_th, p_ab, opts)
            diag['prior_theta_deg'] = float(np.degrees(p_th))
            if bp['M'] is None:
                diag['prior_reason'] = 'prior_roi_empty'
            elif bp['window_saturated']:
                diag['prior_reason'] = 'prior_drift'
            elif bp['score'] < float(opts.get('prior_min_score', PRIOR_MIN_SCORE)):
                diag['prior_reason'] = 'prior_score_low'
            else:
                best = bp
                diag['prior_used'] = True
        except Exception as exc:                      # 先验坏掉不该拖垮整帧
            diag['prior_reason'] = f'prior_error:{type(exc).__name__}'
    if best is None:
        # 退回搜索：**这里才需要 `theta_ref`**（有先验时省掉了那几十 ms）。
        # `theta_ref` 若已由主路径（图像空间）给出就用它，否则现算 `_theta_ref`。
        if theta_ref is None:
            theta_ref = _theta_ref(mask, frame_ph, k, long_side_parallel)
            if theta_ref is None:
                diag['reject'] = 'no_theta_ref'
                _tick('total', t_start)
                return None, diag
            theta_ref += np.radians(float(opts.get('theta_ref_offset_deg', 0.0)))
        diag['theta_ref_deg'] = float(np.degrees(theta_ref))
        diag['theta_alt_deg'] = float(np.degrees(theta_ref)) + 90.0
        best = _search_rect(mask, w_mm, h_mm, theta_ref, opts)
        # ⚠️ **搜索是否用了真实 θ 要留痕**（2026-09-30）。
        # 下面 `diag['theta_ref_deg']` 对 prior 路径根本没算过，**不能直接读它**：
        # 老代码那样写，`%+.1f` 遇到 `None` 会 TypeError，让整棵行为树崩掉
        # （节点里 `format_diag` 就在发布路径上）。所以另存一个"搜索实际用的"。
        diag['theta_search_deg'] = float(np.degrees(theta_ref))
    _tick('search', t)
    diag['score'] = round(float(best['score']), 4)
    if best['score'] < COVERAGE_MIN:
        diag['reject'] = 'ambiguous_theta'
        _tick('total', t_start)
        return None, diag
    if diag['rect_source'] is None:
        diag['rect_source'] = 'prior' if diag['prior_used'] else 'search'

    # 旋转后栅格里的矩形四角 -> 原栅格的 (col, row) -> 占位平面 (a, b)（mm）
    # （任务书 Step 5 的**修正版**：`_search_rect` 保留了 `M`，这里用它反算。）
    #
    # ⚠️ **不能只反算"左上角"就当成角 0**（第一版就是这么写的，实测错 1.2m）：
    # `_rotated` 绕**画布中心**转了 θ（T5b 修正了原先"绕输入栅格中心转"的 bug），
    # 旋转后坐标里的"左上角"映回原栅格时，
    # θ ≈ ±90°/180° 时会落在矩形的**任意一个角**上 —— 合成场景实测 θ = −180°，
    # 反算出来的是 (1200, 1000) 那个角，却被当成 (0, 0)，于是 origin 偏了
    # 一个对角线。**四个角全部反算**，边长与朝向从角点差向量里量出来，
    # 就与 θ 的符号、象限都无关了。
    #
    # ⚠️ **T5b**：`best['M']` 现在带**裁剪偏移**（`_rotated_crop` 把 `- (dx, dy)`
    # 算进平移项了），所以 `Minv` 直接给出的是**全图栅格**的 (col, row) ——
    # 调用方式与改动前完全一致，不需要任何补偿。
    Minv = cv2.invertAffineTransform(best['M'])
    cols = np.array([best['x0'], best['x0'] + w_mm,
                     best['x0'] + w_mm, best['x0']])
    rows = np.array([best['y0'], best['y0'],
                     best['y0'] + h_mm, best['y0'] + h_mm])
    rc = (Minv @ np.stack([cols, rows, np.ones(4)]))[:2].T  # (4, 2) = (col, row)
    # 可选的深度脊线边精修（默认关，见 `_snap_rect_to_ridge` 里那两轮实测）
    diag['snap_edges'] = None
    if opts.get('snap_edges', False):
        t = time.perf_counter()
        ridge = _image_to_raster(_depth_ridge_mm(depth, k), mask.shape,
                                 k, normal, info, info['h_deck'])
        rc, diag['snap_edges'] = _snap_rect_to_ridge(rc, ridge)
        _tick('snap', t)
    # 栅格 (col, row) -> 占位平面 mm：col = a - p0、row = b - q0，
    # 而 (a, b) 又是**相对占位平面原点** `info['origin']` 的。三者一个都不能少 ——
    # `_rasterise` 的第二趟把 origin 挪到了点云质心，它**不是**相机原点。
    plane = (np.asarray(info['origin'], float)[None, :]
             + (rc[:, 0] + float(info['p0']))[:, None] * frame_ph['E1'][None, :]
             + (rc[:, 1] + float(info['q0']))[:, None] * frame_ph['E2'][None, :]
             + float(info['h_deck']) * normal[None, :])
    # 角 0 -> 角 1 是 w_mm（长边），角 1 -> 角 2 是 h_mm（短边）
    d_long = plane[1] - plane[0]
    d_short = plane[2] - plane[1]
    E_long = d_long / np.linalg.norm(d_long)
    E_short = d_short / np.linalg.norm(d_short)
    # --- 定向：`long_side_parallel` **只决定 W/H 哪个是长边**，不碰 E1/E2 的朝向 ---
    #
    # 2026-09-30 操作员裁决：**x/y 坐标轴的方向必须与这个键无关** ——
    # 调伺服参考边时不想每次再去找哪边是 x 轴。所以 E1 恒指画面右、E2 恒指向上，
    # 这个键**只翻转 W/H 这两个数**（= "拟合的托盘尺寸方向"），轴一步不动。
    #
    # 先量**哪条边是"横向"**（与画面近平行）—— 与 `_theta_ref` 用同一个几何量。
    # 然后把"横向边"放进 `rough['E1']`：这样 `_orient` 挑出的 `E2_new`（指向上
    # 的那条）正好是 `rough['E2']`，**它的 W/H 换位分支不会触发**，`W_cfg` 原样
    # 落到返回值的 `W` 上。不这么做的话 `_orient` 会把 W/H 再翻一次，净效果反掉。
    _angs = []
    for _vec in (E_long, E_short):
        _pr = _project_axis(dict(origin=plane[0]), _vec, k)
        if _pr is None:
            _a = 90.0
        else:
            _a = abs(float(np.degrees(np.arctan2(_pr[1], _pr[0]))))
        _angs.append(min(_a, 180.0 - _a))
    _horiz_is_long = _angs[0] <= _angs[1]
    E_h, E_v = (E_long, E_short) if _horiz_is_long else (E_short, E_long)
    W_cfg = long_mm if long_side_parallel else short_mm
    H_cfg = short_mm if long_side_parallel else long_mm
    rough = dict(origin=plane[0], E1=E_h, E2=E_v, nrm=normal,
                 W=W_cfg, H=H_cfg)

    t = time.perf_counter()
    # origin = 画面左下角，E2 指向上，E1 指画面右 —— **与长短边无关**
    found = _orient(rough, W_cfg, H_cfg, k)
    _tick('orient', t)
    # ⚠️ **W/H 已经在 `_orient` 里定好了**（它知道 E2_new 落在哪条物理边上），
    # 这里**不要再按投影边长改一次**。旧代码在这里拿 `_project_px` 量 e1_px/e2_px
    # 定长边 —— 透视缩短会把顺序颠倒，把 `_orient` 刚判对的又改回错的：
    # 现场帧实测长边 1200mm 投影 403px、短边 1000mm 294px（这次对），
    # 而另一次 1200mm 只投出 322px、1000mm 投出 495px（判反，发布四角整体转 90°）。
    # 判长边要用平面内的基向量点积（`_orient` 里就是这么做的），不能用投影像素。

    t = time.perf_counter()
    # ⚠️ **这里喂的是 `_deck_mask` 的稀疏栅格掩码，不是主路径那张稠密掩码**
    # （2026-09-30 实测）。两者是**不同的量具**，不能互换：
    #   * 稠密掩码（`_rect_from_image` 里那张）要的是**形状统计**，所以必须是
    #     图像空间净化过的、完整的一块；
    #   * `_edge_support` 要的是"**这条边附近有没有台面材料**"，探针只有
    #     ±`edge_probe_px`(25)px。稠密掩码要求 `depth > 0`，而现场帧**图像
    #     v>=476 没有有效深度**（托盘近边压在图像下边界上），掩码在竖直方向被
    #     截断：b 跨度 1160mm vs 矩形的 1290mm，只有 90%。于是预测边的近端落在
    #     掩码外 60~70mm，探针够不到 —— 实测 support 从 4/4 掉到 2/4，而且剩下
    #     的两条是**对边**（边1/边3），不满足"相邻两条"，整帧被判 `too_few_edges`。
    edges = _edge_support(found, found['W'], found['H'], mask, k, info, opts)
    _tick('edges', t)
    diag['edges'] = edges
    diag['n_observed_edges'] = sum(1 for e in edges if e['observed'])
    # 检测下限：至少两条**相邻**边（规格 §5.1）
    fams = [i for i, e in enumerate(edges) if e['observed']]
    adjacent = any((fams[j] + 1) % 4 in fams or (fams[j] - 1) % 4 in fams
                   for j in range(len(fams)))
    if diag['n_observed_edges'] < 2 or not adjacent:
        diag['reject'] = 'too_few_edges'
        _tick('total', t_start)
        return None, diag

    # ⚠️ **`degenerate` 判据 2026-09-30 删掉了**（原来在这里：矩形贴到栅格边界
    # 就拒帧）。两个理由，都是实测的：
    #   1. 现场那台机器上它**一直在开火** —— 日志里 `reject=degenerate` 从第 16 帧
    #      起每帧一条、`score` 稳定 0.60（掩码是好的，只是矩形伸出了栅格）。
    #   2. 栅格是**点云 bbox**，而现场图像 `v>=476` 没有有效深度（托盘近边压在
    #      图像下边界上）—— 点云到不了托盘近边，**栅格比托盘实际范围小，
    #      正确的矩形必然伸出去**。所以这条判据对正确的解也开火，它拦不住错的、
    #      只会拦掉对的。
    # 它本来想抓的"搜索窗开小了"由 `prior` 路径的 `window_saturated` 覆盖，
    # 而全画布搜索**没有搜索窗**这个概念 —— 判据在这里本来就没有对象。
    diag['origin_visible'] = bool(edges[0]['observed'] and edges[3]['observed'])
    diag['W_mm'], diag['H_mm'] = float(found['W']), float(found['H'])
    # 长短边哨兵：拿**图像空间**量出的长边方向当参照（与帧的构造完全独立），
    # 差 ~90° 就是 E1/W 错配那一类（`_orient` 的 docstring 记过一次）。
    #
    # ⚠️ 只在首帧/重捕帧跑得了 —— `theta_img` 只有"没有先验"时才算（省那 ~250ms），
    # 先验帧没有它。首帧正是风险最高的一帧（它的结果会被后面所有帧当先验），
    # 所以这个覆盖面是有意的。
    if theta_img is not None:
        d_ls = _long_short_angle_diff(found, normal, float(np.degrees(theta_img)))
        diag['long_short_diff_deg'] = round(d_ls, 2)
        diag['long_short_mismatch'] = bool(d_ls > 45.0)
    _tick('total', t_start)
    return found, diag


