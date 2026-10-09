#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AprilTag → 木托盘：标定与反识别。

这套方案：场景里固定两个 AprilTag（贴在托盘附近的支架上），**先标定出
Tag 与木托盘之间的刚体变换**，之后每一帧由 Tag 位姿反推出木托盘位姿。

标定与运行时是两半：

  标定（离线，一次）—— 操作员在若干帧上手工点击托盘台面四角，配合深度图
      得到每帧的「传感器系 ← 托盘」，同时取该帧的「传感器系 ← Tag」，两者
      相除即得 `T_pallet_tag`。**每个 tag 各得一个矩阵**，不能合成一个：两个
      tag 相距数百毫米，`inv(T_sensor_pallet) @ T_sensor_tag_i` 对 i 是不同的
      刚体变换，合成一个会把原点放到两个 tag 中间、任何 tag 都不在的地方，
      每帧的托盘位姿就整体偏掉半个基线。要融合也是下一步在「传感器系 ←
      托盘」这一层融合，那时每条路径本就该给出同一个答案。

  反识别（在线，每帧）—— 每个可见 tag 各给一个「传感器系 ← 托盘」，再融合。
      可用时间窗抑制 tag 自身的角度抖动。

本模块**只做数学**：不 import 框架的任何东西，不碰 ROS、硬件、黑板。
内参以裸的 fx/fy/cx/cy 传入，位姿以 4×4 齐次矩阵传入传出。这样它可以脱离
一切环境做回归测试；与框架的对接（TagDetection / Pose6D / 黑板）在上层的
skill.py 和 node 里完成。

三个必须知道的点：

1. **变换方向。** 本模块的 `T_pallet_tag` 表示 pallet ← tag，即 `p_pallet =
   T @ p_tag`。运行时的式子是 `T_sensor_pallet = T_sensor_tag @ inv(T_pallet_tag)`。
   注意框架 `core/common/transform.py` 的 `transform_pose(pose, M)` 等于
   `M @ pose`，乘法顺序与这里相反，直接套用会**静默**给出转置的结果。本模块
   全程用矩阵乘法，不经过 `transform_pose`。

2. **`T_pallet_tag` 与坐标系无关。** 它是托盘和 tag 两个物体之间的相对位姿，
   `inv(T_sensor_pallet) @ T_sensor_tag` 里的「传感器系」被约掉了。所以标定时
   用相机系算出来的矩阵，运行时可以直接吃 base_link 系的 tag 位姿，**不需要
   任何转换、也不需要重标**。唯一的要求是被相除的两项在同一个系里。

3. **欧拉角不用碰。** 框架 `Pose6D` 的文档说顺序是 yaw-pitch-roll (ZYX)、
   `transform.py` 写的是 `from_euler('xyz', [roll, pitch, yaw])`，两者数学上
   等价（extrinsic XYZ ↔ intrinsic ZYX），但措辞互相矛盾。本模块内部一律用
   4×4 矩阵，只在最外层与 `Pose6D` 互转，把这个歧义关在门外。

参考实现来自 maduo 项目的 `calibrate_pallet_tag.py`，算法逐位保持一致，
以便用那套数据做回归。

运行与测试
----------
本模块是纯函数库，没有 `__main__`，不单独运行。测试分三层，都在 LeTools
仓库根目录下执行：

    # 1. 算法层 + 技能层：不需要硬件、不需要 ROS
    python3 apps/test_kuavo_5w_skills/test_pallet_pose.py

    # 2. 编排层节点单测：CI 会跑这一条
    #    （.gitlab-ci.yml 的 verify:opensource 里 pytest orchestration/nodes/tests/ -m unit）
    pytest orchestration/nodes/tests/test_node_pallet_pose.py -m unit -v

    # 3. 标定工具：要真机、要两个 tag 都在视野里
    python3 apps/test_camera_internal/pallet_calibration/pallet_calibrate.py --help

第 1 条里含一组回归：拿 maduo 项目 `gt/april_test7/pallet_tag_tf.json` 与
`report.json` 里已经导出的数据重算一遍，逐位比对。算法是从那边搬过来的，
同样的输入必须给出同样的输出；这一步是这次合并唯一能证明"没搬错"的手段。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 深度采样：一个 3x3 的邻域里少于这么多有效点就不信它
MIN_VALID_PIXELS = 9
# 邻域内 p90-p10 超过这个值，说明窗口横跨了两个面（板条缝、物体边缘），
# 取中位数会落在两个面中间，宁可判为无效
MAX_SPREAD_MM = 80.0

# 时间窗平滑的默认值。这是操作员在 maduo 里逐帧看图选出来的，见 smooth_series。
DEFAULT_WINDOW = 15
DEFAULT_SMOOTH_MODE = "savgol"
DEFAULT_SMOOTH_ORDER = 1


# --------------------------------------------------------------------------- #
# 基础几何
# --------------------------------------------------------------------------- #
def average_rigid_transforms(matrices: Sequence[np.ndarray]) -> np.ndarray:
    """多个位姿估计的平均：平移取算术平均，旋转取正交化平均。

    用 `u @ vt` 而**不是** `u @ diag(1,1,det(uv)) @ vt`：后者会把结果强行掰成
    det = +1 的旋转，如果输入本身带反射就会被镜像掉。这里没有理由假设输入
    一定是右手系，保留输入的 det 符号才是忠实的平均。
    """
    out = np.eye(4, dtype=np.float64)
    out[:3, 3] = np.mean([m[:3, 3] for m in matrices], axis=0)
    u, _, vt = np.linalg.svd(np.sum([m[:3, :3] for m in matrices], axis=0))
    out[:3, :3] = u @ vt
    return out


def rotation_angle_deg(R: np.ndarray) -> float:
    """旋转矩阵对应的转角（度）。"""
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))


def fit_plane(points: np.ndarray):
    """最小二乘平面。返回 (单位法向, 质心)；点近共线时返回 (None, None)。"""
    centroid = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points - centroid, full_matrices=False)
    if singular[0] <= 1e-12 or singular[1] / singular[0] < 1e-6:
        return None, None                       # 近共线：定不出平面
    return vt[-1], centroid


def sample_depth_mm(depth: np.ndarray, u: float, v: float,
                    depth_scale: float = 1.0,
                    strict_radius: int = 6, search_radius: int = 16) -> Optional[float]:
    """取某个像素的深度，单位毫米，容忍深度图上的空洞。

    点击落在深度不连续处（物体边缘、遮挡阴影、高光）时，固定窗口会取不到
    完整有效值，于是逐步扩大窗口、只对有效像素取中位数。窗口一旦横跨两个面
    （spread 超阈值）就判为不可信，返回 None —— 那种情况下中位数落在两个面
    中间，比没有值更糟。

    `depth_scale` 把深度图原始值换算成毫米。框架的 `DepthData.scale` 就是指
    这个，不同相机出来的单位不一样，不能假设是毫米。
    """
    h, w = depth.shape[:2]
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return None

    def valid_window(radius: int):
        y0, y1 = max(0, vi - radius), min(h, vi + radius + 1)
        x0, x1 = max(0, ui - radius), min(w, ui + radius + 1)
        block = depth[y0:y1, x0:x1].astype(np.float64)
        block = block[block > 0]
        return block

    strict = valid_window(strict_radius)
    if strict.size >= MIN_VALID_PIXELS:
        spread = float(np.percentile(strict, 90) - np.percentile(strict, 10))
        if spread <= MAX_SPREAD_MM / depth_scale:
            return float(np.median(strict)) * depth_scale

    for radius in range(1, search_radius + 1):
        block = valid_window(radius)
        if block.size < MIN_VALID_PIXELS:
            continue
        spread = float(np.percentile(block, 90) - np.percentile(block, 10))
        if spread <= MAX_SPREAD_MM / depth_scale:
            return float(np.median(block)) * depth_scale
    return None


def order_quad(uv: np.ndarray) -> np.ndarray:
    """把四个点击点排成绕质心的环序，并从最右边的点起头。

    ⛔ **这是死代码，而且它代表的做法是错的 —— 不要拿它来"修点击顺序"。**

    全仓库 `grep -rn order_quad` 只有**定义与导出，零调用点**：
    `pallet_frame_from_clicks()` 用的是**原始点击顺序**（`points[0]` 是原点、
    `points[1]` 定 `e1`、`points[3]` 定 `e2`），标定工具与离线工具都是把原始顺序
    直接喂进去的。

    错在哪：本函数按**图像上最靠右的点**起头，而"最右"取决于**相机视角** ——
    相机一动原点就可能换角，托盘系跟着翻。伺服要的是一个按**物理角**定死的系。

    **正确做法**是操作员固定按 `右下 → 左下 → 左上 → 右上` 点，再由
    `skills/atomic/perception/pallet_servo/algorithm.py` 的
    `PALLET_CLICK_PERMUTATION` / `reorder_pallet_clicks()` 做**固定置换**
    （不是几何启发式）。那份常量连同"为什么非重排不可"的完整理由都在那个文件里。

    保留本函数只为不制造无谓的改动（本方案不改 `pallet_pose` 的既有已合并工作）。
    """
    centred = uv - uv.mean(axis=0)
    order = np.argsort(np.arctan2(centred[:, 1], centred[:, 0]))
    return np.roll(order, -int(np.argmax(uv[order][:, 0])))


# --------------------------------------------------------------------------- #
# 托盘台面坐标系
# --------------------------------------------------------------------------- #
@dataclass
class PalletFrame:
    """托盘台面坐标系，表示在采集它的那个传感器系里。

    `origin` 是第一个点击角（毫米）；`e1` / `e2` 沿台面两条边，`normal` 从台面
    指向相机。`origin` 取第一个角而**不是**质心：质心会让一半的台面落在
    `a < 0` 的区域里，下游按 `[0, W]` 取点时会把它整个丢掉。
    """

    origin: np.ndarray      # (3,) mm
    e1: np.ndarray          # (3,) 单位向量
    e2: np.ndarray          # (3,) 单位向量
    normal: np.ndarray      # (3,) 单位向量
    width_mm: int
    height_mm: int

    def to_matrix(self) -> np.ndarray:
        """4×4 的「传感器系 ← 托盘系」。平移换算成米，与框架的 Pose6D 一致。"""
        T = np.eye(4, dtype=np.float64)
        T[:3, 0] = self.e1
        T[:3, 1] = self.e2
        T[:3, 2] = self.normal
        T[:3, 3] = np.asarray(self.origin, np.float64) / 1000.0
        return T


def pallet_frame_from_clicks(uv: Sequence[Sequence[float]], depth: np.ndarray,
                             fx: float, fy: float, cx: float, cy: float,
                             depth_scale: float = 1.0) -> Optional[PalletFrame]:
    """四个托盘台面点击 + 深度图 → 托盘台面坐标系。

    先把每个点击按该处深度反投影成三维点，四点拟合平面，再在平面内取两条
    边。任何一个点击取不到可信深度就整体返回 None —— 四点定平面缺一不可，
    少一个点得到的平面方向会明显偏。
    """
    points = []
    for u, v in uv:
        mm = sample_depth_mm(depth, u, v, depth_scale)
        if mm is None:
            return None
        z = mm / 1000.0
        points.append([(u - cx) * z / fx, (v - cy) * z / fy, z])
    points = np.array(points, np.float64)

    normal, centroid = fit_plane(points)
    if normal is None:
        return None
    if normal[2] > 0:
        normal = -normal                # 指向相机一侧（相机在台面上方）

    def in_plane(vector: np.ndarray) -> np.ndarray:
        return vector - float(vector @ normal) * normal

    e1 = in_plane(points[1] - points[0])
    e2 = in_plane(points[3] - points[0])
    n1, n2 = np.linalg.norm(e1), np.linalg.norm(e2)
    if n1 < 1e-9 or n2 < 1e-9:
        return None
    e1 /= n1
    e2 /= n2
    origin = points[0] - float((points[0] - centroid) @ normal) * normal
    return PalletFrame(origin=origin * 1000.0, e1=e1, e2=e2, normal=normal,
                       width_mm=int(round(n1 * 1000.0)),
                       height_mm=int(round(n2 * 1000.0)))


# --------------------------------------------------------------------------- #
# 标定：T_pallet_tag
# --------------------------------------------------------------------------- #
@dataclass
class TagObs:
    """一帧里的一个观测：同一个传感器系下的 tag 位姿与托盘系位姿。

    `pallet_size_mm` 是该帧点击算出来的台面尺寸 `(W, H)`，标定会把它聚合起来
    ——伺服的投影需要它，而它本来就藏在 `PalletFrame.width_mm/height_mm` 里，
    只是原来没往上传。
    """

    tag_id: int
    T_sensor_tag: np.ndarray        # 4×4，传感器系 ← tag
    T_sensor_pallet: np.ndarray     # 4×4，传感器系 ← 托盘
    frame: str = ""
    pallet_size_mm: Optional[Sequence[float]] = None


@dataclass
class CalibrationResult:
    """标定输出。`by_tag` 是每个 tag 各一个 T_pallet_tag（pallet ← tag）。"""

    by_tag: Dict[int, np.ndarray] = field(default_factory=dict)
    self_check: Dict[str, object] = field(default_factory=dict)
    n_estimates: int = 0
    pallet_size_mm: Optional[Tuple[int, int]] = None


def calibrate_pallet_tag(observations: Sequence[TagObs]) -> CalibrationResult:
    """多帧多 tag 的观测 → 每个 tag 一个 `T_pallet_tag`（pallet ← tag）。

    自检分两件事，因为它们是两个不同的问题：

      * **重复性** —— 同一个 tag 自己的多个估计彼此是否一致。这才是标定质量
        的度量。
      * **跨 tag 一致性** —— 用每个 tag 单独推出「传感器系 ← 托盘」，两条
        路径给出的托盘位姿是否重合。运行时每帧做的就是这件事，标定时就该
        用同样的方式量它。

    注意**不要**用 tag0→tag1 的相对位姿来查一致性：它的平移是两个 tag 的间距
    绕托盘原点转过去的结果，被一米多的力臂污染，真值 810 mm 的间距会读成
    700 多毫米。要比就比托盘原点。
    """
    by_tag: Dict[int, List[np.ndarray]] = {}
    by_frame: Dict[str, Dict[int, np.ndarray]] = {}
    by_frame_tag: Dict[str, Dict[int, np.ndarray]] = {}
    for obs in observations:
        T = np.linalg.inv(obs.T_sensor_pallet) @ obs.T_sensor_tag
        by_tag.setdefault(obs.tag_id, []).append(T)
        by_frame.setdefault(obs.frame, {})[obs.tag_id] = T
        by_frame_tag.setdefault(obs.frame, {})[obs.tag_id] = obs.T_sensor_tag

    result = CalibrationResult(n_estimates=len(observations))
    if not by_tag:
        result.self_check = {"error": "no observations"}
        return result

    # 重复性
    per_tag: Dict[int, np.ndarray] = {}
    spread: Dict[int, dict] = {}
    for mid, mats in sorted(by_tag.items()):
        mean = average_rigid_transforms(mats)
        per_tag[mid] = mean
        origins = np.array([T[:3, 3] for T in mats])
        offsets = np.linalg.norm(origins - origins.mean(axis=0), axis=1)
        rot_mean = _orthogonal_mean([T[:3, :3] for T in mats])
        angles = np.array([rotation_angle_deg(rot_mean.T @ T[:3, :3]) for T in mats])
        spread[mid] = dict(n=len(mats),
                           pos_sd_mm=float(offsets.std() * 1000.0),
                           pos_max_mm=float(offsets.max() * 1000.0),
                           rot_mean_deg=float(angles.mean()),
                           rot_max_deg=float(angles.max()))

    # 台面尺寸：**每次点击一个样本**，取中位数。取中位数不取均值，是因为点击
    # 偶尔会点到台面外一点、给出一个偏大的尺寸，中位数对它免疫。
    #
    # ⚠️ **必须按"点击/帧"去重，不能按观测收。** 一帧看见两个 tag 就有两个
    # TagObs，而 `pallet_size_mm` 是按帧挂在该帧**每一个** TagObs 上的
    # （`pallet_calibrate.py` 对 by_id 里每个 tag 都挂同一个 (W,H)）。直接按观测
    # 收，就是让那一帧投两次票——一旦某帧只看见一个 tag（真实重标里很常见），
    # 中位数就变成**加权中位数**，而 maduo 侧是按帧取的，两边都不报错却会静默
    # 分叉。同一帧的多个观测共享同一个 `T_sensor_pallet`，拿它做去重键。
    seen_sizes: dict = {}
    missing = 0
    for obs in observations:
        if obs.pallet_size_mm is None:
            missing += 1
            continue
        key = (tuple(np.asarray(obs.T_sensor_pallet, np.float64).ravel().tolist())
               if obs.T_sensor_pallet is not None else id(obs))
        seen_sizes.setdefault(
            key, tuple(int(round(float(v))) for v in obs.pallet_size_mm[:2]))
    sizes = list(seen_sizes.values())
    # `n` 是**点击次数**（去重后），`missing` 是没带尺寸的**观测数** —— 两者单位
    # 不同，所以这里都写明，免得读的人拿 n 跟 --frames 对不上时以为出错。
    size_spread: dict = {"n": len(sizes), "n_observations": len(observations),
                         "missing": missing}
    if sizes:
        arr = np.asarray(sizes, np.float64)
        result.pallet_size_mm = (int(round(float(np.median(arr[:, 0])))),
                                 int(round(float(np.median(arr[:, 1])))))
        size_spread.update(w_median_mm=float(np.median(arr[:, 0])),
                           h_median_mm=float(np.median(arr[:, 1])),
                           w_sd_mm=float(arr[:, 0].std()),
                           h_sd_mm=float(arr[:, 1].std()),
                           w_min_mm=float(arr[:, 0].min()),
                           w_max_mm=float(arr[:, 0].max()),
                           h_min_mm=float(arr[:, 1].min()),
                           h_max_mm=float(arr[:, 1].max()))
    result.by_tag = per_tag

    pooled_sd = float(np.mean([s["pos_sd_mm"] for s in spread.values()]))
    pooled_rot = float(np.mean([s["rot_mean_deg"] for s in spread.values()]))

    # 跨 tag 一致性：每条路径各自推出「传感器系 ← 托盘」，比托盘原点
    cross_mean = cross_max = cross_rot = 0.0
    cross_n = 0
    if len(per_tag) >= 2:
        gaps, rots = [], []
        for _stem, tagmap in by_frame.items():
            poses = [by_frame_tag[_stem][mid] @ np.linalg.inv(per_tag[mid])
                     for mid in tagmap if mid in per_tag]
            if len(poses) < 2:
                continue
            ref = poses[0]
            for other in poses[1:]:
                gaps.append(float(np.linalg.norm(ref[:3, 3] - other[:3, 3]) * 1000.0))
                rots.append(rotation_angle_deg(ref[:3, :3].T @ other[:3, :3]))
        if gaps:
            cross_n = len(gaps)
            cross_mean, cross_max = float(np.mean(gaps)), float(np.max(gaps))
            cross_rot = float(np.mean(rots))

    result.self_check = dict(
        n_estimates=len(observations),
        n_frames=len(by_frame),
        n_tags=len(by_tag),
        pooled_pos_sd_mm=pooled_sd,
        pooled_rot_deg=pooled_rot,
        per_tag_spread={str(k): v for k, v in spread.items()},
        pallet_size_spread_mm=size_spread,
        cross_tag=dict(n=cross_n, mean_mm=cross_mean, max_mm=cross_max,
                       rot_mean_deg=cross_rot),
    )
    return result


def _orthogonal_mean(matrices: Sequence[np.ndarray]) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.sum(matrices, axis=0))
    return u @ vt


# --------------------------------------------------------------------------- #
# 反识别：tag 位姿 → 托盘位姿
# --------------------------------------------------------------------------- #
def pallet_pose_from_tag(T_sensor_tag: np.ndarray,
                         T_pallet_tag: np.ndarray) -> np.ndarray:
    """单个 tag → 「传感器系 ← 托盘」。

    `T_sensor_tag @ inv(T_pallet_tag)`：前者是 sensor ← tag、后者是 tag ← pallet，
    首尾相接正好是 sensor ← pallet。**顺序不能反** —— 反过来乘是另一个量纲，
    而且不会报错。
    """
    return np.asarray(T_sensor_tag, np.float64) @ np.linalg.inv(T_pallet_tag)


def fuse_pallet_poses(poses: Sequence[np.ndarray]) -> Optional[np.ndarray]:
    """把若干条「传感器系 ← 托盘」融合成一个。

    融合放在这一层（而不是把各 tag 的 T_pallet_tag 先合成一个）是有意的：
    两条路径本来就该给出同一个托盘位姿，这里平均的是它们的答案。两个 tag
    的噪声互相独立时能拿到约 1/sqrt(2) 的降噪；如果它们犯同一个错（比如
    标定期的共同系统误差），平均掉不掉。
    """
    live = [p for p in poses if p is not None]
    if not live:
        return None
    if len(live) == 1:
        return np.asarray(live[0], np.float64)
    return average_rigid_transforms(live)


def pallet_pose_from_tags(tag_poses: Dict[int, np.ndarray],
                          T_pallet_tag: Dict[int, np.ndarray]) -> Optional[np.ndarray]:
    """一帧的多个 tag → 托盘位姿。缺标的 tag 直接跳过。"""
    poses = [pallet_pose_from_tag(tag_poses[mid], T_pallet_tag[mid])
             for mid in tag_poses if mid in T_pallet_tag]
    return fuse_pallet_poses(poses)


# --------------------------------------------------------------------------- #
# 时间窗平滑
# --------------------------------------------------------------------------- #
def smooth_series(poses: List[Optional[np.ndarray]], mode: str = DEFAULT_SMOOTH_MODE,
                  width: int = DEFAULT_WINDOW,
                  order: int = DEFAULT_SMOOTH_ORDER) -> List[Optional[np.ndarray]]:
    """对一串位姿逐帧输出平滑结果；输入里的 None（该帧没 tag）原样跳过。

    `savgol` 在窗口均值位姿的切空间里对（平移 + 旋转向量）做低阶多项式最小
    二乘，再把它代回该帧。这样它像均值一样降噪，却是**跟着**运动走而不是
    拖在后面。窗口宽度和阶数是操作员在 maduo 里逐帧看图选出来的（w=15、
    阶=1，胜过了尾随均值、居中均值和二次拟合），不是自动指标选出来的 ——
    那几个自动判据都被验证是循环论证。

    注意时窗只能压掉**快变**的抖动。这套系统里占大头的误差是随视角慢变的
    系统误差，任何窗都压不掉它。
    """
    if mode == "raw":
        return list(poses)
    n = len(poses)
    out: List[Optional[np.ndarray]] = []
    for i in range(n):
        if mode == "trailing":
            a, b = max(0, i - width + 1), i + 1
        else:
            a, b = max(0, i - width // 2), min(n, i + 1 + (width - 1) // 2)
        idx = [j for j in range(a, b) if poses[j] is not None]
        if not idx:
            out.append(None)
            continue
        if mode in ("trailing", "centred") or len(idx) < order + 2:
            out.append(average_rigid_transforms([poses[j] for j in idx]))
            continue
        ref = average_rigid_transforms([poses[j] for j in idx])
        ref_inv = np.linalg.inv(ref)
        t = np.array([j - i for j in idx], np.float64)
        tr = np.array([(ref_inv @ poses[j])[:3, 3] for j in idx])
        rv = np.array([cv2.Rodrigues((ref_inv @ poses[j])[:3, :3])[0].ravel()
                       for j in idx])
        fit = np.zeros(6)
        for c in range(3):
            fit[c] = np.polyval(np.polyfit(t, tr[:, c], order), 0.0)
            fit[3 + c] = np.polyval(np.polyfit(t, rv[:, c], order), 0.0)
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(fit[3:])[0]
        T[:3, 3] = fit[:3]
        out.append(ref @ T)
    return out
