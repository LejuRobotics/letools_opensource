# -*- coding: utf-8 -*-
"""`pallet_detect` 合成场景的共享夹具：`make_truth()` / `render()` / `K` / 假 OBB session。

**逐字抄自**（已整体删除的）`tests/test_pallet_detect.py` 的 63~209 行 —— 旧路线
删除时，这两个纯几何夹具（自己合成台面 + 地面、自己造真值）**不依赖被删的算法**，
`tests/test_obb_detect.py` 仍要用它们，所以原样搬出来单独放。

⚠️ 与 maduo 侧的合成渲染器是同一套；改这里要对照 maduo 的渲染器。
"""
from __future__ import annotations

from types import SimpleNamespace

import cv2
import numpy as np

from ..geom import _corners_mm, _project_px

W_MM, H_MM = 1200.0, 1000.0
FX = FY = 400.0
CX, CY = 320.0, 240.0
IMG_H, IMG_W = 480, 640


class K:
    fx, fy, cx, cy = FX, FY, CX, CY


def quad_px(frame: dict, w: float = W_MM, h: float = H_MM) -> np.ndarray:
    """`frame` 台面四角在 `K` 下的像素位置。"""
    return _project_px(_corners_mm(frame, w, h), K)


def _unit(v):
    return v / np.linalg.norm(v)


def make_truth() -> dict:
    """一个已知真值的台面系。

    ⚠️ 朝向必须**符合操作员 2026-09-22 的约定**（规格 §3）：
      origin = 画面左下角，E1 = 与画面近平行的边、指向画面右，E2 = 指向上。
    相机系里图像右 = +x、图像下 = +y，所以：
      E1 的 x 分量 > 0（向右），E1 的 y 分量 ~ 0（与画面平行）；
      E2 的 y 分量 < 0（向上）。
    """
    nrm = _unit(np.array([0.05, -0.40, -0.915]))          # 指向相机
    e1 = _unit(np.array([1.0, 0.0, 0.0]) - (np.array([1.0, 0.0, 0.0]) @ nrm) * nrm)
    if e1[0] < 0:                                          # 保证指向画面右
        e1 = -e1
    e2 = _unit(np.cross(nrm, e1))
    if e2[1] > 0:                                          # 保证指向上（图像 v 更小）
        e2 = -e2
    return dict(origin=np.array([-600.0, -300.0, 1600.0]), E1=e1, E2=e2, nrm=nrm,
                W=W_MM, H=H_MM)


def _quad_px(frame: dict, w: float = W_MM, h: float = H_MM) -> np.ndarray:
    return _project_px(_corners_mm(frame, w, h), K)


def _near_edge_px(frame: dict, w: float = W_MM, h: float = H_MM) -> np.ndarray:
    """`y=0` 那条边（角 0 -> 角 1）在图像里的两个端点 `(2, 2)`。

    与 `pallet_servo.algorithm.ref_edge_px(frame, size, 'y=0', K)` 是同一个量，
    但那个函数在隔壁模块（伺服那条链），本文件不该为了画一条边去 import 它。
    `_corners_mm` 的顺序是 `(0,0) -> (W,0) -> (W,H) -> (0,H)`，所以角 0/1 之间
    正好是 `y=0` 那条边。
    """
    return _project_px(_corners_mm(frame, w, h)[0:2], K)


def mask_from_truth(truth: dict, cover_quads=()) -> np.ndarray:
    """真值台面掩码：矩形内为 1，`cover_quads` 覆盖掉的部分为 0（模拟箱子遮挡）。

    ⚠️ 只用来测**纯几何**（T2–T4）。它绕开了木色分割，所以这里通过不代表
    真实数据上能过 —— 真实数据要等 T6。
    """
    m = np.zeros((IMG_H, IMG_W), np.uint8)
    cv2.fillPoly(m, [_quad_px(truth).astype(np.int32)], 1)
    for q in cover_quads:
        cv2.fillPoly(m, [np.asarray(q, np.int32)], 0)
    return m.astype(bool)


def render(truth: dict, cover_quads=(), floor_drop_mm: float = 900.0,
           ground_half_mm: float | None = None, box_raise_mm: float = 300.0):
    """出 (color, depth)。台面刷木色，背景刷成**低 floor_drop_mm 的地面**。

    ⚠️ 背景**不能**和台面共面。深度路线（规格 §4 Step 2）就是靠高度分台面与背景的，
    两者共面时它必然失败 —— 那不是算法错，是这个合成场景没造对。
    地面用台面平面沿法向**下移** floor_drop_mm 得到（托盘本身有厚度，
    真实场景里地面就在台面下方）。

    ⚠️ 台面色是 `BGR(93, 145, 190)`（HSV `(16, 130, 190)`）—— T1 报出的问题 1：
    原色 `BGR(60, 130, 190)` 的 S=174 超出 `gd.WOOD_HSV` 上界 145，`_wood_mask`
    命中 0 像素。背景灰 `(150,150,150)` 与箱子色 `(170,90,40)` 实测都不会被
    误判成木色（箱子色 S=195 超出 `WOOD_HSV` 上界 145，被 `_wood_mask` 的
    `inRange` 排除），不用改。

    `ground_half_mm` —— 地面**半边长**，决定这块地面在场景里有多大：

      - `None`（**默认**，保持 T1~T3 的行为不变）：地面铺满整幅图，
        即除了托盘以外的每个像素都打在地面平面上。
      - 数值 `N`：地面平面上只放一块边长 `2*N` mm 的**方块**，方块中心在
        **托盘中心沿法向下移 floor_drop_mm 处**，即整块地面正好在托盘脚下、
        且完全落在托盘的正下方（`2*N` < 托盘短边 1000mm 时）。
        方块以外的像素深度为 0（无效）—— 整帧就只剩"托盘 + 它脚下这块地面"
        两个面。这样地面的格数可以**小于**台面的格数，深度路线的判据
        （"取格数最多的一带"）才有机会选对。

    ⚠️ 地面方块的**位置**不是随便摆的：它必须在托盘正下方，否则"地面"会跑到
    托盘外面去，测的就不是"托盘坐在一块比它小的地面上"这件事了。方块的四角
    建在地面平面内、沿 `E1`/`E2` 摆正，再投影到图像。

    ⚠️ **托盘会挡住方块的一部分**：方块整个在托盘正下方，而相机是斜看的，
    所以只有投影落在托盘四边形**外面**的那部分地面才有深度（`render()` 里
    托盘像素写的是台面深度）—— 回到占位栅格后它是一块 `2N x 2N` 方块的
    **可见部分**，不是完整的方块。实测 `ground_half_mm=350` 时栅格 bbox 是
    710 x 448 mm、**6296 格**，对台面的 **40271 格**（这就是"地面比托盘小"）。
    这是物理正确的遮挡（真实场景里托盘下的地面本来就被托盘挡住），
    场景自检会把这个格数打出来。
    """
    color = np.full((IMG_H, IMG_W, 3), (150, 150, 150), np.uint8)   # 灰地面
    cv2.fillPoly(color, [_quad_px(truth).astype(np.int32)], (93, 145, 190))  # 木色 BGR
    for q in cover_quads:
        cv2.fillPoly(color, [np.asarray(q, np.int32)], (170, 90, 40))        # 箱子色

    o, nrm = np.asarray(truth['origin'], float), np.asarray(truth['nrm'], float)
    e1, e2 = np.asarray(truth['E1'], float), np.asarray(truth['E2'], float)
    m = np.zeros((IMG_H, IMG_W), np.uint8)
    cv2.fillPoly(m, [_quad_px(truth).astype(np.int32)], 1)
    on_pallet = m > 0

    def _plane_z(origin):
        d = -float(origin @ nrm)
        out = np.zeros((IMG_H, IMG_W), np.float32)
        for v in range(IMG_H):
            ray = np.stack([(np.arange(IMG_W) - CX) / FX,
                            np.full(IMG_W, (v - CY) / FY), np.ones(IMG_W)], -1)
            den = ray @ nrm
            ok = np.abs(den) > 1e-6
            z = np.where(ok, -d / np.where(ok, den, 1.0), 0.0)
            out[v] = np.where(ok & (z > 100.0) & (z < 8000.0), z, 0.0)
        return out

    gz = _plane_z(o - floor_drop_mm * nrm)
    pal = _plane_z(o).astype(np.float32)
    if cover_quads:
        # ⚠️ **箱子必须比台面更近**（沿 `+nrm` 抬 `box_raise_mm`）。
        # 原来这里**不抬高**（箱子在深度上就是台面），那时的理由是"合成场景里
        # 无法用它验证扣货"—— 但 2026-10-10 把 `_edge_support` 的量具从**颜色
        # 掩码**换成**深度台面层**之后，这条不成立就成了真问题：量具是深度的，
        # 而箱子在深度上与台面同高，于是**被箱子挡住的那条边在深度量具下依然
        # "有料"**，`test_detect_degenerate` 那 8 条遮挡用例全部失效
        # （该拒的不拒、该只剩对边的变成四条边都在）。
        # 真实场景里箱子是坐/放在托盘上的，本来就该抬高 —— 这是**把场景修对**，
        # 不是为了让测试变绿调松判据。
        cz = _plane_z(o + box_raise_mm * nrm)
        cm = np.zeros((IMG_H, IMG_W), np.uint8)
        for q in cover_quads:
            cv2.fillPoly(cm, [np.asarray(q, np.int32)], 1)
        cover, cz_raise = cm > 0, cz
    else:
        cover, cz_raise = None, None
    if ground_half_mm is None:
        depth = np.where(on_pallet, pal, gz).astype(np.float32)
        # ⚠️ 遮挡**要在最后一步盖**：箱子压在托盘上时 `on_pallet` 也为真，
        # 先写 `pal` 再盖箱子才不会被 `np.where(on_pallet, ...)` 顶掉。
        if cover is not None:
            depth = np.where(cover & (cz_raise > 0), cz_raise,
                             depth).astype(np.float32)
    else:
        n_ = float(ground_half_mm)
        cen = (o + 0.5 * float(truth['W']) * e1 + 0.5 * float(truth['H']) * e2
               - floor_drop_mm * nrm)                      # 托盘中心正下方的地面点
        sq3 = np.stack([cen - n_ * e1 - n_ * e2, cen + n_ * e1 - n_ * e2,
                        cen + n_ * e1 + n_ * e2, cen - n_ * e1 + n_ * e2])
        sq = np.zeros((IMG_H, IMG_W), np.uint8)
        cv2.fillPoly(sq, [_project_px(sq3, K).astype(np.int32)], 1)
        depth = np.where(on_pallet, _plane_z(o),
                         np.where(sq > 0, gz, 0.0)).astype(np.float32)
    return color, depth



class FakeObbSession:
    """伪装成 `onnxruntime.InferenceSession`：`run()` 把预置四角"编码"回去。

    用它把 `detect_pallet_frame` 的**模型那一环**换掉（`opts={'session': ...}`），
    专测 `frame_from_obb` + 选框 + 组装的**算法搬迁** —— 真模型的连通性验证是
    另一回事（要真 .onnx）。构造时把目标四角**先映射进 letterbox 的 640 空间**，
    这样 `detect._decode` 减掉补边后能解回原图坐标。

    `run()` 返回的形状是 `(1, 6, 1)`（= 导出图 `(1,6,8400)` 的 N=1 形态），
    `detect` 会 `[0].T` 成 `(1,6) = [cx, cy, w, h, conf, angle]`（角度是**原始弧度**）。
    """

    def __init__(self, corners_px: np.ndarray, color: np.ndarray, conf: float = 0.9,
                 imgsz: int = 640):
        from ..detect import _letterbox
        _, ratio, pad = _letterbox(np.asarray(color), (imgsz, imgsz))
        q = np.asarray(corners_px, float).reshape(4, 2) * float(ratio) + np.asarray(pad)
        c = q.mean(axis=0)
        a, b = q[1] - q[0], q[3] - q[0]
        w_, h_ = float(np.linalg.norm(a)), float(np.linalg.norm(b))
        ang = float(np.arctan2(a[1], a[0]))
        self._row = np.array([[c[0], c[1], w_, h_, float(conf), ang]], float)  # (1,6)

    def get_inputs(self):
        return [SimpleNamespace(name='images')]

    def run(self, _outputs, _feed):
        return [self._row.T[None]]          # (1, 6, 1)


def extract_landmarks(truth: dict, t: float = 0.55) -> np.ndarray:
    """真值台面四角，缩进 `t`（= 介于 `frame_from_obb` 的幅面 0.45~0.55 之间）。

    给假 OBB 用：`frame_from_obb` 的 4 自由度解在"框比托盘略小"时残差较小，
    直接喂真值四角也可以（残差 0），但缩一点更接近真实检测器的行为。
    """
    q = quad_px(truth)
    c = q.mean(axis=0)
    return c + (q - c) * float(t)
