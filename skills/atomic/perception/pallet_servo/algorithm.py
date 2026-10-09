#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""托盘伺服误差：投影、取边、垂距、角度差。

被控量是**图像空间**的三个量：

    e_bottom_px  箱子底边 → 托盘底边（图像投影）的带符号垂距
    e_right_px   箱子右边 → 托盘右边（图像投影）的带符号垂距
    theta_rad    托盘底边 − 箱子底边 的角度差，带符号，正常 ±45°
                 （**屏幕顺时针为正**，定号与那次翻转见 `servo_error` 里的注释）

本模块**只做纯函数**：不 import 框架的任何东西，不碰 ROS、硬件、黑板，也不
记日志。这样它可以脱离一切环境做逐位回归（与 `pallet_pose/algorithm.py` 同一
套理由）。日志是副作用；拒绝原因、warn、诊断量一律作为**返回值**给出，由
上层去记——这正是 `Reject` 存在的理由。

除几何外这里还收着一个**纯参数解析**小函数 `parse_bool_param`：两个节点
（`node_pallet_servo` / `node_inject_servo_input`）都要用它，而它是纯的、
零依赖的，放这里两边共用一个实现，不会各自分叉（为什么不能用 `bool(...)`
顶替，见那个函数的 docstring）。

四个必须知道的点：

1. **单位在这里是分裂的。** 托盘系内部的几何量一律**毫米**，而
   `T_cam_pallet` 的平移是**米**（与 `PalletFrame.to_matrix()` 和框架 Pose6D
   一致）。`project_pallet_points` 负责这一步换算，写错了会静默差 1000 倍。

2. **图像坐标是 `u` 向右、`v` 向下。** 所有像素量都在这个约定下，所以
   "底边"是 `v` 更大的那条边，`atan2(Δv, Δu)` 的正方向在屏幕上是**顺时针**。

3. **直线无向。** 一条边是 p0→p1 还是 p1→p0 是同一条线，角度必须折进半圈
   （`fold_line_angle`）。不折的话，两个视觉上重合的边会算出 178° 的"角度差"。

4. **垂距取中点。** `e = (M_box − foot) · n`，`M_box` 是箱子边的**中点**。两条
   线不平行时"线到线"是一个区间，取端点没有意义，取中点才和伺服的语义一致。

运行与测试
----------
    # 算法层合成测试：不需要硬件、不需要 ROS、不读图
    python3 apps/test_kuavo_5w_skills/test_pallet_servo.py

    # 编排层节点单测：CI 会跑这条
    #   （.gitlab-ci.yml 的 verify:opensource → pytest orchestration/nodes/tests/ -m unit）
    pytest orchestration/nodes/tests/test_node_pallet_servo.py -m unit -v
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np

# 参考边投影后短于这么多像素就判该帧退化。**这是拍出来的初值，没有数据支撑**：
# 它兜的是"投影明显退化"，不是标定出来的阈值。
MIN_EDGE_PX = 20.0
# |theta| 超过这个值只置 warn，不拒绝输出
THETA_WARN_RAD = math.radians(45.0)
# 相机后方判据（米）
_BEHIND_CAMERA_EPS_M = 1e-6

REF_EDGE_SPECS = ("y=0", "y=H", "x=W", "x=0")

# 绝对毫米写法的正则：`y=<数>` / `x=<数>`。**只认这两个轴**（台面是 z=0 的矩形）。
_ABS_EDGE_RE = re.compile(r"^\s*([xy])\s*=\s*(-?\d+(?:\.\d+)?)\s*$")


def parse_edge_spec(spec: str) -> Optional[Tuple[str, str, float]]:
    """一条参考边写法 → `("symbol", 原文, 0.0)` 或 `("abs", 轴, 毫米)`。

    两种写法：

      `y=0` / `y=H` / `x=W` / `x=0`  —— 台面矩形的四条边，跟着 `size_mm` 走
      `y=800` / `x=1150`            —— 与 `y=0` / `x=0` 平行、落在该毫米位置上的线

    非法写法返回 `None`（与 `ref_edge_points_mm` 同一个约定：纯函数不记日志、
    不抛异常，失败作为返回值给出去）。

    ⚠️ **符号写法先查表**，查不到才按数字解析 —— 否则 `y=0` 会被当成绝对写法，
    虽然结果一样，但 `H=0` 这种退化尺寸下两种写法的含义会分叉。

    符号写法没有绝对位置，第三项给 `0.0` 而不是 `nan`：`nan != nan`，带 `nan`
    的元组**不等于它自己**，调用方连"解析结果是不是这个"都断言不了。
    """
    raw = str(spec)
    if raw in REF_EDGE_SPECS:
        return ("symbol", raw, 0.0)
    match = _ABS_EDGE_RE.match(raw)
    if match is None:
        return None
    axis = match.group(1)
    value = float(match.group(2))
    if not math.isfinite(value):
        return None
    return ("abs", axis, value)


# --------------------------------------------------------------------------- #
# 拒绝：纯函数不记日志，失败原因必须作为返回值给出去
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Reject:
    """一次拒绝：哪一项没过、实际值多少、阈值多少。

    只返回 `None` 的话，到了上层就只剩一句"失败了"的日志，无从排查——所以
    `detail` 里必须写**实际值和阈值**，不能只写"失败"。

    用 frozen dataclass 而不是手写类：它是个**值对象**（拿它当"这一个原因"来
    比较、往集合里放都说得通），`frozen=True` 顺带给出 `__hash__`——手写类一旦
    定义了 `__eq__`，Python 会把 `__hash__` 置成 `None`，实例就不可哈希了。
    这也与兄弟模块 `pallet_pose/algorithm.py` 里 `TagObs` / `CalibrationResult`
    的约定一致。
    """

    code: str
    detail: str

    def __str__(self) -> str:
        return f"{self.code}: {self.detail}"


# --------------------------------------------------------------------------- #
# 托盘参考边
# --------------------------------------------------------------------------- #
def ref_edge_points_mm(edge_spec: str, size_mm) -> Optional[
        Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """参考边在托盘台面系里的两个端点（**毫米**）。非法写法返回 None。

    台面在该系里是矩形 `[0, W] × [0, H]`、`z = 0`（见 `PalletFrame`）。
    写法见 `parse_edge_spec` —— 四条边界边，或 `y=<毫米>` / `x=<毫米>` 的绝对线。

    注意"非法"只指**写法**：越界的绝对写法（如 `y=99999`）照样返回一段台面外
    的点。范围校验在 `parse_ref_edges(..., size_mm)` 里，本函数拿不到 `size_mm`
    的约束，管不了这一层。
    """
    try:
        W, H = float(size_mm[0]), float(size_mm[1])
    except (TypeError, IndexError, ValueError):
        return None
    table = {
        "y=0": ((0.0, 0.0, 0.0), (W, 0.0, 0.0)),
        "y=H": ((0.0, H, 0.0), (W, H, 0.0)),
        "x=W": ((W, 0.0, 0.0), (W, H, 0.0)),
        "x=0": ((0.0, 0.0, 0.0), (0.0, H, 0.0)),
    }
    hit = table.get(str(edge_spec))
    if hit is not None:
        return hit
    parsed = parse_edge_spec(edge_spec)
    if parsed is None:
        return None
    _, axis, value = parsed
    # 绝对写法：`y=<毫米>` 是沿 x 展开、落在该 y 上的线；`x=<毫米>` 沿 y 展开。
    # 表里那四条边同理由 `size_mm` 给出 —— 同一条几何线，两种写法给出同一对端点。
    # **别去统一端点顺序**：这里两端点的顺序与表里对应的边一致；顺序本身不影响
    # 垂距（`inward_normal` / `edge_offset` 只用端点与台面中心），但会改
    # `foot_outside` 的判据。
    if axis == "y":
        return ((0.0, value, 0.0), (W, value, 0.0))
    return ((value, 0.0, 0.0), (value, H, 0.0))


def project_pallet_points(T_cam_pallet, points_mm, K,
                          D=None) -> Union[np.ndarray, Reject]:
    """托盘系（**毫米**）的点 → 图像像素，成功给 `(N, 2)` float64。

    `T_cam_pallet` 的平移是**米**（与 `PalletFrame.to_matrix()`、框架 Pose6D
    一致），而这里的点是毫米——所以进旋转之前先乘 0.001。漏掉这一步结果会
    差 1000 倍，而且仍然是一对"看着像像素坐标"的数。

    点落在相机后方（或光心上）时必须拒：那时 `projectPoints` 不会报错，
    它给出的是**镜像的**像素，比没有值更糟。
    """
    T = np.asarray(T_cam_pallet, np.float64)
    if T.shape != (4, 4):
        return Reject("T_shape", f"T_cam_pallet 必须是 4x4，实际 {T.shape}")
    P = np.asarray(points_mm, np.float64).reshape(-1, 3)
    bad = np.argwhere(~np.isfinite(P))
    if bad.size:
        # 实际值必须写进 reason —— 这是"拒绝必须带原因"硬要求的一部分
        return Reject("points_not_finite",
                      f"待投影的点里有 NaN/inf：下标 "
                      f"{sorted(set(int(i) for i in bad[:, 0]))}"
                      f"（共 {len(P)} 个点）")
    P_cam = (T[:3, :3] @ (P * 0.001).T).T + T[:3, 3]
    if np.any(P_cam[:, 2] <= _BEHIND_CAMERA_EPS_M):
        return Reject("behind_camera",
                      f"点在相机后方或落在光心上：z = "
                      f"{np.round(P_cam[:, 2], 4).tolist()} m")

    K = np.asarray(K, np.float64).reshape(3, 3)
    dist = None
    if D is not None:
        dist = np.asarray(D, np.float64).reshape(1, -1)
        if dist.size == 0:
            dist = None
    px, _ = cv2.projectPoints(P_cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                              K, dist)
    return px.reshape(-1, 2).astype(np.float64)


def ref_edge_px(T_cam_pallet, size_mm, edge_spec, K, D=None, image_size=None,
                min_edge_px: float = MIN_EDGE_PX) -> Union[np.ndarray, Reject]:
    """托盘参考边 → 图像上的两个端点 `(2, 2)`。

    `image_size=(w, h)` 给了就顺带查"整条边跑到图外"——**两个端点都出界**才算，
    有一端在框里就还能用（那一端仍然提供有效的垂距基准）。
    """
    pts = ref_edge_points_mm(edge_spec, size_mm)
    if pts is None:
        return Reject("bad_edge_spec",
                      f"参考边写法 {edge_spec!r} 既不是 {list(REF_EDGE_SPECS)} "
                      f"之一，也不是 y=<毫米> / x=<毫米>（size_mm={size_mm!r}）")
    px = project_pallet_points(T_cam_pallet, pts, K, D)
    if isinstance(px, Reject):
        return px

    length = float(np.linalg.norm(px[1] - px[0]))
    if length < min_edge_px:
        return Reject("edge_too_short",
                      f"参考边 {edge_spec} 投影后只有 {length:.1f} px"
                      f"（阈值 {min_edge_px:.0f} px）—— 托盘太远或太侧")
    # ⚠️ **`edge_off_image` 判据 2026-09-30 删掉了**（原来这里："两个端点都在图外
    # 就拒整帧"）。操作员在真机上问到的正是它："托盘底边在画面外也应该能有输出吧？"
    # —— 该有，而且**下游的算法本来就支持**：
    #   * `theta` 只用两端点的**方向**算，端点在哪不影响；
    #   * `e_bottom` / `e_right` 的垂足按**无限直线**算（`edge_offset` 的 docstring
    #     明写"落在段外时置 `foot_outside`，**但垂距本身仍然是有效信息，所以照样
    #     输出**"）。
    # 实测（托盘往近处挪、下边从画面底部溜出去）：`y=0` 一端在图外时照常给出
    # `e_bottom=+63.7px e_right=+758.5px theta=-0.07°`，数值有效；两个端点都出去
    # 之后才被这条判据拦住 —— 而那正是算法声明"照样输出"的那种情况。
    # "这条边看不到"已经由 `foot_outside` 的 warn 覆盖（它说明箱子偏出了这条边的
    # 范围），不需要再拒一帧。真正该守的是上面那条 `edge_too_short`。
    #
    # ⚠️ `image_size` 参数**保留**（调用方都在传），但现在它不再参与判断 ——
    # 留着是为了不打断调用方签名，也因为它未来可能用于别的诊断。
    return px


def parse_ref_edges(ref_edges, size_mm=None) -> Union[Tuple[str, str], Reject]:
    """校验并规范化 `ref_edges`：两条互相不平行的合法参考边。

    合法性是**启动时**就该查出来的配置错误，不是每帧的几何退化——配错了每帧
    都会失败，早失败一句好过每帧失败一句。位置的语义是固定的：**第 1 个是
    「底边」**（参与垂距与角度差），**第 2 个是「右边」**（只参与垂距）。

    `size_mm` 给了就顺带查**绝对写法的范围**（`0 ≤ 值 ≤ 对应边长`）——
    `y=H+200` 这种笔误在这里就死，不留到第一帧。**不传就跳过范围校验**
    （给"尺寸还没定"的调用方留路）。
    """
    if not isinstance(ref_edges, (list, tuple)) or len(ref_edges) != 2:
        return Reject("ref_edges_arity",
                      f"ref_edges 必须是两条，实际 {ref_edges!r}")
    first, second = parse_edge_spec(ref_edges[0]), parse_edge_spec(ref_edges[1])
    for name, spec, parsed in (("底边（第 1 个）", ref_edges[0], first),
                               ("右边（第 2 个）", ref_edges[1], second)):
        if parsed is None:
            return Reject("ref_edges_unknown",
                          f"{name} {spec!r} 不是合法参考边（可选项 "
                          f"{list(REF_EDGE_SPECS)}，或 y=<毫米> / x=<毫米>）")
    # 平行判据比**轴**，不比首字符：`y=0` 与 `y=800` 都沿 x 展开，是平行线。
    axis_first = first[1] if first[0] == "abs" else first[1][0]
    axis_second = second[1] if second[0] == "abs" else second[1][0]
    if axis_first == axis_second:
        return Reject("ref_edges_parallel",
                      f"{ref_edges[0]} 与 {ref_edges[1]} 平行——垂距在平行边上没有意义")

    if size_mm is not None:
        if isinstance(size_mm, (str, bytes, bytearray)):
            return Reject("size_mm_bad",
                          f"size_mm 是字符串 {size_mm!r} —— 要的是两个数"
                          f"（量不了绝对写法的范围）")
        try:
            W, H = float(size_mm[0]), float(size_mm[1])
        except (TypeError, IndexError, ValueError):
            return Reject("size_mm_bad",
                          f"size_mm 不是两个数：{size_mm!r}（量不了绝对写法的范围）")
        for spec, parsed in ((ref_edges[0], first), (ref_edges[1], second)):
            if parsed[0] != "abs":
                continue
            _, axis, value = parsed
            limit = H if axis == "y" else W
            if value < 0.0 or value > limit:
                # 实际值与边界都按**十进制原样**写出来（不用 :g）：调用方拿板子上
                # 写的那个数来比对，`1201.0` 不该变成 `1201`。
                return Reject("ref_edge_out_of_range",
                              f"参考边 {spec!r} 落在台面外：{axis}={value}mm "
                              f"超出 0~{limit}mm（台面 {W:g}x{H:g}）")
    return str(ref_edges[0]), str(ref_edges[1])


# --------------------------------------------------------------------------- #
# 点击顺序：操作员只记一条，托盘那边由工具内部重排
# --------------------------------------------------------------------------- #
# 操作员**永远只点这一种顺序**（托盘与箱子同一个顺序）：
OPERATOR_CLICK_ORDER = ("右下", "左下", "左上", "右上")

# 托盘路径专用的**固定置换**：操作员点击的第 i 个点 → 托盘建系用的第
# PALLET_CLICK_PERMUTATION[i] 个点。即
#
#     [右下, 左下, 左上, 右上]    --置换 (1, 0, 3, 2)-->    [左下, 右下, 右上, 左上]
#
# 这是**写死的下标重排**（把 0..3 换个位置），不是按图像几何猜的。
#
# 为什么托盘那边非重排不可：`PalletFrame.to_matrix()` 把 `[e1 | e2 | normal]` 当
# **旋转矩阵**用，这就要求 `e1 × e2 = normal`。托盘从上方看时，操作员那条**顺时针**
# 环序给出的 `e1 × e2` 恰好是 `−normal`，于是 `det = −1` —— 那是**镜面反射，不是
# 旋转**；而 `matrix_to_pose6d()` 走 `Rotation.from_matrix`，会把镜面**静默投影掉**，
# 还原回来就是另一个系，参考边跑到托盘外面去，一路上没有任何报错。逆时针环序才是
# 右手系（`order` 里的下一条边换成 `e1`/`e2` 都没用，整个环一起反号）。
#
# 箱子四角**不重排**：那 4 个点只是一份"边的清单"（底边取前两点、右边取后两点），
# 不建坐标系，没有手性要求。
#
# **不要用 `order_quad()`**：它按"图像上最靠右的点"起头——那取决于**相机视角**，
# 相机一动最右的点就可能换角，托盘系跟着翻。伺服要的是按**物理角**定死的系，所以
# 顺序必须由操作员按物理位置给；这里的重排是固定置换，不是几何启发式。
#
# **放在算法层（而不留在点点工具里）的理由**，与下面的 `handedness_problem` 相同，
# 而且更硬：这个置换**存在的唯一理由就是让 det 落在 +1 一侧**，判据和它分开两个
# 文件存等于把结论和原因拆开。更要紧的是它现在要在**三处**用（离线点点工具、
# 标定工具 `pallet_calibrate.py`、以及钉住它的测试），任何一处各自抄一份都是下一个
# bug 的温床 —— 曾经抄过第二份，而那份注释自称是拦网、实际从不检查工具里的值。
PALLET_CLICK_PERMUTATION = (1, 0, 3, 2)


def reorder_pallet_clicks(points) -> List[Tuple[float, float]]:
    """操作员顺序 `[右下, 左下, 左上, 右上]` → 托盘建系用的逆时针环
    `[左下, 右下, 右上, 左上]`。

    按**固定置换** `PALLET_CLICK_PERMUTATION` 换位置，不按图像几何猜（理由见该常量）。

    重排后（`pallet_frame_from_clicks` 的约定是 `origin=points[0]`、
    `e1=points[1]−points[0]`、`e2=points[3]−points[0]`）：

        origin = **左下**角；e1 沿**下边**向右；e2 沿**左边**向上

    于是 `"y=0"` = 下边、`"x=W"` = 右边、`"y=H"` = 上边、`"x=0"` = 左边。
    """
    pts = [(float(u), float(v)) for u, v in points]
    if len(pts) != len(OPERATOR_CLICK_ORDER):
        raise ValueError(f"托盘要 {len(OPERATOR_CLICK_ORDER)} 个点击点，拿到 {len(pts)} 个")
    return [pts[i] for i in PALLET_CLICK_PERMUTATION]


# --------------------------------------------------------------------------- #
# 托盘建系的手性自检（点点工具与算法层共用）
# --------------------------------------------------------------------------- #
# 手性自检的判据边界：det 落在**哪个假设**那一侧。
#
# 判的是**手性**（镜面 or 不是），**不是正交性**（点得准不准）——这两件事必须分开：
# `pallet_frame_from_clicks` 只把 e1/e2 **归一化**、**不做正交化**，所以
#
#     det = e1 · (e2 × normal) = sin θ        （θ = 两条点击边的实际夹角）
#
# 点得准 → θ ≈ 90° → det ≈ +1；环序点反了 → θ ≈ −90° → det ≈ −1。两个假设相距
# **2.0**，所以边界取 0（两个假设的正中间）就已经很宽；这里再往 +1 一侧收到 0.5，
# 于是"误拒正常点击"要 det 从 +1 掉到 0.5 以下，即 θ 偏离 90° 超过 30° —— 正常点击
# 偏不到那里（1 px 在 1.2 m 边上 ≈ 0.11°）；那条 5° 的正交性警告阈值
# 定义在调用方 `pick_servo_inputs.py` 的 `ORTHO_TOLERANCE_DEG` 处，不在本层。
# 用"det 离 +1 多近"当判据是错的：那是拿容差去要求正交性，一个像素的点击误差就能
# 把正常点击拒掉（实测 1 px → |det−1| ≈ 1.8e-6，而点击噪声远大于此）。
HANDEDNESS_MARGIN = 0.5


def handedness_problem(T_cam_pallet,
                       margin: float = HANDEDNESS_MARGIN) -> Optional[str]:
    """`T_cam_pallet` 的旋转块是不是**右手系**（`det` 在 +1 一侧）。通过返回 None。

    `[e1 | e2 | normal]` 只有在 `e1 × e2 = normal` 时才是旋转矩阵；环序点反了会得到
    `det = −1`——镜面反射。镜面在下游是**静默的**（`matrix_to_pose6d()` 里的
    `Rotation.from_matrix` 会把它投影成"最近的旋转"），所以只能在这里、在源头拦。

    **判据是把 det 判给两个假设中更近的那个**（边界 `det < margin`，默认 0.5），
    不是"det 离 +1 多近"——理由见 `HANDEDNESS_MARGIN`。返回的文本里带上**实际
    行列式**；`det ≈ 0` 单独说清（那是 e1/e2 近乎共线，不是镜像，别再把人往
    "顺序点错了"上引）。

    放在算法层（而不是留在点点工具里）是因为**这段判据在同一个文件上已经回归过
    两次**（容差误拒正常点击、`det≈0` 与镜面共用一句话），而它决定整条离线路径
    的输入对不对 —— 纯函数放这儿，`apps/test_kuavo_5w_skills/test_pallet_servo.py`
    的合成用例才能直接钉住它。
    """
    T = np.asarray(T_cam_pallet, np.float64)
    if T.shape != (4, 4):
        return f"T_cam_pallet 必须是 4x4，实际 {T.shape}"
    det = float(np.linalg.det(T[:3, :3]))
    if not np.isfinite(det):
        return f"T_cam_pallet 的旋转块含 NaN/inf，行列式 = {det}"
    if det < margin:
        if abs(det) < margin:
            # 两个假设（+1 / −1）都不是：det≈0 意味着 e1 ⊥ (e2 × normal)，而三者都
            # 在同一个平面里 → e1 ∥ e2（两条点击边几乎共线），给不出一个坐标系。
            return (f"行列式 = {det:+.6f}，几乎为 0 —— e1 与 e2 **近乎共线**"
                    f"（四个点是不是落在同一条线上了？），这个坐标系定不出来"
                    f"（判据：det 必须 ≥ {margin:g}）")
        return (f"行列式 = {det:+.6f} —— 落在**镜面反射**那一侧"
                f"（判据：det < {margin:g}；+1 = 右手系、−1 = 镜面，两个假设相距 2.0）"
                f" —— 镜面不是旋转，参考边投影出来会跑到托盘外面去")
    return None


# --------------------------------------------------------------------------- #
# 箱子：四角与两条边
# --------------------------------------------------------------------------- #
@dataclass
class BoxObservation:
    """手里那个箱子的观测（YOLO 给，或者人工点点给）。

    `u1/v1/u2/v2` 是轴对齐框，**永远有**；`quad` 是将来"框内边缘拟合"模块填
    的四个角点（顺序见 `box_corners`）。取边优先用 `quad`，没有才退到 AABB——
    加了拟合模块之后，伺服算法和输出格式一行都不用改。

    `quad` 为 None 时走 AABB 合成，两条路径给出的是**同一个数据结构**，
    所以后面取边没有任何分支。
    """

    u1: float = 0.0
    v1: float = 0.0
    u2: float = 0.0
    v2: float = 0.0
    quad: Optional[List[List[float]]] = None
    label: str = ""
    confidence: float = 1.0
    stamp: float = 0.0


def box_corners(obs: BoxObservation) -> Union[np.ndarray, Reject]:
    """箱子的四个角，顺序固定 **`[右下, 左下, 左上, 右上]`**。

    有 `quad` 就用 `quad`，否则由 AABB 合成。AABB 按同一顺序合成
    （图像坐标 u 向右、v 向下）：

        p0 = (u_max, v_max) 右下    p1 = (u_min, v_max) 左下
        p2 = (u_min, v_min) 左上    p3 = (u_max, v_min) 右上

    **不做"最平行配对"**：边的语义由顺序直接给出，不猜。

    ⚠️ **但契约顺序没有被任何自动检查兜住。** 凸环判据只要求相邻边叉积**同号**，
    所以「整个环反着走」和「循环移位」都会被放行——而它们的 `p0→p1` 会取到**别的
    边**，静默出错。`theta` 的 ±45° 也兜不住（线角是**无向**的，换成对边角度不变）。
    **责任在调用方**：操作员按契约点、或将来填 `quad` 的拟合模块按契约填。
    本函数只拦"自交/凹"这类几何上根本不是四边形的输入。详见设计文档 §3.2 的更正段。
    """
    if obs.quad is not None:
        quad = np.asarray(obs.quad, np.float64)
        if quad.shape != (4, 2):
            return Reject("quad_shape", f"quad 必须是 4x2，实际 {quad.shape}")
        if not np.all(np.isfinite(quad)):
            return Reject("quad_not_finite", "quad 里有 NaN/inf")
        bad = _convex_ring_violation(quad)
        if bad is not None:
            return Reject("quad_not_convex_ring", bad)
        return quad

    u_lo, u_hi = sorted((float(obs.u1), float(obs.u2)))
    v_lo, v_hi = sorted((float(obs.v1), float(obs.v2)))
    if not (u_hi - u_lo > 0.0 and v_hi - v_lo > 0.0):
        return Reject("box_degenerate",
                      f"箱子框退化：宽 u_hi-u_lo = {u_hi - u_lo:.1f} px、"
                      f"高 v_hi-v_lo = {v_hi - v_lo:.1f} px，要求两者都 > 0")
    return np.array([[u_hi, v_hi], [u_lo, v_hi], [u_lo, v_lo], [u_hi, v_lo]],
                    np.float64)


def _convex_ring_violation(quad: np.ndarray) -> Optional[str]:
    """四角按声明顺序应构成不自交的**凸环**。返回违规说明，合规返回 None。

    相邻边叉积必须同号（全正或全负）。有一个反号就说明这个环自交或者凹了——
    那多半是角点顺序给错了，而顺序错了会让"底边""右边"取到别的边上去，
    算出来的误差看着像真的。
    """
    cross = []
    for i in range(4):
        a = quad[(i + 1) % 4] - quad[i]
        b = quad[(i + 2) % 4] - quad[(i + 1) % 4]
        cross.append(float(a[0] * b[1] - a[1] * b[0]))
    cross_arr = np.asarray(cross, np.float64)
    shown = np.round(cross_arr, 3).tolist()
    if np.any(np.abs(cross_arr) < 1e-9):
        return f"有相邻边共线（相邻边叉积 {shown}）"
    if not (np.all(cross_arr > 0.0) or np.all(cross_arr < 0.0)):
        return f"不是凸环（相邻边叉积 {shown}）—— 角点顺序可能给错了"
    return None


def box_edges(corners: np.ndarray) -> Dict[str, np.ndarray]:
    """由四角取出参与伺服的两条边：底边 `p0→p1`、右边 `p3→p0`。

    返回 `{"bottom": (2,2), "right": (2,2)}`。用 dict 而不是具名元组，是为了
    和设计文档 §4.1 写的签名一致——这份契约在技能层、节点层、出图工具里都
    要读，键名比位置更不容易看错。
    """
    corners = np.asarray(corners, np.float64)
    return {"bottom": corners[0:2].copy(),
            "right": np.array([corners[3], corners[0]], np.float64)}


# --------------------------------------------------------------------------- #
# 角度：直线无向
# --------------------------------------------------------------------------- #
def fold_line_angle(a: float) -> float:
    """把一个**无向直线**的角度（或两个这样的角之差）折进 `(-pi/2, pi/2]`。

    一条边是 p0→p1 还是 p1→p0 是同一条线，不折的话两个视觉上重合的边会算出
    178° 的"角度差"。折完还有一个好处：用户说的"角度差不会超过 ±45°"就成
    了一条真正的合理性检查。
    """
    a = a % math.pi                       # [0, pi)
    if a > math.pi / 2.0:
        a -= math.pi                      # (-pi/2, pi/2]
    return a


def line_angle(p0, p1) -> float:
    """两点连线的方向角，归一到 `(-pi/2, pi/2]`。

    图像坐标 u 向右、v 向下，所以 `atan2(dv, du)` 的正方向在屏幕上是**顺时针**。
    """
    a = math.atan2(float(p1[1]) - float(p0[1]), float(p1[0]) - float(p0[0]))
    return fold_line_angle(a)


# --------------------------------------------------------------------------- #
# 垂距：中点、内法向、垂足
# --------------------------------------------------------------------------- #
def inward_normal(ref_px, center_px) -> np.ndarray:
    """参考边的单位**内法向**：候选是 `±(-d_v, d_u)`，取指向托盘内部的那个。

    "内部"由托盘自己的几何定死——拿**台面中心的投影**去点乘，而不是看图像
    方位。这样托盘在画面里怎么转，取到的都是同一条边的同一侧。
    """
    ref_px = np.asarray(ref_px, np.float64)
    d = ref_px[1] - ref_px[0]
    d = d / float(np.linalg.norm(d))
    cand = np.array([-d[1], d[0]], np.float64)
    m_ref = ref_px.mean(axis=0)
    if float((np.asarray(center_px, np.float64) - m_ref) @ cand) < 0.0:
        cand = -cand
    return cand


@dataclass
class EdgeOffset:
    """一条托盘参考边对一个箱子边的带符号垂距，含画图要用的诊断量。

    ⚠️ **`e_px` 与 `inward_px` 的符号约定不一样，这是有意的**（2026-09-30）：
      * `e_px` —— **箱子在托盘内侧时为负**（操作员的约定）。`servo_error` 里
        算完把它取负，就是在这一句上翻的。
      * `inward_px` —— **恒为"从参考边指向托盘内部"**，不跟着翻。它给
        `render.py` 画蓝箭头用（手工闸门之一是"蓝箭头指向托盘内部"），
        以及判 `foot_outside`。
    """

    e_px: float
    ref_px: np.ndarray          # (2, 2) 参考边端点
    box_px: np.ndarray          # (2, 2) 箱子边端点
    foot_px: np.ndarray         # (2,) 箱子边中点在参考边**所在直线**上的垂足
    inward_px: np.ndarray       # (2,) 单位内法向
    foot_outside: bool          # 垂足落在线段外（仍输出，只是置 warn）


def edge_offset(ref_px, box_px, inward_px) -> EdgeOffset:
    """`e = (M_box − foot) · n`，`M_box` 是箱子这条边的**中点**。

    取中点而不是"线到线"：两条线不平行时"线到线"是一个区间，取端点没有意义，
    而伺服推的就是这个点——取中点才和伺服的语义一致。

    垂足按**无限直线**算（不截断到线段），落在段外时置 `foot_outside`：那时
    箱子已经偏出这条边的范围，但垂距本身仍然是有效信息，所以照样输出。
    """
    ref_px = np.asarray(ref_px, np.float64)
    box_px = np.asarray(box_px, np.float64)
    n = np.asarray(inward_px, np.float64)
    p0, p1 = ref_px[0], ref_px[1]
    d = p1 - p0
    seg_len = float(np.linalg.norm(d))
    u = d / seg_len
    m_box = box_px.mean(axis=0)
    t = float((m_box - p0) @ u)
    foot = p0 + t * u
    return EdgeOffset(e_px=float((m_box - foot) @ n),
                      ref_px=ref_px, box_px=box_px, foot_px=foot,
                      inward_px=n, foot_outside=bool(t < 0.0 or t > seg_len))


# --------------------------------------------------------------------------- #
# 三个量
# --------------------------------------------------------------------------- #
@dataclass
class ServoError:
    """一次伺服误差的完整结果。

    前三个是被控量，其余是**诊断量**：没有它们就出不了对比图（`render_overlay`
    直接吃它们），真机上也排查不了。最后四个是**出处**——算法层不填，由节点
    盖上去，用来把一帧和黑板对上。
    """

    # —— 三个被控量 ——
    e_bottom_px: float
    e_right_px: float
    theta_rad: float
    # —— 诊断量 ——
    ref_bottom_px: np.ndarray
    ref_right_px: np.ndarray
    box_bottom_px: np.ndarray
    box_right_px: np.ndarray
    pallet_center_px: np.ndarray
    bottom_foot_px: np.ndarray
    right_foot_px: np.ndarray
    bottom_inward_px: np.ndarray
    right_inward_px: np.ndarray
    box_source: str = "aabb"
    warn: List[str] = field(default_factory=list)
    # —— 出处：算法层不填，由节点盖上去 ——
    pallet_version: int = -1
    box_version: int = -1
    t_cam_base_src: str = ""
    stamp: float = 0.0

    def to_log_line(self) -> str:
        """一行给人读的摘要，供节点节流打印（设计文档 §6.2「正常帧」）。"""
        line = (f"e_bottom={self.e_bottom_px:+.1f}px "
                f"e_right={self.e_right_px:+.1f}px "
                f"theta={math.degrees(self.theta_rad):+.2f}deg "
                f"box={self.box_source} pallet_v={self.pallet_version} "
                f"box_v={self.box_version} t_cam_base={self.t_cam_base_src}")
        if self.warn:
            line += " warn=" + "; ".join(self.warn)
        return line


def servo_error(T_cam_pallet, size_mm, ref_edges, obs: BoxObservation, K,
                D=None, image_size=None, min_edge_px: float = MIN_EDGE_PX,
                theta_warn_rad: float = THETA_WARN_RAD
                ) -> Union[ServoError, Reject]:
    """一帧的全部伺服误差。任何一项退化都返回 `Reject`（带实际值与阈值）。

    顺序是刻意的：先判配置（`ref_edges`），再判箱子，最后判托盘参考边——
    配置错误每帧都会犯，先说它，免得每帧都刷一条几何退化的假象。
    """
    parsed = parse_ref_edges(ref_edges)
    if isinstance(parsed, Reject):
        return parsed
    edge_bottom, edge_right = parsed

    corners = box_corners(obs)
    if isinstance(corners, Reject):
        return corners
    edges = box_edges(corners)

    ref_bottom = ref_edge_px(T_cam_pallet, size_mm, edge_bottom, K, D,
                             image_size, min_edge_px)
    if isinstance(ref_bottom, Reject):
        return ref_bottom
    ref_right = ref_edge_px(T_cam_pallet, size_mm, edge_right, K, D,
                            image_size, min_edge_px)
    if isinstance(ref_right, Reject):
        return ref_right

    W, H = float(size_mm[0]), float(size_mm[1])
    center = project_pallet_points(
        T_cam_pallet, [(W / 2.0, H / 2.0, 0.0)], K, D)
    if isinstance(center, Reject):
        return center
    center = center[0]

    n_bottom = inward_normal(ref_bottom, center)
    n_right = inward_normal(ref_right, center)
    off_bottom = edge_offset(ref_bottom, edges["bottom"], n_bottom)
    off_right = edge_offset(ref_right, edges["right"], n_right)
    # ⚠️ **符号约定 2026-09-30 翻过一次**（操作员裁决："箱子在里面输出负数"）。
    # `edge_offset` 里 `e = (M_box − foot) · n`、`n` 是**内法向**，所以箱子落在
    # 托盘内侧时算出来是**正**的 —— 现在把**误差取负**，内侧变负。
    #
    # ⚠️ **只翻 `e_px`，`inward_px` 保持"真·内法向"不翻。** 它有两个别的用处，
    # 都与误差符号无关：
    #   * `render.py` 拿它画那个**蓝箭头**（README 的手工闸门之一是"蓝箭头指向
    #     托盘内部"）—— 翻了箭头就指向外面了；
    #   * `foot_px` / `foot_outside` 判"箱子有没有偏出这条边的线段范围"。
    # 所以这里用 `dataclasses.replace` 只换那一个字段，语义一眼可见。
    off_bottom = replace(off_bottom, e_px=-off_bottom.e_px)
    off_right = replace(off_right, e_px=-off_right.e_px)

    # ⚠️ **极性 2026-09-29 翻过一次**（操作员现场实测"角度是反的"）。
    #    定义改成 `theta = 托盘底边角 − 箱子底边角`，于是**屏幕上顺时针转 → theta 为正**。
    #    改这里而不是在控制器里取负：这是**符号约定**，视觉层与 README 一起改才一致
    #    （README §6 第 11 条原本写"要反转就在控制器取负"，那条对本次改动作废）。
    #    ⚠️ **没有任何测试钉住这个符号** —— `test_render_servo_overlay` 只扫 theta=0，
    #    这里改反了测试照样全绿，所以验收只能靠叠加图上眼看。
    theta = fold_line_angle(line_angle(ref_bottom[0], ref_bottom[1])
                            - line_angle(edges["bottom"][0], edges["bottom"][1]))

    warn: List[str] = []
    if abs(theta) > theta_warn_rad:
        warn.append(f"theta {math.degrees(theta):+.1f}° 超出 "
                    f"±{math.degrees(theta_warn_rad):.0f}°")
    if off_bottom.foot_outside:
        warn.append(f"底边垂足落在参考边 {edge_bottom} 的线段外"
                    f"（箱子已偏出这条边的范围）")
    if off_right.foot_outside:
        warn.append(f"右边垂足落在参考边 {edge_right} 的线段外"
                    f"（箱子已偏出这条边的范围）")

    return ServoError(
        e_bottom_px=off_bottom.e_px,
        e_right_px=off_right.e_px,
        theta_rad=theta,
        ref_bottom_px=ref_bottom,
        ref_right_px=ref_right,
        box_bottom_px=edges["bottom"],
        box_right_px=edges["right"],
        pallet_center_px=center,
        bottom_foot_px=off_bottom.foot_px,
        right_foot_px=off_right.foot_px,
        bottom_inward_px=n_bottom,
        right_inward_px=n_right,
        box_source="quad" if obs.quad is not None else "aabb",
        warn=warn)


# --------------------------------------------------------------------------- #
# 参数解析：两个节点共用一份实现
# --------------------------------------------------------------------------- #
def parse_bool_param(raw, name: str) -> bool:
    """布尔参数：兼容 JSON 布尔值，以及行为树编辑器/黑板传下来的布尔字符串。

    **两个节点共用这一份实现**（`node_inject_servo_input` 的 `enabled`、
    `node_pallet_servo` 的 `use_distortion`）：口径分叉过一次就够呛了，而这两处
    各写一遍正是分叉的入口。

    **不能用 `bool(...)` 顶替**：`bool("false") is True`，而场景 JSON 里
    `{"source": "READ_BOARD", ...}` 走的正是字符串这条路（工厂的 `READ_BOARD`
    分支不像 `RESOLVED` 那样转类型）。于是 `use_distortion: "false"` 会**照开不误**
    （实测两边差 135.918 / 137.000 px，看着都像模像样），`enabled: "false"` 则让
    "真机上必须能关掉"这条硬约束失效。

    口径与 `leju_claw_control.LejuClawControl._as_bool` 一致（判断分支逐条相同，
    只有报错文案更具体）。

    **比 `base_move_to_blackboard_goal_jibot_move._as_bool` 故意更严**，不是笔误：
    那个还认 `"on"`、非字符串一律 `bool(value)` 兜底、而且**永不抛**；这里三条都
    不做，既不认识又解释不了的值**抛 `ValueError`**，由调用方决定怎么处置
    （注入类节点当成"关掉 + 报错"，伺服节点用默认值 + WARNING —— 处置不同，
    但"'false' 不许变成真"这条底线是同一套）。
    """
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, str):
        value = raw.strip().lower()
        if value in ("true", "1", "yes"):
            return True
        if value in ("false", "0", "no"):
            return False
    if isinstance(raw, (int, float)) and raw in (0, 1):
        return bool(raw)
    raise ValueError(f"{name} 必须是布尔值（true/false、1/0、yes/no 都认），"
                     f"实际是 {raw!r}")


def parse_pair_param(value, name: str = "参数"):
    """`[w, h]` 这类**两个正数**的参数 → `(pair, 说明)`，两者必有一个为空。

    `None` / 空序列 → `(None, None)`："没给"不是错误，要不要默认值由调用方定。
    其余情况要么给 `((w, h), None)`，要么给 `(None, "说明")`。

    为什么要专门查一遍（而不是 `float(value[0]), float(value[1])`）：**给字符串
    不会报错，而是静默算错** —— `"12"` 索引出 `'1'`、`'2'` 两个字符，得到
    `(1.0, 2.0)`，还正好通过"都为正"的检查；长度不对的列表同理（少一个数才是
    显式的 IndexError，多一个数被无声忽略）。这类"看着像配了、其实配错了"在
    运行期表现为"每帧被拒"或"整条边跑到图外"，与配置错误毫无相似之处 ——
    所以必须在启动时、在原地说清。

    与 `parse_bool_param` 抛 `ValueError` 的分工：这里返回说明文字，因为两个
    调用方（节点启动校验、技能层启动校验）都要把它拼进**自己的**反馈文案里。
    """
    if value is None:
        return None, None
    if isinstance(value, (str, bytes)):
        return None, (f"{name} 必须是两个数（如 [640, 480]），"
                      f"实际是字符串 {value!r}")
    try:
        items = list(value)
    except TypeError:
        return None, (f"{name} 必须是两个数（如 [640, 480]），"
                      f"实际是 {type(value).__name__} {value!r}")
    if not items:
        return None, None
    if len(items) != 2:
        return None, f"{name} 必须是两个数，实际 {len(items)} 个：{value!r}"
    try:
        pair = (float(items[0]), float(items[1]))
    except (TypeError, ValueError):
        return None, f"{name} 里不是数：{value!r}"
    if not (np.isfinite(pair[0]) and np.isfinite(pair[1])):
        return None, f"{name} 里有 NaN/inf：{value!r}"
    if not (pair[0] > 0.0 and pair[1] > 0.0):
        return None, f"{name} 必须都为正，实际 {pair}"
    return pair, None
