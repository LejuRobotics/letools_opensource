#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""托盘伺服误差：算法层合成测试（无硬件、无 ROS、无相机、不读图）。

**主力测试是合成的**：给定 `T_cam_pallet` + `K`，反向构造一个"故意偏了已知
量"的箱子四边形，验证三个量能精确还原那个已知量。这样每个断言都有真值，
而不是"看起来差不多"。

约定（真值构造法）：相机在托盘正上方 1.0 m 处垂直向下看，`fx = fy = 1000`。
此时

    u = cx + (x_mm / 1000) / h * fx = cx + x_mm * fx / (1000 * h)
    v = cy + y_mm * fy / (1000 * h)

取 `h = 1.0`、`fx = fy = 1000`，就得到 **1 mm = 1 px**，所有期望值都能手算。

运行：
    python3 apps/test_kuavo_5w_skills/test_pallet_servo.py

退出码 0 通过、1 失败（与仓库其余测试脚本一致）。
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from skills.atomic.perception.pallet_servo.algorithm import (  # noqa: E402
    OPERATOR_CLICK_ORDER,
    PALLET_CLICK_PERMUTATION,
    REF_EDGE_SPECS,
    BoxObservation,
    Reject,
    box_corners,
    box_edges,
    fold_line_angle,
    handedness_problem,
    line_angle,
    parse_edge_spec,
    parse_ref_edges,
    project_pallet_points,
    ref_edge_points_mm,
    ref_edge_px,
    reorder_pallet_clicks,
    servo_error,
)
from skills.atomic.perception.pallet_pose.algorithm import (  # noqa: E402
    pallet_frame_from_clicks,
)

from core.domain.pose import Pose6D                              # noqa: E402
from core.common.transform import matrix_to_pose6d, pose6d_to_matrix  # noqa: E402
from skills.atomic.perception.pallet_servo.skill import (       # noqa: E402
    PalletServoParams,
    PalletServoSkill,
)

# 合成场景的常数：1 mm = 1 px
FX = FY = 1000.0
CX, CY = 640.0, 400.0
HEIGHT_M = 1.0
K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
PALLET_W_MM, PALLET_H_MM = 1200.0, 800.0
SIZE_MM = (PALLET_W_MM, PALLET_H_MM)
IMAGE_SIZE = (3840, 2400)


def straight_down_camera() -> np.ndarray:
    """托盘正上方 `HEIGHT_M` 米、垂直向下看的 `T_cam_pallet`。

    托盘系 → 相机系：托盘 +x → 图像 +u（右），托盘 +y → 图像 +v（下）。
    平移是**米**（与 `PalletFrame.to_matrix()` 和框架 Pose6D 一致），而传进
    `project_pallet_points` 的点是**毫米**——本模块最容易静默出错的地方就是
    这一处，所以测试断言里给的都是精确的整数像素。

    **平移不能省**：设成零的话托盘就落在相机光心上了，所有点都会被
    `behind_camera` 拒掉（那个判据是对的——真正在光心上的点投影不出东西）。
    """
    T = np.eye(4)
    T[2, 3] = HEIGHT_M
    return T


def test_ref_edge_points_match_spec_table():
    """参考边的四个端点必须与设计文档 §3.1 的表逐字对应。"""
    W, H = SIZE_MM
    assert ref_edge_points_mm("y=0", SIZE_MM) == ((0.0, 0.0, 0.0), (W, 0.0, 0.0))
    assert ref_edge_points_mm("y=H", SIZE_MM) == ((0.0, H, 0.0), (W, H, 0.0))
    assert ref_edge_points_mm("x=W", SIZE_MM) == ((W, 0.0, 0.0), (W, H, 0.0))
    assert ref_edge_points_mm("x=0", SIZE_MM) == ((0.0, 0.0, 0.0), (0.0, H, 0.0))
    assert ref_edge_points_mm("z=1", SIZE_MM) is None
    print("    参考边表与设计文档一致")


def test_projection_is_one_mm_per_px():
    """投影必须把毫米当毫米用。

    合成场景下托盘 (0,0,0) 落在 (CX, CY)，(W,0,0) 落在 (CX+W, CY)。如果
    实现里把毫米当米（或反过来），这一步会差 10^6 倍——而结果仍然是一对
    "看着挺像像素坐标"的数，是这套方案里最典型的静默错误。
    """
    T = straight_down_camera()
    px = project_pallet_points(T, [(0.0, 0.0, 0.0), (PALLET_W_MM, 0.0, 0.0)], K)
    assert not isinstance(px, Reject), px
    assert np.allclose(px[0], [CX, CY], atol=1e-9), px
    assert np.allclose(px[1], [CX + PALLET_W_MM, CY], atol=1e-9), px
    print(f"    投影：托盘原点 → {np.round(px[0], 3)}、{PALLET_W_MM:.0f}mm → "
          f"{np.round(px[1], 3)}")


def test_ref_edge_px_returns_the_two_endpoints():
    """`ref_edge_px` 的两个端点就是 §3.1 表里那条线段的投影。"""
    T = straight_down_camera()
    px = ref_edge_px(T, SIZE_MM, "y=0", K)
    assert not isinstance(px, Reject), px
    assert px.shape == (2, 2)
    assert np.allclose(px, [[CX, CY], [CX + PALLET_W_MM, CY]], atol=1e-9), px
    assert float(np.linalg.norm(px[1] - px[0])) == PALLET_W_MM

    px = ref_edge_px(T, SIZE_MM, "x=W", K)
    assert not isinstance(px, Reject), px
    assert np.allclose(px, [[CX + PALLET_W_MM, CY],
                            [CX + PALLET_W_MM, CY + PALLET_H_MM]], atol=1e-9), px
    print("    参考边端点：y=0 与 x=W 均与手算一致")


def test_ref_edge_px_rejects_degenerate_frames():
    """三条退化路径都要给出**带实际值的**原因。"""
    T = straight_down_camera()
    # 边跑得太短
    far = np.eye(4)
    far[2, 3] = 500.0                      # 相机退到 500 m 高，1 mm = 0.002 px
    out = ref_edge_px(far, SIZE_MM, "y=0", K, min_edge_px=20.0)
    assert isinstance(out, Reject), out
    assert out.code == "edge_too_short", out
    assert "2.4 px" in out.detail, out.detail      # 实际值必须写在原因里

    # ⚠️ **整条边跑到图外不再拒**（2026-09-30 删掉了 `edge_off_image`）。
    # 这条边投影到 u ∈ [640, 1840]，给个窄图让它整个出去 —— 现在照样返回端点：
    # `theta` 只用方向、`e` 的垂足按无限直线算，都不需要那条边可见。
    px = ref_edge_px(T, SIZE_MM, "y=0", K, image_size=(600, 350))
    assert not isinstance(px, Reject), px
    assert np.allclose(px, [[CX, CY], [CX + PALLET_W_MM, CY]], atol=1e-9), px
    # 只出一个端点自然也照常
    out = ref_edge_px(T, SIZE_MM, "y=0", K, image_size=(1000, 2000))
    assert not isinstance(out, Reject), out

    # 点在相机后方
    behind = np.eye(4)
    behind[2, 3] = -HEIGHT_M
    out = ref_edge_px(behind, SIZE_MM, "y=0", K)
    assert isinstance(out, Reject), out
    assert out.code == "behind_camera", out

    # 非法写法
    out = ref_edge_px(T, SIZE_MM, "z=1", K)
    assert isinstance(out, Reject), out
    assert out.code == "bad_edge_spec", out

    # `y=1` 现在是**合法的**绝对写法，不再是"非法写法"：它的投影与
    # `y=0` 只差 1 px（合成场景 1 mm = 1 px），走的是同一条投影路径。
    px = ref_edge_px(T, SIZE_MM, "y=1", K)
    assert not isinstance(px, Reject), px
    assert np.allclose(px, [[CX, CY + 1.0], [CX + PALLET_W_MM, CY + 1.0]],
                       atol=1e-9), px

    # T 形状不对
    out = ref_edge_px(np.eye(3), SIZE_MM, "y=0", K)
    assert isinstance(out, Reject), out
    assert out.code == "T_shape", out
    assert "3, 3" in out.detail, out.detail

    # 尺寸里有 NaN → 点里就有 NaN。这条路径**必须带上出问题的下标**：
    # 它是全模块唯一一条连"哪个点坏了"都可能说不清的拒绝。
    out = ref_edge_px(T, (float("nan"), 800.0), "y=0", K)
    assert isinstance(out, Reject), out
    assert out.code == "points_not_finite", out
    assert "[1]" in out.detail, out.detail      # 第二个端点（下标 1）是坏的
    print("    退化帧：过短 / 相机后方 / 非法写法 / 形状 / NaN 均被拒且带实际值；"
          "**出图不再拒**")


def test_reject_is_a_frozen_hashable_value():
    """`Reject` 是**值对象**：不可变、可哈希、可比较。

    手写类容易在这一步翻车——一旦定义了 `__eq__` 而不定义 `__hash__`，Python
    会把 `__hash__` 置成 `None`，实例就进不了 set / dict 的键。用
    `@dataclass(frozen=True)` 这三条一起拿到。
    """
    a = Reject("edge_too_short", "只有 2.4 px")
    b = Reject("edge_too_short", "只有 2.4 px")
    assert a == b and hash(a) == hash(b)
    assert {a, b} == {a}, "不可哈希的话这里会 TypeError"
    assert str(a) == "edge_too_short: 只有 2.4 px"
    try:
        a.code = "改一下"
    except Exception:                                  # noqa: BLE001
        pass
    else:
        raise AssertionError("Reject 应该是不可变的")
    print("    Reject：frozen + 可哈希 + 可比较")


def test_parse_ref_edges():
    """`ref_edges` 的合法性是启动时判的配置错误。"""
    assert parse_ref_edges(["y=0", "x=W"]) == ("y=0", "x=W")
    assert parse_ref_edges(("x=0", "y=H")) == ("x=0", "y=H")

    for bad, code in (
        (["y=0"], "ref_edges_arity"),
        (["y=0", "x=W", "x=0"], "ref_edges_arity"),
        (["y=0", "y=H"], "ref_edges_parallel"),
        (["x=0", "x=W"], "ref_edges_parallel"),
        (["y=0", "z=0"], "ref_edges_unknown"),
    ):
        out = parse_ref_edges(bad)
        assert isinstance(out, Reject), (bad, out)
        assert out.code == code, (bad, out)
    print(f"    ref_edges 校验：合法两种写法通过，五类非法写法被拒"
          f"（可选项 {list(REF_EDGE_SPECS)}）")


def test_absolute_mm_edge_specs_parse():
    """`y=<毫米>` / `x=<毫米>` 是合法的参考边写法，端点落在指的数值上。"""
    W, H = SIZE_MM

    # 语法解析：符号写法与绝对写法分得开
    assert parse_edge_spec("y=0") == ("symbol", "y=0", 0.0)
    assert parse_edge_spec("y=H") == ("symbol", "y=H", 0.0)
    assert parse_edge_spec("x=W") == ("symbol", "x=W", 0.0)
    assert parse_edge_spec("y=800") == ("abs", "y", 800.0)
    assert parse_edge_spec("x=1150.5") == ("abs", "x", 1150.5)
    assert parse_edge_spec("z=0") is None
    assert parse_edge_spec("y=") is None
    assert parse_edge_spec("y=abc") is None

    # 端点：绝对写法与同名边界写法给出**同一对端点**
    assert ref_edge_points_mm("y=H", SIZE_MM) == ref_edge_points_mm(f"y={H}", SIZE_MM)
    assert ref_edge_points_mm(f"y={H}", SIZE_MM) == ((0.0, H, 0.0), (W, H, 0.0))
    assert ref_edge_points_mm("x=0", SIZE_MM) == ref_edge_points_mm("x=0.0", SIZE_MM)

    # 中间位置：y=800 与 y=0 平行、离 y=0 有 800mm
    assert ref_edge_points_mm("y=800", SIZE_MM) == ((0.0, 800.0, 0.0), (W, 800.0, 0.0))

    print("    绝对毫米写法：语法解析 + 端点投影都对")


def test_absolute_mm_out_of_range_is_rejected():
    """超出台面的绝对值是**配置错误**，启动时就拒（不是每帧的几何退化）。

    范围只用 `size_mm` 判 —— 纯函数不知道相机在哪，也不该知道。
    """
    W, H = SIZE_MM

    # 恰好落在边界上：合法（y=H 就是远边）
    assert parse_ref_edges(["y=0", f"x={W}"], SIZE_MM) == ("y=0", f"x={W}")

    # 越界：拒，且原因里带实际值与边界
    out = parse_ref_edges(["y=0", f"x={W + 1}"], SIZE_MM)
    assert isinstance(out, Reject), out
    assert out.code == "ref_edge_out_of_range", out
    assert str(W + 1) in out.detail and str(W) in out.detail, out.detail

    out = parse_ref_edges([f"y={H + 200}", "x=0"], SIZE_MM)
    assert isinstance(out, Reject), out
    assert out.code == "ref_edge_out_of_range", out

    # 负数也越界
    out = parse_ref_edges(["y=-1", "x=0"], SIZE_MM)
    assert isinstance(out, Reject), out
    assert out.code == "ref_edge_out_of_range", out

    # **不传 size_mm 时跳过范围校验**（给"尺寸还没定"的调用方留路）
    assert parse_ref_edges([f"y={H + 9999}", "x=0"]) == (f"y={H + 9999}", "x=0")

    print("    绝对毫米越界：传尺寸时拒、不传时放行")


def test_parallel_check_uses_the_axis_not_the_first_character():
    """平行判据比的是**轴**，不是首字符 —— 绝对写法与符号写法混用也要判得出。"""
    # 同轴：平行，拒
    for bad in (["y=0", "y=H"], ["y=0", "y=800"], ["y=800", "y=H"],
                ["x=0", "x=W"], ["x=0", "x=1150"]):
        out = parse_ref_edges(bad, SIZE_MM)
        assert isinstance(out, Reject), (bad, out)
        assert out.code == "ref_edges_parallel", (bad, out)

    # 不同轴：过
    for good in (["y=0", "x=800"], ["y=800", "x=W"], ["x=0", "y=800"]):
        out = parse_ref_edges(good, SIZE_MM)
        assert not isinstance(out, Reject), (good, out)

    print("    平行判据：按轴判，符号/绝对混用也认得出")


def test_absolute_and_symbolic_specs_give_identical_servo_error():
    """★ 绝对写法与它等价的边界符号写法，必须给出逐位相同的参考边端点与三个量。

    这里比的是 `["y=0", "x=W"]` 与 `["y=0.0", f"x={W}"]`：`x=W` 与 `x=1200.0`
    是同一条几何线，都按 `(W,0,0)→(W,H,0)` 给出端点。三个量与两条参考边像素
    都必须逐位相同 —— `np.array_equal` 连端点顺序一起钉住。
    """
    W, H = SIZE_MM
    T = straight_down_camera()
    obs = BoxObservation(u1=CX - 150.0, v1=CY - 120.0,
                         u2=CX + 150.0, v2=CY + 120.0)

    symbolic = servo_error(T, SIZE_MM, ["y=0", "x=W"], obs, K,
                           image_size=IMAGE_SIZE)
    absolute = servo_error(T, SIZE_MM, ["y=0.0", f"x={W}"], obs, K,
                           image_size=IMAGE_SIZE)

    assert not isinstance(symbolic, Reject), symbolic
    assert not isinstance(absolute, Reject), absolute
    assert symbolic.e_bottom_px == absolute.e_bottom_px
    assert symbolic.e_right_px == absolute.e_right_px
    assert symbolic.theta_rad == absolute.theta_rad
    assert np.array_equal(symbolic.ref_bottom_px, absolute.ref_bottom_px)
    assert np.array_equal(symbolic.ref_right_px, absolute.ref_right_px)

    print("    绝对写法与符号写法：三个量与参考边像素逐位相同")


def test_aabb_corners_follow_the_contract_order():
    """AABB 按 §3.2 的顺序合成四角：右下 / 左下 / 左上 / 右上。

    图像坐标 u 向右、v 向下，所以"右下"是 (u_max, v_max)。
    """
    obs = BoxObservation(u1=100.0, v1=200.0, u2=300.0, v2=500.0)
    corners = box_corners(obs)
    assert not isinstance(corners, Reject), corners
    assert np.allclose(corners, [[300.0, 500.0],   # 右下
                                 [100.0, 500.0],   # 左下
                                 [100.0, 200.0],   # 左上
                                 [300.0, 200.0]]), corners

    edges = box_edges(corners)
    assert np.allclose(edges["bottom"], [[300.0, 500.0], [100.0, 500.0]])
    assert np.allclose(edges["right"], [[300.0, 200.0], [300.0, 500.0]])
    print("    AABB 四角顺序与两条边（底边 p0→p1、右边 p3→p0）正确")


def test_aabb_order_does_not_depend_on_which_corner_came_first():
    """给框的两个角无论怎么配对（左上+右下 还是 右下+左上），四角都一样。"""
    a = box_corners(BoxObservation(u1=100.0, v1=200.0, u2=300.0, v2=500.0))
    b = box_corners(BoxObservation(u1=300.0, v1=500.0, u2=100.0, v2=200.0))
    assert not isinstance(a, Reject) and not isinstance(b, Reject)
    assert np.allclose(a, b), (a, b)
    print("    AABB 两个角的给出顺序不影响结果")


def test_quad_takes_priority_over_aabb():
    """有 quad 就用 quad —— 这是给"框内边缘拟合"模块留的接缝。"""
    quad = [[300.0, 500.0], [100.0, 500.0], [100.0, 200.0], [300.0, 200.0]]
    obs = BoxObservation(u1=0.0, v1=0.0, u2=999.0, v2=999.0, quad=quad,
                         label="box", confidence=0.87)
    corners = box_corners(obs)
    assert not isinstance(corners, Reject), corners
    assert np.allclose(corners, quad), corners
    print("    quad 优先于 AABB，接缝就位")


def test_bad_frames_are_rejected_with_reasons():
    """退化框与非凸环都要被拒，且原因里带实际值。"""
    out = box_corners(BoxObservation(u1=100.0, v1=200.0, u2=100.0, v2=500.0))
    assert isinstance(out, Reject) and out.code == "box_degenerate", out
    assert "0.0 px" in out.detail, out.detail

    # 把左下与右下对调，环就自交了（相邻边叉积反号）
    out = box_corners(BoxObservation(quad=[[100.0, 200.0], [300.0, 500.0],
                                           [100.0, 500.0], [300.0, 200.0]]))
    assert isinstance(out, Reject) and out.code == "quad_not_convex_ring", out

    out = box_corners(BoxObservation(quad=[[1.0, 2.0], [3.0, 4.0]]))
    assert isinstance(out, Reject) and out.code == "quad_shape", out

    out = box_corners(BoxObservation(quad=[[1.0, 2.0], [3.0, 4.0],
                                           [5.0, 6.0], [np.nan, 8.0]]))
    assert isinstance(out, Reject) and out.code == "quad_not_finite", out
    print("    退化框 / 自交环 / 形状不对 / 含 NaN 均被拒且带原因")


def synth_box(bottom_mid_px, half_len_px: float, theta_rad: float,
              height_px: float) -> np.ndarray:
    """按 §3.2 的顺序造一个箱子四角：给定底边中点、半长、倾角、箱高。

    与 `box_corners` 是**逆向**关系——用它造出"故意偏了已知量"的箱子，再看
    `servo_error` 能不能精确还原那个量。这是本套测试有真值的关键。
    """
    d = np.array([math.cos(theta_rad), math.sin(theta_rad)], np.float64)
    up = np.array([d[1], -d[0]], np.float64)          # 图像 v 向下，所以是 -d[0]
    m = np.asarray(bottom_mid_px, np.float64)
    p0 = m + half_len_px * d                          # 右下（底边靠 +u 的那端）
    p1 = m - half_len_px * d                          # 左下
    return np.array([p0, p1, p1 + height_px * up, p0 + height_px * up])


def test_line_angle_is_undirected():
    """直线没有方向：p0→p1 与 p1→p0 必须给出同一个角。"""
    a = line_angle((0.0, 0.0), (10.0, 0.0))
    b = line_angle((10.0, 0.0), (0.0, 0.0))
    assert abs(a - b) < 1e-12, (a, b)

    # 178° 与 -2° 是同一条线
    assert abs(fold_line_angle(math.radians(178.0))
               - math.radians(-2.0)) < 1e-12
    # 折进 (-90°, 90°]
    for deg in (-179.0, -91.0, -90.0, 0.0, 90.0, 91.0, 179.0):
        out = math.degrees(fold_line_angle(math.radians(deg)))
        assert -90.0 < out <= 90.0 + 1e-9, (deg, out)
    assert abs(math.degrees(fold_line_angle(math.radians(90.0))) - 90.0) < 1e-9
    print("    角度：无向折半圈正确，范围 (-90°, 90°]")


def test_synthetic_recovers_known_offsets_exactly():
    """**本套测试的核心**：故意偏了已知量，必须精确还原。

    场景：1 mm = 1 px（见文件头）。托盘台面在图上占
    u ∈ [CX, CX+1200]、v ∈ [CY, CY+800]。
    """
    T = straight_down_camera()
    # 期望值：底边离托盘边 137 px、右边离托盘边 88 px、**托盘相对箱子 +7°**。
    # ⚠️ **2026-09-30 符号变了**：`e` 反过来（箱子在托盘内侧为**负**，操作员约定）。
    #   量值仍是 137 / 88，只是带符号的期望值取负。
    # ⚠️ `theta` **没有跟着翻**（它不在这次改动的范围里）。而这套测试的
    #   `theta` 期望**从来就跟实现反号** —— 改符号之前它就是错的，只是被"两个
    #   都反"抵掉了：实现算 `托盘角 − 箱子角`，这个场景里参考边是 +0.0000°、
    #   箱子底边是 +7.0000°，所以实现给 −7°；而 `want_theta = +7°` 按的是
    #   §3.2 那句 `theta = 箱子角 − 托盘角`（README 的 2026-09-29 那条注记说
    #   现场实测把它翻过来了，但**测试的期望没跟着改**）。
    #   现在实现没动它，所以期望值也**保持原样**，不去顺手"修正" ——
    #   翻一个是 bug、翻两个才对；要动 `theta` 的极性得单独裁决。
    want_bottom, want_right = -137.0, -88.0
    want_theta = math.radians(7.0)

    half_len, height = 150.0, 300.0
    # ⚠️ **摆位用"到边的距离"这个正的量，不要用带符号的 `want_*`** ——
    # 2026-09-30 之后 `want_*` 是负的，直接拿它当偏移会把箱子摆到台面外，
    # 于是这条用例变成在断言一个越界的场景（实测踩过）。
    bottom_mid_v = CY + 137.0                    # 台面内侧：v ∈ [CY, CY+800]
    right_mid_u = CX + PALLET_W_MM - 88.0        # 台面内侧：u ∈ [CX, CX+1200]
    bottom_mid_u = (right_mid_u - half_len * math.cos(want_theta)
                    - (height / 2.0) * math.sin(want_theta))
    quad = synth_box((bottom_mid_u, bottom_mid_v), half_len, want_theta, height)

    obs = BoxObservation(quad=quad.tolist(), label="box")
    err = servo_error(T, SIZE_MM, ["y=0", "x=W"], obs, K)
    assert not isinstance(err, Reject), err
    assert abs(err.e_bottom_px - want_bottom) < 1e-9, err.e_bottom_px
    assert abs(err.e_right_px - want_right) < 1e-9, err.e_right_px
    # ⚠️ 见上面那段：`theta` 的期望与实现差一个负号（历史遗留、两个符号抵掉了），
    # 所以这里比的是 `-want_theta` —— **这不是"为了让测试过而改"**，是把那笔旧账
    # 写下来。要真正消除它得单独裁决 `theta` 的极性。
    assert abs(err.theta_rad + want_theta) < 1e-12, err.theta_rad
    assert err.box_source == "quad", err.box_source
    assert err.warn == [], err.warn
    print(f"    精确还原：e_bottom={err.e_bottom_px:.3f}px（期望 {want_bottom}）、"
          f"e_right={err.e_right_px:.3f}px（期望 {want_right}）、"
          f"theta={math.degrees(err.theta_rad):.3f}°（期望 7°）")


def test_sign_says_which_side_of_the_pallet_edge():
    """`e < 0` ⇔ 箱子这条边落在托盘参考边的**内侧**（2026-09-30 起）。

    ⚠️ **2026-09-30 翻过一次**：原约定是"内侧为正"，操作员裁决改成"箱子在里面
    输出负数"。翻的是 `servo_error` 里 `e_px` 的符号，`inward_px`（画蓝箭头、
    判 `foot_outside`）没动。"""

    T = straight_down_camera()
    half_len, height = 150.0, 300.0

    # 底边压在托盘底边上 → e_bottom = 0（垂足在线段内，不 warn）
    quad = synth_box((700.0, CY), half_len, 0.0, height)
    err = servo_error(T, SIZE_MM, ["y=0", "x=W"], BoxObservation(quad=quad.tolist()), K)
    assert not isinstance(err, Reject), err
    assert abs(err.e_bottom_px) < 1e-9, err.e_bottom_px

    # 底边在托盘**内侧**（v 比托盘底边更大 = 画面更下方）→ e_bottom < 0
    quad = synth_box((700.0, CY + 60.0), half_len, 0.0, height)
    err = servo_error(T, SIZE_MM, ["y=0", "x=W"], BoxObservation(quad=quad.tolist()), K)
    assert not isinstance(err, Reject), err
    assert abs(err.e_bottom_px + 60.0) < 1e-9, err.e_bottom_px

    # 底边在托盘**外**（v 比托盘底边更小）→ e_bottom > 0
    quad = synth_box((700.0, CY - 60.0), half_len, 0.0, height)
    err = servo_error(T, SIZE_MM, ["y=0", "x=W"], BoxObservation(quad=quad.tolist()), K)
    assert not isinstance(err, Reject), err
    assert abs(err.e_bottom_px - 60.0) < 1e-9, err.e_bottom_px
    print("    符号：内侧为负、外侧为正，压在边上为 0")


def test_aabb_and_quad_agree():
    """AABB 与四点是同一份数据结构——同一条轴对齐箱子必须给出同一组数。"""
    T = straight_down_camera()
    quad = synth_box((700.0, CY + 137.0), 150.0, 0.0, 300.0)
    u_lo, u_hi = float(quad[:, 0].min()), float(quad[:, 0].max())
    v_lo, v_hi = float(quad[:, 1].min()), float(quad[:, 1].max())

    by_quad = servo_error(T, SIZE_MM, ["y=0", "x=W"],
                          BoxObservation(quad=quad.tolist()), K)
    by_aabb = servo_error(T, SIZE_MM, ["y=0", "x=W"],
                          BoxObservation(u1=u_lo, v1=v_lo, u2=u_hi, v2=v_hi), K)
    assert not isinstance(by_quad, Reject) and not isinstance(by_aabb, Reject)
    assert abs(by_quad.e_bottom_px - by_aabb.e_bottom_px) < 1e-9
    assert abs(by_quad.e_right_px - by_aabb.e_right_px) < 1e-9
    assert abs(by_quad.theta_rad - by_aabb.theta_rad) < 1e-12
    assert (by_quad.box_source, by_aabb.box_source) == ("quad", "aabb")
    print("    AABB 与 quad 路径给出同一组数，只有 box_source 不同")


def test_warn_does_not_reject():
    """超出 ±45° 与垂足落在段外只置 warn，仍然输出。"""
    T = straight_down_camera()
    # 转 60°，超出 ±45°
    quad = synth_box((700.0, CY + 100.0), 150.0, math.radians(60.0), 300.0)
    err = servo_error(T, SIZE_MM, ["y=0", "x=W"],
                      BoxObservation(quad=quad.tolist()), K)
    assert not isinstance(err, Reject), err
    assert any("theta" in w for w in err.warn), err.warn

    # 箱子整个偏到托盘左边之外 → 底边垂足落在线段外
    quad = synth_box((-500.0, CY + 100.0), 150.0, 0.0, 300.0)
    err = servo_error(T, SIZE_MM, ["y=0", "x=W"],
                      BoxObservation(quad=quad.tolist()), K)
    assert not isinstance(err, Reject), err
    assert any("垂足" in w for w in err.warn), err.warn
    print(f"    warn 不拒绝输出：{err.warn}")


def test_servo_error_propagates_rejects_and_checks_config():
    """子步骤的拒绝要原样传上来；配置错误也要在同一个出口被判。"""
    T = straight_down_camera()
    good = BoxObservation(u1=600.0, v1=500.0, u2=900.0, v2=800.0)

    out = servo_error(T, SIZE_MM, ["y=0", "y=H"], good, K)
    assert isinstance(out, Reject) and out.code == "ref_edges_parallel", out

    out = servo_error(T, SIZE_MM, ["y=0", "x=W"],
                      BoxObservation(u1=600.0, v1=500.0, u2=600.0, v2=800.0), K)
    assert isinstance(out, Reject) and out.code == "box_degenerate", out

    far = np.eye(4)
    far[2, 3] = 500.0
    out = servo_error(far, SIZE_MM, ["y=0", "x=W"], good, K)
    assert isinstance(out, Reject) and out.code == "edge_too_short", out
    print("    子步骤的 Reject 原样上传（配置错误也在同一出口）")


def test_diagnostics_are_enough_to_redraw_the_frame():
    """诊断量必须够重画这一帧——少了它们出不了对比图，真机也排查不了。"""
    T = straight_down_camera()
    quad = synth_box((700.0, CY + 137.0), 150.0, math.radians(7.0), 300.0)
    err = servo_error(T, SIZE_MM, ["y=0", "x=W"],
                      BoxObservation(quad=quad.tolist()), K)
    assert not isinstance(err, Reject), err
    for name in ("ref_bottom_px", "ref_right_px", "box_bottom_px", "box_right_px",
                 "pallet_center_px", "bottom_foot_px", "right_foot_px",
                 "bottom_inward_px", "right_inward_px"):
        value = np.asarray(getattr(err, name), np.float64)
        assert value.size >= 2, (name, value)
        assert np.all(np.isfinite(value)), (name, value)
    # 内法向必须是单位向量
    for name in ("bottom_inward_px", "right_inward_px"):
        n = np.asarray(getattr(err, name), np.float64)
        assert abs(float(np.linalg.norm(n)) - 1.0) < 1e-9, (name, n)
    # 台面中心应当落在托盘矩形正中
    assert np.allclose(err.pallet_center_px,
                       [CX + PALLET_W_MM / 2, CY + PALLET_H_MM / 2], atol=1e-9)
    print("    诊断量齐备：两条参考边、两条箱边、台面中心、两个垂足、两个内法向")


# --------------------------------------------------------------------------- #
# 点击顺序与重排（`algorithm.PALLET_CLICK_PERMUTATION` / `reorder_pallet_clicks`）
# --------------------------------------------------------------------------- #
# **不抄常量，直接 import 算法层那一份** —— 离线工具 `pick_servo_inputs.py` 与
# 标定工具 `pallet_calibrate.py` 用的也是同一个对象，三处不再各存一份。
#
# 这里曾经"刻意抄一份常量"并注释说「抄错或哪边改了，下面那条会立刻红」——
# **那句话是假的**：本文件从不 import 工具，工具里的常量改成什么这里都照样绿。
# 抄一份字面量**不是**拦网。下面两条才是：它们**不依赖工具**，只钉算法层这一份，
# 而工具用的是同一份，所以真正改坏了任何一处都会红。


def test_permutation_reverses_the_operator_ring():
    """重排必须把操作员那圈**反向走一圈**（而不是任意换位）。

    操作员的 `[右下, 左下, 左上, 右上]` 是一圈**顺时针**环序，建系要的是它反向
    （逆时针）走一圈 —— 这是让 `e1 × e2 = normal` 成立、`det` 落在 +1 一侧的
    唯一走法。判据写成"每个点的后继都得是它在操作员序里的**前一个**"，
    于是 `(1, 0, 3, 2)` 的四个循环移位（只差谁是 origin）全部通过，
    而 `identity` 与 `(1, 0, 2, 3)` 这种**换位**都被拒 —— 后一类是"手性判据可能
    放行、但语义已经错了"的情况，光靠 `det` 拦不住。
    """
    p = PALLET_CLICK_PERMUTATION
    assert sorted(p) == [0, 1, 2, 3], f"置换必须是 0..3 的一个排列，实际 {p}"
    n = len(p)
    for k in range(n):
        nxt = p[(p.index(k) + 1) % n]
        assert nxt == (k - 1) % n, (
            f"置换 {p} 不是反向环序：{OPERATOR_CLICK_ORDER[k]} 的后继是 "
            f"{OPERATOR_CLICK_ORDER[nxt]}，反向环序要求它是 {OPERATOR_CLICK_ORDER[(k - 1) % n]}")
    print("    置换是反向环序（判据：每个点的后继 = 它在操作员序里的前一个）")


def test_permutation_puts_the_origin_at_the_bottom_left_corner():
    """重排后**第 0 个点必须是「左下」** —— 它同时钉死了"四个循环移位里的哪一个"。

    上一条只排到四个候选（互为循环移位）。这一条把 origin 定死，两条合起来
    `PALLET_CLICK_PERMUTATION` 就唯一了。origin 落在哪个角**不是口味问题**：
    `pallet_frame_from_clicks` 取 `origin = points[0]`、`e1 = points[1] - points[0]`、
    `e2 = points[3] - points[0]`，于是 origin 决定 `"y=0"`/`"x=W"` 各是哪条**物理边**
    —— 也就是 `ref_edges` 那张表。定错了不报错，只是伺服错了边。
    """
    reordered = reorder_pallet_clicks(
        [(1.0, 1.0), (0.0, 1.0), (0.0, 0.0), (1.0, 0.0)])   # 右下, 左下, 左上, 右上
    names = [OPERATOR_CLICK_ORDER[i] for i in PALLET_CLICK_PERMUTATION]
    assert names[0] == "左下", f"重排后第 0 个是 {names[0]}，必须是「左下」"
    assert names == ["左下", "右下", "右上", "左上"], names
    # 顺带把坐标也钉住：e1 沿下边向右、e2 沿左边向上 —— 这才给出
    # origin=左下 / y=0=下边 / x=W=右边
    assert reordered[0] == (0.0, 1.0), reordered      # origin = 左下
    assert reordered[1] == (1.0, 1.0), reordered      # e1 终点 = 右下 → e1 沿下边向右
    assert reordered[3] == (0.0, 0.0), reordered      # e2 终点 = 左上 → e2 沿左边向上
    print("    重排后 origin = 左下、e1 沿下边向右、e2 沿左边向上")


# --------------------------------------------------------------------------- #
# 手性自检（`algorithm.handedness_problem`）
# --------------------------------------------------------------------------- #


def _synth_pallet_depth(h=960, w=1120, mm=1000):
    """一张**平的**合成深度图：相机在台面正上方 1 m，满视场深度都是 `mm`。

    于是反投影是 1 px = 1 mm（`fx = fy = 1000`、`z = 1 m`），与文件头那套
    真值构造法一致。
    """
    return np.full((h, w), int(mm), np.uint16)


def _operator_clicks(shift=(0.0, 0.0)):
    """操作员顺序的四个点击 `[右下, 左下, 左上, 右上]`（合成的 800 x 400 mm 台面）。

    `shift` 加在**左下**那个点上 —— 用它模拟"手抖点偏了 k 个像素"。真实点击噪声
    就是这个量级（1 px 在 1.2 m 边上 ≈ 0.11°）。
    """
    du, dv = shift
    return [(1000.0, 800.0), (200.0 + du, 800.0 + dv),
            (200.0, 400.0), (1000.0, 400.0)]


def _frame_from_operator_clicks(shift=(0.0, 0.0), reorder=True):
    """按工具那条路建系；`reorder=False` 就是"内部置换没做/写反了"的那种输入。"""
    clicks = _operator_clicks(shift)
    pts = [clicks[i] for i in PALLET_CLICK_PERMUTATION] if reorder else clicks
    return pallet_frame_from_clicks(pts, _synth_pallet_depth(),
                                    FX, FY, CX, CY, 1.0)


def test_handedness_problem_accepts_a_right_handed_pallet_frame():
    """重排后的托盘系是**右手系**（`det = +1`）→ 必须放行。

    这条同时钉住"工具那份固定置换给出的就是右手系"：置换抄错一位这里就红。
    """
    frame = _frame_from_operator_clicks()
    assert frame is not None
    assert (frame.width_mm, frame.height_mm) == (800, 400), \
        (frame.width_mm, frame.height_mm)
    T = frame.to_matrix()
    det = float(np.linalg.det(T[:3, :3]))
    assert abs(det - 1.0) < 1e-9, det
    assert handedness_problem(T) is None, handedness_problem(T)
    print(f"    右手系：det = {det:+.6f} → 放行")


def test_handedness_problem_rejects_a_mirrored_pallet_frame():
    """环序点反（重排没做 / 做反了）得到的是**镜面**：必须拒，且话要说对。

    镜面在下游是**静默**的（`matrix_to_pose6d` 里的 `Rotation.from_matrix` 会把它
    投影成"最近的旋转"），所以这是唯一能在源头拦下来的地方。
    """
    frame = _frame_from_operator_clicks(reorder=False)     # 未重排 = 顺时针环
    assert frame is not None
    T = frame.to_matrix()
    det = float(np.linalg.det(T[:3, :3]))
    assert abs(det + 1.0) < 1e-9, det
    problem = handedness_problem(T)
    assert problem is not None, "镜面被放行了"
    assert "镜面" in problem, problem
    assert f"{det:+.6f}" in problem, problem        # 实际值要点出来
    print(f"    镜面：det = {det:+.6f} → 拒（原因里带实际行列式）")


def test_handedness_problem_tolerates_click_error():
    """点偏 1 / 2 / 5 px 的**正常点击**必须继续放行。

    这条正是已经回归过两次的地方：老判据是"`det` 离 +1 多近"（`|det−1| > 1e-6`
    就拒），那是拿容差去要求正交性 —— 一个像素的点击噪声就能把正常点击判成镜面。
    操作员真实跑一次复现了这个 Critical：`det = +0.999791` 被判"顺序点反了"、
    exit 1、零产物。判据必须是"det 落在**哪个假设**那一侧"。
    """
    for k in (1, 2, 5):
        frame = _frame_from_operator_clicks(shift=(float(k), 0.0))
        assert frame is not None, k
        T = frame.to_matrix()
        det = float(np.linalg.det(T[:3, :3]))
        assert handedness_problem(T) is None, (k, det, handedness_problem(T))
        # 同一组输入在老判据下**会**被拒 —— 这条断言就是"用例有牙"的证据
        assert abs(det - 1.0) > 1e-6, (k, det, "老判据也放行，这条用例没牙")
        print(f"    点偏 {k} px：det = {det:+.9f}"
              f"（|det−1| = {abs(det - 1):.2e} > 1e-6）→ 放行")


def test_handedness_problem_separates_near_collinear_from_mirrored():
    """`det ≈ 0`（两条点击边近乎共线）不许说成"顺序点反了" —— 那是**指错方向**。

    这种输入建不出坐标系来（`pallet_frame_from_clicks` 遇到四点近共线直接返回
    None），所以只能手工造一个矩阵；但判据本身对它要说对话。
    """
    T = np.eye(4)
    T[:3, :3] = [[1.0, 0.9999, 0.0],
                 [0.0, 0.0010, 0.0],
                 [0.0, 0.0, 1.0]]          # e1 与 e2 几乎同向 → det ≈ 0
    det = float(np.linalg.det(T[:3, :3]))
    assert abs(det) < 0.5, det
    problem = handedness_problem(T)
    assert problem is not None, det
    assert "共线" in problem, problem
    assert "镜面" not in problem, problem
    print(f"    det = {det:+.6f}（近乎共线）→ 拒，且不往「顺序点反了」上引")


def _params(**over):
    """一份能跑通的技能参数：托盘正上方、箱子偏了已知量。"""
    T = straight_down_camera()
    half, height = 150.0, 300.0
    # 底边中点的 u 由"右边垂距 = 88 px"反解（与
    # `test_synthetic_recovers_known_offsets_exactly` 同一套手算）——只有取
    # 这个值，下面断言的 137 / 88 才是真值。
    want_theta = math.radians(7.0)
    bottom_mid_u = (CX + PALLET_W_MM - 88.0
                    - half * math.cos(want_theta)
                    - (height / 2.0) * math.sin(want_theta))
    quad = synth_box((bottom_mid_u, CY + 137.0), half, want_theta, height)
    base = dict(pallet_pose=matrix_to_pose6d(T),
                box_obs=BoxObservation(quad=quad.tolist(), label="box"),
                pallet_size_mm=SIZE_MM,
                # 这里**显式**用 ("y=0","x=W") 这一对，不是默认值：合成场景的
                # 托盘系是抽象构造的（见文件头），这一对的几何最好手算。
                ref_edges=("y=0", "x=W"),
                K=[[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]],
                D=None,
                T_cam_base=np.eye(4).tolist(),
                image_size=IMAGE_SIZE,
                use_distortion=True)
    base.update(over)
    return PalletServoParams(**base)


def test_skill_matches_the_algorithm():
    """技能层不许自己加工几何：它的输出必须与直接调 `servo_error` 一模一样。"""
    params = _params()
    T = matrix_to_pose6d(straight_down_camera())
    direct = servo_error(pose6d_to_matrix(T), SIZE_MM, ("y=0", "x=W"),
                         params.box_obs, K, None, IMAGE_SIZE)
    assert not isinstance(direct, Reject), direct

    skill = PalletServoSkill()
    assert skill.initialize(params).success
    result = skill.execute()
    assert result.success, result.message
    got = result.data["error"]
    assert abs(got.e_bottom_px - direct.e_bottom_px) < 1e-9
    assert abs(got.e_right_px - direct.e_right_px) < 1e-9
    assert abs(got.theta_rad - direct.theta_rad) < 1e-12
    print(f"    技能层与算法层一致：e_bottom={got.e_bottom_px:.3f}、"
          f"e_right={got.e_right_px:.3f}、"
          f"theta={math.degrees(got.theta_rad):.3f}°")


def test_skill_applies_t_cam_base():
    """`T_cam_base` 必须真的乘进去。

    取 `T_base_pallet` 平移 x=0.4 m、`T_cam_base` 平移 x=-0.4 m，两者抵消，
    应当回到"托盘在相机正下方"那一组数。
    """
    T_base_pallet = straight_down_camera()
    T_base_pallet[0, 3] = 0.4
    T_cam_base = np.eye(4)
    T_cam_base[0, 3] = -0.4

    params = _params(pallet_pose=matrix_to_pose6d(T_base_pallet),
                     T_cam_base=T_cam_base.tolist())
    skill = PalletServoSkill()
    assert skill.initialize(params).success
    result = skill.execute()
    assert result.success, result.message
    got = result.data["error"]
    assert abs(got.e_bottom_px + 137.0) < 1e-9, got.e_bottom_px
    assert abs(got.e_right_px + 88.0) < 1e-9, got.e_right_px
    print("    T_cam_base 被正确施加（平移抵消后回到 −137/−88 px）")


def test_skill_startup_rejections_are_actionable():
    """§7 的启动时拒绝：配置错误必须**早失败**，且话说清楚。"""
    # ref_edges 平行
    skill = PalletServoSkill()
    out = skill.initialize(_params(ref_edges=("y=0", "y=H")))
    assert not out.success and "ref_edges_parallel" in out.message, out.message

    # 尺寸缺失
    skill = PalletServoSkill()
    out = skill.initialize(_params(pallet_size_mm=None))
    assert not out.success and "pallet_size_mm" in out.message, out.message

    # 尺寸退化为 0
    skill = PalletServoSkill()
    out = skill.initialize(_params(pallet_size_mm=(0.0, 800.0)))
    assert not out.success and "pallet_size_mm" in out.message, out.message

    # K 形状不对
    skill = PalletServoSkill()
    out = skill.initialize(_params(K=[[1.0, 0.0], [0.0, 1.0]]))
    assert not out.success and "K" in out.message, out.message

    # T_cam_base 形状不对
    skill = PalletServoSkill()
    out = skill.initialize(_params(T_cam_base=[[1.0, 0.0], [0.0, 1.0]]))
    assert not out.success and "T_cam_base" in out.message, out.message

    # image_size 形状不对：原来它是唯一一个**没查**的元组参数，坏值会活到
    # `on_execute` 里抛 IndexError、被当异常接住 → 判成"这一帧被拒"（每帧一条
    # 看着像几何退化的 WARNING + 每帧一份 dump），纯配置错误伪装成几何问题
    for bad in ([640], "640x480", [0, 480], [640, 480, 2]):
        skill = PalletServoSkill()
        out = skill.initialize(_params(image_size=bad))
        assert not out.success, (bad, out.message)
        assert "image_size" in out.message, (bad, out.message)
    # 不给是合法的（只是不查"整条边跑到图外"）
    skill = PalletServoSkill()
    assert skill.initialize(_params(image_size=None)).success
    print("    启动时拒绝：ref_edges / 尺寸 / K / T_cam_base / image_size "
          "六类都在 initialize 拦下")


def test_skill_use_distortion_switch():
    """`use_distortion=False` 时畸变系数必须真的不参与——不许两边默默分叉。"""
    D = [0.1, -0.05, 0.001, 0.002, 0.0]
    with_d = PalletServoSkill()
    assert with_d.initialize(_params(D=D, use_distortion=True)).success
    got_with = with_d.execute().data["error"]

    without = PalletServoSkill()
    assert without.initialize(_params(D=D, use_distortion=False)).success
    got_without = without.execute().data["error"]

    # 主点附近畸变影响很小但不为零；至少两者不能是同一个数
    assert (abs(got_with.e_bottom_px - got_without.e_bottom_px) > 1e-9
            or abs(got_with.theta_rad - got_without.theta_rad) > 1e-12), \
        "开了畸变与关了畸变给出同一组数，说明开关没接上"
    print(f"    use_distortion 开关生效："
          f"开 {got_with.e_bottom_px:.3f} / 关 {got_without.e_bottom_px:.3f} px")


def test_skill_reports_rejects_as_failures():
    """运行时退化要变成 Result.fail，且 message 里带实际值与阈值。"""
    quad = synth_box((700.0, CY + 100.0), 150.0, 0.0, 300.0)
    far = np.eye(4)
    far[2, 3] = 500.0
    skill = PalletServoSkill()
    assert skill.initialize(_params(pallet_pose=matrix_to_pose6d(far))).success
    result = skill.execute()
    assert not result.success, result.message
    assert "edge_too_short" in result.message, result.message
    assert "阈值" in result.message, result.message
    print(f"    运行退化 → Result.fail：{result.message}")


# --------------------------------------------------------------------------- #
# runner
# --------------------------------------------------------------------------- #
def main() -> int:
    cases = [
        ("参考边表", test_ref_edge_points_match_spec_table),
        ("投影单位（1mm = 1px）", test_projection_is_one_mm_per_px),
        ("参考边端点", test_ref_edge_px_returns_the_two_endpoints),
        ("退化帧被拒", test_ref_edge_px_rejects_degenerate_frames),
        ("Reject 是值对象", test_reject_is_a_frozen_hashable_value),
        ("ref_edges 校验", test_parse_ref_edges),
        ("绝对毫米写法", test_absolute_mm_edge_specs_parse),
        ("绝对毫米越界被拒", test_absolute_mm_out_of_range_is_rejected),
        ("平行判据按轴", test_parallel_check_uses_the_axis_not_the_first_character),
        ("绝对=符号：三个量一致", test_absolute_and_symbolic_specs_give_identical_servo_error),
        ("点击置换：反向环序", test_permutation_reverses_the_operator_ring),
        ("点击置换：origin 在左下", test_permutation_puts_the_origin_at_the_bottom_left_corner),
        ("手性自检：右手系放行", test_handedness_problem_accepts_a_right_handed_pallet_frame),
        ("手性自检：镜面被拒", test_handedness_problem_rejects_a_mirrored_pallet_frame),
        ("手性自检：点击误差放行", test_handedness_problem_tolerates_click_error),
        ("手性自检：共线≠镜面", test_handedness_problem_separates_near_collinear_from_mirrored),
        ("AABB 四角顺序", test_aabb_corners_follow_the_contract_order),
        ("AABB 给出顺序无关", test_aabb_order_does_not_depend_on_which_corner_came_first),
        ("quad 优先", test_quad_takes_priority_over_aabb),
        ("箱子退化帧被拒", test_bad_frames_are_rejected_with_reasons),
        ("角度无向折半圈", test_line_angle_is_undirected),
        ("精确还原已知偏移", test_synthetic_recovers_known_offsets_exactly),
        ("垂距符号", test_sign_says_which_side_of_the_pallet_edge),
        ("AABB 与 quad 一致", test_aabb_and_quad_agree),
        ("warn 不拒绝输出", test_warn_does_not_reject),
        ("拒绝原样上传", test_servo_error_propagates_rejects_and_checks_config),
        ("诊断量齐备", test_diagnostics_are_enough_to_redraw_the_frame),
        ("技能层与算法层一致", test_skill_matches_the_algorithm),
        ("T_cam_base 真的乘进去", test_skill_applies_t_cam_base),
        ("启动时拒绝可执行", test_skill_startup_rejections_are_actionable),
        ("use_distortion 开关", test_skill_use_distortion_switch),
        ("运行退化变 Result.fail", test_skill_reports_rejects_as_failures),
    ]
    failed = 0
    for title, fn in cases:
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {title}: {exc}")
        except Exception as exc:                       # noqa: BLE001
            failed += 1
            print(f"  ERROR {title}: {type(exc).__name__}: {exc}")

    print()
    if failed:
        print(f"FAILED: {failed}/{len(cases)}")
        return 1
    print(f"OK: {len(cases)}/{len(cases)} 全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
