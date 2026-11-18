# -*- coding: utf-8 -*-
"""partial 帧（缺一条边）的初值/重投影修复。

合成数据取自 bag2 真值：真托盘四角 + 真 normal/D，顶边（edge 2）缺失。
旧实现用逐角兜底补出的画面外假顶点做 PnP，θ0 被带到 -43.1°，再被
`θ0 ± 40°` 裁进坏盆地，可见边残差 11.6px、origin 偏 14.5mm。

修复后：partial 不跑 PnP / `_seed_from_obb`，只用 long-side 先验 + θ 多起点。
"""
import numpy as np
import pytest

from ..geom import CameraIntrinsics, _corners_mm, _project_px
from ..frame_geometry import order_quad
from ..solve_edges import solve_frame

pytestmark = pytest.mark.unit

# bag2 的相机 / 台面参数
K = CameraIntrinsics(366.23016357, 366.17419434, 321.08544922, 240.75099182)
NORMAL = np.array([-0.00410537, -0.59423562, -0.80428053])
W, H = 1033.9, 1141.1
D_TRUE = 1368.8

# 真托盘投影四角 [bl, br, tr, tl]（真值），以及逐角兜底会给出的假上顶点
TRUE_QUAD = np.array([[155.6, 374.6], [437.9, 374.1],
                      [398.7, 162.3], [211.0, 163.2]], float)
FAKE_QUAD = np.array([[155.6, 374.6], [437.9, 374.1],
                      [398.7, -66.6], [211.0, -63.2]], float)
TRUE_ORIGIN = np.array([-606.6, 490.6, 1342.5])


def _lines_from_quad(quad, drop=()):
    out = []
    for i in range(4):
        if i in drop:
            out.append(None)
            continue
        p0 = np.asarray(quad[i], float)
        p1 = np.asarray(quad[(i + 1) % 4], float)
        d = p1 - p0
        d /= np.linalg.norm(d)
        out.append({'p0': p0, 'd': d, 'weight': 1.0})
    return out


def _solve(lines, quad_hint, use_pos):
    return solve_frame(lines, K, NORMAL, size_mm=(W, H),
                       long_side_parallel=False, depth=None,
                       use_pos=use_pos, d_lock=D_TRUE,
                       quad_hint=quad_hint)


def test_partial_three_edges_ignore_fake_top_vertex():
    """真三边 + 画面外假上顶点：解必须回到真值（<1mm / 0.1px）。"""
    lines = _lines_from_quad(TRUE_QUAD, drop=(2,))
    found, diag = _solve(lines, FAKE_QUAD, [0, 1, 3])

    assert found is not None, diag
    assert diag['partial'] is True
    assert diag['seed'] == 'long_side_multistart'
    assert diag['seed_starts'] == 12
    # lazy：先验起点已落在正确盆地时不应补跑其余 11 个起点
    assert diag['seed_evals'] == 1

    px = _project_px(_corners_mm(found, W, H), K)
    # 输入真值四角只保留到 0.1px，角点欧氏差会到 ~0.4px；口径用"到真边线的
    # 垂距"（与 solver 的可见边残差一致）。旧实现在这里最大 11.6px。
    max_perp = 0.0
    for i in (0, 1, 3):
        p0 = TRUE_QUAD[i]
        p1 = TRUE_QUAD[(i + 1) % 4]
        u = (p1 - p0) / np.linalg.norm(p1 - p0)
        m = np.array([-u[1], u[0]])
        for j in (i, (i + 1) % 4):
            max_perp = max(max_perp, abs(float((px[j] - p0) @ m)))
    assert max_perp < 0.2, (px, max_perp, diag)
    assert np.linalg.norm(np.asarray(found['origin'], float)
                          - TRUE_ORIGIN) < 1.0, diag

    # partial 的 reproj 只对可见边算，不再被假顶点污染
    assert diag['reproj_px'] < 0.1, diag
    assert diag['reproj_visible_px'] < 0.1, diag
    # 真值四角只到 0.1px；最大端点垂距 0.155px（旧实现是 11.6px）
    assert diag['reproj_edge_px'] < 0.2, diag
    assert len(diag['reproj_edge_detail_px']) == len([0, 1, 3])


def test_partial_fake_and_true_quad_hint_give_same_pose():
    """partial 时 quad_hint 不得影响初值（假/真四角结果一致）。"""
    lines = _lines_from_quad(TRUE_QUAD, drop=(2,))
    f_fake, _ = _solve(lines, FAKE_QUAD, [0, 1, 3])
    f_true, _ = _solve(lines, TRUE_QUAD, [0, 1, 3])
    assert f_fake is not None and f_true is not None
    assert np.allclose(np.asarray(f_fake['origin'], float),
                       np.asarray(f_true['origin'], float), atol=1e-6)


def test_partial_without_quad_does_not_crash_or_fake_D():
    """partial 且 quad 为空（三条线、无 quad_hint）：不崩，也不拿假 quad 锁 D。

    `_resolve_d_lock(..., depth, quad=None, d_lock=None/'depth')` 会直接放弃
    锁 D（返回 None），走自由 D；这是允许的行为，这里钉住它不会抛异常。
    """
    lines = _lines_from_quad(TRUE_QUAD, drop=(2,))
    depth = np.zeros((8, 8), np.float32)
    found, diag = solve_frame(lines, K, NORMAL, size_mm=(W, H),
                              long_side_parallel=False, depth=depth,
                              use_pos=[0, 1, 3], d_lock=None,
                              quad_hint=None)
    assert found is not None, diag
    assert diag['partial'] is True
    assert diag['d_lock'] is None
    assert diag['d_lock_src'] is None
    assert diag['seed'] == 'long_side_multistart'
    assert diag['reproj_px'] < 1.0, diag


@pytest.mark.parametrize('pair', [(0, 1), (0, 3), (1, 2), (2, 3)])
def test_adjacent_two_edges_fixed_D_and_normal_are_exact(pair):
    """两条**相邻**边 + 固定 D + 真 normal：origin 误差应在几 mm 内。

    这是 partial 选边策略的精度前提；对边不在此列（见下一个用例）。
    """
    lines = _lines_from_quad(TRUE_QUAD)
    found, diag = solve_frame(
        lines, K, NORMAL, size_mm=(W, H), long_side_parallel=False,
        depth=None, use_pos=list(pair), d_lock=D_TRUE, quad_hint=FAKE_QUAD)
    assert found is not None, diag
    err = float(np.linalg.norm(np.asarray(found['origin'], float) - TRUE_ORIGIN))
    assert err < 3.0, (pair, err, diag)
    assert diag['reproj_px'] < 1.0, (pair, diag)


@pytest.mark.parametrize('pair', [(1, 3), (0, 2)])
def test_opposite_two_edges_alone_are_not_valid_constraints(pair):
    """两条对边只剩一个自由度：即使 solver 收敛，origin 也偏几百/几千 mm。

    因此 detect 侧必须用 `opposite_pair_only` 拒绝，不能把它们当唯一约束。
    """
    lines = _lines_from_quad(TRUE_QUAD)
    found, diag = solve_frame(
        lines, K, NORMAL, size_mm=(W, H), long_side_parallel=False,
        depth=None, use_pos=list(pair), d_lock=D_TRUE, quad_hint=FAKE_QUAD)
    assert found is not None, diag
    err = float(np.linalg.norm(np.asarray(found['origin'], float) - TRUE_ORIGIN))
    assert err > 100.0, (pair, err, diag)


def test_full_four_edges_behavior_unchanged():
    """四边完整：仍走 PnP，reproj 仍是到 quad 的中位距离（原行为，<1e-6）。"""
    frame_w = dict(origin=np.array([-400.0, -300.0, 1500.0]),
                   E1=np.array([1.0, 0.0, 0.0]),
                   E2=np.array([0.0, -1.0, 0.0]),
                   nrm=np.array([0.0, 0.0, -1.0]))
    quad = order_quad(_project_px(_corners_mm(frame_w, W, H), K))
    lines = _lines_from_quad(quad)
    found, diag = solve_frame(lines, K, frame_w['nrm'], size_mm=(W, H),
                              long_side_parallel=False, use_pos=[0, 1, 2, 3],
                              d_lock=None, quad_hint=quad)

    assert found is not None, diag
    assert diag['partial'] is False
    assert diag['seed'] in ('pnp', 'fallback')
    assert diag['seed_starts'] == 1
    assert diag['reproj_px'] < 1e-6, diag
    px = _project_px(_corners_mm(found, W, H), K)
    assert np.max(np.linalg.norm(px - quad, axis=1)) < 1e-6, (px, quad)


def test_rebuild_origin_from_pair_undoes_the_mirror_solution():
    """两邻边 partial 的镜像解：用观测线交点 + 固定 D 把 origin 拉回真角点。

    2026-11-11 现场：镜像解与正确解相差约 1.57m，`e_bottom` 一度到 5000+，
    `NodePalletObsV2` 连续报 `Δpos≈1.57m`。这里用现场坏帧的 T + 12 个关键点
    画出的底部/右边（edge 0/1），重建后的 origin 应回到正确角点。
    """
    from ..detect_pose import _rebuild_origin_from_line_pair

    K2 = CameraIntrinsics(365.40130615234375, 365.3028564453125,
                          321.3299560546875, 241.44024658203125)
    T = np.array([
        0.9999494868720934, -0.00909372281325473, -0.004281110794276484,
        0.28181050721153006,
        -0.00987607658780378, -0.9680885528847043, -0.2504136914883669,
        1.647910116759203,
        -0.0018673016545311097, 0.2504433228876243, -0.9681294620067795,
        0.49915529182519985,
        0.0, 0.0, 0.0, 1.0]).reshape(4, 4)
    W2, H2 = 950.9, 1276.5
    bad = dict(origin=T[:3, 3] * 1000.0, E1=T[:3, 0], E2=T[:3, 1],
               nrm=T[:3, 2], W=W2, H=H2)
    kp = np.array([
        [-15.4541, 442.2853], [126.1758, 438.1769], [288.8497, 434.0149],
        [444.7490, 429.1743], [433.1959, 265.1455], [420.0499, 106.1071],
        [409.3033, -6.6806], [289.2715, -4.6989], [168.3923, -2.9009],
        [44.2103, -1.5333], [25.5330, 112.7664], [4.8927, 274.2069]])

    def _line(p, q):
        p = np.asarray(p, float)
        q = np.asarray(q, float)
        d = q - p
        d /= np.linalg.norm(d)
        return {'p0': p, 'd': d, 'weight': 1.0}

    lines = [_line(kp[0], kp[3]), _line(kp[3], kp[6]), None, None]
    good = _rebuild_origin_from_line_pair(
        bad, lines, (0, 1), K2, D_mm=897.1)
    assert good is not None
    expected = np.array([-0.668, 0.437, 0.817]) * 1000.0
    # 观测线来自关键点（不是拟合线），允许 ~20mm
    assert float(np.linalg.norm(np.asarray(good['origin']) - expected)) < 20.0
