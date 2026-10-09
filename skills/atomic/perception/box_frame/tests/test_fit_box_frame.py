"""fit_box_frame 的合成自检。跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 skills/atomic/perception/box_frame/tests/test_fit_box_frame.py

**不需要 ROS、不需要相机、不需要任何数据** —— 它自己合成深度图、自己造真值。
"""
from __future__ import annotations

import sys
from pathlib import Path

if __name__ == '__main__':
    # 本文件用的是相对 import（`from ..algorithm import ...`），这样 pytest 收集
    # 得到、也不会把仓库根塞进 sys.path 污染别的测试。但直接 `python3 <本文件>`
    # 跑时 `__package__` 是空的，相对 import 会 ImportError —— 仓库里其余测试脚本
    # 的约定是「直接跑、退出码 0/1」，所以这里把包名补上再让解释器自己去加载包。
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
    __package__ = 'skills.atomic.perception.box_frame.tests'
    import importlib
    importlib.import_module(__package__)

import numpy as np

_fails: list[str] = []


def check(name: str, cond: bool, detail: str = '') -> None:
    if cond:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        _fails.append(name)


def _k():
    from ..algorithm import CameraIntrinsics
    return CameraIntrinsics(368.523, 368.458, 320.364, 245.712)


def _synth_depth_with_box(z_box: float = 800.0, z_bg: float = 1100.0):
    """合成深度图：中央一块 530x350 的箱子顶面，其余是背景。

    尺寸按几何自洽换算（见计划 Global Constraints）：
      530mm @ 800mm -> 244px；350mm @ 800mm -> 161px
    """
    k = _k()
    depth = np.full((480, 640), z_bg, np.uint16)
    depth[170:331, 200:444] = z_box          # 161 行 x 244 列
    return depth, k


def test_geometry_self_consistency() -> None:
    """先钉死换算关系，后面所有合成数据都靠它。"""
    k = _k()
    from ..algorithm import _backproject
    px = _backproject(np.array([200.0, 444.0]), np.array([170.0, 331.0]),
                      np.array([800.0, 800.0]), k)
    du = float(px[1, 0] - px[0, 0])
    dv = float(px[1, 1] - px[0, 1])
    check('244px @800mm ≈ 530mm', abs(du - 530.0) < 8.0, f'{du:.1f}')
    check('161px @800mm ≈ 350mm', abs(dv - 350.0) < 8.0, f'{dv:.1f}')


def test_roi_from_box() -> None:
    from ..algorithm import roi_from_box
    roi = roi_from_box((100.0, 50.0, 300.0, 250.0), (480, 640))
    check('roi 左边界', roi[0] == 100, str(roi))
    check('roi 上边界', roi[1] == 50, str(roi))
    check('roi 右边界', roi[2] == 300, str(roi))
    check('roi 下边界', roi[3] == 250, str(roi))
    check('不外扩', roi == (100, 50, 300, 250), str(roi))
    roi2 = roi_from_box((5.0, 5.0, 635.0, 475.0), (480, 640))
    check('roi 裁剪不越界', roi2 == (5, 5, 635, 475), str(roi2))
    roi3 = roi_from_box((-10.0, -10.0, 700.0, 500.0), (480, 640))
    check('roi 裁到图像内', roi3 == (0, 0, 640, 480), str(roi3))


def test_valid_depth_points() -> None:
    from ..algorithm import valid_depth_points
    k = _k()
    depth = np.zeros((480, 640), np.uint16)
    depth[100:110, 100:110] = 1000
    pts = valid_depth_points(depth, k, (100, 100, 110, 110))
    check('点数', len(pts) == 100, str(len(pts)))
    check('z 正确', np.allclose(pts[:, 2], 1000.0), str(pts[:, 2].mean()))
    check('x 符号（u=100 在主点左侧）', pts[:, 0].max() < 0, str(pts[:, 0].max()))


def test_load_frame() -> None:
    from ..algorithm import load_frame
    seq = Path('/data/Real_Downloads/maduo/test_data/5_test')
    if not seq.exists():
        print('  SKIP  test_load_frame（5_test 不存在）')
        return
    color, depth, k = load_frame(seq, '1789543127.504033')
    check('color 形状', color.shape == (480, 640, 3), str(color.shape))
    check('depth dtype', depth.dtype == np.uint16, str(depth.dtype))
    check('depth 有无效值', (depth == 0).any())
    check('内参 fx', abs(k.fx - 368.523) < 0.01, str(k.fx))


# --------------------------------------------------------------------------- #
# 第一批：几何层
# --------------------------------------------------------------------------- #
def test_depth_discontinuity_finds_step() -> None:
    from ..algorithm import depth_discontinuity
    d = np.full((120, 160), 1200, np.uint16)
    d[60:, :] = 1000                       # 下半部分近 200mm，第 59/60 行间是阶跃
    gx, gy = depth_discontinuity(d)
    mag = np.hypot(gx, gy)
    check('幅图形状', mag.shape == d.shape, str(mag.shape))
    peak_row = int(np.argmax(mag[:, 20:140].mean(axis=1)))
    check('阶跃峰值行', 57 <= peak_row <= 62, f'峰值在 {peak_row} 行')


def test_depth_discontinuity_ignores_invalid() -> None:
    from ..algorithm import depth_discontinuity, GRAD_MIN_MM_PER_PX
    d = np.full((120, 160), 1200, np.uint16)
    d[:, 80:] = 0                          # 右半边全无效
    gx, gy = depth_discontinuity(d)
    mag = np.hypot(gx, gy)
    check('无效值不造假边', float(mag.max()) < GRAD_MIN_MM_PER_PX,
          f'最大幅值 {mag.max():.1f}')


def test_depth_discontinuity_smooth_plane_is_quiet() -> None:
    from ..algorithm import depth_discontinuity, GRAD_MIN_MM_PER_PX
    u = np.arange(160, dtype=np.float64)[None, :]
    d = np.repeat((1200.0 + 0.8 * u).astype(np.uint16), 120, axis=0)
    gx, gy = depth_discontinuity(d)
    mag = np.hypot(gx, gy)
    check('缓变平面不越阈', float(mag.max()) < GRAD_MIN_MM_PER_PX,
          f'最大幅值 {mag.max():.1f}')


def test_split_planes_two_planes() -> None:
    from ..algorithm import split_planes
    rng = np.random.default_rng(0)
    low = np.c_[rng.uniform(-200, 330, 3000), rng.uniform(-150, 200, 3000),
                np.full(3000, 800.0)]
    high = np.c_[rng.uniform(-500, 900, 1500), rng.uniform(-400, 600, 1500),
                 np.full(1500, 1100.0)]
    pts = np.vstack([low, high]) + rng.normal(0, 1.5, (4500, 3))
    planes = split_planes(pts, min_points=500)
    check('分出两个平面', len(planes) == 2, str([p['n_points'] for p in planes]))
    if len(planes) < 2:
        return
    check('最大平面是背景', planes[0]['n_points'] > planes[1]['n_points'],
          str([p['n_points'] for p in planes]))
    check('法向近似 ±z', abs(abs(planes[0]['nrm'][2]) - 1.0) < 0.01,
          str(planes[0]['nrm']))


def test_plane_projection_roundtrip() -> None:
    from ..algorithm import plane_basis, project_to_plane, unproject_from_plane
    nrm = np.array([0.0, 0.0, -1.0])
    origin = np.array([10.0, 20.0, 800.0])
    rng = np.random.default_rng(1)
    pts = np.c_[rng.uniform(-100, 100, 50), rng.uniform(-100, 100, 50),
                np.full(50, 800.0)]
    p2 = project_to_plane(pts, nrm, origin)
    back = unproject_from_plane(p2, nrm, origin)
    check('往返一致', np.allclose(back, pts, atol=1e-6),
          str(np.abs(back - pts).max()))
    e1, e2 = plane_basis(nrm)
    check('基正交', abs(float(e1 @ e2)) < 1e-9, str(float(e1 @ e2)))
    check('基垂直于法向', abs(float(e1 @ nrm)) < 1e-9 and abs(float(e2 @ nrm)) < 1e-9)


def test_segment_top_plane_picks_box_face() -> None:
    """打分是「内点数 × 落在框内的比例」：背景点虽多，但在框内几乎为零。"""
    from ..algorithm import segment_top_plane
    depth, k = _synth_depth_with_box()
    plane, mask, score = segment_top_plane(
        depth, k, (150, 100, 494, 381), (200.0, 170.0, 444.0, 331.0), {})
    check('找到顶面', plane is not None, str(score))
    if plane is None:
        return
    check('顶面 z 接近 800', abs(plane['origin'][2] - 800.0) < 40.0,
          str(plane['origin']))
    check('内点数够', plane['n_points'] > 10000, str(plane['n_points']))
    check('掩码落在箱子区域', bool(mask[250, 300]) and not bool(mask[50, 50]),
          f"箱子内 {mask[250, 300]}  背景 {mask[50, 50]}")


def _synth_rect_pts(w: float, h: float, theta_deg: float, n: int = 6000,
                    bite_frac: float = 0.0, rng=None) -> np.ndarray:
    """合成一个 w×h 矩形内部的点，可选在 (+u,+v) 角挖掉一块模拟遮挡。"""
    rng = rng or np.random.default_rng(7)
    a = rng.uniform(0, w, n)
    b = rng.uniform(0, h, n)
    if bite_frac > 0:
        keep = ~((a > w * (1 - bite_frac)) & (b > h * (1 - bite_frac)))
        a, b = a[keep], b[keep]
    a, b = a - w / 2.0, b - h / 2.0
    th = np.deg2rad(theta_deg)
    x = a * np.cos(th) - b * np.sin(th)
    y = a * np.sin(th) + b * np.cos(th)
    return np.c_[x, y] + rng.normal(0, 3.0, (len(x), 2))


def test_extract_segments_from_rectangle() -> None:
    from ..algorithm import extract_line_segments
    pts = _synth_rect_pts(530.0, 350.0, 12.0)
    segs = extract_line_segments(pts)
    check('至少两段', len(segs) >= 2, str(len(segs)))
    # 四条边线都会返回，其中必然有平行的（u+/u-），所以判据是「存在垂直对」
    # 而不是「最长两条垂直」。
    dirs = [np.asarray(s['dir']) for s in segs]
    has_perp = any(abs(float(dirs[i] @ dirs[j])) < 0.15
                   for i in range(len(dirs)) for j in range(i + 1, len(dirs)))
    check('存在相互垂直的两段', has_perp,
          str([np.round(d, 2).tolist() for d in dirs]))
    if segs:
        check('最长段跨度接近 530',
              abs(segs[0]['span_mm'] - 530.0) < 40.0, f"{segs[0]['span_mm']:.0f}")


def test_candidates_cover_points() -> None:
    from ..algorithm import (SIZE_TOL_MM, extract_line_segments,
                               initial_frame_candidates)
    for bite in (0.0, 0.45):
        pts = _synth_rect_pts(530.0, 350.0, 12.0, bite_frac=bite)
        segs = extract_line_segments(pts)
        cands, diag = initial_frame_candidates(segs, pts, (530.0, 350.0))
        check(f'挖 {bite:.0%} 时建出候选', cands is not None, str(diag))
        if cands is None:
            continue
        # 每个候选都必须装得下所有点 —— 这是「朝向假设」的有效性判据。
        # ⚠️ 2026-09-21 试过删掉这条上界（因为它在 9_test 上误杀了 4/5 帧），
        # **不行**：删掉之后 6_test 选到了错的候选 —— 角度没怎么变、但矩形整体
        # 位置飘了，四角总误差 7.9px → 21.2px。这条上界是有用的。
        for c in cands:
            rel = pts - np.asarray(c['origin2d'])[None, :]
            a, b = rel @ c['e1'], rel @ c['e2']
            check(f'候选({c["W"]:.0f}x{c["H"]:.0f},{c["corner"]},挖{bite:.0%})装得下',
                  a.max() <= c['W'] + SIZE_TOL_MM and b.max() <= c['H'] + SIZE_TOL_MM
                  and a.min() >= -SIZE_TOL_MM and b.min() >= -SIZE_TOL_MM,
                  f'a {a.min():.0f}..{a.max():.0f}  b {b.min():.0f}..{b.max():.0f}')


# --------------------------------------------------------------------------- #
# 第二批：拟合层
# --------------------------------------------------------------------------- #
def test_candidate_to_frame_is_planar() -> None:
    from ..algorithm import candidate_to_frame
    plane = {'nrm': np.array([0.0, 0.0, -1.0]),
             'origin': np.array([0.0, 0.0, 800.0])}
    cand = {'origin2d': np.array([-265.0, -175.0]), 'e1': np.array([1.0, 0.0]),
            'e2': np.array([0.0, 1.0]), 'W': 530.0, 'H': 350.0,
            'long_along_e1': True}
    frame = candidate_to_frame(cand, plane)
    check('origin 在平面上',
          abs(float((frame['origin'] - plane['origin']) @ plane['nrm'])) < 1e-6,
          str(frame['origin']))
    check('E1/E2 正交', abs(float(frame['E1'] @ frame['E2'])) < 1e-9)
    check('nrm 一致', np.allclose(frame['nrm'], plane['nrm']))
    check('W/H 正确', (frame['W'], frame['H']) == (530.0, 350.0),
          f"{frame['W']}x{frame['H']}")


def test_collect_observations_on_synthetic_box() -> None:
    """把真值矩形投影到图像，再在深度不连续图上找它自己的边。"""
    from ..algorithm import (candidate_to_frame, collect_observations,
                               depth_discontinuity, plane_basis, project_to_plane)
    from ..algorithm import _backproject
    depth, k = _synth_depth_with_box()
    gx, gy = depth_discontinuity(depth)
    uv = np.array([[200.0, 170.0], [444.0, 170.0], [444.0, 331.0], [200.0, 331.0]])
    P = _backproject(uv[:, 0], uv[:, 1], np.full(4, 800.0), k)
    nrm = np.array([0.0, 0.0, -1.0])
    origin = np.array([0.0, 0.0, 800.0])
    p2 = project_to_plane(P, nrm, origin)
    cand = {'origin2d': p2[0], 'e1': p2[1] - p2[0], 'e2': p2[3] - p2[0],
            'W': 530.0, 'H': 350.0, 'long_along_e1': True}
    cand['e1'] = cand['e1'] / np.linalg.norm(cand['e1'])
    cand['e2'] = cand['e2'] / np.linalg.norm(cand['e2'])
    frame = candidate_to_frame(cand, {'nrm': nrm, 'origin': origin})
    obs, diag = collect_observations(frame, gx, gy, depth, k,
                                     {'z_range': (750.0, 850.0)})
    used = [e for e in diag['edges'] if e['used']]
    check('四条边都判为可观测', len(used) == 4,
          str([(e['name'], round(e['support'], 2), e['n_obs']) for e in diag['edges']]))
    check('观测点够多', len(obs) > 20, str(len(obs)))


def test_choose_candidate_picks_upright_orientation() -> None:
    """合成箱子在图上宽 244 > 高 161，所以选出的矩形投影也必须宽 > 高。"""
    from ..algorithm import (choose_candidate, depth_discontinuity,
                               extract_line_segments, initial_frame_candidates,
                               project_to_plane, segment_top_plane)
    from ..algorithm import _backproject, _project_px
    from ..algorithm import _corners_mm
    depth, k = _synth_depth_with_box()
    gx, gy = depth_discontinuity(depth)
    box_uv = (200.0, 170.0, 444.0, 331.0)
    plane, mask, _ = segment_top_plane(depth, k, (150, 100, 494, 381), box_uv, {})
    check('平面找到', plane is not None)
    if plane is None:
        return
    vs, us = np.nonzero(mask)
    P = _backproject(us.astype(np.float64), vs.astype(np.float64),
                     depth[vs, us].astype(np.float64), k)
    p2 = project_to_plane(P, plane['nrm'], plane['origin'])
    cands, cdiag = initial_frame_candidates(extract_line_segments(p2), p2,
                                            (530.0, 350.0))
    check('候选建出', cands is not None, str(cdiag))
    if cands is None:
        return
    frame, chdiag = choose_candidate(cands, plane, gx, gy, depth, k,
                                     {'z_range': (750.0, 850.0)})
    check('选出 frame', frame is not None, str(chdiag))
    if frame is None:
        return
    px = _project_px(_corners_mm(frame, frame['W'], frame['H']), k)
    w_px = float(px[:, 0].max() - px[:, 0].min())
    h_px = float(px[:, 1].max() - px[:, 1].min())
    check('矩形投影宽 > 高', w_px > h_px, f'{w_px:.0f}x{h_px:.0f}')


def _chain_to_frame0():
    """跑通 分割 -> 投影 -> 候选 -> 选优，返回 (frame0, plane, depth, k, gx, gy)。"""
    from ..algorithm import (choose_candidate, depth_discontinuity,
                               extract_line_segments, initial_frame_candidates,
                               project_to_plane, segment_top_plane)
    from ..algorithm import _backproject
    depth, k = _synth_depth_with_box()
    gx, gy = depth_discontinuity(depth)
    box_uv = (200.0, 170.0, 444.0, 331.0)
    plane, mask, _ = segment_top_plane(depth, k, (150, 100, 494, 381), box_uv, {})
    if plane is None:
        return None, None, depth, k, gx, gy
    vs, us = np.nonzero(mask)
    P = _backproject(us.astype(np.float64), vs.astype(np.float64),
                     depth[vs, us].astype(np.float64), k)
    p2 = project_to_plane(P, plane['nrm'], plane['origin'])
    cands, _ = initial_frame_candidates(extract_line_segments(p2), p2, (530.0, 350.0))
    if cands is None:
        return None, plane, depth, k, gx, gy
    frame0, _ = choose_candidate(cands, plane, gx, gy, depth, k,
                                 {'z_range': (750.0, 850.0)})
    return frame0, plane, depth, k, gx, gy


def test_solve_updates_held_partition() -> None:
    from ..algorithm import solve_frame
    frame0, _plane, depth, k, _gx, _gy = _chain_to_frame0()
    check('有初值', frame0 is not None)
    if frame0 is None:
        return
    frame, diag = solve_frame(frame0, frame0['_obs'], k, {})
    check('求解返回 frame', frame is not None, str(diag))
    if frame is None:
        return
    updated, held = set(diag['updated']), set(diag['held'])
    check('updated 与 held 不重叠', not (updated & held), str((updated, held)))
    check('updated ∪ held 正好是三自由度',
          updated | held == {'t_e1', 't_e2', 'theta'}, str((updated, held)))


def test_solve_degraded_when_only_one_family() -> None:
    """只给一个族的观测时，另外两个自由度必须如实标 held，且数值不变。"""
    from ..algorithm import solve_frame
    frame0, _plane, depth, k, _gx, _gy = _chain_to_frame0()
    if frame0 is None:
        check('有初值（降级测试前置）', False, '链路没跑通')
        return
    # 只留 'x' 族（索引 1、3）的观测
    only_x = [(i, uv) for i, uv in frame0['_obs'] if i in (1, 3)]
    if not only_x:
        check('能找到 x 族观测（降级测试前置）', False, '没有 x 族观测')
        return
    frame0['_families'] = frozenset({'x'})
    frame, diag = solve_frame(frame0, only_x, k, {})
    check('降级求解成功', frame is not None, str(diag))
    if frame is None:
        return
    check('updated 只有 t_e1', set(diag['updated']) == {'t_e1'},
          str(diag['updated']))
    check('held 是 t_e2 和 theta', set(diag['held']) == {'t_e2', 'theta'},
          str(diag['held']))
    check('confidence 为 degraded', diag['confidence'] == 'degraded',
          str(diag['confidence']))
    check('held 的自由度数值未变',
          abs(float((frame['origin'] - frame0['origin']) @ frame0['E2'])) < 1e-6,
          str(frame['origin'] - frame0['origin']))


def test_order_corners_uv() -> None:
    from ..algorithm import order_corners_uv
    uv = [(50.0, 40.0), (300.0, 40.0), (300.0, 240.0), (50.0, 240.0)]
    out = order_corners_uv(uv)
    check('第一个是右下', tuple(out[0]) == (300.0, 240.0), str(out[0]))
    check('第二个是左下', tuple(out[1]) == (50.0, 240.0), str(out[1]))
    check('第三个是左上', tuple(out[2]) == (50.0, 40.0), str(out[2]))
    check('第四个是右上', tuple(out[3]) == (300.0, 40.0), str(out[3]))
    shuffled = [uv[2], uv[0], uv[3], uv[1]]
    check('与输入顺序无关', np.allclose(order_corners_uv(shuffled), out))
    th = np.deg2rad(30.0)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    rot = [(R @ np.array(p)).tolist() for p in uv]
    out2 = order_corners_uv(rot)
    check('旋转后第一个仍是 u+v 最大',
          int(np.argmax(out2.sum(axis=1))) == 0, str(out2.sum(axis=1)))


def test_fit_box_frame_end_to_end_synthetic() -> None:
    from ..algorithm import fit_box_frame
    depth, k = _synth_depth_with_box()
    color = np.full((480, 640, 3), 128, np.uint8)
    result, diag = fit_box_frame(color, depth, k, (200.0, 170.0, 444.0, 331.0),
                                 target_mm=(530.0, 350.0))
    check('端到端成功', result is not None, str(diag.get('reject')))
    if result is None:
        return
    check('四角数量', len(result['corners_uv']) == 4)
    u, v = zip(*result['corners_uv'])
    check('四角落在箱子足迹内',
          min(u) > 170 and max(u) < 480 and min(v) > 140 and max(v) < 365,
          f'u {min(u):.0f}..{max(u):.0f}  v {min(v):.0f}..{max(v):.0f}')
    check('updated/held 是划分',
          set(diag['updated']) | set(diag['held']) == {'t_e1', 't_e2', 'theta'}
          and not (set(diag['updated']) & set(diag['held'])),
          str((diag['updated'], diag['held'])))


# --------------------------------------------------------------------------- #
# 第三批：交付层
# --------------------------------------------------------------------------- #
def test_render_box_check_marks_pixels() -> None:
    """数值自检：渲染后回读 —— 拟合出的边应该在图上留下红色像素。"""
    from ..algorithm import fit_box_frame, render_box_check
    depth, k = _synth_depth_with_box()
    color = np.full((480, 640, 3), 128, np.uint8)
    box_uv = (200.0, 170.0, 444.0, 331.0)
    result, diag = fit_box_frame(color, depth, k, box_uv, target_mm=(530.0, 350.0))
    if result is None:
        check('端到端成功（渲染前置）', False, str(diag.get('reject')))
        return
    vis = render_box_check(color, result, diag, k, box_uv)
    check('尺寸不变', vis.shape == color.shape, str(vis.shape))
    red = int(((vis[:, :, 2] > 200) & (vis[:, :, 1] < 60) & (vis[:, :, 0] < 60)).sum())
    check('有红色边像素', red > 100, str(red))


def test_render_handles_failure() -> None:
    """失败时也要出一张图，且不能抛异常。"""
    from ..algorithm import render_box_check
    color = np.full((480, 640, 3), 128, np.uint8)
    vis = render_box_check(color, None, {'reject': 'no_plane'}, _k(),
                           (200.0, 170.0, 444.0, 331.0))
    check('失败图尺寸不变', vis.shape == color.shape, str(vis.shape))
    check('失败图有文字', int((vis > 200).sum()) > 50, str(int((vis > 200).sum())))


def test_occlusion_still_covers_points() -> None:
    """把顶面数据挖掉一大块（模拟手臂遮挡），候选矩形仍应装下所有可见点。"""
    from ..algorithm import (SIZE_TOL_MM, extract_line_segments,
                               initial_frame_candidates, project_to_plane,
                               segment_top_plane)
    from ..algorithm import _backproject
    depth, k = _synth_depth_with_box()
    depth[170:260, 360:444] = 1100          # 挖掉右上角约 40% 的可见区域
    box_uv = (200.0, 170.0, 444.0, 331.0)
    plane, mask, _ = segment_top_plane(depth, k, (150, 100, 494, 381), box_uv, {})
    check('遮挡下仍找到平面', plane is not None)
    if plane is None:
        return
    vs, us = np.nonzero(mask)
    P = _backproject(us.astype(np.float64), vs.astype(np.float64),
                     depth[vs, us].astype(np.float64), k)
    p2 = project_to_plane(P, plane['nrm'], plane['origin'])
    segs = extract_line_segments(p2)
    check('遮挡下仍有边线', len(segs) >= 1, str(len(segs)))
    if not segs:
        return
    cands, diag = initial_frame_candidates(segs, p2, (530.0, 350.0))
    check('遮挡下仍建出候选', cands is not None, str(diag))
    if cands is None:
        return
    for c in cands:
        rel = p2 - np.asarray(c['origin2d'])[None, :]
        a, b = rel @ c['e1'], rel @ c['e2']
        check(f'遮挡下候选 {c["W"]:.0f}x{c["H"]:.0f} 装得下点集',
              a.max() <= c['W'] + SIZE_TOL_MM and b.max() <= c['H'] + SIZE_TOL_MM,
              f'a {a.min():.0f}..{a.max():.0f}  b {b.min():.0f}..{b.max():.0f}')


def test_full_chain_under_occlusion() -> None:
    """遮挡下走完整链路：要么给出结果，要么明确报失败 —— 不许崩。"""
    from ..algorithm import fit_box_frame
    depth, k = _synth_depth_with_box()
    depth[170:260, 360:444] = 1100
    color = np.full((480, 640, 3), 128, np.uint8)
    result, diag = fit_box_frame(color, depth, k, (200.0, 170.0, 444.0, 331.0),
                                 target_mm=(530.0, 350.0))
    if result is None:
        check('遮挡下端到端给出明确失败', bool(diag.get('reject')), str(diag))
    else:
        u, v = zip(*result['corners_uv'])
        check('遮挡下四角仍在合理范围',
              min(u) > 150 and max(u) < 520 and min(v) > 130 and max(v) < 400,
              f'u {min(u):.0f}..{max(u):.0f}  v {min(v):.0f}..{max(v):.0f}')
        check('遮挡下 updated/held 仍是划分',
              set(diag['updated']) | set(diag['held'])
              == {'t_e1', 't_e2', 'theta'}, str(diag))



# --------------------------------------------------------------------------- #
# 滑动时间窗（window.BoxFrameWindow）：规则由操作员 2026-09-21 定
# --------------------------------------------------------------------------- #
class _StubFit:
    """替身：按脚本返回结果，用来把窗口逻辑单独测出来（不跑真算法）。"""

    def __init__(self, script):
        self.script = list(script)          # 每项 True(成功) / False(失败)
        self.n_calls = 0

    def __call__(self, color, depth, k, box_uv, **kw):
        ok = self.script[self.n_calls % len(self.script)]
        self.n_calls += 1
        i = self.n_calls
        if not ok:
            return None, {'ok': False, 'reject': 'no_plane'}
        corners = np.array([[10.0 + i, 20.0], [0.0 + i, 20.0],
                            [0.0 + i, 0.0], [10.0 + i, 0.0]])
        return {'corners_uv': corners}, {'ok': True, 'confidence': 'normal'}


def _with_stub(script, fn):
    from .. import window as W
    old = W.fit_box_frame
    W.fit_box_frame = _StubFit(script)
    try:
        return fn(W)
    finally:
        W.fit_box_frame = old


def test_yolo_fallback_corners() -> None:
    from ..window import yolo_fallback_corners
    c = yolo_fallback_corners((100.0, 50.0, 300.0, 250.0))
    check('右下', tuple(c[0]) == (300.0, 250.0), str(c[0]))
    check('左下', tuple(c[1]) == (100.0, 250.0), str(c[1]))
    check('左上', tuple(c[2]) == (100.0, 50.0), str(c[2]))
    check('右上', tuple(c[3]) == (300.0, 50.0), str(c[3]))
    # 顺序必须与 fit_box_frame 一致：右下→左下→左上→右上
    c2 = yolo_fallback_corners((300.0, 250.0, 100.0, 50.0))   # 反向给也一样
    check('反向框同结果', np.allclose(c, c2), str(c2))


def test_window_process_every() -> None:
    def go(W):
        win = W.BoxFrameWindow(_k(), process_every=3)
        got = [win.push(None, None, (0., 0., 1., 1.)) is not None for _ in range(7)]
        return got
    got = _with_stub([True], go)
    check('每 3 帧处理一次', got == [True, False, False, True, False, False, True], str(got))


def test_window_only_one_good_frame() -> None:
    """5 帧里只有 1 帧有结果 -> 就用这一帧（操作员规则）。"""
    def go(W):
        win = W.BoxFrameWindow(_k(), process_every=1)
        last = None
        for _ in range(5):
            last = win.push(None, None, (0., 0., 1., 1.))
        return last
    out = _with_stub([True, False, False, False, False], go)
    check('source=single', out['source'] == 'single', str(out['source']))
    check('n_used=1', out['n_used'] == 1, str(out['n_used']))
    check('用的是那一帧的四角', np.allclose(out['corners_uv'][0], (11.0, 20.0)),
          str(out['corners_uv'][0]))


def test_window_all_fail_falls_back_to_yolo() -> None:
    """5 帧全失败 -> 退回原始 YOLO 框，**并且要标注**。"""
    def go(W):
        win = W.BoxFrameWindow(_k(), process_every=1)
        last = None
        for _ in range(5):
            last = win.push(None, None, (100.0, 50.0, 300.0, 250.0))
        return last
    from ..window import yolo_fallback_corners
    out = _with_stub([False], go)
    check('source=yolo_fallback', out['source'] == 'yolo_fallback', str(out['source']))
    check('n_used=0', out['n_used'] == 0, str(out['n_used']))
    check('用的是 YOLO 框四角',
          np.allclose(out['corners_uv'], yolo_fallback_corners((100., 50., 300., 250.))),
          str(out['corners_uv']))
    check('带上了失败原因', out['rejects'] == ['no_plane'] * 5, str(out['rejects']))


def test_window_mean_of_good_frames() -> None:
    """≥2 帧有结果 -> 求平均；失败帧不参与。"""
    def go(W):
        win = W.BoxFrameWindow(_k(), process_every=1)
        last = None
        for _ in range(5):
            last = win.push(None, None, (0., 0., 1., 1.))
        return last
    out = _with_stub([True, True, False, True, False], go)
    # 成功的是第 1/2/4 次调用 -> 右下 u = 11,12,14 -> 平均 12.333
    check('source=window', out['source'] == 'window', str(out['source']))
    check('n_used=3', out['n_used'] == 3, str(out['n_used']))
    check('n_failed=2', out['n_failed'] == 2, str(out['n_failed']))
    check('是平均值', abs(out['corners_uv'][0][0] - (11 + 12 + 14) / 3) < 1e-9,
          str(out['corners_uv'][0]))
    check('spread 有值', out['spread_px'] is not None and out['spread_px'] > 0,
          str(out['spread_px']))


def test_window_median_option() -> None:
    def go(W):
        win = W.BoxFrameWindow(_k(), process_every=1, agg='median')
        last = None
        for _ in range(3):
            last = win.push(None, None, (0., 0., 1., 1.))
        return last
    out = _with_stub([True, True, True], go)
    check('中位数', abs(out['corners_uv'][0][0] - 12.0) < 1e-9, str(out['corners_uv'][0]))


def test_window_partial_window_still_returns() -> None:
    """窗口还没满也要出结果（有几帧算几帧），不额外等。"""
    def go(W):
        win = W.BoxFrameWindow(_k(), process_every=1)
        return win.push(None, None, (0., 0., 1., 1.)), win
    out, win = _with_stub([True], go)
    check('不满窗也出结果', out is not None and out['n_slots'] == 1, str(out))
    check('ready 为 False', win.ready is False)
    check('n_slots', win.n_slots == 1, str(win.n_slots))


def test_window_survives_exception() -> None:
    """单帧抛异常不能拖垮整条链路 —— 记成失败帧继续走。"""
    def go(W):
        from .. import window as M
        old = M.fit_box_frame
        def boom(*a, **kw):
            raise RuntimeError('炸了')
        M.fit_box_frame = boom
        try:
            win = M.BoxFrameWindow(_k(), process_every=1)
            return win.push(None, None, (100., 50., 300., 250.))
        finally:
            M.fit_box_frame = old
    out = go(None)
    check('异常记成失败帧', out['source'] == 'yolo_fallback', str(out['source']))
    check('原因里带 exception', 'exception:RuntimeError' in out['rejects'], str(out['rejects']))


def test_render_window_marks_fallback() -> None:
    """兜底结果必须在图上看得见（项目规矩：低置信度要可见）。"""
    from ..window import render_window_check, yolo_fallback_corners
    img = np.full((120, 160, 3), 40, np.uint8)
    box = (40.0, 20.0, 120.0, 100.0)
    out = {'corners_uv': yolo_fallback_corners(box), 'source': 'yolo_fallback',
           'n_used': 0, 'n_slots': 5, 'n_failed': 5, 'spread_px': None,
           'rejects': ['no_plane'] * 5}
    vis = render_window_check(img, out, box)
    check('尺寸不变', vis.shape == img.shape)
    red = ((vis[:, :, 2] == 255) & (vis[:, :, 1] == 0) & (vis[:, :, 0] == 0))
    check('有红色（兜底）', int(red.sum()) > 20, str(int(red.sum())))
    out2 = dict(out, source='window', n_used=5, n_failed=0, spread_px=1.2)
    vis2 = render_window_check(img, out2, box)
    green = ((vis2[:, :, 1] == 255) & (vis2[:, :, 2] == 0))
    check('正常时是绿色', int(green.sum()) > 20, str(int(green.sum())))




def test_build_payload() -> None:
    """窗口输出 -> 对外 payload。ROS 那层只搬运，业务逻辑全在这里测。"""
    from ..window import build_payload, yolo_fallback_corners
    box = (100.0, 50.0, 300.0, 250.0)

    # 正常：右下(300,250) 左下(100,250) -> 下边水平 -> 0°
    out = {'corners_uv': np.array([[300.0, 250.0], [100.0, 250.0],
                                   [100.0, 50.0], [300.0, 50.0]]),
           'source': 'window', 'n_used': 5, 'n_slots': 5, 'n_failed': 0,
           'spread_px': 3.5, 'rejects': []}
    p = build_payload(out, box, 61.2)
    check('四角顺序原样透传', p['corners_uv'][0] == (300.0, 250.0), str(p['corners_uv']))
    check('水平时角度 0', abs(p['angle_deg']) < 1e-9, str(p['angle_deg']))
    check('valid', p['valid'] is True)
    check('source', p['source'] == 'window')
    check('spread 透传', abs(p['spread_px'] - 3.5) < 1e-9, str(p['spread_px']))
    check('latency 透传', abs(p['latency_ms'] - 61.2) < 1e-9, str(p['latency_ms']))
    check('box_uv 是 左上,右下', p['box_uv'] == [(100.0, 50.0), (300.0, 250.0)],
          str(p['box_uv']))

    # 角度：右边比左边**低** 10px -> 正角（图像 +y 向下）
    out2 = dict(out, corners_uv=np.array([[300.0, 260.0], [100.0, 250.0],
                                          [100.0, 50.0], [300.0, 60.0]]))
    a = build_payload(out2, box)['angle_deg']
    check('右边低 -> 正角', a > 0 and abs(a - np.degrees(np.arctan2(10.0, 200.0))) < 1e-6,
          str(a))

    # 兜底：valid=False，角度仍然照给（就是输入框的角度），spread=-1
    out3 = {'corners_uv': yolo_fallback_corners(box), 'source': 'yolo_fallback',
            'n_used': 0, 'n_slots': 5, 'n_failed': 5, 'spread_px': None,
            'rejects': ['no_plane', 'no_plane']}
    p3 = build_payload(out3, box)
    check('兜底 valid=False', p3['valid'] is False, str(p3['valid']))
    check('兜底 spread=-1', p3['spread_px'] == -1.0, str(p3['spread_px']))
    check('兜底角度 = YOLO 框的角度', abs(p3['angle_deg']) < 1e-9, str(p3['angle_deg']))
    check('rejects 逗号拼接', p3['rejects'] == 'no_plane,no_plane', str(p3['rejects']))
    check('latency 缺省为 0', p3['latency_ms'] == 0.0, str(p3['latency_ms']))

    # 反向给的框也要归一化
    p4 = build_payload(out, (300.0, 250.0, 100.0, 50.0))
    check('反向框归一化', p4['box_uv'] == [(100.0, 50.0), (300.0, 250.0)],
          str(p4['box_uv']))


def test_orientation_hint_on_real_ambiguous_frame() -> None:
    """**实测**：先验「数据说不对就拒绝」这条契约在真数据上成立。

    ⚠️ 这个测试在本轮（2026-09-22）**换了测的东西**，原来的测法已经失效：
    它靠「两种朝向假设都建得出候选」来造歧义，而候选枚举现在只剩一支
    （见 `initial_frame_candidates`：e1 按构造就是点集短边那一对，
    `long_along_e1=True` 那一支从来没错过、已删），所以这种歧义**不可能再有**。

    改成测契约：拿 6_test 真帧（长边在图像里是**横**的）分别给三种提示 ——
      * `auto`        -> 出结果，`applied=False`（没污染）
      * `horizontal`  -> 出结果，`applied=True`，最终 frame 的朝向也是 horizontal
      * `vertical`    -> **直接 reject `orientation_conflict`**（宁可走 YOLO 框
                          fallback，也不输出一个先验说不通的朝向）
    """
    from ..algorithm import fit_box_frame
    seq = Path('/data/Real_Downloads/maduo/test_data/6_test')
    if not seq.exists():
        print('  SKIP  test_orientation_hint_on_real_ambiguous_frame（6_test 不存在）')
        return
    from ..algorithm import load_frame
    stem = '1789543228.215563'
    color, depth, k = load_frame(seq, stem)
    # 6_test 帧1 的 YOLO 框（见 box_out/6_test/boxes.json）
    box = (186.9, 250.2, 461.4, 432.0)

    got = {}
    for hint in ('auto', 'horizontal', 'vertical'):
        res, diag = fit_box_frame(color, depth, k, box, opts={'orientation': hint})
        got[hint] = (res, diag)

    res_a, diag_a = got['auto']
    check('auto 能出结果', res_a is not None, str(diag_a.get('reject')))
    check('auto 时 applied=False',
          diag_a['orientation_hint']['applied'] is False,
          str(diag_a.get('orientation_hint')))

    res_h, diag_h = got['horizontal']
    check('horizontal 提示能出结果', res_h is not None, str(diag_h.get('reject')))
    if res_h is not None:
        check('horizontal 提示 applied=True',
              diag_h['orientation_hint']['applied'] is True,
              str(diag_h.get('orientation_hint')))
        # 输出 frame 的长边，在图像上必须跟着提示走（最终复核那一关）
        from ..algorithm import frame_long_axis
        ax = frame_long_axis(res_h['frame'], k)
        check('horizontal 提示下最终朝向正确', ax == 'horizontal', str(ax))
        check('输出朝向复核写进了诊断',
              diag_h['orientation_hint'].get('final') == 'horizontal',
              str(diag_h['orientation_hint'].get('final')))
        check('用上提示后四角与自动选一致',
              np.allclose(res_a['corners_uv'], res_h['corners_uv'], atol=1e-6),
              str(res_h['corners_uv'] - res_a['corners_uv']))

    res_v, diag_v = got['vertical']
    check('反向提示直接判失败', res_v is None, str(res_v is not None))
    check('反向提示的 reject 写明是朝向冲突',
          diag_v.get('reject') == 'orientation_conflict', str(diag_v.get('reject')))
    check('反向提示标 conflict', diag_v['orientation_hint']['conflict'] is True,
          str(diag_v.get('orientation_hint')))


def test_window_orientation_in_payload() -> None:
    """窗口层要能把「用上了 / 冲突 / 没给」三种情形如实带到 payload 上。

    这三种在诊断上**必须区分**：`''` 只是「没接服务」，`'conflict'` 是
    「上游的朝向和数据打架」—— 后者更严重，消费端要告警。
    """
    from .. import window as W
    from ..window import build_payload, yolo_fallback_corners
    box = (0.0, 0.0, 10.0, 10.0)
    c = np.array([[10.0, 10.0], [0.0, 10.0], [0.0, 0.0], [10.0, 0.0]])

    def out(hint, conflict):
        per = [{'ok': True, 'reject': None, 'confidence': 'normal',
                'orientation': hint, 'orient_conflict': conflict}]
        return {'corners_uv': c, 'source': 'single', 'n_used': 1, 'n_slots': 1,
                'n_failed': 0, 'spread_px': 0.0, 'orient_conflict': conflict,
                'rejects': [], 'per_frame': per}

    check('用上了 -> 透传提示',
          build_payload(out('horizontal', False), box)['orientation'] == 'horizontal')
    check('没给 -> 空串',
          build_payload(out(None, False), box)['orientation'] == '')
    check('冲突 -> conflict（不是空串）',
          build_payload(out(None, True), box)['orientation'] == 'conflict')

    # 兜底那条路径也要带上冲突标记
    fb = {'corners_uv': yolo_fallback_corners(box), 'source': 'yolo_fallback',
          'n_used': 0, 'n_slots': 1, 'n_failed': 1, 'spread_px': None,
          'orient_conflict': True, 'rejects': ['no_plane'],
          'per_frame': [{'ok': False, 'reject': 'no_plane', 'confidence': None,
                         'orientation': None, 'orient_conflict': True}]}
    check('兜底时也带冲突标记',
          build_payload(fb, box)['orientation'] == 'conflict')

    # 真跑一次窗口（用替身），确认 `orient_conflict` 被聚合出来
    def go(WW):
        old = WW.fit_box_frame

        def spy(color, depth, k, box_uv, **kw):
            return {'corners_uv': c}, {'ok': True, 'confidence': 'normal',
                                       'orientation_hint': {'hint': 'vertical',
                                                            'applied': False,
                                                            'conflict': True}}
        WW.fit_box_frame = spy
        try:
            w = WW.BoxFrameWindow(_k(), process_every=1, orientation='vertical')
            return w.push(None, None, box)
        finally:
            WW.fit_box_frame = old
    o = go(W)
    check('窗口聚合出 conflict', o['orient_conflict'] is True, str(o.get('orient_conflict')))
    check('conflict 时 payload 里也是 conflict',
          build_payload(o, box)['orientation'] == 'conflict',
          str(build_payload(o, box)['orientation']))


def test_payload_matches_message_fields() -> None:
    """payload 的字段集合必须和 .msg 对得上 —— 防止改了 msg 忘了改这里。"""
    from ..window import build_payload, yolo_fallback_corners
    box = (100.0, 50.0, 300.0, 250.0)
    p = build_payload({'corners_uv': yolo_fallback_corners(box),
                       'source': 'single', 'n_used': 1, 'n_slots': 1,
                       'n_failed': 0, 'spread_px': 0.0, 'rejects': []}, box)
    # 消息包也在本仓库里（infrastructure/ros_packages/src/ros_vision/），
    # 从测试文件往上反查仓库根再定位 —— 别写死绝对路径。
    _root = Path(__file__).resolve().parents[5]
    msg = (_root / 'infrastructure' / 'ros_packages' / 'src' / 'ros_vision'
           / 'box_detection_msgs' / 'msg' / 'BoxDetection.msg')
    if not msg.exists():
        print('  SKIP  .msg 不存在')
        return
    fields = set()
    for line in msg.read_text(encoding='utf-8').splitlines():
        line = line.split('#')[0].strip()
        if not line:
            continue
        typ, name = line.split()[-2], line.split()[-1]
        if typ == 'std_msgs/Header':          # header 由节点自己填，不在 payload 里
            continue
        fields.add(name)
    check('payload 字段集 == .msg 字段集（除 header 外）', set(p) == fields,
          f'payload 多 {set(p) - fields}，msg 多 {fields - set(p)}')


# --------------------------------------------------------------------------- #
# 朝向先验：掐掉「整框转 90°」（操作员 2026-09-21 要求）
#
# ⚠️ 注意这里**没法**用合成箱子测「提示选对了朝向」：合成箱子在图像上是
# 244x161，长边必然沿图像 u，两种 `long_along_e1` 假设里只有一种装得下点集，
# 另一条在建候选阶段就被毙了 —— 也就是说**自动选本来就不会错**，提示无从体现。
# 真正需要提示的是「可见区近似各向同性、两种假设都能装下」的场合，那要靠
# 实测数据（9_test）。这里测的是**接口契约**：归一化、筛选、冲突回退、留痕。
# --------------------------------------------------------------------------- #
def test_normalize_orientation() -> None:
    from ..algorithm import normalize_orientation
    check('horizontal 字面量', normalize_orientation('horizontal') == 'horizontal')
    check('大小写无关', normalize_orientation('HORIZONTAL') == 'horizontal')
    check('vertical 字面量', normalize_orientation('vertical') == 'vertical')
    check('中文「横」', normalize_orientation('横') == 'horizontal')
    check('中文「竖着」', normalize_orientation('竖着') == 'vertical')
    check('tape 的 0 -> 横', normalize_orientation(0) == 'horizontal')
    check('tape 的 90 -> 竖', normalize_orientation(90) == 'vertical')
    check('tape 的 -1 -> 不提示', normalize_orientation(-1) is None)
    check('None -> 不提示', normalize_orientation(None) is None)
    check("'auto' -> 不提示", normalize_orientation('auto') is None)
    check("空串 -> 不提示", normalize_orientation('') is None)
    # 认不出的必须**报错**，不能默默当没有 —— 否则「以为用了提示」和「其实没用」
    # 在结果上分不出来
    try:
        normalize_orientation('斜着')
        check('认不出的提示要报错', False, '没报错')
    except ValueError:
        check('认不出的提示要报错', True)


def test_tape_orientation_along_long_edge() -> None:
    """胶带沿长边贴时，`tape_orientation_deg` 与长边朝向同向（见常量注释）。"""
    from .. import algorithm as F
    old = F.TAPE_ALONG_LONG_EDGE
    try:
        F.TAPE_ALONG_LONG_EDGE = True
        check('0 -> horizontal', F.tape_orientation_to_hint(0) == 'horizontal')
        check('90 -> vertical', F.tape_orientation_to_hint(90) == 'vertical')
        check('-1 -> None', F.tape_orientation_to_hint(-1) is None)
        check('其它值 -> None', F.tape_orientation_to_hint(45) is None)
        # 实物若是横着贴，只改这一个常量即可（改完差 90°）
        F.TAPE_ALONG_LONG_EDGE = False
        check('横着贴时 0 -> vertical',
              F.tape_orientation_to_hint(0) == 'vertical')
    finally:
        F.TAPE_ALONG_LONG_EDGE = old


def test_filter_by_orientation_picks_matching() -> None:
    """筛选只留下长边方向对得上的候选，并把「用没用上」如实记进诊断。"""
    from ..algorithm import filter_by_orientation, plane_basis
    from ..algorithm import _project_px
    from ..algorithm import CameraIntrinsics
    from ..algorithm import _corners_mm
    k = CameraIntrinsics(368.523, 368.458, 320.364, 245.712)
    plane = {'nrm': np.array([0.0, 0.0, -1.0]),
             'origin': np.array([0.0, 0.0, 800.0])}
    E1, E2 = plane_basis(plane['nrm'])
    # 两个候选，同一个角、同一个 2D 基，只有「530 配哪条轴」不同 —— 正是
    # `initial_frame_candidates` 枚举的那两种假设。
    base = {'origin2d': np.array([-200.0, -150.0]),
            'e1': np.array([1.0, 0.0]), 'e2': np.array([0.0, 1.0]),
            'corner': 'u+/v+', 'extrap_mm': 0.0}
    cands = [dict(base, W=350.0, H=530.0, long_along_e1=False),
             dict(base, W=530.0, H=350.0, long_along_e1=True)]

    def long_axis(c):
        fr = {'origin': plane['origin'] + c['origin2d'][0] * E1 + c['origin2d'][1] * E2,
              'E1': E1, 'E2': E2}
        W, H = c['W'], c['H']
        c4 = _corners_mm(fr, W, H)
        i0, i1 = (0, 1) if W >= H else (0, 3)
        px = _project_px(c4[[i0, i1]], k)
        du, dv = abs(float(px[1, 0] - px[0, 0])), abs(float(px[1, 1] - px[0, 1]))
        return 'horizontal' if du >= dv else 'vertical'

    # 平面基 E1 = (0,-1,0)、E2 = (-1,0,0)，所以 2D 的 e1 对应图像**竖直**、
    # e2 对应图像**水平**。两种假设的图像朝向恰好相反，正好用来测筛选。
    ax = [long_axis(c) for c in cands]
    check('两种假设的图像朝向相反', ax[0] != ax[1], str(ax))

    for want in ('horizontal', 'vertical'):
        kept, d = filter_by_orientation(cands, plane, k, want)
        check(f'hint={want} 时只留 1 个', len(kept) == 1, str(len(kept)))
        check(f'hint={want} 时留下的是对的那个',
              long_axis(kept[0]) == want, str(long_axis(kept[0])))
        check(f'hint={want} 时 applied=True', d['applied'] is True, str(d))
        check(f'hint={want} 时无冲突', d['conflict'] is False, str(d))

    kept, d = filter_by_orientation(cands, plane, k, None)
    check('hint=None 时原样放行', len(kept) == 2 and d['applied'] is False, str(d))

    # 提示与数据打架：**直接判失败**（操作员 2026-09-22：「宁可没有结果走
    # YOLO 框 fallback，也不能接受错误的结果」）—— 不回退、不筛空后再硬选。
    bad = 'vertical' if ax[0] == 'horizontal' else 'horizontal'
    kept, d = filter_by_orientation([cands[0]], plane, k, bad)
    check('冲突时不留候选', len(kept) == 0, str(len(kept)))
    check('冲突时 applied=False', d['applied'] is False, str(d))
    check('冲突时 conflict=True', d['conflict'] is True, str(d))
    check('冲突时给出 reject 原因', d.get('reject') == 'orientation_conflict', str(d))


def test_orientation_hint_end_to_end_synthetic() -> None:
    """整条链路跑通：提示能用上、写进 diag、不改变本来正确的自动选。"""
    from ..algorithm import fit_box_frame
    depth, k = _synth_depth_with_box()
    color = np.full((480, 640, 3), 90, np.uint8)
    box_uv = (200.0, 170.0, 444.0, 331.0)
    res_a, diag_a = fit_box_frame(color, depth, k, box_uv, opts={'orientation': 'auto'})
    check('auto 能出结果', res_a is not None, str(diag_a.get('reject')))
    check('auto 时 applied=False',
          diag_a['orientation_hint']['applied'] is False,
          str(diag_a.get('orientation_hint')))
    # 合成箱子长边在图像上是**水平**的（244px 宽 vs 161px 高）
    res_h, diag_h = fit_box_frame(color, depth, k, box_uv,
                                  opts={'orientation': 'horizontal'})
    check('horizontal 提示能出结果', res_h is not None, str(diag_h.get('reject')))
    check('horizontal 提示被用上', diag_h['orientation_hint']['applied'] is True,
          str(diag_h.get('orientation_hint')))
    check('用上提示后四角与自动选一致',
          np.allclose(res_a['corners_uv'], res_h['corners_uv'], atol=1e-6),
          str(res_h['corners_uv'] - res_a['corners_uv']))
    # 反向提示：一个候选都对不上 -> **直接判失败**，让上层走 YOLO 框 fallback。
    # 操作员 2026-09-22：「宁可没有结果走 yolo 框回归这条 fallback，也不能接受
    # 错误的结果」—— 原来是「不硬筛空、标 conflict、照样出结果」，那等于明知
    # 先验和数据打架还拿一个先验说不通的朝向去输出。
    res_v, diag_v = fit_box_frame(color, depth, k, box_uv,
                                  opts={'orientation': 'vertical'})
    check('反向提示直接判失败', res_v is None, str(res_v is not None))
    check('反向提示的 reject 写明是朝向冲突',
          diag_v.get('reject') == 'orientation_conflict', str(diag_v.get('reject')))
    check('反向提示标 conflict', diag_v['orientation_hint']['conflict'] is True,
          str(diag_v.get('orientation_hint')))
    # 冲突虽然这一帧没有结果，但**必须让上层看得见** —— 窗口层从 diag 里读
    # `orientation_hint`，据此在 payload 上写 `orientation='conflict'`，
    # 消费端才能区分「上游没给朝向」和「给了但和数据打架」。
    from ..window import _hint_of
    hint_used, conflict = _hint_of(diag_v)
    check('冲突能被窗口层读到', conflict is True, str((hint_used, conflict)))
    check('冲突时没有「用上的提示」', hint_used is None, str(hint_used))


def test_hint_from_instances() -> None:
    """`/infer_carton_pose` 回包 -> 朝向：必须挑**和这只箱子 IoU 最大**的实例。

    场上有两个箱子时拿错实例的朝向，比不给提示更坏 —— 它会把正确的朝向筛掉。
    """
    from ..window import hint_from_instances
    box = (186.9, 250.2, 461.4, 432.0)             # 6_test 帧1 的 YOLO 框
    far = [100.0, 100.0, 200.0, 200.0]             # 远处另一个箱子
    near = [180.0, 245.0, 465.0, 435.0]            # 就是它
    check('挑 IoU 最大的那条',
          hint_from_instances(far + near, [0, 90], box) == 'vertical',
          str(hint_from_instances(far + near, [0, 90], box)))
    check('顺序反过来也对',
          hint_from_instances(near + far, [90, 0], box) == 'vertical')
    check('全不重叠 -> None（宁可不提示）',
          hint_from_instances(far, [0], box) is None)
    check('-1 -> None', hint_from_instances(near, [-1], box) is None)
    check('空回包 -> None', hint_from_instances([], [], box) is None)
    check('数组对不齐 -> None', hint_from_instances(near, [0, 90], box) is None)
    check('index 直取', hint_from_instances(near, [90], box, index=0) == 'vertical')
    check('index 越界 -> None', hint_from_instances(near, [90], box, index=5) is None)


def test_window_set_orientation() -> None:
    """时间窗能运行时改朝向先验（上游服务回一次就改，不用重建窗口）。"""
    from .. import window as W
    win = W.BoxFrameWindow(_k(), process_every=1)
    check('默认没提示', win.orientation is None, str(win.orientation))
    check('设成 horizontal', win.set_orientation('horizontal') == 'horizontal')
    check('读回来是 horizontal', win.orientation == 'horizontal')
    check('设成 tape 的 0', win.set_orientation(0) == 'horizontal')
    check('设成 tape 的 90', win.set_orientation(90) == 'vertical')
    check('tape 的 -1 清掉提示', win.set_orientation(-1) is None)
    check('清掉后读回 None', win.orientation is None, str(win.orientation))
    check('构造时也能直接给',
          W.BoxFrameWindow(_k(), orientation='vertical').orientation == 'vertical')
    # 传进 fit_box_frame 的 opts 里必须真的带上
    seen = {}

    def go(WW):
        old = WW.fit_box_frame
        def spy(color, depth, k, box_uv, **kw):
            seen.update(kw.get('opts') or {})
            return None, {'ok': False, 'reject': 'no_plane'}
        WW.fit_box_frame = spy
        try:
            w = WW.BoxFrameWindow(_k(), process_every=1, orientation='horizontal')
            w.push(None, None, (0., 0., 1., 1.))
        finally:
            WW.fit_box_frame = old
    go(W)
    check('opts 里带上了 orientation', seen.get('orientation') == 'horizontal',
          str(seen))


ALL_TESTS = [
    test_geometry_self_consistency, test_roi_from_box, test_valid_depth_points,
    test_load_frame,
    test_depth_discontinuity_finds_step,
    test_depth_discontinuity_ignores_invalid,
    test_depth_discontinuity_smooth_plane_is_quiet,
    test_split_planes_two_planes, test_plane_projection_roundtrip,
    test_segment_top_plane_picks_box_face,
    test_extract_segments_from_rectangle, test_candidates_cover_points,
    test_candidate_to_frame_is_planar, test_collect_observations_on_synthetic_box,
    test_choose_candidate_picks_upright_orientation,
    test_solve_updates_held_partition, test_solve_degraded_when_only_one_family,
    test_order_corners_uv, test_fit_box_frame_end_to_end_synthetic,
    test_render_box_check_marks_pixels, test_render_handles_failure,
    test_occlusion_still_covers_points, test_full_chain_under_occlusion,
    test_yolo_fallback_corners, test_window_process_every,
    test_window_only_one_good_frame, test_window_all_fail_falls_back_to_yolo,
    test_window_mean_of_good_frames, test_window_median_option,
    test_window_partial_window_still_returns, test_window_survives_exception,
    test_render_window_marks_fallback,
    test_build_payload,
    test_normalize_orientation, test_tape_orientation_along_long_edge,
    test_filter_by_orientation_picks_matching,
    test_orientation_hint_end_to_end_synthetic,
    test_hint_from_instances, test_window_set_orientation,
    test_orientation_hint_on_real_ambiguous_frame,
    test_window_orientation_in_payload,
    test_payload_matches_message_fields,
]

if __name__ == '__main__':
    for fn in ALL_TESTS:
        print(fn.__name__)
        fn()
    print()
    if _fails:
        print(f'{len(_fails)} 项失败：{", ".join(_fails)}')
        sys.exit(1)
    print('全部通过')
