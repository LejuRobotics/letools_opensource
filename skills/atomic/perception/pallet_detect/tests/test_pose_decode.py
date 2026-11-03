# -*- coding: utf-8 -*-
"""`pose_decode` 纯函数自检（不需要模型 / ROS / 相机）。

跑法（退出码 0 通过）：

    cd <仓库根>
    OMP_NUM_THREADS=1 python3 skills/atomic/perception/pallet_detect/tests/test_pose_decode.py

覆盖：解码坐标还原（含出画点不 clamp）、conf 过滤、AABB NMS、2/3/4 点
加权 TLS、离群点丢弃、轮廓翻转后边分组不变、求交退化。
"""
from __future__ import annotations

import sys
from pathlib import Path

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
    __package__ = 'skills.atomic.perception.pallet_detect.tests'
    import importlib
    importlib.import_module(__package__)

import numpy as np

from ..pose_decode import (EDGES, EdgeLine, PoseDetection, decode_pose,
                           fit_edges, intersect_edges, nms_pose, pick_pose)

# --------------------------------------------------------------------------- #
# 造数
# --------------------------------------------------------------------------- #
RECT = np.array([[100.0, 400.0],   # 0 corner_bl
                 [400.0, 400.0],   # 3 corner_br
                 [400.0, 100.0],   # 6 corner_tr
                 [100.0, 100.0]])  # 9 corner_tl


def _rect_keypoints() -> np.ndarray:
    """矩形四边各 4 点（相邻边共用角点），顺序与 EDGES 一致。"""
    bl, br, tr, tl = RECT
    return np.array([
        bl, bl + (br - bl) / 3.0, bl + 2.0 * (br - bl) / 3.0, br,     # 0..3
        br + (tr - br) / 3.0, br + 2.0 * (tr - br) / 3.0, tr,         # 4..6
        tr + (tl - tr) / 3.0, tr + 2.0 * (tl - tr) / 3.0, tl,         # 7..9
        tl + (bl - tl) / 3.0, tl + 2.0 * (bl - tl) / 3.0,             # 10..11
    ])


def _det(conf: float, box) -> PoseDetection:
    return PoseDetection(bbox_xyxy=np.asarray(box, np.float64), conf=conf,
                         kp_xy=np.zeros((12, 2)), kp_conf=np.ones(12))


# --------------------------------------------------------------------------- #
def test_decode_restore() -> None:
    """u=(u_in-pad)/r、v=(v_in-pad)/r；出画点不 clamp；conf 过滤。"""
    ratio, left, top = 0.5, 80.0, 60.0

    # 原图 bbox [100, 200, 300, 400] → 输入 cx/cy/w/h = 180 / 210 / 100 / 100
    row_ok = np.zeros(41)
    row_ok[0:5] = [180.0, 210.0, 100.0, 100.0, 0.9]
    # 关键点：第 0 个在原图 (200, 300)；第 1 个故意出画 (-5, 700)
    kps = np.zeros((12, 3))
    kps[:, 2] = 0.8
    kps[0] = [200.0 * ratio + left, 300.0 * ratio + top, 0.8]
    kps[1] = [-5.0 * ratio + left, 700.0 * ratio + top, 0.8]
    row_ok[5:] = kps.reshape(-1)

    row_low = row_ok.copy()
    row_low[4] = 0.1

    dets = decode_pose(np.stack([row_ok, row_low]), ratio, (left, top),
                       (480, 640), conf_thr=0.5)
    assert len(dets) == 1, f'conf 过滤失败: {len(dets)}'
    d = dets[0]
    assert d.conf == 0.9
    assert np.allclose(d.bbox_xyxy, [100.0, 200.0, 300.0, 400.0]), d.bbox_xyxy
    assert np.allclose(d.kp_xy[0], [200.0, 300.0]), d.kp_xy[0]
    # 出画点保持原值（不 clamp）
    assert np.allclose(d.kp_xy[1], [-5.0, 700.0]), d.kp_xy[1]

    # kpt_conf_thr 把低置信点置 0，但不改坐标
    kps[2] = [300.0 * ratio + left, 300.0 * ratio + top, 0.2]
    row_ok[5:] = kps.reshape(-1)
    d2 = decode_pose(row_ok[None], ratio, (left, top), (480, 640),
                     conf_thr=0.5, kpt_conf_thr=0.5)[0]
    assert d2.kp_conf[2] == 0.0 and np.allclose(d2.kp_xy[2], [300.0, 300.0])


def test_nms_aabb() -> None:
    a = _det(0.9, [0, 0, 100, 100])
    b = _det(0.8, [10, 10, 110, 110])   # IoU ≈ 0.68
    c = _det(0.7, [200, 200, 300, 300])
    kept = nms_pose([a, b, c], 0.45, (480, 640))
    assert kept == [a, c], [k.conf for k in kept]
    assert nms_pose([], 0.45, (480, 640)) == []
    # 顺序按 conf 降序，与输入顺序无关
    kept2 = nms_pose([c, b, a], 0.45, (480, 640))
    assert [k.conf for k in kept2] == [0.9, 0.7]


def test_pick_pose() -> None:
    img_size = (480, 640)
    # 画面下方中心是 (320, 480)
    near = _det(0.5, [270, 380, 370, 480])
    far = _det(0.9, [0, 0, 50, 50])
    assert pick_pose([far, near], img_size) is near
    assert pick_pose([far, near], img_size, how='conf') is far
    assert pick_pose([far, near], img_size, how='area') is near
    assert pick_pose([], img_size) is None


def test_fit_edges_counts() -> None:
    """4 点、3 点、2 点三条路径，以及默认全 4 点。"""
    kp = _rect_keypoints()

    # 4 点全用：四条边都拟合，idx 与 EDGES 一致
    lines = fit_edges(kp, np.ones(12))
    assert all(ln is not None for ln in lines)
    for ln, idx in zip(lines, EDGES):
        assert ln.idx == idx, (ln.idx, idx)
        assert ln.n_points == 4
        assert abs(np.linalg.norm(ln.d) - 1.0) < 1e-12
        assert ln.residual_px < 1e-9

    # 底边只留 0、3 → 2 点；其余边点不足 → None
    conf = np.zeros(12)
    conf[[0, 3]] = 1.0
    lines = fit_edges(kp, conf)
    assert lines[0] is not None and lines[0].n_points == 2
    assert lines[1] is None and lines[2] is None and lines[3] is None
    assert np.allclose(lines[0].p0, [250.0, 400.0])


def test_fit_edges_outlier() -> None:
    """最大垂距超限时丢掉最差点重拟合。"""
    kp = _rect_keypoints()
    kp[2, 1] += 50.0            # 底边中间点向外拉 50px
    conf = np.zeros(12)
    conf[[0, 1, 2, 3]] = 1.0
    lines = fit_edges(kp, conf, line_res_px=12.0)
    ln = lines[0]
    assert ln is not None
    assert ln.n_points == 3, ln.n_points
    assert ln.idx == (0, 1, 3), ln.idx
    assert ln.residual_px < 1e-9, ln.residual_px

    # 阈值放宽到 100 时不丢点
    ln2 = fit_edges(kp, conf, line_res_px=100.0)[0]
    assert ln2.n_points == 4 and ln2.idx == (0, 1, 2, 3)


def test_flip_does_not_change_grouping() -> None:
    """轮廓整体镜像后，边仍按下标分组，四条边与求交结果都有效。"""
    kp = _rect_keypoints()
    h, w = 480.0, 640.0
    flipped = kp.copy()
    flipped[:, 0] = w - flipped[:, 0]     # 水平镜像
    flipped[:, 1] = h - flipped[:, 1]     # 垂直镜像（相当于旋转 180°）

    lines = fit_edges(flipped, np.ones(12))
    assert all(ln is not None for ln in lines)
    assert [ln.idx for ln in lines] == list(EDGES)

    quad = intersect_edges(lines, (h, w))
    assert quad is not None and quad.shape == (4, 2)
    expect = np.array([[w - 100.0, h - 400.0],   # bl 翻转后
                       [w - 400.0, h - 400.0],
                       [w - 400.0, h - 100.0],
                       [w - 100.0, h - 100.0]])
    assert np.allclose(quad, expect), quad


def test_intersect_degenerate() -> None:
    good = fit_edges(_rect_keypoints(), np.ones(12))
    quad = intersect_edges(good, (480, 640))
    assert quad is not None
    assert np.allclose(quad, RECT[[0, 1, 2, 3]]), quad

    # 任一条边缺失 → None
    missing = list(good)
    missing[2] = None
    assert intersect_edges(missing, (480, 640)) is None

    # 两线近平行 → None
    par = [EdgeLine(np.array([0.0, 0.0]), np.array([1.0, 0.0]),
                    2, 0.0, 1.0, (0, 1)),
           EdgeLine(np.array([0.0, 10.0]), np.array([1.0, 1e-9]),
                    2, 0.0, 1.0, (1, 2)),
           EdgeLine(np.array([0.0, 20.0]), np.array([1.0, 0.0]),
                    2, 0.0, 1.0, (2, 3)),
           EdgeLine(np.array([0.0, 30.0]), np.array([1.0, 0.0]),
                    2, 0.0, 1.0, (3, 0))]
    assert intersect_edges(par, (480, 640)) is None

    # 交点离画面中心超过 3 倍对角线 → None
    far = [
        EdgeLine(np.array([100.0, 400.0]), np.array([1.0, 0.0]),
                 2, 0.0, 1.0, (0, 1)),                       # edge0: y=400
        EdgeLine(np.array([100000.0, 0.0]), np.array([0.0, 1.0]),
                 2, 0.0, 1.0, (1, 2)),                       # edge1: x=100000
        EdgeLine(np.array([0.0, 0.0]), np.array([1.0, 0.0]),
                 2, 0.0, 1.0, (2, 3)),                       # edge2: y=0
        EdgeLine(np.array([100.0, 100.0]), np.array([0.0, 1.0]),
                 2, 0.0, 1.0, (3, 0)),                       # edge3: x=100
    ]
    assert intersect_edges(far, (480, 640)) is None
    # 不给 size 时只做平行/缺失检查
    assert intersect_edges(far) is not None


def main() -> None:
    tests = [test_decode_restore, test_nms_aabb, test_pick_pose,
             test_fit_edges_counts, test_fit_edges_outlier,
             test_flip_does_not_change_grouping, test_intersect_degenerate]
    for fn in tests:
        fn()
        print(f'  PASS  {fn.__name__}')
    print(f'\n{len(tests)} 项全部通过')


if __name__ == '__main__':
    main()
