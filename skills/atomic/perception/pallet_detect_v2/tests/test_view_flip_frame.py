# -*- coding: utf-8 -*-
"""从托盘两侧看：解出的 frame 必须始终 **X 朝右、Y 朝远、法向朝上**。

机器人放 1–3 箱时在托盘近端、4/5 箱时移到远端，画面里的托盘整体转了 180°。
这里用合成相机把同一块托盘从两侧各解一次，钉住两件事：

1. `_to_spec` 的出口约定与视角无关 —— `E1` 投影朝右、`E2` 投影朝上；
2. `nrm = E1 × E2` 始终朝相机（相机在托盘上方 = 朝上），**不会翻成朝下**。
"""
import numpy as np
import pytest

from ..frame_geometry import order_quad
from ..geom import CameraIntrinsics, _corners_mm, _project_px
from ..solve_edges import solve_frame

pytestmark = pytest.mark.unit

K = CameraIntrinsics(600.0, 600.0, 320.0, 240.0)
W, H = 1000.0, 1200.0
WORLD_UP = np.array([0.0, 0.0, 1.0])


def _look_at(camera, target):
    """标准 OpenCV 朝向：+x 右、+y 下、+z 前。相机在目标上方斜看。"""
    C = np.asarray(camera, float)
    target = np.asarray(target, float)
    z = target - C
    z /= np.linalg.norm(z)
    x = np.cross(z, WORLD_UP)          # 世界 up 投影到像面 -> 相机右
    if np.linalg.norm(x) < 1e-9:       # 正俯视：退化，换一个参考
        x = np.cross(z, np.array([0.0, 1.0, 0.0]))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    return R, -R @ C


def _project(R, t, points_mm):
    P = np.asarray(points_mm, float) * 0.001
    pc = P @ R.T + t
    uv = pc[:, :2] / pc[:, 2:3]
    return np.column_stack([K.fx * uv[:, 0] + K.cx,
                            K.fy * uv[:, 1] + K.cy])


def _projected_dir(origin_cam, unit_cam):
    def uv(P):
        q = np.asarray(P, float)
        q = q[:2] / q[2]
        return np.array([K.fx * q[0] + K.cx, K.fy * q[1] + K.cy])
    d = uv(origin_cam + 300.0 * unit_cam) - uv(origin_cam)
    return d / np.linalg.norm(d)


@pytest.mark.parametrize("side, camera", [
    ("近端", np.array([0.5, -0.8, 1.2])),
    ("远端", np.array([0.5, 1.8, 1.2])),
])
def test_opposite_view_keeps_x_right_y_far_and_normal_up(side, camera):
    frame_w = dict(origin=np.zeros(3), E1=np.array([1.0, 0.0, 0.0]),
                   E2=np.array([0.0, 1.0, 0.0]), nrm=WORLD_UP, W=W, H=H)
    corners = _corners_mm(frame_w, W, H)
    center = np.array([W, H, 0.0]) * 0.001 * 0.5

    R, t = _look_at(camera, center)
    px = _project(R, t, corners)
    q = order_quad(px)
    lines = []
    for i in range(4):
        p0, p1 = q[i], q[(i + 1) % 4]
        d = p1 - p0
        d /= np.linalg.norm(d)
        lines.append({'p0': p0, 'd': d, 'weight': 1.0})

    n_true = R @ WORLD_UP
    n_true /= np.linalg.norm(n_true)
    found, diag = solve_frame(lines, K, n_true, size_mm=(W, H),
                              long_side_parallel=False,
                              use_pos=[0, 1, 2, 3], quad_hint=q)

    assert found is not None, (side, diag)
    assert diag['reproj_px'] < 1e-6, (side, diag)
    assert np.allclose(_project_px(_corners_mm(found, W, H), K), q,
                       atol=1e-6), side

    # X 朝右、Y 朝远（图像里 v 减小），与“哪一侧看”无关
    assert _projected_dir(found['origin'], found['E1'])[0] > 0.99, side
    assert _projected_dir(found['origin'], found['E2'])[1] < -0.1, side

    # 法向朝相机（相机在上方 -> 世界朝上），不会翻成朝下
    assert float(found['nrm'] @ n_true) > 0.9999, (side, found['nrm'], n_true)
    assert found['nrm'][2] < 0.0, (side, found['nrm'])
