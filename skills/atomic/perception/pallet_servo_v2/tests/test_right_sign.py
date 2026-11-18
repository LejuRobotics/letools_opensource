# -*- coding: utf-8 -*-
"""托盘路线 `e_right` 的正方向必须是**台面 +x（画面右）**。

2026-11-11 现场 slot 4：参考边是 `x=380`（台面左半边），旧实现用参考边的
内法向定符号，内法向指向 +x，于是“箱子在参考线左侧”被算成 **+**；
控制器 `vy = gain * e_right`（+y 向画面左）会继续往左走，而箱子需要往右。
"""
import numpy as np

from ..algorithm import BoxObservation, servo_error
from skills.atomic.perception.pallet_detect_v2.geom import CameraIntrinsics, _corners_mm, _project_px


def _scene():
    Kc = CameraIntrinsics(500.0, 500.0, 320.0, 240.0)
    K = np.array([[500.0, 0.0, 320.0],
                  [0.0, 500.0, 240.0],
                  [0.0, 0.0, 1.0]])
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, 1.0]
    T[:3, :3] = np.array([[1.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0],
                          [0.0, 0.0, -1.0]])
    return Kc, K, T


def _box_quad(Kc, x_mm=250.0, width_mm=100.0):
    # 箱子放在台面平面上，y 在 200..300mm（相机 y 向下 -> 负）
    box = dict(origin=np.array([x_mm, -300.0, 1000.0]),
               E1=np.array([1.0, 0.0, 0.0]),
               E2=np.array([0.0, 1.0, 0.0]),
               nrm=np.array([0.0, 0.0, -1.0]), W=width_mm, H=100.0)
    return BoxObservation(quad=_project_px(
        _corners_mm(box, width_mm, 100.0), Kc).tolist())


def test_left_half_reference_uses_screen_right_sign():
    Kc, K, T = _scene()
    W, H = 1000.0, 1000.0
    # 箱子右边在 x=350，参考线 x=380/620 都在它右边 -> e_right 必须为负
    obs = _box_quad(Kc, x_mm=250.0)
    left = servo_error(T, (W, H), ['y=50', 'x=380'], obs, K, None, (640, 480))
    right = servo_error(T, (W, H), ['y=50', 'x=620'], obs, K, None, (640, 480))
    assert left.e_right_px < 0.0, left.e_right_px
    assert right.e_right_px < 0.0, right.e_right_px
    # 两条参考线相距 240mm，1m 处 f=500 -> 约 120px；几何关系一致
    assert abs(abs(right.e_right_px - left.e_right_px) - 120.0) < 2.0


def test_moving_the_box_right_increases_e_right():
    Kc, K, T = _scene()
    W, H = 1000.0, 1000.0
    a = servo_error(T, (W, H), ['y=50', 'x=380'],
                    _box_quad(Kc, x_mm=250.0), K, None, (640, 480))
    b = servo_error(T, (W, H), ['y=50', 'x=380'],
                    _box_quad(Kc, x_mm=300.0), K, None, (640, 480))
    assert b.e_right_px > a.e_right_px + 10.0, (a.e_right_px, b.e_right_px)
