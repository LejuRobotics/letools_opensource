# -*- coding: utf-8 -*-
"""首帧法向拟合的**合理性自检**（2026-10-22 现场事故）。

事故现场（两个日志、同一模型）：

    17:44  normal=[-0.0042,-0.9388,-0.3445]（侧向，与光轴 69.8°）  D=241mm   pub=0
    17:49  normal=[ 0.0297,-0.2672,-0.9632]（正对托盘，与光轴 15.6°）D=970mm  pub=91

第一次的 TF 初值本身就歪了，拟合"与初值只差 0.05°、内点 487"看着很自信，
旧代码照样缓存 -> D 闸门把每一帧都拒掉，只能重启。本文件钉三件事：

1. `normal_is_plausible`：光轴夹角 + 拟合平面距离两道闸都能拒掉歪拟合；
2. `fit_floor_normal_checked`：初值侧向时**用光轴当初值重试**能找回台面；
3. 两次都不合格时返回 `None` —— 调用方据此**不写缓存**（不会永久生效）。

跑法：`python3 -m pytest skills/atomic/perception/pallet_detect/tests/test_normal_guard.py -q`
（不需要 ROS / 相机 / 模型）。
"""
from __future__ import annotations

import numpy as np

from ..normals import fit_floor_normal_checked, normal_is_plausible


class K:
    fx = fy = 400.0
    cx, cy = 320.0, 240.0


def _flat_depth(z_mm: float = 1000.0, h: int = 480, w: int = 640):
    """一整张都是同一深度 -> 一个正对光轴的平面（法向 [-z]）。"""
    return np.full((h, w), float(z_mm), np.float32)


# 现场两次的日志值（2026-10-22）
GOOD_NORMAL = np.array([0.0297, -0.2672, -0.9632])
BAD_NORMAL = np.array([-0.0042, -0.9388, -0.3445])


def test_good_field_normal_passes():
    ok, why = normal_is_plausible(GOOD_NORMAL, max_tilt_deg=60.0,
                                  plane_mm=970.0, d_ref_mm=990.0, d_tol=0.35)
    assert ok, why


def test_lateral_field_normal_is_rejected_by_tilt():
    ok, why = normal_is_plausible(BAD_NORMAL, max_tilt_deg=60.0,
                                  plane_mm=241.0, d_ref_mm=990.0, d_tol=0.35)
    assert not ok
    assert '光轴夹角' in why and '69' in why, why


def test_correct_direction_but_wrong_plane_is_rejected():
    """方向对、但拟到了别的一层（241mm vs 990mm）也必须拒。"""
    ok, why = normal_is_plausible([0.0, 0.0, -1.0], max_tilt_deg=60.0,
                                  plane_mm=241.0, d_ref_mm=990.0, d_tol=0.35)
    assert not ok and '拟合平面' in why, why


def test_floor_vs_deck_distance_is_tolerated():
    """拟合到的常是地面（~1100mm），与台面 ref=990 差 ~11%，必须放行 ——
    这是自检闸（35%），不是发布闸（10%）。"""
    ok, why = normal_is_plausible([0.0, 0.0, -1.0], max_tilt_deg=60.0,
                                  plane_mm=1100.0, d_ref_mm=990.0, d_tol=0.35)
    assert ok, why


def test_gates_can_be_disabled():
    ok, why = normal_is_plausible([0.0, 1.0, 0.0], max_tilt_deg=0.0,
                                  plane_mm=1.0, d_ref_mm=990.0, d_tol=0.0)
    assert ok, why


def test_checked_fit_reports_plane_distance():
    n, n_inl, plane_mm, why = fit_floor_normal_checked(
        _flat_depth(1000.0), K, np.array([0.0, 0.0, -1.0]),
        max_tilt_deg=60.0, d_ref_mm=990.0, d_tol=0.35)
    assert n is not None and why == ''
    assert n_inl > 400
    assert abs(float(n[2])) > 0.99
    assert plane_mm is not None
    assert abs(plane_mm - 1000.0) < 5.0, plane_mm


def test_sideways_prior_is_recovered_with_optical_axis():
    """★ 事故复现：TF 初值侧向 -> 第一次拟合在 ±25° 窗里找不到台面；
    改用光轴初值必须把台面找回来，并明确说出"首次不合格"。"""
    n, _n_inl, plane_mm, why = fit_floor_normal_checked(
        _flat_depth(1000.0), K, np.array([0.0, -1.0, 0.0]),
        max_tilt_deg=60.0, d_ref_mm=990.0, d_tol=0.35)
    assert n is not None and why, why
    assert '光轴初值' in why, why
    assert abs(float(n[2])) > 0.99
    assert plane_mm is not None
    assert abs(plane_mm - 1000.0) < 5.0, plane_mm


def test_both_candidates_bad_returns_none_so_caller_does_not_cache():
    """两次都不合格 -> None。调用方据此回退且**不写缓存**（这是自愈的前提）。"""
    n, _n_inl, plane_mm, why = fit_floor_normal_checked(
        _flat_depth(1000.0), K, np.array([0.0, -1.0, 0.0]),
        max_tilt_deg=60.0, d_ref_mm=3000.0, d_tol=0.35)
    assert n is None and plane_mm is None
    assert '两次都不合格' in why, why
