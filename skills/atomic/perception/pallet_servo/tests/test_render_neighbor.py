# -*- coding: utf-8 -*-
"""`render_neighbor_overlay` 的**方向**（2026-10-17 真机踩过）。

这里只钉一件事：**黄线画在绿线的哪一侧**。数值自检（红线长度 == HUD 上的数）
早就有了，但"长度对、方向反"照样能过 —— 真机就这么踩了一次：
`e_right` 是负数（手里箱偏左、该往右走），图上黄线却画在左边。

跑法::

    python3 -m pytest skills/atomic/perception/pallet_servo/tests/test_render_neighbor.py -q
"""
from types import SimpleNamespace

import numpy as np

from skills.atomic.perception.pallet_servo.render import (
    COLOR_REF_EDGE,
    render_neighbor_overlay,
)

# 手里箱：右边 x=420（竖线）、底边 y=380（横线）——都是轴对齐，便于按坐标直查。
_QUAD = np.array([[420.0, 380.0], [380.0, 380.0],
                  [380.0, 300.0], [420.0, 300.0]], float)
_BOTTOM = np.array([[420.0, 380.0], [380.0, 380.0]], float)
_SIDE = np.array([[420.0, 380.0], [420.0, 300.0]], float)


def _err(e_bottom=0.0, e_right=0.0):
    """最小可渲染的"邻居路线误差"（字段与 `ServoError` 同名）。"""
    return SimpleNamespace(
        e_bottom_px=float(e_bottom), e_right_px=float(e_right), theta_rad=0.0,
        held_quad_px=_QUAD.copy(),
        held_bottom_px=_BOTTOM.copy(),
        held_side_px=_SIDE.copy(),
        # 底边正方向 = 往里走 = 画面向上；横向正方向 = 画面向右
        u_bottom=np.array([0.0, -1.0]),
        u_side_shift=np.array([1.0, 0.0]),
        u_side=np.array([1.0, 0.0]),
        u_far=np.array([0.0, -1.0]),
        warn=[], box_source="neighbor")


def _render(e_bottom=0.0, e_right=0.0):
    return render_neighbor_overlay(np.zeros((480, 640, 3), np.uint8),
                                   error=_err(e_bottom, e_right),
                                   neighbors=None, hud_lines=())


def _is_yellow(canvas, u, v) -> bool:
    return bool(np.all(canvas[int(v), int(u)] == np.array(COLOR_REF_EDGE, np.uint8)))


def test_negative_e_right_puts_the_target_on_the_right():
    """★ `e_right < 0`（手里箱偏左）→ 黄线在**右**边。真机就是这么反的。"""
    c = _render(e_right=-20.0)
    # 目标侧边 = 绿线（x=420）向右 20px
    assert _is_yellow(c, 440, 360), (
        "e_right=-20 时黄线应画在 x≈440（右边），实际不在 —— 横向平移方向反了")
    assert not _is_yellow(c, 400, 360), "e_right=-20 时左边不该有黄线"


def test_positive_e_right_puts_the_target_on_the_left():
    """★ `e_right > 0`（手里箱偏右）→ 黄线在**左**边（与 1、2 箱同口径）。"""
    c = _render(e_right=20.0)
    assert _is_yellow(c, 400, 360), (
        "e_right=+20 时黄线应画在 x≈400（左边）")
    assert not _is_yellow(c, 440, 360), "e_right=+20 时右边不该有黄线"


def test_positive_e_bottom_puts_the_target_above():
    """★ `e_bottom > 0`（还没到位）→ 黄线在**上**方（往里走）。"""
    c = _render(e_bottom=25.0)
    # 目标底边 = 绿线（y=380）向上 25px；横向黄线在右侧、不干扰这一列
    assert _is_yellow(c, 390, 355), (
        "e_bottom=+25 时黄线应画在 y≈355（上方）")
    assert not _is_yellow(c, 390, 405), "e_bottom=+25 时下方不该有黄线"


def test_negative_e_bottom_puts_the_target_below():
    c = _render(e_bottom=-25.0)
    assert _is_yellow(c, 390, 405), "e_bottom=-25（推过头）时黄线应画在下方"
    assert not _is_yellow(c, 390, 355), "e_bottom=-25 时上方不该有黄线"
