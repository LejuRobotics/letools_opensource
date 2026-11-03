#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`neighbor.py` 的构造场景自检 —— **不需要 ROS、不需要相机、不需要模型文件**。

跑法::

    cd <仓库根>
    python3 -m pytest skills/atomic/perception/pallet_servo/tests/test_neighbor.py -q

用例与期望值是从 `maduo/tools/neighbor_servo.py --selftest` **逐条搬过来的**
（那份脚本的 `--selftest` 全绿，是行为基线）：约束只有一个 —— 搬进来之后断言
的**条件与被测数一个字不改**，只把数据形态从 dict 换成 `BoxObservation`
（四角填 `quad`）。

为什么场景要**合成**而不是拿真图回归
====================================
邻居路线的判据全部是**相对**的（相邻间隙、与手里箱的对位比例），跟箱子的绝对
像素尺寸无关；真图上反而分不清"判据挑错了邻居"还是"检测本身漂了"。合成场景把
"谁挨着谁、谁对位、同排从哪端起算"钉死，判据一旦改松改紧立刻红。

⚠️ 合成场景用的是**真实尺寸**（530×350mm，见 `neighbor.py` 的「真实尺寸」一节），
那是为了让邻居的**排列**贴合真实托盘 —— **生产判据一条都不碰长宽比**，这几条钉
尺寸/布局的断言改箱子尺寸时就会变，那正是要报警的时候。

⚠️ 第 8 组（档位表）搬过来时**换了被测对象**：`slot -> side` 那张表是**配置**，
按接入契约 §4 进 `board.json` 的 `servo_slots`，不属于算法层（硬约束 §4「不写死
垛型」）。算法层仍然拥有的是 `NeighborSlot.side` 的**取值白名单**，所以原脚本
"side 只认 left/right/none" 这条断言原文保留，另外两条（1/2 走托盘、3 right /
4 none / 5 left）的等价物是"三个合法值都构造得出、非法值构造即炸"。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import pytest

# 以仓库根为参照 import（与 test_render_servo_overlay.py 同一套做法）。
if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))

from skills.atomic.perception.pallet_servo.algorithm import (  # noqa: E402
    BoxObservation,
    Reject,
)
from skills.atomic.perception.pallet_servo.neighbor import (  # noqa: E402
    DEGENERATE_SPAN_PX,
    NeighborError,
    NeighborSlot,
    neighbor_servo,
)

# --------------------------------------------------------------------------- #
# 合成场景：真实尺寸（530x350mm）与布局，见 neighbor.py 的「真实尺寸」一节
# --------------------------------------------------------------------------- #
BOX_LONG_MM, BOX_SHORT_MM = 530.0, 350.0
PX_PER_MM = 0.63                       # 只为把场景画得像照片，不影响任何判据
L_LONG = BOX_LONG_MM * PX_PER_MM       # 334px：长边在图像里占多少像素
L_SHORT = BOX_SHORT_MM * PX_PER_MM     # 220px：短边

# 左列三箱**长边横放**（宽 530、深 350）；右列两箱**长边竖放**（宽 350、深 530）。
# 两列总深都是 ~1050mm，底边齐平 —— 与操作员的示意图一致。
_X_L = (335.0, 335.0 + L_LONG)         # 左列：宽 = 530mm
_X_R = (_X_L[1], _X_L[1] + L_SHORT)    # 右列：宽 = 350mm
_V_BOT = 680.0                         # 两列共用的底（子图像 v 向下，底 = v 大）
_V_5 = (_V_BOT - 3 * L_SHORT, _V_BOT - 2 * L_SHORT)
_V_2 = (_V_BOT - 2 * L_SHORT, _V_BOT - 1 * L_SHORT)
_V_3 = (_V_BOT - 1 * L_SHORT, _V_BOT)
_V_1 = (_V_BOT - 1 * L_LONG, _V_BOT)
_V_4 = (_V_BOT - 2 * L_LONG, _V_BOT - 1 * L_LONG)
ROWS = {'5': _V_5, '2': _V_2, '3': _V_3, '4': _V_4, '1': _V_1}
COLS = {'L': _X_L, 'R': _X_R}
PLACE = {3: ('L', '3'), 2: ('L', '2'), 5: ('L', '5'),
         1: ('R', '1'), 4: ('R', '4')}

# 档位 -> (手里箱, 上邻居, 侧邻居, 是否翻转画面)。1、2 走托盘路线，不在本层。
CASES = {
    3: (3, 2, 1, False),
    5: (5, 2, 4, True),
    4: (4, 1, None, True),
}
# 配置里的 side（画面口径，与 board.json 的 servo_slots 一致）
SIDE_OF = {3: 'right', 4: 'none', 5: 'left'}


def _rect(x0, y0, x1, y1) -> np.ndarray:
    """图像坐标矩形 -> 契约序四角（右下 左下 左上 右上）。

    与 `algorithm.box_corners` 的 AABB 分支同一顺序，也过得了它的凸环判据。
    """
    return np.array([[x1, y1], [x0, y1], [x0, y0], [x1, y0]], float)


def layout(flipped: bool = False) -> Dict[int, np.ndarray]:
    """按示意图造五个箱子；`flipped=True` 模拟相机转了 180°（上下左右都反）。

    翻转是"沿画面中心镜像两个坐标轴"。镜像会把环序反过来（顺 -> 逆），这里不再
    依赖 `order_corners_uv` 去重排，而是**直接从镜像后的矩形边界重新构造**契约序
    四角 —— 对轴对齐矩形两者等价，且不必在测试里再养一份排序实现。
    """
    out = {}
    for k, (col, row) in PLACE.items():
        x0, x1 = COLS[col]
        y0, y1 = ROWS[row]
        out[k] = _rect(x0, y0, x1, y1)
    if flipped:
        allp = np.vstack(list(out.values()))
        cx, cy = allp[:, 0].mean(), allp[:, 1].mean()
        out = {k: _rect(2 * cx - float(q[:, 0].max()), 2 * cy - float(q[:, 1].max()),
                        2 * cx - float(q[:, 0].min()), 2 * cy - float(q[:, 1].min()))
               for k, q in out.items()}
    return out


def _obs(quad, conf: float = 0.9) -> BoxObservation:
    """一张候选观测：四角进 `quad`（`box_corners` 优先用它）。"""
    return BoxObservation(quad=np.asarray(quad, float).tolist(), confidence=conf)


def _scene(lay: Dict[int, np.ndarray], hk: int, confs=None, drop=()):
    """造 `(candidates, held)`：`held` 就是候选里的**同一个对象**。

    ⚠️ 一定要这么造。给手里箱另造一个"四角一模一样"的副本，会被当成它自己的
    邻居（中心重合、方向算不出来）—— `neighbor.py` 拿 `dup_of_held` 兜这种情况，
    但那是"调用方传错了"的保险，不是正常路子。
    """
    confs = confs or {}
    ks = [k for k in sorted(lay) if k not in drop]
    cands = [_obs(lay[k], confs.get(k, 0.9)) for k in ks]
    return cands, cands[ks.index(hk)]


def _which(lay, center) -> Optional[int]:
    """诊断量里的中心坐标 -> 是哪个编号的箱子（最近的那个）。"""
    if center is None:
        return None
    c = np.asarray(center, float)
    best, bestd = None, None
    for k, q in lay.items():
        d = float(np.linalg.norm(_center(q) - c))
        if bestd is None or d < bestd:
            best, bestd = k, d
    return best


def _center(q) -> np.ndarray:
    return np.asarray(q, float).reshape(4, 2).mean(axis=0)


# --------------------------------------------------------------------------- #
# 0. 合成场景的几何前提
# --------------------------------------------------------------------------- #
def test_scene_geometry_premises():
    lay = layout()
    ratio = (BOX_LONG_MM / BOX_SHORT_MM)
    for k, want in ((5, ratio), (4, 1.0 / ratio)):
        q = lay[k]
        w = float(q[:, 0].max() - q[:, 0].min())
        h = float(q[:, 1].max() - q[:, 1].min())
        got = w / h
        assert abs(got - want) < 0.02, f'{k} 号 宽高比 {got:.3f} 期望 {want:.3f}'

    assert abs(float(lay[3][:, 1].max())
               - float(lay[1][:, 1].max())) < 1e-6, '两列底边不齐平'
    # 两列总深差 10mm（1050 vs 1060）—— 操作员裁定忽略不计，这里**钉住**它。
    # 未翻转的画面里 5 与 4 的底边差 (2x短 - 长) = 170mm = 107px。
    d_5_4 = abs(float(lay[5][:, 1].max()) - float(lay[4][:, 1].max()))
    assert abs(d_5_4 - (2 * L_SHORT - L_LONG)) < 1e-6, f'{d_5_4:.0f}px'


# --------------------------------------------------------------------------- #
# 1. 三个档位：邻居指认与理想值
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('slot', sorted(CASES))
def test_slot_neighbors_and_ideal_values(slot):
    hk, uk, sk, flip = CASES[slot]
    lay = layout(flipped=flip)
    r = neighbor_servo(*_scene(lay, hk), NeighborSlot(side=SIDE_OF[slot]))
    assert isinstance(r, NeighborError), f'slot {slot} 没结果：{r}'
    assert _which(lay, r.diag.get('up_center')) == uk
    assert _which(lay, r.diag.get('side_center')) == sk
    # 5 号带 +3.15px 的已知残差（两列总深差 10mm，操作员裁定忽略），容差按 4px；
    # 其余档位仍是严格 0。
    assert abs(r.e_bottom_px) < 4.0, f'{r.e_bottom_px:+.2f}'
    assert abs(r.e_right_px) < 3.0, f'{r.e_right_px:+.2f}'
    assert abs(math.degrees(r.theta_rad)) < 0.5, f'{math.degrees(r.theta_rad):+.3f}deg'


# --------------------------------------------------------------------------- #
# 2. 挪手里箱 -> 两个方向都成比例，且两项同号
# --------------------------------------------------------------------------- #
def _rigid_rot(lay, deg):
    """把整个场景**刚性旋转**（四角一起转，不重新拟 AABB）。

    ⚠️ 不能转完再取 AABB：那样盒子之间的相对位置会被改掉，"对齐"的场景变成
    不对齐，用例就失去了判别力（第一版踩过）。
    """
    allp = np.vstack(list(lay.values()))
    c = allp.mean(axis=0)
    th = np.radians(float(deg))
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return {k: (np.asarray(q, float) - c) @ R.T + c for k, q in lay.items()}


@pytest.mark.parametrize('slot', sorted(CASES))
@pytest.mark.parametrize('deg', [0.0, 10.0, 20.0, 30.0])
def test_rigid_rotation_does_not_shift_the_errors(slot, deg):
    """★ **整个场景刚性转 30° 后，到位态的误差必须仍然是 0**（2026-10-17 真机踩）。

    真机相机是斜看的（那一帧 `u_far` 与竖直差 27.5°），而旧实现里：
      * `e_bottom` 把中点差投影到 `u_far`；
      * `e_right` 取「两个框中心的图像 x 差」和「两条近边中点差 · u_side」。
    两者都把**沿"列方向"的深度差**漏进了横向/纵向 —— 合成场景刚性转 30° 时
    `e_right` 从 0 变成 **−28px**（slot3），真机上就是"明明对齐了还显示几十像素"。

    现在两条轴都取**边到边的垂直距离 / 沿接触边方向的分量**，对刚性旋转不变。
    """
    hk, uk, sk, flip = CASES[slot]
    lay = _rigid_rot(layout(flipped=flip), deg)
    r = neighbor_servo(*_scene(lay, hk), NeighborSlot(side=SIDE_OF[slot]))
    assert isinstance(r, NeighborError), f'slot {slot} 转 {deg}°：{r}'
    # 5 号带 **−3.15px** 的已知残差（两列总深差 10mm，操作员裁定忽略不计），
    # 与 `test_slot_neighbors_and_ideal_values` 用同一个 4px 容差。
    assert abs(r.e_bottom_px) < 4.0, (
        f'slot {slot} 转 {deg}°：到位态 e_bottom={r.e_bottom_px:+.2f}（应≈0）')
    assert abs(r.e_right_px) < 2.0, (
        f'slot {slot} 转 {deg}°：到位态 e_right={r.e_right_px:+.2f}（应≈0；'
        f'旧口径在这一步是 −28px）')


@pytest.mark.parametrize('slot', sorted(CASES))
def test_applying_the_target_shift_zeroes_the_error(slot):
    """★ 把手里的箱子移到"图上黄线画的位置"之后，两个误差必须归零。

    黄线的平移方向就是这里用的两个方向：
        底边  `+e_bottom · u_bottom`
        横向  `−e_right  · u_side_shift`
    所以这一条同时钉住**符号**与**可视化的方向** —— 任何一处反了，移过去之后
    误差不会归零（而是变成两倍）。`e_right` 在真机上就画反过一次。
    """
    hk, uk, sk, flip = CASES[slot]
    lay = layout(flipped=flip)
    slotcfg = NeighborSlot(side=SIDE_OF[slot])
    ks = [k for k in sorted(lay)]

    # 先用一个"没对好"的姿态造出非零误差（纯平移，邻居判据照样成立）
    q = lay[hk].copy()
    q[:, 0] += 25.0
    q[:, 1] -= 18.0
    d2 = [_obs(lay[k]) for k in ks]
    d2[ks.index(hk)] = _obs(q)
    r = neighbor_servo(d2, d2[ks.index(hk)], slotcfg)
    assert isinstance(r, NeighborError), f'slot {slot} 未对齐帧：{r}'
    assert abs(r.e_bottom_px) > 5.0 and abs(r.e_right_px) > 5.0, (
        f'slot {slot}：这一帧本就该有明显误差（否则用例测不到方向）'
        f' e_bottom={r.e_bottom_px:+.1f} e_right={r.e_right_px:+.1f}')

    delta = (np.asarray(r.u_bottom, float) * r.e_bottom_px
             - np.asarray(r.u_side_shift, float) * r.e_right_px)
    q2 = q + delta
    d3 = [_obs(lay[k]) for k in ks]
    d3[ks.index(hk)] = _obs(q2)
    r2 = neighbor_servo(d3, d3[ks.index(hk)], slotcfg)
    assert isinstance(r2, NeighborError), f'slot {slot} 校正后：{r2}'
    assert abs(r2.e_bottom_px) < 1.5, (
        f'slot {slot}：按 u_bottom 平移 {r.e_bottom_px:+.1f}px 后 e_bottom 仍为 '
        f'{r2.e_bottom_px:+.1f}（方向反了会让它变成两倍）')
    assert abs(r2.e_right_px) < 1.5, (
        f'slot {slot}：按 −u_side_shift 平移 {r.e_right_px:+.1f}px 后 e_right 仍为 '
        f'{r2.e_right_px:+.1f}（黄线方向反了就是这个症状）')


@pytest.mark.parametrize('slot', sorted(CASES))
def test_lateral_offset_does_not_leak_into_e_bottom(slot):
    """★ **横向偏多少都不许进 `e_bottom`**（2026-10-17 真机踩）。

    真机那一帧 `u_far` 与竖直差 **27.5°**（上邻居在画面左边 112px）。旧实现把
    「侧邻居下边中点 − 手里下边中点」投影到 `u_far` 上，于是「手里箱偏右 100px」
    被算进了底边误差：**62px 的真值被算成 165px**，叠加图上目标底边高得离谱。

    判据：只看四角几何 —— 把手里箱**纯横向**平移 80px（底边高度一点没变），
    `e_bottom` 必须基本不变；同时 `u_far` 确实被带斜了（否则这条用例没测到东西）。
    """
    hk, uk, sk, flip = CASES[slot]
    lay = layout(flipped=flip)
    slotcfg = NeighborSlot(side=SIDE_OF[slot])
    base = neighbor_servo(*_scene(lay, hk), slotcfg)
    assert isinstance(base, NeighborError), f'slot {slot} 基准：{base}'

    q = lay[hk].copy()
    q[:, 0] += 80.0                       # 纯横向：v 一个像素都不动
    ks = [k for k in sorted(lay)]
    d2 = [_obs(lay[k]) for k in ks]
    d2[ks.index(hk)] = _obs(q)
    r = neighbor_servo(d2, d2[ks.index(hk)], slotcfg)
    assert isinstance(r, NeighborError), f'slot {slot} 横移后：{r}'

    # 前提：这一挪确实把参照方向带斜了（否则旧公式也不会错，用例失去判别力）
    u_far = np.asarray(r.diag['u_far'], np.float64)
    assert abs(float(u_far[0])) > 0.10, (
        f'slot {slot}: 横移 80px 后 u_far={np.round(u_far, 3)} 仍是竖直的 —— '
        f'这条用例测不到"横向漏进底边"那个病')

    assert abs(r.e_bottom_px - base.e_bottom_px) < 3.0, (
        f'slot {slot}: 纯横向挪 80px 把 e_bottom 从 {base.e_bottom_px:+.2f} '
        f'带到了 {r.e_bottom_px:+.2f} —— 横向误差漏进了底边（旧实现的病）')


@pytest.mark.parametrize('slot', [s for s in sorted(CASES) if CASES[s][2] is not None])
@pytest.mark.parametrize('axis,tag', [(0, 'u'), (1, 'v')])
def test_move_held_is_proportional(slot, axis, tag):
    hk, uk, sk, flip = CASES[slot]
    lay = layout(flipped=flip)
    slotcfg = NeighborSlot(side=SIDE_OF[slot])
    base = neighbor_servo(*_scene(lay, hk), slotcfg)
    assert isinstance(base, NeighborError), f'slot {slot} 基准：{base}'

    # 把手里箱在原坐标上挪 12px（两个方向各一次）
    ks = [k for k in sorted(lay)]
    d2 = [_obs(lay[k]) for k in ks]
    hi = ks.index(hk)
    q = lay[hk].copy()
    q[:, axis] += 12.0
    d2[hi] = _obs(q)                       # 挪过的那个就是新的"手里箱"
    r = neighbor_servo(d2, d2[hi], slotcfg)
    assert isinstance(r, NeighborError), f'slot {slot} 挪 {tag}+12 没结果：{r}'

    d_b = r.e_bottom_px - base.e_bottom_px
    d_r = r.e_right_px - base.e_right_px
    if axis == 1:
        assert abs(abs(d_b) - 12.0) < 3.0, f'纵向挪 12 -> e_bottom 变 {d_b:+.2f}'
        t1, t2 = r.diag['t1_bottom'], r.diag['t2_bottom']
        b1, b2 = base.diag['t1_bottom'], base.diag['t2_bottom']
        assert (t1 - b1) * (t2 - b2) > 0, \
            f'纵向两项不同号（t1 {b1:+.1f}->{t1:+.1f}，t2 {b2:+.1f}->{t2:+.1f}）'
    else:
        assert abs(abs(d_r) - 12.0) < 3.0, f'横向挪 12 -> e_right 变 {d_r:+.2f}'


# --------------------------------------------------------------------------- #
# 3. 反例 A：拿 5 时上邻居必须是 2，不能是 3
# --------------------------------------------------------------------------- #
def test_counterexample_a_up_is_2_not_3():
    lay = layout(flipped=True)
    r = neighbor_servo(*_scene(lay, 5), NeighborSlot(side='left'))
    assert isinstance(r, NeighborError), str(r)
    assert _which(lay, r.diag.get('up_center')) == 2
    hc = _center(lay[5])
    assert float((_center(lay[3]) - hc) @ r.diag['u_far']) > 0, \
        '3 其实在 u_far 正向上，应当靠"对位"排除'


# --------------------------------------------------------------------------- #
# 4. 反例 B：拿 4 时 2 不能被当成侧邻居
# --------------------------------------------------------------------------- #
def test_counterexample_b_2_is_not_side_neighbor():
    lay = layout(flipped=True)
    # 故意配成 left，看会不会误抓 2
    r = neighbor_servo(*_scene(lay, 4), NeighborSlot(side='left'))
    assert isinstance(r, NeighborError), str(r)
    assert r.diag['w_side'] == 0.0, str(r.diag.get('side_missing'))
    r2 = neighbor_servo(*_scene(lay, 4), NeighborSlot(side='none'))
    assert isinstance(r2, NeighborError), str(r2)
    assert abs(r2.e_bottom_px - r.e_bottom_px) < 1e-9, '配置 none 时结果应一致'


# --------------------------------------------------------------------------- #
# 4b. 真机冻结帧：两个默认阈值必须容得下"还没到位"的误差
# --------------------------------------------------------------------------- #
# 2026-10-18 抓的 slot 3 实拍（图像 640x480）。这一帧两个邻居**都只差一点**：
#   * 上邻居：间隙 gy ≈ 0.40× 手里箱列向跨度（旧阈值 0.35 拒）；
#   * 右侧邻居：沿列方向的重叠 oy ≈ 0.45（旧阈值 0.50 拒），且它被画面右缘截断。
# 伺服开始时"还没到位"正是常态 —— 这 0.4 个箱深就是伺服的被控量。阈值比它紧，
# 就等于"误差越大越找不到参照"，方向正好反了。
REAL_HELD = _rect(177.5, 244.0, 494.3, 450.3)   # 四角带 ~1° 旋转，这里取外接
REAL_UP = _rect(275.0, 23.0, 527.0, 165.0)
REAL_SIDE = _rect(517.0, 87.0, 650.0, 341.0)    # 右缘 650 > 640：被截断


def _real_frame():
    cands = [_obs(REAL_UP), _obs(REAL_SIDE), _obs(REAL_HELD)]
    return cands, cands[2]


def test_real_frame_finds_both_neighbors_with_default_thresholds():
    r = neighbor_servo(*_real_frame(), NeighborSlot(side='right'),
                       image_size=(640, 480))
    assert isinstance(r, NeighborError), f'真机帧又被拒了：{r}'
    assert r.diag['w_side'] > 0.0, '右侧邻居没被认出来'
    assert float(r.diag['up_center'][1]) < 200.0, '上邻居认错了（应在画面上方）'
    # 这一帧还没到位：底边误差为正（前进），横向偏右为负（往右挪）。
    assert r.e_bottom_px > 60.0, f'e_bottom={r.e_bottom_px:+.1f}'
    assert r.e_right_px < 0.0, f'e_right={r.e_right_px:+.1f}'
    # 被截断的侧邻居要告警（中心/边可能有偏），但不拒帧。
    assert any('截断' in w for w in r.warn), r.warn


def test_real_frame_would_be_rejected_by_the_old_thresholds():
    # 旧值（0.35 / 0.50）在这里**两侧都拒**：上邻居 gy=0.40>0.35、侧邻居 oy=0.45<0.50。
    r = neighbor_servo(*_real_frame(),
                       NeighborSlot(side='right', max_gap=0.35, min_overlap=0.50),
                       image_size=(640, 480))
    assert isinstance(r, Reject) and r.code == 'neighbor_no_up', str(r)
    # 诊断必须报判据真正用的 ox（排方向重叠），不能只报侧邻居用的 oy。
    assert 'ox=' in r.detail and 'oy=' in r.detail, r.detail


# --------------------------------------------------------------------------- #
# 5. 侧邻居缺失 -> 只用上邻居，不报错
# --------------------------------------------------------------------------- #
def test_side_missing_degrades_to_up_only():
    lay = layout(flipped=False)
    r = neighbor_servo(*_scene(lay, 3, drop=(1,)), NeighborSlot(side='right'))
    assert isinstance(r, NeighborError), str(r)
    assert r.diag['w_side'] == 0.0
    assert r.diag['t1_bottom'] is None


# --------------------------------------------------------------------------- #
# 6. 上邻居缺失 -> 明确失败，不乱挑
# --------------------------------------------------------------------------- #
def test_up_missing_is_rejected():
    lay = layout(flipped=False)
    r = neighbor_servo(*_scene(lay, 3, drop=(2,)), NeighborSlot(side='right'))
    assert isinstance(r, Reject), f'本该被拒，却给出 {r}'
    assert r.code == 'neighbor_no_up', r.code
    assert 'neighbor_no_up' == r.code  # 与老脚本的 reason 同名


# --------------------------------------------------------------------------- #
# 7. 手里箱的副本传进来 -> 不被当成邻居
# --------------------------------------------------------------------------- #
def test_duplicate_of_held_is_skipped():
    lay = layout(flipped=False)
    cands, held = _scene(lay, 3)
    cands = cands + [_obs(lay[3].copy())]      # 副本（内容相同、另一个对象）
    r = neighbor_servo(cands, held, NeighborSlot(side='right'))
    assert isinstance(r, NeighborError), str(r)
    assert r.diag['dup_of_held'] == 1, str(r.diag['dup_of_held'])
    assert _which(lay, r.diag['up_center']) == 2


# --------------------------------------------------------------------------- #
# 8. 档位：合法 side 白名单（slot 表本身在 board.json 的 servo_slots）
# --------------------------------------------------------------------------- #
def test_slot_side_whitelist():
    for side in ('left', 'right', 'none'):
        s = NeighborSlot(side=side)
        assert s.side == side
    with pytest.raises(ValueError):
        NeighborSlot(side='up')
    with pytest.raises(ValueError):
        NeighborSlot(side='')
    with pytest.raises(ValueError):
        NeighborSlot(side='Right')


# --------------------------------------------------------------------------- #
# 9. 四角一律走 box_corners()：没有 quad 时用 AABB 合成，两条路径同结果
# --------------------------------------------------------------------------- #
def test_clipped_reference_is_warned_but_not_rejected():
    """★ 参照框伸出画面 -> **告警但照出数**（2026-10-17 真机：上/侧邻居都被截）。

    截断的检测框中心与边都是偏的（实测 slot4 的上邻居四角 y 到 −27、框
    209x203 而真实约 230x286），误差会跟着偏几十像素。现场需要知道这一点，
    但**不能拒帧** —— 视野就摆在那儿，拒了就没数可用。
    """
    slotcfg = NeighborSlot(side='right')
    # 合成场景：v 到 680、u 到 889 —— 640x480 的画布一定截断
    small = neighbor_servo(*_scene(layout(), 3), slotcfg, image_size=(640, 480))
    assert isinstance(small, NeighborError), small
    assert any('截断' in w for w in small.warn), small.warn

    # 画布足够大 -> 不该有这条（同一份几何）
    big = neighbor_servo(*_scene(layout(), 3), slotcfg, image_size=(4000, 4000))
    assert isinstance(big, NeighborError), big
    assert not any('截断' in w for w in big.warn), big.warn
    # 不传 image_size -> 不查（离线/合成场景的默认）
    none = neighbor_servo(*_scene(layout(), 3), slotcfg)
    assert isinstance(none, NeighborError)
    assert not any('截断' in w for w in none.warn), none.warn


def test_aabb_and_quad_give_same_result():
    lay = layout(flipped=False)
    ks = sorted(lay)
    aabb = []
    for k in ks:
        x0, y0 = lay[k][:, 0].min(), lay[k][:, 1].min()
        x1, y1 = lay[k][:, 0].max(), lay[k][:, 1].max()
        aabb.append(BoxObservation(u1=x0, v1=y0, u2=x1, v2=y1, confidence=0.9))
    held = aabb[ks.index(3)]
    r = neighbor_servo(aabb, held, NeighborSlot(side='right'))
    assert isinstance(r, NeighborError), str(r)
    assert abs(r.e_bottom_px) < 4.0 and abs(r.e_right_px) < 3.0


# --------------------------------------------------------------------------- #
# 10. 手里箱的框退化（u/v 同为一点）-> Reject 而不是抛异常
# --------------------------------------------------------------------------- #
def test_held_with_bad_quad_is_rejected_not_raised():
    lay = layout(flipped=False)
    cands, _ = _scene(lay, 3)
    bad = BoxObservation(u1=10.0, v1=10.0, u2=10.0, v2=10.0)   # 退化框
    r = neighbor_servo(cands + [bad], bad, NeighborSlot(side='right'))
    assert isinstance(r, Reject), str(r)
    assert r.code == 'neighbor_no_held', r.code


def test_box_corners_reject_is_not_raised():
    """`box_corners` 的 Reject 直接进候选时被跳过（计数），不毁整帧。"""
    lay = layout(flipped=False)
    cands, held = _scene(lay, 3)
    junk = BoxObservation(quad=[[0, 0], [1, 0], [0, 1], [0, 0]])  # 自交环
    r = neighbor_servo(cands + [junk], held, NeighborSlot(side='right'))
    assert isinstance(r, NeighborError), str(r)
    assert r.diag['n_bad_corners'] == 1, str(r.diag)
    assert any('四角取不出来' in w for w in r.warn)


# --------------------------------------------------------------------------- #
# 11. ★ 符号：底边「还没到位 -> 正」，横向「偏右 -> 正」，且 3/4/5 同一套
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('slot', sorted(CASES))
def test_signs_are_the_pallet_route_convention(slot):
    """★ **符号约定必须与 1、2 档（托盘路线）逐字相同**（2026-10-17 操作员裁定）。

    判据（操作员原话）：「手里箱子底边在伺服边下方时应该发正的让其前进，
    超过的时候应该发负的让其后退」—— `e_bottom > 0` = **还没到位**。

    造法：把手里箱沿 `−u_far`（往机器人这边、即"还没推到"）挪 20px，
    `e_bottom` 必须**变大**（变正）。反方向挪则必须变小。

    ⚠️ **4、5 档（画面翻转）与 3 档用同一套符号** —— 机器人挪到托盘对面时
    相机跟着转 180°，画面**左右**反了、**上下没反**，"远"在两种视角下都还是
    画面里的"上"。所以这里对三档用同一条断言，不做任何按 `flip` 的分支。
    """
    hk, uk, sk, flip = CASES[slot]
    lay = layout(flipped=flip)
    slotcfg = NeighborSlot(side=SIDE_OF[slot])
    base = neighbor_servo(*_scene(lay, hk), slotcfg)
    assert isinstance(base, NeighborError), f'slot {slot} 基准：{base}'
    u_far = base.diag['u_far']
    assert u_far is not None

    # 沿 −u_far 挪 20px = "还没到位" -> e_bottom 必须变大
    q = lay[hk].copy()
    q -= 20.0 * np.asarray(u_far, np.float64)
    ks = [k for k in sorted(lay)]
    d2 = [_obs(lay[k]) for k in ks]
    d2[ks.index(hk)] = _obs(q)
    moved = neighbor_servo(d2, d2[ks.index(hk)], slotcfg)
    assert isinstance(moved, NeighborError), f'slot {slot} 挪动后：{moved}'
    assert moved.e_bottom_px - base.e_bottom_px > 12.0, (
        f'slot {slot}：往机器人这边（−u_far）挪 20px 后 e_bottom 必须变大，'
        f'实际 {base.e_bottom_px:+.2f} -> {moved.e_bottom_px:+.2f} '
        f'（正 = 前进；符号反了现场会把箱子推离目标）')


@pytest.mark.parametrize('slot', [s for s in sorted(CASES) if CASES[s][2] is not None])
def test_positive_e_right_means_held_is_to_the_right(slot):
    """★ 横向：`e_right > 0` = 手里箱**偏画面右**（与托盘路线同口径）。

    造法：把手里箱整体沿 `+u` 挪 20px，`e_right` 必须变大。
    """
    hk, uk, sk, flip = CASES[slot]
    lay = layout(flipped=flip)
    slotcfg = NeighborSlot(side=SIDE_OF[slot])
    base = neighbor_servo(*_scene(lay, hk), slotcfg)
    assert isinstance(base, NeighborError)
    q = lay[hk].copy()
    q[:, 0] += 20.0
    ks = [k for k in sorted(lay)]
    d2 = [_obs(lay[k]) for k in ks]
    d2[ks.index(hk)] = _obs(q)
    moved = neighbor_servo(d2, d2[ks.index(hk)], slotcfg)
    assert isinstance(moved, NeighborError)
    assert moved.e_right_px - base.e_right_px > 10.0, (
        f'slot {slot}：向右挪 20px 后 e_right 必须变大，'
        f'实际 {base.e_right_px:+.2f} -> {moved.e_right_px:+.2f}')


# --------------------------------------------------------------------------- #
# 12. 手里箱短边退化 -> neighbor_edge_too_short
# --------------------------------------------------------------------------- #
def test_degenerate_held_edge_is_rejected():
    """手里箱窄成一条 5px 的竖条（< DEGENERATE_SPAN_PX）时顶/底边量不出误差。

    上邻居仍在原地（竖直方向贴着、沿排方向与这条竖条完全重叠），所以这帧不是
    "没有上邻居"，而是**有参照、只是参照边退化** —— 要的正是这条。
    """
    lay = layout(flipped=False)
    ks = sorted(lay)
    q3 = lay[3]
    y0, y1 = float(q3[:, 1].min()), float(q3[:, 1].max())
    u_c = float(q3[:, 0].mean())
    thin = _rect(u_c - 2.5, y0, u_c + 2.5, y1)
    dets = [_obs(thin if k == 3 else lay[k]) for k in ks]
    held = dets[ks.index(3)]
    r = neighbor_servo(dets, held, NeighborSlot(side='right'))
    assert isinstance(r, Reject), f'本该被拒，却给出 {r}'
    assert r.code == 'neighbor_edge_too_short', r.code
    assert '5.0px' in r.detail, r.detail
    assert DEGENERATE_SPAN_PX > 5.0
