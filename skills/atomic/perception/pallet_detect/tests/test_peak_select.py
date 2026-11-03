# -*- coding: utf-8 -*-
"""深度直方图**选峰**的自检（`solve_obb._pick_peak_bin`）。

背景（真机 881 帧，`maduo/debug/jitter/WORKLOG_jitter.md` §9/§11）：
托盘大面积出画时，框内沿法向同时存在两个近等高的峰（台面 ~870-910、
地面/别的层 ~650-690）。原实现取 `argmax`，近峰只要以 0.006~0.034 的占比
优势胜出就会把 D 锁到 657（低 26%），位姿跳 300mm。

改成"在占比 >= `PEAK_MIN` 的峰里挑**最接近 `D_REF_MM`** 的那个"之后，
真机 `>100mm` 的大跳 20 → 13、max 379.7 → 335.9mm，角点距不变。

本文件钉三件事：

1. **只有一个峰达标时与旧的 `argmax` 完全等价** —— 这是本改动在多数帧上
   "零行为变化"的依据，也是它不会悄悄改掉 601 帧回归的原因；
2. **两个峰都达标时按先验挑**（而不是挑大的）；
3. **占比不够的峰不参与候选** —— 否则噪声峰也会被当成"台面"。

跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 skills/atomic/perception/pallet_detect/tests/test_peak_select.py
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

from .. import solve_obb as SO

_fails: list[str] = []


def check(name: str, cond: bool, detail: str = '') -> None:
    if cond:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        _fails.append(name)


def _hists():
    """构造两份直方图。

    `edges` 是按 40mm 分箱的**边界**；两个峰的 bin 中心分别是 -890 与 -660
    （负号对应调用方传未翻符号法向的情况 —— 选峰用 `abs(ctr)` 比较）。
    """
    bin_mm = 40.0
    lo = -1200.0
    edges = np.arange(lo, lo + bin_mm * 11 + 1e-9, bin_mm)
    centers = 0.5 * (edges[:-1] + edges[1:])
    i_deck = int(np.argmin(np.abs(centers - (-890.0))))
    i_other = int(np.argmin(np.abs(centers - (-660.0))))
    return edges, centers, i_deck, i_other


def test_single_peak_is_identical_to_argmax():
    """★ 只有一个峰达标时必须与旧的 `argmax` 逐位一致（多数帧靠这条不变）。"""
    edges, centers, i_deck, _ = _hists()
    hist = np.zeros(len(edges) - 1, int)
    hist[i_deck] = 800
    hist[2] = 30                      # 3.6% —— 远低于 PEAK_MIN，不参与候选
    hist[5] = 20
    share = hist / hist.sum()
    old = int(np.argmax(hist))
    new = SO._pick_peak_bin(hist, edges, share)
    check('单峰：与 argmax 一致', new == old, f'new={new} old={old}')
    check('单峰：选中的是台面', new == i_deck, f'new={new} deck={i_deck}')


def test_two_peaks_pick_the_one_near_the_prior():
    """★ 两个峰都达标时按 `D_REF_MM` 挑，**不是**挑占比大的。"""
    edges, centers, i_deck, i_other = _hists()
    hist = np.zeros(len(edges) - 1, int)
    hist[i_other] = 520               # 地面层略大
    hist[i_deck] = 500                # 台面略小 —— 旧实现会选错
    share = hist / hist.sum()
    check('先验生效：旧 argmax 确实会选错',
          int(np.argmax(hist)) == i_other, '构造失败')
    new = SO._pick_peak_bin(hist, edges, share)
    check('双峰：选台面而不是最大的', new == i_deck,
          f'new={new} deck={i_deck} other={i_other}')


def test_peak_below_min_share_is_not_a_candidate():
    """占比不够的峰不参与候选 —— 否则噪声峰也会被当成台面。"""
    edges, centers, i_deck, i_other = _hists()
    hist = np.zeros(len(edges) - 1, int)
    hist[i_deck] = 500
    hist[i_other] = 100               # 100/600 = 16.7% < PEAK_MIN(20%)，不该入选
    share = hist / hist.sum()
    new = SO._pick_peak_bin(hist, edges, share)
    check('低占比峰不参与', new == i_deck, f'new={new} deck={i_deck}')
    check('门槛确实是 0.20',
          abs(SO.PEAK_MIN - 0.20) < 1e-12, f'PEAK_MIN={SO.PEAK_MIN}')


def test_peak_min_disabled_falls_back_to_argmax():
    """`PEAK_MIN <= 0` = 关闭选峰（回到旧行为），留给离线对照用。"""
    edges, centers, i_deck, i_other = _hists()
    hist = np.zeros(len(edges) - 1, int)
    hist[i_deck] = 500
    hist[i_other] = 520
    share = hist / hist.sum()
    old_min = SO.PEAK_MIN
    try:
        SO.PEAK_MIN = 0.0
        new = SO._pick_peak_bin(hist, edges, share)
    finally:
        SO.PEAK_MIN = old_min
    check('关闭后退回最大 bin', new == i_other, f'new={new} other={i_other}')


def test_lock_threshold_is_zero_for_obb():
    """OBB 的锁 D 阈值必须是 0（真机 881 帧实测：max 990.9 → 379.7mm）。

    ⚠️ POSE 前端**不用**这个常量（它有自己的 `POSE_D_LOCK_INLIER=0.30`）——
    那条前端在阈值降到 0 时可见角点距退化 26%，是实测过的取舍。
    """
    check('OBB 的 D_LOCK_INLIER == 0', abs(SO.D_LOCK_INLIER) < 1e-12,
          f'D_LOCK_INLIER={SO.D_LOCK_INLIER}')
    from .. import detect_pose as DP
    check('POSE 保持 0.30', abs(DP.POSE_D_LOCK_INLIER - 0.30) < 1e-12,
          f'POSE_D_LOCK_INLIER={DP.POSE_D_LOCK_INLIER}')


def main():
    print('== 选峰：单峰与 argmax 等价 ==')
    test_single_peak_is_identical_to_argmax()
    print('== 选峰：双峰按先验挑 ==')
    test_two_peaks_pick_the_one_near_the_prior()
    print('== 选峰：低占比峰不参与 ==')
    test_peak_below_min_share_is_not_a_candidate()
    print('== 选峰：可关闭 ==')
    test_peak_min_disabled_falls_back_to_argmax()
    print('== 阈值常量（分前端）==')
    test_lock_threshold_is_zero_for_obb()
    print()
    if _fails:
        print(f'{len(_fails)} 项失败：{", ".join(_fails)}')
        return 1
    print('全部通过')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
