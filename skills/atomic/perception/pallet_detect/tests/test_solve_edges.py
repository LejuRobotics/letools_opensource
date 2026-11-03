#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`solve_edges.solve_frame` 与现行 `solve_obb.frame_from_obb` 的数值等价验证。

跑法（不需要 ROS / 相机 / 模型）：

    cd <仓库根>
    python3 skills/atomic/perception/pallet_detect/tests/test_solve_edges.py

或 pytest（本文件在 `pallet_detect/tests` 下，用相对 import）。

数据：`/home/lejucv/maduo/test_data/obbval`（可用 `OBBVAL_DIR` 覆盖）

    obb.json    601 帧，每帧 `corners_px`
    meta.json   每帧 `K=[fx,fy,cx,cy]`、`nrm`
    frames.npz  键 `"<stem>/depth"`，uint16 毫米

逐帧把 OBB 四角拆成四条直线喂给 `solve_frame`，与 `frame_from_obb` 比**投影四角**。

## 闸门：先按基准自己的重投影质量分类

基准 `frame_from_obb` 在少数 bag 上会被自己的初值（`a=b=0` / `d=1200`）推入边界
坏解（实测 `2026-09-18-18-48-44` 全部 38 帧、`2026-09-30-11-04-26` 全部 85 帧，
自身重投影 152~354 px）。拿这种解逐点要求等价等于要求复现坏解，所以分类：

* **基准健康**（`ref_reproj <= 20 px`）：**不放宽** —— 逐帧四角 max 差 <= 0.5 px，
  且新解重投影不能比基准差（>0.05 px 判失败）。
* **基准坏解**（`ref_reproj > 20 px` 且新解 `reproj < 0.5 * ref_reproj`，即明显更优）：
  不要求逐点相等，只断言 `new_reproj <= ref_reproj`；输出两边 reproj 中位。
* 其余帧（含 `ref_reproj > 20` 但新解没有明显更优的）按**严格**口径处理：
  逐帧 max 差 <= 0.5 px —— 基准不健康、新解也没改善，同样不能算通过。

全局中位差仍要求 <= 0.1 px。不做参数扫描。
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

if __name__ == '__main__':
    # 与同目录其它测试一致：直接 `python3 <本文件>` 跑时补包名。
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
    __package__ = 'skills.atomic.perception.pallet_detect.tests'
    import importlib
    importlib.import_module(__package__)

import numpy as np

from ..geom import CameraIntrinsics, _corners_mm, _project_px
from ..solve_edges import solve_frame
from ..solve_obb import frame_from_obb, order_quad, usable_edges

DATA_DIR = Path(os.environ.get('OBBVAL_DIR', '/home/lejucv/maduo/test_data/obbval'))
SIZE_MM = (1200.0, 1000.0)          # 与现行生产路径一致：W=min=1000 沿 E1
MEDIAN_TOL_PX = 0.1
MAX_TOL_PX = 0.5
# 基准自身重投影（投影四角 vs 输入 quad）<= 这个值算健康。
REF_HEALTHY_MAX_PX = 20.0
# 新解 reproj 低于基准的这个比例，才算"基准坏解且新解明显更优"。
BAD_IMPROVE_RATIO = 0.5
# 严格帧里允许的新解重投影劣化（只为吃浮点/迭代噪声）。
REPROJ_WORSE_TOL_PX = 0.05


def _lines_of(quad):
    lines = []
    for i in range(4):
        d = quad[(i + 1) % 4] - quad[i]
        lines.append({'p0': quad[i], 'd': d / np.linalg.norm(d)})
    return lines


def _corners_px(frame, k):
    return _project_px(_corners_mm(frame, min(SIZE_MM), max(SIZE_MM)), k)


def _reproj_px(frame, quad, k):
    """投影四角到输入 quad 的逐角距离中位（与 `frame_from_obb` 的 `reproj_px` 同口径）。"""
    px = _corners_px(frame, k)
    return float(np.median(np.linalg.norm(px - np.asarray(quad, float), axis=1)))


def test_solve_edges_matches_frame_from_obb():
    required = [DATA_DIR / 'obb.json', DATA_DIR / 'meta.json', DATA_DIR / 'frames.npz']
    missing = [str(p) for p in required if not p.exists()]
    if missing:
        import pytest
        pytest.skip('obbval 数据不在：' + ', '.join(missing))

    with open(DATA_DIR / 'obb.json') as f:
        obb = json.load(f)
    with open(DATA_DIR / 'meta.json') as f:
        meta = json.load(f)

    stems = list(obb)
    diffs_med = []
    diffs_max = []
    times = []
    failures = []
    strict_rows = []          # (stem, maxdiff, new_reproj, ref_reproj, diag)
    bad_rows = []             # (stem, new_reproj, ref_reproj)

    with np.load(DATA_DIR / 'frames.npz') as npz:
        for stem in stems:
            mm = meta[stem]
            k = CameraIntrinsics(*mm['K'])
            nrm = np.asarray(mm['nrm'], float)
            quad = order_quad(obb[stem]['corners_px'])
            lines = _lines_of(quad)
            use_pos = usable_edges(quad)
            depth = npz[stem + '/depth']

            t0 = time.perf_counter()
            f_new, diag = solve_frame(lines, k, nrm, size_mm=SIZE_MM,
                                      long_side_parallel=False, depth=depth,
                                      use_pos=use_pos)
            times.append((time.perf_counter() - t0) * 1000.0)

            f_ref, _ = frame_from_obb({'corners_px': quad}, k, nrm,
                                      size_mm=SIZE_MM, depth=depth,
                                      long_side_parallel=False)
            if f_new is None or f_ref is None:
                failures.append((stem, 'solver_returned_none',
                                 None if f_new is None else diag))
                continue

            px_new = _corners_px(f_new, k)
            px_ref = _corners_px(f_ref, k)
            d = np.linalg.norm(px_new - px_ref, axis=1)
            med = float(np.median(d))
            mx = float(np.max(d))
            new_reproj = _reproj_px(f_new, quad, k)
            ref_reproj = _reproj_px(f_ref, quad, k)
            diffs_med.append(med)
            diffs_max.append(mx)

            if ref_reproj > REF_HEALTHY_MAX_PX and \
                    new_reproj < BAD_IMPROVE_RATIO * ref_reproj:
                # 基准被自己的初值困在坏解、新解明显更优：只要求不更差。
                bad_rows.append((stem, new_reproj, ref_reproj))
                if new_reproj > ref_reproj + REPROJ_WORSE_TOL_PX:
                    failures.append((stem, 'bad_baseline_new_worse', {
                        'new_reproj_px': round(new_reproj, 4),
                        'ref_reproj_px': round(ref_reproj, 4),
                    }))
            else:
                # 严格口径：基准健康，或基准不健康但新解也没明显改善。
                strict_rows.append((stem, mx, new_reproj, ref_reproj, diag))
                if mx > MAX_TOL_PX:
                    failures.append((stem, 'corner_diff', {
                        'median_px': round(med, 6),
                        'max_px': round(mx, 6),
                        'new_reproj_px': round(new_reproj, 4),
                        'ref_reproj_px': round(ref_reproj, 4),
                        'use_pos': use_pos,
                        'd_lock': diag.get('d_lock'),
                        'iterations': diag.get('iterations'),
                    }))
                elif new_reproj > ref_reproj + REPROJ_WORSE_TOL_PX:
                    failures.append((stem, 'reproj_worse', {
                        'new_reproj_px': round(new_reproj, 4),
                        'ref_reproj_px': round(ref_reproj, 4),
                        'max_px': round(mx, 6),
                    }))

    med = np.asarray(diffs_med)
    mx = np.asarray(diffs_max)
    ts = np.asarray(times)
    strict_mx = np.asarray([r[1] for r in strict_rows])
    bad_new = np.asarray([r[1] for r in bad_rows])
    bad_ref = np.asarray([r[2] for r in bad_rows])

    def pct(a, q):
        return float(np.percentile(a, q)) if len(a) else float('nan')

    print('\n===== solve_edges vs frame_from_obb（%d 帧）=====' % len(stems))
    print('全部帧四角差: 中位 %.6f px | p90 %.6f px | max %.6f px'
          % (pct(med, 50), pct(med, 90),
             float(mx.max()) if len(mx) else float('nan')))
    print('严格帧（基准健康 <= %.0f px，或基准不健康但新解无显著改善）: %d 帧'
          % (REF_HEALTHY_MAX_PX, len(strict_rows)))
    if len(strict_mx):
        print('  逐帧最大差: 中位 %.6f px | p90 %.6f px | max %.6f px | >%.1f px: %d'
              % (pct(strict_mx, 50), pct(strict_mx, 90), float(strict_mx.max()),
                 MAX_TOL_PX, int(np.sum(strict_mx > MAX_TOL_PX))))
    print('基准坏解帧（ref > %.0f px 且 new < %.1f×ref）: %d 帧'
          % (REF_HEALTHY_MAX_PX, BAD_IMPROVE_RATIO, len(bad_rows)))
    if len(bad_rows):
        print('  基准 reproj 中位 %.3f px | 新解 reproj 中位 %.3f px | 新解 max %.3f px'
              % (pct(bad_ref, 50), pct(bad_new, 50), float(bad_new.max())))
        n_better = int(np.sum(bad_new <= bad_ref + REPROJ_WORSE_TOL_PX))
        print('  新解 reproj <= 基准: %d / %d 帧' % (n_better, len(bad_rows)))
    print('solve_frame 单帧耗时: p50 %.3f ms | p90 %.3f ms | mean %.3f ms'
          % (pct(ts, 50), pct(ts, 90), float(ts.mean())))
    if failures:
        by_bag = {}
        for stem, _, _ in failures:
            bag = stem.rsplit('_', 1)[0]
            by_bag[bag] = by_bag.get(bag, 0) + 1
        print('失败帧按 bag:')
        for bag, n in sorted(by_bag.items()):
            print('   %s: %d' % (bag, n))
        print('失败帧明细（前 10 条）:')
        for item in failures[:10]:
            print('  ', item)

    assert len(med) == len(stems), '有 %d 帧没有参与比较' % (len(stems) - len(med))
    assert pct(med, 50) <= MEDIAN_TOL_PX, \
        '中位差 %.6f px > %.1f px' % (pct(med, 50), MEDIAN_TOL_PX)
    assert not failures, \
        '严格口径失败 %d 帧（逐帧 max 差 > %.1f px 或新解 reproj 更差）' % (
            len(failures), MAX_TOL_PX)


if __name__ == '__main__':
    test_solve_edges_matches_frame_from_obb()
