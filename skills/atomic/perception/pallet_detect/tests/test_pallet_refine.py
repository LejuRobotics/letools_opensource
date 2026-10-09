"""`pallet_detect.refine` 的合成自检。跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 skills/atomic/perception/pallet_detect/tests/test_pallet_refine.py

**不需要 ROS、不需要相机、不需要任何数据** —— 它自己合成图像、自己造真值。

两组用例：

  A. **几何**（`test_geometry`）—— 把台面刷成亮块、背景压暗，四条边就是四条强梯度线，
     给初值一个人为扰动，看细化能不能解回真值。
     退化情形用**局部高斯模糊**把某条边的对比度压掉（模拟看不清 / 被柔和遮挡）。
     ⚠️ 这里**不用"画一条遮挡带"**来模拟遮挡：遮挡物自己会引入新的边，
     那也是一条梯度线，会被吸附进去 —— 那测的就不是"这条边看不见"了。
  B. **分级更新**（`test_grading`）—— 直接控制哪几条边有观测，检查哪些自由度
     该更新、哪些**必须原样保留初值**。这一组把 `_snap_edge` 换掉，绕开图像，
     只测调度逻辑。

⚠️ 与 maduo 的同名测试（`debug/3test/refine_synthetic_test.py`）逐条相同，
只是换成了相对 import。
"""
from __future__ import annotations

import sys
from pathlib import Path

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
    __package__ = 'skills.atomic.perception.pallet_detect.tests'
    import importlib
    importlib.import_module(__package__)

import cv2
import numpy as np

from .. import refine as rpf
from ..refine import (  # noqa: F401
    _corners_mm, _move, _project_px, refine_pallet_frame,
)

_fails: list[str] = []


def check(name: str, cond: bool, detail: str = '') -> None:
    if cond:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        _fails.append(name)


W_MM, H_MM = 1200.0, 800.0
FX = FY = 400.0
CX, CY = 320.0, 240.0
EDGE_NAMES = ['y=0', 'x=W', 'y=H', 'x=0']


class K:
    fx, fy, cx, cy = FX, FY, CX, CY




def _unit(v):
    return v / np.linalg.norm(v)


def make_truth() -> dict:
    nrm = _unit(np.array([0.05, -0.40, -0.915]))
    e1 = _unit(np.array([1.0, 0.0, 0.0]) - (np.array([1.0, 0.0, 0.0]) @ nrm) * nrm)
    e2 = _unit(np.cross(nrm, e1))
    return dict(origin=np.array([-550.0, -180.0, 1500.0]), E1=e1, E2=e2, nrm=nrm,
                W=W_MM, H=H_MM)


def _band_quad(p0, p1, half):
    """沿线段 p0->p1 的一条带宽 half 的矩形，四个角。"""
    d = p1 - p0
    L = np.hypot(*d)
    u = d / L
    n = np.array([-u[1], u[0]])
    return np.array([p0 - 30 * u + half * n, p1 + 30 * u + half * n,
                     p1 + 30 * u - half * n, p0 - 30 * u - half * n])


def render(truth: dict, blur_edges=(), blur_half=40.0, sigma=12.0, feather=13.0) -> np.ndarray:
    """把台面刷亮、背景压暗；`blur_edges` 里的边用**羽化混合**局部模糊掉。

    ⚠️ 这里必须用羽化 alpha，**不能**写 `img[region>0] = blurred[region>0]`：
    那样会在模糊带的边界上留一条台阶，而**那条台阶本身就是一条强梯度**。
    抠图工具"就近取峰"会稳稳吸到它上面 —— 那就在测一条我自己画出来的假边，
    跟真边分不开（实测：假边直线度 0.26px，比真边还"直"）。
    """
    img = np.full((480, 640, 3), 45, np.uint8)
    q_px = _project_px(_corners_mm(truth, W_MM, H_MM), K)
    cv2.fillPoly(img, [q_px.astype(np.int32)], (205, 205, 205))
    if blur_edges:
        blurred = cv2.GaussianBlur(img, (0, 0), sigmaX=sigma).astype(np.float32)
        alpha = np.zeros(img.shape[:2], np.float32)
        for nm in blur_edges:
            i = EDGE_NAMES.index(nm)
            cv2.fillPoly(alpha, [_band_quad(q_px[i], q_px[(i + 1) % 4],
                                            blur_half).astype(np.int32)], 1.0)
        # 羽化用 `feather`，而带本身要**比羽化宽得多**，否则 alpha 在带中心
        # 都到不了 1，边只被糊掉一半、梯度照样强（实测：那样四条边一条都没糊掉）。
        alpha = cv2.GaussianBlur(alpha, (0, 0), sigmaX=feather)[:, :, None]
        img = (img.astype(np.float32) * (1 - alpha) + blurred * alpha).astype(np.uint8)
    return img


def blank_depth() -> np.ndarray:
    return np.zeros((480, 640), np.float32)


def _errors(refined: dict, truth: dict):
    d = refined['origin'] - truth['origin']
    return (float(d @ truth['E1']), float(d @ truth['E2']),
            float(np.degrees(np.arccos(np.clip(refined['E1'] @ truth['E1'], -1, 1)))))


PERTURB = (35.0, -25.0, 1.6)     # tx mm, ty mm, theta deg
TOL = (3.0, 3.0, 0.15)           # 平移 mm, 转角 deg
FULL = ['t_e1', 't_e2', 'theta']


def test_geometry() -> list[bool]:
    """四条边按对比度逐个压掉，检查退化时的行为。

    ⚠️ 「只剩对边」时**正确的期望不是解回真值**：只看得见一族边，另一族的平移
    和转角在数学上就不可观测（孔径问题），按 §4.2 应当**原样保留初值**。
    所以这里的判据分两半：
      - 被 updated 的分量 -> 必须收敛到真值；
      - 被 held 的分量    -> 必须**恰好等于初值**（一个毫米都不许动）。
    """
    truth = make_truth()
    print(f'真值四角像素:\n{np.round(_project_px(_corners_mm(truth, W_MM, H_MM), K), 1)}')
    print(f'扰动 = tx{PERTURB[0]:+.0f} ty{PERTURB[1]:+.0f} theta{PERTURB[2]:+.1f}deg\n')
    results = []
    for name, blur in (('4 边全可见', ()),
                       ('y=H 压掉（3 边）', ('y=H',)),
                       ('y=0,y=H 压掉（只剩 x 族）', ('y=0', 'y=H')),
                       ('x=W,x=0 压掉（只剩 y 族）', ('x=W', 'x=0'))):
        color = render(truth, blur)
        init = _move(truth, tx=PERTURB[0], ty=PERTURB[1], theta=np.radians(PERTURB[2]))
        init['W'], init['H'] = W_MM, H_MM
        refined, diag = refine_pallet_frame(init, color, blank_depth(), K,
                                            target_mm=(W_MM, H_MM),
                                            opts=dict(plane=False))
        if refined is None:
            print(f'  {name:30s} 拒绝: {diag["reject"]}'); results.append(False); continue
        et, en, eth = _errors(refined, truth)
        upd, held = diag['updated'], diag['held']
        # 被 held 的分量应当**恰好留着初值的扰动**（一个毫米都不许动）
        want_tx = 0.0 if 't_e1' in upd else PERTURB[0]
        want_ty = 0.0 if 't_e2' in upd else PERTURB[1]
        want_th = 0.0 if 'theta' in upd else PERTURB[2]
        held_ok = (abs(et - want_tx) < TOL[0] if 't_e1' in held else True) and \
                  (abs(en - want_ty) < TOL[1] if 't_e2' in held else True) and \
                  (abs(eth - want_th) < TOL[2] if 'theta' in held else True)
        if set(upd) == set(FULL):
            ok = held_ok and abs(et) < TOL[0] and abs(en) < TOL[1] and eth < TOL[2]
            note = '解回真值'
        else:
            # 只有一族边时**不可能**解回真值：另一族的平移在数学上不可观测，
            # 而 θ 按 §4.2 被按住 —— 一条被转过的直线，光靠平移永远对不齐。
            # 所以这里只能断言"确实改善了"：可观测的那一维明显往真值走，
            # 且残差远小于初值。精度上界由被按住的 θ 决定，不是算法出错。
            axis_err = abs(et) if 't_e1' in upd else abs(en)
            axis_pert = abs(PERTURB[0]) if 't_e1' in upd else abs(PERTURB[1])
            ok = held_ok and axis_err < 0.75 * axis_pert and diag['rms_px'] < 2.5
            note = f'只改可观测维：{axis_err:.1f}/{axis_pert:.0f}mm（{axis_err/axis_pert:.0%} 未消）'
        print(f'  {name:30s} {"OK " if ok else "**FAIL**"} updated={"+".join(upd):16s} '
              f'tx={et:+7.2f} ty={en:+7.2f} th={eth:5.3f}  '
              f'rms={diag["rms_px"]:.2f}px obs={diag["n_observations"]}  {note}')
        results.append(ok)
    return results


def _fake_snap(active_names):
    """换掉 `_snap_edge`：只让 active_names 里的边产生观测。"""
    def snap(gx, gy, p0, p1, step_px, half_px, grad_min, mag=None):
        # 靠长度认出是哪条边 —— 合成场景里四条边长度不同
        name = _which_edge(p0, p1)
        if name not in active_names:
            return [], 20
        d = np.asarray(p1, float) - np.asarray(p0, float)
        L = float(np.hypot(*d))
        u = d / L
        n = np.array([-u[1], u[0]])
        out = []
        for t in np.linspace(0.1 * L, 0.9 * L, 20):
            base = np.asarray(p0, float) + t * u
            out.append((base + 1.0 * n, 1.0))    # 恒定的 1px 偏移当作"观测"
        return out, 20
    return snap


_TRUTH_Q = None


def _which_edge(p0, p1):
    d = np.asarray(p1, float) - np.asarray(p0, float)
    for i in range(4):
        t = _TRUTH_Q[(i + 1) % 4] - _TRUTH_Q[i]
        if abs(float(d @ t) / (np.hypot(*d) * np.hypot(*t)) - 1.0) < 1e-3:
            return EDGE_NAMES[i]
    return '?'


def test_grading() -> list[bool]:
    """§4.2：哪些边有观测 -> 哪些自由度被更新、哪些必须原样透传。"""
    global _TRUTH_Q
    truth = make_truth()
    _TRUTH_Q = _project_px(_corners_mm(truth, W_MM, H_MM), K)
    color = render(truth)
    original = rpf._snap_edge
    cases = [
        (['y=0', 'x=W', 'y=H', 'x=0'], FULL, []),
        (['y=0', 'x=W', 'y=H'], FULL, []),               # 3 边
        (['y=0', 'x=W'], FULL, []),                      # 一个角
        (['y=0', 'y=H'], ['t_e2'], ['t_e1', 'theta']),   # 只有沿 E1 的对边
        (['x=W', 'x=0'], ['t_e1'], ['t_e2', 'theta']),   # 只有沿 E2 的对边
        (['y=0'], ['t_e2'], ['t_e1', 'theta']),          # 一条边
        (['x=W'], ['t_e1'], ['t_e2', 'theta']),          # 一条边
        ([], None, None),                                # 0 边 -> 拒绝
    ]
    results = []
    for active, want_upd, want_held in cases:
        rpf._snap_edge = _fake_snap(active)
        try:
            init = _move(truth, tx=PERTURB[0], ty=PERTURB[1],
                         theta=np.radians(PERTURB[2]))
            init['W'], init['H'] = W_MM, H_MM
            refined, diag = refine_pallet_frame(init, color, blank_depth(), K,
                                                target_mm=(W_MM, H_MM),
                                                opts=dict(plane=False))
        finally:
            rpf._snap_edge = original
        label = '+'.join(active) if active else '(0 边)'
        if want_upd is None:
            ok = refined is None and diag['reject'] == 'no_edges'
            print(f'  {label:30s} {"OK " if ok else "**FAIL**"} 拒绝={diag["reject"]}')
            results.append(ok); continue
        ok = (refined is not None and diag['updated'] == want_upd
              and diag['held'] == want_held)
        # 被 held 的自由度必须**一个都没动**：把细化结果投回初值基上看
        moved = []
        if refined is not None:
            d = refined['origin'] - init['origin']
            for ax, nm in ((init['E1'], 't_e1'), (init['E2'], 't_e2')):
                if nm in want_held and abs(float(d @ ax)) > 1e-6:
                    moved.append(nm)
            if 'theta' in want_held and \
                    np.degrees(np.arccos(np.clip(refined['E1'] @ init['E1'], -1, 1))) > 1e-6:
                moved.append('theta')
        if moved:
            ok = False
        print(f'  {label:30s} {"OK " if ok else "**FAIL**"} updated={"+".join(diag["updated"]):14s} '
              f'held={"+".join(diag["held"]) or "-":14s}'
              + (f'  **被held却动了: {moved}**' if moved else ''))
        results.append(ok)
    return results

def main() -> None:
    print('=== A. 几何：初值扰动后能否解回真值 ===')
    a = test_geometry()
    print('\n=== B. 分级更新：该动的动、该保持的纹丝不动 ===')
    b = test_grading()
    for i, ok in enumerate(a):
        check(f'几何 #{i}', bool(ok))
    for i, ok in enumerate(b):
        check(f'分级更新 #{i}', bool(ok))
    total, passed = len(a) + len(b), sum(a) + sum(b)
    print(f'\n{passed}/{total} 通过')
    if _fails:
        print(f'{len(_fails)} 项失败：{", ".join(_fails)}')
        raise SystemExit(1)
    print('全部通过')


if __name__ == '__main__':
    main()
