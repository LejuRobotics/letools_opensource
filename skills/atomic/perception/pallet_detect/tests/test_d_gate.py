# -*- coding: utf-8 -*-
"""**D 闸门**的自检（D 反解 + 容差判据 + 拒绝串）。跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 skills/atomic/perception/pallet_detect/tests/test_d_gate.py

**不需要 ROS、不需要相机、不需要真模型。**

D 是"相机原点到台面、沿台面法向的距离"（毫米），是这条链路里唯一一个
**看图看不出来**的量 —— 所以它只能靠外部先验（板子上的 `pallet_d_mm`）兜住，
超差的帧**不发布**。本文件钉四件事：

1. **反解读得对**：`D_from_frame` 用 `origin · n̂`，且 n̂ **必须翻到 z>0** ——
   不翻会得到**负的 D**（量纲对、符号错）。`frame['nrm']` 是相机俯视时的 z<0 法向
   （`_to_spec` 明写它由 `E1×E2` 定、不单独翻符号），所以这一翻不是防御性代码。
2. **与旧式逐位一致**：重构前这段算法内联在 `user_rule` 里；改成调用
   `dsearch.D_from_frame` 之后，601 帧上必须 `max abs diff == 0.0`
   （有 `obbval` 数据包时才跑，包不在就 SKIP —— 它是搬迁回归，不是日常自检）。
3. **两种口径一致**：`TILT_DEG == 0`（现行硬锁）下，用 `frame['nrm']`
   （`D_from_frame(f)`）与显式传先验法向（`D_from_frame(f, normal)`）数值相同
   —— 但**只到浮点末位**（601 帧实测最大差 4.5e-13mm），因为 `frame['nrm']` 是
   `_to_spec` 用 `cross(E1,E2)` 重算的，与传入的 `normal` 不是同一个浮点数。
   ⚠️ 将来若放开倾斜，这两条会**真的**分离 —— 那时这条用例会红，**是提醒，不是误报**。
4. **容差判据与拒绝串**：`D_dev` 的边界（`ref<=0` → `inf`，语义是"关闸"而不是
   "全部超限"），以及 `format_rejects` 对 `D_out_of_tol` 必须写出
   **实际 D / 参考 D / 偏差 / 容差** 四个量（消息契约：不能只写"失败了"）。
"""
from __future__ import annotations

import sys
from pathlib import Path

if __name__ == '__main__':
    # 相对 import 让 pytest 收集得到、又不污染 sys.path；直接 `python3 <本文件>`
    # 跑时 `__package__` 是空的，所以补上包名再让解释器自己加载包（同仓库约定）。
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
    __package__ = 'skills.atomic.perception.pallet_detect.tests'
    import importlib
    importlib.import_module(__package__)

import numpy as np

from ..dsearch import D_dev, D_from_frame
from ..payload import format_rejects
from . import _synthetic as S

_fails: list[str] = []

# obbval 数据包（maduo 侧）。它**不在** LeTools 仓里，所以只有跑回归的机器上有。
_PACK = Path('/home/lejucv/maduo/test_data/obbval')


def check(name: str, cond: bool, detail: str = '') -> None:
    if cond:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        _fails.append(name)


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# --------------------------------------------------------------------------- #
def test_d_from_frame_reads_the_deck_distance():
    """`origin · n̂`（n̂ 翻到 z>0）就是那个 d，且是**正数**。"""
    fr = S.make_truth()
    n_hat = _unit(fr['nrm'])
    if float(n_hat @ np.array([0., 0., 1.])) < 0:
        n_hat = -n_hat
    expect = float(np.asarray(fr['origin'], float) @ n_hat)
    got = D_from_frame(fr)

    check('合成帧的 D 与手算一致', abs(got - expect) <= 1e-9,
          f'got={got!r} expect={expect!r}')
    check('D 是正数（相机在台面前方）', got > 0.0, f'D={got:.1f}mm')
    check('量级是毫米（1m 上下）', 500.0 < got < 3000.0, f'D={got:.1f}mm')


def test_d_from_frame_flips_the_normal():
    """**不翻符号会算成负的 D** —— 这一翻是必需项，不是防御性代码。

    合成帧的 `nrm` 是"指向相机"的台面外法向（z<0），与现场帧同构。
    """
    fr = S.make_truth()
    n_raw = _unit(fr['nrm'])
    naive = float(np.asarray(fr['origin'], float) @ n_raw)

    check('前提：该帧的 nrm 真的是 z<0（相机俯视）',
          float(n_raw[2]) < 0.0, f'nrm.z={float(n_raw[2]):.4f}')
    check('不翻符号确实得到负的 D（这就是坑）', naive < 0.0, f'{naive:.1f}mm')
    check('D_from_frame 翻回来了，与 -naive 一致',
          abs(D_from_frame(fr) + naive) <= 1e-9,
          f'D={D_from_frame(fr):.4f} -naive={-naive:.4f}')


def test_d_dev_boundary():
    """`D_dev` = 相对偏差；`ref<=0` 返回 `inf`（**调用方据此关闸**，不是判超限）。"""
    check('完全一致 → 0', D_dev(890.0, 890.0) == 0.0)
    check('高 10% → 0.1', abs(D_dev(979.0, 890.0) - 0.1) < 1e-12,
          f'{D_dev(979.0, 890.0)!r}')
    check('低 10% → 0.1', abs(D_dev(801.0, 890.0) - 0.1) < 1e-12,
          f'{D_dev(801.0, 890.0)!r}')
    check('偏差是双向的（只看绝对值）', D_dev(700.0, 890.0) > 0.2)
    check('ref=0 → inf（= 关闸信号）', D_dev(890.0, 0.0) == float('inf'))
    check('ref<0 → inf（同上）', D_dev(890.0, -5.0) == float('inf'))

    # 节点里的判据就是 `D_dev(D, ref) > tol`：边界是**严格大于**。
    # ⚠️ 别拿"正好 10%"当边界样本 —— `979.0/890.0` 在浮点上略大于 1.1
    # （`dev = 0.10000000000000009`），那样断言的是浮点表示的巧合，不是判据。
    tol = 0.1
    check('判据：与参考值相等不拒（严格大于）', not (D_dev(890.0, 890.0) > tol))
    check('判据：10.1% 拒', D_dev(980.0, 890.0) > tol,
          f'{D_dev(980.0, 890.0):.4f}')


def test_format_rejects_carries_values_and_thresholds():
    """`D_out_of_tol` 的拒绝串必须带**实际 D / 参考 D / 偏差 / 容差**。

    消息契约（`PalletDetection.msg`）要求"必须写实际值与阈值，不能只写失败"：
    只写"超限"的话，排查时分不清是 **D 算歪了** 还是 **参考值给错了**。
    """
    s = format_rejects({'reject': 'D_out_of_tol', 'D_mm': 980.0,
                        'D_ref_mm': 890.0, 'D_tol': 0.1})
    check('以 reject=D_out_of_tol 开头', s.startswith('reject=D_out_of_tol'), s)
    check('带实际 D', 'D=980.0mm' in s, s)
    check('带参考 D', 'ref=890.0mm' in s, s)
    check('带偏差百分比', 'dev=10.1%' in s, s)
    check('带容差', 'tol=10.0%' in s, s)
    check('没有失败时仍返回空串（老契约不变）',
          format_rejects({'reject': None}) == '')


def test_d_from_frame_matches_legacy_inline_on_obbval():
    """★ 601 帧上与**重构前的内联式**逐位一致（有数据包才跑）。

    重构前这段算法写在 `user_rule.frame_user_rule` 里；现在它调用
    `dsearch.D_from_frame`。这条网钉住"搬走了、但数没变"。

    ⚠️ 传进去的必须是**原始 `normal`**：`D_from_frame` 内部自己做
    「`asarray` → 除以 `norm` → 判符号」。传已经归一化过的向量会**多除一次**，
    末位可能差 1 ulp —— 那不是 bug，但会让"逐位一致"这条网失去意义。
    """
    if not (_PACK / 'valpack.py').exists():
        print('  SKIP  没有 obbval 数据包，跳过逐位一致回归')
        return
    sys.path.insert(0, str(_PACK))
    import valpack as VP                                            # noqa: E402
    from .. import solve_obb as SO                                  # noqa: E402
    from ..geom import CameraIntrinsics                             # noqa: E402

    worst = 0.0
    worst_nrm = 0.0
    n = 0
    for s in VP.stems():
        fr = VP.load(s)
        obb = VP.obb(s)
        if obb is None:
            continue
        k = CameraIntrinsics(*fr['K'])
        normal = fr['nrm']
        f, _ = SO.frame_from_obb(obb, k, normal, depth=fr['depth'])
        if f is None:
            continue
        n += 1
        # 重构前的内联式，逐字照抄
        n0 = np.asarray(normal, float); n0 = n0 / np.linalg.norm(n0)
        if float(n0 @ np.array([0., 0., 1.])) < 0:
            n0 = -n0
        legacy = float(np.asarray(f['origin'], float) @ n0)

        worst = max(worst, abs(legacy - D_from_frame(f, normal)))
        worst_nrm = max(worst_nrm, abs(legacy - D_from_frame(f)))

    check(f'601 帧与旧内联式**逐位**一致（{n} 帧）',
          n == 601 and worst == 0.0, f'n={n} max={worst:.3e}')
    # ⚠️ 默认口径（用 `frame['nrm']`）**不是逐位**相同的，只到浮点末位：
    # `frame['nrm']` 是 `_to_spec` 用 `cross(E1, E2)` **重算**出来的向量，与传进来的
    # `normal` 不是同一个浮点数。实测 601 帧最大差 4.5e-13mm（相对 ~3e-16）。
    # 对 10% 的闸门完全无影响，但**别把它写成 `== 0.0`** —— 那是在断言浮点巧合。
    check(f'用 frame["nrm"] 的默认口径一致（{n} 帧，到浮点末位 1e-6mm）',
          n == 601 and worst_nrm < 1e-6, f'n={n} max={worst_nrm:.3e}')


def main() -> int:
    print('== D 反解读得对 ==')
    test_d_from_frame_reads_the_deck_distance()
    print('== 法向必须翻到 z>0 ==')
    test_d_from_frame_flips_the_normal()
    print('== D_dev 边界 ==')
    test_d_dev_boundary()
    print('== 拒绝串带实际值与阈值 ==')
    test_format_rejects_carries_values_and_thresholds()
    print('== 与重构前的内联式逐位一致（有数据包才跑）==')
    test_d_from_frame_matches_legacy_inline_on_obbval()
    print()
    if _fails:
        print(f'{len(_fails)} 项失败：{", ".join(_fails)}')
        return 1
    print('全部通过')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
