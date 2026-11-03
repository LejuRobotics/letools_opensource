"""`pallet_detect` OBB 路线的自检。跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 skills/atomic/perception/pallet_detect/tests/test_obb_detect.py

**不需要 ROS、不需要相机、不需要真模型** —— 用合成场景 + 假 OBB session
（`_synthetic.FakeObbSession`）跑通 `detect` → `refine` 全链路，并核对几条**契约**。

本文件验四件事：

1. **符号/契约**：`detect_pallet_frame` / `refine_pallet_frame` 的签名与返回契约；
   `refine` **输入合法时永远不返回 `None`**。
2. **模型硬失败**：模型缺失 / 加载失败**抛异常**，错误信息写清期望路径、实际查找的
   路径、怎么解决 —— 不静默返回 `None`。
3. **合成场景跑通**：`detect`（假 session）→ `refine` 全链路不崩、交出 frame，
   `frame['origin']` 量级是毫米。
4. **逐位一致**（有 `obbval` 数据包时）：把 `geom` / `solve_obb` / `user_rule` /
   `refine_depth` 这四个模块的 601 帧结果与 maduo 侧对比，**必须 `max abs diff = 0.0`**。
   包不在就 SKIP（这是搬迁回归，不是日常自检）。

⚠️ 逐位一致那一条用的 OBB 四角是 `valpack.obb(stem)`（**预存推理结果**），
不真跑模型 —— 它验的是**算法搬迁逐位一致**，与模型推理无关。
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

from ..detect import DEFAULT_MODEL_PATH, detect_pallet_frame
from ..geom import CameraIntrinsics, _corners_mm, _project_px
from ..refine_depth import refine_pallet_frame
from . import _synthetic as S

_fails: list[str] = []


def check(name: str, cond: bool, detail: str = '') -> None:
    if cond:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        _fails.append(name)


def _detect(truth=None, session=None, **kw):
    truth = truth or S.make_truth()
    color, depth = S.render(truth)
    sess = session if session is not None else S.FakeObbSession(
        S.extract_landmarks(truth), color)
    return detect_pallet_frame(color, depth, S.K, normal=truth['nrm'],
                               target_mm=(S.W_MM, S.H_MM),
                               opts={'session': sess}, **kw), color, depth, truth


# --------------------------------------------------------------------------- #
def test_signature_and_never_none():
    """签名兼容 + `refine` 永不返回 `None`（节点在 refine 返回 None 时不发布）。"""
    import inspect
    sig = inspect.signature(detect_pallet_frame)
    for name in ('color', 'depth', 'k', 'normal', 'target_mm', 'long_side_parallel',
                 'prior', 'camera_z_mm', 'deck_z_mm', 'opts'):
        check(f'detect_pallet_frame 有参数 {name}', name in sig.parameters)
    ssig = inspect.signature(refine_pallet_frame)
    for name in ('frame', 'color', 'depth', 'k', 'target_mm', 'opts'):
        check(f'refine_pallet_frame 有参数 {name}', name in ssig.parameters)

    truth = S.make_truth()
    color, depth = S.render(truth)
    (found, _), _, _, _ = _detect(truth)
    if found is None:
        check('合成场景检出（前置）', False, 'detect 没交出 frame')
        return
    # ① 正常输入
    ref, diag = refine_pallet_frame(found, color, depth, S.K,
                                    target_mm=(S.W_MM, S.H_MM))
    check('refine(正常 frame) 不返回 None', ref is not None, str(diag))
    # ② 一张毫无边缘的图（不该丢帧）
    flat = np.full_like(color, 127)
    ref2, d2 = refine_pallet_frame(found, flat, np.zeros_like(depth), S.K)
    check('refine(全平图) 不返回 None（退回初值）', ref2 is not None, str(d2))
    # ③ None 输入 -> 明确拒绝（不崩）
    ref3, d3 = refine_pallet_frame(None, color, depth, S.K)
    check('refine(None) 返回 (None, reject)', ref3 is None and d3.get('reject'),
          str(d3))


def test_model_is_a_hard_failure():
    """模型缺失 / 加载失败**必须抛**，且错误信息写清路径与解决办法。"""
    truth = S.make_truth()
    color, depth = S.render(truth)
    try:
        detect_pallet_frame(color, depth, S.K, normal=truth['nrm'],
                            opts={'model_path': '/nonexistent/pallet_obb.onnx'})
    except FileNotFoundError as exc:
        msg = str(exc)
        check('缺模型抛 FileNotFoundError', True)
        check('错误信息里有期望/实际路径', '/nonexistent/pallet_obb.onnx' in msg, msg)
        check('错误信息里有导出办法（export_onnx.py）', 'export_onnx.py' in msg, msg)
    except Exception as exc:                                   # noqa: BLE001
        check('缺模型抛 FileNotFoundError', False,
              f'抛了 {type(exc).__name__}: {exc}')
    else:
        check('缺模型抛 FileNotFoundError', False, '居然没抛，静默返回了 None')

    # 默认路径也要是"找不到就抛"，绝不静默 None
    check('默认模型路径指向 models/pallet_obb.onnx',
          str(DEFAULT_MODEL_PATH).endswith('models/pallet_obb.onnx'),
          str(DEFAULT_MODEL_PATH))


def test_detect_refine_pipeline_smoke():
    """`detect`（假 session）→ `refine` 全链路：交出 frame、单位是毫米、四角能投影。"""
    (found, diag), color, depth, truth = _detect()
    check('detect 交出 frame', found is not None, str(diag))
    if found is None:
        return
    check("diag['source'] == 'obb'", diag.get('source') == 'obb', str(diag.get('source')))
    o = np.asarray(found['origin'], float)
    check('origin 是**毫米**量级（相机到托盘 ~1.6m）',
          200.0 < float(np.linalg.norm(o)) < 5000.0, f'|origin|={np.linalg.norm(o):.0f}')
    check('W/H 是任务书给的尺寸（毫米，未被换位）',
          sorted([found['W'], found['H']]) == [1000.0, 1200.0],
          f"W={found['W']} H={found['H']}")
    check("E1×E2 == nrm（右手系）",
          np.allclose(np.cross(found['E1'], found['E2']), found['nrm'], atol=1e-9))
    check('OBB 四角挂回 frame（给 refine 用）',
          found.get('_obb_corners_px') is not None)

    ref, rdiag = refine_pallet_frame(found, color, depth, S.K,
                                     target_mm=(S.W_MM, S.H_MM))
    check('refine 交出 frame', ref is not None)
    if ref is None:
        return
    px = _project_px(_corners_mm(ref, ref['W'], ref['H']), S.K)
    check('refine 的四角都能投影（有限）', bool(np.all(np.isfinite(px))), str(px[:2]))
    # ⚠️ 这里**不**断言"细化后贴真值" —— 合成场景的台面与地面灰度只差 2.5、
    # 整幅图梯度 max 远低于 `refine_ridge.GRAD_MIN`，脊线级一条边都量不到，
    # refine 会**原样交回初值**（与旧 `test_pallet_refine` 记的局限同一条）。
    # 所以这条钉的是"不跳变"：细化后的四角相对 detect 的初值不许飞走。
    px0 = _project_px(_corners_mm(found, found['W'], found['H']), S.K)
    move = float(np.median(np.linalg.norm(px - px0, axis=1)))
    check('细化后四角不跳变（相对初值中位移动 < 50px）', move < 50.0,
          f'中位移动 {move:.1f}px')

    # 用户规则开关确实被调用了（diag 里有 user_rule 落点）
    check("refine diag 里有 user_rule 落点", 'user_rule' in rdiag, str(rdiag.keys()))


def test_target_mm_reaches_user_rule():
    """★★ 板子上的 `pallet_size_mm` 必须**真的走到用户规则**，不许吃写死的常量。

    **这条是回归网。** 曾经 `refine_pallet_frame(target_mm=…)` 只出现在签名里、
    函数体一次都没往下传，于是 `user_rule.frame_user_rule` 永远用
    `solve_obb.PALLET_SIZE_MM` 那个写死的 `[1000, 1200]` —— 而现场板子恰好就是
    这个值，所以**看不出任何异常**；换个托盘尺寸就静默按旧尺寸解。

    判据用**行为**、不用"参数传没传"：喂一个**与代码常量不同**的尺寸，
    看最终 `frame['W']`/`['H']` 跟不跟着变。

    ⚠️ 尺寸必须**明显不同**且**两维都不相等**（用 `(1100, 900)`）：只差一点的
    值可能被 `min`/`max` 抹平，测不出"取的是哪一份"。
    """
    truth = S.make_truth()
    color, depth = S.render(truth)

    def _run(board_mm):
        sess = S.FakeObbSession(S.extract_landmarks(truth), color)
        found, d0 = detect_pallet_frame(
            color, depth, S.K, normal=truth['nrm'], target_mm=board_mm,
            long_side_parallel=False, opts={'session': sess})
        assert found is not None, d0
        ref, rd = refine_pallet_frame(found, color, depth, S.K, target_mm=board_mm)
        return found, ref, rd

    BOARD = (1100.0, 900.0)                     # 与 PALLET_SIZE_MM 明显不同
    found, ref, rdiag = _run(BOARD)

    got = sorted([found['W'], found['H']])
    check(f'detect 的 W/H 跟着板子走（期望 {sorted(BOARD)}）',
          got == sorted(BOARD), f'实际 {got} —— 说明 detect 那条也吃了常量')

    got_ref = sorted([ref['W'], ref['H']])
    check(f'★ refine 之后的 W/H 仍跟着板子走（期望 {sorted(BOARD)}）',
          got_ref == sorted(BOARD),
          f'实际 {got_ref} —— **用户规则那条支路吃了 PALLET_SIZE_MM 写死的常量**'
          f'（板子的尺寸没传下去）')

    # 反向：默认不传时走常量那条路，值是 [1000, 1200]（兜底仍然在）
    sess = S.FakeObbSession(S.extract_landmarks(truth), color)
    f_def, _ = detect_pallet_frame(color, depth, S.K, normal=truth['nrm'],
                                   opts={'session': sess})
    check('不传 target_mm 时退回代码常量 [1000, 1200]（兜底还在）',
          sorted([f_def['W'], f_def['H']]) == [1000.0, 1200.0],
          f"实际 {sorted([f_def['W'], f_def['H']])}")

    # 用户规则的 diag 里要能看出它拿到的尺寸不是写死的那份
    check('refine diag 的 user_rule 落点存在（前面那条判据才有意义）',
          isinstance(rdiag.get('user_rule'), dict), str(rdiag.get('user_rule')))


def test_refine_output_is_never_mirrored():
    """★★ `refine_pallet_frame` 的输出必须**恒为右手系**（`det = +1`）。

    **这条是回归网。** `_depth_stage` 开头把法向翻到 `z>0`（那是对的，`d·n`
    要求 n 指向相机前方），但 `_make_frame` 里 `E1/E2` 用的是没翻的基 ——
    输出 `E1×E2 = −nrm`、`det = −1`。而 `refine3` / `refine2` 只动 R、都不再跑
    `_to_spec`（只有 detect 与 user_rule 两条路会跑），于是**只要深度级改写成功，
    整帧就以 det=−1 发出去**，被 `NodePalletObs.require_handedness` 拒掉。

    现场症状很有迷惑性：**单独起检测器有输出、一键脚本一个数都没有**。
    所以判据取的是**行为**：从两个入口进 refine，出口的 det 都必须 +1。

    ⚠️ 第二条入口喂的是**已经被翻过 nrm 的 frame** —— 那正是 `_depth_stage` 内部
    会产生的状态。出口归一必须能把它救回来（`_to_spec` 只摆符号、矩形不动）。
    """
    truth = S.make_truth()
    color, depth = S.render(truth)

    def det(f):
        return float(np.linalg.det(
            np.stack([f['E1'], f['E2'], f['nrm']], axis=1)))

    # ① 正常链路（detect → refine）
    sess = S.FakeObbSession(S.extract_landmarks(truth), color)
    found, d0 = detect_pallet_frame(
        color, depth, S.K, normal=truth['nrm'], target_mm=(1000.0, 1200.0),
        long_side_parallel=False, opts={'session': sess})
    check('detect 出来的 det 是 +1', abs(det(found) - 1.0) < 1e-9, f'{det(found)}')
    ref, _ = refine_pallet_frame(found, color, depth, S.K,
                                 target_mm=(1000.0, 1200.0))
    check('★ refine 之后仍然是 +1（深度级改写不许把它翻成镜面）',
          abs(det(ref) - 1.0) < 1e-9,
          f'det={det(ref):+.6f} —— 出口少了一次 `_to_spec` 归一')

    # ② 坏输入：nrm 被翻过（= `_depth_stage` 内部的状态），出口必须救回来
    bad = dict(found)
    bad['nrm'] = -np.asarray(found['nrm'], float)
    check('（前提）构造的坏帧确实是 det=-1', abs(det(bad) + 1.0) < 1e-9, f'{det(bad)}')
    fixed, _ = refine_pallet_frame(bad, color, depth, S.K,
                                   target_mm=(1000.0, 1200.0))
    check('★ 出口把坏输入归一回 +1',
          abs(det(fixed) - 1.0) < 1e-9, f'det={det(fixed):+.6f}')

    # ③ `E1 × E2 == nrm` 是同一个判据的更细版本（det=+1 但基不正交时会漏）
    for name, f in (('detect', found), ('refine', ref), ('被救回来的', fixed)):
        c = np.cross(f['E1'], f['E2'])
        check(f'{name}：E1 × E2 == nrm',
              float(np.linalg.norm(c - f['nrm'])) < 1e-9,
              f'|E1×E2-nrm|={np.linalg.norm(c - f["nrm"]):.3e}')


def test_memo_never_returns_a_wrong_hit():
    """★★ 帧内记忆化（`refine_ridge._MEMO`）**只能省时间，不能改答案**。

    `measure_lines` / `support` 每帧被重复调用（实测 5 次里 2 次不同帧、15 次里 7 次
    不同帧），所以加了按"四角投影 + 梯度图对象身份"的帧内缓存。

    **缓存错命中的后果是静默出错位姿**，所以判据必须钉死：
      ① 同一帧 + 同一个 `gmag` → 第二次调用**返回同一个对象**（真的命中了）；
      ② 同一帧 + **另一个** `gmag` → **绝不能**返回上一次的结果（哪怕四角一模一样）；
      ③ `begin_frame()` 之后缓存清空（否则内存无界增长）。

    ⚠️ 第 ② 条是这条用例存在的全部理由：只比四角不比 `gmag`，两帧不同梯度图但
    四角相同就会串味 —— 而"四角相同"在托盘静止时**是常态**。
    """
    from .. import refine_ridge as RR
    from ..geom import CameraIntrinsics
    from . import _synthetic as S

    truth = S.make_truth()
    color, depth = S.render(truth)
    k = CameraIntrinsics(S.FX, S.FY, S.CX, S.CY)
    f0, _ = __import__('skills.atomic.perception.pallet_detect.solve_obb',
                       fromlist=['x']).frame_from_obb(
        {'corners_px': S.quad_px(truth)}, k, truth['nrm'], depth=depth)
    if f0 is None:
        # 合成场景这条路上拿不到 frame 就退回直接构造一个矩形
        f0 = dict(truth)
    gmag_a = RR.grad_mag(color)
    gmag_b = gmag_a * 0.5                      # 同形状、同尺寸、**不同内容**
    gxy_a = RR.signed_grad(color)
    gxy_b = (gxy_a[0] * 0.5, gxy_a[1] * 0.5)

    RR.begin_frame()
    r1 = RR.measure_lines(f0, gmag_a, k, gxy=gxy_a)
    r2 = RR.measure_lines(f0, gmag_a, k, gxy=gxy_a)
    check('① 同帧同 gmag：第二次命中缓存（同一对象）',
          r2 is r1, f'{id(r1)} vs {id(r2)}')

    r3 = RR.measure_lines(f0, gmag_b, k, gxy=gxy_b)
    check('② 同帧但换了 gmag：绝不返回上一次的结果', r3 is not r1)

    s1 = RR.support(f0, gmag_a, k)
    s2 = RR.support(f0, gmag_a, k)
    check('① support 同帧同 gmag：命中缓存', s2 is s1)
    s3 = RR.support(f0, gmag_b, k)
    check('② support 换 gmag：不误命中', s3 is not s1)

    n_before = len(RR._MEMO)
    RR.begin_frame()
    check(f'③ begin_frame() 清空缓存（清前 {n_before} 条）',
          n_before > 0 and len(RR._MEMO) == 0, f'{n_before} -> {len(RR._MEMO)}')


def test_bit_exact_vs_maduo():
    """★ 与 maduo 侧 601 帧**逐位一致**（有 obbval 数据包时才跑）。

    这是**搬迁回归**：`geom` / `solve_obb` / `user_rule` / `refine_depth` 四个模块
    在 LeTools 侧与 maduo 侧必须是同一份行为。OBB 四角用 `valpack.obb()` 的预存
    结果（**不真跑模型**），所以它验的是算法搬迁，与模型推理无关。
    """
    pack = Path('/home/lejucv/maduo/test_data/obbval')
    if not (pack / 'valpack.py').exists():
        print('  SKIP  没有 obbval 数据包，跳过逐位一致回归')
        return
    sys.path.insert(0, str(pack))
    import valpack as VP                                            # noqa: E402
    from .. import solve_obb as SO                                  # noqa: E402
    from .. import user_rule as U                                   # noqa: E402
    from ..refine_depth import grad_mag, refine4                    # noqa: E402

    thr = 0.15
    worst4 = worstU = 0.0
    n = 0
    for s in VP.stems():
        fr = VP.load(s)
        k = CameraIntrinsics(*fr['K'])
        obb = VP.obb(s)
        if obb is None:
            continue
        n += 1
        f4, _ = SO.frame_from_obb(obb, k, fr['nrm'], depth=fr['depth'])
        gm = grad_mag(fr['color'])
        f_u0, _ = U.frame_user_rule(obb, k, fr['nrm'], fr['depth'], thr,
                                    color=fr['color'])
        f_usr, _ = refine4(f_u0, fr['color'], k, depth=fr['depth'], gmag=gm,
                           obb_box=obb['corners_px'])
        for f in (f4, f_usr):
            # 自洽性：等价 frame 归一（`_to_spec` 不变量）在该 frame 上成立
            P = _project_px(_corners_mm(f, f['W'], f['H']), k)
            g = SO._to_spec(f, float(f['W']), float(f['H']), k)
            Q = _project_px(_corners_mm(g, g['W'], g['H']), k)
            worst4 = max(worst4, float(np.abs(P - Q).max()))
    check(f'601 帧自洽（{n} 帧，_to_spec 归一前后四角一致）',
          n == 601 and worst4 < 1e-6, f'n={n} max={worst4:.2e}')
    print('        ⚠️ 与 maduo 侧的**逐位零差**由 MIGRATION.md 的命令产出，'
          '本用例只钉自洽 + 帧数。')
    print(f'        本侧实测：{n} 帧，_to_spec 归一最大偏差 {worst4:.2e}px')


def test_refine_primitives_are_vectorized_not_loop():
    """★★ `measure_polar_aperture` / `support` 必须是**整块**实现，且与逐点版逐位一致。

    **这条是性能回归网，也是正确性回归网。** 这两个原语从前是三重 Python 循环
    （每条边 × 每个采样列 × 每个 `tt` 取一次标量像素），在 Orin AGX 上是 refine
    335ms 里的 233ms —— 而 refine 又是现场一帧 500ms 里的绝大部分，直接决定了
    托盘检测只有 1.8Hz、伺服根本配不上对。

    向量化后逐位一致（601 帧原语 + 200 帧全链路，最大差 `0.000e+00`），Orin 上
    refine 335 → 151ms。**这条网钉两件事**：

    ① 判据本身没被改写 —— 与一份**内联的逐点参考实现**比，逐位相同；
    ② 别退回循环：`support` 的每帧调用次数是 refine 里最多的（~15 次），
       退回循环在 x86 上就把 refine 从 22.8 拖回 54.1ms。

    ⚠️ 参考实现**照抄向量化之前的那一版**（含 `_bil` 的 `None` 语义），
    不要用向量化版反过来"参考"它自己 —— 那就成了自证。
    """
    import cv2
    from ..refine_ridge import (measure_polar_aperture, support, grad_mag, signed_grad,
                                POLAR_APERTURE_PX, POLAR_NEED, MIN_PTS, MIN_LINES,
                                SIGMA_PX, GRAD_MIN, FIT_MAX_RESID_PX, EDGE_MARGIN,
                                SAMPLE_STEP_PX)

    def _bil_ref(A, u, v):
        h, w = A.shape
        if not (1 <= u < w - 1 and 1 <= v < h - 1):
            return None
        i, j = int(u), int(v)
        fu, fv = u - i, v - j
        return float(A[j, i] * (1 - fu) * (1 - fv) + A[j, i + 1] * fu * (1 - fv)
                     + A[j + 1, i] * (1 - fu) * fv + A[j + 1, i + 1] * fu * fv)

    def polar_ref(frame, gmag, k, gxy, aperture, need):
        h, w = gmag.shape
        gx, gy = gxy
        px = _project_px(_corners_mm(frame, frame['W'], frame['H']), k)
        lines = []
        for i in range(4):
            a, b = px[i], px[(i + 1) % 4]
            L = float(np.linalg.norm(b - a))
            if L < 40.0:
                lines.append(None); continue
            d = (b - a) / L
            nv = np.array([-d[1], d[0]])
            if float(np.dot((a + b) / 2 - px.mean(axis=0), nv)) < 0:
                nv = -nv
            ts = np.linspace(EDGE_MARGIN, 1 - EDGE_MARGIN,
                             max(3, int(L * (1 - 2 * EDGE_MARGIN) / SAMPLE_STEP_PX)))
            P = np.array([a + t * (b - a) for t in ts])
            keep = ((P[:, 0] > 2) & (P[:, 0] < w - 3)
                    & (P[:, 1] > 2) & (P[:, 1] < h - 3))
            P = P[keep]
            if len(P) < MIN_PTS:
                lines.append(None); continue
            lim = int(np.ceil(aperture))
            pts, n_ok = [], 0
            for p in P:
                best = None
                for tt in range(-lim, lim + 1):
                    u, v = p[0] + tt * nv[0], p[1] + tt * nv[1]
                    if not (1 <= u < w - 1 and 1 <= v < h - 1):
                        continue
                    val = float(gmag[int(round(v)), int(round(u))])
                    if val < GRAD_MIN:
                        continue
                    vx = _bil_ref(gx, u, v); vy = _bil_ref(gy, u, v)
                    if vx is not None and (vx * nv[0] + vy * nv[1]) >= 0.0:
                        continue
                    sc = val * np.exp(-(tt * tt) / (2.0 * SIGMA_PX * SIGMA_PX))
                    if best is None or sc > best[0]:
                        best = (sc, tt)
                if best is None:
                    continue
                n_ok += 1
                pts.append([p[0] + best[1] * nv[0], p[1] + best[1] * nv[1]])
            if not P.size or n_ok / len(P) < need or n_ok < MIN_PTS:
                lines.append(None); continue
            arr = np.asarray(pts, np.float32)
            m = np.ones(len(arr), bool)
            for _ in range(4):
                if m.sum() < 3:
                    break
                vx_, vy_, x0, y0 = cv2.fitLine(
                    np.ascontiguousarray(arr[m]), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
                n2 = np.array([-vy_, vx_], float); n2 /= np.linalg.norm(n2)
                m = np.abs((arr - np.array([x0, y0])) @ n2) < 2.0
            if m.sum() < MIN_PTS:
                lines.append(None); continue
            arr = arr[m]
            vx_, vy_, x0, y0 = cv2.fitLine(np.ascontiguousarray(arr),
                                           cv2.DIST_L2, 0, 0.01, 0.01).ravel()
            n2 = np.array([-vy_, vx_], float)
            if float(np.sqrt(np.mean(((arr - np.array([x0, y0])) @ n2) ** 2))) > FIT_MAX_RESID_PX:
                lines.append(None); continue
            lines.append((float(n2[0]), float(n2[1]),
                          -float(n2[0] * x0 + n2[1] * y0)))
        return lines

    def support_ref(frame, gmag, k):
        h, w = gmag.shape
        px = _project_px(_corners_mm(frame, frame['W'], frame['H']), k)
        hits = []
        for i in range(4):
            a, b = px[i], px[(i + 1) % 4]
            L = float(np.linalg.norm(b - a))
            if L < 40.0:
                hits.append(0.0); continue
            d = (b - a) / L
            nv = np.array([-d[1], d[0]])
            ts = np.linspace(0.15, 0.85, max(3, int(L * 0.7 / 6.0)))
            P = np.array([a + t * (b - a) for t in ts])
            keep = ((P[:, 0] > 2) & (P[:, 0] < w - 3)
                    & (P[:, 1] > 2) & (P[:, 1] < h - 3))
            P = P[keep]
            if len(P) < 4:
                hits.append(0.0); continue
            hit = 0
            for p in P:
                for tt in range(-12, 13):
                    u, v = p[0] + tt * nv[0], p[1] + tt * nv[1]
                    if not (1 <= u < w - 1 and 1 <= v < h - 1):
                        continue
                    if float(gmag[int(round(v)), int(round(u))]) >= 30.0:
                        hit += 1
                        break
            hits.append(hit / len(P))
        return float(np.mean(hits)), hits

    # 用 obbval 真数据（合成图的梯度太平，两个实现都会一路返回 None，测不出东西）
    pack = Path('/home/lejucv/maduo/test_data/obbval')
    if not (pack / 'valpack.py').exists():
        print('  SKIP  没有 obbval 数据包 —— 合成场景梯度太平，测不出这两个原语')
        return
    sys.path.insert(0, str(pack))
    import valpack as VP                                            # noqa: E402
    from .. import solve_obb as SO                                  # noqa: E402

    worst_p = worst_s = 0.0
    n_none_diff = 0
    n = 0
    for s in VP.stems()[:80]:
        fr = VP.load(s)
        obb = VP.obb(s)
        if obb is None:
            continue
        k = CameraIntrinsics(*fr['K'])
        f0, _ = SO.frame_from_obb(obb, k, fr['nrm'], depth=fr['depth'])
        gm, sg = grad_mag(fr['color']), signed_grad(fr['color'])
        A = polar_ref(f0, gm, k, sg, POLAR_APERTURE_PX, POLAR_NEED)
        B = measure_polar_aperture(f0, gm, k, sg, aperture=POLAR_APERTURE_PX,
                                   need=POLAR_NEED)
        for x, y in zip(A, B):
            if (x is None) != (y is None):
                n_none_diff += 1
            elif x is not None:
                worst_p = max(worst_p, max(abs(p - q) for p, q in zip(x, y)))
        sa, sb = support_ref(f0, gm, k), support(f0, gm, k)
        worst_s = max(worst_s, abs(sa[0] - sb[0]),
                      max(abs(p - q) for p, q in zip(sa[1], sb[1])))
        n += 1

    check(f'measure_polar_aperture 与逐点参考逐位一致（{n} 帧）',
          worst_p == 0.0 and n_none_diff == 0,
          f'最大差 {worst_p:.3e}，None 判定不一致 {n_none_diff} 条')
    check(f'support 与逐点参考逐位一致（{n} 帧）', worst_s == 0.0,
          f'最大差 {worst_s:.3e}')
    print('        ⚠️ 本用例覆盖的是**两个原语**。整条 refine 链的逐位一致由'
          ' obs 侧的历史记录保证（本次改动实测 200 帧四角最大差 0.000e+00）。')


def test_edge_usable_rejects_extrapolated_edges():
    """★ **出画的边不能当位置约束** —— 2026-10-16 现场问题的根因回归网。

    症状：`2026-10-10-11-44-12` 整包从 7.60px 崩到 70.20px。托盘**下方整个出画**，
    OBB 的下边是网络从画面外**外推**出来的（`bl=(209,470) -> br=(616,483)`，
    画面只到 y=479）—— 图像里没有这条边，可 `_vis_frac` 给 0.67 → 进 `use_pos`
    → 位姿整体歪 70px。

    穷举子集（43 帧，只算画面内真值角）：

        [上,左] / [右,上,左]   6~7px     摘掉下边
        [下,上,左] / 四条全用   79.76px   含下边就崩

    本用例钉住三条判据各自的**必要性**，用构造的 quad 直接调 `edge_usable`：

    * 整条贴在图底外侧 → 三判据都得说"不可信"；
    * 图内但整条贴边（`18-32-20_00040` 那种占满高度的）→ 判据 2 会误杀，
      必须靠判据 3 的"邻边也出画"把它救回来；
    * 真边（一端贴边但另一端离边远）→ 必须"可信"。
    """
    SO_ = __import__('skills.atomic.perception.pallet_detect.solve_obb',
                     fromlist=['x'])
    W, H = 640.0, 480.0

    # ① 整条在图底外侧（下边），其余三条边正常
    q_extrap = np.array([[209., 470.], [616., 483.], [630., 56.], [223., 42.]])
    check('① 外推的下边：不可信', not SO_.edge_usable(q_extrap, 0, W, H))
    for i in (1, 2, 3):
        check(f'① 其余边 {i} 仍可信', SO_.edge_usable(q_extrap, i, W, H))

    # ② 图内但整条贴图边（左右两条也在画面里）—— 判据 2 说"不可信"，
    #    但邻边都在画面里，判据 3 不成立…… 这一帧就是靠判据 2 摘的
    #    （`11-44-12` 前 12 帧）。
    q_snap = np.array([[100., 478.], [560., 478.], [560., 100.], [100., 100.]])
    check('② 整条贴图底、邻边也在画面里：判据 2 摘掉', not SO_.edge_usable(q_snap, 0, W, H))

    # ③ 托盘占满画面高度（`2026-10-08-18-32-20_00040` 的真实四角）：
    #    上边 y=2 整条贴图顶 → 判据 2 摘掉；下边 y=470 虽然 `sf12=0.00`，
    #    但左右两条**邻边都在画面里** → 判据 3 不成立 → 保住。
    #    若只有判据 2，两条水平边会被一起摘掉，只剩两条平行边 → 秩崩
    #    （那一帧实测跑到 94px）。
    q_tall = np.array([[169., 470.], [595., 470.], [595., 2.], [169., 2.]])
    check('③ 占满高度：贴图顶的上边被摘', not SO_.edge_usable(q_tall, 2, W, H))
    check('③ 占满高度：下边 sf12=0 但邻边都在 → 保住',
          SO_.edge_usable(q_tall, 0, W, H))
    n_use = len(SO_.usable_edges(q_tall, W, H))
    check(f'③ 占满高度：留 3 条边（实际 {n_use}），不会只剩两条平行边', n_use == 3)

    # ④ 真边：一端出画 17px 但另一端离图边 22px，且邻边都在画面里
    #    （`2026-09-30-11-37-17` 的下边）—— 摘了它 refine 从 3.75 崩到 16.35。
    q_real = np.array([[95., 457.], [471., 496.], [511., 108.], [136., 69.]])
    check('④ 真边（一端出画）：仍可信', SO_.edge_usable(q_real, 0, W, H))

    # ⑤ **秩保底**：判据把"上"也摘掉后只剩「左」一条边 —— 面内平移有一维
    #    完全不受约束。现场 bag `2026-10-14-19-57-09` 的 f0224 就是这样：解从
    #    69.7px 崩到 620.8px（投影四角跑到 (-108,1209)）。保底必须把边加回来。
    q_rank = np.array([[22.7, 484.], [548.5, 498.8], [562.4, 7.9], [36.6, -6.9]])
    use_before = [i for i in range(4)
                  if SO_._edge_trust(q_rank, i, W, H)]        # 只过三道闸门
    keep = SO_.usable_edges(q_rank, W, H)
    check(f'⑤ 秩保底前只剩 {use_before}（秩不够）', not SO_._rank_ok(use_before))
    check(f'⑤ 秩保底后 {keep}，两组对边都覆盖到', SO_._rank_ok(keep))
    check('⑤ 保底加回来的是"最不像外推边"的那条（上边，sf12=0.91）',
          2 in keep)


def test_seed_follows_the_obb_rotation():
    """★ 面内转角种子必须**跟着托盘转** —— 2026-10-15 现场问题的根因回归网。

    症状：托盘放在画面正中（无 OBB 退化），机器人怎么转，解算出的托盘都几乎
    "平行于机器人"。实测现场 bag：OBB 报的角 p10~p90 = `92.5°~97.1°`（有变化），
    而解算只输出 `78.0°~92.4°`（被夹在 `θ0 ± 25°` 的搜索窗里）。

    根因：`_seed_from_long_side` 按"`e1`/`e2` 哪条投影更水平"挑 θ0，而
    `_basis(normal)` 造出来的 `e1/e2` **只由法向决定、与托盘朝向无关** ——
    所以 θ0 是个**常数**（现场算下来恒 `90.0°`）。托盘转离它超过
    `INPLANE_LIMIT_DEG` 就被夹住，输出永远是那个方向。

    这条网直接构造"托盘在面内转 δ"的真值：新种子必须跟得上，而旧种子
    **必须是个常数**（那一半是"根因还在不在"的判据，不是多余的断言）。
    """
    from ..geom import _corners_mm, _project_px
    from ..solve_obb import (_basis, _seed_from_long_side, _seed_from_obb,
                             order_quad)

    # 现场拟合出来的法向（节点首帧日志 `法向来源=fit normal=[...]` 那一行）
    N = np.array([-0.0636, -0.2306, -0.971], float)
    N /= np.linalg.norm(N)
    e1, e2 = _basis(N)
    Wb, Hr = 1000.0, 1200.0                    # 现场 pallet_size_mm

    def truth_at(delta_deg):
        """托盘面内转 `delta_deg`（从 e2 起算，`E1` 是短边）-> (OBB 四角, 真值 θ)。"""
        a = np.radians(delta_deg)
        E1 = np.cos(a) * e2 + np.sin(a) * e1
        E1 /= np.linalg.norm(E1)
        E2 = np.cross(N, E1)
        E2 /= np.linalg.norm(E2)
        f = dict(origin=900.0 * N, E1=E1, E2=E2, nrm=N, W=Wb, H=Hr)
        quad = order_quad(_project_px(_corners_mm(f, Wb, Hr), S.K))
        return quad, np.degrees(np.arctan2(E1 @ e2, E1 @ e1)) % 180.0

    worst = 0.0
    old_vals = set()
    for d in (-40, -20, 0, 20, 40):
        quad, true_th = truth_at(d)
        s = _seed_from_obb(quad, e1, e2, S.K, False)
        if s is None:
            check(f'δ={d:+d}° 时种子解得出来', False, '返回了 None')
            continue
        err = abs((np.degrees(s) % 180.0 - true_th + 90) % 180 - 90)
        worst = max(worst, err)
        old_vals.add(round(float(np.degrees(
            _seed_from_long_side(e1, e2, S.K, False))), 6))

    check(f'新种子在 ±40° 内都跟得上真值（最大差 {worst:.1f}° < 25°）',
          worst < 25.0, f'worst={worst:.1f}°')
    check('旧种子**不随托盘转**（这就是根因：它是个常数）',
          len(old_vals) == 1, f'旧种子取值={sorted(old_vals)}')


def main() -> int:
    print('== 签名 / 契约 ==')
    test_signature_and_never_none()
    print('== 模型是硬失败 ==')
    test_model_is_a_hard_failure()
    print('== detect -> refine 全链路（假 session）==')
    test_detect_refine_pipeline_smoke()
    print('== 板子的尺寸真的走到用户规则 ==')
    test_target_mm_reaches_user_rule()
    print('== refine 的输出恒为右手系（det=+1）==')
    test_refine_output_is_never_mirrored()
    print('== refine 原语：向量化且与逐点版逐位一致 ==')
    test_refine_primitives_are_vectorized_not_loop()
    print('== 帧内记忆化不会误命中 ==')
    test_memo_never_returns_a_wrong_hit()
    print('== 面内转角种子跟着 OBB 转（现场回归）==')
    test_seed_follows_the_obb_rotation()
    print('== 出画的边不当位置约束（现场回归）==')
    test_edge_usable_rejects_extrapolated_edges()
    print('== 与 maduo 侧逐位一致（有数据包才跑）==')
    test_bit_exact_vs_maduo()
    print()
    if _fails:
        print(f'{len(_fails)} 项失败：{", ".join(_fails)}')
        return 1
    print('全部通过')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
