"""`pallet_detect` 的合成自检。跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 skills/atomic/perception/pallet_detect/tests/test_pallet_detect.py

**不需要 ROS、不需要相机、不需要任何数据** —— 它自己合成深度图、自己造真值。

这是本模块**唯一能自己判对错**的验证：真实数据上没有真值位姿，只能对照操作员
指定的结果。所有用例都是**几何真值**下的断言，不看图、不靠目视。

⚠️ 与 maduo 的同名测试（`debug/3test/detect_synthetic_test.py`）相比，本文件
**删掉了四个依赖数据集的用例** —— `test_search_crop_equivalence` /
`test_rasterise_single_pass` / `test_close_near_content` / `test_render_check`：
它们要 `test_data/` + `gt/`（真图、点击真值）才能跑，真机部署不需要。
那四个留在 maduo 侧，改动算法时两边都要跑。
"""
from __future__ import annotations

import sys
from pathlib import Path

if __name__ == '__main__':
    # 本文件用的是相对 import（`from .. import algorithm`），这样 pytest 收集
    # 得到、也不会把仓库根塞进 sys.path 污染别的测试。但直接 `python3 <本文件>`
    # 跑时 `__package__` 是空的，相对 import 会 ImportError —— 仓库里其余测试脚本
    # 的约定是「直接跑、退出码 0/1」，所以这里把包名补上再让解释器自己去加载包。
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
    __package__ = 'skills.atomic.perception.pallet_detect.tests'
    import importlib
    importlib.import_module(__package__)

import cv2
import numpy as np

from .. import algorithm as alg
from ..algorithm import _corners_mm, _project_px


W_MM, H_MM = 1200.0, 1000.0
FX = FY = 400.0
CX, CY = 320.0, 240.0
IMG_H, IMG_W = 480, 640

_fails: list[str] = []


def check(name: str, cond: bool, detail: str = '') -> None:
    if cond:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        _fails.append(name)


class K:
    fx, fy, cx, cy = FX, FY, CX, CY


def _unit(v):
    return v / np.linalg.norm(v)


def make_truth() -> dict:
    """一个已知真值的台面系。

    ⚠️ 朝向必须**符合操作员 2026-09-22 的约定**（规格 §3）：
      origin = 画面左下角，E1 = 与画面近平行的边、指向画面右，E2 = 指向上。
    相机系里图像右 = +x、图像下 = +y，所以：
      E1 的 x 分量 > 0（向右），E1 的 y 分量 ~ 0（与画面平行）；
      E2 的 y 分量 < 0（向上）。
    """
    nrm = _unit(np.array([0.05, -0.40, -0.915]))          # 指向相机
    e1 = _unit(np.array([1.0, 0.0, 0.0]) - (np.array([1.0, 0.0, 0.0]) @ nrm) * nrm)
    if e1[0] < 0:                                          # 保证指向画面右
        e1 = -e1
    e2 = _unit(np.cross(nrm, e1))
    if e2[1] > 0:                                          # 保证指向上（图像 v 更小）
        e2 = -e2
    return dict(origin=np.array([-600.0, -300.0, 1600.0]), E1=e1, E2=e2, nrm=nrm,
                W=W_MM, H=H_MM)


def _quad_px(frame: dict, w: float = W_MM, h: float = H_MM) -> np.ndarray:
    return _project_px(_corners_mm(frame, w, h), K)


def mask_from_truth(truth: dict, cover_quads=()) -> np.ndarray:
    """真值台面掩码：矩形内为 1，`cover_quads` 覆盖掉的部分为 0（模拟箱子遮挡）。

    ⚠️ 只用来测**纯几何**（T2–T4）。它绕开了木色分割，所以这里通过不代表
    真实数据上能过 —— 真实数据要等 T6。
    """
    m = np.zeros((IMG_H, IMG_W), np.uint8)
    cv2.fillPoly(m, [_quad_px(truth).astype(np.int32)], 1)
    for q in cover_quads:
        cv2.fillPoly(m, [np.asarray(q, np.int32)], 0)
    return m.astype(bool)


def render(truth: dict, cover_quads=(), floor_drop_mm: float = 900.0,
           ground_half_mm: float | None = None):
    """出 (color, depth)。台面刷木色，背景刷成**低 floor_drop_mm 的地面**。

    ⚠️ 背景**不能**和台面共面。深度路线（规格 §4 Step 2）就是靠高度分台面与背景的，
    两者共面时它必然失败 —— 那不是算法错，是这个合成场景没造对。
    地面用台面平面沿法向**下移** floor_drop_mm 得到（托盘本身有厚度，
    真实场景里地面就在台面下方）。

    ⚠️ 台面色是 `BGR(93, 145, 190)`（HSV `(16, 130, 190)`）—— T1 报出的问题 1：
    原色 `BGR(60, 130, 190)` 的 S=174 超出 `gd.WOOD_HSV` 上界 145，`_wood_mask`
    命中 0 像素。背景灰 `(150,150,150)` 与箱子色 `(170,90,40)` 实测都不会被
    误判成木色（箱子色 S=195 超出 `WOOD_HSV` 上界 145，被 `_wood_mask` 的
    `inRange` 排除），不用改。

    `ground_half_mm` —— 地面**半边长**，决定这块地面在场景里有多大：

      - `None`（**默认**，保持 T1~T3 的行为不变）：地面铺满整幅图，
        即除了托盘以外的每个像素都打在地面平面上。
      - 数值 `N`：地面平面上只放一块边长 `2*N` mm 的**方块**，方块中心在
        **托盘中心沿法向下移 floor_drop_mm 处**，即整块地面正好在托盘脚下、
        且完全落在托盘的正下方（`2*N` < 托盘短边 1000mm 时）。
        方块以外的像素深度为 0（无效）—— 整帧就只剩"托盘 + 它脚下这块地面"
        两个面。这样地面的格数可以**小于**台面的格数，深度路线的判据
        （"取格数最多的一带"）才有机会选对。

    ⚠️ 地面方块的**位置**不是随便摆的：它必须在托盘正下方，否则"地面"会跑到
    托盘外面去，测的就不是"托盘坐在一块比它小的地面上"这件事了。方块的四角
    建在地面平面内、沿 `E1`/`E2` 摆正，再投影到图像。

    ⚠️ **托盘会挡住方块的一部分**：方块整个在托盘正下方，而相机是斜看的，
    所以只有投影落在托盘四边形**外面**的那部分地面才有深度（`render()` 里
    托盘像素写的是台面深度）—— 回到占位栅格后它是一块 `2N x 2N` 方块的
    **可见部分**，不是完整的方块。实测 `ground_half_mm=350` 时栅格 bbox 是
    710 x 448 mm、**6296 格**，对台面的 **40271 格**（这就是"地面比托盘小"）。
    这是物理正确的遮挡（真实场景里托盘下的地面本来就被托盘挡住），
    场景自检会把这个格数打出来。
    """
    color = np.full((IMG_H, IMG_W, 3), (150, 150, 150), np.uint8)   # 灰地面
    cv2.fillPoly(color, [_quad_px(truth).astype(np.int32)], (93, 145, 190))  # 木色 BGR
    for q in cover_quads:
        cv2.fillPoly(color, [np.asarray(q, np.int32)], (170, 90, 40))        # 箱子色

    o, nrm = np.asarray(truth['origin'], float), np.asarray(truth['nrm'], float)
    e1, e2 = np.asarray(truth['E1'], float), np.asarray(truth['E2'], float)
    m = np.zeros((IMG_H, IMG_W), np.uint8)
    cv2.fillPoly(m, [_quad_px(truth).astype(np.int32)], 1)
    on_pallet = m > 0

    def _plane_z(origin):
        d = -float(origin @ nrm)
        out = np.zeros((IMG_H, IMG_W), np.float32)
        for v in range(IMG_H):
            ray = np.stack([(np.arange(IMG_W) - CX) / FX,
                            np.full(IMG_W, (v - CY) / FY), np.ones(IMG_W)], -1)
            den = ray @ nrm
            ok = np.abs(den) > 1e-6
            z = np.where(ok, -d / np.where(ok, den, 1.0), 0.0)
            out[v] = np.where(ok & (z > 100.0) & (z < 8000.0), z, 0.0)
        return out

    gz = _plane_z(o - floor_drop_mm * nrm)
    if ground_half_mm is None:
        depth = np.where(on_pallet, _plane_z(o), gz).astype(np.float32)
    else:
        n_ = float(ground_half_mm)
        cen = (o + 0.5 * float(truth['W']) * e1 + 0.5 * float(truth['H']) * e2
               - floor_drop_mm * nrm)                      # 托盘中心正下方的地面点
        sq3 = np.stack([cen - n_ * e1 - n_ * e2, cen + n_ * e1 - n_ * e2,
                        cen + n_ * e1 + n_ * e2, cen - n_ * e1 + n_ * e2])
        sq = np.zeros((IMG_H, IMG_W), np.uint8)
        cv2.fillPoly(sq, [_project_px(sq3, K).astype(np.int32)], 1)
        depth = np.where(on_pallet, _plane_z(o),
                         np.where(sq > 0, gz, 0.0)).astype(np.float32)
    return color, depth


def test_basis() -> list[bool]:
    """占位基：正交、右手、u 指向画面右。"""
    out = []
    for name, n in (('俯视略倾斜', np.array([0.05, -0.40, -0.915])),
                    ('几乎垂直向下', np.array([0.0, 0.0, -1.0])),
                    ('退化：法向几乎水平', np.array([0.0, -0.999, -0.04]))):
        n = _unit(n)
        u, v = alg._basis_from_normal(n)
        ok_orth = abs(float(u @ v)) < 1e-6 and abs(float(u @ n)) < 1e-6 \
            and abs(float(v @ n)) < 1e-6
        ok_unit = abs(np.linalg.norm(u) - 1) < 1e-9 and abs(np.linalg.norm(v) - 1) < 1e-9
        # 平面内最接近 +x 的方向：u 的 x 分量应当 >= 0
        ok_right = float(u[0]) >= -1e-9
        ok = ok_orth and ok_unit and ok_right
        print(f'  {name:20s} {"OK " if ok else "**FAIL**"} u={np.round(u, 3)} v={np.round(v, 3)}')
        out.append(ok)
    return out


def truth_mask_on_raster(truth: dict, mask: np.ndarray, height: np.ndarray,
                         info: dict, k, cover_quads=()) -> np.ndarray:
    """把真值台面矩形投到**占位栅格**上，形状与 `mask` 一致。

    ⚠️ 三处容易写错，都踩过：
      1. 四角是相机系 mm，必须先**减去占位平面原点** `info['origin']`
         才能投影到 (a, b)；
      2. (a, b) 减 `info['p0']`/`info['q0']` 得栅格像素坐标；
      3. 栅格 1px = 1mm，但体素是 `voxel_mm` 一格 —— 两者**不是一回事**，
         比掩码时用像素坐标即可，不要混用体素索引。

    `cover_quads` —— 可选，图像像素坐标下的遮挡多边形（与 `render()` 的
    `cover_quads` 同一组），在**栅格上**把这几块抠掉。真值矩形投到栅格后
    和 `mask` 一样是"扣掉遮挡"的，两者才能同口径比 —— 否则断言变成
    "mask ≈ 真值矩形 − 遮挡面积"，连几何都对不上。
    （⚠️ 这样**仍然测不到**"扣货"逻辑，原因见 `test_deck_mask` 的 docstring。）
    """
    o = np.asarray(info['origin'], float)
    c = _corners_mm(truth, truth['W'], truth['H']) - o
    ab = np.stack([c @ truth['E1'] - info['p0'], c @ truth['E2'] - info['q0']], -1)
    out = np.zeros(mask.shape, np.uint8)
    cv2.fillPoly(out, [np.round(ab).astype(np.int32)], 1)
    for poly in _cover_on_raster(cover_quads, truth, info):
        cv2.fillPoly(out, [poly.astype(np.int32)], 0)
    return out > 0


def _cover_on_raster(cover_quads, truth: dict, info: dict) -> list[np.ndarray]:
    """图像像素坐标下的遮挡多边形 -> 占位栅格坐标下的多边形列表（int32）。

    ⚠️ 三处必须与 `render()` 对齐，否则抠出来的区域会偏，测试就成了假阳性：

      1. **先在图像里跟台面四边形求交**。测试给的遮挡块会伸出台面（画到背景上），
         那些像素在 `render()` 里深度落在**地面**上；若照样用台面平面反投影，
         它们会落到台面足迹**内部**，把不该抠的台面格抠掉。
         求交后每个顶点都在台面内，反投影到台面平面必然有效。
      2. **必须画成实心多边形，不能只按像素打点**。图像像素 → 台面平面是放大
         的（z/fx ≈ 5mm/px），一个像素对应栅格上约 5x5 格；只打点会抠出一个
         稀疏散点带，抠掉的格数只有实际遮挡区的 1/25（实测踩过）。
         投影是射影变换、保直线，所以取交点多边形的**顶点**逐个反投影再
         `fillPoly` 就等价于整块投影。
      3. 箱子是**画在台面上的色块**（`render()` 不抬高它的深度），所以用**台面
         平面**反投影，与它在图像里的位置一致。
    """
    nrm = np.asarray(truth['nrm'], float)
    d = -float(np.asarray(truth['origin'], float) @ nrm)
    o = np.asarray(info['origin'], float)
    e1, e2 = np.asarray(truth['E1'], float), np.asarray(truth['E2'], float)
    pal = np.round(_quad_px(truth)).astype(np.float32)

    polys = []
    for q in cover_quads:
        quad = np.round(np.asarray(q, float)).astype(np.float32)
        area, inter = cv2.intersectConvexConvex(quad, pal)
        if area <= 0 or inter is None:
            continue
        pts = np.asarray(inter, np.float64).reshape(-1, 2)
        u = (pts[:, 0] - CX) / FX
        v = (pts[:, 1] - CY) / FY
        ray = np.stack([u, v, np.ones(len(pts))], -1)
        den = ray @ nrm
        if np.any(np.abs(den) <= 1e-6):
            raise AssertionError('遮挡多边形顶点反投影到台面平面失败（射线与平面平行）')
        z = -d / den
        p3 = ray * z[:, None]
        r = p3 - o
        poly = np.stack([np.round(r @ e1 - info['p0']), np.round(r @ e2 - info['q0'])],
                        -1)
        polys.append(poly)
    return polys


def test_deck_mask() -> list[bool]:
    """掩码 = 真值矩形；箱子盖住的那块要被扣掉。

    ⚠️ T3 自审发现：这条用例原来 `want` 没扣遮挡，断言实际是
    "mask ≈ 真值矩形 − 遮挡面积"，**没有验证"扣货"逻辑**。现在 `want` 也扣掉
    同一组 `cover_quads`（`truth_mask_on_raster(..., cover_quads=covers)`），
    两者同口径，IoU 从 0.849 回到 0.992。

    ⚠️ **但"扣货"仍然没有被这条用例验证 —— 实测确认它验证不了。**
    变异测试（把 `_deck_mask` 里 `deck &= ~(wood & (height-deck_h >= BOX_RAISE_MM))`
    一行改成 no-op）后，这条用例**照样通过**（`[True, True]`），两个场景的 mask
    格数一字不差（1021444 / 1021444）。原因有两条，各自独立：

      1. **箱子那块的像素根本不在台面带上**。`render()` 的 `cover_quads` 只刷颜色、
         **不抬高深度**，而箱子色 `(170,90,40)` 的 S=195 超出 `WOOD_HSV` 上界 145，
         被 `_wood_mask` 的 `inRange` 排除 → 箱区像素连 `wood` 都不是，
         压根进不了 `deck`，轮不到"扣货"去删。抠出来只有 ~900 格残留，
         那是遮挡区**边缘**的混合像素。
      2. **就算把箱子刷成暖木色，那一行也是死代码**：`deck` 已经要求
         `|h-deck_h| <= DECK_BAND_MM`(40mm)，而"扣货"删的是 `h-deck_h >= BOX_RAISE_MM`(90mm)，
         两个集合的交集为空。实测：把箱子刷成木色 + 抬高 60/200mm，真实模块与变异
         模块的输出**逐格相同**（`diff=0`）。

    所以这条用例现在的定位是"**台面矩形 + 遮挡区域抠除**的几何回归"，
    不是"扣货逻辑"的验证。详见 `.sdd-detect/task-3-report.md` 的「T3b 补充」一节。
    """
    out = []
    truth = make_truth()
    q = _quad_px(truth)
    cover = np.array([q[3] + np.array([-60, -120]), q[2] + np.array([60, -120]),
                      q[2] + np.array([60, 20]), q[3] + np.array([-60, 20])])
    for name, covers in (('无遮挡', ()), ('箱子遮一角', (cover,))):
        color, depth = render(truth, covers)
        mask, height, info = alg._deck_mask(color, depth, K, truth['nrm'])
        want = truth_mask_on_raster(truth, mask, height, info, K, cover_quads=covers)
        inter = int((mask & want).sum())
        union = int((mask | want).sum())
        iou = inter / max(union, 1)
        ok = iou > 0.80
        print(f'  {name:12s} {"OK " if ok else "**FAIL**"} IoU={iou:.3f} '
              f'mask={int(mask.sum())} want={int(want.sum())} '
              f'source={info["source"]} h_deck={info["h_deck"]}')
        out.append(ok)
    return out


def _band_selfcheck(height: np.ndarray, info: dict, truth: dict, k) -> dict:
    """数出**地面带**与**台面带**的格数，自证场景确实造对了。

    ⚠️ **不能把"h≈0 就是台面"写死** —— 占位 frame 的 `origin` 取的是点云高度的
    中位数，哪一面大它就在哪一面上，所以两个场景里 `h=0` 对应的面**不一样**
    （实测：场景 A 里 `h=0` 是台面、`h=-900` 是地面；场景 B 里 `h=0` 是地面、
    `h=900` 是台面）。写死就会把场景 B 的地面/台面数反。

    ⚠️ 也不能用"带内真值占比最高"来挑台面带：场景 A 里那块地面方块**整个在托盘
    正下方**，所以它的带内真值占比也是 1.000，挑不出来（实测踩过）。

    这里用**真值台面平面在占位 frame 里的高度** `h_plane` 当参照：
    离它最近的带是台面带，离它约 `floor_drop_mm` 的那个带是地面带。
    """
    o = np.asarray(info['origin'], float)
    h_plane = float((np.asarray(truth['origin'], float) - o) @ truth['nrm'])
    c = _corners_mm(truth, truth['W'], truth['H']) - o
    ab = np.stack([c @ truth['E1'] - info['p0'], c @ truth['E2'] - info['q0']], -1)
    rect = np.zeros(height.shape, np.uint8)
    cv2.fillPoly(rect, [np.round(ab).astype(np.int32)], 1)
    rect = rect > 0

    valid = height > -1e3
    cell = float(alg.VOXEL_MM)
    bins = np.round(height[valid] / cell) * cell
    uniq, cnt = np.unique(bins, return_counts=True)
    keep = cnt >= alg.MIN_DEPTH_CELLS
    bands = []
    for u, c_ in zip(uniq[keep], cnt[keep]):
        band = valid & (np.abs(height - float(u)) <= cell * 0.5)
        bands.append(dict(h=float(u), cells=int(c_),
                          truth=float((band & rect).sum()) / max(int(band.sum()), 1)))
    if not bands:
        return dict(ground=None, deck=None, bands=bands, h_plane=h_plane)
    deck = min(bands, key=lambda b: abs(b['h'] - h_plane))
    rest = [b for b in bands if b is not deck]
    if not rest:
        return dict(ground=None, deck=deck, bands=bands, h_plane=h_plane)
    ground = max(rest, key=lambda b: b['cells'])
    return dict(ground=ground, deck=deck, bands=bands, h_plane=h_plane)


def _print_selfcheck(tag: str, sc: dict) -> None:
    if sc.get('ground') is None:
        print(f'    [场景自检 {tag}] 带数不足，无法分辨地面/台面：{sc["bands"]}')
        return
    g, d = sc['ground'], sc['deck']
    print(f'    [场景自检 {tag}] 真值台面平面 h={sc["h_plane"]:.0f}  |  '
          f'地面带 h={g["h"]:.0f} 格数={g["cells"]} '
          f'(带内真值占比 {g["truth"]:.3f})  |  '
          f'台面带 h={d["h"]:.0f} 格数={d["cells"]} (带内真值占比 {d["truth"]:.3f})  '
          f'-> 地面{"<" if g["cells"] < d["cells"] else ">"}台面')


def test_deck_mask_deadcode() -> list[bool]:
    """把两条实测结论钉在测试里，免得以后有人重新踩：

      1. `_deck_mask` 里"扣货"那一行（`deck &= ~(wood & (h-deck_h >= BOX_RAISE_MM))`）
         是**死代码** —— `deck` 已要求 `|h-deck_h| <= DECK_BAND_MM`(40)，
         而它删的是 `h-deck_h >= BOX_RAISE_MM`(90)，交集为空。
         **判定用的两个常量取自 `gd` 本体**，所以哪天有人把 `DECK_BAND_MM` 调到
         >= `BOX_RAISE_MM`，这条会立刻 FAIL，提醒那一行不再是死代码。
      2. `render()` 的 `cover_quads` **不抬高深度**，所以箱子在深度上就是台面；
         合成场景里无法用它来验证"扣货"。

    这里不碰 `detect_pallet_frame.py`，只读它的常量。
    """
    out = []
    gd = alg
    # ⚠️ `BOX_RAISE_MM`（90.0）**不在本模块里** —— 那一行"扣货"代码合并进 LeTools
    # 时已经被删掉了（`_deck_mask` 里留了说明），所以它没有别的用处。
    # 这里按 maduo 的 `ground_detector.BOX_RAISE_MM` 的值镜像一份，只为钉住
    # "万一有人把 DECK_BAND_MM 调大，那条判据就不再是死代码"这件事。
    BOX_RAISE_MM = 90.0

    ok = gd.DECK_BAND_MM < BOX_RAISE_MM
    print(f'  DECK_BAND_MM={gd.DECK_BAND_MM:.0f} < BOX_RAISE_MM={BOX_RAISE_MM:.0f} '
          f'{"OK " if ok else "**FAIL**"}（成立时"扣货"一行不可达 = 死代码）')
    out.append(ok)

    # 随机穷举：任意 height/wood，加上"扣货"前后结果恒等 -> 死代码
    rng = np.random.default_rng(0)
    same = True
    for _ in range(50):
        hgt = rng.normal(0, 200, (120, 120)).astype(np.float32)
        hgt[rng.random((120, 120)) < 0.3] = -1e3
        wd = rng.random((120, 120)) < 0.5
        deck_h = float(rng.normal(0, 200))
        a = wd & (np.abs(hgt - deck_h) <= gd.DECK_BAND_MM)
        b = a.copy()
        b &= ~(wd & ((hgt - deck_h) >= BOX_RAISE_MM))
        same &= bool(np.array_equal(a, b))
    print(f'  50 组随机 height/wood 加不加"扣货"结果相同 {"OK " if same else "**FAIL**"}'
          f'（相同 = 那一行不可达）')
    out.append(same)
    return out


def test_deck_mask_depth_path() -> list[bool]:
    """强制走深度路线，测**两种**场景。

    ⚠️ 为什么要分两种：深度路线的判据是"取格数最多的高度带"（规格 §4 Step 2）。
      这个判据**只在台面是最大的那一带时成立**。T3 在 5_test 上实测：
      栅栏形托盘缝里漏出的**地面比托盘大**（20.7% vs 8.7%），判据必然选错，
      而且**不是调参能救的**（合并容差试了 4 种、bbox 归一化都试过）。
      所以这里既测"该成的成"，也测"该败的败得干净"。

    操作员 2026-09-23 裁决：**「先不救，5_test 上接受失败」** —— 所以场景 B
    的"通过"意思是"**它确实如预期地失败了**"，不是期望它成功。
    """
    truth = make_truth()
    out = []

    # --- 场景 A：地面比托盘小 -> 台面是最大的一带 -> 深度路线应当成功 ---
    #     地面方块边长 2*350=700mm < 托盘短边 1000mm，摆在托盘正下方。
    color, depth = render(truth, ground_half_mm=350.0)
    gray = color.copy()
    cv2.fillPoly(gray, [_quad_px(truth).astype(np.int32)], (150, 150, 150))  # 台面刷灰
    mask, height, info = alg._deck_mask(gray, depth, K, truth['nrm'])
    want = truth_mask_on_raster(truth, mask, height, info, K)
    iou = int((mask & want).sum()) / max(int((mask | want).sum()), 1)
    sc = _band_selfcheck(height, info, truth, K)
    _print_selfcheck('A', sc)
    # 场景前提自检：地面带必须真的比台面带小，否则测的就不是"地面比托盘小"
    ok_sc = sc.get('ground') is not None and sc['ground']['cells'] < sc['deck']['cells']
    ok = (info['source'] == 'depth') and iou > 0.80 and ok_sc
    if not ok_sc:
        print('    **FAIL** 场景 A 前提不成立：地面带没有比台面带小')
    print(f'  A 地面较小 {"OK " if ok else "**FAIL**"} source={info["source"]} '
          f'IoU={iou:.3f} h_deck={info["h_deck"]}')
    out.append(ok)

    # --- 场景 B：地面铺满整幅图（比托盘大）-> 判据必然选到地面 ---
    #     这不是 bug，是**已知的结构性局限**（规格 §4 Step 2）。
    #     这里只断言"它确实选错了"，把局限**钉在测试里**，
    #     免得以后有人以为深度路线在所有场景都能用。
    color2, depth2 = render(truth, ground_half_mm=None)   # 地面铺满
    gray2 = color2.copy()
    cv2.fillPoly(gray2, [_quad_px(truth).astype(np.int32)], (150, 150, 150))
    mask2, height2, info2 = alg._deck_mask(gray2, depth2, K, truth['nrm'])
    want2 = truth_mask_on_raster(truth, mask2, height2, info2, K)
    iou2 = int((mask2 & want2).sum()) / max(int((mask2 | want2).sum()), 1)
    sc2 = _band_selfcheck(height2, info2, truth, K)
    _print_selfcheck('B', sc2)
    # 场景前提自检：地面带必须真的比台面带大（这才是"已知局限"的触发条件）
    ok_sc2 = sc2.get('ground') is not None and sc2['ground']['cells'] > sc2['deck']['cells']
    # 断言的是"选错了"（IoU 低），且**走的是深度路线** —— 记录局限，不是期望成功
    ok2 = (info2['source'] == 'depth') and iou2 < 0.50 and ok_sc2
    if not ok_sc2:
        print('    **FAIL** 场景 B 前提不成立：地面带没有比台面带大')
    print(f'  B 地面铺满 {"OK " if ok2 else "**FAIL**"} source={info2["source"]} '
          f'IoU={iou2:.3f}  <- 已知局限：地面比托盘大时判据必错（规格 §4 Step 2）')
    out.append(ok2)
    return out


def make_truth_short_parallel() -> dict:
    """把 `make_truth()` 的 E1/E2 互换 —— 让 **1000 那条边**成为"与画面近平行"的那条。

    ⚠️ **为什么需要这个**：`make_truth()` 建出来的场景里，与画面近平行的是 **E1(1200)**，
    也就是**长边平行**（实测图像方向角 1.61° vs E2 的 80.32°）。而操作员给的先验
    `LONG_SIDE_PARALLEL=False` 说的是 **5_test 短边平行**。两者**恰好相反**。
    T4 原先只用一个场景测两个先验，于是 `lsp=False` 必然差一个象限而 FAIL ——
    **那是测试场景与先验不匹配，不是 `_theta_ref` 的缺陷**（T4 实测报告已定位）。
    所以这里补一个与先验匹配的场景：**先验和场景必须对得上，否则测的是错配。**
    """
    t = make_truth()
    t2 = dict(t)
    t2['E1'], t2['E2'] = t['E2'], t['E1']
    t2['W'], t2['H'] = t['H'], t['W']
    return t2


def test_theta_ref() -> list[bool]:
    """θ_ref 要落在真值朝向附近，且**误差远小于 ±45° 的半象限**。

    两个场景各测一次，**先验必须与场景匹配**（见 `make_truth_short_parallel` 的说明）：
      - 长边平行的场景配 `lsp=True`；
      - 短边平行的场景配 `lsp=False` —— 这条与 5_test 的真实先验一致。
    """
    out = []
    for scene, lsp, name in ((make_truth(), True, '长边平行场景 lsp=True'),
                             (make_truth_short_parallel(), False, '短边平行场景 lsp=False')):
        mask = mask_from_truth(scene)
        u, v = alg._basis_from_normal(scene['nrm'])
        # ⚠️ **`origin` 必须是平面上的点，不能是 `np.zeros(3)`**（2026-09-30 修）。
        # `_project_axis` 现在取方向**两端各 500mm** 的点投影相减；若原点是相机
        # 光心，两端点一个落在光心、一个落到相机后方，投影退化、`_theta_ref`
        # 返回 None。这里用真值 frame 的 origin（就在台面平面上）。
        frame = dict(origin=np.asarray(scene['origin'], float),
                     E1=u, E2=v, nrm=scene['nrm'])
        true_theta = float(np.arctan2(float(scene['E1'] @ v), float(scene['E1'] @ u)))
        th = alg._theta_ref(mask, frame, K, long_side_parallel=lsp)
        if th is None:
            print(f'  {name:24s} **FAIL** 返回 None'); out.append(False); continue
        d = np.degrees((th - true_theta + np.pi / 2) % np.pi - np.pi / 2)
        ok = abs(d) < 10.0
        print(f'  {name:24s} {"OK " if ok else "**FAIL**"} '
              f'θ_ref={np.degrees(th):+7.2f}° 真值={np.degrees(true_theta):+7.2f}° 差={d:+.2f}°')
        out.append(ok)
    return out


def _errors(found: dict, truth: dict):
    d = found['origin'] - truth['origin']
    return (float(d @ truth['E1']), float(d @ truth['E2']),
            float(np.degrees(np.arccos(np.clip(found['E1'] @ truth['E1'], -1, 1)))))


TOL_MM, TOL_DEG = 50.0, 2.0        # 规格 §4.1：detect 只需落进 refine 的收敛域
# 先验路径 vs 全画布搜索的四角容差（推导见 `test_prior_track` 的 docstring）
TOL_PRIOR_CORNER_MM = 5.0


def test_detect_full() -> list[bool]:
    """4 边全可见：应当解回真值到收敛域以内。"""
    out = []
    truth = make_truth()
    color, depth = render(truth)
    found, diag = alg.detect_pallet_frame(
        color, depth, K, normal=truth['nrm'], target_mm=(W_MM, H_MM),
        long_side_parallel=True, opts=dict(voxel_mm=2.0))
    if found is None:
        print(f'  **FAIL** 拒绝: {diag["reject"]}'); return [False]
    et, en, eth = _errors(found, truth)
    ok = abs(et) < TOL_MM and abs(en) < TOL_MM and eth < TOL_DEG
    print(f'  4 边  {"OK " if ok else "**FAIL**"} '
          f'dE1={et:+7.1f}mm dE2={en:+7.1f}mm dθ={eth:5.2f}°  '
          f'score={diag["score"]:.3f} 路={diag["rect_source"]} '
          f'θ_ref={_fmt_theta(diag["theta_ref_deg"])} 边={diag["n_observed_edges"]}')
    return [ok]


def _fmt_theta(v) -> str:
    """`diag['theta_ref_deg']` 的 None 安全打印。

    ⚠️ 主路径（`rect_source='dense_rect'`）**不经过 `_theta_ref`**，所以这个键是
    `None` —— 那是设计如此（绕开它正是主路径的目的），不是缺算。
    """
    return 'n/a(主路径)' if v is None else f'{v:+.1f}°'


def break_edge_box(truth: dict, side: str, span: float = 840.0,
                   depth: float = 230.0, inset: float = 180.0) -> np.ndarray:
    """在**托盘系**里放一个箱子，把 `side` 那条边**打断**（遮住它的中段）。

    --- 为什么必须换成这种造法（T5e，2026-09-23）---

    旧造法（`cover_band`）是在**图像**里沿某条边盖一条 240px 宽的带子。它在
    斜视投影下**物理上做不到"只遮一条边"**：托盘 1000mm 的深度方向在图像里只
    投影出 ~167px，带子一盖就是 **690/1000mm**，把台面 74.8%/87.5% 的像素吞掉。
    更致命的是，被遮的那条边**在掩码里一点痕迹都不剩**（实测 b 剖面：真值边
    附近 25mm 一档全是 0 格，然后突然跳到 25764 格），于是 coverage 沿该方向
    出现一条**长平台** —— 实测平台宽 == 被遮宽度（厚 30/100/300mm 的墙 ->
    平台 44/114/316mm），真值落在平台的一端，`argmax` 挑到平台另一端，
    误差 = 被遮宽度。**位姿在该方向物理不可测**，不是判据问题。
    （`_thin.py` / `_final.py` 的实测数据在 `.sdd-detect/progress.md` 的 T5e 一节。）

    箱子是**有限尺寸**的（`span` < 边长），所以它压住一条边的中段时，台面在该
    方向**仍有一部分没被盖住**，平台塌掉。实测：箱子造的"遮 1 边/遮 2 边相邻"
    误差 1~2mm，而墙造的同类用例误差 304mm。

    `side` ∈ {'a_lo','a_hi','b_lo','b_hi'}：托盘系里 a∈[0,1200]、b∈[0,1000]。
    箱子沿被遮边方向 `span` 宽、法向 `depth` 厚、两端各内缩 `inset`。
    """
    W, H = float(truth['W']), float(truth['H'])
    o = np.asarray(truth['origin'], float)
    e1 = np.asarray(truth['E1'], float)
    e2 = np.asarray(truth['E2'], float)
    if side == 'b_lo':
        a0, b0, da, db = inset, -30.0, span, depth
    elif side == 'b_hi':
        a0, b0, da, db = inset, H - depth + 30.0, span, depth
    elif side == 'a_lo':
        a0, b0, da, db = -30.0, inset, depth, span
    elif side == 'a_hi':
        a0, b0, da, db = W - depth + 30.0, inset, depth, span
    else:
        raise ValueError(f'unknown side {side!r}')
    pts = np.array([[a0, b0], [a0 + da, b0], [a0 + da, b0 + db], [a0, b0 + db]], float)
    world = o[None, :] + pts[:, 0:1] * e1[None, :] + pts[:, 1:2] * e2[None, :]
    return _project_px(world, K)


def test_detect_degenerate() -> list[bool]:
    """遮挡退化（规格 §5.1）：**相邻 2 边可见 -> 输出；只有 1 边 / 0 边 / 只剩对边 -> 拒绝**。

    --- T5e（2026-09-23）：换掉了造遮挡的方式，§5.1 的意图**第一次真正达成** ---

    前面 T5 / T5b / T5c / T5d 四轮都在"用图像里的**横贯带**模拟遮挡"这个造法上
    打转，四轮都没让"该输出的输出"。T5e 查清了根因，**根因不在判据、不在参数，
    在造法**：

      * 横贯带在斜视投影下**不可能只遮一条边** —— 托盘 1000mm 的深度方向只投影
        ~167px，带子一盖就是 690/1000mm（台面 74.8%/87.5% 的像素）。
      * 更关键：被遮的那条边**在掩码里没有留下任何痕迹**（b 剖面实测：真值边
        附近 25mm 一档全是 0 格，然后跳到 25764 格）。于是 coverage 沿该方向是
        一条**长平台**，**平台宽 == 被遮宽度**（实测墙厚 30/100/300mm ->
        平台 44/114/316mm），真值落在平台一端、`argmax` 挑到另一端，
        误差 = 被遮宽度。**位姿在该方向物理不可测。**
      * 所以旧断言里那句"`score` 已达物理最优、任何 θ/平移都不会更高"是**对的**，
        但结论下错了：物理最优**不等于**位姿可解 —— 最优是一整条脊时，
        搜索只能随便挑一点。旧测试把"不可测"记成了"判据太严"。

    T5e 改用**托盘系里的有限尺寸箱子**（`break_edge_box`）造遮挡 —— 这也是真实
    场景里的样子（箱子摆在托盘上，不是一堵无限长的墙）。箱子只压住一条边的
    **中段**，台面在该方向仍有未被盖住的部分，平台随之塌掉。实测：

      | 用例                    | source | 可见边 | score | 误差    | 结果 |
      |-------------------------|--------|--------|-------|---------|------|
      | 无遮挡                  | color | 0,1,2,3| 0.993 |  2 mm   | 输出 |
      | 遮 1 边 b_lo            | color | 1,2,3  | 0.848 |  2 mm   | 输出 |
      | 遮 1 边 a_hi            | color | 0,2,3  | 0.882 |  1 mm   | 输出 |
      | 遮 2 边**相邻** b_lo+a_hi | color | 2,3    | 0.737 |  1 mm   | 输出 |
      | 遮 2 边**相邻** a_lo+b_lo | color | 1,2    | 0.740 |  2 mm   | 输出 |
      | 遮 2 条**对边** a_lo+a_hi | color | 0,2    | 0.772 |   —     | 拒绝 |
      | 遮 2 条**对边** b_lo+b_hi | color | 1,3    | 0.702 |   —     | 拒绝 |
      | 遮 3 边                 | color | 3      | 0.592 |   —     | 拒绝 |
      | 遮 4 边                 | color | 无     | 0.484 |   —     | 拒绝 |

    断言按 **`diag['edges']` 里 `observed` 的边号**写，不按"误差小"写 ——
    这样它测的是**判据**（哪几条边被认成边），而不是被遮宽度的数值。
    另外前两条钉住 `source == 'color'` 与 `color_coverage >= WOOD_COVERAGE_MIN`：
    谁把 `_color_coverage` 改回 `÷band`，覆盖率会掉到 0.25 以下、`source` 变
    `depth`、掩码糊成整块地面，这里立刻 FAIL。
    """
    out = []
    truth = make_truth()
    o = np.asarray(truth['origin'], float)
    e1 = np.asarray(truth['E1'], float)
    e2 = np.asarray(truth['E2'], float)

    # (名字, 打断哪几条边, 期望的输出?, 期望的可见边集合)
    cases = [
        ('无遮挡', (), True, {0, 1, 2, 3}),
        ('遮 1 边 b_lo（3 边可见）', ('b_lo',), True, {1, 2, 3}),
        ('遮 1 边 a_hi（3 边可见）', ('a_hi',), True, {0, 2, 3}),
        ('遮 2 边 -> 相邻 2 边 b_lo+a_hi', ('b_lo', 'a_hi'), True, {2, 3}),
        ('遮 2 边 -> 相邻 2 边 a_lo+b_lo', ('a_lo', 'b_lo'), True, {1, 2}),
        ('遮 2 条对边 a_lo+a_hi', ('a_lo', 'a_hi'), False, None),
        ('遮 2 条对边 b_lo+b_hi', ('b_lo', 'b_hi'), False, None),
        ('遮 3 边（只剩 1 边）', ('b_lo', 'a_hi', 'b_hi'), False, None),
        ('遮 4 边（0 边可见）', ('b_lo', 'a_hi', 'b_hi', 'a_lo'), False, None),
    ]
    for name, sides, want_ok, want_obs in cases:
        covers = tuple(break_edge_box(truth, s) for s in sides)
        color, depth = render(truth, covers)
        found, diag = alg.detect_pallet_frame(
            color, depth, K, normal=truth['nrm'], target_mm=(W_MM, H_MM),
            long_side_parallel=True, opts=dict(voxel_mm=2.0))
        obs = {i for i, e in enumerate(diag['edges']) if e['observed']}
        if want_obs is not None:
            # 该输出的：边集必须**恰好**是期望的那几条，且位姿落在收敛域内
            ok = (found is not None and obs == want_obs
                  and diag['source'] == 'color'
                  and diag['color_coverage'] is not None
                  and diag['color_coverage'] >= alg.WOOD_COVERAGE_MIN)
            if found is not None:
                d = found['origin'] - o
                err = max(abs(float(d @ e1)), abs(float(d @ e2)))
                ok = ok and err < TOL_MM
                extra = (f'边={sorted(obs)} dE1={float(d@e1):+6.1f} '
                         f'dE2={float(d@e2):+6.1f} score={diag["score"]:.3f} '
                         f'source={diag["source"]}')
            else:
                extra = f'**拒绝了** reject={diag["reject"]} 边={sorted(obs)}'
        else:
            # 该拒绝的：必须**没有两条相邻的可见边**，且如实拒绝。
            # ⚠️ 不能写成 `len(obs) <= 1` —— "遮 2 条对边"的 `obs` 是 {0,2} / {1,3}，
            # **两条边都可见、但互不相邻**，正是 §5.1 要拒的那种。判据是
            # "有没有相邻的一对"，不是"有几条"。
            adjacent = any((i + 1) % 4 in obs or (i - 1) % 4 in obs for i in obs)
            ok = (found is None and not adjacent
                  and diag['reject'] in ('too_few_edges', 'ambiguous_theta'))
            extra = (f'边={sorted(obs)} reject={diag["reject"]} '
                     f'score={diag["score"]} source={diag["source"]}')
        print(f'  {name:32s} {"OK " if ok else "**FAIL**"} {extra}')
        out.append(ok)
    return out




def test_mask_area_ratio() -> list[bool]:
    """T5c：**掩码面积比**判据（`alg.MASK_AREA_RATIO_MAX`）—— 把实测的五个数字钉住。

    这是 T5c 的**唯一验收判据**。判据本身是**新加的**（不是调 `COVERAGE_MIN`）：
    `COVERAGE_MIN` 是下界，管不了"掩码本身是不是台面"；掩码被 25x25 闭运算糊成
    "整块地面"后，任何已知尺寸矩形都拿满分。能分开"整块地面"与"台面"的只有
    **掩码的绝对大小** —— 所以拒绝码是 `mask_too_large`，诊断里带
    `diag['mask_area_ratio']`（= 掩码格数 / `target_mm` 乘积）。

    钉住的三档（**voxel 默认 5.0mm**，与任务书那张表同口径）：

      | 用例                    | source | 掩码格数   | 面积比 | 期望 |
      |-------------------------|--------|-----------|--------|------|
      | 颜色路线（无遮挡）       | color |  1 195 439 | 0.996  | 不拒 |
      | 深度路线 场景A（地面小） | depth  |  1 216 589 | 1.014  | 不拒 |
      | 深度路线 遮 3 边         | depth  | 11 647 460 | 9.706  | 拒   |
      | 深度路线 遮对边          | depth  | 11 647 460 | 9.706  | 拒   |

    ⚠️ **5_test 真实数据（面积比 0.64、coverage 0.261）不写进这条**：它要读真实
    数据、跑得慢，而且**它不是这条判据拦下的**（0.64 < 3.0）—— 它靠 `COVERAGE_MIN`
    拒绝。**两者是不同的问题**，写进来会让人误以为一个判据修两个。
    5_test 的数字记在 `.sdd-detect/task-5-report.md` 的「T5c」一节。

    ⚠️ **T5d 复核：换分母不影响这条的任何数字**（实测逐条不变）。理由：
      1. 上面四条里**只有第一条**是颜色路线（无遮挡，`wood` 充足），而它新旧
         定义下都是 `color`（新 1.000 / 旧 1.000），掩码逐格不变；
      2/3/4 条是**深度路线** —— 第 2 条把台面刷成灰（`wood` 全灭），第 3/4 条
         `wood` 本来就全灭，三条都走深度路线，与 `_color_coverage` 无关。
      所以"掩码糊成整块地面"这一档仍由本条拦下（9.7072 > 3.0），**判据没有被削弱**。
    """
    out = []
    truth = make_truth()
    pallet = W_MM * H_MM

    def _ratio(mask, diag):
        return diag['mask_area_ratio'] if diag is not None else int(mask.sum()) / pallet

    # --- 1. 颜色路线（无遮挡）：不拒 ---
    color, depth = render(truth)
    mask, _h, info = alg._deck_mask(color, depth, K, truth['nrm'])
    r_color = int(mask.sum()) / pallet
    found, diag = alg.detect_pallet_frame(
        color, depth, K, normal=truth['nrm'], target_mm=(W_MM, H_MM),
        long_side_parallel=True, opts=dict(voxel_mm=2.0))
    ok1 = (info['source'] == 'color' and r_color < alg.MASK_AREA_RATIO_MAX
           and found is not None and diag['reject'] is None)
    print(f'  颜色路线 无遮挡      {"OK " if ok1 else "**FAIL**"} '
          f'source={info["source"]} 掩码格数={int(mask.sum())} 面积比={r_color:.4f} '
          f'(阈值 {alg.MASK_AREA_RATIO_MAX}) 输出={"有" if found is not None else "无"}')
    out.append(ok1)

    # --- 2. 深度路线 场景 A（地面比托盘小）：面积比 ~1.0，**面积判据不许开火** ---
    #
    # ⚠️ 这里**不**断言 `detect_pallet_frame` 返回非 None：场景 A 的掩码虽然正确
    # （对真值 IoU 0.98），但**它填满了自己那张栅格** —— `_rasterise` 是按点云范围
    # 建栅格的，而地面方块整个在托盘正下方，于是栅格 ≈ 地面方块范围，掩码占比
    # **99.0%**，最优矩形必然贴到边界 → 被**既有的** `degenerate` 判据拒绝。
    # 这是 T5c **之前就存在**的行为（T5c 只加"面积过大"这一条，1.0 < 3.0 走不到），
    # 与本次改动无关，**不在本任务范围内**（任务书：不要顺手改别的）。
    # 所以这里只钉住"**面积判据在这条上不开火**"这一件事。
    color2, depth2 = render(truth, ground_half_mm=350.0)
    gray2 = color2.copy()
    cv2.fillPoly(gray2, [_quad_px(truth).astype(np.int32)], (150, 150, 150))  # 台面刷灰
    mask2, _h2, info2 = alg._deck_mask(gray2, depth2, K, truth['nrm'])
    r_depth = int(mask2.sum()) / pallet
    found2, diag2 = alg.detect_pallet_frame(
        gray2, depth2, K, normal=truth['nrm'], target_mm=(W_MM, H_MM),
        long_side_parallel=True, opts=dict(voxel_mm=2.0))
    ok2 = (info2['source'] == 'depth' and r_depth < alg.MASK_AREA_RATIO_MAX
           and diag2['reject'] != 'mask_too_large')
    print(f'  深度路线 场景A 地面小 {"OK " if ok2 else "**FAIL**"} '
          f'source={info2["source"]} 掩码格数={int(mask2.sum())} 面积比={r_depth:.4f} '
          f'面积判据{"未开火" if diag2["reject"] != "mask_too_large" else "**误开火**"}'
          f'（detect 的实际 reject={diag2["reject"]}，见 docstring：既有问题）')
    out.append(ok2)

    # --- 3. 深度路线 遮 3 边 / 遮对边：**该拒**，且必须是被面积判据拒的 ---
    q = _quad_px(truth)

    def cover_band(i, half=120.0):
        p0, p1 = q[i], q[(i + 1) % 4]
        d = p1 - p0
        L = float(np.hypot(*d))
        u = d / L
        n = np.array([-u[1], u[0]])
        return np.array([p0 - 40 * u + half * n, p1 + 40 * u + half * n,
                         p1 + 40 * u - half * n, p0 - 40 * u - half * n])

    for name, idx in (('遮 3 边', (0, 1, 2)), ('遮 2 条对边', (0, 2))):
        covers = tuple(cover_band(i) for i in idx)
        c3, d3 = render(truth, covers)
        found3, diag3 = alg.detect_pallet_frame(
            c3, d3, K, normal=truth['nrm'], target_mm=(W_MM, H_MM),
            long_side_parallel=True, opts=dict(voxel_mm=2.0))
        r = diag3['mask_area_ratio']
        ok3 = (found3 is None and diag3['reject'] == 'mask_too_large'
               and r is not None and r > alg.MASK_AREA_RATIO_MAX)
        print(f'  深度路线 {name:10s} {"OK " if ok3 else "**FAIL**"} '
              f'source={diag3["source"]} 掩码格数={diag3["mask_cells"]} '
              f'面积比={r} reject={diag3["reject"]}')
        out.append(ok3)

    # --- 4. 阈值确实落在实测的可分区间里（将来谁动它，这里会 FAIL）---
    ok4 = r_color < alg.MASK_AREA_RATIO_MAX < 9.7
    print(f'  阈值位置  {"OK " if ok4 else "**FAIL**"} '
          f'该成功的两条最大 {max(r_color, r_depth):.4f} < '
          f'{alg.MASK_AREA_RATIO_MAX} < 9.7（该拒绝的两条）')
    out.append(ok4)
    return out


def test_detect_theta_ref_offset() -> list[bool]:
    """θ_ref 被人为搞偏 ±20°，搜索窗应当还兜得住（规格 §7.1）。"""
    out = []
    truth = make_truth()
    color, depth = render(truth)
    for off in (-20.0, +20.0):
        found, diag = alg.detect_pallet_frame(
            color, depth, K, normal=truth['nrm'], target_mm=(W_MM, H_MM),
            long_side_parallel=True,
            opts=dict(voxel_mm=2.0, theta_ref_offset_deg=off))
        if found is None:
            print(f'  θ_ref{off:+.0f}° **FAIL** 拒绝: {diag["reject"]}'); out.append(False); continue
        et, en, eth = _errors(found, truth)
        ok = abs(et) < TOL_MM and abs(en) < TOL_MM and eth < TOL_DEG
        print(f'  θ_ref{off:+.0f}°  {"OK " if ok else "**FAIL**"} '
              f'dE1={et:+7.1f} dE2={en:+7.1f} dθ={eth:5.2f}°')
        out.append(ok)
    return out


def test_theta_from_image() -> list[bool]:
    """T11：**图像空间主路径**（2026-09-30 加）—— θ 从哪来、守卫怎么回退、哨兵报不报。

    主路径替换的是 `_theta_ref`（不是 `_search_rect`）：图像掩码 -> 单应反查成稠密
    米制掩码 -> `minAreaRect` 的**长边方向**。为什么必须换：现场帧上
    `_theta_ref` 取栅格掩码的协方差主轴，给出 34.44°/124.44°，而托盘长边是
    83.49° —— 两个候选轴**都不是**托盘的边，`long_side_parallel` 只能二选一。
    """
    out = []
    truth = make_truth()

    # (1) 完整场景：主路径接管，量出的尺寸应接近标称，位姿落进收敛域
    color, depth = render(truth)
    found, diag = alg.detect_pallet_frame(
        color, depth, K, normal=truth['nrm'], target_mm=(W_MM, H_MM),
        long_side_parallel=True, opts=dict(voxel_mm=2.0))
    meas = diag['dense_measured_mm']
    ok = (diag['rect_source'] == 'image_theta'
          and meas is not None
          and abs(meas[0] - W_MM) < 0.15 * W_MM and abs(meas[1] - H_MM) < 0.15 * H_MM
          and found is not None
          and diag['long_short_mismatch'] is False
          and abs(diag['long_short_diff_deg']) < 10.0)
    print(f'  完整场景 {"OK " if ok else "**FAIL**"} rect_source={diag["rect_source"]} '
          f'fill={diag["dense_fill"]} 测得={meas} 哨兵差={diag["long_short_diff_deg"]}°')
    out.append(ok)

    # (2) L 形掩码（遮 2 条相邻边）：形状不像矩形，守卫必须回退到 `_theta_ref`。
    #     不回退的话主轴会转 90°，位姿被带到 184mm 外（实测）。
    covers = (break_edge_box(truth, 'b_lo'), break_edge_box(truth, 'a_hi'))
    color2, depth2 = render(truth, covers)
    _, diag2 = alg.detect_pallet_frame(
        color2, depth2, K, normal=truth['nrm'], target_mm=(W_MM, H_MM),
        long_side_parallel=True, opts=dict(voxel_mm=2.0))
    ok2 = (diag2['rect_source'] == 'search' and diag2['rect_why'] == 'dense_aspect')
    print(f'  L 形回退  {"OK " if ok2 else "**FAIL**"} rect_source={diag2["rect_source"]} '
          f'why={diag2["rect_why"]}')
    out.append(ok2)

    # (3) 哨兵：**只把 W/H 对调**（= `_orient` 记过的那类"位姿对、E1/W 错配"），
    #     必须报 ~90°。⚠️ 不能连 E1/E2 一起转 —— 那样帧本身还是自洽的，
    #     第一版负对照就是这么写的，实测 diff 仍是 0.50°，等于没测。
    bad = dict(found)
    bad['W'], bad['H'] = found['H'], found['W']
    d_bad = alg._long_short_angle_diff(bad, truth['nrm'], diag['dense_angle_deg'])
    d_ok = alg._long_short_angle_diff(found, truth['nrm'], diag['dense_angle_deg'])
    ok3 = d_bad > 45.0 and d_ok < 45.0
    print(f'  哨兵正反  {"OK " if ok3 else "**FAIL**"} 正常={d_ok:.2f}° 对调={d_bad:.2f}°')
    out.append(ok3)
    return out


def test_prior_track() -> list[bool]:
    """T9b：**先验跟踪路径**的正确性与回退行为。

    `detect_pallet_frame(prior=上一帧的 frame)` 只在上一帧位姿附近搜。
    这条测试钉住三件事：

      1. **先验误差在窗内时，结果与全画布搜索逐位相同** —— 这是"提速不改结果"。
         实测先验偏 120mm/4° 仍逐位相同。
      2. **先验误差超出窗（±150mm / ±5°）时，`prior_used=False` 且退回全搜索**，
         结果仍与全搜索逐位相同 —— 这是"能自愈"。
      3. **`score` 拦不住先验漂移** —— 这条是**反面**用例，防止有人把回退条件
         简化成"只看 score"：实测先验偏 250mm 时 score 仍有 0.91（远超
         `COVERAGE_MIN` 0.35），只有"顶到窗边"（`window_saturated`）能发现它。

    ⚠️ 判据用**四角最大差**而不是"dE1/dE2 都为 0"，容差也不是拍的：
    先验路径的 θ 细扫网格**锚在先验 θ 上**（`prior ± 3°`，步长 0.25°），
    而全画布搜索锚在 `theta_ref` 上（先 2° 粗搜再 ±4° 细搜）—— 两张网格不同相，
    所以同一个解在两边的 θ 可以差**一整格**。四角在半径
    `sqrt(1200²+1000²)/2 = 781mm` 处，0.25° 对应 `781*sin(0.25°) = 3.41mm`。
    **容差取 5mm**（实测最差 3.890mm）。
    真正的"跟丢"是几十~几百毫米，与这个量级差一个数量级以上。

    ⚠️ **2026-09-30 从 2mm 放到 5mm，并且原因变了。** 原来取 2mm 是按"差**半个**
    步长（0.125° -> 1.7mm）"推的，那是 `theta_ref` 与先验 θ 恰好差 2.56°、
    两张 0.25° 网格错开半个相位时的巧合。现在首帧的 θ 改由
    `_theta_from_image`（图像空间）给，两个搜索**共用同一个 θ 锚点**，
    相位不再错开 —— 于是它们的分歧暴露成"**差一整格**"：实测两帧 E1 夹角恰好
    `0.2500°`（一个细扫步长）、origin 差 3.736mm、四角差最大 3.890mm。
    根因还是那个老问题：**覆盖度地形是平的**（同一掩码四个象限
    score 0.667/0.700/0.700/0.683），平局由粗搜的胜负决定，两边不保证一致。
    这不是"跟丢"，是解本身在 5mm 内不可分辨。
    """
    out = []
    truth = make_truth()
    o = np.asarray(truth['origin'], float)
    e1 = np.asarray(truth['E1'], float)
    e2 = np.asarray(truth['E2'], float)
    nrm = np.asarray(truth['nrm'], float)

    def perturb(da, db, dth_deg):
        """造一个"上一帧"的 frame：在台面系内平移 + 绕法向转。"""
        t = np.radians(dth_deg)
        c, s = np.cos(t), np.sin(t)
        return dict(origin=o + da * e1 + db * e2, E1=c * e1 + s * e2,
                    E2=-s * e1 + c * e2, nrm=nrm, W=truth['W'], H=truth['H'])

    color, depth = render(truth, ())
    kw = dict(normal=nrm, target_mm=(W_MM, H_MM), long_side_parallel=True,
              opts=dict(voxel_mm=2.0))
    ref, dref = alg.detect_pallet_frame(color, depth, K, **kw)
    qref = _corners_mm(ref, ref['W'], ref['H'])

    # (名字, 台面系内平移 a/mm, b/mm, 绕法向转角/°, 期望用先验?)
    #
    # ⚠️ **窗口 2026-09-29 从 150mm / 5° 收到 60mm / 3°**（现场实测相邻帧位移
    # 最大 9.9mm、dθ 最大 0.47°，收窄把 `search` 段从 175~200ms 压到 66~80ms）。
    # 这张表里的偏移量是**相对窗口**定的，判据是"窗内 -> 用先验、窗外 -> 退回
    # 全局"，与窗口的绝对值无关，但**数值要跟着窗口一起缩**。
    cases = [
        ('先验=真值', 0.0, 0.0, 0.0, True),
        ('先验偏 20mm', 20.0, 0.0, 0.0, True),
        ('先验偏 40mm / 40mm', 40.0, 40.0, 0.0, True),
        ('先验偏 50mm / 2°', 50.0, 0.0, 2.0, True),
        ('先验偏 80mm（刚出窗）', 80.0, 0.0, 0.0, False),
        ('先验偏 100mm', 100.0, 0.0, 0.0, False),
        ('先验偏 160mm', 160.0, 0.0, 0.0, False),
        ('先验转角偏 6°', 0.0, 0.0, 6.0, False),
    ]
    for name, da, db, dth, want_prior in cases:
        f, di = alg.detect_pallet_frame(color, depth, K, prior=perturb(da, db, dth), **kw)
        if f is None:
            ok = False
            extra = f'**拒绝了** reject={di["reject"]}'
        else:
            dq = float(np.abs(np.asarray(_corners_mm(f, f['W'], f['H'])) - qref).max())
            # 2mm 的来由见 docstring（θ 网格相位差半个步长 = 1.7mm），不是拍的
            ok = (di['prior_used'] == want_prior and dq <= TOL_PRIOR_CORNER_MM)
            extra = (f'用先验={di["prior_used"]}（期望 {want_prior}） '
                     f'reason={di["prior_reason"]} score={di["score"]:.4f} 四角差={dq:.3f}mm')
        print(f'  {name:26s} {"OK " if ok else "**FAIL**"} {extra}')
        out.append(ok)

    # 反面用例：score 拦不住漂移
    f, di = alg.detect_pallet_frame(color, depth, K, prior=perturb(250.0, 0.0, 0.0), **kw)
    low = (di['score'] is not None and di['score'] > alg.COVERAGE_MIN)
    print(f'  {"score 拦不住漂移（反面）":26s} {"OK " if low else "**FAIL**"} '
          f'先验偏 250mm 时 score={di["score"]} > COVERAGE_MIN={alg.COVERAGE_MIN} —— '
          f'所以回退**只能**靠 window_saturated，不能只看 score')
    out.append(low)
    return out


def test_fit_floor_normal() -> list[bool]:
    """T10：从深度拟合法向 —— 初值被人为拧歪 12.4°，看能不能救回来。

    ⚠️ **判据是"拧歪的初值能不能被救回来"，不是"拟合值 == 真值"**：合成场景里
    地面是台面平面**沿法向平移** `floor_drop_mm` 得到的，方向与台面**相同**，
    所以拟合结果本来就该等于真值法向。但直接断言相等会退化成"实现细节的回归"
    （RANSAC 是抽样的），所以写成**上界**：与真值夹角 < 1°（实测 0.0x°）、
    内点数够（说明真找到了那个大平面）。

    12.4° 是 2026-09-29 现场 TF 链实测的偏差量级（见 `FIT_NORMAL_*` 的注释），
    而窗是 ±25° —— 这正是要覆盖的"窗内但初值明显歪"的真实情况。
    """
    out = []
    truth = make_truth()
    _color, depth = render(truth)
    nrm = np.asarray(truth['nrm'], float)

    # 把初值拧歪 12.4°（绕一个与法向不平行的轴，否则等于没拧）
    axis = _unit(np.array([1.0, 0.0, 0.0]) - (np.array([1.0, 0.0, 0.0]) @ nrm) * nrm)
    a = np.radians(12.4)
    prior = _unit(nrm * np.cos(a) + np.cross(axis, nrm) * np.sin(a))
    ang_prior = np.degrees(np.arccos(np.clip(float(prior @ nrm), -1, 1)))

    got, n_inl = alg.fit_floor_normal(depth, K, prior)
    if got is None:
        print(f'  **FAIL** 拟合失败（初值偏 {ang_prior:.1f}°）')
        return [False]
    ang = np.degrees(np.arccos(np.clip(abs(float(got @ nrm)), -1, 1)))
    ok = ang < 1.0 and n_inl >= alg.FIT_NORMAL_MIN_INLIERS
    print(f'  初值偏 {ang_prior:5.1f}° -> 拟合偏 {ang:5.2f}°  内点 {n_inl}  '
          f'{"OK " if ok else "**FAIL**"}')
    out.append(ok)

    # 符号约定：结果与初值同侧（n·prior > 0）—— 调用方后续约定依赖这一条
    out.append(float(got @ prior) > 0.0)

    # 空深度 -> 失败而不是崩
    got2, n2 = alg.fit_floor_normal(np.zeros_like(depth), K, prior)
    out.append(got2 is None and n2 == 0)

    # 初值退化成零向量 -> 失败而不是除零
    got3, n3 = alg.fit_floor_normal(depth, K, np.zeros(3))
    out.append(got3 is None and n3 == 0)

    # 初值拧出窗口（80°）-> 失败。**调用方据此回退初值**，
    # 而不是拿一个窗外的平面硬用（那正是"锁到墙上"）。
    far = _unit(nrm * np.cos(np.radians(80.0))
                + np.cross(axis, nrm) * np.sin(np.radians(80.0)))
    got4, n4 = alg.fit_floor_normal(depth, K, far)
    out.append(got4 is None and n4 == 0)

    # 固定种子 -> 同一帧两次调用逐位相同（回归测试才钉得住它）
    r1, _ = alg.fit_floor_normal(depth, K, prior)
    r2, _ = alg.fit_floor_normal(depth, K, prior)
    out.append(r1 is not None and np.array_equal(r1, r2))
    return out


ALL_TESTS = [
    test_basis,
    test_deck_mask,
    test_deck_mask_deadcode,
    test_deck_mask_depth_path,
    test_theta_ref,
    test_detect_full,
    test_detect_degenerate,
    test_mask_area_ratio,
    test_detect_theta_ref_offset,
    test_prior_track,
    test_fit_floor_normal,
    test_theta_from_image,
]

# 每个 test_* 返回一个 bool 列表；这里逐条转成 check() 记进 _fails。
_GROUPS = [
    ('T2 占位基', test_basis),
    ('T3 台面掩码', test_deck_mask),
    ('T3 扣货死代码', test_deck_mask_deadcode),
    ('T3 深度路线', test_deck_mask_depth_path),
    ('T4 参考角 θ_ref', test_theta_ref),
    ('T5 检测 4 边', test_detect_full),
    ('T5 检测 退化', test_detect_degenerate),
    ('T5c 掩码面积判据', test_mask_area_ratio),
    ('T5 θ_ref 偏移 ±20°', test_detect_theta_ref_offset),
    ('T9b 先验跟踪', test_prior_track),
    ('T10 深度拟合台面法向', test_fit_floor_normal),
    ('T11 图像空间 θ', test_theta_from_image),
]


def main() -> None:
    truth = make_truth()
    print('真值四角像素:\n' + str(np.round(_quad_px(truth), 1)))
    print(f'  origin = {np.round(truth["origin"], 1)}')
    print(f'  E1 = {np.round(truth["E1"], 3)}  (应当 x>0：指向画面右)')
    print(f'  E2 = {np.round(truth["E2"], 3)}  (应当 y<0：指向上)')
    print('渲染器自检（合成器本身）：')
    color, depth = render(truth)
    m = mask_from_truth(truth)
    print(f'  图像 {color.shape}  深度 {depth.shape}')
    print(f'  真值掩码 {int(m.sum())} px  深度有效 {int((depth > 0).sum())} px')
    assert int(m.sum()) > 10000, '真值掩码太小，渲染有问题'
    assert int((depth > 0).sum()) > 10000, '深度有效点太少，渲染有问题'

    total = passed = 0
    for name, fn in _GROUPS:
        print(f'\n=== {name} ===')
        res = fn()
        for i, ok in enumerate(res):
            check(f'{name} #{i}', bool(ok))
        total += len(res)
        passed += sum(bool(v) for v in res)
        print(f'  {name}: {sum(bool(v) for v in res)}/{len(res)}')
    print(f'\n{passed}/{total} 通过')
    if _fails:
        print(f'{len(_fails)} 项失败：{", ".join(_fails)}')
        raise SystemExit(1)
    print('全部通过')


if __name__ == '__main__':
    main()
