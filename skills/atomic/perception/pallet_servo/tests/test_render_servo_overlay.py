#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`render_servo_overlay` 的数值自检。跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 skills/atomic/perception/pallet_servo/tests/test_render_servo_overlay.py

**不需要 ROS、不需要相机、不需要任何数据** —— 自己合成一张图和一个 ServoError。

**需要 `colorlog`**（`core.common.logger` 的依赖，在 `requirements.txt` 里）：
`render.py` 用 `get_logger` 报"字体找不到 / 台面投影失败"这类**人要看**的警告。
本机系统 python3 没装它（`/usr/bin/python3` → `ModuleNotFoundError: No module
named 'colorlog'`），所以下面这条命令要在装了 `requirements.txt` 的解释器里跑
（本次实测用的是 `~/miniconda3/envs/sixd/bin/python`）。

**为什么出图必须自检**：人眼看到叠加图不对时，第一嫌疑应该是画图代码，不是数据。
一张画错了的图会让人去怀疑本来正确的算法，改坏它。

两个夹具
--------
`_straight_down()` —— 一般的**偏离态**：箱子转过一个角、离参考边有几十像素，
五样东西都在图内，用来钉住"每个图层都画了、探针都取得到色"。

`_in_position()` —— **伺服到位态**（`e_bottom ≈ e_right ≈ theta ≈ 0`）：箱子四角
**故意取成托盘台面四角本身**（按契约顺序 `[右下, 左下, 左上, 右上]` 重排），于是
箱子底边 = 参考边 `y=0`、箱子右边 = 参考边 `x=W` = 台面矩形的下边/右边 ——
**四条线共线**。这是本轮修复的核心判据：修复前这一帧的 8 条探针全红（后画的线
把先画的整条盖住），修复后必须**返回空**。夹具用的是反向重合（箱子底边是
`(1240,50)→(40,50)`，参考边是 `(40,50)→(1240,50)`），顺带钉住 `_same_pixels`
**必须判反向**。
"""
from __future__ import annotations

import sys
from pathlib import Path

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
    __package__ = 'skills.atomic.perception.pallet_servo.tests'
    import importlib
    importlib.import_module(__package__)

import numpy as np

from skills.atomic.perception.pallet_servo.algorithm import (
    BoxObservation,
    ServoError,
    project_pallet_points,
    servo_error,
)
import cv2

from skills.atomic.perception.pallet_servo.render import (
    COLOR_BOX_EDGE,
    COLOR_FOOT,
    COLOR_INWARD,
    COLOR_OFFSET,
    COLOR_PALLET_QUAD,
    COLOR_REF_EDGE,
    COLOR_YOLO_BOX,
    _merge_collinear,
    _same_pixels,
    _snap_probes_to_drawn,
    cjk_font,
    numeric_self_check,
    render_servo_overlay,
)

FAILS = []


def check(name, cond, detail=""):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        FAILS.append(name)


FX = FY = 1000.0
CX, CY = 640.0, 400.0
PALLET_W_MM, PALLET_H_MM = 1200.0, 800.0
K_MAT = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])

# 转过角的箱子（契约顺序：右下 → 左下 → 左上 → 右上）
QUAD = [[980.0, 1080.0], [700.0, 1120.0], [680.0, 800.0], [960.0, 760.0]]

# 画布尺寸。**与传给 `servo_error` 的 `image_size` 必须一致**：不一致会让探针
# 越界（自检会报"探针落在画布外"，那是好事，不是静默）。
CANVAS_W, CANVAS_H = 1280, 1200
IMAGE_SIZE = (CANVAS_W, CANVAS_H)

# 自检用的底色。故意用一个**既不是 0 也不是任何图层色**的值：文字判据靠它
# （"这块该有文字的地方出现了既非底色、也非线色的像素"）。
BG_FILL = 7
LAYER_COLORS = (COLOR_REF_EDGE, COLOR_BOX_EDGE, COLOR_INWARD, COLOR_FOOT,
                COLOR_OFFSET, COLOR_PALLET_QUAD, COLOR_YOLO_BOX)


def _straight_down():
    """相机正对托盘、深度 1 m。

    ⚠️ 平移**不是 brief 里的 `(0, 0, 1.0)`**，brief 那个值配这份夹具是坏的：
    托盘台面 1200x800 mm 在 1 m 深、fx=1000 下投影成 u ∈ [640, 1840]、
    v ∈ [400, 1200]，超出 brief 那个 1280x800 的画布 —— 箱子四角会被裁掉。
    （2026-09-30 之前 `x=W` 那条参考边两个端点出图还会被 `edge_off_image` 拒；
    那条判据已删，所以现在纯粹是"画布装不下"的问题。）

    这里只动 `T` 的平移，让**台面中心落在光轴上**（`u = cx`、`v = cy`）——
    于是四个角投在 u ∈ [40, 1240]、v ∈ [50, 850]。`QUAD`、`size_mm`、`K`
    全部照 brief 原值没动，只把画布从 800 高放到 1200 高（箱子四角在
    v ≈ 760~1120，800 高会把它们裁掉）。这样五样东西**都在图内**，
    自检探针才有意义。
    """
    T = np.eye(4)
    T[:3, 3] = (-0.6, -0.35, 1.0)
    return T


def _in_position_error(T):
    """**伺服到位态**的 `ServoError`：三个被控量都 ≈ 0。

    做法是把箱子四角**取成托盘台面四角本身**（`_project_pallet_quad` 的顺序是
    `[左下, 右下, 右上, 左上]`，而箱子的契约顺序是 `[右下, 左下, 左上, 右上]`，
    重排一次），于是：

        箱子底边 = 台面下边 = 参考边 y=0
        箱子右边 = 台面右边 = 参考边 x=W

    **四条线共线**（而且是反向重合：箱子底边是 `(1240,50)→(40,50)`，参考边是
    `(40,50)→(1240,50)`）。这不是像素巧合，是几何上同一段三维线段。

    托盘在图像里是轴对齐的矩形，所以 `theta` 精确为 0；箱子中点与参考边中点
    也精确重合，所以 `e_bottom` / `e_right` 精确为 0。
    """
    quad = _project_pallet_quad_px(T)
    box_quad = [quad[1], quad[0], quad[3], quad[2]]      # 左下→右下→右上→左上
    err = servo_error(T, (PALLET_W_MM, PALLET_H_MM), ["y=0", "x=W"],
                      BoxObservation(quad=[list(p) for p in box_quad]), K_MAT,
                      image_size=IMAGE_SIZE)
    assert isinstance(err, ServoError), f"到位态夹具坏了：{err}"
    return err


def _project_pallet_quad_px(T):
    """台面四角的像素坐标 `[左下, 右下, 右上, 左上]`（与 render 内部同一套）。"""
    pts_mm = [(0.0, 0.0, 0.0), (PALLET_W_MM, 0.0, 0.0),
              (PALLET_W_MM, PALLET_H_MM, 0.0), (0.0, PALLET_H_MM, 0.0)]
    px = project_pallet_points(T, pts_mm, K_MAT)
    assert not hasattr(px, "code"), f"台面投影失败：{px}"
    return np.asarray(px, np.float64)


def _render(color, err, T, *, with_yolo=True, hud=True):
    kwargs = dict(error=err, T_cam_pallet=T,
                  size_mm=(PALLET_W_MM, PALLET_H_MM), K=K_MAT)
    if with_yolo:
        kwargs["yolo_uv"] = (680.0, 760.0, 980.0, 1120.0)
    if hud:
        kwargs["hud_lines"] = ("e_bottom=+0.0px", "e_right=+0.0px",
                               "theta=+0.00deg")
    return render_servo_overlay(color, **kwargs)


def _line_pixels(canvas):
    """画布上**精确等于某个图层线色**的像素掩码（`_text_pixels` 的反面）。

    线一律 `LINE_8`（无抗锯齿），所以线像素**精确等于**某个 `COLOR_*`。
    """
    is_line = np.zeros(canvas.shape[:2], bool)
    for c in LAYER_COLORS:
        is_line |= np.all(canvas == np.asarray(c, np.uint8), axis=-1)
    return is_line


# PIL 抗锯齿会往字形墨迹框**外**溢出一圈半色调像素，所以判"差异落在标签上"
# 时把框松开这么一点。1 是抗锯齿的量级，不是试出来的常数。
_LABEL_PAD_PX = 1


def _label_ink_box(text):
    """`text` 在**同一个** `cjk_font()` 下、以锚点为原点量出来的墨迹框
    `(left, top, right, bottom)`。

    用被测代码自己的字体量，所以框的大小**跟着字号走** —— 把字号调大，
    判据跟着放大，不需要改任何常数。
    """
    from PIL import Image, ImageDraw
    draw = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    return draw.textbbox((0, 0), text, font=cjk_font())


def _text_pixels(patch):
    """该区域里**既不是底色、也不是任何图层线色**的像素掩码。

    这就是"文字像素"的判据。底色是已知的纯色（`BG_FILL`），线一律 `LINE_8`
    （无抗锯齿），所以线的像素**精确等于**某个 `COLOR_*`，不会被算成文字。
    """
    return np.any(patch != BG_FILL, axis=-1) & ~_line_pixels(patch)


def test_labels_actually_draw_text():
    """把 `_label` / `_draw_labels` 改成空操作必须让这条变红。

    中文标签是 spec §8.3 的门面（`cv2.putText` 认不了中文，必须走 PIL），
    而在修复前**没有任何一条断言画布上出现了文字像素** —— 评审实测把 `_label`
    改成空操作，21 条 check 全绿。

    锚点选的是 **HUD 第一行的 `(12, 24)`**，因为那里是**全画布唯一确定干净**的
    位置：图层全是线（`LINE_8`，颜色精确等于 `COLOR_*`），而这个区域里
    一个线像素都没有（实测 `layer_px=0`）—— 托盘台面左边在 u=40 且 v 从 50 起，
    箱子边在 v ≈ 760~1120，都够不着 v ∈ [24, 46)。所以这块区域里出现"非底色、
    非线色"的像素，**只可能是文字**。

    另加一条：图层标签 `托盘台面` 锚在 `quad[0]` 角点上。**这一条必须不带 HUD**
    （`hud_lines=()`）：带上 HUD 时该区域被 **HUD 白字的溢出**填满，实测（到位态、
    hud=3）该区域 `text_px=647`，而其中「托盘台面」自己的贡献是 **0** —— 评审
    实测「删掉『托盘台面』标签，整轮仍然全绿」，就是因为这条断言测的是别人的字。
    去掉 HUD 之后该区域里只有这一个标签能到达（实测 `hud=0` 时 `text_px=375`，
    删标签后必须变红，见本轮变异 V6）。
    """
    T = _straight_down()
    err = _in_position_error(T)
    canvas, _ = _render(np.full((CANVAS_H, CANVAS_W, 3), BG_FILL, np.uint8), err, T)

    # (a) 干净的 HUD 区域
    hud = canvas[24:46, 12:72]
    check("标签真的画出了文字像素（HUD 干净区域）", bool(_text_pixels(hud).any()),
          f"(12,24) 往右下 60x22 里没有文字像素（全 {BG_FILL} 或全是线色）")
    check("HUD 区域本身没有线经过（判据干净）",
          not any(np.all(hud == np.asarray(c, np.uint8), axis=-1).any()
                  for c in LAYER_COLORS),
          "HUD 区域里出现了线色，这条判据就不再干净")

    # (b) 图层标签 `托盘台面`：**不带 HUD**，锚在 quad[0] 角点上，该区域只有它能到达
    canvas_b, _ = render_servo_overlay(
        np.full((CANVAS_H, CANVAS_W, 3), BG_FILL, np.uint8), error=err,
        T_cam_pallet=T, size_mm=(PALLET_W_MM, PALLET_H_MM), K=K_MAT,
        yolo_uv=(680.0, 760.0, 980.0, 1120.0), hud_lines=())
    q0 = np.round(_project_pallet_quad_px(T)[0]).astype(int)
    patch = canvas_b[q0[1] + 5:q0[1] + 22, q0[0]:q0[0] + 60]
    check("标签真的画出了文字像素（托盘台面 锚点，无 HUD）",
          bool(_text_pixels(patch).any()),
          "锚点附近没有既非底色、也非图层线色的像素")


def test_ref_edge_labels_use_the_callers_text():
    """★ 参考边标签用**调用方给的原文**，不用写死的"底边"/"右边"。

    写死的话，`ref_edges=["y=H","x=0"]` 这种组合下标签是**错的** —— 操作员会
    照着图去核对哪条边是哪条，**指错边比不标更糟**。

    判据选 probe 名字而不是像素：`ref_specs` 的 `tag` 同时进 probe 名字和图上
    文字，断言前者既直接又不必找采样区域。

    ⚠️ **夹具必须是偏离态，不能用到位态。** 到位态下参考边与箱子边/台面边**共线**，
    `_merge_collinear` 只画优先级最高的那条、被跳过的连探针都不产生（见模块
    docstring）—— 那时 `托盘参考边 底边` 这个探针**根本不存在**，断言它等于在断言
    "共线合并没生效"。本用例要钉的是标签的来源，与合并是两码事。

    ⚠️ **两次渲染的 kwargs 必须逐字相同**（只差 `ref_edge_labels`），否则下面
    "线条像素完全相同"那一条比的是两张不同的图。所以这里用一个闭包把公共
    kwargs 钉住，而不是一边走 `_render`、一边裸调 —— `_render` 会带上
    `yolo_uv`/`hud_lines`，裸调不带，灰色的 YOLO 框只在一边出现。
    """
    T = _straight_down()
    err = servo_error(T, (PALLET_W_MM, PALLET_H_MM), ["y=0", "x=W"],
                      BoxObservation(quad=QUAD), K_MAT, image_size=IMAGE_SIZE)
    assert isinstance(err, ServoError), err
    check("参考边标签：夹具是偏离态（两条参考边都在）",
          isinstance(err.ref_bottom_px, np.ndarray) and
          isinstance(err.ref_right_px, np.ndarray), repr(err))

    def _draw(**extra):
        return render_servo_overlay(
            np.full((CANVAS_H, CANVAS_W, 3), BG_FILL, np.uint8), error=err,
            T_cam_pallet=T, size_mm=(PALLET_W_MM, PALLET_H_MM), K=K_MAT,
            yolo_uv=(680.0, 760.0, 980.0, 1120.0),
            hud_lines=("e_bottom=+0.0px", "e_right=+0.0px", "theta=+0.00deg"),
            **extra)

    # 默认（不给 ref_edge_labels）：退回位置名
    _, probes_default = _draw()
    names = " ".join(p[0] for p in probes_default)
    assert "托盘参考边 底边" in names, names
    assert "托盘参考边 右边" in names, names

    # 给了原文：用原文，且**不再出现**位置名
    canvas, probes = _draw(ref_edge_labels=("y=0", "x=W"))
    names = " ".join(p[0] for p in probes)
    assert "托盘参考边 y=0" in names, names
    assert "托盘参考边 x=W" in names, names
    assert "托盘参考边 底边" not in names, names

    # 画出来的图**必须自检通过** —— 换了标签不该把图弄坏
    problems = numeric_self_check(canvas, probes)
    assert not problems, "; ".join(problems)

    # ★ 换的只是文字，不是几何 —— 拆成两条各有各的牙：
    #   ① 画布差异必须**非空**（标签真的生效了，这是需求本身）；
    #   ② 差异必须**只**落在标签文字够得到的地方（几何没被顺手改坏）。
    #
    #   brief 原来那句"两组的线条像素必须完全相同"在这个夹具上**恒红**，而且
    #   红的原因与几何无关：参考边标签锚在 `ref[0]`（就画在黄线**本身上**）、
    #   颜色又**就是** `COLOR_REF_EDGE`，换个字形必然改变一批"精确等于黄线色"
    #   的像素（实测 42 px，全在两条黄线附近）。反向验证：把 `_draw_labels`
    #   换成空操作后，两次渲染的线条掩码逐位相同 —— 差异 100% 来自文字。
    #   **不为了迁就那条断言去改 `render.py` 的标签锚点/颜色**：那是画面的
    #   可读性，不该让一条测试的方便凌驾于它。
    canvas_default, _ = _draw()
    diff = np.any(canvas_default != canvas, axis=-1)
    assert diff.any(), "换了 ref_edge_labels 却一个像素都没变 —— 标签根本没生效"

    # ② 允许范围 = 两组标签文字本身的墨迹框（同一个 `cjk_font()`、同一个落笔点
    #    量出来，**不是试出来的数**）。抗锯齿的边界像素可能溢出墨迹框 1 px，
    #    所以松开 `_LABEL_PAD_PX`。
    anchors = ((np.asarray(err.ref_bottom_px)[0], "底边"),
               (np.asarray(err.ref_right_px)[0], "右边"),
               (np.asarray(err.ref_bottom_px)[0], "y=0"),
               (np.asarray(err.ref_right_px)[0], "x=W"))
    allowed = np.zeros(diff.shape, bool)
    for uv, tag in anchors:
        left, top, right, bottom = _label_ink_box(f"托盘{tag}")
        x0, y0 = int(round(float(uv[0]))), int(round(float(uv[1])))
        allowed[y0 + top - _LABEL_PAD_PX:y0 + bottom + _LABEL_PAD_PX,
                x0 + left - _LABEL_PAD_PX:x0 + right + _LABEL_PAD_PX] = True
    stray = int((diff & ~allowed).sum())
    assert stray == 0, (
        f"{stray} 个像素在标签墨迹框之外变了 —— 标签的实现动到了几何"
        f"（差异共 {int(diff.sum())} px）")

    print(f"    参考边标签：用调用方原文；差异 {int(diff.sum())} px 且全落在标签"
          f"墨迹框内（几何未动）")


def test_in_position_frame_is_green():
    """**V1（本轮核心判据）**：到位态那一帧，`numeric_self_check` 必须返回空。

    修复前：四条线共线，后画的整条盖住先画的，**8 条探针全红**
    （托盘参考边 底/右、托盘台面 边0/边1、箱子 底/右、垂距线 底/右）——
    Task 5 会因此拒绝发图，而那正是操作员最想看的一帧。
    """
    T = _straight_down()
    err = _in_position_error(T)
    check("到位态：三个被控量都 ≈ 0",
          abs(err.e_bottom_px) < 1e-6 and abs(err.e_right_px) < 1e-6
          and abs(err.theta_rad) < 1e-9,
          f"e_bottom={err.e_bottom_px!r} e_right={err.e_right_px!r} "
          f"theta={err.theta_rad!r}")

    canvas, probes = _render(
        np.full((CANVAS_H, CANVAS_W, 3), BG_FILL, np.uint8), err, T)
    names = [p[0] for p in probes]
    problems = numeric_self_check(canvas, probes)
    check("到位态：数值自检返回空（V1 核心）", not problems, "; ".join(problems))

    # 重合的线只画一条：被跳过的既不画也不探
    check("到位态：重合的箱子边取代了参考边（箱子边优先级最高）",
          "箱子 底边" in names and "箱子 右边" in names
          and not any(n.startswith("托盘参考边") for n in names), str(names))
    check("到位态：被台面矩形盖住的边也不再画（不产生探针）",
          not any(n.startswith("托盘台面 边0") or n.startswith("托盘台面 边1")
                  for n in names), str(names))
    check("到位态：垂距为零 → 不画垂距线、不画垂足，也不探这两者",
          not any(n.startswith("垂距线") or n.startswith("垂足") for n in names),
          str(names))
    check("到位态：不重合的线照画（台面 边2/边3、内法向）",
          "托盘台面 边2" in names and "托盘台面 边3" in names
          and "内法向 底边" in names and "内法向 右边" in names, str(names))
    check("到位态：没有同名重复探针（每条线最多画一次）",
          len(names) == len(set(names)), str(names))


def test_coincidence_rule():
    """`_same_pixels` 必须**判反向**、必须判"取整后同像素"，`_merge_collinear`
    必须按优先级留一条。

    ⚠️ 判据是**取整后的整数端点相等**，不是浮点相等：`_draw_segment` 把端点吸附到
    整数像素，相差 0.4 px 的两条线**像素上只有一条**。判浮点相等会让整个
    `|d| ≤ 0.5 px` 的亚像素带都红（伺服收敛后箱子正停在这一带）。
    """
    a = np.array([[0.0, 0.0], [10.0, 0.0]])
    check("同向重合被认出来", _same_pixels(a, np.array([[0.0, 0.0], [10.0, 0.0]])))
    check("**反向**重合也被认出来（参考边 x=W 与台面边1 就是这种）",
          _same_pixels(a, np.array([[10.0, 0.0], [0.0, 0.0]])))
    check("平移一像素不算重合", not _same_pixels(a, np.array([[0.0, 1.0], [10.0, 1.0]])))
    check("共线但不同长度不算重合",
          not _same_pixels(a, np.array([[0.0, 0.0], [20.0, 0.0]])))
    # 亚像素带：0.4 px 的差取整后是同一批像素 —— 必须判成重合（F1 的核心）
    check("相差 0.4 px 判成同一批像素（亚像素带，F1 核心）",
          _same_pixels(a, np.array([[0.0, 0.4], [10.0, 0.4]])))
    check("相差 0.4 px 反向也判成同一批像素",
          _same_pixels(a, np.array([[10.0, 0.4], [0.0, 0.4]])))
    check("相差 0.6 px 取整后不同像素，不算重合",
          not _same_pixels(a, np.array([[0.0, 0.6], [10.0, 0.6]])))

    # 优先级：箱子 > 参考边 > 台面矩形 > YOLO 框；同一条线只留优先级最高的那个 key
    seg = ((0.0, 0.0), (10.0, 0.0))
    kept = _merge_collinear([("box", seg), ("ref", seg),
                             ("quad", ((10.0, 0.0), (0.0, 0.0))),
                             ("yolo", ((0.0, 0.4), (10.0, 0.4))),
                             ("other", ((0.0, 5.0), (10.0, 5.0)))])
    check("重合时只留优先级最高的 key", "box" in kept and "ref" not in kept
          and "quad" not in kept, str(kept))
    check("YOLO 框边优先级最低：与箱子边同像素时被跳过（F3）",
          "yolo" not in kept, str(kept))
    check("不重合的线照留", "other" in kept, str(kept))


def _scan_canvas(cases, yolo=None, hud=("a", "b", "c")):
    """扫一批**自造四角**的箱子，返回 `(总帧数, 红帧列表)`。

    ⚠️ 箱子四角**必须自己构造**（`box_quad`），**不能**复用 `_project_pallet_quad_px`
    那条渲染器的投影路径 —— 用同一条路径造出来的 `d = 0` 那一帧与渲染器逐位相同、
    必然被合并，**是构造出来的绿**，红带会被跨过去（第一轮就栽在这里）。

    ⚠️ **两个维度都要扫**：偏移量 `e`（沿参考边内法向）与横向错开 `Δu`（沿参考边
    切向）。红带只在前者的亚像素带、以及后者的共线窗口里出现。
    """
    red = []
    total = 0
    for tag, quad in cases:
        err = servo_error(_straight_down(), (PALLET_W_MM, PALLET_H_MM), ["y=0", "x=W"],
                          BoxObservation(quad=[list(p) for p in quad]), K_MAT,
                          image_size=IMAGE_SIZE)
        if not isinstance(err, ServoError):
            continue
        total += 1
        kw = dict(error=err, T_cam_pallet=_straight_down(),
                  size_mm=(PALLET_W_MM, PALLET_H_MM), K=K_MAT, hud_lines=hud)
        if yolo is not None:
            kw["yolo_uv"] = yolo
        canvas, probes = render_servo_overlay(
            np.full((CANVAS_H, CANVAS_W, 3), BG_FILL, np.uint8), **kw)
        problems = numeric_self_check(canvas, probes)
        if problems:
            red.append((tag, problems))
    return total, red


# 托盘在图像里轴对齐：参考边 y=0 → v=50、x=W → u=1240，台面矩形 u∈[40,1240]、v∈[50,850]。
# 这几个值**自己量出来**（`_project_pallet_quad_px` 一次），后面箱子四角全按它们构造。
_PQ = None


def _pq():
    global _PQ
    if _PQ is None:
        _PQ = _project_pallet_quad_px(_straight_down())
    return _PQ


def _scan_edges():
    """托盘矩形的 (u_left, u_right, v_top, v_bottom) —— 轴对齐，取投影结果的极值。"""
    q = _pq()
    return (float(q[:, 0].min()), float(q[:, 0].max()),
            float(q[:, 1].min()), float(q[:, 1].max()))


def box_quad(e_bottom, e_right, du=0.0, dv=0.0, theta_deg=0.0):
    """**自己构造**箱子四角（契约顺序 `[右下, 左下, 左上, 右上]`）。

    `e_bottom` / `e_right` 是沿参考边**内法向**的偏移，`du` / `dv` 是沿参考边
    **切向**的横向错开。箱子与托盘同尺寸，所以 `e_bottom = e_right = 0` 时
    四条线共线（到位态）。

    `theta_deg` 是绕**箱子中心**的旋转（图像坐标系里顺时针为正，u 向右、v 向下）。
    ⚠️ **这一维不能省** —— 第一轮/第二轮的所有扫描（含 `test_convergence_scan`）
    全部是 `theta = 0`，于是漏掉了整整一类红帧：箱子转过去一点之后，箱子右边
    中点会横向漂到别处，而 `右边 垂距` 标签锚在**箱子边中点**上，正好盖住
    `托盘参考边 右边` 的探针（实测 `theta = -2°`、垂距 1~4 px 时 7 帧）。

    ⚠️ **参考边 `y=0` 投影在 `v = vt`（图像上边），不是 `vb`。** 内法向指向托盘
    内部 = `+v`，所以箱子底边的 `v = vt + e_bottom`；右边同理 `u = ur − e_right`。
    曾经写成 `vb + e_bottom` —— 那样"垂距 1.5 px"的用例实际测的是 `e ≈ 801 px`，
    判据是假的（`_MIN_OFFSET_PX` 的变异根本红不了）。
    """
    ul, ur, vt, vb = _scan_edges()
    u_r = ur - e_right + du
    v_b = vt + e_bottom + dv
    quad = np.array([[u_r, v_b], [ul + du, v_b],
                     [ul + du, vb + dv], [u_r, vb + dv]])
    if theta_deg:
        c = quad.mean(axis=0)
        th = np.radians(float(theta_deg))
        rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        quad = (quad - c) @ rot.T + c
    return quad


def test_convergence_scan():
    """**F5（本轮验收判据）**：整个收敛区间必须全绿 —— 三个维度都扫。

    修复前实测：亚像素 `d ∈ [0,2]` 步长 0.05 有 20 帧红（`|d| ≤ 0.5`），
    随机连续族 4000 帧有 28 帧红（0.70%）。这一条就是钉住它们的。

    粗扫只到 **300 px** 而不是 400：再往下箱子边整条跑到画布外，"探针落在画布外"
    是**几何事实**不是画图 bug（自检现在对"探针在外"直接跳过，见
    `numeric_self_check`），不属于本模块的收敛区间。

    五个维度（`e` 是沿内法向的偏移量、`du`/`dv` 是沿切向的横向错开、
    `theta` 是绕箱子中心的旋转）：

      | 维度 | 范围 | 步长 | 钉的是 |
      |---|---|---|---|
      | `e_bottom` | `[-2, 2]` | 0.05 | C1a 的亚像素红带 |
      | `e_right`  | `[-2, 2]` | 0.05 | 同上（另一轴）|
      | `e_*`      | `[0, 300]` | 1.0 | C1b 的箭头共线区 |
      | `du` / `dv`| `[0, 2]` | 0.05 | C1b 的触发条件（`|Δu| < 1.25`）|
      | `theta`    | `[-10, 10]` | 1.0 | **修复轮 3 新增**：旋转让箱子边中点横向漂移 |

    ⚠️ **`theta` 这一维是修复轮 3 补的，不是可有可无。** 前三轮（含本函数上一版）
    的所有扫描全部 `theta = 0`，于是整类红帧被漏掉：箱子转过去一点之后，
    `右边 垂距` 标签（锚在**箱子右边中点**上、色是 `COLOR_OFFSET` 红）会盖住
    `托盘参考边 右边` 的探针（实测 `theta = -2°`、垂距 1~4 px 时 7 帧），
    以及内法向箭头杆落回垂距线上（`theta = +2°`、垂距 3.5~8.5 px 时 4 帧）。
    前者由"F8 挪位改到文字之后"修掉，后者由"箭头切向挪开的方向背离箱子中点"修掉。

    `theta` 用**两个网格**：粗网格 `[-10, 10]` 步长 1.0 配 `e ∈ [0, 10]` 步长 0.5
    （441 帧，与独立扫描同口径），细网格 `[-2, 2]` 步长 0.25（红帧集中在
    `|theta| ≈ 2`，粗网格可能跨过去）。

    另加一类：**箱子边与托盘边共线但比托盘边短**（箱子居中附近，跨度盖住托盘边
    中点）—— 短的那条只盖住长的那条的中间一段，探针必须被挪到还看得见的一段上
    （F8），否则这一带（`e ≈ 0`，伺服收敛）会红。
    """
    e_axis = np.arange(-2.0, 2.0001, 0.05)
    # 粗扫到 300 px（不是 400）：再往下箱子底边/右边就整条跑到画布外了
    # （1280x1200 的画布，v 只有 1200、u 只有 1280），"探针落在画布外"是**几何
    # 事实**不是画图 bug —— 自检现在对"探针在外"直接跳过，不属于本模块的收敛区间。
    # 收敛区间本身（0~10 px）与 300 px 这一段全在内。
    coarse = np.arange(0.0, 300.0001, 1.0)
    lateral = np.arange(0.0, 2.0001, 0.05)

    cases = []
    cases += [(f"亚像素 e_bottom={d:+.2f}", box_quad(float(d), 60.0)) for d in e_axis]
    cases += [(f"亚像素 e_right={d:+.2f}", box_quad(60.0, float(d))) for d in e_axis]
    cases += [(f"粗扫 e_bottom={d:.0f}", box_quad(float(d), 60.0)) for d in coarse]
    cases += [(f"粗扫 e_right={d:.0f}", box_quad(60.0, float(d))) for d in coarse]
    cases += [(f"横向 du={d:+.2f}", box_quad(200.0, 60.0, du=float(d))) for d in lateral]
    cases += [(f"横向 dv={d:+.2f}", box_quad(60.0, 200.0, dv=float(d))) for d in lateral]

    # ---- F2（修复轮 3）：`theta` 维度。**所有旧扫描都漏了这一维。** ----
    # 两轴同偏移 `e`，配绕箱子中心的旋转 `theta`。粗网格与独立扫描同口径
    # （441 帧），细网格覆盖红帧密集的 `|theta| ≈ 2`。
    for th in np.arange(-10.0, 10.0001, 1.0):
        for e in np.arange(0.0, 10.0001, 0.5):
            cases.append((f"旋转+偏移 theta={th:+.1f} e={e:.2f}",
                          box_quad(float(e), float(e), theta_deg=float(th))))
    for th in np.arange(-2.0, 2.0001, 0.25):
        for e in np.arange(0.0, 10.0001, 0.5):
            cases.append((f"旋转细扫 theta={th:+.2f} e={e:.2f}",
                          box_quad(float(e), float(e), theta_deg=float(th))))

    # 共线但箱子比托盘边短：箱子底边贴着台面下边（±2 px 亚像素），横向错开 ±40 px，
    # 跨度仍盖住托盘边的中点（否则探针本来就不在被盖的那一段里，测不到 F8）。
    ul, ur, vt, vb = _scan_edges()
    for dv in np.arange(-2.0, 2.0001, 0.25):
        for off in np.arange(-40.0, 40.0001, 4.0):
            u_r = ur - 300.0 + off
            if u_r - 500.0 < ul:
                continue
            cases.append((f"共线短边 dv={dv:+.2f} off={off:+.0f}",
                          np.array([[u_r, vb + dv], [u_r - 500.0, vb + dv],
                                    [u_r - 500.0, vt], [u_r, vt]])))

    total, red = _scan_canvas(cases)
    check(f"收敛区间扫描全绿（{total} 帧，含亚像素带 + 横向错开 + 共线短边 + theta 旋转）",
          not red,
          f"{len(red)} 帧红，前 3 条：" + " | ".join(
              f"{t}: {'; '.join(p)}" for t, p in red[:3]))

    # ---- F4：`_MIN_OFFSET_PX` 的语义钉在 1.0（半个线宽），不是 2.0 ----
    # 2 px 的垂距**画得出来也看得见**；跳过它会让"图上没有垂距线、而 HUD 上
    # e_bottom 是个非零数"，图和数打架，而且探针集合会在阈值处从 15 条跳到 11 条。
    # 判据直接看**探针集合**（不是看自检红不红 —— 少了探针自检照样绿）。
    _c, probes_f4 = render_servo_overlay(
        np.full((CANVAS_H, CANVAS_W, 3), BG_FILL, np.uint8),
        error=servo_error(_straight_down(), (PALLET_W_MM, PALLET_H_MM),
                          ["y=0", "x=W"],
                          BoxObservation(quad=[list(p) for p in box_quad(1.5, 60.0)]),
                          K_MAT, image_size=IMAGE_SIZE),
        T_cam_pallet=_straight_down(), size_mm=(PALLET_W_MM, PALLET_H_MM), K=K_MAT,
        hud_lines=())
    names_f4 = [p[0] for p in probes_f4]
    check("垂距 1.5 px 仍然画垂距线 + 垂足（`_MIN_OFFSET_PX` = 1.0，F4）",
          "垂距线 底边" in names_f4 and "垂足 底边" in names_f4, str(names_f4))
    check("垂距 1.5 px 时自检也是绿的", not numeric_self_check(_c, probes_f4),
          "; ".join(numeric_self_check(_c, probes_f4)))


def main():
    T = _straight_down()
    err = servo_error(T, (PALLET_W_MM, PALLET_H_MM), ["y=0", "x=W"],
                      BoxObservation(quad=QUAD), K_MAT,
                      image_size=IMAGE_SIZE)
    check("servo_error 出得来（前置）", isinstance(err, ServoError), repr(err))
    if not isinstance(err, ServoError):
        print(f"\n失败 {len(FAILS)} 条：{FAILS}")
        return 1

    color = np.full((CANVAS_H, CANVAS_W, 3), 40, np.uint8)    # 灰底，BGR
    canvas, probes = _render(color, err, T)

    check("返回的是 BGR 三通道同尺寸图",
          canvas.shape == color.shape and canvas.dtype == np.uint8, str(canvas.shape))
    check("原图没被改", not np.array_equal(canvas, color) and color[0, 0, 0] == 40,
          "画布该是拷贝")
    check("探针非空", len(probes) > 0, str(len(probes)))

    # ---- 每个图层都在 ----
    names = " ".join(p[0] for p in probes)
    for want in ("托盘台面", "YOLO", "托盘参考边", "箱子", "垂足"):
        check(f"探针里有「{want}」", want in names, names)

    # ---- 核心：探针处就是目标颜色 ----
    problems = numeric_self_check(canvas, probes)
    check("数值自检通过", not problems, "; ".join(problems))

    # ---- 自检真的会红：故意把画布涂掉 ----
    # `probes` 是**渲染器已经挪过位**的那一份（F8 在 `render_servo_overlay` 内部
    # 跑完才返回），所以这里直接拿它探一张被涂黑的画布，走的就是"图被画坏"
    # 那条路 —— 每条线的本色像素一个不剩，自检必须报出来。
    ruined = canvas.copy()
    ruined[:, :] = 0
    problems = numeric_self_check(ruined, probes)
    check("自检能发现画错了（反向用例）", len(problems) > 0,
          "把画布涂黑之后自检必须报错")

    # ---- F8 的反向用例：挪位**不能**把"没画"救回来 ----
    # 整条线一个本色像素都没有时必须保持几何中点、照旧报红。用一条远离任何
    # 图层的假线段构造：探针挪遍整条也找不到本色。
    fake = [("假线", (700.0, 700.0), (123, 45, 67))]
    seg = ((640.0, 700.0), (760.0, 700.0))
    _snap_probes_to_drawn(canvas, fake, [(0, seg, (123, 45, 67))])
    check("F8：整条线都没画时探针**不动**（保持几何中点，照旧报红）",
          fake[0][1] == (700.0, 700.0), str(fake[0][1]))
    check("F8：整条线都没画时自检仍然红",
          len(numeric_self_check(canvas, fake)) == 1,
          "; ".join(numeric_self_check(canvas, fake)))
    # 正向：把那条线画上本色之后，挪位必须能找到它
    drawn = canvas.copy()
    cv2.line(drawn, (640, 700), (760, 700), (123, 45, 67), 2, lineType=cv2.LINE_8)
    moved = [("假线", (700.0, 700.0), (123, 45, 67))]
    _snap_probes_to_drawn(drawn, moved, [(0, seg, (123, 45, 67))])
    check("F8：画上了本色时自检绿", not numeric_self_check(drawn, moved),
          "; ".join(numeric_self_check(drawn, moved)))

    # ---- 探针落在画布外 -> 一律跳过，不算问题（2026-09-30 第二次修）----
    #
    # ⚠️ 这条判据改过两次，两次都是**现场假报**逼出来的：
    #   第一次：箱子下边出画，探针（线段中点）在外 -> 整帧不发图。
    #           放宽成"线段有一截进得来就跳过"。
    #   第二次：**垂足**出画 —— 它是个**单点**探针，没有"另一截"可挪，
    #           第一版的放宽救不了它（`垂足 底边 (645, 827)`，图高 800）。
    #   现在：**只要探针在画布外就跳过**。
    #
    # 为什么彻底去掉："在画布外"对"画图代码有没有 bug"没有分辨力。`cv2.line`
    # 自己会裁 —— 完全在外面的线段本来就画不出来，有一截进来的一定被画上。
    # 所以它只说明"这一帧几何跑出了画面"，那是正常工况。
    #
    # ⚠️ **代价明说**：探针在外不再有任何告警。信息还在（那一层就是没画出来）。

    # (a) 单点探针在画布外（就是 `垂足` 那个形状）-> 跳过
    #     ⚠️ 探针坐标要相对**本用例的画布**（`CANVAS_W=1280, CANVAS_H=1200`）取；
    #     现场那个 `(645, 827)` 配的是 1280x800 的真机图，在 1200 高的画布上
    #     是**在画布内**的，直接抄过来测不到这条分支。
    single_out = [("垂足 底边", (645.0, 1230.0), COLOR_FOOT)]
    check("单点探针在画布外 -> 跳过（现场 `垂足 底边` 的形状）",
          not numeric_self_check(canvas, single_out),
          "; ".join(numeric_self_check(canvas, single_out)))

    # (b) 整条线段都在画布外 -> 也跳过（画不出来，也就无所谓画错）
    off = [("越界线", (-10.0, 5.0), COLOR_REF_EDGE),
           ("越界线", (-10.0, 60.0), COLOR_REF_EDGE)]
    check("整条在画布外的线也跳过", not numeric_self_check(canvas, off),
          "; ".join(numeric_self_check(canvas, off)))

    # (c) **反向**：画布内的探针取不到本色，照旧要报 —— 判据不能松到没有
    dark = np.zeros_like(canvas)
    probe_in = [("箱子 底边", (640.0, 700.0), COLOR_BOX_EDGE)]
    probs_in = numeric_self_check(dark, probe_in)
    check("画布内取不到本色仍然要报（判据没松过头）",
          len(probs_in) == 1 and "没有目标颜色" in probs_in[0],
          "; ".join(probs_in))

    # ---- 颜色约定（与 apps 那份 render_overlay 一致，不新造一套）----
    check("参考边是黄", COLOR_REF_EDGE == (0, 220, 255), str(COLOR_REF_EDGE))
    check("箱子边是绿", COLOR_BOX_EDGE == (0, 255, 120), str(COLOR_BOX_EDGE))
    check("内法向是蓝", COLOR_INWARD == (255, 160, 0), str(COLOR_INWARD))
    check("垂足是紫", COLOR_FOOT == (255, 80, 200), str(COLOR_FOOT))
    check("垂距线是红", COLOR_OFFSET == (0, 0, 255), str(COLOR_OFFSET))
    check("托盘台面是青（新增）", COLOR_PALLET_QUAD == (255, 255, 0),
          str(COLOR_PALLET_QUAD))
    check("YOLO 框是灰（新增）", COLOR_YOLO_BOX == (160, 160, 160),
          str(COLOR_YOLO_BOX))

    # ---- 不传 yolo_uv 也要能跑 ----
    canvas2, probes2 = _render(color, err, T, with_yolo=False, hud=False)
    check("没有 yolo_uv 也能出图", not numeric_self_check(canvas2, probes2),
          "; ".join(numeric_self_check(canvas2, probes2)))
    check("没有 yolo_uv 时探针里没有 YOLO",
          "YOLO" not in " ".join(p[0] for p in probes2))



    # ---- 修复轮新增（F1 / F2 / F3 / F4 / F5 / F6 / F8）----
    print()
    test_labels_actually_draw_text()
    test_in_position_frame_is_green()
    test_coincidence_rule()
    test_convergence_scan()
    test_ref_edge_labels_use_the_callers_text()

    print()
    if FAILS:
        print(f"失败 {len(FAILS)} 条：{FAILS}")
        return 1
    print("全部通过")
    return 0


if __name__ == '__main__':
    sys.exit(main())
