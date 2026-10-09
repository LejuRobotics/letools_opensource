# -*- coding: utf-8 -*-
"""把伺服误差画成一张对比图，并在给人看之前先做数值自检。

**纯函数、零状态、不 import ROS。**

**为什么要自检**：人眼看到叠加图不对时，第一嫌疑应该是画图代码，不是数据。
一张画错了的图会让人去怀疑本来正确的算法，改坏它。所以画完立刻在"本该是某个
颜色"的位置采样，颜色不对就把问题**打印出来**，而不是照样发出去让人看。

图上的中文**一律走 PIL 画**，不用 `cv2.putText`（它只认 ASCII，中文会变成一串
问号）。

配色与 `apps/test_camera_internal/pallet_servo_sim/render_overlay.py` **一致**，
而且**现在就是从这里取的** —— 那边已经不再自己定义那五个颜色与三个绘制原语
（`draw_segment` / `draw_arrow` / `round_xy`）和 CJK 字体那套，改成
`from skills.atomic.perception.pallet_servo.render import ...`。**这里现在是唯一
出处**，改配色只改这一处。本模块另有两个特有图层（托盘台面矩形、YOLO 输入框）。

画序的一个非显然点 —— **已被"共线合并"取代**
------------------------------------------------
默认 `ref_edges=["y=0","x=W"]` 时，参考边**就是**台面矩形的下边和右边
（参考边本来就该贴在台面边上，见 `algorithm.ref_edge_points_mm` 的表与
`_project_pallet_quad` 的 `pts_mm`），两者**在几何上是同一对三维线段**。伺服
到位时箱子底边/右边也落到同一批直线上 —— 四条线共线。

**处理办法不是调画序，也不是放宽自检容差，而是承认它们本来就是同一条线：
画出同一批像素的线段只画优先级最高的一条，被跳过的既不画也不探。** 优先级
（高 → 低）：

    箱子底边/右边  >  参考边 底/右  >  台面矩形 边0..3  >  YOLO 框 四条边

箱子边优先级最高是因为**箱子边是测量结果、参考边是目标**：两者重合 = 到位，
这时图上显示绿色（"箱子已经贴到托盘边上了"）比显示黄色信息量更大。YOLO 框最低
是因为它是**输入**、箱子边是**精修结果**（AABB 兜底路径下两者是同一条线段）。

⚠️ **判据是"取整后的整数端点相等"，不是浮点相等**（`_same_pixels`）：`draw_segment`
把端点吸附到整数像素，相差 0.4 px 的两条线**像素上只有一条**。判浮点相等会让
`|d| ≤ 0.5 px` 的整个亚像素带都红 —— 而伺服收敛后箱子正停在这一带里。
这里**没有容差常数**，判据是离散的、精确的。

⚠️ **别再走"调画序"那条老路。** 曾经把台面矩形挪到参考边之后画、靠"3 px 黄线
从两侧各露 1 px"让两个探针都取到色 —— 那只是**换哪一条失败**：偏移量在 0~7 px
的整个收敛区间里，垂足圆/文字总会把露出来的那 1 px 盖掉，自检照样红。实测
（修复前，伺服到位夹具）**8 条探针全红**。重合是几何事实，不是像素巧合，只能按
几何处理。

**被跳过的线不产生探针。** 探针的语义是"这条线画对了吗" —— 没画的线没有探针
可失败。这不是顺手删，是修复的关键。

**垂距为零时不画垂距线、不画垂足。** `e ≈ 0` 时垂距线的两个端点重合，画出来
就是一个点，还必然被垂足圆盖住；语义上也是对的 —— 垂距为零时本来就没有垂距
可看。

**内法向箭头沿参考边切向挪开 10 px，并改到实线与垂距线之后画。** 垂距线在几何
上**必然沿内法向**，所以 θ=0 时箭头与它共线，后画的红线整条盖住蓝箭头。挪开是
避让不是审美（推导见 `_ARROW_SIDE_PX`），改画序是双保险。

标签的落笔时机
--------------
各图层**不再就地调 PIL**，而是把 `(锚点, 文字, 颜色)` 攒进 `pending_labels`，
最后**一次 PIL 会话**画完（`_draw_labels`）。这不是画序变化（文字仍然最后一遍
画），只是把 N 次 4.6 MB 画布的 BGR↔RGB 往返压成 1 次：`_label` 单次实测
17.6 ms，7 个图层标签 + 3 行 HUD ≈ 180 ms，而 Task 5 会在伺服 `update()` 里
调它，伺服 tick 是 10 Hz = 100 ms —— 不压这一下那一 tick 直接超时。
"""
from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from core.common.logger import get_logger

try:                                                        # pragma: no cover
    from PIL import Image, ImageDraw, ImageFont
except ImportError:                                          # pragma: no cover
    Image = None

logger = get_logger(__name__)

# 每种元素一个固定颜色（BGR，因为画布是 cv2 的通道序）。
# ⚠️ **前五个的定义已经从 `apps/.../render_overlay.py` 删掉、改成从这里 import** ——
# 两个工具看同一套图，配色分叉会让人以为是不同的东西。这里现在是**唯一出处**。
COLOR_REF_EDGE = (0, 220, 255)      # 黄：托盘参考边
COLOR_BOX_EDGE = (0, 255, 120)      # 绿：箱子底边/右边
COLOR_INWARD = (255, 160, 0)        # 蓝：内法向箭头
COLOR_FOOT = (255, 80, 200)         # 紫：垂足
COLOR_OFFSET = (0, 0, 255)          # 红：垂距线
# 本模块特有的两个
COLOR_PALLET_QUAD = (255, 255, 0)   # 青：托盘台面矩形
COLOR_YOLO_BOX = (160, 160, 160)    # 灰：YOLO 输入框

Probe = Tuple[str, Tuple[float, float], Tuple[int, int, int]]
Label = Tuple[Tuple[float, float], str, Tuple[int, int, int]]

# 垂距短于**半个线宽**时画出来就是一个点。线宽 2 px → 1 px。
# 不取更大的值：2 px 的间隙**画得出来也看得见**，跳过它会让"图上没有垂距线、
# 而 HUD 上 e_bottom 是个非零数"，图和数打架（跨过阈值时探针集合还会从 15 条
# 掉到 11 条，操作员在最后 2 px 里看到两个图层凭空消失）。
_MIN_OFFSET_PX = 1.0
# 垂足圆半径与探针取点半径（圆环上）。探针取**圆环上**而不是圆心 —— 圆心正是
# 垂距线的端点，实心圆会把它整个盖住。
_FOOT_RADIUS_PX = 5.0
# 内法向箭头长度；探针取杆的中点（避开头部，见 F5）。
_ARROW_LEN_PX = 40.0
# 箭头沿参考边**切向**挪开的距离。**不是审美，是避让**：θ=0（轴对齐 / 伺服收敛
# 后）时，参考边、箱子边、垂距线、台面边**全部落在同一条内法向直线上** ——
# 箭头若也画在那条线上，必然被后画的某一层盖住，探针恒红（实测 e_bottom = 1…400
# px、横向错开 < 1.25 px 时恒红）。
#
# 距离怎么来的：探针取 7x7 邻域（半宽 **3** px），最粗的线是 3 px（半宽 **1.5** px），
# 3 + 1.5 = 4.5 px 是"箭头像素不落进那条线上各图层自检窗"的**下界**；取 10 px
# 是留一倍余量（也让箭头杆的像素整个离开那条线，不只是"窗里还能找到蓝色"）。
#
# ⚠️ **方向不固定，见画箭头那一段的 `delta` / `side`**：恒定 `+t̂` 只在"箱子中点
# 横向不漂"时够用；箱子中点横向漂到 ≈ ±10 px（旋转 2°、或两轴同偏）时，箭头杆
# 正好落回垂距线上，而那时垂距线很短（实测 3.7~9.5 px），**整条都在杆里** ——
# F8 沿线段挪位找不到任何有红色的位置，照旧报红。改成"背离箱子中点"之后，两者
# 切向分离量有下界 `_ARROW_SIDE_PX = 10 px`（推导见那一处），避让才真的成立。
_ARROW_SIDE_PX = 10.0

# 画中文要一个真有 CJK 字形的字体文件：PIL 的 load_default() 只有 ASCII 位图，
# 中文会画成空白/豆腐块，而图上那些标签正是操作员判读闸门时要看的东西。
# 不写死单个路径 —— 现场机器的字体安装五花八门，逐个试、都没有才退回默认。
CJK_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "/System/Library/Fonts/PingFang.ttc",
    "C:/Windows/Fonts/msyh.ttc",
)
font_cache: dict = {}


def cjk_font(size: int = 18):
    """挑一个能画中文的字体；都找不到就退回默认（并在日志里说一声）。

    ⚠️ `NotoSansCJK-Regular.ttc` 是个**字体集合**，`ImageFont.truetype` 默认取
    `index=0`（日文）。简体在第 2 个 index 上 —— 不传 `index` 时"简体中文"会
    画成日文字形（部分字不同）。
    """
    if size in font_cache:
        return font_cache[size]
    font = None
    for path in CJK_FONT_CANDIDATES:
        for index in (2, 0):            # 先试简体那个 index，再退回默认
            try:
                font = ImageFont.truetype(path, size, index=index)
                break
            except (OSError, ValueError):
                continue
        if font is not None:
            break
    if font is None:
        logger.warning("没找到任何 CJK 字体（试过 %s），图上的中文标签会画不出来；"
                       "线本身不受影响", list(CJK_FONT_CANDIDATES))
        font = ImageFont.load_default()
    font_cache[size] = font
    return font


def round_xy(uv) -> Tuple[int, int]:
    return int(round(float(uv[0]))), int(round(float(uv[1])))


def draw_segment(canvas: np.ndarray, p0, p1, color, thickness: int = 2) -> None:
    """一条**抗锯齿关闭**的线。

    自检依赖"线上就是那个颜色"。AA 会混掉**细线**的像素：实测 1 px 线在 180 个
    角度里有 122 个角度中心像素不是纯色；2~3 px 线则 180/180 都是纯色（中心离
    边缘够远）。本模块有 1 px 的图层（YOLO 框），所以一律关 AA。
    """
    cv2.line(canvas, round_xy(p0), round_xy(p1), color, thickness,
             lineType=cv2.LINE_8)


def draw_arrow(canvas: np.ndarray, p0, p1, color) -> None:
    cv2.arrowedLine(canvas, round_xy(p0), round_xy(p1), color, 2,
                    line_type=cv2.LINE_8, tipLength=0.25)


def _label(draw, uv, text: str, color) -> None:
    """往一个**已经打开的** PIL 画布上写一行中文。走 PIL —— `cv2.putText` 认不了中文。

    ⚠️ 这是**破坏性**的：它盖住的就是它画上去的那些像素。所以锚点要选在不参与
    自检的位置（各图层的**端点**，探针取的是**中点**），而且**所有标签最后一遍
    一次画完**（`_draw_labels`）—— 一次 PIL 会话，不是每个标签一趟。
    """
    x, y = int(round(uv[0])), int(round(uv[1]))
    draw.text((x, y), text, fill=(int(color[2]), int(color[1]), int(color[0])),
              font=cjk_font())


def _draw_labels(canvas: np.ndarray, labels: Iterable[Label]) -> None:
    """**一次** PIL 会话画完所有中文标签（含 HUD）。

    单开一趟 PIL 要整张画布 BGR→RGB→BGR 往返一次（1280x1200 实测 4.6 MB、
    17.6 ms）；7 个图层标签 + 3 行 HUD 分开画就是 ~180 ms，而伺服 tick 是
    100 ms —— 分开画会让那一 tick 直接超时。攒成一个列表只往返一次。
    """
    labels = list(labels)
    if Image is None or not labels:                          # pragma: no cover
        return
    bgr = Image.fromarray(canvas[:, :, ::-1])
    draw = ImageDraw.Draw(bgr)
    for uv, text, color in labels:
        _label(draw, uv, text, color)
    canvas[:, :, :] = np.asarray(bgr)[:, :, ::-1]


def _same_pixels(a, b) -> bool:
    """两条线段会不会画出**同一批像素**（正/反向都算）。

    判据是**取整后的整数端点相等** —— `draw_segment` 用 `round_xy` 把端点吸附
    到整数像素，所以端点取整后相同的两条线段，画出来逐像素相同：后画的整条盖住
    先画的，`cv2.line` 画不了两次。

    ⚠️ **不能判浮点相等。** 相差 0.4 px 的两条线几何上是两条、**像素上只有一条**
    （都吸附到同一个整数像素）。判浮点相等会把这种帧当成"两条线"，于是两层都画、
    都探，前面那层必然被盖住 —— 实测 |d| ≤ 0.5 px 的整个亚像素带都会红，而伺服
    收敛后箱子就停在这一带里。

    ⚠️ 也**不用 `np.allclose` 的默认 rtol**：它随坐标值变化（u=1240 处等效阈值
    0.0124 px，是 atol 的 12 倍），判据会变成位置相关的。

    ⚠️ 这里**没有容差常数**，这正是要点：判据是"渲染后是不是同一批像素"，是个
    **离散的、精确的**判断，不是"调到能过"的阈值。

    ⚠️ **必须判反向**：参考边 `x=W` 是 `(W,0,0)→(W,H,0)`，台面矩形边1 是
    `quad[1]→quad[2]` = `(W,0,0)→(W,H,0)`（同向）；但换个 `ref_edges`（比如
    `"x=0"`）就可能反向 —— 只判正向会漏。
    """
    a0, a1 = round_xy(a[0]), round_xy(a[1])
    b0, b1 = round_xy(b[0]), round_xy(b[1])
    return (a0 == b0 and a1 == b1) or (a0 == b1 and a1 == b0)


def _merge_collinear(segments) -> set:
    """按优先级合并**画出同一批像素**的线段，返回**该画的那些 key**。

    `segments` 是 `(key, (p0, p1))` 的序列，**按优先级从高到低**排好。一条线
    如果与前面（优先级更高）已经保留的某条线**画出同一批像素**，就**不画、也不探**
    —— 后画的整条会盖住先画的，先画的那条的探针必然失败（实测：伺服到位那一帧
    8 条探针全红）。

    **被跳过的线不产生探针**：探针的语义是"这条线画对了吗"，没画的线没有探针
    可失败。

    判据见 `_same_pixels`（取整端点相等）——**不是浮点相等**，因为 0.4 px 的差
    在渲染后就是同一批像素。
    """
    kept: set = set()
    drawn: List = []
    for key, seg in segments:
        if any(_same_pixels(seg, d) for d in drawn):
            continue
        kept.add(key)
        drawn.append(seg)
    return kept


def _project_pallet_quad(T_cam_pallet, size_mm, K, D=None):
    """托盘台面矩形的四个像素角，顺序 `[左下, 右下, 右上, 左上]`。

    与消息里的 `corners_uv` 同一个顺序（从 `origin` 起、沿 `e1` 绕一圈）——
    但**这里是伺服自己投的**，不读消息。参考边就是同一个 `T_cam_pallet` 投的，
    所以两者必然贴合。

    名字**不得改**：`infrastructure/.../pallet_detection_msgs/msg/PalletDetection.msg`
    的注释直接引用了 `_project_pallet_quad()` 这个名字。
    """
    from .algorithm import project_pallet_points

    W, H = float(size_mm[0]), float(size_mm[1])
    pts_mm = [(0.0, 0.0, 0.0), (W, 0.0, 0.0), (W, H, 0.0), (0.0, H, 0.0)]
    px = project_pallet_points(T_cam_pallet, pts_mm, K, D)
    if hasattr(px, "code"):            # Reject
        return None
    return np.asarray(px, np.float64)


def render_servo_overlay(color_bgr: np.ndarray, *, error,
                         T_cam_pallet, size_mm, K, D=None,
                         yolo_uv=None, hud_lines: Sequence[str] = (),
                         ref_edge_labels: Optional[Sequence[str]] = None,
                         ) -> Tuple[np.ndarray, List[Probe]]:
    """画一张对比图，同时返回**每个元素的中点**作为自检探针。

    图层（从下到上）：
      1. YOLO 输入框（灰，四条 1 px 边）—— 给了 `yolo_uv` 才画
      2. 托盘参考边（黄）
      3. 托盘台面矩形（青）—— 伺服自己从 `T_cam_pallet` + `size_mm` 投影
      4. 箱子底边/右边（绿）
      5. 垂距线（红）+ 垂足（紫，空心圆）
      6. 内法向箭头（蓝）—— **攒到实线与垂距线之后画**，见 `_ARROW_SIDE_PX`
      7. HUD 文字 + 各图层的标签（**攒起来最后一遍一次画完**，见 `_draw_labels`）

    ⚠️ **画出同一批像素的实线只画一条。** 默认 `ref_edges=["y=0","x=W"]` 时参考边
    就是台面矩形的下边和右边，**在几何上是同一对三维线段**；伺服到位时箱子底边/
    右边也落到同一批直线上。按优先级 `箱子 > 参考边 > 台面矩形 > YOLO 框` 合并，
    **被跳过的既不画也不探**（理由见模块 docstring 与 `_merge_collinear`）。

    ⚠️ **内法向箭头沿参考边切向挪开 `_ARROW_SIDE_PX`，方向背离箱子中点**，
    并改到最后画。垂距线在几何上**必然沿内法向**（`foot` 是 `m_box` 在参考边上的
    投影），所以 θ=0（伺服收敛后）时箭头、垂距线、参考边、箱子边全在同一条直线上
    —— 箭头画在那条线上必然被后画的某一层盖住（实测 e_bottom = 1…400 px、
    横向错开 < 1.25 px 时恒红）。方向判据与分离量下界见画箭头那一段。

    ⚠️ **方向翻转（`delta` 过零）会让箭头从 `+10` 跳到 `−10`，切向跳 20 px。**
    这是这个方案的**固有代价**，已知并接受：

      * 箭头的**切向位置不承载信息** —— 它只是"内法向朝哪边"的方向指示，跳变
        不会让人读出错误的东西（跳变前后**法向方向 `n` 完全不变**）。
      * **真正会碰撞的区间是 `dist ∈ [_MIN_OFFSET_PX, ~10] px`**（`dist` 是垂距，
        `dist < _MIN_OFFSET_PX` 时垂距线根本不画、也不探）；跳变发生在
        `delta ≈ 0` —— 两个区间**不是同一个**，所以跳变不会与"垂距线时有时无"
        叠加成"看着线在闪"。
      * `delta` 在 0 附近抖动时箭头会来回跳。**可接受，不要为消除它加死区或平滑**
        —— 那会重新引入"避让失效"的窗口（死区内 `side` 不翻，箭头又会落回垂距线上）。

    ⚠️ 挪开的那一步会顺手把箭头从**垂距线的探针**上挪走：垂距线沿内法向、探针取
    它的中点，而箭头杆正好横穿那一点。垂距线的中点探针因此在「箭头杆盖住中点」时
    取不到红色 —— 由 F8 的挪位兜住（沿垂距线挪到还看得见的一段）。**这正是 F8
    存在的第二个理由**（第一个是"共线但不同长"）。

    ⚠️ **垂距为零时不画垂距线、不画垂足，也不探这两者**：`e ≈ 0` 时垂距线的两个
    端点重合，画出来就是一个点，还必然被垂足圆盖住。语义上也对 —— 垂距为零时
    本来就没有垂距可看。

    返回 `(canvas_bgr, probes)`；`probes` 交给 `numeric_self_check` 用。探针取
    **线段中点**是因为自检要在"这条线上本该有颜色"的地方采样 —— 中点一定在线上，
    端点可能在箭头或文字下面被判读不到。

    `color_bgr` **就是 BGR**，本函数不做通道翻转（`cv_bridge` 的 `bgr8` 正是
    这个序）；`canvas` 是它的拷贝，原图不被改动。
    """
    canvas = np.asarray(color_bgr, np.uint8).copy()
    probes: List[Probe] = []
    pending_labels: List[Label] = []
    # F8 用：实线探针的 (探针下标, 线段, 本色)。**只在实线画完、文字之前**用来
    # 把被别的图层部分盖住的探针挪到还看得见的一段上（见 `_snap_probes_to_drawn`）。
    snap_specs: List = []

    quad = _project_pallet_quad(T_cam_pallet, size_mm, K, D)

    # —— 实线图层的重合合并 ——
    # 优先级（高 → 低）：箱子底边/右边 > 参考边 底/右 > 台面矩形 边0..3 > YOLO 框边。
    # 箱子边是**测量结果**、参考边是**目标**：两者重合 = 到位，这时显示绿色
    # （"箱子已经贴到托盘边上了"）比显示黄色信息量更大。
    # YOLO 框**优先级最低**：它是**输入**、箱子边是**精修结果**（`box_frame` 的
    # AABB 兜底路径产出的四角**就是 YOLO 框本身**，5 帧全失败时 box_bottom ==
    # YOLO 下边、box_right == YOLO 右边），两者重合时显示精修结果信息量更大。
    # 参考边的标签用**调用方给的原文**（用户写的 `y=0` / `y=800`），不用
    # 写死的"底边"/"右边" —— 那两句话只对默认组合成立，换个 `ref_edges`
    # 就是错的，而操作员会照着图去核对（指错边比不标更糟）。
    # 不给就退回位置名（老调用方不受影响）。
    if ref_edge_labels is not None and len(ref_edge_labels) == 2:
        bottom_tag, right_tag = str(ref_edge_labels[0]), str(ref_edge_labels[1])
    else:
        bottom_tag, right_tag = "底边", "右边"
    ref_specs = (("ref_bottom", error.ref_bottom_px, bottom_tag),
                 ("ref_right", error.ref_right_px, right_tag))
    box_specs = (("box_bottom", error.box_bottom_px, "底边"),
                 ("box_right", error.box_right_px, "右边"))
    quad_specs = ([(f"quad{k}", (quad[k], quad[(k + 1) % 4]))
                   for k in range(4)] if quad is not None else [])
    # YOLO 框拆成四条 1 px 边，边顺序与探针一一对应（左/右/下/上）。**不再用
    # `cv2.rectangle`**：那样它绕过合并列表，与箱子边/台面边共线时会被整条盖住，
    # 三条 YOLO 探针恒红 —— 而 Task 5 传的正是 `self._last_box_uv`，那是一整条
    # 工作模式必红，不是巧合。视觉不变：1 px 灰框本来就是四条线拼的。
    yolo_specs = []
    if yolo_uv is not None:
        yu0, yu1 = sorted((float(yolo_uv[0]), float(yolo_uv[2])))
        yv0, yv1 = sorted((float(yolo_uv[1]), float(yolo_uv[3])))
        yolo_specs = [("yolo_left", ((yu0, yv0), (yu0, yv1)), "左边"),
                      ("yolo_right", ((yu1, yv0), (yu1, yv1)), "右边"),
                      ("yolo_bottom", ((yu0, yv1), (yu1, yv1)), "下边"),
                      ("yolo_top", ((yu0, yv0), (yu1, yv0)), "上边")]
    kept = _merge_collinear(
        [(k, seg) for k, seg, _ in box_specs]
        + [(k, seg) for k, seg, _ in ref_specs]
        + quad_specs
        + [(k, seg) for k, seg, _ in yolo_specs])

    # 1) YOLO 输入框（四条 1 px 边，各自判是否被跳过；被跳过的边不产生探针）
    #    探针仍然是原来的三条（左/右/下）：**上边不设探针**。上边那一带是箱子
    #    底边的"XX 垂距"标签落笔的地方（标签锚在箱子边中点上，最后一遍画），
    #    在那里取灰色会被文字盖住 —— 实测随机连续族里 32/4000 帧因此变红。
    #    画四条边是为了逐边参与合并，不是为了逐边取探针。
    yolo_probed = {"yolo_left", "yolo_right", "yolo_bottom"}
    for key, seg, tag in yolo_specs:
        if key not in kept:
            continue
        draw_segment(canvas, seg[0], seg[1], COLOR_YOLO_BOX, 1)
        if key in yolo_probed:
            mid = ((seg[0][0] + seg[1][0]) / 2.0, (seg[0][1] + seg[1][1]) / 2.0)
            probes.append((f"YOLO 框 {tag}", mid, COLOR_YOLO_BOX))
            snap_specs.append((len(probes) - 1, seg, COLOR_YOLO_BOX))
    if yolo_specs:
        pending_labels.append(((yolo_specs[0][1][0][0], yolo_specs[0][1][0][1]),
                               "YOLO 输入框", COLOR_YOLO_BOX))

    # 2) 托盘参考边
    for key, seg, tag in ref_specs:
        if key not in kept:
            continue
        ref = np.asarray(seg, np.float64)
        draw_segment(canvas, ref[0], ref[1], COLOR_REF_EDGE, 3)
        mid = (ref[0] + ref[1]) / 2.0
        probes.append((f"托盘参考边 {tag}", tuple(mid), COLOR_REF_EDGE))
        snap_specs.append((len(probes) - 1, seg, COLOR_REF_EDGE))
        pending_labels.append((tuple(ref[0]), f"托盘{tag}", COLOR_REF_EDGE))

    # 3) 托盘台面矩形
    if quad is not None:
        drawn_any = False
        for k in range(4):
            if f"quad{k}" not in kept:
                continue
            a, b = quad[k], quad[(k + 1) % 4]
            draw_segment(canvas, a, b, COLOR_PALLET_QUAD, 2)
            mid = (a + b) / 2.0
            probes.append((f"托盘台面 边{k}", tuple(mid), COLOR_PALLET_QUAD))
            snap_specs.append((len(probes) - 1, (a, b), COLOR_PALLET_QUAD))
            drawn_any = True
        if drawn_any:
            pending_labels.append((tuple(quad[0]), "托盘台面", COLOR_PALLET_QUAD))
    else:
        logger.warning("托盘台面投影失败（Reject）—— 图上不画这一层")

    # 4) 箱子底边/右边
    for key, seg, tag in box_specs:
        if key not in kept:
            continue
        box = np.asarray(seg, np.float64)
        draw_segment(canvas, box[0], box[1], COLOR_BOX_EDGE, 3)
        mid = (box[0] + box[1]) / 2.0
        probes.append((f"箱子 {tag}", tuple(mid), COLOR_BOX_EDGE))
        snap_specs.append((len(probes) - 1, seg, COLOR_BOX_EDGE))

    # 5) 垂距线 + 垂足（先画），箭头**攒到最后画**（见下面 6)）
    arrow_specs = []
    for ref, box, n, foot, tag in (
            (error.ref_bottom_px, error.box_bottom_px, error.bottom_inward_px,
             error.bottom_foot_px, "底边"),
            (error.ref_right_px, error.box_right_px, error.right_inward_px,
             error.right_foot_px, "右边")):
        ref = np.asarray(ref, np.float64)
        n = np.asarray(n, np.float64)
        mid_ref = (ref[0] + ref[1]) / 2.0
        m_box = (np.asarray(box[0], np.float64)
                 + np.asarray(box[1], np.float64)) / 2.0
        tangent = ref[1] - ref[0]
        tangent = tangent / float(np.linalg.norm(tangent))
        # 箭头沿参考边**切向**挪开 `_ARROW_SIDE_PX`，方向**背离箱子中点**：
        # 恒定 `+t̂` 只在"箱子中点横向不漂"时够用 —— 箱子中点横向漂到 ≈ ±10 px
        # （旋转 2°、或两轴同偏）时，箭头杆正好落回垂距线上，而那时垂距线很短
        # （实测 3.7~9.5 px），**整条都在杆里** → 探针恒红（F8 也救不了）。
        #
        # `delta` 的定义（坐标系写死）：**参考边自身的切向**，从参考边中点指向
        # 箱子这条边的中点：
        #     delta = (m_box − mid_ref) · t̂        t̂ = (ref[1] − ref[0]) / |…|
        # ⚠️ `t̂` 的**正方向随参考边写法变**（`y=0` 的 `t̂` 指向 +u、`x=W` 的 `t̂`
        # 指向 +v，两者不是同一个方向），所以**不要**给 `delta` 赋绝对含义 ——
        # 这里只用它的**符号**：`side = +1`（delta ≤ 0）/ `−1`（delta > 0），
        # 也就是"往箱子中点的**反方向**挪"。两个参考边共用这一条判据。
        #
        # 分离量下界：垂距线（连同它的探针、垂足探针）都在切向坐标 `delta` 附近，
        # 箭头杆在 `_ARROW_SIDE_PX · side`，两者的切向距离
        #     |_ARROW_SIDE_PX·side − delta| = _ARROW_SIDE_PX + |delta| ≥ 10 px
        # 而"箭头像素不落进别的图层自检窗"只需 ≥ 3（探针 7x7 窗半宽）+ 1（箭头
        # 杆半宽）= **4 px**（见 `_ARROW_SIDE_PX`）—— 10 px 有 2.5 倍余量，恒不相交。
        delta = float((m_box - mid_ref) @ tangent)
        side = 1.0 if delta <= 0.0 else -1.0
        arrow_start = mid_ref + tangent * (_ARROW_SIDE_PX * side)
        arrow_specs.append((arrow_start, arrow_start + n * _ARROW_LEN_PX, tag))
        # 探针取**杆的中点**：在杆上、避开头部（头部占尖端 0.25×40 = 10 px），
        # 而且有物理意义（内法向方向的指示）。
        probes.append((f"内法向 {tag}",
                       tuple(arrow_start + n * (_ARROW_LEN_PX / 2.0)),
                       COLOR_INWARD))

        foot = np.asarray(foot, np.float64)
        d = foot - m_box
        dist = float(np.linalg.norm(d))
        if dist < _MIN_OFFSET_PX:
            # 垂距为零（或短于半个线宽）：**没有垂距可看** —— 不画垂距线、不画
            # 垂足圆，也就不探这两者（画出来必然被圆盖住，探哪都失败）。
            continue

        draw_segment(canvas, m_box, foot, COLOR_OFFSET, 2)
        cv2.circle(canvas, round_xy(foot), int(_FOOT_RADIUS_PX), COLOR_FOOT, 2,
                   lineType=cv2.LINE_8)
        # 垂距线的中点做探针（两个端点分别在中点与垂足上，中点更安全）
        probes.append((f"垂距线 {tag}", tuple((m_box + foot) / 2.0), COLOR_OFFSET))
        snap_specs.append((len(probes) - 1, (m_box, foot), COLOR_OFFSET))
        # 垂足探针取**圆环上、垂直于垂距线方向**的点：垂距线沿 `d` 方向，垂直于
        # 它的方向一定离开那条线（线宽 2~3 px，垂直偏移 5 px 已经出线）；沿 `d`
        # 方向取点会落在线上。空心圆不盖圆心，但圆环上才一定没有线。
        perp = np.array([-d[1], d[0]], np.float64) / dist
        probes.append((f"垂足 {tag}", tuple(foot + perp * _FOOT_RADIUS_PX),
                       COLOR_FOOT))
        pending_labels.append((tuple(m_box), f"{tag} 垂距", COLOR_OFFSET))

    # 6) 内法向箭头 —— **实线与垂距线之后**画，于是没有任何实线盖得住它。
    #    挪开 `_ARROW_SIDE_PX` 保证它也不盖住别的图层的探针（那些探针都在那条
    #    内法向直线上，横向偏移 0，与箭头相距 10 px ≫ 箭头半宽 + 探针窗半径）。
    for p0, p1, _tag in arrow_specs:
        draw_arrow(canvas, p0, p1, COLOR_INWARD)

    # 7) 文字：各图层标签 + HUD。**一次 PIL 会话**，见 `_draw_labels`。
    #    文字**必须排在 F8 挪位之前** —— 理由见下面 7.5。
    y = 24.0
    for line in hud_lines:
        pending_labels.append(((12.0, y), line, (255, 255, 255)))
        y += 22.0
    _draw_labels(canvas, pending_labels)

    # 7.5) **F8**：实线探针被别的图层**部分**盖住时，沿它自己那条线挪到还看得见
    #      的一段上。**必须在所有实线画完之后、而且要在文字之后** —— 挪位是拿
    #      "探针的 7x7 邻域里有没有本色"当"这个位置在图上看得见"的判据，而文字
    #      也会盖住像素。挪位跑在文字之前时，它找到的位置**随后被文字盖住**，
    #      自检照样红（实测 `theta = -2°`、垂距 1~4 px：参考边右探针被
    #      `右边 垂距` 标签盖掉）。放到最后，看到的才是**最终画布**。
    #      ⚠️ 挪位之后探针仍可能落在**同色的**文字上（`右边 垂距` 标签就是
    #      `COLOR_OFFSET` 红，与垂距线同色）—— 语义与可接受性见 `_snap_probes_to_drawn`。
    _snap_probes_to_drawn(canvas, probes, snap_specs)

    return canvas, probes


def _probe_has_color(canvas: np.ndarray, uv, want, tolerance: int = 8) -> bool:
    """`uv` 的 7x7 邻域里有没有目标颜色（越界算没有，由调用方报出来）。"""
    h, w = canvas.shape[:2]
    ui, vi = int(round(float(uv[0]))), int(round(float(uv[1])))
    if not (0 <= ui < w and 0 <= vi < h):
        return False
    patch = canvas[max(0, vi - 3):vi + 4, max(0, ui - 3):ui + 4]
    want_arr = np.asarray(want, np.int16)
    return bool(np.any(np.all(np.abs(patch.astype(np.int16) - want_arr)
                              <= tolerance, axis=-1)))


def _snap_probes_to_drawn(canvas: np.ndarray, probes: List[Probe],
                          specs, tolerance: int = 8) -> None:
    """**F8**：实线探针被别的图层**部分**盖住时，沿它自己那条线挪到还看得见的一段。

    `specs` 是 `(探针下标, 线段, 本色)` 的序列。对每个探针：

      1. 它的 7x7 邻域**已有本色** → **不动**（绝大多数走这条，零开销）；
      2. 否则沿线段参数 `t` 从**中点向两端**各走一步、每步 **1 像素**
         （`step = 1 / 线段长度`，由"每像素一个采样点"推出，不是试出来的常数），
         取第一个 7x7 邻域有本色的采样点；
      3. **整条走完都没有 → 保持几何中点**（于是照旧报红）。

    ⚠️ 为什么要有这一条：两条线**共线但长度不同**（箱子边与托盘边对齐、箱子比
    托盘边短）时，短的那条后画、只盖住长的那条**中间一段**，而长的那条的探针
    恰好取在中点上 —— 整条线段既不是"同一批像素"（`_same_pixels` 判不了），
    又不能整条跳过（跳过会让箱子外的那两段整个消失）。**只挪探针是唯一既保住
    观感、又不让自检误报的做法。** 实测随机连续族 4000 帧里 8 帧（0.20%）是
    这个形状，且集中在 `e ≈ 0`（伺服收敛）那一带 —— 正是最该看的状态。

    ⚠️ **代价（有意取舍）**：F8 之后探针验证的是"**这条线段上某处**是本色"，
    不再是"**这个点**是本色"。于是"线画对了但只画了一半（长度错）"这一类
    **不再会被抓到**；"没画 / 画错色 / 画错位置"仍然抓得到（整条都没有本色）。

    ⚠️ 不用"枚举线段上的像素点再比颜色"：`cv2.line` 的栅格化（Bresenham 的取整
    规则）与按参数取点**不保证逐像素一致**，会出现"画对了但采样点没踩到"。
    这里复用既有的 7x7 邻域判据，从几何中点向两端走。
    """
    for idx, seg, want in specs:
        name, uv, color = probes[idx]
        if _probe_has_color(canvas, uv, want, tolerance):
            continue
        p0 = np.asarray(seg[0], np.float64)
        p1 = np.asarray(seg[1], np.float64)
        d = p1 - p0
        length = float(np.linalg.norm(d))
        if length <= 0.0:
            continue
        step = 1.0 / length                       # 每步 = 1 像素
        found = None
        for k in range(1, int(np.ceil(0.5 / step)) + 1):
            for t in (0.5 - k * step, 0.5 + k * step):
                if not (0.0 <= t <= 1.0):
                    continue
                cand = p0 + d * t
                if _probe_has_color(canvas, cand, want, tolerance):
                    found = cand
                    break
            if found is not None:
                break
        if found is not None:
            probes[idx] = (name, (float(found[0]), float(found[1])), color)


def numeric_self_check(canvas: np.ndarray, probes: Iterable[Probe],
                       tolerance: int = 8) -> List[str]:
    """在**本该是目标颜色**的位置采样；返回问题列表，空表示通过。

    取 7x7 邻域而不是一个像素：线宽 2~3 px，中点在浮点取整后可能偏一像素，
    单点采样会把"画对了"误判成"画错了"。

    ⚠️ 实线探针在 `render_servo_overlay` 里已经过 **F8 的挪位**（见
    `_snap_probes_to_drawn`）：两条线共线但长度不同时，长的那条的几何中点可能
    正好落在短的那条盖住的像素上。挪位之后本函数验证的是"**这条线段上某处**
    是本色"，不再是"这个点"——"没画 / 画错色 / 画错位置"仍然抓得到。

    ⚠️ **探针落在画布外 -> 直接跳过，不算问题**（2026-09-30 第二次修）。
    第一次只放宽了"线段有一截进得来"的情况，还是不够 —— 又被拦住两次：

      * `YOLO 框 下边 (685, 803)` / `箱子 底边 (676, 844)`（图高 800）：箱子出画
      * `垂足 底边 (645, 827)`：它是个**单点**探针（不是线段），没有"另一截"
        可挪，所以第一次的放宽救不了它

    为什么该彻底去掉：**"在画布外"对"画图代码有没有 bug"没有任何分辨力。**
    `cv2.line` 自己会裁 —— 完全在外面的线段本来就一个像素都不该画上、也画不出来；
    有一截进来的则一定被画上。所以"探针在外"不管哪种情况都不是画错的证据，
    它只说明**这一帧的几何确实跑出了画面**，而那是正常工况（托盘/箱子在视野边缘）。

    代价（明说）：**"探针在外"这件事不再有任何告警**。它的信息还在 ——
    叠加图上那一层就是没画出来，那本身是可见的。
    """
    problems: List[str] = []
    h, w = canvas.shape[:2]
    for name, uv, want in probes:
        ui, vi = int(round(float(uv[0]))), int(round(float(uv[1])))
        if not (0 <= ui < w and 0 <= vi < h):
            continue                     # 在画布外：画不出来，也不该画（见 docstring）
        patch = canvas[max(0, vi - 3):vi + 4, max(0, ui - 3):ui + 4]
        want_arr = np.asarray(want, np.int16)
        if not np.any(np.all(np.abs(patch.astype(np.int16) - want_arr)
                             <= tolerance, axis=-1)):
            problems.append(f"{name}: ({ui}, {vi}) 附近没有目标颜色 "
                            f"{tuple(int(c) for c in want)}")
    return problems
