#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**核心**：YOLO-OBB 的旋转框 → 台面坐标系 `frame`，**法向锁定 + 四边直线**，4 自由度。

这个文件是整个方案的结论所在。两处改动（都有实测支撑）把可见角点距中位
从 24.19px 压到 **14.62px**、IoU 从 0.836 提到 **0.881**。

    6-DoF 四角 PnP（旧做法）              24.19 px / IoU 0.836 / 贴边线垂距 16.76 px
    4-DoF 法向锁定（四角）                18.72 px / IoU 0.872 / 12.42 px
    **4-DoF 法向锁定 + 四边直线（本文件）** **14.62 px** / **0.881** / **9.45 px**

---

## 改动一：锁法向，自由度从 6 降到 4

把"OBB 四角算出的位姿"与"真值四角算出的位姿"的**旋转差分解**开（601 帧）：

| 分量 | 中位 | p90 |
|---|---|---|
| **法向差（倾斜）** | **11.20°** | 30.31° |
| 面内转角差 | **1.93°** | 4.84° |

用户说"OBB 的角度较准"—— 准的是**面内转角**，**倾斜是错的**。倾斜为什么必然错：
OBB 是**外接矩形**，它的 **高/宽 = 1.034**，而 1000×1200 在该位姿下投出来是 **1.129**
（相对差 −8.5%）。**倾斜角只由宽高比决定**，框偏"方"→ 倾角必然偏小、位置必然偏出去
（实测原点差中位 **186mm**、p90 411mm）。

**不要让四角去解倾斜。** 托盘放在地面上 → 台面与地面平行 → 台面法向就是流水线里
already 有的**地面法向**（`valpack.load(stem)['nrm']`）。锁上它，只剩
**平面深度 1 + 平面内平移 2 + 面内转角 1 = 4 个自由度**。

## 改动二：用**框的边**，不用框的角

601 帧逐边/逐角实测：

| | 中位 | p90 |
|---|---|---|
| OBB 框的**四角** vs 真值四角（逐点距） | **28.59 px** | 60.5 px |
| OBB 框的**四边所在直线** vs 真值边直线（垂距） | **2.6~5.1 px** | 5.9~13.1 px |

**差 6~11 倍。** 原因：外接矩形比梯形宽，多出来的那一截全落在**边的方向上**
（把角推出去），而直线的位置几乎不动。逐边：

| 框边 | 直线垂距中位 | 直线夹角中位 |
|---|---|---|
| 近边（下） | 2.62 px | 2.23° |
| 远边（上） | 5.06 px | 2.49° |
| 右边 | 2.89 px | 8.67° |
| 左边 | 2.92 px | 3.41° |

顶/底两条边连**方向**都只有 ~2.3°（用户说的"角度由顶边与底边确定"）；左右边的
**位置**也准（2.9px）但**方向**差 3.4~8.7° —— 真值里左右边本来就不平行
（真值**左右边夹角中位 12.41°**、p90 26.15°），矩形只能取一个折中方向。这是
**梯形本身的几何**，不是 OBB 的误差（OBB 自身顶底夹角恒 0.00°）。

四条边**等权**最好：给远/近边加权（×3）或给左右边降权（×0.5、×0.25）**都更差**
（p90 从 26.4 涨到 29.3~30.1）—— 左右边的偏是**系统性**的，等权时四条边误差互相
抵消，单独放大任何一组反而把解拽偏。

---

## ⚠️ 三个会静默出错的地方（都踩过）

1. **法向必须翻到 z>0**。`nrm` 是台面朝上的外法向、z 恒为负；而 `d·n` 这个参数化
   要求 n 指向相机前方，否则 d>0 会把平面放到**相机背后**。背后的点与前面的点共射线、
   **投影数值上完全一致** —— 残差照样小、自检全过 —— 但四角是**镜像**的，梯形永远
   拟不出来（实测 `2026-10-10-14-22-38` 那批帧崩到 1245px）。
2. **面内转角由 `long_side_parallel` 定死，不试第二个候选**。OBB 的宽高比
   1.034 ≈ 1，"哪条边是长边"在框上**根本不可分** —— 靠残差去试两个候选
   （θ0 = 0 与 θ0 = π/2）等于把这件事交给数据投票，同一场景不同帧可能投出不同结果。
   这个信息**用户给得出、而且给得对**：`long_side_parallel` 说明与画面近平行的
   那条托盘边是不是长边，直接定死 θ0。
   ⚠️ **给错就明确偏 90°**（而不是被"两个都试"兜住）—— 错误应该是可见的。
   ⚠️ **别用改 `pallet_size_mm` 顺序来达到这个目的**：`[1000, 1200]` 与
   `[1200, 1000]` 完全等价（内部取 min/max），改了没用，还会和 `ref_edges` 对不上。
3. **不要用 `_orient`**。它会按"E2_new 落在哪条物理边上"交换 W/H，把矩形翻个面
   （实测 10.84px → 1245px）。W/H 是任务书的硬输入、不是可解的量。本文件自己写
   `_to_spec`：只摆 E1/E2 的**符号**与 origin，W/H 不动。
"""
from __future__ import annotations

import numpy as np

from .geom import CameraIntrinsics, _corners_mm, _project_px
from scipy.optimize import least_squares

TILT_DEG = 0.0             # 法向允许绕两个面内轴各偏多少度；0 = 硬锁
TILT_RHO = 0.02            # `tilt_deg>0` 时，偏离量的弱先验权重（越小越硬）
DEPTH_RANGE_MM = (300.0, 4000.0)
# 深度种子（解算起点）。实测（601 帧）4 个种子与这 1 个种子**逐位相同**的精度
# （角点中位都是 14.62px），而 4 个种子要 29.6ms、1 个只要 11.1ms。代价面在深度
# 方向是**单峰**的（深度错 500mm 只让投影四角整体缩放，soft_l1 下代价单调上升），
# 不需要多起点兜底。
DEPTH_SEED_MM = 1200.0
# 面内转角只在两个候选各自 ± 这么多度里找（见文件头第 2 条）
#
# ⚠️ **这个窗口是"回旋余量"，不是"角度分辨率"** —— 种子的职责才是给角度。
# 2026-10-15 现场把它从 25° 放到 40°：窗口太窄时，只要种子偏了（比如 OBB 角度
# 本身有几十度的系统偏差），解算会被**夹在窗口边缘**、输出一个"稳定但错"的角度，
# 而日志上看不出任何异常。放宽容错，让**图像残差**去决定最终角度。
INPLANE_LIMIT_DEG = 40.0
PALLET_SIZE_MM = [1000.0, 1200.0]

# ⚠️ **上面这个常量是"没给尺寸时"的兜底，不是生产路径的尺寸来源。**
# 生产路径一路显式传板子上的 `pallet_size_mm`：
#
#     ROS 节点 `~target_mm`
#       -> detect.detect_pallet_frame(target_mm=…)
#       -> refine_depth.refine_pallet_frame(target_mm=…)
#       -> user_rule.frame_user_rule(size_mm=…)      ★ 用户规则的尺寸在这
#
# 兜底只在**离线脚本 / 单测**没传时生效。换了托盘尺寸却忘了改它，
# 解出来的矩形会按旧尺寸定，而残差照样小、`valid` 照样 true（**静默**）。
# ⚠️ `user_rule.frame_user_rule` 的默认值**故意**是 `None`（走这里的兜底）而
# 不是直接引用本常量 —— 见那个函数的 docstring。
#
# ⚠️ **顺序本身不重要**（`frame_from_obb` / `user_rule` 都取 `min`/`max`），
# 它定的是 **`E1`/`E2` 的长度**：`Wb = min(size)` 恒沿 `E1`、`Hr = max(size)`
# 恒沿 `E2`，也就是 `frame['W']`/`['H']`。下游伺服按 `(W,0)` / `(W,H)` 投影，
# 所以 **`W`/`H` 必须与 `ref_edges` 的 `x=` 那条边对上**（参考边本身可以落在
# 台面外；2026-10-26 起不再按尺寸做范围检查）。

# 边量测的**可见长度加权**：OBB 的某条边大部分跑到画面外时，它在图上只剩很短
# 一截，那条直线的方向几乎全靠噪声定 —— 不该和完整可见的边等权。
# 权重取该边落在画面内的长度占比（下限 `EDGE_W_MIN`）。
# 实测（218 个退化帧，可见角误差中位）：等权 7.05px -> 按可见长度加权 **6.06px**。
# 见 `_edge_weights`。
EDGE_WEIGHT_ON = False  # 出画的边直接**不参与位置约束**，不需要降权
EDGE_W_MIN = 0.05
EDGE_W_FLOOR = 0.5

# 边的可见占比低于这个值时，这条边**完全不参与位置约束** —— 它的两个端点里
# 至少有一个是 OBB 从画面外外推出来的，图像里没有对应像素，位置无从谈起。
# 实测 `2026-10-10-11-44-12_00240`：右边可见占比 0.03，它的位置约束把"右"角拽了 71mm。
EDGE_MIN_VIS = 0.5
# 位置约束"够用"的最少边数（另有"有一对对边"也算够）。见 `frame_from_obb`。
RANK_MIN_EDGES = 3

# ⚠️ **`EDGE_MIN_VIS` 单独用是不够的**，见 `edge_usable` 的长注释。这两个数是
# 加在它上面的第二、第三道闸门（内缩画面的安全可见占比 + 邻边共同出画）。
SAFE_EPS_MM = 8.0          # 判定"自己几乎不在画面里"时，画面向内缩的像素
SAFE_MIN_FRAC = 0.15
SAFE_EPS2_MM = 12.0        # 判定"自己贴边"时，画面向内缩的像素
SAFE_MIN_FRAC2 = 0.30

# 锁 d 的前提：框内深度**主峰占比**不低于这个值。与 `_plane_dist_mm` 的第二个
# 返回值同口径。
#
# ⚠️ **2026-10-19 改成 0**：真机 881 帧（`test_data/pallet_obb/images`，11 个 bag）
# 实测，"该锁没锁"的帧只占 2.3%，却贡献了 **89% 的大跳**（Δpos 中位 567mm、
# max 990mm）。机制是 `peak < 本值` 时 D 从"锁死"变成**自由变量**，而深度方向
# 在这套几何下近乎不可观测 —— 解能跑到 800mm 外。
#
#      D_LOCK_INLIER | Δpos 中位 | p90   | max    | >100mm | 角点距
#      0.30（旧）    | 11.29     | 34.25 | 990.9  | 34     | 40.35px
#      0.00（现行）  | 11.23     | 31.57 | 379.7  | 20     | 40.35px
#
# **宁可锁一个稍偏的 D（几十毫米），也不要让 D 自由（800mm）。** 精度不变。
# 详见 `maduo/debug/jitter/WORKLOG_jitter.md` §7。
#
# **唯一真相源**：`refine3.D_DEPTH_INLIER` / `refine4.D_INLIER_MIN` 都是本值的别名
# （经 `edge_trust.D_INLIER`），三处不会再各写一份。
D_LOCK_INLIER = 0.0

# 选峰的**最低占比**：占比低于它的 bin 不参与候选（仍保留"最大 bin"作兜底）。
#
# ⚠️ **2026-10-19 新加**：托盘大面积出画时，框内沿法向会出现**两个近等高的峰**
# （台面 ~870-910、地面/别的层 ~650-690），原实现取 `argmax`，近峰只要以
# 0.006~0.034 的占比优势胜出就会把 D 锁到 657（低 26%），位姿跳 300mm。
# 改成"在占比 >= 本值的峰里选**最接近 `D_REF_MM`** 的那个"之后：
#
#      PEAK_MIN | Δpos 中位 | p90   | max    | >100mm | 角点距
#      不启用   | 11.23     | 31.57 | 379.7  | 20     | 36.87px
#      0.30     | 11.23     | 31.23 | 335.9  | 15     | 36.82px
#      0.20     | 11.20     | 30.47 | 335.9  | 13     | 36.82px
#
# 取 0.20：bag06 那两对的正确峰占比只有 0.291/0.297，0.30 够不着、0.20 够得着
# （实测那两对从 299mm 降到 58.5mm、`>100mm` 2→0，角点距一字不变）。
PEAK_MIN = 0.20

# 选峰时的距离先验（毫米，相机原点到台面）。与板上 `pallet_d_mm` 同义，但
# **这里是纯函数库、读不到 ROS 参数**，所以只作为"多峰时挑哪个"的软先验：
# 只在多个峰都过 `PEAK_MIN` 时用于二选一，不参与任何闸门。
#
# ⚠️ 实测各 bag 已锁帧的 D 中位只在 888.0~900.7，用 bag 中位与用 890 **逐帧结果
# 完全相同**（881 帧 0 帧不同），所以不必做成逐场景可配。
D_REF_MM = 890.0

# 深度层的**分箱宽度**与**峰半宽**（毫米），见 `_plane_dist_mm`。
D_BIN_MM = 40.0
D_PEAK_HALF_MM = 60.0


def _vis_frac(p, q, w_img, h_img, n=60):
    """线段 `p->q` 落在画面内的长度占比。"""
    d = np.asarray(q, float) - np.asarray(p, float)
    if float(np.linalg.norm(d)) < 1e-9:
        return 0.0
    ts = np.linspace(0.0, 1.0, n)
    P = np.asarray(p, float)[None, :] + ts[:, None] * d[None, :]
    ok = ((P[:, 0] >= 0) & (P[:, 0] <= w_img - 1) &
          (P[:, 1] >= 0) & (P[:, 1] <= h_img - 1))
    return float(ok.mean())


def _safe_frac(p, q, eps, w_img=640.0, h_img=480.0, n=80):
    """线段 `p->q` 落在**向内缩 `eps` 像素**之后的画面内的长度占比。

    与 `_vis_frac` 的区别：贴着画面边界的那一截**不算**。托盘被画面裁掉时，
    OBB 从画面外**外推**出来的那条边照样能有一大截落在画面内（`_vis_frac` 给
    0.67），但它在"离边界有安全距离"的区域里**长度为 0** —— 因为那一段图像里
    根本没有对应的像素。
    """
    d = np.asarray(q, float) - np.asarray(p, float)
    if float(np.linalg.norm(d)) < 1e-9:
        return 0.0
    ts = np.linspace(0.0, 1.0, n)
    P = np.asarray(p, float)[None, :] + ts[:, None] * d[None, :]
    ok = ((P[:, 0] >= eps) & (P[:, 0] <= w_img - 1 - eps) &
          (P[:, 1] >= eps) & (P[:, 1] <= h_img - 1 - eps))
    return float(ok.mean())


def _edge_trust(quad, i, w_img=640.0, h_img=480.0):
    """单边判据：OBB 的第 `i` 条边**本身**像不像一条真边。见 `usable_edges`。"""
    if _vis_frac(quad[i], quad[(i + 1) % 4], w_img, h_img) < EDGE_MIN_VIS:
        return False
    a, b = quad[i], quad[(i + 1) % 4]
    if _safe_frac(a, b, SAFE_EPS_MM, w_img, h_img) < SAFE_MIN_FRAC:
        return False
    if _safe_frac(a, b, SAFE_EPS2_MM, w_img, h_img) < SAFE_MIN_FRAC2:
        nb_out = (_vis_frac(quad[(i - 1) % 4], quad[i], w_img, h_img) < EDGE_MIN_VIS
                  or _vis_frac(quad[(i + 1) % 4], quad[(i + 2) % 4],
                               w_img, h_img) < EDGE_MIN_VIS)
        if nb_out:
            return False
    return True


def _rank_ok(keep):
    """剩余的可信边能不能定住位姿。

    ⚠️ **必要条件是同时覆盖两组对边**：边序 `0,1,2,3` = 下/右/上/左，
    `{0,2}` 与 `{1,3}` 各是一条"横向"与一条"纵向"的约束方向。只剩一组
    （= 只剩 1 条边，或只剩两条**对边**）时，面内平移有**一维完全不受约束**，
    最小二乘会沿那个方向飘走 —— 现场 bag `2026-10-14-19-57-09` 的 `f0224`：
    只剩 `{2,3}` 时解还好好的，`usable_edges` 把上边也摘掉后只剩 `{3}`，
    投影四角直接跑到 `(-108,1209)`，重投影误差 **69.7 -> 620.8px**。
    """
    return bool(set(keep) & {0, 2}) and bool(set(keep) & {1, 3})


def usable_edges(quad, w_img=640.0, h_img=480.0):
    """OBB 的**四条边里哪些能当位置量测**（`frame_from_obb` 的 `use_pos`）。

    ## 为什么 `_vis_frac` 单独用不够（用户实测指出）

    `2026-10-10-11-44-12`：托盘下方**整个出画**，OBB 的下边从画面外外推出来
    （`bl=(209,470) -> br=(616,483)`，画面只到 y=479），图像里根本没有这条边，
    可 `_vis_frac` 给 0.67 → 判可信 → 进 `use_pos` → **位姿整体歪 70px**。
    穷举子集证明下边是唯一把解带歪的那条：

        [上,左] / [右,上,左]  6~7px      摘掉下边
        [下,上,左] / 四条全用  79.76px    含下边就崩

    ## 三道闸门（全部**不看真值**，运行期可算）

    1. `_vis_frac >= EDGE_MIN_VIS`（原有）；
    2. 自己**几乎不在画面里**：`_safe_frac(SAFE_EPS_MM) < SAFE_MIN_FRAC`
       —— 整条贴着图边 = 画面裁出来的；
    3. 自己**贴边**（`_safe_frac(SAFE_EPS2_MM) < SAFE_MIN_FRAC2`）**且邻边也出画**
       —— "这一侧整个出画"。第 2 条单用会在 `2026-10-08-18-32-20_00040` 上出事
       （托盘占满画面高度，两条水平边都贴边 → 一起摘掉 → 只剩两条平行边 →
       秩崩 → 解跑到 94px），第 3 条把"整条在图外"与"图内但贴边"分开。

    ## 第 4 步：**秩保底**（`_rank_ok`）

    三道闸门各自都只看单条边，合起来仍可能把某**一侧**摘光。所以摘完再检查一次：
    不够就按 `_safe_frac(SAFE_EPS2_MM)` **从大到小**（= 最不像外推边的优先）
    把边加回来，直到秩够。

    ⚠️ 实测（现场 bag 814 帧，见 `debug/field_ab.py`）：加秩保底前 14 帧只剩 1 条或
    两条对边，最坏重投影误差 **620.8px**；加保底后最坏 **36.2px**，support 不变。
    obbval 601 帧上**一帧都不受影响**（那边本来就没有秩不够的帧）。
    """
    base = [i for i in range(4)
            if _vis_frac(quad[i], quad[(i + 1) % 4], w_img, h_img) >= EDGE_MIN_VIS]
    keep = [i for i in base if _edge_trust(quad, i, w_img, h_img)]
    if not _rank_ok(keep):
        cand = sorted((i for i in base if i not in keep),
                      key=lambda i: -_safe_frac(quad[i], quad[(i + 1) % 4],
                                                SAFE_EPS2_MM, w_img, h_img))
        for i in cand:
            keep.append(i)
            if _rank_ok(keep):
                break
    return sorted(keep)


def edge_usable(quad, i, w_img=640.0, h_img=480.0):
    """第 `i` 条边最终**在不在** `usable_edges(quad)` 里。诊断/测试用。"""
    return i in usable_edges(quad, w_img, h_img)


def _plane_dist_mm(n, depth, box, k, border=6, stride=1):
    """框内有效深度点沿 `n` 的**主峰中位距离**（毫米）。返回 `(D_mm, 峰占比)`。

    ⚠️ **不要改回"取全局中位再迭代收紧"**。框内深度沿法向是**多峰**的
    （台面 / 地面 / 近处的木档·箱子叠在一起），全局中位会落在两峰之间，
    迭代收紧于是收敛到**错的那一层**。实测（157 个"可用真值角 >= 2 且深度自洽"
    的裁判帧，基准 = 真值角反投影）：

        全局中位 + 迭代收紧   |D/D真值-1| 中位 3.98%  p90 **39.78%**  偏差>10% 45/157
        **主峰（本函数）**              中位 2.70%  p90  **4.56%**  偏差>10%  1/157

    坏得最狠的两个包（`2026-10-10-11-44-12` 比值 1.417、`2026-10-10-14-22-38`
    1.264、内点率只有 0.058）：其代表帧 `14-22-38_00000` 的直方图是

        [878,922)  65976 点   <- 台面（真值角反投影 935、OBB 边线解 934 都在这）
        [620,663)  33048 点   <- 旧算法选中的层（给出 724）
        [361,533)  18576 点   <- 地面
        其余                    <- 近处木档 / 箱子

    台面在图像里是**一整块**、占的面积最大，所以**多数帧最大的峰就是台面**。
    现场确认：托盘大面积出画的那几帧，D 的正确范围就在 880 附近。

    ⚠️ **但"多数帧"不等于"每一帧"**（2026-10-19 补）：托盘出画时会同时出现
    两个近等高的峰（台面 ~870-910、地面/别的层 ~650-690），`argmax` 只要近峰
    以 0.006~0.034 的占比优势胜出就会选错，D 锁到 657（低 26%）→ 位姿跳 300mm。
    所以现在走 `_pick_peak_bin()`：**在占比 >= `PEAK_MIN` 的峰里挑最接近
    `D_REF_MM` 的那个**；只有一个峰达标时与旧的 `argmax` **完全等价**。
    真机 881 帧实测：`>100mm` 的大跳 20 → 13，max 379.7 → 335.9mm，
    可见角点距 36.87 → 36.82px（不变）。

    第二个返回值是**选中那个峰**的占比（选中峰那个 bin 的点数 / 全部有效点数），
    用来判断"框里是不是一层主导平面" —— 与旧的"内点率"同用途、不同口径。

    `stride>1` 按行优先顺序对框内有效点跳采（`ys[::stride]`），只用于 POSE
    路线提速（同一帧的 D 只量一次时把这次测量本身也压一半）。`stride=1`
    与旧实现**逐位一致**。改动后须用 601 帧验证 D 的主峰没有漂（见
    `debug/pose_ab`）。OBB 路径（`frame_from_obb` / `solve_edges._resolve_d_lock`）
    不传 `stride`，保持默认 1。
    """
    import cv2
    h, w = depth.shape
    m = np.zeros((h, w), np.uint8)
    cv2.fillPoly(m, [np.round(np.asarray(box, np.float64)).astype(np.int32)], 1)
    m = cv2.erode(m, np.ones((border, border), np.uint8))
    ys, xs = np.nonzero((m > 0) & (depth > 0))
    if stride > 1:
        ys, xs = ys[::stride], xs[::stride]
    if len(ys) < 300:
        return None, 0.0
    z = depth[ys, xs] / 1000.0
    P = np.stack([(xs - k.cx) * z / k.fx, (ys - k.cy) * z / k.fy, z], 1)
    d = P @ np.asarray(n, float) * 1000.0            # 毫米
    lo, hi = float(d.min()), float(d.max())
    nb = max(4, int(round((hi - lo) / D_BIN_MM)))
    hist, edges = np.histogram(d, bins=nb)
    total = max(int(hist.sum()), 1)
    share = hist / total
    ipk = _pick_peak_bin(hist, edges, share)
    ctr = 0.5 * (float(edges[ipk]) + float(edges[ipk + 1]))
    sel = np.abs(d - ctr) <= D_PEAK_HALF_MM
    if sel.sum() < 100:
        return None, 0.0
    # 返回**选中那个峰**的占比（不是最大峰的）—— 下游用它判"这一层靠不靠得住"，
    # 选错峰时占比本来就该低，不能拿别的峰的数字去担保。
    return float(np.median(d[sel])), float(share[ipk])


def _pick_peak_bin(hist, edges, share):
    """选深度直方图的峰：多峰时**在占比达标的峰里挑最接近 `D_REF_MM` 的那个**。

    只有一个峰达标（绝大多数帧）时与旧的 `argmax` 完全等价 —— 这是本改动
    在多数帧上"零行为变化"的原因。

    `D_REF_MM` 只是**软先验**：它决定"两个近等高的峰里哪个是台面"，
    不参与任何硬闸门（锁不锁 D 由 `D_LOCK_INLIER` 判）。取 `abs(ctr)` 是因为
    法向可能翻过符号（调用方有时传未翻的 `nrm`），峰可能在负半轴。
    """
    ipk = int(np.argmax(hist))
    if PEAK_MIN <= 0.0:
        return ipk
    ok = np.flatnonzero(share >= PEAK_MIN)
    if ok.size <= 1:
        return int(ok[0]) if ok.size == 1 else ipk
    ctrs = 0.5 * (edges[ok] + edges[ok + 1])
    return int(ok[int(np.argmin(np.abs(np.abs(ctrs) - D_REF_MM)))])


def _edge_weights(quad, w_img=640.0, h_img=480.0):
    """四条边的量测权重。画面内那部分越短，权重越低（下限 `EDGE_W_MIN`）。

    ⚠️ `EDGE_WEIGHT_ON=False` 时必须返回**全 1.0**，不能靠把 floor 调大来关 ——
    那样会得到 0.05 的均匀权重，`soft_l1` 的 `f_scale=5.0` 就不再起作用（损失退化成
    二次），解算行为**变了**反而更差（实测 00240：真的全 1.0 时框到 x=770，
    均匀 0.05 时缩到 x=630）。关就用这个开关，别动 floor。
    """
    if not EDGE_WEIGHT_ON:
        return [1.0] * 4
    wt = []
    for i in range(4):
        f = _vis_frac(quad[i], quad[(i + 1) % 4], w_img, h_img)
        wt.append(1.0 if f >= EDGE_W_FLOOR else max(f / EDGE_W_FLOOR, EDGE_W_MIN))
    return wt


def _basis(nrm):
    """在 `nrm` 张成的平面里造一组正交基 `(e1, e2)`。"""
    n = np.asarray(nrm, float)
    n = n / np.linalg.norm(n)
    t = np.array([0.0, 0.0, 1.0])
    if abs(float(n @ t)) > 0.9:
        t = np.array([1.0, 0.0, 0.0])
    e1 = np.cross(t, n)
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    return e1, e2 / np.linalg.norm(e2)


def _rodrigues(axis, ang):
    a = float(np.linalg.norm(axis))
    if a < 1e-12:
        return np.eye(3)
    ax = np.asarray(axis, float) / a
    Kx = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(a) * Kx + (1 - np.cos(a)) * (Kx @ Kx)


def order_quad(q):
    """OBB 的四角按几何排成 `左下, 右下, 右上, 左上`（图像 v 向下）。

    **不设角序约定的输入靠这个归一** —— 与真值的 `name_corners` 同一套做法
    （y 分上下、组内按 x 分左右），两边就能逐点比。
    """
    q = np.asarray(q, float).reshape(4, 2)
    o = np.argsort(q[:, 1])
    top, bot = q[o[:2]], q[o[2:]]
    if top[:, 1].mean() > bot[:, 1].mean():
        top, bot = bot, top
    tl, tr = top[np.argsort(top[:, 0])]
    bl, br = bot[np.argsort(bot[:, 0])]
    return np.array([bl, br, tr, tl], float)


def _line_plane_of(p, q, k):
    """图像上过 `p`、`q` 的直线 -> 过相机光心的三维平面单位法向。

    原理：图像直线 `a·u + b·v + c = 0` 反投影成过光心的平面，法向
    `n = (a·fx, b·fy, a·cx + b·cy + c)`。
    """
    d = np.asarray(q, float) - np.asarray(p, float)
    d = d / np.linalg.norm(d)
    a, b = -d[1], d[0]
    c = -float(a * p[0] + b * p[1])
    n3 = np.array([a * k.fx, b * k.fy, a * k.cx + b * k.cy + c], float)
    nn = float(np.linalg.norm(n3))
    return n3 / nn if nn > 1e-12 else np.array([0.0, 0.0, 1.0])


def _to_spec(f, W, H, k, _checked=False):
    """把 `frame` 归一化到规格约定（见 `geom.py` 的说明）**而不改变矩形本身**。

    * **`W` 恒沿底边、`H` 恒沿右边 —— 不换位**；
    * `E1` 指画面右、`E2` 指画面里更上方，并保持 `E1 × E2 = nrm`；
    * `origin` = 四角里**投影 u 最小、v 最大**的那个（画面左下角）。

    ## 做法：先定角，再定符号

    曾经的写法是"按图像方向翻 E1/E2 的符号，最后再重挑一次 origin"。这条路上
    有个必须同时满足的约束：矩形四角是 `origin + a·E1 + b·E2`（`a∈{0,W}`、
    `b∈{0,H}`）。**光翻符号不把 `origin` 挪到对角，矩形就被整体搬走**；而
    翻完再"按投影重挑 origin"，挑中的角配翻转后的 `E1/E2` 会重建出**另一个
    矩形**（从该角沿 `-E2` 走，落在原矩形的另一侧）。实测
    `2026-10-10-11-44-12_00240`：矩形被搬到 `origin=[-2519, 402, 872]`，
    四角全在画面外，框整个消失。

    所以改成**先选角、再由这个角反推符号**，两者天然自洽：

    1. 用**原始** `E1/E2` 算出四角，取投影 `u-v` 最小的那个当规格 origin
       （画面左下角）；
    2. 从它出发的两条边，**按长度**认出哪条是 `W` 边（沿 `E1`）、哪条是 `H` 边；
    3. 各自的符号取"投影朝画面右 / 朝画面里更上方"的那个。

    这样 `origin` 与 `E1/E2` 一定描述**同一组四角**，不需要事后修补。

    ## 不变量（`_checked=False` 时函数末尾有断言守着）

    把 `E1` 翻符号、并把 `origin` 挪到**对角**（`o + W·E1`）得到的是描述
    **同一个 3D 矩形**的等价 frame（`E2` 同理，`s2` 组合亦然）—— 本函数对
    等价 frame 必须输出**同一组四角**。这是"先选角、再反推符号"的正确性判据；
    全 601 帧（含强制走退化/锁 D 分支）实测偏差 <= 7e-13 px。

    ⚠️ 该不变量成立的前提是 `proj_of` 里**先把向量单位化**：`o_new + 300·u`
    若拿未单位化的 `v`（长 1000 / 1200）去加，300mm 的取样点会落到矩形外、
    投影几乎不随 `u` 变，符号判反 —— 结果是把 `W`/`H` 认到**另一条边**上。
    """
    E1 = np.asarray(f['E1'], float)
    E2 = np.asarray(f['E2'], float)
    nrm = np.asarray(f['nrm'], float)
    o = np.asarray(f['origin'], float)

    corners = _corners_mm(dict(f, E1=E1, E2=E2, origin=o), W, H)
    px = _project_px(corners, k)
    m = int(np.argmin(px[:, 0] - px[:, 1]))          # 画面左下角
    o_new = corners[m]
    # 从该角出发的两条边（矩形四角按 (0,0),(W,0),(W,H),(0,H) 排，相邻即两族）
    # ⚠️ 认 W/H **必须用三维长度**：投影会被透视压缩，实测某帧两条边的
    # 投影长 269.7 / 287.6（看似后者的更接近 W），而三维长是 1000 / 1200 ——
    # 用投影长度会认反，把 W/H 配错边。
    e_a = corners[(m + 1) % 4] - o_new
    e_b = corners[(m + 3) % 4] - o_new
    if abs(float(np.linalg.norm(e_a)) - W) <= abs(float(np.linalg.norm(e_b)) - W):
        v1, v2 = e_a, e_b
    else:
        v1, v2 = e_b, e_a

    def proj_of(v):
        u = v / max(float(np.linalg.norm(v)), 1e-9)   # ⚠️ 必须单位化！
        p = _project_px(np.stack([o_new, o_new + 300.0 * u]), k)
        return float(p[1, 0] - p[0, 0]), float(p[1, 1] - p[0, 1])

    if proj_of(v1)[0] < 0:                            # E1 指画面右
        v1 = -v1
    if proj_of(v2)[1] > 0:                            # E2 指画面里更上方
        v2 = -v2
    g = dict(f)
    g['E1'] = v1 / np.linalg.norm(v1)
    g['E2'] = v2 / np.linalg.norm(v2)
    # `nrm` 由 `E1 × E2` 定（规格要求的就是这一条），不单独翻符号 ——
    # 下游（`refine3/_ridge_stage`、`refine4/_depth_stage`）本来就会把它翻到 z>0。
    g['nrm'] = np.cross(g['E1'], g['E2'])
    g['origin'] = o_new
    if not _checked:
        # 不变量（验收用）：等价 frame 归一 —— 翻 `E1` 的符号并把 `origin` 挪到
        # **对角**（`o + W·E1`；`E2` 同理挪 `+H·E2`）描述的是**同一个 3D 矩形**，
        # 必须输出**同一组四角**。全 601 帧（含强制走退化/锁 D 分支）实测偏差
        # <= 7e-13 px。
        #
        # ⚠️ 校验递归调用走 `_TO_SPEC_SELF`（本函数自身的引用）而不是全局名
        # `_to_spec`，这样诊断脚本把 `solve_obb._to_spec` 换成 spy/lambda 时
        # 不会被打断（`_checked=True` 的内层调用也不再递归）。
        g_flip = _TO_SPEC_SELF(dict(f, E1=-E1, origin=o + W * E1),
                               W, H, k, _checked=True)
        assert np.allclose(_project_px(_corners_mm(g, W, H), k),
                           _project_px(_corners_mm(g_flip, W, H), k), atol=1e-6), \
            '_to_spec 对等价 frame 给出了不同四角（矩形被搬走）'
    return g


_TO_SPEC_SELF = _to_spec       # 供上面的不变量校验递归调用（不受 monkeypatch 影响）


def _edge_dir_in_plane(p, q, e1, e2, k, depth_mm=1000.0):
    """图像上一条边 `p→q` 的方向 -> **托盘平面内**的方向（用 e1/e2 表出）。

    两个端点各自反投影成一条射线，与"过原点、法向 `n = e1×e2`、距离 `depth_mm`
    的平面"求交；两交点之差就是这条边在**该平面上**的方向。

    ⚠️ 必须真的求交、不能拿像素方向直接当三维方向：透视投影下同一条三维直线，
    在图像上的方向会随位置变，直接拿 `(Δu, Δv)` 映射回三维会有系统偏差。
    ⚠️ 用**有限距离**的平面（不是"无穷远"）：这个方向只在托盘所在的那一带才有意义。
    """
    n = np.cross(e1, e2)
    pts = []
    for uv in (p, q):
        v = np.array([(float(uv[0]) - k.cx) / k.fx,
                      (float(uv[1]) - k.cy) / k.fy, 1.0])
        denom = float(v @ n)
        if abs(denom) < 1e-9:
            return None
        pts.append(depth_mm / denom * v)
    d = np.asarray(pts[1]) - np.asarray(pts[0])
    d = d - float(d @ n) * n                    # 投回面内
    nn = float(np.linalg.norm(d))
    return d / nn if nn > 1e-12 else None


def _seed_from_obb(quad, e1, e2, k, long_side_parallel):
    """**面内转角种子 θ0 由 OBB 框自己的面内转角给出。**

    ⚠️ 这条是 2026-10-15 现场问题的根因修复，**别改回按"哪条基更水平"挑**：

    `_basis(normal)` 造出来的 `e1/e2` **只由法向决定，与托盘朝向无关** ——
    托盘在面内怎么转，`e1`/`e2` 投影到图像上的方向都不变。所以
    `_seed_from_long_side`（按"哪条基更水平"挑）给出的 θ0 **根本不随托盘转**，
    而解算只在 `θ0 ± INPLANE_LIMIT_DEG` 里搜 —— **托盘一旦转离那个固定
    方向超过这个窗口，解算必然被夹住，出来就永远是那个方向**。

    做法：`order_quad` 已经把四角排成 `[左下, 右下, 右上, 左上]`，于是
    `边0 = 左下→右下`（"底边"）、`边1 = 右下→右上`（"右边"）。把这条边的两个
    端点反投影到托盘平面上求方向，就是这条边在**面内**的方向。

    `long_side_parallel` 在这里的职责**只剩一件**：OBB 宽高比 ≈ 1，"哪条边是长边"
    在框上不可分，所以它决定**取底边还是取右边当 `E1`**（`E1` 恒是短边）。
    它**不是角度来源** —— 角度只能来自 OBB 框本身。

    ⚠️ **不要在这里"按投影活跃度再判一次哪条是短边"**：`order_quad` 的"上/下"
    分组在托盘转到 ≈45° 时本来就不稳定（两条边一样斜），再加一层判据只会在那
    一带引入翻边。`long_side_parallel` 是**用户给的先验**，直接用。
    """
    c = order_quad(quad)                        # [左下, 右下, 右上, 左上]
    idx = 0 if not long_side_parallel else 1
    d = _edge_dir_in_plane(c[idx], c[(idx + 1) % 4], e1, e2, k)
    if d is None:
        return None
    return float(np.arctan2(d @ e2, d @ e1)) % np.pi


def _seed_from_long_side(e1, e2, k, long_side_parallel, probe_mm=300.0):
    """按 `long_side_parallel` 先验给出面内转角种子 θ0。

    `θ` 的含义：`E1 = cosθ·e1 + sinθ·e2`，而 **`E1` 恒是短边**（`frame_from_obb`
    里 `Wb = min(size_mm)`）。所以 `θ = 0` 让短边沿 `e1`、`θ = π/2` 让它沿 `e2`。

    `long_side_parallel` 说明**与画面近平行的那条托盘边是不是长边**：

        False -> 平行的是**短边** -> 短边应当水平 -> 选投影更水平的那条基
        True  -> 平行的是**长边** -> 短边应当竖直 -> 选投影更竖直的那条基

    ⚠️ **不能写死 `θ0 = 0` 或 `π/2`**：`_basis` 有两个分支
    （`|n·z|>0.9` 走 `t=(1,0,0)`、否则 `t=(0,0,1)`），`e1` 的物理指向**随之翻转**。
    实测本数据集：`09-18-18-48-44`（`|n·z|=0.653`）的 `e1` 投影角 **-0.8°（水平）**，
    而 `10-10-14-22-38`（`|n·z|=0.977`）的 `e1` 投影角是 **90.0°（竖直）** ——
    写死数值会让其中一半的包错 90°。

    这里按"投影到图像上更水平/更竖直"直接选基，**与 `_basis` 走哪个分支无关**。
    """
    def dev_from_horizontal(v):
        d = _project_px(np.stack([np.zeros(3), probe_mm * np.asarray(v, float)]), k)
        a = float(np.degrees(np.arctan2(d[1, 1] - d[0, 1], d[1, 0] - d[0, 0]))) % 180.0
        return min(a, 180.0 - a)                     # 到水平线的夹角，[0, 90]

    dev1, dev2 = dev_from_horizontal(e1), dev_from_horizontal(e2)
    want_horizontal = not long_side_parallel          # 短边平行 -> 短边水平
    if want_horizontal:
        return 0.0 if dev1 <= dev2 else np.pi / 2
    return 0.0 if dev1 >= dev2 else np.pi / 2


def frame_from_obb(obb, k: CameraIntrinsics, normal,
                   size_mm=PALLET_SIZE_MM, tilt_deg=TILT_DEG,
                   depth=None, d_range=None, long_side_parallel=None):
    """OBB + 已知台面法向 + `long_side_parallel` -> `frame`。返回 `(frame | None, diag)`。

    ## `long_side_parallel`：面内转角的**唯一**依据（默认必传）

    OBB 的宽高比 1.034 ≈ 1，"哪条边是长边"在框上**根本不可分** ——
    所以不能靠残差去试两个候选（那是数据投票，同一场景不同帧会投出不同结果）。
    `long_side_parallel=True` 表示与画面近平行的那条托盘边**是长边**（⇒ 短边近平行时
    传 `False`）。它把 θ0 定死，解算只在 θ0 ± `INPLANE_LIMIT_DEG` 内找。
    ⚠️ **给错就明确偏 90°** —— 这是刻意的：错误应当可见，而不是被兜底掩盖。
    ⚠️ 传 `None` 会退回"两个候选都试、用残差挑"的旧行为，**只为离线诊断保留**，
    生产路径必须传值。

    ⚠️ **单位**：`obj` 与 `d` 都是**毫米**，解出来的 `origin` 就是毫米
    （曾经在这里乘过一次 1000，位姿跑到 1000 米外而重投影误差照样很小）。

    ## 出画的边：摘除 + 必要时锁深度

    托盘有一边跑到画面外时，那条边在图上只剩很短一截，它的**方向几乎全靠噪声**。
    两个措施：

    1. 有边**大部分出画**（可见占比 < `EDGE_MIN_VIS`）时，这条边**从位置约束里摘掉**
       （`use_pos`）。
    2. 剩下的边**定不住深度**时（不足 3 条、且没有一对对边），把平面距离 `d`
       **锁死**在深度测量值上（`depth` + `obb` 给出，主峰占比须 >= `D_LOCK_INLIER`）
       —— 那时 d 根本不可辨，锁上它只用剩下的自由度去贴合量到的边。

    第 2 条是必须的：边出画时位姿会沿"把整块托盘缩进画面"的方向滑动 —— 上边和
    左边照样贴得严丝合缝（垂距 ≈0），尺寸先验也照样满足，图像里**没有任何证据**
    能把它拉回来。实测 `2026-10-10-11-44-12_00240`：不锁时框缩到 x=635
    （OBB 是 664、真值 639），锁到深度之后回到 x=738。
    """
    quad = order_quad(obb['corners_px'])
    Wb, Hr = float(min(size_mm)), float(max(size_mm))    # 短边沿底边、长边沿右边
    # 用**四条边所在的直线**做量测（不是四个角）
    pls = [_line_plane_of(quad[i], quad[(i + 1) % 4], k) for i in range(4)]

    e1, e2 = _basis(normal)
    n0 = np.asarray(normal, float)
    n0 = n0 / np.linalg.norm(n0)
    if float(n0 @ np.array([0.0, 0.0, 1.0])) < 0:
        n0 = -n0                                        # 见文件头 ⚠️ 第 1 条
    free = tilt_deg > 0.0

    def frame_of(p):
        d, a, b, th = p[0], p[1], p[2], p[3]
        n = _rodrigues(p[4] * e1 + p[5] * e2, 1.0) @ n0 if free else n0
        c, s = np.cos(th), np.sin(th)
        return dict(origin=d * n + a * e1 + b * e2,
                    E1=c * e1 + s * e2, E2=-s * e1 + c * e2, nrm=n, W=Wb, H=Hr)

    # ⚠️ 出画的边**根本不参与位置约束**。
    #
    # OBB 的某条边大部分在画面外时，它的远端角点是 YOLO 从图外**外推**出来的
    # （`11-44-12_00240`：右边可见占比 3%，"右"角 x=637 而画面只到 639）。把这条
    # 外推出来的直线当量测，等于拿一串不存在的像素去约束位姿。
    #
    # 更致命的是**秩**：托盘只剩两条相邻边时能约束位姿的只有那半边的边，深度方向
    # 近乎不可观测（实测该帧雅可比最小奇异值 0.4 vs 最大 768）。在**画面外**的点上
    # 前后挪 20px，解出的 D 就从 1116 跳到 1158 —— 这个 D 完全是被噪声推出来的。
    #
    # 所以两件事一起做：
    #   1. **不可信**的边（`edge_usable`：`_vis_frac` + 内缩安全占比 + 邻边共同出画），
    #      从位置约束里**摘掉**；
    #   2. **只要有任意一条边不可信**，就把 d **锁死在深度测量值上**（`depth` 给出，
    #      主峰占比须 >= `D_LOCK_INLIER`），只解面内 (a, b, θ)。
    # 第 2 条不会像上一版那样把可见边带偏 —— 因为外推边已经不在方程里搅和了。
    #
    # ⚠️ **判据是"四条边全都可信"，不是旧的 `RANK_MIN_EDGES`（边数 >= 3 或有一对
    # 对边）**。旧判据把"边多"当成"D 解得准"的替身，实测是**反的**：现场 bag 815 帧
    # 按不可信边数分层（D0 = 本函数用边线几何解出的 D，D_depth = 深度全图主层）——
    #
    #     不可信边数 | 帧数 |  |D0| 中位 |  |D0| std | D_depth 中位 | 偏差中位
    #         0     |  105 |   962.4  |   92.3   |    903.0     |  10.2%
    #         1     |  526 |   722.1  |  117.0   |    899.2     |  19.7%   ← 最差
    #         2     |   60 |   899.1  |  108.6   |    904.1     |   0.4%   ← 准
    #         3     |  124 |   908.6  |    0.2   |    904.8     |   0.4%   ← 极准极稳
    #
    # 即"边数多"与"D 解得准"**无关** —— 深度方向在这套几何下近乎不可观测，多一条边
    # 只是多一组几乎不含深度信息的约束，反而把解往边上拽。所以只在**四条边一条不缺**
    # 时才把 D 当未知量，缺一条就锚到深度上。
    # ⚠️ `RANK_MIN_EDGES` 保留（别的模块/诊断脚本可能引用），但**本函数不再读它**。
    wt = np.sqrt(_edge_weights(quad))
    use_pos = usable_edges(quad)
    all_vis = len(use_pos) == 4
    d_lock = None
    d_info = None
    if depth is not None and not all_vis:
        Dm, peak = _plane_dist_mm(n0, depth, obb['corners_px'], k)
        if Dm and peak >= D_LOCK_INLIER:
            d_lock = float(Dm)
            if d_range is not None:
                d_lock = float(min(max(d_lock, d_range[0]), d_range[1]))
            d_info = (round(d_lock, 1), round(peak, 3))

    def res(p):
        if d_lock is not None:
            d_use, (a, b, th) = d_lock, (p[0], p[1], p[2])
            tilt = p[3:5] if free else None
        else:
            d_use, (a, b, th) = p[0], (p[1], p[2], p[3])
            tilt = p[4:6] if free else None
        c, s2 = np.cos(th), np.sin(th)
        n_ = _rodrigues(tilt[0] * e1 + tilt[1] * e2, 1.0) @ n0 if free else n0
        C = _corners_mm(dict(origin=d_use * n_ + a * e1 + b * e2,
                             E1=c * e1 + s2 * e2, E2=-s2 * e1 + c * e2,
                             nrm=n_, W=Wb, H=Hr), Wb, Hr)
        r = []
        for i in use_pos:
            for v in (C[i], C[(i + 1) % 4]):            # 这条边的两个端点都在平面上
                r.append(float(pls[i] @ v) * wt[i])
        if free:
            r.append(tilt_rho_ * 400.0 * float(np.hypot(tilt[0], tilt[1])))
        return np.asarray(r)

    tilt_rho_ = TILT_RHO
    best = None
    # θ 的种子：`long_side_parallel` 直接定死（见 docstring）；只有传 None 时才退回
    # "两个候选都试、用残差挑"的旧行为（**离线诊断专用**）。
    if long_side_parallel is None:
        th_seeds = (0.0, np.pi / 2)
    else:
        _s = _seed_from_obb(quad, e1, e2, k, long_side_parallel)
        th_seeds = (_s,) if _s is not None else \
            (_seed_from_long_side(e1, e2, k, long_side_parallel),)
    for th0 in th_seeds:
        # 参数向量：锁 D 时 (a, b, θ[, tilt1, tilt2])，否则 (d, a, b, θ[, ...])。
        # θ 那一位的边界必须**以 th0 为中心**（见文件头 ⚠️ 第 2 条）。
        if d_lock is not None:
            x0v, lov, hiv, xsv = ([0.0, 0.0], [-1500.0, -1500.0],
                                  [1500.0, 1500.0], [300.0, 300.0])
        else:
            x0v, lov, hiv, xsv = ([DEPTH_SEED_MM, 0.0, 0.0],
                                  [DEPTH_RANGE_MM[0], -1500.0, -1500.0],
                                  [DEPTH_RANGE_MM[1], 1500.0, 1500.0],
                                  [500.0, 300.0, 300.0])
        x0v.append(th0)
        lov.append(th0 - np.radians(INPLANE_LIMIT_DEG))
        hiv.append(th0 + np.radians(INPLANE_LIMIT_DEG))
        xsv.append(0.05)
        if free:
            x0v += [0.0, 0.0]
            lov += [-np.radians(tilt_deg)] * 2
            hiv += [np.radians(tilt_deg)] * 2
            xsv += [0.05, 0.05]
        try:
            sol = least_squares(res, np.array(x0v), loss='soft_l1', f_scale=5.0,
                                max_nfev=400, bounds=(lov, hiv), x_scale=xsv)
        except Exception:                               # noqa: BLE001
            continue
        if best is None or sol.cost < best.cost:
            best = sol
    if best is None:
        return None, {'reject': 'lsq_failed'}
    if d_lock is None:
        f = frame_of(best.x)
    else:
        a, b, th = best.x[0], best.x[1], best.x[2]
        n_ = n0
        c, s2 = np.cos(th), np.sin(th)
        f = dict(origin=d_lock * n_ + a * e1 + b * e2, E1=c * e1 + s2 * e2,
                 E2=-s2 * e1 + c * e2, nrm=n_, W=Wb, H=Hr)
    found = _to_spec(f, Wb, Hr, k)                      # 见文件头 ⚠️ 第 3 条
    px = _project_px(_corners_mm(found, Wb, Hr), k)
    reproj = float(np.median(np.linalg.norm(px - quad, axis=1)))
    if free:
        tilt = float(np.degrees(np.hypot(best.x[4] if d_lock is None else best.x[3],
                                         best.x[5] if d_lock is None else best.x[4])))
    else:
        tilt = 0.0
    return found, {'method': 'obb4dof', 'cost': round(float(best.cost), 2),
                   'd_lock': d_info,
                   'reproj_px': round(reproj, 3),
                   'depth_mm': round(float(best.x[0] if d_lock is None else d_lock), 1),
                   'inplane_deg': round(float(np.degrees(
                       best.x[3] if d_lock is None else best.x[2])), 2),
                   'tilt_deg': round(tilt, 2)}
