#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**平面距离 D 的范围搜索**。

`D` = 相机原点到台面、沿台面法向的距离（**毫米**）。它是整条链路里唯一一个
"看图看不出来"的量：给定法向之后，托盘的远近只表现为投影尺寸，而投影尺寸和
"托盘多大"是**不可分**的 —— 远一点的大托盘和近一点的小托盘在图像上完全一样。

所以 D 必须由**外部先验**兜住。部署时机位固定，这个范围可以直接量出来：

    import dsearch
    dsearch.D_RANGE_MM = (750.0, 1100.0)      # 只在这个范围内找 D
    # 或逐帧传：refine4(..., d_range=(700.0, 1000.0))

## 什么时候用得上

只有**退化帧**用得上 —— 量到的边里没有任何一对对边、且总数 < 3（托盘一半出画、
只剩两条相邻边）。这时 D 在数学上**完全不可辨**（Jacobian 最小奇异值为 0），
"框的大小"会原样继承初值，表现就是"绿框比托盘小一圈"。

## 怎么在范围内挑

对每个候选 D：固定它，解面内 (a, b, θ) 贴合**量到的边线**，再拿**没量到的那几条边**
的脊线支持度打分（打分逻辑在 `refine3._ridge_stage`，不在这儿）。

⚠️ 两个坑：
* 打分只能用**没量到的边**。量到的那几条是按它们拟合出来的，天生 0 残差，
  算进去只会稀释分辨力。
* 若没量到的边也全是**出画**的（`14-22-38_00990` 就是这样，只剩两条边可见），
  分数处处为 0，argmax 退化成抽签、会跳到范围端点。所以范围**宁可宽**。

## 成本

粗扫 `D_SEARCH_N` 点 + `D_SEARCH_ROUNDS` 轮加密，只在退化帧上跑。实测本机
（4090、`OMP_NUM_THREADS=1`）整链路约 92ms/帧（不设范围 87ms）。
"""
from __future__ import annotations

import numpy as np


# D 的**合法范围**（毫米），None = 不约束（默认）。
#
# ⚠️ 这份采集数据横跨多台机位，工作距离从 ~650 到 ~1650mm 都有，**一个窄范围
# 盖不住**（750~1100 只覆盖 6/13 个包）。而且在能覆盖的那部分帧上它也是负收益：
# 全量中位 5.22 -> 5.30，31 帧变差、4 帧变好。
#
# **部署时按那台机位量一个真实范围再设**，用 `refine4(..., d_range=(lo, hi))`
# 或直接改这里。它的用途是挡住离群的 D，不是当精确约束用。
D_RANGE_MM = None

# 在范围内怎么挑 D：由 `refine3._ridge_stage` 做 —— 对每个候选 D 解一次面内
# (a,b,θ) 贴合量到的边线，再拿"没量到的那几条边"的脊线支持度投票。见那里的注释。
D_SEARCH_N = 9        # 粗扫点数
D_SEARCH_ROUNDS = 2   # 加密轮数（每轮步长缩到 1/3）
D_SEARCH_GAIN = 0.02  # 搜索的胜出裕度：候选 D 的支持度要**超过 d0 基准这么多**才采纳。
                      # 没有这个门就会在"打分无分辨力"的帧上抽签（见 refine3 注释）。


def clamp_D(D, rng):
    """把 D 夹进范围。`rng=None` 时原样返回。"""
    if rng is None or D is None:
        return D
    lo, hi = float(rng[0]), float(rng[1])
    return float(min(max(float(D), lo), hi))


def D_from_frame(frame, normal=None) -> float:
    """`frame` → 平面距离 D（**毫米**）= `origin · n̂`。**D 反解的单一真相源。**

    `frame` 的约定是 `origin = d·n + a·e1 + b·e2`（`solve_obb` / `refine_depth`），
    其中 `e1,e2 ⊥ n`、`|n|=1`，所以点乘法向后 `a·e1`、`b·e2` 两项归零，
    剩下的正是那个 `d`。

    ⚠️ **法向必须翻到 `z>0` 再点乘。** `solve_obb._to_spec` 里明写 `nrm` 由
    `E1 × E2` 定、**不单独翻符号**（相机俯视台面时它是 z<0）。不翻的话
    `origin · nrm` 是**负的 D**，量纲对、符号错。

    `normal` 给了就用它（不读 `frame['nrm']`），不给就用 `frame['nrm']`。
    两种口径在 `solve_obb.TILT_DEG == 0`（现行值，硬锁）下数值相同 —— 法向被锁死成
    先验，`frame['nrm']` 就是翻转后的先验法向。⚠️ **但只到浮点末位**：`frame['nrm']`
    是 `_to_spec` 用 `cross(E1, E2)` **重算**出来的，与传进来的 `normal` 不是同一个
    浮点数（601 帧实测最大差 4.5e-13mm）。对 10% 量级的闸门无影响，但**别拿它当
    逐位一致的依据**。若将来放开倾斜，两者会真的分离，届时以显式传 `normal` 为准。

    ⚠️ **要逐位复现旧代码的行为，要传原始 `normal`、不要传已归一化的那个。**
    旧实现（原 `user_rule` 内联）的运算序列是「`asarray` → 除以 `norm` → 判符号」；
    传一个已经归一化过的向量进来会**多除一次**，末位可能差 1 ulp。
    """
    n = np.asarray(frame['nrm'] if normal is None else normal, float)
    n = n / np.linalg.norm(n)
    if float(n @ np.array([0.0, 0.0, 1.0])) < 0.0:
        n = -n
    return float(np.asarray(frame['origin'], float) @ n)


def D_dev(D, ref_mm) -> float:
    """`D` 相对 `ref_mm` 的偏差 `|D/ref - 1|`。

    `ref_mm <= 0` 返回 `inf` —— 调用方据此把闸门**关掉**（而不是判成超限）。
    """
    r = float(ref_mm)
    if r <= 0.0:
        return float('inf')
    return abs(float(D) / r - 1.0)
