#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`pallet_frame` 配对判据的合成自检。跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 skills/atomic/perception/pallet_frame/tests/test_pairing.py

**不需要 ROS、不需要相机、不需要任何数据** —— 时间戳自己造。

**不用相对 import**：本文件一律走绝对路径 `from skills.atomic.perception...`，
这样 pytest 收集得到（`pytest --collect-only` 对本文件仍是 0 条：函数名不是
`test_*`、也不在 pytest.ini 的 testpaths 里），直接 `python3 <本文件>` 跑时也
不依赖 `__package__`。仓库里其余测试脚本的约定是「直接跑、退出码 0/1」。
"""
from __future__ import annotations

import sys
from pathlib import Path

if __name__ == '__main__':
    # parents[5]：tests/ → pallet_frame → perception → atomic → skills → 仓库根。
    # 补上仓库根，下面的绝对 import 才找得到 `skills` 包。
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))

import numpy as np

from skills.atomic.perception.pallet_frame.algorithm import (
    PalletObservation,
    PairReject,
    nearest_pair,
)

FAILS = []


def check(name, cond, detail=""):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        FAILS.append(name)


def main():
    print("== nearest_pair：同帧配得上 ==")
    # 托盘耗时 500ms、箱子耗时 60ms，但两者看的是同一帧图像（stamp 都是 3.300）
    r = nearest_pair([3.300, 3.100], [3.300, 3.100], max_dt_s=0.05)
    check("同帧配对成功", isinstance(r, tuple), repr(r))
    check("选中的是最近的", r == (0, 0), repr(r))

    print("== 耗时差不是问题（同一个 stamp） ==")
    r = nearest_pair([10.0], [10.0], max_dt_s=0.05)
    check("同一个 stamp 一定配上", r == (0, 0), repr(r))

    print("== 差一帧配不上 ==")
    # 30fps 下一帧 = 33ms；差两帧 = 66ms > 50ms
    r = nearest_pair([3.300], [3.366], max_dt_s=0.05)
    check("超阈值返回 PairReject", isinstance(r, PairReject), repr(r))
    check("code 是 pair_dt", getattr(r, 'code', '') == 'pair_dt', repr(r))
    check("detail 带实际值与阈值",
          '0.066' in r.detail and '0.05' in r.detail, repr(r))

    print("== detail 在 epoch 量级下不自相矛盾 ==")
    # 真实 ROS 时间戳量级（~1.7e9）下 dt 落在 2.4e-7 的网格上：这一对的实际差是
    # 0.0500001907...，`.6f` 会把它印成 "0.050000"，日志变成「差 0.050000 却超过
    # 0.05」——看日志的人第一反应是代码有 bug。`.9f` 必须把实际值印出来。
    base = 1700000000.0
    b = base + 0.0500002
    dt_epoch = abs(b - base)
    r = nearest_pair([base], [b], max_dt_s=0.05)
    check("epoch 量级超阈值仍判 pair_dt",
          isinstance(r, PairReject) and r.code == 'pair_dt', repr(r))
    check("dt 打印到 9 位小数（能看出确实超了）",
          f'{dt_epoch:.9f}' in r.detail, repr(r.detail))
    check("stamp 打印到 9 位小数",
          f'{b:.9f}' in r.detail, repr(r.detail))

    print("== 只有一边 ==")
    r = nearest_pair([], [3.3], max_dt_s=0.05)
    check("空托盘 → pair_none", isinstance(r, PairReject) and r.code == 'pair_none', repr(r))
    check("pair_none 的 detail 带两边计数（托盘 0）",
          '托盘 0' in r.detail and '箱子 1' in r.detail, repr(r))
    r = nearest_pair([3.3], [], max_dt_s=0.05)
    check("空箱子 → pair_none", isinstance(r, PairReject) and r.code == 'pair_none', repr(r))
    check("pair_none 的 detail 带两边计数（箱子 0）",
          '托盘 1' in r.detail and '箱子 0' in r.detail, repr(r))

    print("== 多对多：挑全局最近的一对 ==")
    r = nearest_pair([1.00, 2.00, 3.00], [1.01, 2.50], max_dt_s=0.05)
    check("挑到 1.00/1.01", r == (0, 0), repr(r))
    r = nearest_pair([1.00, 2.00, 3.00], [2.49, 3.02], max_dt_s=0.05)
    check("挑到 3.00/3.02", r == (2, 1), repr(r))

    print("== 并列最小 dt：先遇到者胜 ==")
    # 两边 dt 都是 0.01（1.0/1.01 与 1.02/1.01），规则是保留字典序最小的 (i, j)
    r = nearest_pair([1.0, 1.02], [1.01], max_dt_s=0.05)
    check("并列取先遇到的 (0, 0)", r == (0, 0), repr(r))

    print("== max_dt_s 边界是闭区间（向量二进制精确可表示） ==")
    # 1.25 - 1.0 == 0.25 在二进制浮点下**精确成立**，所以这条用例能真正区分
    # `dt > max`（配上）与 `dt >= max`（拒绝）。用 1.05 - 1.00 就不行：它是
    # 0.050000000000000044，严格大于 0.05，两种实现都判拒绝，抓不住变异。
    r = nearest_pair([1.0], [1.25], max_dt_s=0.25)
    check("正好等于阈值算配上", r == (0, 0), repr(r))
    check("边界向量确实精确等于阈值", (1.25 - 1.0) == 0.25, repr(1.25 - 1.0))

    print("== 非有限时间戳必须拒绝（不能静默配上） ==")
    nan = float('nan')
    r = nearest_pair([nan, 3.3], [3.3], max_dt_s=0.05)
    check("[nan, 3.3] vs [3.3] → PairReject（不选 NaN 那个）",
          isinstance(r, PairReject), repr(r))
    check("[nan, 3.3] vs [3.3] → code 是 pair_nan",
          getattr(r, 'code', '') == 'pair_nan', repr(r))
    check("[nan, 3.3] 的 detail 带实际值（计数与下标）",
          '1/2' in r.detail and '[0]' in r.detail, repr(r.detail))

    r = nearest_pair([nan], [3.3], max_dt_s=0.05)
    check("[nan] vs [3.3] → PairReject（单个 NaN 也配不上）",
          isinstance(r, PairReject), repr(r))
    check("[nan] vs [3.3] → code 是 pair_nan",
          getattr(r, 'code', '') == 'pair_nan', repr(r))
    check("[nan] 的 detail 带实际值（原始列表）",
          'nan' in r.detail and '3.3' in r.detail, repr(r.detail))

    r = nearest_pair([3.3], [nan, float('inf')], max_dt_s=0.05)
    check("箱子侧非有限同样拒绝",
          isinstance(r, PairReject) and r.code == 'pair_nan', repr(r))
    check("箱子侧 detail 带实际值",
          '2/2' in r.detail, repr(r.detail))

    print("== 阈值本身坏了必须拒绝（判在 pair_none 之前） ==")
    # 只过滤两侧时间戳是不够的：`dt > nan` 恒为 False，阈值是 NaN 时任何一对都
    # 被当成配上 —— 门禁静默失效且一个 reject 都不报。上层 PalletFrameWindow
    # 从 ROS 参数读 max_dt_s，参数没设/类型转换失败拿到的正是 NaN。
    r = nearest_pair([1.0], [3.0], float('nan'))
    check("max_dt_s=nan → pair_threshold（差 2 秒的不能配上），detail 带实际值",
          isinstance(r, PairReject) and r.code == 'pair_threshold'
          and 'nan' in r.detail, repr(r))

    r = nearest_pair([1.0], [3.0], float('inf'))
    check("max_dt_s=inf → pair_threshold（inf 会放行任意差），detail 带实际值",
          isinstance(r, PairReject) and r.code == 'pair_threshold'
          and 'inf' in r.detail, repr(r))

    r = nearest_pair([1.0], [3.0], 0.0)
    check("max_dt_s=0.0 → pair_threshold（配置写错，不是「严格」），detail 带实际值",
          isinstance(r, PairReject) and r.code == 'pair_threshold'
          and '0.0' in r.detail, repr(r))

    print("== 两侧同时 <= 0 必须拒绝（单侧为 0 是合法的仿真时间） ==")
    # rospy.Time().to_sec() 默认就是 0.0；两路同时停在 0 上，真实时刻更可能是
    # 「一个墙钟、一个 0」，差着整个 epoch。
    r = nearest_pair([0.0], [0.0], max_dt_s=0.05)
    check("[0.0] vs [0.0] → pair_stamp_zero（不能静默配上），detail 带两侧实际值",
          isinstance(r, PairReject) and r.code == 'pair_stamp_zero'
          and '0.000000000' in r.detail, repr(r))

    # 反向：单侧为 0 不能误伤 —— use_sim_time 下第一帧的 stamp 合法地就是 0。
    # 这条是防「判据写过头」的：改成「任一侧 <= 0 就拒」时它必须变红。
    r = nearest_pair([0.0], [0.05], max_dt_s=0.05)
    check("[0.0] vs [0.05] → 要能配上（单侧为 0 合法）", r == (0, 0), repr(r))

    print("== PalletFrameWindow：配对 + 平滑 ==")
    from skills.atomic.perception.pallet_frame.window import PalletFrameWindow
    from skills.atomic.perception.pallet_servo.algorithm import BoxObservation

    def _T(x):
        T = np.eye(4)
        T[0, 3] = x / 1000.0          # 毫米 → 米
        return T

    def _quad(du=0.0):
        return [[980.0 + du, 1080.0], [700.0 + du, 1120.0],
                [680.0 + du, 800.0], [960.0 + du, 760.0]]

    w = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=3)
    check("空窗 resolve 返回 None", w.resolve() is None)

    # 只有托盘、没有箱子 → pair_none
    w.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=1.00))
    r = w.resolve()
    check("只有托盘 → pair_none",
          isinstance(r, PairReject) and r.code == 'pair_none', repr(r))

    # 补上箱子 → 配成对
    w.push_box(BoxObservation(quad=_quad(), stamp=1.00))
    r = w.resolve()
    check("配上对返回 PairedFrame", hasattr(r, 'n_pairs'), repr(r))
    check("第一对 n_pairs=1", r.n_pairs == 1, repr(r))
    check("dt_s 是 0", abs(r.dt_s) < 1e-9, repr(r))

    # **同一对再 resolve 一次不该重复报**（节点每 tick 都调）
    check("同一对不重复报", w.resolve() is None)

    # 再来两对 → 窗满 3
    for k, ts in enumerate([1.10, 1.20]):
        w.push_pallet(PalletObservation(T_cam_pallet=_T(10.0 * (k + 1)), stamp=ts))
        w.push_box(BoxObservation(quad=_quad(du=2.0 * (k + 1)), stamp=ts))
        r = w.resolve()
    check("窗满后 n_pairs=3", r.n_pairs == 3, repr(r))

    # 平滑：三对的托盘 x 分别是 0/10/20mm → 均值 10mm
    check("托盘位置取平均", abs(r.T_cam_pallet[0, 3] - 0.010) < 1e-9,
          repr(r.T_cam_pallet[0, 3]))
    # 箱子的 du 分别是 0/2/4 → 均值 2
    check("箱子四角逐点平均", abs(r.box_quad[0, 0] - 982.0) < 1e-9,
          repr(r.box_quad[0, 0]))

    # 超过窗长的旧对要被丢掉：再推一对，n_pairs 仍是 3
    w.push_pallet(PalletObservation(T_cam_pallet=_T(30.0), stamp=1.30))
    w.push_box(BoxObservation(quad=_quad(du=6.0), stamp=1.30))
    r = w.resolve()
    check("窗长封顶 3", r.n_pairs == 3, repr(r))
    check("最旧的一对被丢掉（x 均值 = (10+20+30)/3）",
          abs(r.T_cam_pallet[0, 3] - 0.020) < 1e-9, repr(r.T_cam_pallet[0, 3]))

    print("== stale 检查 ==")
    w2 = PalletFrameWindow(max_dt_s=0.05, window=1, stale_s=0.5)
    w2.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=1.00))
    w2.push_box(BoxObservation(quad=_quad(), stamp=1.00))
    r = w2.resolve(now=1.20)
    check("不陈旧 → 正常出", hasattr(r, 'n_pairs'), repr(r))
    w2.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=2.00))
    w2.push_box(BoxObservation(quad=_quad(), stamp=2.00))
    r = w2.resolve(now=2.80)
    check("陈旧 → stale",
          isinstance(r, PairReject) and r.code == 'stale', repr(r))

    print("== 刚体平均：只有旋转不同时，平均后必须仍是旋转矩阵 ==")
    # `average_pairs` 若把托盘位姿写成逐元素线性平均（`np.mean`），两个各转
    # 90°、平移相同的位姿平均出来会得到 0 矩阵 —— det=0、R@R.T≠I。而它照样
    # 能当"位姿"用下去，参考边投影出来是歪的，且**静默**。
    from skills.atomic.perception.pallet_frame.algorithm import average_pairs

    def _rot_z(deg):
        T = np.eye(4)
        c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
        T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
        return T

    rot_pairs = [(PalletObservation(T_cam_pallet=_rot_z(0.0), stamp=1.0),
                  BoxObservation(quad=_quad(), stamp=1.0)),
                 (PalletObservation(T_cam_pallet=_rot_z(90.0), stamp=1.1),
                  BoxObservation(quad=_quad(du=2.0), stamp=1.1))]
    pf = average_pairs(rot_pairs)
    R = pf.T_cam_pallet[:3, :3]
    check("平均后的旋转块 det ≈ +1", abs(np.linalg.det(R) - 1.0) < 1e-9,
          repr(np.linalg.det(R)))
    check("平均后的旋转块是正交阵（R @ R.T ≈ I）",
          float(np.max(np.abs(R @ R.T - np.eye(3)))) < 1e-9,
          repr(float(np.max(np.abs(R @ R.T - np.eye(3))))))
    # 0° 与 90° 的刚体平均是 45°（SVD 正交化），不是 0 矩阵
    check("0° 与 90° 的刚体平均是 45°",
          abs(pf.T_cam_pallet[0, 0] - np.cos(np.radians(45.0))) < 1e-9,
          repr(pf.T_cam_pallet[0, 0]))

    print("== 坏观测（两侧 stamp 同时为 0）不该挤掉好观测 ==")
    # `nearest_pair` 的 pair_stamp_zero 判在"选出全局最小 dt 之后"，而坏对的 dt
    # 往往是 0（两侧都停在 0.0 上），天然是"最小"的那一个。若不先跳过坏观测再选，
    # 整次 resolve 会被 reject，好观测被坏观测挤掉 —— 而坏观测要等 pair_cache
    # 次推入才出得去，这期间每个 tick 都是同一个 reject，伺服一个数都出不来。
    w3 = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=1)
    w3.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=0.0))
    w3.push_box(BoxObservation(quad=_quad(), stamp=0.0))
    w3.push_pallet(PalletObservation(T_cam_pallet=_T(5.0), stamp=1.00))
    w3.push_box(BoxObservation(quad=_quad(du=3.0), stamp=1.00))
    r = w3.resolve()
    check("坏对被跳过，好观测照常配上", hasattr(r, 'n_pairs'), repr(r))
    check("配的是好观测（x=5mm，不是坏对的 0）",
          hasattr(r, 'n_pairs') and abs(r.T_cam_pallet[0, 3] - 0.005) < 1e-9,
          repr(r))

    # 反向：两侧**都只有** 0.0（真实故障：两路都忘了填 header）时不能静默吞掉，
    # 必须仍然报 pair_stamp_zero —— 剔完两边都空会变成 pair_none，操作员看到的
    # 就成了"没有观测"而不是"时间戳没填"。
    w4 = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=1)
    w4.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=0.0))
    w4.push_box(BoxObservation(quad=_quad(), stamp=0.0))
    r = w4.resolve()
    check("两侧都只有 0.0 → 仍报 pair_stamp_zero（不静默吞成 pair_none）",
          isinstance(r, PairReject) and r.code == 'pair_stamp_zero', repr(r))

    print("== 稳态下每一帧都出数（旧对不能被反复重报） ==")
    # `nearest_pair` 并列 dt 时"先遇到者胜"，而缓存里越旧的观测下标越小 —— 稳态
    # 下（每一对的 dt 都是 0）选出来的永远是**最早那一对**。去重若只比对一次就
    # 返回 None，窗在报出第一对之后会**永久沉默**（直到旧对被 pair_cache 挤出去，
    # 才吐出一个过期好几帧的位姿）。
    w5 = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=5)
    got = []
    for ts in [1.00, 1.10, 1.20, 1.30, 1.40, 1.50]:
        w5.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=ts))
        w5.push_box(BoxObservation(quad=_quad(), stamp=ts))
        got.append(w5.resolve())
    check("6 帧全出数（不是只出第一帧）",
          all(hasattr(x, 'n_pairs') for x in got), repr([repr(x) for x in got]))
    check("每帧报的都是那一帧的最新对（pallet_stamp 递增到 1.50）",
          [round(x.pallet_stamp, 2) for x in got if hasattr(x, 'n_pairs')]
          == [1.00, 1.10, 1.20, 1.30, 1.40, 1.50],
          repr([getattr(x, 'pallet_stamp', None) for x in got]))
    check("同一 tick 重复 resolve 仍是 None（不重复出数）",
          w5.resolve() is None)

    print("== 只有一边推进（托盘慢 500ms）不该报 pair_none 刷屏 ==")
    # 托盘检测慢一个量级时，箱子会一直领先几百毫秒 —— 这是**稳态**。这时窗里
    # "比上次更新的箱子" 有、托盘没有，不能每个 tick 报一条 "托盘 0 个观测"。
    w6 = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=3)
    w6.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=1.00))
    w6.push_box(BoxObservation(quad=_quad(), stamp=1.00))
    r = w6.resolve()
    check("先出一对", hasattr(r, 'n_pairs'), repr(r))
    for ts in [1.10, 1.20, 1.30]:                 # 只有箱子在推进
        w6.push_box(BoxObservation(quad=_quad(), stamp=ts))
        r = w6.resolve()
        check(f"只有箱子推进（{ts}）→ None（没有新的配对，不是 pair_none）",
              r is None, repr(r))

    print("== spread_*：窗内各帧到**平均值**的最远距离（不是两两最大差） ==")
    # `PairedFrame.spread_mm` / `spread_deg` 是 brief 明确承诺的输出，Task 6 的
    # 字段表要写它、render/自检层拿它判"这一窗稳不稳"。它恒为 0 是**静默错**，
    # 所以这里把口径钉死：参照物是**平均值**，不是"别的帧"。
    def _pair(x_mm, deg, stamp):
        return (PalletObservation(T_cam_pallet=_rot_z(deg) @ _T(x_mm),
                                  stamp=stamp),
                BoxObservation(quad=_quad(), stamp=stamp))

    # 只有平移差：0 / 30 / 60mm → 均值 30mm，最远的那两帧各离均值 30mm。
    # 注意 60mm 那帧离 0mm 那帧差 60mm —— 若实现写成"两两最大差"，这里会得到
    # 60 而不是 30，所以这条断言能区分两种口径。
    pf = average_pairs([_pair(0.0, 0.0, 1.00),
                        _pair(30.0, 0.0, 1.10),
                        _pair(60.0, 0.0, 1.20)])
    check("只有平移差：spread_mm == 30.0（到均值的最远距离）",
          abs(pf.spread_mm - 30.0) < 1e-9, repr(pf.spread_mm))
    check("只有平移差：spread_deg == 0.0（朝向没变）",
          abs(pf.spread_deg) < 1e-9, repr(pf.spread_deg))

    # 两帧差 30mm → 各自离均值 15mm。这条专门钉"到均值"这个口径：
    # 写成"到其他帧的最大距离"时它是 30，写成"到均值"时它是 15。
    pf = average_pairs([_pair(0.0, 0.0, 1.00), _pair(30.0, 0.0, 1.10)])
    check("两帧差 30mm → spread_mm == 15.0（不是 30）",
          abs(pf.spread_mm - 15.0) < 1e-9, repr(pf.spread_mm))

    # 只有旋转差：0° 与 90° → 刚体平均是 45°，各自离均值 45°
    pf = average_pairs([_pair(0.0, 0.0, 1.00), _pair(0.0, 90.0, 1.10)])
    check("只有旋转差：spread_deg == 45.0（到均值的最远转角）",
          abs(pf.spread_deg - 45.0) < 1e-9, repr(pf.spread_deg))
    check("只有旋转差：spread_mm == 0.0（位置没变）",
          abs(pf.spread_mm) < 1e-9, repr(pf.spread_mm))

    print("== agg：mean / median 是两条不同的路径，各自等于手算值 ==")
    # Task 5 的 `agg` 参数可配 mean/median，brief 说两条都要。只断言"两者不相等"
    # 钉不住哪个是哪个 —— 这里把两条路径的**手算值**都写死。
    # 四对箱子的 du 分别是 0/0/0/60（一个离群点），mean 被离群点拽走、median 不。
    # **托盘位姿必须互不相同**：四对的托盘 x 是 0/0/0/60mm、各自再转 0/10/20/30°。
    # 若四对都是同一个单位阵，"agg 不影响托盘位姿"那条断言对任何聚合器都恒真
    # （作用在同一个矩阵上结果都一样）—— 那种断言看起来在覆盖刚体平均，其实
    # 什么都没覆盖。位姿互不相同之后，mean 与 median 在托盘上也不同，断言才有
    # 判别力：托盘被误接上 median 时立刻变红。
    quad_pairs = [
        (PalletObservation(
            T_cam_pallet=_rot_z(10.0 * k) @ _T(x_mm), stamp=1.0 + 0.1 * k),
         BoxObservation(quad=_quad(du=du), stamp=1.0 + 0.1 * k))
        for k, (du, x_mm) in enumerate([(0.0, 0.0), (0.0, 0.0),
                                        (0.0, 0.0), (60.0, 60.0)])]
    pf_mean = average_pairs(quad_pairs, agg="mean")
    pf_med = average_pairs(quad_pairs, agg="median")
    # quad[0][0] = 980+du → 980/980/980/1040：mean = 995，median = (980+980)/2 = 980
    check("agg=mean：quad[0][0] == 995.0（被离群点拽走）",
          abs(pf_mean.box_quad[0, 0] - 995.0) < 1e-9, repr(pf_mean.box_quad[0, 0]))
    check("agg=median：quad[0][0] == 980.0（离群点不动它）",
          abs(pf_med.box_quad[0, 0] - 980.0) < 1e-9, repr(pf_med.box_quad[0, 0]))
    # quad[1][0] = 700+du：mean = 715，median = 700
    check("agg=mean：quad[1][0] == 715.0",
          abs(pf_mean.box_quad[1, 0] - 715.0) < 1e-9, repr(pf_mean.box_quad[1, 0]))
    check("agg=median：quad[1][0] == 700.0",
          abs(pf_med.box_quad[1, 0] - 700.0) < 1e-9, repr(pf_med.box_quad[1, 0]))
    check("两条路径确实不同（离群点场景 995 vs 980）",
          pf_mean.box_quad[0, 0] != pf_med.box_quad[0, 0],
          f"{pf_mean.box_quad[0, 0]!r} vs {pf_med.box_quad[0, 0]!r}")
    # 托盘位姿不受 agg 影响（median 只在箱子四角上生效）
    check("agg 不影响托盘位姿（托盘一律刚体平均）",
          np.allclose(pf_mean.T_cam_pallet, pf_med.T_cam_pallet),
          repr((pf_mean.T_cam_pallet, pf_med.T_cam_pallet)))
    # 拼错静默退回 mean（F7 选 (b)：合法值写进 docstring，行为不变）
    pf_typo = average_pairs(quad_pairs, agg="meean")
    check("agg 拼错静默按 mean（有意，见 docstring）",
          np.allclose(pf_typo.box_quad, pf_mean.box_quad),
          repr(pf_typo.box_quad))

    print("== dt_s：取**最新那一对**的两侧时间差 ==")
    # `dt_s` 是给上层判"这对到底差多少"用的。用二进制精确可表示的向量
    # （1.25 - 1.0 == 0.25 精确成立）—— 本模块既有的约定，见 nearest_pair 的
    # docstring：ROS epoch 的 ulp 是 2.4e-7，别用会掉进量化误差的值。
    pf = average_pairs([(PalletObservation(T_cam_pallet=_T(0.0), stamp=1.0),
                         BoxObservation(quad=_quad(), stamp=1.0)),
                        (PalletObservation(T_cam_pallet=_T(0.0), stamp=2.0),
                         BoxObservation(quad=_quad(), stamp=2.25))])
    check("dt_s == 0.25（最新那一对的两侧差）",
          abs(pf.dt_s - 0.25) < 1e-12, repr(pf.dt_s))
    check("dt_s 的向量确实二进制精确可表示（(2.25-2.0)==0.25）",
          (2.25 - 2.0) == 0.25, repr(2.25 - 2.0))
    check("pallet_stamp / box_stamp 取最新那一对（2.0 / 2.25）",
          pf.pallet_stamp == 2.0 and pf.box_stamp == 2.25,
          repr((pf.pallet_stamp, pf.box_stamp)))
    # 单对：1.0 / 1.25 → 0.25
    pf = average_pairs([(PalletObservation(T_cam_pallet=_T(0.0), stamp=1.0),
                         BoxObservation(quad=_quad(), stamp=1.25))])
    check("单对 dt_s == 0.25", abs(pf.dt_s - 0.25) < 1e-12, repr(pf.dt_s))

    print("== 某一路停摆 → 永久 None（两种含义在返回类型上不可区分） ==")
    # 首对成功之后，任一侧停摆都会让 `_newer_than_last()` 那一侧为空 →
    # `_best_pair` 得到 `pair_none` → `resolve()` 返回 `None`，而且**永远**如此
    # （`_last_key` 只被"严格更新的观测"回写）。这是**真的**（伺服节点每 tick
    # 拿到 `None` 静默 RUNNING，黑板停在旧值，操作员看不到"检测器挂了"），
    # 所以把两条方向都写死 —— liveness 判据不在本层，由 Task 5 的节点负责。
    w7 = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=3)
    w7.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=1.00))
    w7.push_box(BoxObservation(quad=_quad(), stamp=1.00))
    check("先出一对", hasattr(w7.resolve(), 'n_pairs'))
    for ts in [1.10, 1.20, 1.30]:                 # 托盘停摆，只有箱子在推进
        w7.push_box(BoxObservation(quad=_quad(), stamp=ts))
        r = w7.resolve()
        check(f"托盘停摆（{ts}）→ None（不是 PairReject，也不是 pair_none）",
              r is None, repr(r))
    # 托盘重新出数（stamp 更新）→ 立刻恢复，证明上面的 None 是"没有更新的配对"
    w7.push_pallet(PalletObservation(T_cam_pallet=_T(7.0), stamp=1.40))
    w7.push_box(BoxObservation(quad=_quad(), stamp=1.40))
    r = w7.resolve()
    check("托盘恢复出数后立刻恢复出数（None 不是永久卡死）",
          hasattr(r, 'n_pairs'), repr(r))

    w8 = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=3)
    w8.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=1.00))
    w8.push_box(BoxObservation(quad=_quad(), stamp=1.00))
    check("先出一对（反方向）", hasattr(w8.resolve(), 'n_pairs'))
    for ts in [1.10, 1.20, 1.30]:                 # 箱子停摆，只有托盘在推进
        w8.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=ts))
        r = w8.resolve()
        check(f"箱子停摆（{ts}）→ None（反方向同样静默）", r is None, repr(r))

    # **停摆 + `stale_s>0` 仍然是 None，不是 `stale`。** `stale` 判在**去重之后**
    # （见 `resolve()` 里 `stale` 那段）：停摆侧根本没有"比水位线更新的观测"，
    # 走不到那个分支。这里 `now=5.0`、托盘停在 1.00（age=4.0s）远超 `stale_s=0.5`
    # —— 若谁把 `stale` 判到去重之前、或在停摆分支里返回 `stale`，这条立刻红。
    # 这是"延迟闸不是存活闸"的落地：检测器死了它一个 `stale` 都不该报。
    w8b = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=3, stale_s=0.5)
    w8b.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=1.00))
    w8b.push_box(BoxObservation(quad=_quad(), stamp=1.00))
    r = w8b.resolve(now=1.10)
    check("stale_s>0 且不陈旧 → 正常出", hasattr(r, 'n_pairs'), repr(r))
    for ts in [1.10, 1.20, 1.30]:                 # 托盘停摆，箱子推进；now 越来越旧
        w8b.push_box(BoxObservation(quad=_quad(), stamp=ts))
        r = w8b.resolve(now=5.00)
        check(f"停摆 + stale_s>0（{ts}）→ None（不是 stale）",
              r is None, repr(r))

    print("== window 与 pair_cache 互相独立（window 不被 pair_cache 封顶） ==")
    # `pair_cache` / `window` 都是 Task 5 的 ROS 参数，配错不会报错。这里钉住
    # **两个旋钮各自生效**：`pair_cache=5, window=8` 就是平滑 8 帧，不会只剩 5。
    # 曾经这里断言的是"n_pairs 封顶 5"—— 那钉的是构造里 `min(window, pair_cache)`
    # 那个钳位本身，而不是任何真实的语义（钳位一去、这条必红，且其余全绿，说明
    # 没有代码依赖它）。`pair_cache` 只决定"每一侧留几个观测供配对"，`_pairs`
    # 是持强引用的 deque，已配好的对不会因为观测被挤出去而消失 —— 所以它
    # 在定义上就管不着成对序列的窗长。
    w9 = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=8)
    for k in range(8):
        ts = 1.00 + 0.10 * k
        w9.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=ts))
        w9.push_box(BoxObservation(quad=_quad(), stamp=ts))
        r = w9.resolve()
    check("window=8 + pair_cache=5 → n_pairs 到 8（window 不被 pair_cache 封顶）",
          r.n_pairs == 8, repr(r.n_pairs))
    # 反向：`pair_cache` 照样管着**观测缓存**（每侧只留 5 个），两个旋钮各管各的。
    check("pair_cache=5 仍然封住每侧观测缓存（len == 5）",
          len(w9._pallets) == 5 and len(w9._boxes) == 5,
          repr((len(w9._pallets), len(w9._boxes))))

    print("== 时钟回跳 → 永久静默，只有 reset() 能救 ==")
    # 时钟**整体回跳**（bag 循环播放 / 换时间源 / 重进 initialise()）后，
    # `_last_key` 变成未来值，此后所有观测都"不更新" → resolve() 永久 None，
    # 伺服黑板停在旧值、一个错都不报。这条前提必须写死给 Task 5 的调用方看。
    w10 = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=3)
    w10.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=10.00))
    w10.push_box(BoxObservation(quad=_quad(), stamp=10.00))
    check("回跳前正常出数", hasattr(w10.resolve(), 'n_pairs'))
    for ts in [1.00, 1.10, 1.20]:                 # 时钟回跳到 1.x
        w10.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=ts))
        w10.push_box(BoxObservation(quad=_quad(), stamp=ts))
        r = w10.resolve()
        check(f"时钟回跳后（{ts}）→ 永久 None（静默，不报错）",
              r is None, repr(r))
    w10.reset()
    w10.push_pallet(PalletObservation(T_cam_pallet=_T(0.0), stamp=1.30))
    w10.push_box(BoxObservation(quad=_quad(), stamp=1.30))
    r = w10.resolve()
    check("reset() 之后恢复出数（唯一出路）", hasattr(r, 'n_pairs'), repr(r))

    print()
    if FAILS:
        print(f"失败 {len(FAILS)} 条：{FAILS}")
        return 1
    print("全部通过")
    return 0


if __name__ == '__main__':
    sys.exit(main())
