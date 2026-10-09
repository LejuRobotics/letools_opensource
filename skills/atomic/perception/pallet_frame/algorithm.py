# -*- coding: utf-8 -*-
"""托盘观测的配对判据：**纯函数、零状态、不 import ROS**。

## 配对的依据

**两个观测的「输入图像采集时刻」（`header.stamp`）是否落在同一帧上。**

一旦按图像时刻配对，**耗时差自动消失**：

    图像第 100 帧  t=3.300s
       ├─ 箱子检测：  耗时  60ms  → 3.360s 算完，stamp 填 3.300
       └─ 托盘检测：  耗时 500ms  → 3.800s 算完，stamp 填 3.300   ← 同一个数

两个检测器看的是同一帧图像，只是算完的时刻差 440ms。耗时差 440ms 还是 4s 都一样。

## 代价

**配不上就 reject，不猜、不外推、不用旧的。** 托盘检测慢的时候伺服输出率被它
卡住。这是刻意的：宁可少出数，不出错数 —— 每一个输出都必须是两个检测器看
**同一帧图像**算出来的。

## 拒绝原因（五种，互不重叠）

判定按下面的顺序**短路**执行，先命中的先返回，所以五种永不重叠：任何一次调用
最多只可能命中一种。

- `pair_threshold` —— `max_dt_s` 本身不是正有限值（NaN / inf / 0 / 负数）。
  **阈值坏是配置错误，比「没有观测」更根本，所以判在最前面。** 上层
  `PalletFrameWindow` 从 ROS 参数读 `max_dt_s`，参数没设或类型转换失败时拿到
  的正是 NaN —— ROS 里读 float 的常见坑。此时 `dt > nan` 恒为 False，任何一对
  都被当成「配得上」，整个配对门禁**静默失效且不报任何 reject**；`inf` 同理
  放行任意差。`0` 或负数也拒：它意味着「只接受完全同时刻」，那是配置写错了而
  不是「严格」—— `dt > 0` 对任何有微小差的一对都成立，结果是全部 reject，看
  起来像「功能坏了」而不是「配错了」。
- `pair_none` —— 压根没有观测（任一组为空）。
- `pair_nan`  —— 有观测，但时间戳是非有限值（NaN/inf）。`header.stamp` 直接填
  成 NaN（或 inf）在 ROS 里是常态，**坏时间戳不能当成有效观测**：
  它与任何东西的比较都是 False，放过去就是「静默配上」，还会把真正配得上的
  那一对挤掉。兄弟模块 `pallet_servo/algorithm.py` 对非有限也是显式拒绝
  （`points_not_finite` / `quad_not_finite`），这里保持同一套态度。
  （注意：`0.0` 是**有限值**，它进不了这个出口，走的是下面的
  `pair_stamp_zero`。）
- `pair_stamp_zero` —— 有观测、时间戳也有限，但选出的一对**两侧同时 ≤ 0**。
  `0.0` 是有限值，`pair_nan` 出口盖不住它，而 `rospy.Time().to_sec()` 的默认值
  **就是 0.0** —— ROS 里 `Header()` 忘了填、填 0 附近的默认值都是常态。
  仿真时间（`use_sim_time`）下
  第一帧的 stamp 合法地就是 0，所以**单侧**为 0 不能判坏；但两路观测**同时**
  停在 0 上，真实时刻更可能是「一个墙钟、一个 0」，差着整个 epoch —— 配上就是
  错的。
- `pair_dt`   —— 有观测、时间戳也有限、选出的一对也不在「两侧同时 ≤ 0」上，但
  最接近的一对差得超过 `max_dt_s`。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple, Union

import numpy as np


@dataclass
class PalletObservation:
    """一帧托盘观测。

    `T_cam_pallet` 是 4×4 的「相机系 ← 托盘系」，**不做任何校验**（手性与形状
    由 `NodePalletObs` 的闸门负责，见 spec §5.2）。这里只装数据。

    ⚠️ **`source` 在成对链路上恒为空串 —— 它不是"没填"，是"装不下"。**
    `NodePalletServo._push_pallet()` 填的是 `getattr(pose, "label", "")`，而黑板上的
    `latest_pallet` 是 `Pose6D`（**只有六个数**：x/y/z/yaw/pitch/roll，没有 `label`），
    所以那个 `getattr` **永远**取不到东西。检测器的 `source` 只出现在
    `NodePalletObs` 自己的 WARNING 文案里（`valid=false` / stamp 那两条），
    **不进黑板**。设计说 `source` 是"出了事第一件事就是看它"（颜色/深度选错层是
    已知失败模式）—— 现场要看它就去 `NodePalletObs` 的日志里看，别指望这个字段。
    """

    T_cam_pallet: np.ndarray
    stamp: float = 0.0
    source: str = ""
    valid: bool = True


@dataclass(frozen=True)
class PairReject:
    """配对失败：哪个原因、实际差多少、阈值多少。

    只返回 `None` 的话，到了上层就只剩一句"配不上"，无从排查 —— 所以 `detail`
    里必须写**实际值和阈值**。与 `pallet_servo.algorithm.Reject` 同构。
    """

    code: str
    detail: str

    def __str__(self) -> str:
        return f"{self.code}: {self.detail}"


def nearest_pair(pallet_stamps: Sequence[float], box_stamps: Sequence[float],
                 max_dt_s: float) -> Union[Tuple[int, int], PairReject]:
    """在两组时间戳里挑**时间差最小的一对**，返回 `(托盘下标, 箱子下标)`。

    五种拒绝（按判定顺序，互不重叠）：`max_dt_s` 不是正有限值 →
    `pair_threshold`；任一组为空 → `pair_none`；时间戳里有非有限值 →
    `pair_nan`；选中的一对两侧同时 ≤ 0 → `pair_stamp_zero`；最小差超过
    `max_dt_s` → `pair_dt`。五者的 `detail` 都带实际值（阈值坏时带阈值本身）。

    **`max_dt_s` 也校验**（`pair_threshold`）：只过滤两侧时间戳是不够的 ——
    `dt > nan` 恒为 False，阈值是 NaN 时**任何**一对都被当成配上，门禁静默失效
    且一个 reject 都不报。所以判在 `pair_none` 之前：阈值坏是配置错误，比「没
    有观测」更根本，该最先说。`<= 0` 一并拒（配置写错，不是「严格」）。

    **两侧同时 ≤ 0 也拒**（`pair_stamp_zero`）：`0.0` 是有限值，`pair_nan` 盖不住
    它，而 `rospy.Time().to_sec()` 默认就是 0.0。仿真时间下第一帧的 stamp 合法
    地就是 0，所以**单侧**为 0 不能判坏；但两路同时停在 0 上，真实时刻更可能是
    「一个墙钟、一个 0」，差着整个 epoch。见模块 docstring 里那段说明。

    **边界是闭区间，且不加任何容差**：`dt == max_dt_s` 算配上，判定就是
    `dt > float(max_dt_s)`。为什么不加 `eps`：ROS epoch 时间戳（约 1.7e9）的双
    精度 ulp 是 **2.4e-7 秒**，`dt` 只能落在 2.4e-7 的网格上、自带不超过一个
    ulp（≈2.4e-7）的量化误差。比它小的容差（如 1e-9）对任何一次真实判定都是
    空操作，只是把边界测试哄绿；比它大的容差则会让「正好差一帧」这个边界变得
    不可解释 —— 一对真值差超过阈值的观测会因为落在哪个网格格里而被放过。所以
    这里用精确比较，边界只对**二进制精确可表示**的向量成立（如 `1.25 - 1.0
    == 0.25`），测试也照这个来构造。至于那 2.4e-7 的量化误差本身：它比 30fps 的
    一帧（33ms）小 5 个数量级，对「同一帧 / 差一帧」的语义毫无影响。

    **并列最小 dt 时先遇到者胜**：遍历是 `for i in 托盘: for j in 箱子:`，更新
    条件是严格小于（`dt < best[0]`），所以并列时保留的是字典序最小的
    `(i, j)`。两路时间戳各自单调时，并列只可能出现在两条序列等距错开的情形，
    取哪一个都不改变「同一帧」的语义；定成规则只是为了结果可复现。

    复杂度 O(n·m)：上层（`PalletFrameWindow`）会把两边截到 `pair_cache` 个
    （默认 5），5×5=25 次比较，在 10 Hz 的伺服循环里可以忽略。**本函数自己不
    截断**，O(n·m) 由调用方负责。
    """
    # 阈值本身先校验，而且判在 `pair_none` 之前：阈值坏是配置错误，比「没有观测」
    # 更根本。只过滤时间戳是不够的 —— `dt > nan` 恒为 False，阈值是 NaN 时任何
    # 一对都被当成配上，整个门禁静默失效；inf 则放行任意差。<= 0 同样是配置写错
    # （「只接受完全同时刻」不是「严格」），会让所有有微小差的一对全被拒。
    v = float(max_dt_s)
    if not math.isfinite(v) or v <= 0.0:
        if math.isnan(v):
            why = ("NaN 常见于上层从 ROS 参数读它、而参数没设或类型转换失败")
        elif v == float("inf"):
            why = ("inf 会放行任意时间差 —— dt > inf 恒为 False，"
                   "等于把这道门禁整个关掉")
        else:
            why = (f"{v!r} 会让**任何**有微小时间差的一对全被拒"
                   f"（dt > {v!r} 对几乎所有观测都成立），等于把功能关掉。"
                   f"0 或负数通常是「想临时关掉它」时手滑写下的")
        return PairReject(
            "pair_threshold",
            f"配不成对：阈值 max_dt_s={v!r} 不是正有限值"
            f"（要求 0 < max_dt_s < inf）；{why}")

    if not pallet_stamps or not box_stamps:
        return PairReject(
            "pair_none",
            f"配不成对：托盘 {len(pallet_stamps)} 个观测、箱子 "
            f"{len(box_stamps)} 个观测（两边都要有）")

    # 非有限的时间戳先挑出来。不能只靠后面的 `dt > max` 兜底：NaN 与任何数比较
    # 都是 False，`dt < best[0]` 换不掉它、`dt > max` 也拦不住它，于是 NaN 会
    # 一路走到成功返回 —— 而且它一旦先成为 best，真正精确匹配的那一对就被丢了。
    p_bad = [k for k, ts in enumerate(pallet_stamps) if not math.isfinite(ts)]
    b_bad = [k for k, ts in enumerate(box_stamps) if not math.isfinite(ts)]
    if p_bad or b_bad:
        return PairReject(
            "pair_nan",
            f"配不成对：时间戳里有非有限值（NaN/inf）—— 托盘 "
            f"{len(p_bad)}/{len(pallet_stamps)} 个、箱子 "
            f"{len(b_bad)}/{len(box_stamps)} 个"
            f"（非有限值的下标：托盘 {p_bad}、箱子 {b_bad}）；"
            f"原始列表：托盘 {list(pallet_stamps)}、箱子 {list(box_stamps)}"
            f" —— header.stamp 忘了填/填成 NaN 就是这样，"
            f"坏时间戳不能当成有效观测")

    best = None                       # (dt, i, j)
    for i, ts in enumerate(pallet_stamps):
        for j, bs in enumerate(box_stamps):
            dt = abs(float(ts) - float(bs))
            if best is None or dt < best[0]:
                best = (dt, i, j)

    dt, i, j = best

    # 两侧同时 <= 0：`0.0` 是有限值，上面的 pair_nan 出口盖不住它，而
    # `rospy.Time().to_sec()` 的默认值**就是 0.0** —— ROS 里 `Header()` 忘了填
    # 是常态。仿真时间（use_sim_time）下第一帧的 stamp 合法地就是 0，所以**单侧**
    # 为 0 不能判坏；但两路观测同时都停在 0 上，真实时刻更可能是「一个墙钟、
    # 一个 0」，差着整个 epoch —— 配上就是错的。放在 pair_dt 之前判：这种配对
    # 的 dt 往往是 0，走 dt 判据会一路配上。
    if float(pallet_stamps[i]) <= 0.0 and float(box_stamps[j]) <= 0.0:
        return PairReject(
            "pair_stamp_zero",
            f"配不成对：选中的一对两侧时间戳同时 <= 0（托盘 stamp="
            f"{float(pallet_stamps[i]):.9f}、箱子 stamp="
            f"{float(box_stamps[j]):.9f}）—— 0.0 是 rospy.Time() 的默认值，"
            f"更可能是有一路忘了填 header；仿真时间下单侧为 0 合法，"
            f"但两路同时为 0 不能当成同一帧")

    if dt > float(max_dt_s):
        return PairReject(
            "pair_dt",
            f"配不成对：最接近的一对时间戳差 {dt:.9f}s，超过阈值 "
            f"{float(max_dt_s):g}s（托盘 stamp={float(pallet_stamps[i]):.9f}、"
            f"箱子 stamp={float(box_stamps[j]):.9f}）"
            f" —— 检查检测器的 header.stamp 是不是填成了处理完成时刻")
    return i, j


@dataclass
class PairedFrame:
    """一对（或多对平均后）的托盘位姿 + 箱子四角，**两者来自同一批配对**。

    `n_pairs` 是参与平均的对数（1 = 没平滑）。

    `spread_mm` / `spread_deg` 是**窗内各帧到平均值的最大偏离**，用来判"这一窗
    稳不稳"：

        spread_mm  = max_i ‖ t_i − t_avg ‖₂ × 1000     （毫米）
        spread_deg = max_i 转角( inv(T_avg) · T_i )     （度）

    ⚠️ **这两个字段目前没有消费者**（措辞更正：早先这里写的是"Task 6 的字段表里
    有它们，render / 自检层拿它做判断"，**那句话是错的** —— `render_servo_overlay`
    的入参是 `ServoError`，而 `ServoError` 里**没有** spread 字段；`NodePalletServo`
    也一个都没读，它只用 `T_cam_pallet` / `box_quad` / `n_pairs` / `box_stamp`）。
    留着是给**后续**判"这一窗稳不稳"用的，接上要动 render 的入参（那是接口变更，
    不在当初那一轮的范围）。

    两个口径必须记牢，否则会把它当别的量用：

    * **参照物是"平均值"，不是"别的帧"。** 两帧差 30mm 时 `spread_mm == 15`
      （各自离均值 15mm），不是 30。它是"这一窗有多散"，不是"两两之间差多远"。
    * **它是 max 不是 rms / 标准差**：一个离群点就会把它顶上去 —— 这正是想要的
      （窗里混进一帧坏观测时要能看出来）。
    * 只有一帧时两个 spread 都是 0（离均值 0）；`n_pairs == 1` 时"稳不稳"无从
      判断，别把 0 读成"很稳"。

    `pallet_stamp` / `box_stamp` 取窗里**最新那一对**的两侧 stamp（不是平均、
    也不是最早那一对）。`dt_s = abs(pallet_stamp - box_stamp)`，即**最新那一对**
    两侧的时间差 —— 上层拿它判"这对到底差多少"，它不是窗内各帧 dt 的平均。

    ⚠️ **`source` 在成对链路上恒为空串**（`average_pairs` 从 `PalletObservation`
    抄来，而那边就取不到值，见它的 docstring）。检测器的 `source` **只进
    `NodePalletObs` 的节点日志，不进黑板**。要它就得先让托盘生产者在黑板上留一个
    位子 —— 那是接口变更，不是"填一下就有了"。
    """

    T_cam_pallet: np.ndarray      # (4, 4) 相机系 ← 托盘系
    box_quad: np.ndarray          # (4, 2) 像素，顺序 [右下, 左下, 左上, 右上]
    n_pairs: int = 1
    spread_mm: float = 0.0
    spread_deg: float = 0.0
    pallet_stamp: float = 0.0
    box_stamp: float = 0.0
    dt_s: float = 0.0
    source: str = ""


def average_pairs(pairs: Sequence[Tuple[PalletObservation, "object"]],
                  agg: str = "mean") -> PairedFrame:
    """把一串配对平均成一个 `PairedFrame`。

    **托盘位姿用刚体平均，箱子四角逐点平均。**

    ⚠️ 托盘**不能**用逐元素线性平均：旋转矩阵线性平均之后不再是旋转矩阵
    （行列式会漂离 ±1），而 `[e1|e2|normal]` 被当旋转矩阵用，漂了之后参考边
    投影出来会歪 —— 而且是**静默**歪。所以复用 `pallet_pose.algorithm` 的
    `average_rigid_transforms`（SVD 正交化平均，且**保留输入的 det 符号**，
    不会把镜面偷偷掰成正的）。

    `agg`：**合法值只有 `"mean"` 和 `"median"` 两个**（Task 5 的 ROS 参数
    `agg` 直接透传到这里）。mean / median **只作用在箱子四角上**；托盘位姿一律
    走刚体平均，`agg` 对它无效 —— 旋转矩阵不能逐元素取中位数。

    ⚠️ **拼错（`"meean"` / `"Mean"` / 空串 …）静默退回 `mean`，不报错、不告警。**
    这是**有意**的，不是漏写，理由三条：

    * `algorithm.py` 是**纯函数、零状态、不 log**（架构约束）—— 没有"告警"这个
      动作可用。要"显式拒绝"就得返回 `PairReject`，而那是"**配不成对**"的语义，
      用它表示"聚合方式不认识"是把两种故障混成一个出口，比静默更糟。
    * 与 `nearest_pair` 对 `max_dt_s` 的严格态度**不冲突**：阈值坏会让配对门禁
      **静默失效**（`dt > nan` 恒 False，任何一对都放行），所以必须显式拒；而
      `agg` 拼错只是**从 mean 换成 mean**，结果仍是这个窗里各帧的一个合法平均，
      不会放出错数。
    * 合法值就在**调用方（Task 5 的 ROS 参数）**那一侧可查：`window.py` 的
      `PalletFrameWindow` docstring 里列了它，Task 5 的 `agg` 参数处要注明
      "只认 mean / median，拼错静默按 mean"。

    想让拼错可见，**该在 Task 5 的节点侧做**（读参数时白名单校验一次并告警），
    不在这个纯函数里做 —— 那才是不破坏本模块"零状态、不 log"的地方。

    `spread_*` 与 `dt_s` 的口径见 `PairedFrame` 的 docstring（到**均值**的最远
    距离、取**最新那一对**的 stamp）。
    """
    # 延迟 import：`pallet_pose.algorithm` 会拉进 cv2，而本模块的配对判据
    # （`nearest_pair`）不该为了一个平均去付这个代价 —— 测试里只跑配对时
    # 用不上 cv2。
    from skills.atomic.perception.pallet_pose.algorithm import (
        average_rigid_transforms,
        rotation_angle_deg,
    )

    Ts = [p.T_cam_pallet for p, _ in pairs]
    T_avg = average_rigid_transforms(Ts)

    quads = np.asarray([np.asarray(b.quad, np.float64) for _, b in pairs],
                       np.float64)                    # (n, 4, 2)
    if agg == "median":
        quad = np.median(quads, axis=0)
    else:
        quad = quads.mean(axis=0)

    # spread：窗内各帧到平均值的**最远**距离（位置 mm、朝向 deg）
    spread_mm = 0.0
    spread_deg = 0.0
    for T in Ts:
        d = T[:3, 3] - T_avg[:3, 3]
        spread_mm = max(spread_mm, float(np.linalg.norm(d)) * 1000.0)
        # 相对旋转的转角：`inv(T_avg) @ T` 的旋转块
        rel = np.linalg.inv(T_avg) @ T
        spread_deg = max(spread_deg, rotation_angle_deg(rel[:3, :3]))

    pallet_stamp = float(pairs[-1][0].stamp)
    box_stamp = float(pairs[-1][1].stamp)
    return PairedFrame(
        T_cam_pallet=T_avg,
        box_quad=quad,
        n_pairs=len(pairs),
        spread_mm=spread_mm,
        spread_deg=spread_deg,
        pallet_stamp=pallet_stamp,
        box_stamp=box_stamp,
        dt_s=abs(pallet_stamp - box_stamp),
        source=str(pairs[-1][0].source),
    )
