# 托盘观测的配对与平滑

把两路观测（**托盘位姿** + **箱子四角**）按**输入图像的采集时刻**配成对，
再在成对序列上做窗，交给 `pallet_servo` 算三个误差量。

**为什么要配对**：伺服算的是「箱子相对托盘」的位置，而托盘与箱子的检测耗时
可能差一个量级（箱子实测 ~60ms）。两者必须来自**同一帧图像**才能相减 ——
否则误差里混着相机运动，**而三个数照样算得出来**（静默错）。

```text
/pallet/detection ──► NodePalletObs ──┐
                        latest_pallet │
                                      ├─► NodePalletServo ──► latest_servo_error
/box/detection ────► NodeBoxObs ──────┤      （配对 + 平滑 + 出图）
                        latest_box_obs┘
```

| 文件 | 作用 |
|---|---|
| `algorithm.py` | **纯函数**：配对判据 `nearest_pair`、成对序列平均 `average_pairs`。零框架依赖、零 ROS |
| `window.py` | `PalletFrameWindow`：两路缓存 + 配对 + 成对序列（**有状态**，也不 import ROS） |
| `tests/test_pairing.py` | 脚本式自检（退出码 0/1，**不进 CI**）|

> ⚠️ 本模块**不是** `box_frame/window.py` 的扩展：那个管「一路观测 + 多帧平均」，
> 本模块管「两路观测 + 配对」，是两件事。

# 1. 接口

## 配对判据（纯函数）

```python
from skills.atomic.perception.pallet_frame import nearest_pair, PairReject

found = nearest_pair(pallet_stamps, box_stamps, max_dt_s=0.05)
#   -> (i, j)        配上对，i 是托盘下标、j 是箱子下标
#   -> PairReject    code ∈ 下表五种（外加 window 层的 "stale"）
```

**边界是闭区间**：`dt == max_dt_s` 算配上，判定就是 `dt > float(max_dt_s)`，
**不加任何容差**。理由：ROS epoch 时间戳（约 1.7e9）的双精度 ulp 是 **2.4e-7 秒**，
比它小的容差（如 1e-9）对任何一次真实判定都是空操作、只是把边界测试哄绿；
比它大的容差会让「正好差一帧」这个边界变得不可解释。复杂度 O(n·m)，两边各只留
5 个，25 次比较，在 10 Hz 的伺服循环里可以忽略。

判据按下面的顺序**短路**执行，先命中的先返回，所以五种**互不重叠**：

| `code` | 含义 |
|---|---|
| `pair_threshold` | `max_dt_s` 不是正有限值（判在 `pair_none` **之前**）|
| `pair_none` | 有一边压根没有观测 |
| `pair_nan` | 时间戳里有非有限值（NaN/inf）|
| `pair_stamp_zero` | 选中的一对**两侧同时** ≤ 0 |
| `pair_dt` | 最接近的一对时间戳差超阈值（`detail` 带**实际值与阈值**）|

⚠️ **`pair_threshold` 与 `pair_nan` 是两道防「门禁静默失效」的闸，不是多余的**：

- **`dt > nan` 恒为 False** —— 阈值是 NaN 时**任何一对都被当成配上**，门禁
  静默失效且一个 reject 都不报。NaN 常见于上层从 ROS 参数读 `max_dt_s` 而参数
  没设 / 类型转换失败；`inf` 同理放行任意差。
- **NaN 与任何数比较都是 False** —— 它换不掉 best、也拦不住，会一路走到成功返回，
  **而且一旦先成为 best，真正精确匹配的那一对就被丢了**。

⚠️ **`pair_stamp_zero` 只在两侧同时 ≤ 0 时拒**：`0.0` 是有限值，`pair_nan` 盖不住它，
而 `rospy.Time().to_sec()` 的默认值**就是 0.0**。仿真时间（`use_sim_time`）下第一帧的
stamp 合法地就是 0，所以**单侧**为 0 不能判坏；但两路观测**同时**停在 0 上，
真实时刻更可能是「一个墙钟、一个 0」，差着整个 epoch —— 配上就是错的。

## 时间窗

```python
from skills.atomic.perception.pallet_frame import (
    PalletFrameWindow, PalletObservation)

win = PalletFrameWindow(max_dt_s=0.05, pair_cache=5, window=5,
                        agg="mean", stale_s=0.0)
win.push_pallet(PalletObservation(T_cam_pallet=T, stamp=3.300))
win.push_box(box_observation)                    # BoxObservation，带 stamp
out = win.resolve(now=time.time())
#   -> PairedFrame   成对成功（含平滑结果）
#   -> PairReject    配不上，code 还可能是 "stale"
#   -> None          **没有新的配对**（见注意事项 ⑥ —— 这个返回值有两种含义）
```

`PairedFrame` 的字段：

| 字段 | 说明 |
|---|---|
| `T_cam_pallet` | 4×4，相机系 ← 托盘系，**窗上刚体平均后**的 |
| `box_quad` | 4×2 像素，**逐点平均后**的四角，顺序同 `BoxObservation.quad` |
| `n_pairs` | 参与平均的对数（1 = 没平滑）|
| `spread_mm` / `spread_deg` | 窗内各帧到平均值的**最远**距离（口径见注意事项 ⑩）|
| `pallet_stamp` / `box_stamp` / `dt_s` | 这一对的出处（取窗里**最新那一对**）|

状态与生命周期：

```python
win.n_pairs      # 窗里现在有几对（上限是构造时的 window）
win.reset()      # 清空两路缓存 + 成对序列 + 归零水位线（时钟回跳的唯一出路）
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_dt_s` | `0.05` | 配对允许的最大时间戳差。30fps 下一帧 = 33ms |
| `pair_cache` | `5` | **每侧**留多少个观测供配对。**不是窗长**。⚠️ **回溯窗口 ≈ `pair_cache` × 伺服 tick 周期**（push 是按 tick 发生的，不是检测器一出帧就推）—— 10 Hz 下 `5` ≈ **500 ms**，**必须大于托盘检测器的端到端延迟** |
| `window` | `5` | **成对序列**上取几对做平均（1 = 不平滑）—— **与 `pair_cache` 独立** |
| `agg` | `mean` | `mean` / `median`。**`median` 只在箱子四角上生效**，托盘一律走刚体平均 |
| `stale_s` | `0.0` | **延迟闸不是存活闸**（见注意事项 ⑨）：配对后这一对有多旧 |

# 2. 快速测试

```bash
cd <仓库根>
python3 skills/atomic/perception/pallet_frame/tests/test_pairing.py   # 0 通过 / 1 失败
```

**不需要 ROS、不需要相机、不需要任何数据** —— 时间戳自己造。它跑四组用例：
五种配对判据（含闭区间边界与坏阈值）、窗的平滑、`stale` 与 `None` 的两种含义、
时钟回跳与 `reset()`。

# 3. 注意事项

**① 配对依据是 `header.stamp`（输入图像的采集时刻），不是处理完成时刻。**
这条是整个模块成立的前提：

```text
图像第 100 帧  t=3.300s
   ├─ 箱子检测：  耗时  60ms  → 3.360s 算完，stamp 填 3.300
   └─ 托盘检测：  耗时 500ms  → 3.800s 算完，stamp 填 3.300   ← 同一个数
```

按图像时刻配对，**耗时差自动消失**。按「算完的时刻」配对，耗时的差会被当成
场景的时间差 —— 要么永远配不上，要么配上一对**其实是不同帧的图像**。
填错的表现：伺服频繁报 `pair_dt`（`detail` 里带实际差值）。

**② 配不上就 reject，不猜、不外推、不用旧的。** 托盘检测慢的时候伺服输出率被它
卡住。这是刻意的：**宁可少出数，不出错数** —— 每一个输出都必须是两个检测器看
**同一帧图像**算出来的。

**③ 托盘位姿必须用刚体平均，不能用逐元素线性平均。** 旋转矩阵线性平均之后不再是
旋转矩阵（行列式会漂离 ±1），而 `[e1|e2|normal]` 被当旋转矩阵用，漂了之后参考边
投影出来会歪 —— 而且是**静默**歪。这里复用 `pallet_pose.algorithm` 的
`average_rigid_transforms`（SVD 正交化平均，且**保留输入的 det 符号**，不会把
镜面偷偷掰成正的）。

**④ 平滑放在这里、不放在检测器里。** 「托盘与箱子同窗长同聚合」如果靠两边各自
实现，就是两份独立的滞后，谁改一边就静默失配。而且因为**每一对都是同一帧图像**，
配对之后再做窗，窗内两边的相位**天然对齐** —— 同一个窗、同一批对，没有第二份
实现可以走偏。

**⑤ `pair_cache` 与 `window` 是两个独立的旋钮，谁都不封顶谁。**
`pair_cache` = **每侧**留几个观测供配对；`window` = **成对序列**上取几对做平均。
`pair_cache=5, window=8` 就是老老实实平滑 8 帧，**不必**连带把 `pair_cache` 开大 ——
开大它只会加深配对回溯、让坏观测多留几个 tick，与平滑几帧无关。（早期文档写过
「`window` 的上限由 `pair_cache` 卡住」，那个钳位已经删掉了。）

**⑥ `resolve()` 返回 `None` 有**两种**含义，且不可区分。**
调用方每 tick 都调它，不加这道闸就是「同一对反复出数、版本号一直涨、下游白算」。
但返回 `None` 的还有第二种情形：

1. **同一对已经报过了**（正常去重）；
2. **某一侧没有新观测**（托盘检测停了 / 箱子检测停了 / 那一路挂了）。

情形 2 是**真的**：首对成功之后任一侧停摆，`None` 会**永远**返回下去。
⚠️ **两者在返回类型上不可区分** —— 所以 **liveness 判据不在窗这一层**：
`NodePalletServo` 拿到 `None` 只会静默 `RUNNING`，**黑板停在旧值，操作员看不到
「检测器挂了」**。存活告警由节点层的 `live_timeout_s` 负责（见
`pallet_servo/README.md`）；`stale_s` 顶不上这个用（注意事项 ⑨）。

**⑦ 时钟整体回跳（bag 循环播放 / 换时间源 / 重进 `initialise()`）会让窗永久静默
—— 必须靠 `reset()`。** `_last_key` 是「已经消费掉的观测」的水位线：回跳之后新观测
的 stamp 全都小于它，`resolve()` 于是**永久返回 `None`**，伺服黑板停在回跳前那一帧
的值上，**没有任何报错**。本层只看得见 stamp，分不清「时间倒退了」与「检测器没出数」
（两者都长成 `None`），所以**发现回跳是调用方的责任**：
`NodePalletServo.initialise()` / 重进流程里已经接了 `reset()`。

**⑧ `median` 只作用在箱子四角上。** 对旋转矩阵不能逐元素取中位数 —— 托盘位姿
一律走刚体平均，与 `agg` 无关。另外 `agg` **拼错会静默按 `mean`**（纯函数层不 log）。
可见性归节点层：`NodePalletServo` 读参数时做白名单校验并记 WARNING。

**⑨ `stale_s` 是延迟闸，不是存活闸，两个别混着写。**
它管的是「**配对之后**这一对有多旧」（`now - pallet.stamp`，判在去重**之后**），
**不是**「检测器还活着吗」：

- 它判在去重之后 —— 已经报过的那一对在上一层就被挡掉了，根本走不到它，所以
  「某路停摆」永远不会变成 `stale`，而是变成 `None`（注意事项 ⑥）；
- `max_dt_s` 判在它**之前**：`max_dt_s=0.05` + 一个 500ms 的托盘延迟会**先被判
  `pair_dt`**，`stale` 根本轮不到。

所以配在「慢但健康」的检测器上时它会误报，而「检测器死了」它一个都不报。
要用它，`now` 必须是**托盘 stamp 的同一个时钟**，且 `stale_s` 要大于该检测器的
正常端到端延迟。**存活判据是 `live_timeout_s`。**

**⑩ `spread_mm` / `spread_deg` 的口径有三处容易读错：**

- **参照物是「平均值」，不是「别的帧」**：两帧差 30mm 时 `spread_mm == 15`
  （各自离均值 15mm），**不是 30**；
- **它是 `max` 不是 rms / 标准差**：一个离群点就会把它顶上去 —— 这正是想要的；
- **只有一帧时两个 spread 都是 0**（离均值 0）；`n_pairs == 1` 时「稳不稳」无从
  判断，**别把 0 读成「很稳」**。

`dt_s` 同理：它是**最新那一对**两侧的时间差，**不是**窗内各帧 dt 的平均。

**⑪ `pallet_frame: "base_link"` 模式（AprilTag 老路径）绕过配对。**
配对存在的理由是「**两个视觉检测器**的耗时差一个量级，必须按图像时刻对齐」。
老路径的生产者 `node_pallet_pose.py` **全文没有任何 `stamp` 写入** → 让配对跑起来
它**只会产出空**（箱子的 stamp 是图像采集时刻，比 tick 时刻早一个检测耗时，
`dt ≈ 60ms > max_dt_s = 50ms` → `pair_dt`）。所以 **`base_link` 走改动前的版本门禁
直算路径，完全不碰配对**。⚠️ 改这块之前先读这条 —— 以为「两种模式都走配对」会把
老路径改坏，而且**没有任何报错**。

# 4. 相关文档

- 上游：`orchestration/nodes/node_pallet_obs.py`（托盘，`/pallet/detection`）、
  `node_box_obs.py`（箱子，`/box/detection`）
- 下游：`orchestration/nodes/node_pallet_servo.py`、
  `skills/atomic/perception/pallet_servo/`（消费方，含参数表与叠加图）
- 消息契约：`infrastructure/.../ros_vision/pallet_detection_msgs/`
