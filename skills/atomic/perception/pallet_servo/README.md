# 托盘伺服误差（图像空间）

读「托盘位姿 + 箱子框」，算出**三个误差数**（两条带符号的像素垂距 + 一个角度差），
给下游控制器把箱子伺服到木托盘上的指定位置。

**只算误差，不做控制律。** 控制律、抓取流程都不在这里。

- **托盘位姿从哪来**：`orchestration/nodes/node_pallet_obs.py` 的 `NodePalletObs`
  —— 订阅视觉托盘检测话题（默认 `/pallet/detection`）并写 `latest_pallet` +
  `latest_pallet_version` + `latest_pallet_stamp`
- **箱子观测从哪来**：`orchestration/nodes/node_box_obs.py` 的 `NodeBoxObs`
  —— 订阅箱子检测话题（默认 `/box/detection`）并写 `latest_box_obs`，
  与本模块**只通过黑板键相连**，伺服节点一行都不用改
- **两路怎么对齐**：按**输入图像的采集时刻**配对 + 平滑，见 §1「配对」与
  `skills/atomic/perception/pallet_frame/README.md`
- **三个数发到哪** → `/pallet_servo/dis`（§1「对外接口是话题，不是黑板」）
- **现在能上真机吗** → §3 第 1 条（还剩 3 条）
- **踩过的坑、评审记录** → `NOTES.md`（本文档只管接口和怎么跑）

---

# 1. 接口

## 输入

### `latest_pallet` + `latest_pallet_version`

类型 **`core.domain.pose.Pose6D`**（`x, y, z, yaw, pitch, roll`）。

| 字段 | 单位 | 说明 |
|------|------|------|
| `x` `y` `z` | **米** | 托盘位置 |
| `yaw` `pitch` `roll` | **弧度** | 托盘姿态 |

**按哪个坐标系解释由 `pallet_frame` 参数定**，默认 `"camera"`：

| 取值 | 谁写的 | 节点做什么 |
|---|---|---|
| `"camera"`（**默认**）| 视觉托盘检测器（`NodePalletObs`）| `T_cam_base` 是**声明过的**单位阵，**不查 TF** |
| `"base_link"` | AprilTag 那条老路（`NodePalletPose`）| 解析 `T_cam_base` 换成相机系（TF → 参数 → 单位阵，**回退是静默的**）|

⚠️ **配错会让参考边整体偏掉，而三个数照样算得出来。** 认不出来的值退回
`"camera"` 并记 WARNING（退回的方向是安全的：那条路不查 TF、没有静默回退路径）。

⚠️ **`"base_link"` 模式绕过配对**（配对产出的对是空的，见 §1 的配对小节）。
改这块之前先读 `skills/atomic/perception/pallet_frame/README.md` 注意事项 ⑪。

> 配对要用的**图像采集时刻**在另一个键上：`latest_pallet_stamp`（没有它配对会
> 退回节点 tick 时刻，两个检测器的耗时差又回来了）。`NodePalletObs` 与
> `latest_pallet` 一起写它。

| 生产者 | 谁 |
|--------|-----|
| 真机（视觉） | **`NodePalletObs`**（`orchestration/nodes/node_pallet_obs.py`）—— 订阅 `/pallet/detection` |
| 真机（老路径） | `NodePalletPose`（`skills/atomic/perception/pallet_pose/`）|
| 离线 | `NodeInjectServoInput`（§2）|

### `latest_box_obs` + `latest_box_obs_version`

类型 **`skills.atomic.perception.pallet_servo.algorithm.BoxObservation`**。

| 字段 | 必填 | 说明 |
|------|------|------|
| `u1` `v1` `u2` `v2` | ✅ | 轴对齐框的两个角，**像素**。顺序无所谓，算法内部排序（`algorithm.py:312` 有 `sorted()`）|
| `quad` | ❌ | 4 个角点 `[[u,v]×4]`，**顺序是契约**：`[右下, 左下, 左上, 右上]` |
| `label` | ❌ | 默认 `""` |
| `confidence` | ❌ | 默认 `1.0` |
| `stamp` | ❌ | 默认 `0.0` |

**取边优先用 `quad`，没有才退到 AABB。** 两条路径给出同一个数据结构，所以加了
「框内边缘拟合」模块之后，**本模块一行都不用改**。

> 表里的 ✅ 是**语义上必须有**，不是"不填会报配置错误"：`BoxObservation` 的
> `u1/v1/u2/v2` 在代码里都带默认值 `0.0`，**漏填不报错**，真正的表现是运行期
> **每帧 `box_degenerate` 被拒**。只有离线模拟源那一侧会在启动时强制要求这四个字段。

| 生产者 | 谁 |
|--------|-----|
| 真机 | **`NodeBoxObs`**（`orchestration/nodes/node_box_obs.py`）—— 订阅箱子检测话题 |
| 离线 | `NodeInjectServoInput`（§2）|

### 配对：托盘与箱子必须来自同一帧图像

**两个观测的 `stamp` 是「输入图像的采集时刻」，配对按它做**（容差 `max_dt_s`，
默认 0.05s）。两个检测器的耗时差因此**不是问题** —— 它们看的是同一帧图像，
只是算完的时刻差了几百毫秒。

**配不上就 reject，不猜、不外推、不用旧的**（`pair_none` / `pair_dt` / `pair_nan` /
`pair_stamp_zero` / `pair_threshold` / `stale`）。托盘检测慢的时候输出率被它卡住 ——
这是刻意的：宁可少出数，不出错数。

配对之后**只有一个窗**：托盘位姿与箱子四角来自**同一批配对**，两边相位天然对齐，
不存在"两边各自开窗、谁改一边就静默失配"。细节（五种 reject 的含义、`None` 的两种
含义、`spread_*` 的口径）见 `skills/atomic/perception/pallet_frame/README.md`。

## 输出

### 对外接口是话题，不是黑板

**行为树之外的控制器请订阅 `/pallet_servo/dis`**（`pallet_servo_msgs/PalletServoError`）。

| | 黑板 `latest_servo_error` | 话题 `/pallet_servo/dis` |
|---|---|---|
| 谁能看见 | **只有行为树那个进程**（py_trees 的 `Blackboard` 是进程内单例）| 任何人（ROS 话题）|
| 用途 | **树内**节点之间的实时值 | **对外**的承诺接口 |
| 装什么 | 完整 `ServoError`（含参考边端点、垂足、内法向等诊断量）| 三个被控量 + `valid` + `reject` |

⚠️ **别拿黑板当控制器的接口。** 机器人上的伺服环路 / MPC **不在树里**，它读不到
黑板 —— 不是"不方便"，是**根本看不见**。两者同源同帧，但**只有话题是承诺过的接口**。

字段、`valid` 的语义、`header.stamp` 的口径、心跳周期，全部写在
`infrastructure/ros_packages/src/ros_vision/pallet_servo_msgs/msg/PalletServoError.msg`
的注释里 —— **那里是契约，这里是索引**。

消费方的标准动作三步：判 `valid` → 有效就拿三个量伺服 → 无效就**不要动**、看
`reject` 查明原因。

### `latest_servo_error` + `latest_servo_error_version`（**树内**，见上一节）

类型 **`skills.atomic.perception.pallet_servo.algorithm.ServoError`**。

**三个被控量**

| 字段 | 含义 |
|------|------|
| `e_bottom_px` | 箱子**底边**中点到托盘**底边**（图像投影）的带符号垂距，像素 |
| `e_right_px` | 箱子**右边**中点到托盘**右边**（图像投影）的带符号垂距，像素 |
| `theta_rad` | 箱子底边与托盘底边的角度差，带符号，弧度，正常 ±45° 以内 |

**符号约定：`e < 0` ⇔ 箱子这条边落在托盘参考边的内侧**（2026-09-30 起；原来是「内侧为正」，操作员裁定翻成「箱子在里面输出负数」）。

内法向 `n` 由**托盘自己的几何**定死（候选 `±(−d_v, d_u)`，取与「台面中心的投影 −
参考边中点」点积为正的那个），所以**不依赖托盘在画面里的朝向**：

```
e = (箱子边的中点 − 它在参考边所在直线上的垂足) · n
```

垂距取**中点**而不是"线到线"：两线不平行时线到线是一个区间，而伺服推的就是这个点。
垂足落在**参考线段之外**时置 `foot_outside` warn，**仍然输出**（箱子已偏出这条边的
范围，但垂距本身还是有效信息）。

`theta` 的定号：`line_angle = atan2(Δv, Δu)`、**`theta = 托盘底边角 − 箱子底边角`**。
图像 `u` 向右、`v` 向下，所以**正方向在屏幕上看起来是顺时针** ——
**箱子在画面里顺时针转 → `theta` 为正**；折进 `(−90°, 90°]`。

> ⚠️ **极性 2026-09-29 翻过一次。** 原定义是 `箱子 − 托盘`，操作员现场实测"角度是反的"，
> 改成现在这个。**改视觉层而不是让控制器取负**：极性是符号约定，两处一起改才一致
> —— 下面的第 11 条原本写"要反转就在控制器取负"，那条对本次改动作废。
>
> ⚠️ **没有测试钉住这个符号**（收敛扫描只扫 `theta = 0`），改反了测试照样全绿，验收
> 只能靠叠加图上眼看。

`|theta| > 45°` **只置 warn、不拒绝输出**（`theta` 超出正常范围本身不是"算不出来"）。

**诊断量**（出对比图、真机排查用）：`ref_bottom_px` / `ref_right_px`（两条托盘参考边
的端点像素）、`box_bottom_px` / `box_right_px`（配到的箱子边端点）、`pallet_center_px`、
`bottom_foot_px` / `right_foot_px`（垂足）、`bottom_inward_px` / `right_inward_px`
（内法向）、`box_source`（`"quad"` / `"aabb"`）、`warn`（字符串列表）

**出处**：`pallet_version` / `box_version` / `t_cam_base_src`（老路径按 TF 解析器的降级
级次给 `"tf"` / `"cache"` / `"param"` / `"identity"`；`pallet_frame: "camera"` 时是
**声明值** `identity(相机系位姿)` —— 不是回退。**照日志核对时用带括号的完整串**）/
`stamp`

## 节点参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `pallet_key` | `latest_pallet` | 读托盘位姿的键 |
| `box_key` | `latest_box_obs` | 读箱子观测的键 |
| `key` | `latest_servo_error` | 写伺服误差的键 |
| `pallet_frame` | `camera` | 黑板上的托盘位姿按哪个坐标系解释。`"camera"` = 相机系（视觉链路，**不查 TF**、走配对）；`"base_link"` = 老路径（查 TF、**绕开配对**）。**认不出来的值退回 `camera`** 并记 WARNING |
| `ref_edges` | `["y=0", "x=W"]` | 参考边。**两种形状**：`["边A","边B"]` 一组，或 `[["边A","边B"], [...]]` **多组**（按箱位切换，见下面「按箱位切换」）。第 1 个是底边（参与垂距与角度差）、第 2 个是右边（只参与垂距）。写法与坐标系见下面那张图 |
| `pallet_size_mm` | — | 台面 `[W, H]` 毫米。**必须显式写**（参数优先，参数没给才退回读 `config_path`）。⚠️ **这个值错了不会报错**，只会让参考边投到错的位置，而三个数照样算得出来 |
| `max_dt_s` | `0.05` | 配对允许的最大时间戳差（秒）。**只认正有限值**，否则退回默认值并记 WARNING |
| `pair_cache` | `5` | **每侧**留几个观测供配对。**不是窗长** —— 与 `window` 是两个独立旋钮。⚠️ **回溯窗口 ≈ `pair_cache` × 伺服 tick 周期**（10 Hz 下 `5` ≈ **500 ms**），**必须大于托盘检测器的端到端延迟** |
| `window` | `5` | **成对序列**上取几对做平均（1 = 不平滑）|
| `agg` | `mean` | `mean` / `median`，**只作用在箱子四角上**（托盘一律走刚体平均）。拼错退回 `mean` 并记 WARNING |
| `stale_s` | `0.0` | **延迟闸不是存活闸**：配对成功后这一对有多旧算陈旧（0 = 不查）。判在**去重之后**，所以某路停摆**永远轮不到它**（那时走的是 `None`）|
| `live_timeout_s` | `5.0` | **存活闸**：某一侧多久没有新观测就记一次 WARNING（0 = 不查）。认不了的值退回默认值 —— 负值若按 0 处理就是静默关掉唯一的检测器存活告警。补的是 `None` 那两种含义**不可区分**的那一格 |
| `config_path` | 见说明 | 标定结果路径。默认值按**源码位置**解析成 `<仓库根>/config/pallet_tag.yaml` 的**绝对路径**，与进程 CWD 无关。**只有 `pallet_size_mm` 没写时才会去读它** |
| `K` | — | 内参，**嵌套 3×3**（平铺 9 个数启动时就报配置错误）。**规则只有一条：没写就实时读 `/camera/color/camera_info`；写了就一个话题都不碰，用写的那个。** 写死值不是兜底、是**钉死**——真机上**别写**（写死的值换分辨率/换相机就过期，而那是静默的错），离线复现与单测**写**（结果不取决于跑在哪台机器上）|
| `D` | — | 畸变系数，同上（相机给了就用，都没有 = 无畸变）|
| `use_distortion` | `true` | 投影带不带畸变。认字符串布尔 `true/false`、`1/0`、`yes/no` |
| `image_size` | — | `[w, h]`；有了就顺带查"整条边跑到图外"。规则同 `K` |
| `camera_frame` | `camera_color_optical_frame` | 相机光学帧名 |
| `base_frames` | `[base_link, base_link_lb, base_footprint]` | 基座帧候选，按顺序试 |
| `T_cam_base_param` | — | 显式 4×4，TF 查不到且缓存失效时用 |
| `log_every_n` | `30` | 正常帧的节流日志间隔 |
| `dump_on` | `["reject", "warn"]` | 异常帧落盘条件；`"all"` = 全量 |
| `dump_dir` | `log/pallet_servo_dump` | 落盘目录；给空则关闭 |
| `dump_min_interval_sec` | `1.0` | 两份 dump 之间至少隔多久 |
| `overlay_out` | `/pallet_servo/overlay` | 叠加图发到哪个话题；**给空串 = 完全不发图**（省 CPU、省带宽）|
| `overlay_period_s` | `0.2` | 发图降频周期。伺服 10 Hz，图 5 Hz 就够看，而画图 + 编码不便宜 |
| `image_color` | `/camera/color/image_raw` | 叠加图的底图话题 |
| `box_uv_in` | `/box/yolo_box` | YOLO 框话题（**只为画灰框**）；缺了不影响出数，只是图上少一层 |
| `servo_error_out` | `/pallet_servo/dis` | 伺服误差发到哪个话题；**给空串 = 完全不发**（连发布器都不建）。它是**功能性输出**（下游控制器要用），默认**开** |
| `servo_error_heartbeat_s` | `1.0` | 无效帧的重发周期（秒）。`0` = 只在内容改变时发 —— 但要接受"停摆了就彻底没消息"，那与"话题名写错了"分不开 |

### 台面坐标系（调参考边前先看这张图）

```text
              +y  (沿 E2，投影到图像里朝上)
               ↑
               |
       (0,H)   ●───────────────●  (W,H)
               │               │
               │      台面      │
               │  [0,W]×[0,H]  │
               │    z = 0      │
       (0,0)   ●───────────────●  (W,0)
               └───────────────→  +x  (沿 E1，投影到图像里朝右)
            原点 = 画面**左下角**

  四条边： y=0 → 近边（画面里靠下）    y=H → 远边（画面里靠上）
           x=0 → 左边                x=W → 右边
```

⚠️ **轴向是相对相机定的，不是相对托盘物理定的**（`algorithm.py` 的 `_orient`）：
`E2` 取「投影到图像里 v 更小」的那条边方向，`E1 = E2 × nrm` 且**保证指向画面右**，
原点取四个角里「u 小、v 大」的那个。所以 **相机装法一变，「哪条边是 y=0」跟着变** ——
**换工位后必须重新在叠加图上确认一次**（见下面的摆放约定）。

### 参考边的两种写法

| 写法 | 含义 |
|---|---|
| `y=<毫米>` / `x=<毫米>` | 与同名轴平行的直线。**板子里一律用这个** |
| `y=0` / `y=H` / `x=W` / `x=0` | 台面矩形的四条边，**跟着 `pallet_size_mm` 走**。代码认，但板子里不要写 |

绝对写法的**范围是 `0 ~ 对应边长`**（`y=` 比 H、`x=` 比 W），超出在**启动时**
就拒（`ref_edge_out_of_range`），不留到第一帧。

为什么板子里用绝对值：`y=H` 与 `y=1000` 在 H=1000 时是同一件事，但**绝对值只有
一个意思** —— 看板子的人不用先去查 `pallet_size_mm` 是多少，也不会因为改了尺寸
而让某条边悄悄跟着挪。代价是换托盘时 `ref_edges` 与 `pallet_size_mm` **要一起改**。

`ref_edges` 四条边的物理含义（**换了建系顺序这张表整体变**，见 §3 第 2 条）：

| 值 | 物理边 | 值 | 物理边 |
|------|------|------|------|
| `"y=0"` | **下边** | `"x=0"` | 左边 |
| `"y=H"` | 上边 | `"x=W"` | **右边** |

⚠️ **摆放约定（2026-09-24 操作员确认）**：台面 `[1200, 1000]` mm，**1200 是长边、1000 是短边**。
摆放时 **短边（1000）与机器人平行**，即**长边（1200）垂直于机器人正面**；
`origin` 取台面的**左下角**（`e1` 沿长边、`e2` 沿短边，`e1 × e2 = normal` 指向相机）。

⚠️ **托盘摆反 90° 不会报错** —— 参考边会取到**别的物理边上**，而三个数照样算得出来，
伺服照着错的参考边把箱子送过去。现场唯一能看出不对的是**人眼**在叠加图上
（`/pallet_servo/overlay`）看青框贴不贴托盘。

### 按箱位切换：`ref_edges` 可以是多组

板上写 N 组参考边，服务里第 N 个箱子用第 N 组：

```json
["y=0", "x=1200"]                            # 一组（单箱位，等价于老写法）
[["y=0","x=1200"], ["y=1000","x=1200"]]      # 两组，call 1 / call 2 分别用
```

**节点启动时是「未激活」的** —— 没 call 过之前一帧有效数据都不出，话题上发的是
`valid=false` 的「伺服未启动」。要出数就 call：

```bash
rosservice call /pallet_servo/slot "slot: 1"   # 第 1 个箱子
rosservice call /pallet_servo/slot "slot: 2"   # 第 2 个箱子
rosservice call /pallet_servo/slot "slot: 0"   # 本次伺服结束
```

⚠️ **服务从 1 数，数组从 0 数** —— `slot=2` 用 `ref_edges[1]`。越界调用
（`slot > 组数`、负数）返回 `ok=false` **且不改当前状态** —— 手滑发个 `3`
不会把正在跑的伺服停掉。

⚠️ **整张表在 `initialise()` 里一次校验完** —— 第 3 组写错了启动时就报，
不留到轮到它才发现（那时箱子已经在托盘上了）。

⚠️ **两个箱子同时在视野里时选箱可能挑错**：`latest_box_obs` 只有**一个**箱子，
选箱按「置信度 + 面积 + 中心深度」打分。箱位 1 的没搬走、箱位 2 的已放下时，
节点很可能挑到旧的，而**三个数照样算得出来**（只是基准是另一个箱位）、**不报错**。
靠流程保证「先搬走再放」。

## 叠加图（`overlay_out`）

发到 `/pallet_servo/overlay`（`sensor_msgs/Image`），底图取 `image_color`，
按 `overlay_period_s` 降频。

| 颜色 | 画的是什么 |
|------|-----------|
| **黄线** | 托盘的两条参考边，**标签就是你写的那两条原文**（`y=0` / `x=1200`）—— 现场照着图核对"我配的边对不对"用的就是它 |
| **绿线** | 箱子的底边与右边 |
| **青线** | 托盘台面矩形（四条边）|
| **灰线** | YOLO 输入框（`box_uv_in`，**只是输入**）|
| **蓝箭头** | 参考边的内法向 |
| **红线** | 垂距线（箱子边的中点 → 参考边上的垂足）；**紫圆** = 垂足 |

**出图前先做数值自检**（`render.numeric_self_check`）：在「本该是某个颜色」的位置
采样，色不对就把问题打出来。理由是人眼看到图不对时第一嫌疑应该是**画图代码**，
不是数据 —— 一张画错了的图会让人去改本来正确的算法。**没通过自检的那一帧不发图**
（并按「探针名 + 失败种类」去重告警），而不是发出去让人看。

三条"照抄会错"的口径：

- **重合的线只画一条**，优先级 `箱子边 > 参考边 > 台面矩形边 > YOLO 框边`，
  被跳过的**既不画也不探**（没画的线没有探针可失败）。**这是几何事实不是像素
  巧合**：默认 `ref_edges = ["y=0","x=W"]` 的两条参考边，在托盘台面系里**就是**
  台面矩形的边0 与边1（同一个 `T_cam_pallet` 投出来，像素级完全重合）。
- **判据是「取整后的端点相等」，不是「浮点相等」**：失效是**渲染问题不是几何
  问题** —— `_draw_segment` 把端点吸附到整数像素，相差 0.4px 的两条线**画出来
  是同一批像素**。判浮点相等会让 `|d| ≤ 0.5px` 的整个亚像素带都红，而伺服收敛
  后箱子正停在这一带里。这里**没有容差常数**。
- **还有第三类：共线但长度不同** —— 短的那条只盖住长的那条的中间一段，探针又从
  几何中点向两端走、找到还看得见的一段（`_snap_probes_to_drawn`，挪位**跑在
  画文字之后**，否则找到的位置随后被字盖掉）。**代价**：探针验证的变成
  「**这条线段上某处**是本色」，于是「线画对了但只画了一半」**不再会被抓到**。

> ⚠️ 验收这类自检时，**扫描维度必须含 `theta`**（不能只扫 `theta = 0`）——
> 只扫零角会漏掉「探针的几何中点落在另一个图层的文字下面」这一整类。

## `T_cam_base` 怎么来的

按 **TF → 显式参数 `T_cam_base_param` → 单位阵** 逐级解析。

TF 用**短超时（0.05 s）+ 最近一次成功值缓存**—— 标定工具那种阻塞查询会把伺服环路
卡死。当前用的是哪一级，写在输出的 `t_cam_base_src` 字段里。

**"连续 5 帧失败才降级"只在已经有缓存时成立**：冷启动（还没成功过一次）时**第一次
查询失败就直接走 `param` / `identity`**，不会等满 5 帧。降级与恢复各记一次 WARNING。

# 2. 快速测试

## 2.1 不用点（回放上一次的点击）

**输入是一份抓好的帧**（`--capture` 落下的三件套）。下面这条命令里
`--corners` / `--box-corners` 是**回放**已有的点击，所以不弹取点窗、结果可复现：

```bash
cd /data/Real_Downloads/LeTools        # 替换为本机 LeTools 根目录

python3 apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py \
    --capture-dir /tmp/servo_sim \
    --corners 486.0 416.6 82.4 425.7 137.3 41.4 439.7 37.2 \
    --box-corners 425.3 449.8 245.7 453.9 255.4 197.9 405.8 196.9 \
    --stage both --box-mode quad --out-dir /tmp/servo_sim
```

> 第一次用之前先抓一帧：`--capture --stage 1 --out-dir /tmp/servo_sim`
> （要在**装了 ROS 的那台机器**上跑；抓帧会先落盘，之后反复点点都读同一份）。

`--corners` / `--box-corners` 是**回放已有的点击**（8 个数 = 4 个点 × u v），
所以**不弹取点窗**、结果可复现。预期输出：

```text
 求出 T_cam_pallet：平移 (-0.5846, 0.4474, 0.9333) m，台面 1052 x 1150 mm
 参考边 y=0 投影到 [[89.6, 422.3], [491.9, 419.4]] px
 数值自检通过（探针都取到了目标颜色）
 三个量：e_bottom = -33.9 px   e_right = +67.4 px   theta = -0.30°
 写出 /tmp/servo_sim/stage3_overlay.png
 写出 /tmp/servo_sim/pallet_servo_sim.json
```

**看图**（不能跳，图是唯一的验证手段）：`/tmp/servo_sim/stage3_overlay.png`

| 颜色 | 画的是什么 |
|------|-----------|
| **黄线** | 托盘的两条参考边（投影到画面上）|
| **绿线** | 箱子的底边与右边（`render_overlay.py:40` `COLOR_BOX_EDGE = (0, 255, 120)`）|
| **红线** | **垂距线**：箱子边的中点 → 参考边上的垂足（`COLOR_OFFSET = (0, 0, 255)`）|
| **蓝箭头** | 参考边的内法向 |

要确认的是：**蓝箭头指向托盘内部**，且**红线（垂距）方向与箭头一致**。
（方向一致只在 `e < 0` 时表现为"沿箭头方向"—— 上面这个例子里 `e_bottom = −33.9 px`
正好是这种情况。看符号以 §1 的契约为准，别只看箭头。）

> ⚠️ 这三个数的**绝对值带一个常数偏置**（参考边整体偏了一点，成因见 `NOTES.md` §2
> 闸门①）。**离线可以用它验符号与动态，不要拿它的绝对值当真机上的目标值。**
> 这条不影响真机路径。

## 2.2 自己点

```bash
# 抓一帧（真机；会先落盘到 out-dir，之后都读这一份）
python3 apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py \
    --capture --stage 1 --out-dir /tmp/servo_sim

# 阶段一：点托盘台面四角 → stage1_overlay.png（**在这儿看图确认参考边贴合托盘**）
python3 apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py \
    --capture-dir /tmp/servo_sim --stage 1 --out-dir /tmp/servo_sim

# 阶段二：接着同一份 stage1.json 点箱子四角（托盘不用重点）
#    ⚠️ 阶段二**不能再 --capture** —— 那会重抓一帧覆盖 color.png，而 stage1.json
#    记的是同一个路径，于是拿旧位姿配新图。工具会直接拒绝这种组合。
python3 apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py \
    --capture-dir /tmp/servo_sim --stage 2 --box-mode quad --out-dir /tmp/servo_sim
```

分两阶段是因为**投影错了后面三个量全错，而且看不出来** —— 阶段一结束处有一道人工闸门。

| 参数 | 干什么 |
|------|--------|
| `--capture` | **真机抓一帧**，先落盘再继续。主路径；**不能与 `--stage 2` 同用** |
| `--capture-dir <目录>` | **读抓好的那一份**。内参/图尺寸/深度尺度**取自 `meta.json`**，不用手敲。抓一次，反复点 |
| `--color + --depth + --fx/--fy/--cx/--cy` | 手敲内参。**敲错一个 `cx` 会让所有像素量整体偏，闸门不一定看得出来** —— 能用 `--capture-dir` 就别用这条 |
| `--dataset <目录>` | 读一份外部的帧束目录（回放旁路，不依赖相机）|

| 参数 | 干什么 |
|------|--------|
| `--box-mode aabb` | 点 2 个对角（左上、右下）—— YOLO 给的就是这两个 |
| `--box-mode quad` | 点 4 个角，**必须按契约顺序**（§3 第 2 条）|

产物落在 `--out-dir`：`stage1.json` / `stage1_overlay.png` / `stage3_overlay.png` /
`pallet_servo_sim.json`。

## 2.3 接进行为树

> **真机上现成的场景在 `orchestration/scenarios/pallet_servo_real_v1/`** ——
> 那里托盘走**视觉托盘检测器**（`NodePalletObs` 订阅 `/pallet/detection`），箱子走
> `NodeBoxObs`（订阅 `/box/detection`），**两路都是自动的**。
> 本节下面这段是**纯离线**的骨架：托盘和箱子都由那份 JSON 提供。
> 前置条件、`K` 从哪抄、跑之前要改什么，都在那个场景的 `README.md` 里。

工具最后会打印一段**可直接粘进场景 JSON** 的参数块，值必须与本次点点一致：

```json
  "ref_edges":      ["y=0", "x=W"],
  "pallet_size_mm": [1052, 1150],
  "K": [[368.5230712890625, 0.0, 320.3636169433594],
        [0.0, 368.4582824707031, 245.7123260498047],
        [0.0, 0.0, 1.0]],
  "image_size": [640, 480]
```

场景骨架：

```json
{ "name": "Parallel", "label": "servo_offline_par",
  "params": { "policy": { "value": "success_on_one", "source": "CUSTOM", "data_type": "string" } },
  "childs": [
    { "name": "NodeInjectServoInput", "label": "inject_servo_input",
      "params": {
        "sim_path": { "value": "/tmp/servo_sim/pallet_servo_sim.json", "source": "CUSTOM", "data_type": "string" },
        "enabled":  { "value": "true", "source": "CUSTOM", "data_type": "string" } },
      "childs": [], "childBoard": [] },
    { "name": "NodePalletServo", "label": "pallet_servo",
      "params": {
        "ref_edges":      { "value": ["y=0", "x=W"], "source": "CUSTOM", "data_type": "strArr" },
        "pallet_size_mm": { "value": [1052, 1150], "source": "CUSTOM", "data_type": "floatArr" },
        "K":              { "value": [[368.5230712890625, 0.0, 320.3636169433594],
                                      [0.0, 368.4582824707031, 245.7123260498047],
                                      [0.0, 0.0, 1.0]], "source": "CUSTOM", "data_type": "floatArr" },
        "image_size":     { "value": [640, 480], "source": "CUSTOM", "data_type": "intArr" } },
      "childs": [], "childBoard": [] },
    { "name": "WaitSeconds", "label": "run_for",
      "params": { "duration_sec": { "value": "5.0", "source": "CUSTOM", "data_type": "float" } },
      "childs": [], "childBoard": [] }
  ], "childBoard": [] }
```

**先干跑一次**确认节点能被工厂按类名解析、黑板键注册成功（不碰硬件、不弹窗）：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
    --scenario <场景目录> --dry-run --tick-once
```

单元测试：

```bash
# 脚本式（不在 CI 的收集范围内，要手动跑）
python3 apps/test_kuavo_5w_skills/test_pallet_servo.py                  # 算法层 + 技能层
python3 apps/test_camera_internal/pallet_servo_sim/test_capture_one_frame.py

# CI 会跑（verify:opensource 跑的正是 pytest orchestration/nodes/tests/ -m unit）
pytest orchestration/nodes/tests/test_node_pallet_servo.py -m unit -q
pytest orchestration/nodes/tests/test_node_box_obs.py -m unit -q        # 箱子观测节点
pytest orchestration/nodes/tests/test_box_obs_to_servo_e2e.py -m unit -q  # 端到端合成
```

# 3. 注意事项

1. **上真机前还剩这几条**（箱子生产者与标定手性两条**已经修掉**，见下）。

   **新的视觉链路（`pallet_frame: "camera"`，本仓库的默认）只要第 1 条 + 一个在跑
   的 `/pallet/detection` 生产者 + 第 6 条（要发话题的话）**；第 3~5 条是
   **`base_link` 老路径（AprilTag）** 才需要的，走新链路时**不用碰**。

   | # | 卡点 | 现状 |
   |------|------|------|
   | 1 | **箱子检测话题** | ✅ **已有**：`NodeBoxObs`（`orchestration/nodes/node_box_obs.py`）订阅 `/box/detection`（`box_detection_msgs/BoxDetection`），写 `latest_box_obs` |
   | 2 | **标定手性** | ✅ **已修**：标定工具现在会重排 + 手性自检 + **拒绝写出镜面标定**（见本条末尾） |
   | 3 | （老路径）`config/camera_config.yaml` 的 `launch_apriltag` | 当前 `false`，此时 `lifecycle_mixin` **根本不构造** `PerceptionAdapter`（`lifecycle_mixin.py:135`），`hardware.perception` 是 `None`，`NodePercep` 会爆。**要改成 `true`** |
   | 4 | （老路径）`config/apriltag_tags.yaml` | 当前 id 1–4 / size 0.06，**和 maduo 用的标签对不上**，要填真实 id 与尺寸 |
   | 5 | （老路径）`config/pallet_tag.yaml` | **必须重标**（见下）——老路径下 `pallet_size_mm` 不给就从这里读，两者都没有 = 启动时 FAILURE。**新链路写场景 `py_tree.json` 的 `pallet_size_mm`，不读这个文件** |
   | 6 | **`pallet_servo_msgs` 编译过** | 只影响**发话题**：没编过时节点记一条 WARNING 说"只写黑板，行为树之外的控制器拿不到误差"，其余照常 |

   **`NodeBoxObs` 的三个参数就是接到真机的全部动作**：`topic`（默认 `/box/detection`）、
   `msg_type`（默认 `box_detection_msgs/BoxDetection`，运行期按全名解析，**换话题/换类型
   都只改参数**）、`box_key`（默认 `latest_box_obs`，要与本节点的 `box_key` 对得上）。
   它**不 import 那个 catkin 包**（按字段名取），所以 LeTools 不多一条构建依赖。

   ⚠️ **`valid == false` 的帧一个字节都不写黑板。** 上游 `.msg` 的注释原话：
   「`false` = 四角就是原始 YOLO 框，别拿去做伺服」——那时箱子旋转没恢复，
   拿它算出来的 `theta` 是**错的**，而四角看着规规矩矩是个矩形、`e_bottom_px` 也
   像模像样。所以这种帧被跳过，下游继续用上一帧的好值。详见 `NOTES.md`。

   **接真机时把模拟源关掉**（伺服节点一行不改）：`NodeInjectServoInput` 的
   `enabled: false` —— 此时它**运行期一个键都不写**并返回 `SUCCESS`（不占着树，否则
   "真机上关掉它"会把整条分支永远卡住）。认 `true/false`、`1/0`、`yes/no` 的
   **字符串与 JSON 布尔值**。两点别记反：**构造时它仍会写那 4 个键的初值**；
   而**解释不了的值虽然也按"关掉"处理并报 ERROR，但 `update()` 返回的是 `FAILURE`**。

   > ### ⚠️ 标定工具修好了手性 —— **旧标定一律作废，必须重标**
   >
   > 标定工具以前**没做重排**（离线点点工具做了、它没做），顺时针点就会建出**镜面**
   > 的 `T_cam_pallet`，进而写出镜面的 `T_pallet_tag`。而**镜面在下游是静默的**：
   > `matrix_to_pose6d()` 走 `Rotation.from_matrix`，把镜面投影成"最近的旋转"，
   > `NodePalletPose` 写在黑板上的 `Pose6D` **还原不回**标定出来的系，伺服一路吃到
   > 被镜像的位姿而**没有任何报错**。
   >
   > 现在工具会：**内部重排**（与离线工具共用算法层同一份置换）→ **手性自检**
   > （`det` 必须落在 +1 一侧，判据是符号测试 `det < 0.5`）→ **写盘前拒绝镜面**
   > （算出来是镜面就一个文件都不写，除非显式给 `--allow-mirrored`）。
   > 落盘的 yaml 里还多了一个 `T_pallet_tag_det` 字段，现场一眼能看出是不是镜面。
   >
   > **语义变了**：修好之后 `x=0` = 左边、`x=W` = 右边（以前是反的）。所以
   > **任何既有的 `config/pallet_tag.yaml` 都不能再用，必须重标**；`ref_edges`
   > 的取值也要按 §6.2 那张新表重挑。这也是**两条路径第一次真正一致**的时刻——
   > §6.2 那个"写错了不报错、只是伺服错了边"的陷阱被填掉了。详见 `NOTES.md` §6。

2. **点击顺序是契约，不是偏好。** 托盘与箱子**都是** `右下 → 左下 → 左上 → 右上`。
   它决定托盘系的轴向、也决定箱子哪条边是"底边"。**点错顺序不会报错**，只会让参考边
   落到别的物理边上，而三个量照样算得出来。

   - **托盘那边工具会内部重排**（固定置换 `(1, 0, 3, 2)`）：`PalletFrame` 要求
     `e1 × e2 = normal`，而图像上顺时针的环序给出的是 `−normal`（`det = −1`，
     **镜面不是旋转**）。工具建出 `T_cam_pallet` 后会**自检手性**，这是镜面唯一能在
     源头被拦下的地方。
   - **箱子那边不重排**，`[p0,p1,p2,p3] = [右下,左下,左上,右上]`，
     `底边 = p0→p1`、`右边 = p3→p0`。四个点只是一份"边的清单"（不建坐标系，没有手性
     要求），所以原样使用。**但这条契约没有任何自动检查兜住** —— 凸环自检只要求相邻边
     叉积**同号**，只拦"自交/凹"；**整个环反着走**（叉积仍同号 → 放行，但 `p0→p1`
     取到的是**顶边**）和**循环移位**（换个人起头 → `bottom`/`right` 一起错位）
     都会被放行、静默取错边；`theta` 的 ±45° 也兜不住（线角是无向的）。

   **责任在调用方**：操作员按契约点，或将来填 `quad` 的边缘拟合模块按契约填。

3. **两条路径的托盘系现在一致了**（2026-09-21 修）。以前不一致：离线点点工具做了
   重排（右手系），而**标定工具没做**，顺时针点就建出左手系（实测每帧
   `det = −1.000000`），于是同一组物理边在两条路径上是**反的**，写错**不报错**、
   只是伺服错了边。标定工具补上重排之后，两边共用算法层同一份
   `PALLET_CLICK_PERMUTATION`：

   | `ref_edges` | 物理边（**两条路径现在相同**）|
   |------|------|
   | `"y=0"` | **下边**（左下 → 右下）|
   | `"y=H"` | 上边（左上 → 右上）|
   | `"x=0"` | 左边（左下 → 左上）|
   | `"x=W"` | **右边**（右下 → 右上）|

   **但这条一致性是"标定必须重跑"换来的**：任何在修复之前标出来的
   `config/pallet_tag.yaml` 都还是左手系（`x=0`/`x=W` 反着），**必须重标**。
   标定工具现在会**拒绝写出镜面标定**，所以重标之后不会再出现这种情况。

4. **消费方必须用 `_version` 判新旧。** **被拒的帧不写黑板，版本号也不自增** ——
   如果之前成功过一帧、随后来的帧被拒了，**黑板上的值还是上一帧的**，而只看值不看
   版本号的消费者会拿到**过期误差**（而且它看着完全正常）。这是整套设计的约定：
   `latest_pallet` / `latest_box_obs` / `latest_servo_error` 每个键都配了 `_version`。
   被拒时**不写 `None`** 是因为环路上的瞬时拒绝（相机晃一下、托盘被挡一下）不该把目标
   清掉、让机器人停下来。

   节点在**正常路径**上持续返回 `RUNNING`（dry-run 下 `SUCCESS`；配置错误或缺台面尺寸
   时 `FAILURE`），**两个版本号都没变时不重算**（版本门禁照抄
   `NodePalletPose.update()`），并且**在同一个被拒帧上不反复刷屏**（版本门禁会记住
   这一帧），但**每一帧被拒都会记 WARNING —— 实际是两条**，技能层一条（`skill.py:249`）
   加节点层一条（`node_pallet_servo.py:669`），都带实际值与阈值。带 `warn` 的正常帧
   另有 `skill.py:251` 一条 INFO。

   节点会写 `latest_servo_error` 的初值（`None` / `0`），所以消费者 `getattr` 不会
   `KeyError`；上游缺输入是**正常的"等输入"状态**，不是错误。

5. **离线路径的 `T_cam_base = 单位阵` 是「声明」，不是「回退」—— 那条路上没有失效模式。**
   离线点点的托盘位姿是**相机系**的，`pallet_frame` 走默认值 `"camera"`。这个模式下节点
   **根本不构造** `TfCamBaseResolver`（`node_pallet_servo.py:1008-1011` 只在
   `base_link` 时才建它），`_resolve_t_cam_base()` 直接返回声明的单位阵，
   `t_cam_base_src` 恒为 `identity(相机系位姿)`。

   所以**离线这条路不会**"在一台 TF 真能解出 `camera_color_optical_frame ← base_link`
   的机器上，被那个真实变换多乘一次" —— 那一级压根不存在。**不用再为此去动
   `base_frames`。**

   **要确认 TF 的是老路径（`pallet_frame: "base_link"`）**：只有它才构造解析器，按
   **TF → 显式参数 `T_cam_base_param` → 单位阵** 逐级解析，而 TF 查不到时是**静默**
   退回下一级（只有一条 WARNING）。在那条路上，如果黑板上的位姿其实是相机系、TF 又
   真能解出来，三个量就会变成一个完全没有意义的值而没有任何报错 —— 所以跑老路径前
   确认 `t_cam_base_src` 是 `"tf"` 或 `"param"`（**不要是 `"identity"`**）。

   真机走的就是老路径，它本来就**该**走 TF；离线那条路则不碰 TF。

6. **内参有两个来源，能默默不一致。** `pallet_servo_sim.json` 里的
   `K/D/image_size/use_distortion` 和伺服节点自己的 `params` 是两份 —— 不一致时误差会
   **静默算错**（值仍然像模像样）。两个节点启动时都把自己那一份打进 INFO，**跑之前对
   一眼**。

7. **`use_distortion` 必须与算像素的那一侧一致。** 伺服若在原图（带畸变）上算像素，
   投影就必须带 `D`；反之不带。不一致时**不会报错**，只是默默分叉（实测：开
   135.918 px / 关 137.000 px）。

8. **`NodeInjectServoInput` 是源节点，持续 `RUNNING`、永远不会自己结束。** 所以
   **不能单独放进 `Sequence`**，也**不能放进必须完成的 `Parallel(SuccessOnAll)` 支路**
   —— 那样它下游的伺服节点**永远不会被 tick**。**解决**：放进 `Parallel`，由**别的**
   子节点决定何时收（固定时长用 `WaitSeconds`，人工看够了再收用 `WaitForEnter`）。

   另外它**播完就停手、不再重读 JSON**：读 JSON 只在 `initialise()` 里发生一次，而它
   持续 `RUNNING`，`initialise()` 不会再被重进。所以「改点点 → 重新生成 JSON → 看伺服
   实时变化」**必须重进这一支或重跑场景**才生效。（`box` 写成**列表**时是一条条往下播
   的，每写一条版本号自增，所以看得到"箱子在动"。）

9. **单位是分裂的。** 托盘系内部的几何量一律**毫米**（台面 `[0, W] × [0, H]`，`z = 0`），
   而 `T_cam_pallet` 的平移是**米**（与 `PalletFrame.to_matrix()`、框架 `Pose6D` 一致）。
   投影前那一步换算写错了会**静默差 1000 倍**，而且结果仍然是一对"看着像像素坐标"的数。

10. **直线无向，`theta` 折进 `(−90°, 90°]`。** 一条边是 p0→p1 还是 p1→p0 是同一条线；
    不折的话两个视觉上重合的边会算出 178° 的"角度差"。所以**"转 180°"是同一个角** ——
    下游别指望用 `theta` 分辨"箱子掉了个头"，那要靠别的量。

11. **`e` 的极性 2026-09-30 翻过来了：内侧为负**（指 `e_bottom` / `e_right`，
    **不是 `theta`**）。闸门②当时裁决「先暂定这样，后续可能要反转」——
    现场调试时操作员确定要**反**，于是在**算法层**翻（`servo_error` 里那一句），
    契约（本文档 §接口 与 `PalletServoError.msg`）一起改。
    ⚠️ **翻的只是 `e_px` 这一个字段；`inward_px` 仍是「从参考边指向托盘内部」** ——
    它给叠加图画蓝箭头（手工闸门之一是「蓝箭头指向托盘内部」）、给 `foot_outside`
    用，都与误差符号无关。翻的时候别连它一起翻。

12. **离线数不能反过来当"托盘在机器人坐标系里的位置"用**（同第 5 条）。

# 4. 相关文档

| 文档 | 内容 |
|------|------|
| **`docs/托盘伺服部署与跑通.md`** | **从零跑到三个差值**的操作手册：两条路径的完整命令、点击顺序、现场要定哪几个值、注意事项。**在一台新机器上部署就看这一份**，不解释原理 |
| `NOTES.md` | 踩过的坑、人工闸门评审记录、日志与 dump 细节、`NodeBoxObs` 的三个决定、本次修的三处静默错 |
| `pallet_frame/README.md` | **配对与平滑**：两个观测按图像时刻配对、五种 reject、`None` 的两种含义、`spread_*` 口径 |
| `orchestration/nodes/node_pallet_obs.py` | 托盘观测生产者（`latest_pallet` + `_stamp`），订阅 `/pallet/detection` |
| `pallet_pose/README.md` | `base_link` 老路径的托盘位姿模块（`latest_pallet` 的另一个生产者）|
| `orchestration/nodes/node_box_obs.py` | 箱子观测生产者（`latest_box_obs`），订阅 `/box/detection` |
| `apps/test_camera_internal/pallet_servo_sim/` | 点点工具与出图工具 |
| `apps/test_camera_internal/pallet_calibration/` | 标定工具（会重排 + 手性自检 + 拒绝写出镜面标定）|
