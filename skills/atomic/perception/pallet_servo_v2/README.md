# 托盘伺服误差（图像空间，v2）

> **v2 当前链路**：托盘用 POSE 12 关键点 + call 0 记录（D/法向），
> `slot=0` 记录、`slot=-1` 退出；不要再套用 v1 的 `slot=0` 停止语义。
> 部署见 `docs_internal/托盘伺服部署与跑通.md`。

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
  `skills/atomic/perception/pallet_frame_v2/README.md`
- **三个数发到哪** → `/pallet_servo_v2/dis`（§1「对外接口是话题，不是黑板」）
- **现在能上真机吗** → §3 第 1 条（还剩 3 条）

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
改这块之前先读 `skills/atomic/perception/pallet_frame_v2/README.md` 注意事项 ⑪。

> 配对要用的**图像采集时刻**在另一个键上：`latest_pallet_stamp`（没有它配对会
> 退回节点 tick 时刻，两个检测器的耗时差又回来了）。`NodePalletObs` 与
> `latest_pallet` 一起写它。

| 生产者 | 谁 |
|--------|-----|
| 真机（视觉） | **`NodePalletObs`**（`orchestration/nodes/node_pallet_obs.py`）—— 订阅 `/pallet/detection` |
| 真机（老路径） | `NodePalletPose`（`skills/atomic/perception/pallet_pose/`）|
| 离线 | `NodeInjectServoInput`（§2）|

### `latest_box_obs` + `latest_box_obs_version`

类型 **`skills.atomic.perception.pallet_servo_v2.algorithm.BoxObservation`**。

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
含义、`spread_*` 的口径）见 `skills/atomic/perception/pallet_frame_v2/README.md`。

## 输出

### 对外接口是话题，不是黑板

**行为树之外的控制器请订阅 `/pallet_servo_v2/dis`**（`pallet_servo_msgs_v2/PalletServoError`）。

| | 黑板 `latest_servo_error` | 话题 `/pallet_servo_v2/dis` |
|---|---|---|
| 谁能看见 | **只有行为树那个进程**（py_trees 的 `Blackboard` 是进程内单例）| 任何人（ROS 话题）|
| 用途 | **树内**节点之间的实时值 | **对外**的承诺接口 |
| 装什么 | 完整 `ServoError`（含参考边端点、垂足、内法向等诊断量）| 三个被控量 + `valid` + `reject` |

⚠️ **别拿黑板当控制器的接口。** 机器人上的伺服环路 / MPC **不在树里**，它读不到
黑板 —— 不是"不方便"，是**根本看不见**。两者同源同帧，但**只有话题是承诺过的接口**。

字段、`valid` 的语义、`header.stamp` 的口径、心跳周期，全部写在
`infrastructure/ros_packages/src/ros_vision/pallet_servo_msgs_v2/msg/PalletServoError.msg`
的注释里 —— **那里是契约，这里是索引**。

消费方的标准动作三步：判 `valid` → 有效就拿三个量伺服 → 无效就**不要动**、看
`reject` 查明原因。

### `latest_servo_error` + `latest_servo_error_version`（**树内**，见上一节）

类型 **`skills.atomic.perception.pallet_servo_v2.algorithm.ServoError`**。

**三个被控量**

| 字段 | 含义 |
|------|------|
| `e_bottom_px` | 箱子**底边**中点到托盘**底边**（图像投影）的带符号垂距，像素 |
| `e_right_px` | 箱子**右边**中点到托盘**右边**（图像投影）的带符号垂距，像素 |
| `theta_rad` | 箱子底边与托盘底边的角度差，带符号，弧度，正常 ±45° 以内 |

**符号约定：`e < 0` ⇔ 箱子这条边落在托盘参考边的内侧**（2026-09-30 起；原来是「内侧为正」，操作员裁定翻成「箱子在里面输出负数」）。

⚠️ **邻居路线（`box_types` 里 `mode: neighbor` 的槽位；本现场是 2/4/5 档）
用逐字相同的约定**（`neighbor.py`）：

* `e_bottom > 0` ⇔ 手里箱**还没到位**（底边在参照线靠机器人这一侧）→ **前进**；
  推过头为负 → **后退**。与「底边在伺服边下方发正、超过发负」是同一句话。
* `e_right > 0` ⇔ 手里箱**偏画面右**。
* **机器人挪到托盘对面（画面左右翻转）时不做任何符号翻转** —— 相机跟着
  转 180°，画面**上下没反**，"往里走"两种视角下都还是画面里的"上"。
  现场实测过：以为要翻、翻了反而反了。

> ⚠️ **节点可按槽位给输出加固定偏置**：`box_types` 每个槽位可选
> `e_right_bias_px` / `e_bottom_bias_px`（默认 0）。`NodePalletServo` 在写黑板、
> 发话题、出叠加图**之前**执行 `e_* = 原始值 + bias`，所以话题/黑板上的值可能
> 带一个**按档位固定**的常数偏移；`theta_rad` 不加偏置。偏置不改变算法，只改
> "控制器把哪个实际位置当成 0"。配置见下面的「`box_types`」。

内法向 `n` 由**托盘自己的几何**定死（候选 `±(−d_v, d_u)`，取与「台面中心的投影 −
参考边中点」点积为正的那个），所以**不依赖托盘在画面里的朝向**：

```
e = (箱子边的中点 − 它在参考边所在直线上的垂足) · n
```

垂距取**中点**而不是"线到线"：两线不平行时线到线是一个区间，而伺服推的就是这个点。
垂足落在**参考线段之外**时置 `foot_outside` warn，**仍然输出**（箱子已偏出这条边的
范围，但垂距本身还是有效信息）。

`theta` 的定号：`line_angle = atan2(Δv, Δu)`、**`theta = 箱子底边角 − 托盘底边角`**。
与 `e_*` 同向：都是「手里箱 − 参照」。控制律 `dtheta = gain * theta_rad` 是正反馈
口径，所以这个符号反了就是**越转越偏**；折进 `(−90°, 90°]`。

> ⚠️ **极性翻过两次。** 2026-09-29 从「箱子 − 托盘」改成「托盘 − 箱子」；2026-10-20
> 真机实测角度仍是反的，翻回「箱子 − 托盘」。**改视觉层而不是让控制器取负**：极性
> 是符号约定，两处一起改才一致。
>
> ⚠️ **没有测试钉住这个符号**（收敛扫描只扫 `theta = 0`），改反了测试照样全绿，验收
> 只能靠叠加图上眼看。

`|theta| > 45°` **只置 warn、不拒绝输出**（`theta` 超出正常范围本身不是"算不出来"）。
⚠️ **warn 与日志行里的 theta 都是弧度**（2026-10-08 起），与 `theta_rad` 同单位 ——
warn 文案长这样：`theta +0.872rad 超出 ±0.785rad`。

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
| `box_types` | —（旧键：`ref_edges` + `servo_slots`） | **箱型表**：每个箱型一张槽位表，槽位数量与编号独立。槽位里 `ref_edges` 是 final（`stage=0`）的待伺服边，`initial.ref_edges` 是 initial（`stage=1`）的覆盖；`mode` / `side` / `box_plane_height_mm` / 偏置 / `normal_rot_deg` 也在槽位里。写法与坐标系见下面那张图。⚠️ 与旧键 `ref_edges` / `servo_slots` **不能同时配置** |
| `box_plane_height_mm` | `250` | **箱子四角所在平面高出托盘台面的毫米数**（现场箱高；近大远小修正，`0` = 关）。给了之后：四角反投影到该高度的平面 → 沿托盘法向落到台面 → 重新投影，再与台面参考边比。**每个槽位可在 `box_types` 里用同名字段覆盖**（两层码放第二层更高）。没有 `quad` 的观测不做修正 |
| `neighbor_enabled` | `true` | 邻居伺服总开关。`false` = 所有槽位按 `pallet` 跑，`box_types` 原样保留（重新设 true 即恢复）。见下面「`neighbor_enabled`」 |
| `pallet_record_path` | 空 = `<仓库根>/log/pallet_servo_pallet_record.json` | `slot=0` 观测托盘后保存 base_link 系法向、台面高度 `plane_z_mm`、D 与尺寸的文件；跨开关机保留，下一次 call 0 成功后覆盖 |
| `pallet_record_samples` | `5` | call 0 目标采样帧数 |
| `pallet_record_min_samples` | `3` | call 0 最少可接受帧数；不足则本次失败、旧文件保留 |
| `pallet_record_timeout_s` | `8.0` | call 0 服务超时；超时仍不满足则 `ok=false` |
| `pallet_record_max_spread_mm` | `10.0` | 样本位置散布上限（mm）；超限继续采样，超时仍超限则失败。`0` = 不检查 |
| `pallet_record_max_spread_deg` | `2.0` | 样本法向散布上限（度）；同上 |
| `bag_record_enabled` | `false`（板子设 `true`） | 伺服阶段 bag 记录总开关。只在 `slot>=1` 记录；`slot=-1`/`0` 停止；切阶段换新 bag |
| `bag_dir` | 空 = `<仓库根>/log/pallet_servo_bags` | bag 根目录；按 `YYYYMMDD` 子目录存放 |
| `bag_topics` | v1+v2 两套检测/伺服话题 + 相机/TF | 默认同时录 v1 与 v2 名字，避免 v2 链路 bag 里没有检测结果 |
| `bag_split_mb` | `500` | 单卷上限（MB），`rosbag record --split --size` 自动切换 |
| `bag_keep_days` | `1` | 启动时删除更早的日期目录；1 = 只留今天 |
| `bag_stop_timeout_s` | `5.0` | 停止时等 rosbag 写完尾部索引的秒数，超时才强杀 |
| `pallet_size_out` | `/pallet_servo_v2/pallet_size_mm` | call 0 实测尺寸的 latched 话题；检测器订阅后**同一运行里**用它覆盖 `target_mm`。空串 = 不发 |
| `candidates_key` | `latest_box_candidates` | 邻居路线读的**候选列表**键（`NodeBoxObs` 写）。只有 `mode == "neighbor"` 的档用它；托盘路线一个字都不看 |
| `pallet_size_fixed_mm` | `[1000, 1200]` | ★ **伺服的台面尺寸，优先级最高**，压过下面整条「检测器上报 > call 0 记录 > `pallet_size_mm`」链。按 **(W, H)** 写（与 `pallet_size_mm` 的 `[短, 长]` **顺序相反**）。同一份值还会发到 `pallet_size_out` 保证检测器与伺服同源。**给 `[]` 关掉**，退回旧链路 |
| `pallet_size_mm` | — | 台面 `[短, 长]` 毫米。**只在 `pallet_size_fixed_mm` 关掉时**才轮到它（那时链路是：检测器上报 > call 0 记录 > 本参数 / `config_path`）。pallet 档会等尺寸，neighbor 档不需要尺寸 |
| `max_dt_s` | `0.05` | 配对允许的最大时间戳差（秒）。**只认正有限值**，否则退回默认值并记 WARNING |
| `pair_cache` | `5` | **每侧**留几个观测供配对。**不是窗长** —— 与 `window` 是两个独立旋钮。⚠️ **回溯窗口 ≈ `pair_cache` × 伺服 tick 周期**（10 Hz 下 `5` ≈ **500 ms**），**必须大于托盘检测器的端到端延迟** |
| `window` | `3` | **成对序列**上取几对做平均（1 = 不平滑）。⚠️ 默认值与槽位配置无关，只影响平滑；板子上当前也是 `3` |
| `agg` | `mean` | `mean` / `median`，**只作用在箱子四角上**（托盘一律走刚体平均）。拼错退回 `mean` 并记 WARNING |
| `stale_s` | `0.0` | **延迟闸不是存活闸**：配对成功后这一对有多旧算陈旧（0 = 不查）。判在**去重之后**，所以某路停摆**永远轮不到它**（那时走的是 `None`）|
| `live_timeout_s` | `5.0` | **存活闸**：某一侧多久没有新观测就记一次 WARNING（0 = 不查）。认不了的值退回默认值 —— 负值若按 0 处理就是静默关掉唯一的检测器存活告警。补的是 `None` 那两种含义**不可区分**的那一格 |
| `config_path` | 见说明 | 标定结果路径。默认值按**源码位置**解析成 `<仓库根>/config/pallet_tag.yaml` 的**绝对路径**，与进程 CWD 无关。**只有 `pallet_size_mm` 没写时才会去读它** |
| `K` | — | 内参，**嵌套 3×3**（平铺 9 个数启动时就报配置错误）。**规则只有一条：没写就实时读 `/camera/color/camera_info`；写了就一个话题都不碰，用写的那个。** 写死值不是兜底、是**钉死**——真机上**别写**（写死的值换分辨率/换相机就过期，而那是静默的错），离线复现与单测**写**（结果不取决于跑在哪台机器上）|
| `D` | — | 畸变系数，同上（相机给了就用，都没有 = 无畸变）|
| `use_distortion` | `true` | 投影带不带畸变。认字符串布尔 `true/false`、`1/0`、`yes/no` |
| `image_size` | — | `[w, h]`；有了就顺带查"整条边跑到图外"。规则同 `K` |
| `camera_frame` | `camera_color_optical_frame` | 相机光学帧名 |
| `camera` | `camera` | 向哪个相机命名空间要内参（`camera_info`）；只在 `K` / `D` / `image_size` **没给全**时才去问 |
| `base_frames` | `[base_link, base_link_lb, base_footprint]` | 基座帧候选，按顺序试 |
| `T_cam_base_param` | — | 显式 4×4，TF 查不到且缓存失效时用 |
| `log_every_n` | `30` | 正常帧的节流日志间隔 |
| `dump_on` | `["reject", "warn"]` | 异常帧落盘条件；`"all"` = 全量 |
| `dump_dir` | `log/pallet_servo_dump` | 落盘目录；给空则关闭 |
| `dump_min_interval_sec` | `1.0` | 两份 dump 之间至少隔多久 |
| `diag_period_s` | `2.0` | 诊断心跳周期（秒）；`0` = 关闭心跳（尝试摘要仍写） |
| `diag_log_path` | 空 = 托盘记录同目录 `pallet_servo_v2_diag.jsonl` | 诊断 JSONL 路径；`off` / `-` = 关闭 |
| `overlay_out` | `/pallet_servo_v2/overlay` | 叠加图发到哪个话题；**给空串 = 完全不发图**（省 CPU、省带宽）|
| `overlay_period_s` | `0.2` | 发图降频周期。伺服 10 Hz，图 5 Hz 就够看，而画图 + 编码不便宜 |
| `image_color` | `/camera/color/image_raw` | 叠加图的底图话题 |
| `servo_error_out` | `/pallet_servo_v2/dis` | 伺服误差发到哪个话题；**给空串 = 完全不发**（连发布器都不建）。它是**功能性输出**（下游控制器要用），默认**开** |
| `servo_error_heartbeat_s` | `1.0` | 无效帧的重发周期（秒）。`0` = 只在内容改变时发 —— 但要接受"停摆了就彻底没消息"，那与"话题名写错了"分不开 |

### 箱高修正（`box_plane_height_mm`）

参考边在托盘台面（`z=0`）上，而箱子四角是被抬起来看的。每帧先用**四角**和
`T_cam_pallet` 做一次极轻的反算（实测 < 0.1ms）：四角 `undistort` 成射线 → 与
`D − h` 的箱顶平面求交 → 沿法向落回台面 → 重新投影，再与参考边比。**没有 `quad`
的观测不修正**（AABB 的底边本来就贴台面）。

* 全局默认 `250mm`；`box_types` 每个槽位可用同名字段覆盖（两层码放：第二层 `500`）。
* 启动日志会打印全局值与所有按档覆盖值；它们只取一次，之后**每帧不再重算**。
* 修正失败（四角退化、平面在相机后、自交等）**原样使用旧四角**，并在日志
  （每 `log_every_n` 帧）与叠加图 HUD 的 `高度修正=跳过(原因)` 里写明原因。
* 邻居路线不读这个参数。

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

### 参考边的三种写法

| 写法 | 含义 |
|---|---|
| `y=<毫米>` / `x=<毫米>` | 与同名轴平行的绝对线 |
| `y=0` / `y=H` / `x=W` / `x=0` | 台面矩形的四条边，跟着当前台面尺寸走 |
| `y=0.3H` / `x=W-100` | **相对尺寸表达式**：支持 `W` / `H`、数字、括号与 `+ - * /`；`0.3H` 这种隐式乘法也认 |

绝对写法和表达式**都可以落在台面外**（2026-10-26 起取消范围检查 —— 参考边是
"想对齐的位置"）。启动时只查**写法**与**两条不平行**；表达式在每一帧按当前
`W/H` 求值。

⚠️ **当前尺寸从哪来**：默认是 **`pallet_size_fixed_mm`**（`[1000, 1200]`，现场口径，
2026-11-19 操作员裁定）。把它设成 `[]` 才退回旧链：检测器每帧上报的 `size_mm`
> **call 0 记录**里的 `pallet_size_mm` > 参数/标定文件（离线/测试兜底）。

固定值的意义是**检测器与伺服同源**：同一份数既用来投参考边（`x=W` / `y=0.2H`），
也经 `pallet_size_out` 发给检测器建 `T_cam_pallet`。分成两套数时会静默出现
「青框比托盘大一圈、方向还拧着」。

为什么现场越来越倾向用表达式：`x=W-100` 只表达"离右边 100mm"，换托盘尺寸时
不用跟着改；但**表达式会跟着尺寸变**，如果某条边必须钉死在绝对位置，就继续写
`x=950`。两种写法可以混用（第 1 条绝对、第 2 条相对都合法）。

`ref_edges` 四条边的物理含义（**换了建系顺序这张表整体变**，见 §3 第 2 条）：

| 值 | 物理边 | 值 | 物理边 |
|------|------|------|------|
| `"y=0"` | **下边** | `"x=0"` | 左边 |
| `"y=H"` | 上边 | `"x=W"` | **右边** |

⚠️ **摆放约定（2026-09-24 操作员确认）**：台面 1200×1000 mm，**1200 是长边、1000 是短边**。
摆放时 **短边（1000）与机器人平行**，即**长边（1200）垂直于机器人正面**；
`origin` 取台面的**左下角**；按当前尺寸口径 `size_mm=[短, 长]=[1000,1200]`，
**`W=1000` 沿 `E1`（短边）、`H=1200` 沿 `E2`（长边）**，`e1 × e2 = normal` 指向相机。

⚠️ **托盘摆反 90° 不会报错** —— 参考边会取到**别的物理边上**，而三个数照样算得出来，
伺服照着错的参考边把箱子送过去。现场唯一能看出不对的是**人眼**在叠加图上
（`/pallet_servo_v2/overlay`）看青框贴不贴托盘。

### 箱型 / 槽位 / 初终阶段：`box_types`

`box_types` 每个箱型一张槽位表，**槽位数量与编号独立**：

```json
[
  {"box_type": 0, "name": "默认箱型", "slots": [
    {"slot": 1, "mode": "pallet",
     "ref_edges": ["y=0", "x=950"],
     "initial": {"ref_edges": ["y=0", "x=950"]},
     "box_plane_height_mm": 250},
    {"slot": 2, "mode": "neighbor", "side": "right",
     "ref_edges": ["y=0", "x=580"],
     "e_right_bias_px": 0.0}
  ]},
  {"box_type": 1, "name": "大箱", "slots": [
    {"slot": 1, "mode": "pallet", "ref_edges": ["y=0", "x=1200"]}
  ]}
]
```

- **`ref_edges` 是 final（`stage=0`）的待伺服边**；`initial.ref_edges` 是
  **initial（`stage=1`）的覆盖**，没写就回落到槽位级 `ref_edges`。
- 阶段覆盖只认 `ref_edges` / `box_plane_height_mm` / `e_right_bias_px` /
  `e_bottom_bias_px`；**`mode` / `side` / `normal_rot_deg` 是槽位属性**，
  不随初/终阶段变化（写进阶段覆盖会在启动时报配置错误）。
- 槽位必须从 **1 连续编号**；每个箱型的数量可以不同，`slot` 只在当前箱型内编号。
- **旧板子的 `ref_edges` + `servo_slots` 仍兼容**，等价于「箱型 0、只有 final」；
  但**不能与 `box_types` 同时配置** —— 两份同时存在启动即报配置错误。
- `mode` 只认 `pallet` / `neighbor`，`side` 只认 `left` / `right` / `none`
  （**画面口径**），写错启动时报配置错误。
- `e_right_bias_px` / `e_bottom_bias_px`（可选，默认 0）：该槽位**最终输出**的
  固定偏置，`输出 = 算法原始值 + bias`。加在写黑板 / 发话题 / 出叠加图之前，
  所以 `latest_servo_error`、`/pallet_servo_v2/dis`、叠加图看到的是同一个值。
  ⚠️ 它是**目标位置偏置**，不是误差修正：控制器把 `e_*` 驱到 0，所以
  `e_right_bias_px = -15` 会让机械臂停在"原始差 +15px"的位置上。
- `normal_rot_deg`：`[roll, pitch, yaw]`（度，应用顺序 `Rz @ Ry @ Rx`），切到
  这一档时把 **call 0 记录的 base_link 法向**旋转后再交给检测器。机器人换到
  托盘对面时 yaw 填 `180`；俯仰/横滚变了也要填。缺省 `[0,0,0]`。

### `neighbor_enabled`：暂时禁用邻居路线

`neighbor_enabled: false` 时**所有槽位都按 `pallet` 跑**，但 `box_types`
里的邻居配置（`mode`/`side`/偏置/法向旋转）**原样保留** —— 重新设成 true
就恢复，不用改表。

当前两层码放（`haicheng_box_internal`）就是 `false`，所有箱子伺服托盘上的
固定参考边。启动日志会点名哪些档被改写；如果这些档的 `ref_edges` 还是邻箱
占位 `["y=0","x=0"]`，启动会发 ERROR，且**这些档的激活会被拒绝** ——
上真机前必须改成各自的固定边。

### 服务：`slot` / `box_type` / `stage`

**节点启动时是「未激活」** —— 没 call 过之前一帧有效数据都不出，话题上发的是
`valid=false` 的「伺服未启动」。2026-11-13 起服务一次传三个数：

```bash
rosservice call /pallet_servo_v2/slot "slot: 0"                          # 观测托盘
rosservice call /pallet_servo_v2/slot "slot: 1"                          # 箱型 0 第 1 档，最终
rosservice call /pallet_servo_v2/slot "slot: 1, box_type: 0, stage: 1"   # 箱型 0 第 1 档，初伺服
rosservice call /pallet_servo_v2/slot "slot: 1, box_type: 1"             # 箱型 1 第 1 档，最终
rosservice call /pallet_servo_v2/slot "slot: -1"                         # 退出本次码放
```

* `box_type` 默认 **0**；旧调用只写 `slot` 时等价于 `box_type=0, stage=0`。
* `stage` 只有 **0=final（默认）** / **1=initial**；写别的值直接拒绝且不改状态。
* `slot=0` 是**阻塞调用**：只等托盘检测，采 `pallet_record_samples` 帧
  （最少 `pallet_record_min_samples`），按 `pallet_record_max_spread_mm` /
  `_deg` 检查稳定性，把 **base_link 系**的 `D_mm`、单位法向、以及托盘尺寸
  写进 `pallet_record_path`。成功才 `ok=true`；超时/不稳/写盘失败都
  `ok=false` 且**不覆盖旧文件**。下一次 `slot=0` 重新采样并覆盖。
  * 记录写成功后无条件发一次 `/pallet_servo_v2/pallet_record_epoch`（latched
    `Int32`），检测器据此重读**整条**记录。
  * **失败/超时会自动退回 `slot=-1`**：停在 0 会让检测器一直按观测模式跑。
* 观测期间检测器收到 `detect_mode=3`：**照常出托盘位姿，但忽略文件里的旧记录**，
  走 TF + 深度自检。
* `slot=-1` 是退出，回到未激活并停检测；记录文件保留。
* **越界 / 未知箱型 / 该箱型没有这个槽位 / 非法 stage** → `ok=false`
  **且不改当前状态**。

### 记录文件里的"已码放槽位"

每次成功 call **`stage=0`（最终伺服）**，节点除了切档，还会把这一档写进
**同一个** `pallet_record_path`。记录按**箱型**分表：

```json
"placements_by_box_type": {
  "0": {
    "placed_slots": [1, 2],
    "placed_count": 2,
    "updated_at": "2026-11-13T16:34:16+0800",
    "slots": {
      "1": {"box_type": 0, "box_type_name": "默认箱型", "slot": 1,
            "stage": 0, "stage_name": "final",
            "called_at": "2026-11-13T16:34:10+0800",
            "called_at_epoch": 1762158850.1, "call_count": 1,
            "mode": "pallet", "side": "none",
            "ref_edges": ["y=0", "x=950"],
            "box_plane_height_mm": 250.0,
            "normal_rot_deg": [0.0, 0.0, 0.0]}
    }
  }
}
```

* **只有最终伺服算已码放**；`stage=1`（初伺服）只记日志，不写 `placed_slots`
  —— 最终伺服失败时不能提前把槽位标成已占。
* 不同箱型各有一张表：箱型 B 的 1 号位不会影响箱型 A 的 1 号位。
* 重复 call 同一档只更新 `called_at` / `call_count`，`placed_slots` 不重复。
* **call 0 成功会用一份全新的 record 覆盖文件**，因此上一托盘的
  `placements_by_box_type` 自动清空；call 0 失败则旧文件原样保留。
* 没有 call 0 记录（没有文件）时，call 档位只切档、不创建位置记录文件。
* 写盘失败只记 WARNING，不影响伺服；内存里的记录已经更新，下一次 call 会重试。
* 老记录里的扁平 `placed_slots` / `placements` 只按**箱型 0** 读取。

### 下一个槽位推算脚本（`next_slot.py`）

拆垛侧每次拿到手里的箱子后，可以调这个**只读**脚本，按横/竖方向返回下一个
应码放的槽位（不修改记录文件；真正写入靠伺服 call 档位）：

```bash
# 横：3 → 2 → 5 → 8 → 7 → 10（箱型 0）
python3 skills/atomic/perception/pallet_servo_v2/next_slot.py \
    --orientation 横 --box-type 0 --record log/pallet_servo_pallet_record.json
# stdout: 3（机器可读；解释信息在 stderr）

# 不同箱型槽位数不同时，带 --board 按该箱型的槽位表过滤顺序
python3 skills/atomic/perception/pallet_servo_v2/next_slot.py \
    --orientation 横 --box-type 1 --board orchestration/scenarios/scenarios_internal/pallet_servo_real_v2/board.json

# 竖：1 → 4 → 6 → 9
python3 skills/atomic/perception/pallet_servo_v2/next_slot.py --orientation 竖 --box-type 0

# JSON 输出（含已占位、剩余序列）
python3 skills/atomic/perception/pallet_servo_v2/next_slot.py --orientation h --box-type 0 --json
```

* 已占位来自 `placements_by_box_type[<box_type>].placed_slots`；老记录退回扁平
  `placed_slots` / `placements`（只当箱型 0）。
* 该方向全满时**退出码 1**；记录缺失/损坏退出码 2；方向写错退出码 3。
* 默认记录路径 `<仓库根>/log/pallet_servo_pallet_record.json`，可用
  `--record` 覆盖。
* `--board` 可选：给了就只在该箱型 `box_types` 实际配置的槽位里选，
  避免算出该箱型不存在的位置（箱型缺槽 / 方向没有槽位时退出码 1）。
* 横/竖是**两张独立顺序表**（`next_slot.PLACE_ORDERS`），一边满了不影响另一边。

⚠️ **服务从 1 数，槽位表从 0 数** —— `slot=2` 用该箱型 `slots[1]`。

⚠️ **整张 `box_types` 在启动时一次校验完** —— 槽位不连续、pallet 档缺
`ref_edges`、阶段覆盖里写了 `mode` 之类，启动时就报，不留到轮到它才发现。

⚠️ **两个箱子同时在视野里时选箱可能挑错**：`latest_box_obs` 只有**一个**箱子，
选箱判据在**生产者**那边（`carton_obb`：到画面下方中心的距离 + 面积 + 中心深度，
见那个包的 README）。箱位 1 的没搬走、箱位 2 的已放下时，可能挑到旧的那个，
而**三个数照样算得出来**（只是基准是另一个箱位）、**不报错**。靠流程保证
「先搬走再放」。

### 诊断日志（`pallet_servo_v2_diag.jsonl`）

现场排查“检测器有数据但伺服不动”时，只看两路检测器心跳是不够的。伺服节点每
`diag_period_s`（默认 2s）写一条 **JSONL 心跳**，并在每次换档/退出时写一条
**尝试摘要**；默认路径：

```text
<托盘记录文件所在目录>/pallet_servo_v2_diag.jsonl
```

可用参数 `diag_log_path` 覆盖；写 `off` / `-` 关闭。

`heartbeat` 包含：

```text
slot / slot_mode / active / ticks / frames
counts: pair_ok, pair_none, pair_reject, servo_ok, servo_reject, dis_valid, dis_invalid
last:   pair_dt_s, pallet_stamp, box_stamp, n_pairs, e_bottom_px, e_right_px, theta_rad, reason
input:  托盘/箱子是否有值、版本号、stamp、距最后一次观测的秒数
feedback: 当前 feedback_message（拒帧原因在这里）
```

判读顺序：

1. `pair_ok` 一直是 0 → 两路时间戳/配对问题（看 `input.*.stamp` 与 `pair_dt`）；
2. 有 `pair_ok`、没有 `servo_ok` → 几何被拒，看 `last.reason`；
3. `servo_ok` 和 `dis_valid` 都在涨 → 视觉/伺服已出数，问题在控制器或行为树。

`NodePalletObsV2` / `NodeBoxObsV2` 另有 5s 心跳：`收到 / 写入 / 最近stamp / 反馈`。

## 叠加图（`overlay_out`）

发到 `/pallet_servo_v2/overlay`（`sensor_msgs/Image`），底图取 `image_color`，
按 `overlay_period_s` 降频。

**每一层都是「有就画、没有就不画」—— 不要求两路检测都有结果。** 三个数要托盘与
纸箱**都出了结果、而且配得上对**才算得出来，而排查时最需要的恰恰是"只有一边有
结果"的那些帧：那种帧以前整张图都不发，于是操作员分不出是**这一路检测没出东西**
还是**叠加图这条链坏了**。现在缺哪一路就不画哪一层，HUD 上写明缺在哪一步；
**未激活**（还没 call 箱位）和**被拒的帧**（几何退化）也照出图。

| 颜色 | 画的是什么 | 什么时候没有 |
|------|-----------|------------|
| **黄线** | 托盘的两条参考边，**标签就是你写的那两条原文**（`y=0` / `x=1200`）—— 现场照着图核对"我配的边对不对"用的就是它 | 托盘位姿、台面尺寸、`K` 三样不齐，或者还没 call 箱位（没有"这一帧用哪一组"可言）|
| **青线** | 托盘台面矩形（四条边）| 同上 |
| **绿线** | 有 `error`（三个数算出来了）时是箱子的**底边与右边**；没有 `error` 时是**箱子四角的四条边**（没有"哪两条参与伺服"的语义，要看的是整圈框歪不歪）| 上游没给箱子四角 |
| **灰线** | 纸箱外接框（四角的 AABB，**只是输入**）| 黑板上的箱子观测缺 |
| **蓝箭头** | 参考边的内法向 | **只有算出三个数才有** |
| **红线** | 垂距线（箱子边的中点 → 参考边上的垂足）；**紫圆** = 垂足 | 同上 |

图左上角还有一个 **HUD**（三行白字）：算出三个数时是三个量的读数 + 出处
（`pallet_v` / `box_v` / `t_cam_base` / `box`）+ `size=` 与 `from_upstream=`；
**没算出来时**第一行是 `⚠ 没有伺服差：<原因>`，第二行是
`托盘 有/缺 / 纸箱 有/缺`。**它是拿来与 `rostopic echo /pallet_servo_v2/dis` 对读的** ——
所以几行里的数与话题消息**同源同单位**（`e_*` 像素、**`theta` 弧度**、
2026-10-08 操作员裁定；从前 HUD 印度，与话题的 `theta_rad` 差 57.3 倍，
"图上是对的"就成了一句空话）。

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

## 2.1 接进行为树

> **真机上现成的场景在 `orchestration/scenarios/scenarios_internal/pallet_servo_real_v2/`** ——
> 那里托盘走**视觉托盘检测器**（`NodePalletObs` 订阅 `/pallet/detection`），箱子走
> `NodeBoxObs`（订阅 `/box/detection`），**两路都是自动的**。
> 本节下面这段是**纯离线**的骨架：托盘和箱子都由那份 JSON 提供。
> 前置条件、`K` 从哪抄、跑之前要改什么，都在那个场景的 `README.md` 里。

下面的参数块必须与现场标定和取点结果一致：

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
   | 4 | （老路径）`config/apriltag_tags.yaml` | 当前 id 1–4 / size 0.06，要填现场标签的真实 id 与尺寸 |
   | 5 | （老路径）`config/pallet_tag.yaml` | **必须重标**（见下）——老路径下 `pallet_size_mm` 不给就从这里读。**新链路不读这个文件**：默认尺寸来自 `pallet_size_fixed_mm`；把它关掉时才依次看检测器上报的 `latest_pallet_size_mm` 与 call 0 记录 |
   | 6 | **`pallet_servo_msgs_v2` 编译过** | 只影响**发话题**：没编过时节点记一条 WARNING 说"只写黑板，行为树之外的控制器拿不到误差"，其余照常 |

   **`NodeBoxObs` 的三个参数就是接到真机的全部动作**：`topic`（默认 `/box/detection`）、
   `msg_type`（默认 `box_detection_msgs/BoxDetection`，运行期按全名解析，**换话题/换类型
   都只改参数**）、`box_key`（默认 `latest_box_obs`，要与本节点的 `box_key` 对得上）。
   它**不 import 那个 catkin 包**（按字段名取），所以 LeTools 不多一条构建依赖。

   ⚠️ **`valid == false` 的帧一个字节都不写黑板。** 上游 `.msg` 的注释原话：
   「`false` = 四角就是原始 YOLO 框，别拿去做伺服」——那时箱子旋转没恢复，
   拿它算出来的 `theta` 是**错的**，而四角看着规规矩矩是个矩形、`e_bottom_px` 也
   像模像样。所以这种帧被跳过，下游继续用上一帧的好值。

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
   > §6.2 那个"写错了不报错、只是伺服错了边"的陷阱被填掉了。

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
| `pallet_frame_v2/README.md` | **配对与平滑**：两个观测按图像时刻配对、五种 reject、`None` 的两种含义、`spread_*` 口径 |
| `orchestration/nodes/node_pallet_obs.py` | 托盘观测生产者（`latest_pallet` + `_stamp`），订阅 `/pallet/detection` |
| `pallet_pose/README.md` | `base_link` 老路径的托盘位姿模块（`latest_pallet` 的另一个生产者）|
| `orchestration/nodes/node_box_obs.py` | 箱子观测生产者（`latest_box_obs`），订阅 `/box/detection` |
