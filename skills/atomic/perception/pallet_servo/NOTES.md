# 托盘伺服误差 —— 工程笔记

`README.md` 是接口与跑法。**这里放踩过的坑、评审记录、日志与 dump 的细节。**
出问题、要改这个模块、或者要接真机时再看。

## 目录

- [1. 链路与文件](#1-链路与文件)
- [2. 人工闸门记录（2026-09-20）](#2-人工闸门记录2026-09-20)
- [3. 四个坑](#3-四个坑)
- [4. 离线隐含 `T_cam_base = 单位阵`](#4-离线隐含-t_cam_base--单位阵)
- [5. 点击顺序：为什么必须固定](#5-点击顺序为什么必须固定)
- [6. 两条路径的托盘系为什么不一致](#6-两条路径的托盘系为什么不一致)
- [7. 日志与 dump](#7-日志与-dump)
- [8. 已知边界](#8-已知边界)
- [9. 测试怎么跑](#9-测试怎么跑)
- [10. 排查用的原文与出处](#10-排查用的原文与出处)
- [11. 箱子观测节点 `NodeBoxObs`](#11-箱子观测节点-nodeboxobs)
- [12. 2026-09-21 修的三处静默错](#12-2026-09-21-修的三处静默错)
- [13. 真机清单（2026-09-21）](#13-真机清单2026-09-21)

---

## 1. 链路与文件

```
NodePalletObs    →  latest_pallet + _version + _stamp ┐   ← 视觉检测器（新，默认）
NodeBoxObs       →  latest_box_obs + _version         ├→ NodePalletServo
NodePalletPose   →  latest_pallet + _version          ┘   ← AprilTag 老路径
                                                    （pallet_frame: "base_link"）
                                                             ↓
                                                  latest_servo_error + _version
```

新链路里两个观测按**输入图像的采集时刻**配对再平滑
（`skills/atomic/perception/pallet_frame/`）；**`base_link` 老路径绕过配对**，
与改动前逐位一致。

离线时左列两个源由 `NodeInjectServoInput` 冒充（它写的就是**真实链路里的那两个
键**，所以伺服节点一行不用改）。

| 文件 | 作用 |
|---|---|
| `algorithm.py` | 纯函数：投影、取边、垂距、角度差。**零框架依赖**，可脱离环境回归 |
| `skill.py` | `PalletServoSkill`，接进技能层；不接 `hardware`、不查 TF |
| `orchestration/nodes/node_pallet_servo.py` | 行为树节点：读黑板两个输入 → 写 `latest_servo_error`；TF 解析、版本门禁、分级日志、异常帧 dump |
| `orchestration/nodes/node_pallet_obs.py` | **托盘观测生产者**（新链路）：订阅 `/pallet/detection` → `latest_pallet` + `_version` + `_stamp` |
| `skills/atomic/perception/pallet_frame/` | **配对与平滑**：两路观测按图像时刻配对 + 成对序列刚体平均（纯函数 + 一个有状态窗）|
| `orchestration/nodes/node_inject_servo_input.py` | **离线模拟源** |
| `apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py` | tkinter 点点工具：两阶段 + 目视闸门 + 出对比图 |
| `apps/test_camera_internal/pallet_servo_sim/render_overlay.py` | 画图 / 数值自检（出图前先自检，别让人看一张画错了的图）|

**`T_cam_base @ pose6d_to_matrix(pose)` 这笔几何在两个地方各算了一次**：技能层算给
算法用（`PalletServoSkill.on_execute`），节点算给 dump 用
（`NodePalletServo._maybe_dump`）。dump 回归网把它们的结果钉在一起，所以不违反
"节点不加工几何"；**但那个"钉住"只在 `T_cam_base` 不是单位阵时才成立** ——
单位阵下乘法可交换，谁把技能层的组合顺序调过来，两条路径照样给出同一组数。

所以 `test_node_pallet_servo.py` 里专门有一条**非单位阵**的用例
（`test_node_dump_pins_the_composition_order_with_a_non_identity_t_cam_base`）：
取一个**绕 x 转 5°、带平移**的 `T_cam_base`，断言 dump 里的 `T_cam_pallet` 等于
`T_cam_base @ T_base_pallet`、与**交换顺序**的结果**明显不同**，且重跑出来的三个量与
技能层**逐位一致**。**改这里的组合顺序，那条会红。**

## 2. 人工闸门记录（2026-09-20）

> **闸门①：参考边投影贴合托盘 —— 通过（带保留）。**
> 操作员的话是「稍微歪了点」但「先用这个」。偏差来源**是深度在托盘角上不可信，
> 不是算法错**：`5_test` frame 0 上四个点击角点到拟合平面的距离是
> `−31.5 / +32.1 / −34.1 / +33.4 mm`（正负交替，"拧"着），左下比右下近 **128 mm**
> （两者都在近边，本该几乎等高，约等于一块木托盘的厚度），对边长度也不等
> （`1054 / 992`、`1152 / 1178 mm`）。木托盘角上深度不连续（板条缝、台面与侧面
> 交界），取深度取到了别的东西。工具的正交性警告阈值是 5°，而这类错位表现为
> "边变长"（实测只到 3.37°），**所以没有报警**。
>
> **影响**：参考边是伺服的基准，它整体偏移会**平移伺服的零点** —— 离线跑出来的
> 两个 `e` 的**绝对值带着这个常数偏置**，而**符号、变化趋势、控制律的行为不受
> 影响**。**这条不传染真机路径**（真机的托盘位姿走 tag 反算，不经过点点）。
>
> **闸门②：三个量的符号与方向 —— 通过（带保留）。**
> 蓝箭头（内法向）**指向托盘内部** ✓、红线（垂距）方向与箭头**一致** ✓。
> 但**极性是暂定的**：操作员原话「这个正负先暂定这样，后续可能要反转」。
> 闸门②那一次的实测值（`5_test` frame 0）：`e_bottom = −33.9 px`、
> `e_right = +67.4 px`、`theta = −0.30°` —— 绝对值带闸门①的偏置，这一关只看符号与方向。

## 3. 四个坑

1. **单位是分裂的。** 托盘系内部的几何量一律**毫米**（台面 `[0, W] × [0, H]`，
   `z = 0`），而 `T_cam_pallet` 的平移是**米**（与 `PalletFrame.to_matrix()`、
   框架 `Pose6D` 一致）。投影前那一步换算写错了会**静默差 1000 倍**，而且结果仍然
   是一对"看着像像素坐标"的数。

2. **箱子四角的顺序就是边的语义，点错不会报错。** 凸环自检只挡"自交/凹"：整环反着走
   或换个起头都会被放行，而底边已经取到别的角上了，三个量照样算得出来。

3. **直线无向，`theta` 折进 (−90°, 90°]。** 一条边是 p0→p1 还是 p1→p0 是同一条线；
   不折的话两个视觉上重合的边会算出 178° 的"角度差"。所以**"转 180°"是同一个角**
   —— 下游别指望用 `theta` 分辨"箱子掉了个头"，那要靠别的量。

4. **`use_distortion` 必须与算像素的那一侧一致。** 伺服若在原图（带畸变）上算像素，
   投影就必须带 `D`；反之不带。两边不一致时**不会报错**，只是默默分叉
   （实测：开 135.918 px / 关 137.000 px，差得不多但确实是两个数）。离线工具把这一帧
   实际用没用畸变如实写进 JSON（`use_distortion = D is not None`），就是给下游对参数用的。

   **还有一条不影响输出但要知道**：离线点点的托盘位姿是**相机系**的，写进黑板时冒充
   `base_link`，由 `T_cam_base = I` 抵消。三个量全在像素空间，base_link 这个约定在结果
   里不露面，所以不影响输出；但它意味着**离线数不能反过来当"托盘在机器人坐标系里的
   位置"用**。

## 4. 离线隐含 `T_cam_base = 单位阵`

**离线路径拿到的是「声明」的单位阵，不是「回退」的单位阵 —— 所以这里没有失效模式。**
离线点点的托盘位姿是**相机系**的，`pallet_frame` 走默认值 `"camera"`；这个模式下节点
**根本不构造** `TfCamBaseResolver`（`node_pallet_servo.py:1008-1011` 只在 `base_link`
时才建它），`_resolve_t_cam_base()` 直接返回声明的单位阵，`t_cam_base_src` 恒为
`identity(相机系位姿)`。

因此**离线这条路不会**"在一台有 ROS master、且 TF 真能解出
`camera_color_optical_frame ← base_link` 的机器上，被那个真实变换多乘一次" —— 那一级
压根不存在。**也不用再去动 `base_frames`**（早先"把它指到一个不存在的帧名逼它走回退"
的建议，防的是一个已经不可达的路径）。

**要确认 TF 的是老路径（`pallet_frame: "base_link"`）**：只有它才构造解析器，按
**TF → 显式参数 `T_cam_base_param` → 单位阵** 逐级解析，而 TF 查不到时是**静默**退回
下一级（只有一条 WARNING）。在那条路上，如果黑板上的位姿其实是相机系、TF 又真能解
出来，三个量就会变成一个完全没有意义的值而没有任何报错 —— 唯一能看出这事的地方是
日志行里的 `t_cam_base=` 字段（在**正常帧的节流日志**里，默认 DEBUG，要看就
`config/log_config.yaml` 开到 `DEBUG`；或者看 `log/pallet_servo_dump/` 里落盘的
`"t_cam_base_src"`）。

**跑老路径前确认 `t_cam_base_src` 是 `"tf"` 或 `"param"`** —— `"identity"` 说明 TF
一级都没查上，而老路径上黑板里的位姿**必须**是 `base_link` 系才对。真机走的就是老路径，
它本来就**该**走 TF；离线那条路则不碰 TF。

## 5. 点击顺序：为什么必须固定

操作员点的 `[右下, 左下, 左上, 右上]` 会被工具**反向走一圈**重排成
`[左下, 右下, 右上, 左上]`，然后 `origin = points[0]`、`e1 = points[1]−points[0]`、
`e2 = points[3]−points[0]`。

不是口味问题，是**唯一能让托盘系成立的方向**：`PalletFrame` 把 `[e1 | e2 | normal]`
当**旋转矩阵**用，要求 `e1 × e2 = normal`；而托盘从上方看时，图像上**顺时针**的环序
给出的恰好是 `−normal`，`det = −1` —— 那是**镜面反射，不是旋转**。换了哪个相邻边当
`e1/e2` 都一样，全都反号。

⚠️ **镜面是静默的**：`matrix_to_pose6d()` 内部走 `Rotation.from_matrix`，它会把镜面
**投影掉**，还原回来就成了另一个系（实测：直接算投影落在托盘下边上，往返之后跑到了
托盘外面）。工具在建出 `T_cam_pallet` 后会**自检手性**（`det(R)` 必须落在 +1 一侧），
这是那个静默镜面**唯一能在源头拦下**的地方。

### 仓库里那句「点击顺序无所谓」是错的

`apps/test_camera_internal/pallet_calibration/pick_pallet_corners.py:14` 写着
「点击顺序无所谓，`algorithm.order_quad()` 会按质心角度重排」—— **这句话是错的**：

- `pallet_frame_from_clicks()` **直接用原始点击顺序**（`points[0]` 是原点、
  `points[1]` 定 `e1`、`points[3]` 定 `e2`），`pallet_calibrate.py` 也是把原始点击顺序
  直接喂进去的；
- `grep -rn order_quad` 只有**定义与导出**，**全仓库没有任何调用点**。

也就是说：**现有标定工具一直在默默依赖操作员每次都按同一个顺序点，却在提示里告诉
操作员顺序无所谓。**

**为什么不能用 `order_quad()` 来救**：它按"图像上最靠右的点"起头，而"最右"取决于
**相机视角** —— 相机一动，原点可能在两个角之间跳，托盘系跟着翻。伺服要的是一个
**按物理角定死**的系，所以顺序必须由操作员按物理位置给，不能由图像几何猜。本工具用的
是**固定置换**（第 i 个点去哪，写死），不是按几何猜。

## 6. 两条路径的托盘系（**已于 2026-09-21 修好，本节保留历史**）

> **现状**：标定工具补上了重排（与离线工具共用算法层同一份
> `PALLET_CLICK_PERMUTATION`）+ 手性自检 + 写盘前拒绝镜面，**两条路径现在一致**：
> `"y=0"`=下边、`"x=0"`=左边、`"x=W"`=右边，两边相同。
> **代价是修复前标出来的 `config/pallet_tag.yaml` 一律作废、必须重标。**
>
> 下面记的是**当初为什么不一致**，留着是因为"为什么会错成这样"比"现在对了"更值得记住。

`T_pallet_tag` 里烘的托盘系来自 maduo 当年标定时操作员点的顺序 —— 那次点的是
**顺时针**，建出来的是**左手系**（`pallet_pose_regression.json` 里每帧
`det = −1.000000`）。

**根因**：重排只加在了**离线点点工具**上（`pick_servo_inputs.py`），而**标定工具
`pallet_calibrate.py` 从来没加过** —— 它把原始点击顺序直接喂给
`pallet_frame_from_clicks`，于是顺时针点就得到 `det = −1` 的 `T_cam_pallet`，
再传下去就是镜面的 `T_pallet_tag`。而且它当时还在打印「顺序无所谓，程序会按质心
角度自己排序」——**一句彻头彻尾的错话**（`order_quad()` 全仓库零调用点），
`pick_pallet_corners.py` 里还有同一句错的第二份。

- **下边两边一致**：只是方向相反，而 `line_angle` 是无向的、`inward_normal` 靠台面
  中心定，所以方向反**不影响结果**。
- **但 `x=0` / `x=W` 是反的**：同一组物理边，**离线写 `["y=0","x=W"]`、真机要写
  `["y=0","x=0"]`** —— 写错了**不报错**，只是伺服错了边。

### ⚠️ 真机路径的位姿是镜面，而它过不了 `Pose6D`

`PalletPoseSkill` 用 `matrix_to_pose6d()` 把反推出来的托盘位姿写进黑板，而那个函数
内部走 `Rotation.from_matrix` —— **它会把镜面静默投影掉**。实测：运行时反推的托盘位姿
`det = −1.000000`，经 `matrix_to_pose6d` 往返后 `det = +1.000000`、往返最大元素差
1.576（平移不差）。也就是说 **`NodePalletPose` 写在黑板上的 `Pose6D` 还原不回标定出来
的托盘系**。这是既有已合并代码里的一个静默缺陷，今天无害（`latest_pallet` 没有别的
消费者），**但真机 tag 路径接上伺服时，伺服会吃到这个被镜像的位姿**。

**真机接入前必须三选一**（本方案不擅自改 `pallet_pose`）：

1. 重标一次，按**逆时针**点（代价是重跑标定）；
2. 让 `latest_pallet` 带得起 4×4（契约变更）；
3. 在 `PalletPoseSkill` 里显式拒绝 `det < 0` 的输入，把问题挡在标定阶段。

**几乎零成本的核对**：真机接入时用同一帧分别用 tag 路径与点点路径建系，看 `e1`/`e2`
是否同向。

## 7. 日志与 dump

- **默认跑法下三个量一个都不落盘。** 正常帧与"等输入"的日志是 **DEBUG**，而仓库的
  `config/log_config.yaml` 是 `level: "INFO"` —— 所以 `e_bottom_px` / `e_right_px` /
  `theta_rad` **默认一条都写不进日志**（要看就开 DEBUG）。不落盘的例外只有两条，
  都不是这三个数：被拒的帧有一条带实际值与阈值的 WARNING，以及框架自己每帧一条
  `Skill [pallet_servo] initialized.`（INFO，`skills/base/skill_base.py:30`）。
  后者的量级是 10 行/秒 —— 是框架的统一行为（每个技能都这样），本模块不改它，
  但读日志的人该知道它在那儿。
- **dump 有一个"既没拒也没 warn 的帧不留 dump"的洞。** 设计里说它靠"节流日志里每帧
  都有三个量的值"兜 —— **但那条兜底默认不生效**（见上一条）。要用上那条安全网，
  **得给这次运行开 DEBUG**。
- **dump 默认限流：两份之间至少隔 1 秒，逐字相同的那份不写。** `dump_on` 默认只写
  拒绝帧与带 warn 的帧，落在 `log/pallet_servo_dump/`。这件事非做不可：伺服按 10 Hz
  跑，而"整帧被拒"可以是**持续**状态（托盘太远或太侧时 `edge_too_short` 一直成立，
  箱子观测却每帧都在更新 → 每帧都是新的版本对），不拦就是一帧一份文件
  （≈1.5 KB/份 → 15 KB/s、50 MB/h）外带每份一条 WARNING。被跳过的份数**会写在下一份
  的日志里**（逐字相同几份、离得太近几份），不是无声丢弃。
  - 要"每一帧都留一份"（调试期）：把 `dump_min_interval_sec` 设成 `0`。
  - `dump_on: "all"` 同样受这个节流约束（全量落盘 ≠ 解除限流）。
  - 被拒帧的 **WARNING 仍是每帧一条**（设计 §6.2 写明的契约：拒绝必须带实际值与阈值）。
    默认跑法下现场的量级因此是 **约 21 行/秒**（10 Hz × 技能一条 + 节点一条，外加
    dump 每秒最多一条），不是无上限。
- **dump 整段包在 `try` 里**：dump 是诊断设施，它不该有能力把节点弄死。上游给的类型
  不对（`latest_box_obs` 是 dict、`latest_pallet` 是 list）时取字段会失败 —— 只记
  WARNING（按原文去重，不是每帧一条），**绝不抛**，这一帧的伺服结果不受影响。

### dump 的定位（2026-09-24）

dump 现在只做一件事：**异常帧的完整输入留档**（默认 `dump_on: ["reject", "warn"]`）。
复现那件事交给 **话题 + `rosbag record`** —— 录一段 bag，`rostopic echo
/pallet_servo/dis` 就能看到当时的三个数。

两者的分工：**bag 录的是结果，dump 录的是当时的完整输入**（4×4 矩阵、K、箱子四角），
能喂回 `algorithm.servo_error` 重跑。

⚠️ **只装三个被控量的话题录下来是复现不了叠加图的** —— 叠加图要参考边端点、垂足、
内法向那些诊断量，而话题里没有（那些量只在黑板的 `ServoError` 上）。将来要能用 bag
复现叠加图，得往消息里加字段。

### 单测全绿、真机才撞出来的一个 bug（2026-09-24）

**停摆帧在话题上是每 tick 一条（10 Hz），而设计要的是每秒一条。**

去重键取的是 `reject` 的**原文**，而停摆文案里嵌着 `已经 8.2s` —— **那个数每 tick
都在长**，键于是恒不相等，去重形同虚设。这正是本项目反复踩的那个坑（"告警去重按
稳定键，不按整条文案"），`_check_liveness` 与 `_warn_overlay` 里都防了，**却在
`_publish_error` 里又踩了一遍**。

**为什么 16 条单测一条都没抓到**：那些用例调的是 `_publish_invalid("A")` 这种
**不含读数的短文案**，正好绕开了这一格。真机 `rostopic echo` 上一眼就看出来了
（停摆之后每秒 ~10 条，间隔 0.10s）。

修法与既有的 `_overlay_dedup_key` 同构：`_reject_dedup_key()` 把 `已经 8.2s` 砍成
`已经 <t>s` 再当键（**发出去的 `msg.reject` 仍是原文**，秒数留给人看）。
修后实测间隔 **1.00 秒**。

> **教训**：去重键的用例必须喂**真实形状的文案**（带逐帧在变的读数），
> 喂 `"A"` / `"B"` 只能验到"键变了就发"，验不到"读数在变时键该不该变"。

## 8. 已知边界

- **`MIN_EDGE_PX = 20` 是拍出来的初值，没有数据支撑。** 它兜的是"投影明显退化"，
  不是标定出来的阈值。
- **`latest_pallet` 的参考系在现有代码里是自相矛盾的**（`pallet_pose/README.md`
  §6 第 3 条）：本节点靠显式解析 `T_cam_base` 吸收这一步，不依赖那个矛盾的结论。
- **深度尺度从未被独立标定过**，它若整体偏，会同时影响离线点点算出的托盘位姿
  （进而平移伺服零点）。真机那条路径不受影响。
- **`pallet_pose` 的两个既有问题**（本方案不改，但读这份笔记的人多半会碰到）：
  - `self_check["n_frames"]` 在**生产路径**上其实是 **tag 数**，不是帧数：
    `TagObs.frame` 在 `pallet_calibrate.py` 里传的是 tag id、在测试里传的是帧名，
    于是 `by_frame` 按 tag 分组、`n_frames = len(by_frame)` = tag 数（2）而不是帧数（5）。
    测试里因为传的是帧名，这个键是对的 —— **所以测试永远发现不了**。
  - `pick_pallet_corners.py:14` 那句"点击顺序无所谓"是错的（见 §5）。
- **本模块不做伺服控制器。** 控制律、`e > 0` 到底对应"往哪边推"、要不要对 `theta`
  取负，都是下游控制器的事。

## 9. 测试怎么跑

```bash
cd /data/Real_Downloads/LeTools

# 算法层合成测试 + 技能层单测（不读图、不要硬件/ROS）
python3 apps/test_kuavo_5w_skills/test_pallet_servo.py
#   → OK: 26/26 全部通过

# 既有模块的回归（本方案改了它的标定产物口径：新增 pallet_size_mm）
python3 apps/test_kuavo_5w_skills/test_pallet_pose.py
#   → OK: 8/8 全部通过

# 本模块的节点单测（CI 会跑）
pytest orchestration/nodes/tests/test_node_pallet_servo.py -m unit -q
#   → 88 passed
```

**整目录跑法在本机要加一个 ignore**（2026-09-20 实测）：整目录
`pytest orchestration/nodes/tests/ -m unit` 会在**收集期**中断 ——
`test_node_tag_to_arm_goal.py` import 了本机没有的子模块 `kuavo_humanoid_sdk`
（CI 里有，所以那边没事）。本地正确跑法：

```bash
pytest orchestration/nodes/tests/ -m unit -q \
       --ignore=orchestration/nodes/tests/test_node_tag_to_arm_goal.py
#   → 2 failed, 146 passed
```

那 **2 条失败是既有的、与本方案无关**：`test_node_compute_pick_goal.py` 与
`test_set_back_walk_goal.py` 同样 import 不到 `kuavo_humanoid_sdk`
（`ModuleNotFoundError`）。判断新改动有没有弄坏东西，只看**本模块的用例是否全绿**。

> **做"改回去验旧行为"这类验证（变异测试）时必须先清 `__pycache__`**：
> CPython 判断 `.pyc` 是否过期用的是 `(源文件 mtime, 源文件 size)`，同一秒内改回去
> 可能被当成新鲜字节码，于是**你以为在跑改回去的版本，实际跑的是被变异的**，
> 而且不报任何错。

## 10. 排查用的原文与出处

**报错与日志原文**（按原文去日志里搜，比按描述搜快）：

```text
K 形状必须是 (3, 3)，实际 (9,)          ← 给扁平 9 个数时报这个；"平铺 9 个数"这种写法从来没被接收过
NodeInjectServoInput: 模拟输入就绪：… K=… D=… image_size=… —— 拿去和 NodePalletServo 的 params 对一遍
NodePalletServo:  NodePalletServo 初始化：… use_distortion=… K=… D=… image_size=…
```

**干跑（`--dry-run --tick-once`）的预期输出**——`keys=(…) -> …` 那行是该节点读哪两个键、
写哪个键的唯一书面形态：

```text
node_name = NodeInjectServoInput
node_name = NodePalletServo
node_name = WaitSeconds
[dry-run] 已加载树，根节点: …
NodeInjectServoInput: 模拟输入就绪：…（一行）
NodePalletServo: NodePalletServo 初始化：keys=(latest_pallet, latest_box_obs) -> latest_servo_error …
```

**坏输入为什么只许 FAILURE、不许抛**：py_trees 的 `Behaviour.tick()` **不接**这两个钩子
抛出的异常，抛出去就是**整棵树连伺服节点每帧的日志一起没**。半截 JSON 也接得住 ——
工具的 `write_text` 是**非原子写盘**，tick 撞上写盘就会读到半截文件。

**模拟源所在的 `Parallel` 该怎么收**：源节点持续 `RUNNING` 是仓库既有约定，见
`node_percep.py`。固定时长用 `WaitSeconds`，人工看够了再收用 `WaitForEnter`。

**离线为什么要人工点点**：当前数据（`5_test`）**没有二维码**，没有可用的 tag 反算，
所以上游两份输入只能用人工点点造出来 —— 这正是走这条路的原因。

**设计文档里的位置**（在 `docs/superpowers/`，**已 gitignore、不随代码走**）：
§5.1 黑板版本号约定、§6.2 日志分级与"拒绝必须带实际值与阈值"、§9 第 4 条镜像处置、
§10 "不做的事"清单。

## 11. 箱子观测节点 `NodeBoxObs`

`orchestration/nodes/node_box_obs.py`。这是 `latest_box_obs` 的**第一个真生产者**
（在这之前那个键全文没有生产者，只有离线模拟源与测试在写）。

**上游契约不是我猜的**：定在
`infrastructure/ros_packages/src/ros_vision/box_detection_msgs/msg/BoxDetection.msg`，
发布者 `infrastructure/.../detection_industrial_yolo/box_detection/`（算法在
`skills/atomic/perception/box_frame/`），交接文档 `WORKLOG_box_frame_fit.md` §17。
四个关键字段：`corners_uv`（**四角，顺序就是 `BoxObservation.quad` 的契约顺序**
`右下→左下→左上→右上`，直接填）、`valid`、`source`、`spread_px`。

**三个决定，每个都有理由：**

1. **按字段名取（鸭子类型），不 import `box_detection_msgs`。** 那是个 **catkin
   包**（虽然和本仓库同源，但要单独放进 ROS 工作区编译），硬 import 会让 LeTools
   多一条构建依赖；按字段名取则换个"形状一样、
   包名不同"的消息照样能用。订阅本身需要消息类，所以 `msg_type` 参数给**全名**、
   运行期用 `roslib.message.get_message_class()` 解析 —— **换话题/换类型都只改参数**。
   > 顺带：解析写成**模块级纯函数**（`parse_box_message()`），所以它**能进 CI**
   > ——`apps/test_camera_internal/**` 被 rsync 排除，只有放进
   > `orchestration/nodes/tests/` 的东西才跑得到。

2. **`valid == false` 的帧一个字节都不写黑板。** 上游 `.msg` 的注释原话：
   「`false` = 四角就是原始 YOLO 框，别拿去做伺服」。那时箱子旋转没恢复，
   `theta` 是**错的**，而四角看着规规矩矩是个矩形、`e_bottom_px` 也像模像样 ——
   标准的静默错。跳过它，下游继续用上一帧的好值（与"被拒的帧不写黑板"同构）。
   > **但「没有 `valid` 字段」≠「`valid = false`」**。`vision_msgs/Detection2DArray`
   > 之类根本没有这个说法，当成 false 会把本来能用的 AABB 全拒掉、节点永远不写。
   > 所以判据写成 `has_valid_field and require_valid and not valid`，两条用例互为反面。
   > 这个坑是**变异测试逼出来的**：节点级用例区分不出解析层的默认值，得直接在
   > 解析层断言 `valid is True`。

3. **同一条观测只写一次。** 回调来一条存一条，而 `update()` 每 tick 都跑 ——
   不加这道闸就是"相机 30~40 Hz、tick 10 Hz，每 tick 把同一帧重写一遍"：版本号一直
   涨，下游版本门禁每次放行，伺服拿同一个框白算。`NodeInjectServoInput` 的注释把
   这条写死了（「播完就停手，不要拿最后一条反复写……10 Hz 白算」）。

**`spread_px` 做成了 `max_spread_px` 参数**（默认 0 = 不查）：窗内四角到平均值的
最远距离，超阈值说明这一窗不稳，跳过。

## 12. 2026-09-21 修的三处静默错

三处都是"**不报错、只是算错**"那一类，而且是同一个调研里挖出来的。

### 12.1 标定工具的手性（最严重的一条）

见 §6。改了三件事：把置换提升到算法层（**三处共用一份**）→ 工具内部重排 →
建系后手性自检（`det < 0.5` 的**符号测试**，不是"离 +1 多近"）→ **写盘前拒绝镜面**。

**最后那道闸是整条链的安全网**，也是唯一不依赖"某人记得别删那行重排"的一环：
无论上游哪里出错，只要算出来是镜面就在写盘这一步被拦下、**一个文件都不写**。
逃生口 `--allow-mirrored` 只该在复现历史镜面数据（maduo 那份就是）时用。
yaml 里新增 `T_pallet_tag_det` 字段，现场拿到标定结果一眼能判断。

> 变异验证：把拒绝那一步改成恒假 → 测试立刻红（镜面照写、退出码 0）。

### 12.2 真机抓帧从来没跑通过（`Result` 被当成帧）

`hw.camera.wait_for_next_synchronized_camera_frame()` 的契约是 `-> Result`，
而 `Result` 是普通 dataclass、**没有 `__getattr__` 代理**。所以

```python
frame = hw.camera.wait_for_next_synchronized_camera_frame(cam)
if frame is None:        # ← 永不成立：bool(Result.ok(...)) 恒为 True（实测）
    ...
frame.color_image        # ← AttributeError
```

**两个工具都这么写过**（`pick_servo_inputs.py` 的 `--capture`、`pallet_calibrate.py`
的标定），而且 `main()` 都不接异常 → 真机上一按就是 traceback。`log/` 下 477 份
日志里**没有一次成功的 `--capture`**，与这个结论吻合。

修法是把解包抽成 `core/interfaces/i_camera.py` 的 `unwrap_synchronized_frame()`
纯函数 —— 放那儿是因为**只有放那儿才测得到**（工具目录不进 CI）。
两个工具各改一行。
> 变异验证：改回旧写法 → 4/5 变红，精确报出 `AttributeError: 'Result' object has no attribute 'color_image'`。

### 12.3 `depth_scale` 两端差 1000 倍

| 端 | 位置 | 语义 |
|---|---|---|
| 生产 | `camera_adapter.py:1008` `scale=0.001  # Orbbec深度图单位是毫米` | **raw → 米** |
| 消费 | `pallet_pose/algorithm.py:127-128`「把原始值换算成**毫米**」 | **raw → 毫米** |

`--dataset` 把它钉成 `1.0`（对：那份数据是毫米，实测中位数 1171），
抓帧那条读 `0.001` 当 raw→mm 用 → **托盘会小 1000 倍**，而 yaml 照样写得出来。
**没有擅自改语义**（相机到底发什么单位只有真机量一次说了算），而是加了三条：

1. **深度量级闸**：`原始中位数 × depth_scale` 必须落在 `100~10000 mm`
2. **台面尺寸闸**：算出来的 `W`/`H` 必须在 `100~3000 mm`
3. **`--depth-scale` 在 `--capture` 下能生效**（以前被**静默丢弃**）

于是这条错再也不可能静默：要么被闸门拦下并说清原因，要么操作员一个 flag 就能纠正。

### 12.4 顺带修的

- `--capture-dir`：读回 `--capture` 落下的 `color.png`/`depth.png`/`meta.json`。
  **`meta.json` 以前全仓库零读者**（只写不读），所以"先落盘保证可复现"一直是主张；
  这条让它兑现，也让"抓一帧 → 反复离线点点"成立。
- **`--capture` 与 `--stage 2` 互斥**：以前"先 `--capture --stage 1` 再
  `--capture --stage 2`"会重抓一帧覆盖 `color.png`，而 `stage1.json` 记的是同一个
  路径 → `same_frame_as_stage1()` 因**路径字面相等**而短路放行 → **拿旧位姿配新图**。
  堵死比加内容指纹零误报、零新状态。
- `order_quad()` 加了死代码指针注释（防止后人"顺手用它修顺序"）。

## 13. 真机清单（2026-09-21）

本节只做一件事：把**从机器人上电到黑板上出现三个数**该做的事按顺序列出来。
具体的接口表、参数含义、单条坑的解释**不在这里重复**，指到各自的家：

| 想知道什么 | 去哪 |
|---|---|
| 抓帧 / 点点 / 看叠图，命令怎么写 | `apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py --help` 与 `pallet_servo/README.md` §2 |
| 标定怎么做、产出什么（**只对 `base_link` 老路径**）| `apps/test_camera_internal/pallet_calibration/pallet_calibrate.py --help` |
| 配对与平滑（两路观测怎么对齐、`None` 的两种含义）| `skills/atomic/perception/pallet_frame/README.md` |
| 行为树场景的前置条件与参数 | `orchestration/scenarios/pallet_servo_real_v1/README.md` |
| 三个量的契约（符号、单位、参考边） | `pallet_servo/README.md` §1 |
| 坑的来龙去脉 | 本文档 §1~§12 |

### 13.1 两条路径，先走哪条

```
路径 A  相机抓帧 → 手点托盘四角 + 箱子四角 → 三个数
        pick_servo_inputs.py 一个工具跑完（--capture / --capture-dir）
        不需要 apriltag、不需要标定、不需要行为树、不需要箱子话题

路径 B  相机 → 视觉托盘检测器 → /pallet/detection → NodePalletObs ─┐
                                                                     │
        相机 → carton_box_yolo → box_detection → /box/detection →    ├→ NodePalletServo
                                     NodeBoxObs → latest_box_obs ────┘   → 三个数
        orchestration/scenarios/pallet_servo_real_v1
        需要：pallet_detection_msgs 编译过 + 视觉托盘检测器在跑 + 箱子那两个包在跑
        **K 不用填** —— NodePalletServo 自己读 /camera/color/camera_info
        **pallet_size_mm 必须在场景 JSON 里显式写**
```

**先跑路径 A。** 它把"相机通不通、抓帧对不对、深度单位是什么、点点算得对不对"
一次性全验掉，而这些正是路径 B 里**最难查的部分**（B 出错时你分不清是相机没起来、
视觉托盘检测器没出数、还是两个检测器的 `header.stamp` 对不上）。A 出三个数之后，
B 才有一个已知正确的参照。

两条路的终点是同一个：`/pallet_servo/dis` 上的三个数（黑板上也写一份，那是树内用的）。

### 13.2 机器人上的一次性准备

| # | 做什么 | 为什么 | 怎么确认 |
|---|---|---|---|
| 1 | 编译 `infrastructure/ros_packages` | 两道硬卡：`core/common/ros_environment.py:44` 目录不存在直接抛 `FileNotFoundError`；`perception_adapter.py:16` 是**模块级** `from apriltag_ros.msg import AprilTagDetectionArray`，**没包 try/except** | `ls <repo>/infrastructure/ros_packages/devel` |
| 2 | 装 `apriltag` catkin 包（AprilRobotics 那个 C 库的 ROS 封装） | `apriltag_ros/package.xml:29,46` 依赖它，而**仓库里没有 vendored**（`find infrastructure -name apriltag` 为空） | `rospack find apriltag` |
| 3 | 编译 `pallet_detection_msgs` | `NodePalletObs` 运行期要能 import 到消息类（**与本仓库的 `infrastructure/ros_packages` 一起编**）| `python3 -c "from pallet_detection_msgs.msg import PalletDetection"` |
| 4 | 起视觉托盘检测器 | `/pallet/detection` 的生产者，**在本仓库之外** | `rostopic hz /pallet/detection` |
| 5 | 起 `carton_box_yolo` + `box_detection` | `/box/detection` 的生产者（`box_detection_msgs` 要一起编出来）| `rostopic hz /box/detection` |

> **第 3~5 条是新链路（`pallet_frame: "camera"`）的全部新增动作** ——
> 不再需要 `config/apriltag_tags.yaml`、`launch_apriltag`、
> `config/pallet_tag.yaml`。台面尺寸改由场景 `py_tree.json` 的
> `pallet_size_mm` 提供（**必须显式写**，参数没给才退回读老标定文件，
> 而那个文件已经不存在了）。

> **（只对 `base_link` 老路径）改 `launch_apriltag` 时别顺手改 `driver_only`。**
> `camera_config.yaml:21` 那个 `driver_only: true` 是**死配置** ——
> `lifecycle_mixin.py:89` 无条件用
> `self.config.get('camera_driver_only', False)` 覆盖它，而全仓库没有任何地方
> 设过 `camera_driver_only`，恒为 `False`。读 YAML 会得出"还要改 driver_only"
> 的结论，那是错的。

### 13.3 每次上机的顺序

```bash
# 0) 机器人站立、bringup 起来（base_link 等 TF 由它发布）
# 1) roscore（或紧随机器人 bringup）
rostopic list            # 有回应才算通

# 2) 路径 A：抓一帧
cd <repo>
python3 apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py \
    --capture --stage 1 --out-dir /tmp/servo_sim

#    抓不到时先看两路图像在不在发，再调大超时（默认 5 秒）
rostopic hz /camera/color/image_raw /camera/depth/image_raw
python3 ... --capture --capture-timeout 20 --out-dir /tmp/servo_sim

# 3) 路径 A：点点（弹窗，需要 DISPLAY + tkinter）
python3 ... --capture-dir /tmp/servo_sim --stage 1 --out-dir /tmp/servo_sim
python3 ... --capture-dir /tmp/servo_sim --stage 2 --box-mode quad --out-dir /tmp/servo_sim

# 4) 路径 B：先干跑，再真跑
bash orchestration/scenarios/pallet_servo_real_v1/start_behavior_tree.sh --dry-run --tick-once
bash orchestration/scenarios/pallet_servo_real_v1/start_behavior_tree.sh
```

相机**不用手动 roslaunch**：`CameraAdapter.initialize()` 自己会起
`config/camera_orbbec.launch`（`camera_adapter.py:206-241`）。

**⚠️ 但也正因为它无条件起相机**：如果机器人自己的 bringup 已经起过 Orbbec，
这里就是第二次打开同一个 USB 设备。适配器起完 3 秒会检查 `roslaunch` 还活着没有，
死了就报 `Orbbec 相机未连接 / USB 权限不足 / orbbec_camera 包未编译`
（`camera_adapter.py:251-253`）—— 真相可能是"**相机已经被别人起着了**"。
上机前先 `rostopic list | grep camera` 看一眼。

### 13.4 只能现场量、读代码读不出来的三件事

> 第 1、3 条是 **`base_link` 老路径（AprilTag）** 的；新的视觉链路
> （`pallet_frame: "camera"`）**不查 TF、也不读 tag 尺寸**，只要第 2 条。

1. **（老路径）实物 tag 的黑框外沿尺寸。** `apriltag_ros` 拿它反算 tag 到相机的
   距离，写错一倍距离就差一倍；而 `T_pallet_tag` 照样是个合法刚体变换、照样写得
   出来、下游一路上没有任何报错。**量的时候是黑框外沿，不是整张纸含白边。**
   （记录里有 180 mm 与 96 mm 两个数，分属不同批标签 —— 以现场卡尺为准。）

2. **`depth_scale` 到底是 raw→毫米 还是 raw→米。** 生产端
   `camera_adapter.py:1008` 给 `0.001`（注释说"Orbbec 深度图单位是毫米"，即
   raw→**米**），消费端 `pallet_pose/algorithm.py:127` 要的是 raw→**毫米**，
   差 1000 倍。见 §12.3。抓帧时 `meta.json` 会如实记下相机报的值与深度中位数，
   **拿它俩对一次量级就能定案**：中位数 × scale 应落在 100~10000 mm。

3. **（老路径）`T_cam_base` 查不查得到。** 老路径的托盘位姿是 base_link 系的，
   伺服要把它投影回图像就必须有 `T_cam_base`。节点按
   **TF → `T_cam_base_param` → 单位阵** 逐级回退，**回退到单位阵是静默的**
   （只有一条 WARNING），而 base 系的位姿被当成相机系直接投影，三个量看着像模像样。
   上机后第一件事：

   ```bash
   rosrun tf tf_echo base_link camera_color_optical_frame
   ```

   查不到就说明那条链断了：驱动只发布
   `camera_link → camera_color_optical_frame`，而 `base_link → ... → camera_base`
   要靠机器人自己的 URDF / 静态 TF（`dynamic_biped/launch/orbbec_sensor_robot_enable.launch`
   里那几条 `static_transform_publisher`）。跑起来后日志里的 `t_cam_base_src`
   必须显示 `tf`。**新链路（`"camera"`）走的是声明过的单位阵、不查 TF。**

### 13.5 箱子那半段（**已接上**）

原来说"本次不接"，现在 `pallet_servo_real_v1/py_tree.json` 里已经是 `NodeBoxObs`
（订阅 `/box/detection`），`NodeInjectServoInput` 那一段已经不在这条链路上了。
`NodeBoxObs` 是 `/box/detection` 的**消费者**；生产者已经进 LeTools 了：

| 环节 | 在哪 | 状态 |
|---|---|---|
| `box_detection` 发 `/box/detection` | `infrastructure/.../detection_industrial_yolo/box_detection/`（算法在 `skills/atomic/perception/box_frame/`） | 已迁移，实测 `valid=true source=window` |
| 它要的输入 `/box/yolo_box`（`PolygonStamped`） | `infrastructure/.../detection_industrial_yolo/carton_box_yolo/` | 已补齐（此前全仓没有真生产者） |
| 消息包 `box_detection_msgs` | `infrastructure/.../ros_vision/box_detection_msgs/` | 要拷进机器人的工作区 `catkin_make` |

起节点前 `export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1`（实测差 1.6×）。
走场景入口（`start_behavior_tree.sh`）时不用自己 export —— 脚本自己设了
（2026-09-30 起；三个检测器那边由各自 launch 的 `<env>` 设）。上面这条说的是
**手敲 `rosrun` / 离线脚本**那种。

深度话题默认值已改成 Orbbec 的 `/camera/depth/image_raw`（原来是 RealSense 的
`aligned_depth_to_color/image_raw`，`depth_registration:=true` 只改点云话题名），
换相机时用 `~image_depth` 参数改。

细节见 `skills/atomic/perception/box_frame/README.md` 与场景 `README.md` 的前置条件表。


### 13.6 已知的、跑起来才看得出的静默错

| 静默错 | 现在的拦截 |
|---|---|
| 抓帧把 `Result` 当帧用 | 已修，且有墓碑测试（`test_camera_frame_contract.py` 里那条断言旧写法必然 `AttributeError`）|
| 标定建出镜面托盘系 | 已修，写盘前拒绝（`--allow-mirrored` 可强行放行） |
| `K` 是占位符（全 0） | **启动即配置错误**。全 0 的 K 形状合法，会让每个投影点塌到主点上而三个量看着正常。**现在内参默认从 `/camera/color/camera_info` 实时读**，场景里不用写 K，这条只在"相机起不来、退回写死值"时才会碰到 |
| 写死的 K 过期 | 内参**话题优先**：换了分辨率/机器人/相机型号，用相机的、不用场景里写死的。用的是哪个写在日志的 `内参来源=` |
| 托盘位姿是镜面 | `pallet_calibrate.py` 出的 yaml 里带 `T_pallet_tag_det` 字段，现场一眼看得出来（老路径）。新链路由 `NodePalletObs.require_handedness` 在入口拒收 |
| **配对按"处理完成时刻"做** | 伺服会频繁报 `pair_dt`（带实际差值）—— **这是看得见的**，不是静默的 |
| 台面尺寸 `pallet_size_mm` 写错 | **静默**：值错了不报错，只是参考边投到错的位置，而三个数照样算得出来。**当前值 `[1200, 1000]` 操作员已确认**（2026-09-24）；⚠️ 摆托盘时**短边（1000）与机器人平行**，摆反 90° 不报错 |
| 正常帧什么都不打印 | **未修**：三个数是 `logger.debug`，而日志默认 INFO —— 跑之前把级别改 DEBUG，`start_behavior_tree.sh` 每次会提醒 |
