# pallet_servo_real_v1 —— 托盘伺服误差（真机）

把真机上的一整条链路接起来，跑完在黑板上留下**三个数**：

```
相机 ──► 视觉托盘检测器 ──► /pallet/detection ──► NodePalletObs ──┐
        （在本仓库之外）                            latest_pallet    │
                                                      +_stamp        ├─► NodePalletServo
相机 ──► carton_box_yolo ──► box_detection ──► /box/detection ──────┤   latest_servo_error
                                                NodeBoxObs           │   （e_bottom_px /
                                                latest_box_obs ──────┘    e_right_px / theta_rad）
```

**托盘与箱子都是视觉检测器，全自动。** 两路观测按**输入图像的采集时刻**
（`header.stamp`）配对（容差 `max_dt_s`，默认 0.05s），再在成对序列上做窗平均 ——
两个检测器的耗时差因此不是问题，它们看的是**同一帧图像**。

**这个场景不会动机器人** —— 它只把三个数**算出来并发出去**，谁拿它去动机器人是
另一件事。三个数实时发在 **`/pallet_servo/dis`**（`pallet_servo_msgs/PalletServoError`），
行为树之外的控制器订阅它就能做伺服：

```bash
rostopic echo /pallet_servo/dis
```

⚠️ **话题上"一条消息都没有"才是异常**（roscore 挂了 / 话题名写错 / 节点没跑）。
本节点保证**任何时刻都有一条不超过 1 秒的消息**：算不出来的时候也发
（`valid=false` + `reject` 说明原因，三个量是 `NaN`）。消费方**必须先判 `valid`**。

黑板上的 `latest_servo_error` 仍在写，但那是**树内**用的实时值 —— 树外的进程看不见
黑板，**对外接口是话题**（见 `skills/atomic/perception/pallet_servo/README.md` §1）。

## 前置条件

按顺序，缺一样都跑不到三个数：

| # | 条件 | 怎么确认 |
|---|------|---------|
| 1 | `pallet_detection_msgs` 编译过 | `python3 -c "from pallet_detection_msgs.msg import PalletDetection"` |
| 2 | **视觉托盘检测器**在跑、在发 `/pallet/detection` | `rostopic hz /pallet/detection` |
| 3 | `carton_box_yolo` + `box_detection` 两个包在跑、在发 `/box/detection` | `rostopic hz /box/detection` |
| 4 | `infrastructure/ros_packages` **编译过** | `ls <repo>/infrastructure/ros_packages/devel` |
| 5 | `pallet_servo_msgs` 编译过（**发话题 + 箱位服务都要它**）| `python3 -c "from pallet_servo_msgs.msg import PalletServoError"` |

第 5 条缺了**行为不一样，要分清**：

- 缺的是**发布器**（话题起不来）→ 节点记一条 WARNING 说"只写黑板，控制器拿不到误差"，
  照常算、照常出叠加图，**只是话题上没有消息**。
- 缺的是**箱位服务**（`/pallet_servo/slot` 起不来）→ 节点**永远停在未激活**，
  话题上全是 `valid=false` 的「伺服未启动」，**一帧有效数都不出**。日志会说清这一点。

## 参数改哪里

**全部在 `board.json`**（本目录），24 个键分三组 `pallet` / `box` / `servo`，
每条带 `remark` 写清什么时候该改它。**改完重启行为树生效。**

```bash
grep -n '"key"' board.json     # 看有哪些键
```

两条路：

| 谁读 | 怎么读 |
|---|---|
| 行为树内三个节点 | `py_tree.json` 里写 `{"source": "READ_BOARD"}`，节点经黑板取值 |
| 三个检测器（独立进程，看不见黑板）| `start_all.sh` 从板子读出来，当 `roslaunch` 命令行 arg 传下去 |

⚠️ **`py_tree.json` 里那些 `READ_BOARD` 是"值从板子来"的标记 —— 不要把值写回去。**
写回去等于板子上那份失效，而两处不一致时**没有报错**（节点用 JSON 里那个）。

⚠️ **换托盘时 `pallet_size_mm` 与 `ref_edges` 要一起改** —— `ref_edges` 写的是绝对
毫米，边界的边就落在尺寸上。

### 台面尺寸 `pallet_size_mm` 必须显式写

托盘台面尺寸 `[W, H]`（毫米）**只能来自 `board.json` 的 `pallet_size_mm`**，
**不写就是启动即 FAILURE**（`NodePalletObs` 与 `NodePalletServo` 都读它）。

⚠️ **这个值错了不会报错**，只会让两条参考边投到错的位置 —— 而三个数照样算得出来。
当前写的是 `[1200.0, 1000.0]`（毫米），**操作员已确认**。

### 坐标系与参考边

**调参考边之前先看 `skills/atomic/perception/pallet_servo/README.md` 里那张台面
坐标系图**（原点在画面左下角、`+x` 朝右、`+y` 朝上，四条边各是哪条）。

⚠️ **轴向是相对相机定的，不是相对托盘物理定的** —— **换工位后必须在叠加图上
重新确认一次**，别照抄上次的 `ref_edges`。

⚠️ **摆放约定（2026-09-24 操作员确认）**：台面 `[1200, 1000]` mm，**1200 是长边、1000 是短边**。
摆放时 **短边（1000）与机器人平行**，即**长边（1200）垂直于机器人正面**；
`origin` 取台面的**左下角**（`e1` 沿长边、`e2` 沿短边，`e1 × e2 = normal` 指向相机）。

⚠️ **托盘摆反 90° 不会报错** —— 参考边会取到**别的物理边上**，而三个数照样算得出来，
伺服照着错的参考边把箱子送过去。现场唯一能看出不对的是**人眼**在叠加图上
（`/pallet_servo/overlay`）看青框贴不贴托盘 —— 图上黄线的标签就是你写的那两条原文。

### `K` 不用填 —— 节点自己读

`NodePalletServo` 的内参（`K` / `D` / `image_size`）**默认从
`/camera/color/camera_info` 实时读**，场景 JSON 里一个字都不用写。

**规则只有一条：场景里没写 `K` → 读话题；写了 `K` → 一个话题都不碰，就用写的那个。**
写死值**不是兜底**，是**钉死**。这样定的两个理由：

- 写死的值会过期——换分辨率、换台机器人、换个相机型号，JSON 里的 `K` 不会跟着变，
  而那是**静默**的错（三个量照算，只是全偏）。所以**真机上别写 `K`**。
- 反过来，写 `K` 的人是在说"就用这个"。离线复现、单测、相机不在场时**写 `K`**，
  这样结果不取决于跑在哪台机器上。

所以本场景的 `py_tree.json` 里**没有 `K` 这一项**。要离线复现、或者相机起不来时
想钉死内参，再往 `NodePalletServo` 的 params 里加 `K`（嵌套 3×3）即可。

日志里的 `内参来源=` 会写明这一版用的到底是哪一个，**现场第一件要核对的就是它**：
真机上应当是 `/camera/color/camera_info`；显示 `场景参数（场景里写了 K，按显式钉死
处理，没去问相机）` 说明场景里多写了一个 `K`，那是**故意的**还是**抄错了**要自己认；
显示 `场景参数（读不到 /camera/color/camera_info（...））` 才是相机没起来。

## 接口

| 节点 | 读 | 写 |
|------|-----|-----|
| `NodePalletObs` | —（订阅 `/pallet/detection`）| `latest_pallet` + `_version` + `_stamp` |
| `NodeBoxObs` | —（订阅 `/box/detection`）| `latest_box_obs` + `_version`（观测自带 `stamp`）|
| `NodePalletServo` | 上面三个键 + 内参 | `latest_servo_error` + `_version`；**话题** `/pallet_servo/dis` |

对外还有两个接口：

| 接口 | 类型 | 谁用 |
|---|---|---|
| `/pallet_servo/dis` | `pallet_servo_msgs/PalletServoError` | **下游控制器**订阅三个数 |
| `/pallet_servo/slot` | `pallet_servo_msgs/SetServoSlot` | **上游**告诉节点现在处理第几个箱位 |

`NodePalletServo` 的输出字段与全部参数见 `skills/atomic/perception/pallet_servo/README.md`
§1；配对与平滑的细节见 `skills/atomic/perception/pallet_frame/README.md`。

## 怎么跑

**一条命令起停全栈**（三个检测器 + 行为树）：

```bash
cd <repo>/orchestration/scenarios/pallet_servo_real_v1

# 先干跑一次（不碰硬件、不弹窗）
bash start_behavior_tree.sh --dry-run --tick-once

# 正式跑：需要 roscore + 相机已起（脚本不含这两个）
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.x.x          # 本机 IP
bash start_all.sh
```

`start_all.sh` 做四件事：检查 `roscore` → **把上一轮残留的检测器杀掉重启**
（它们的参数是上一版 `board.json` 的，沿用等于"改了板子却不生效"）→ 从 `board.json`
读值、起三个检测器并等三路话题出数 → 前台跑行为树。**Ctrl+C 会连三个检测器一起收。**

⚠️ **线程数不用你 export。** 三个检测器由各自的 launch 用 `<env>` 设
（`roslaunch` 的 `<env>` 是**覆盖**继承来的环境）；行为树进程是 `python3` 直接起的、
前面没有 `roslaunch`，由 `start_behavior_tree.sh` 自己设 —— 这两处 2026-09-30 起
都齐了。**你在终端里手敲的离线脚本 / `rosrun` 单跑还要自己 export**，
理由与实测见 `docs/托盘伺服部署与跑通.md` §0.1b。

**只想跑行为树**（检测器已经在别处起好了）仍可以：

```bash
bash start_behavior_tree.sh
```

三个检测器也可以单独起（`roslaunch` 用各自的默认值，**不读 `board.json`**）：

```bash
roslaunch pallet_detection pallet_detection.launch
roslaunch carton_box_yolo carton_box_yolo.launch
roslaunch box_detection box_detection.launch
```

### ⚠️ 节点启动时是「未激活」的 —— 要 call 一次才出数

`NodePalletServo` 在接到箱位之前**一帧有效数据都不出**（话题上是 `valid=false` 的
「伺服未启动」）。开工时 call 一次：

```bash
rosservice call /pallet_servo/slot "slot: 1"   # 第 1 个箱子（用 ref_edges 第 1 组）
```

**`slot` 是箱位号，从 1 数**；`slot: 0` 表示本次伺服结束。板子里 `ref_edges`
有几组，就能 call 到几。**越界调用返回 `ok=false` 且不改当前状态。**

### ⚠️ 跑之前先把日志级别调成 DEBUG

**正常情况下那三个数是 `logger.debug`**，而 `config/log_config.yaml` 默认是 `INFO`
—— 照默认跑，**一个数都看不到**，只剩"没有任何 ERROR"这一条线索，分不清
"在正常工作"和"压根没收到输入"（`等输入` 那句同样是 DEBUG）。

```bash
# config/log_config.yaml 里把 level 改成 "DEBUG"
grep -n 'level:' config/log_config.yaml
```

`start_behavior_tree.sh` 每次跑都会替你检查这一条并提醒。

`WaitSeconds` 到点整棵树就收（`run_for` 的 `duration_sec`，**2026-09-29 起默认
86400 秒 = 24 小时** —— 原来那 30 秒会让整条链在开工半分钟后就自己退出，而
`start_all.sh` 跟着收摊，现场看到的是"跑一会就没了"）。届时看日志：

```bash
grep '托盘伺服第' log/*.log | tail -30
```

⚠️ **这是"跑够久"，不是"call slot 0 就收工"。** `rosservice call /pallet_servo/slot
"slot: 0"` 只把节点置回未激活（`_active_slot = 0`），**树不会结束** —— 想真正
"call 0 就退出"需要一个"等停止条件"的节点，那是新功能，现在没有。要提前收工
用 Ctrl+C。

> 这个场景**不**取代 `pallet_servo/README.md` §2 的那条路。抓帧、点点、看叠图
> 仍然在那边做（路径 A）；本场景只负责"两路都换成实时的"这一步。

## 注意事项

1. **托盘与箱子必须来自同一帧图像。** 伺服按两个观测的 `header.stamp`
   （**输入图像的采集时刻**）配对，容差 `max_dt_s` 默认 0.05s。**配不上就不出数**，
   日志里写 `pair_dt`（带实际差值）或 `pair_nan` / `pair_stamp_zero` / `stale` ——
   那不是坏了，是这一帧的两个检测结果不该相减。托盘检测慢的时候输出率被它卡住，
   这是刻意的：宁可少出数，不出错数。

   ⚠️ **"检测器挂了"不长成上面任何一种 reject，而是长成"没有新数"** ——
   `resolve()` 返回 `None` 时节点静默 `RUNNING`、黑板停在旧值。节点层的
   `live_timeout_s`（默认 5.0s）负责把这种停摆报出来，**它是存活闸**；
   `stale_s` 是延迟闸，两者别混。

2. **`pallet_frame` 必须是 `"camera"`（本场景的默认，也已显式写在 JSON 里）。**
   本场景的托盘位姿是**相机系**的，节点**不查 TF**，`T_cam_base` 是声明过的单位阵。
   TF 那条链只在 **`base_link` 模式（AprilTag 老路径）** 下才用得上 ——
   那条路还**绕过配对**（老路径的生产者不写 stamp）。配错会让参考边整体偏掉，
   而三个数照样算得出来。

3. **`NodePalletObs.require_handedness: true` 是本场景的闸门。** 上游检测器给出
   镜像的 `T_cam_pallet` 时会**拒收**（镜面在下游是静默的）。`require_valid: true`
   同理：`valid == false` 的帧一个字节都不写黑板。

4. **这个场景是"源节点"树，不能塞进 `Sequence`，也不能塞进必须全部成功的
   `Parallel(SuccessOnAll)`。** 里头的 `NodePalletObs` / `NodeBoxObs` /
   `NodePalletServo` 在没有新数据时都返回 **RUNNING**（源节点口径：自己决定写什么，
   何时收由父节点定）。所以根是 `Parallel(success_on_one)` + 一个 `WaitSeconds`
   来收场 —— 改结构前先读一遍 `pallet_servo/README.md` §3 的理由。

5. **改 `board.json` 或 `py_tree.json` 之后先干跑一次。** JSON 里写错一个字段名
   不会报错，只会让那个参数取默认值；`--dry-run --tick-once` 能把"节点根本没被
   解析到"和"参数没生效"两类问题挡在下真机之前。板子上**有键但 tree 没引用**
   （死配置）与**tree 引用了但板上没有**（回退代码默认值）各有一条 WARNING，
   干跑时都会打出来。

6. 本场景的 `start_behavior_tree.sh` 用**脚本位置**推仓库根，不写死部署路径；
   仓库里另外几份里出现的 `/media/data/LeTools` 是那台机器上的约定，不影响这份。
