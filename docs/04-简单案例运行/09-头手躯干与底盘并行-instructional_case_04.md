# 09 头手躯干与底盘并行-instructional_case_04

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

本文中的相对路径一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

**难度**：⭐⭐⭐

**适合读者**：了解 [08-头手躯干串行](08-头手躯干串行-instructional_case_03.md)，想了解多 planner 同请求与 JiBot 底盘并行编排的人。本案例不是当前 case03 简单加一个底盘节点。

**前置知识**：`Parallel` 的执行顺序（见 [07](07-手臂与头部并行-instructional_case_02.md)）、`MultiTimedCommandMove`（见 [08](08-头手躯干串行-instructional_case_03.md)）。

**核心概念**：用一个持续 `RUNNING` 的守护节点管理底盘控制权；`success_on_one` 让主序列结束时自动终止守护节点；区分 JiBot 相对导航与 TimedCmd 底盘 planner。

**学习目标**：

1. 看懂「控制权守护 + 动作序列」的外层结构；
2. 理解底盘节点为什么要排在 `MultiTimedCommandMove` 前面；
3. 知道「只提交不等待」的底盘节点不验收到达，时间估算不能代替真实任务状态。

**由简到难进阶路线**：[08](08-头手躯干串行-instructional_case_03.md)（头部 + 双臂 + 躯干）→ **本篇**（加入底盘）→ [10](10-搬箱场景-grasp_mtbf.md)（导航、识别、抓取、放置的完整链路）。

---

## 一、场景定位

| 维度 | 值 |
|------|----|
| 场景目录 | `orchestration/scenarios/instructional_case_04/` |
| 主树 | `py_tree.json`，根 `Parallel(jibot_control_guard_and_motion)`，共 **18 个节点** |
| 子树 | `py_tree_child.json` 为空集合，本案例不使用子树 |
| 黑板 | `board.json`：独立的头部角度、planner `3/8/9` 命令组和 JiBot 参数；不沿用 case03 的 planner `2` 位姿格式 |
| 硬件配置 | 无 `hardware_config.json`；JiBot 由 `config/app_config.yaml` 中的 `navigation_chassis_backend: jibot` 选择 |
| 运动部位 | 头部、双臂、下肢 / 躯干、JiBot 底盘 |
| 运行环境 | 干跑、**真机（JiBot 底盘）**。MuJoCo 仿真中没有 JiBot 导航服务，底盘节点无法执行 |

---

## 二、树结构

```text
Parallel(jibot_control_guard_and_motion)  policy=success_on_one
├── NodeEnableVelControl  give_control_to_jibot_navigation   把底盘控制权交给 JiBot 导航，结束时恢复
└── Sequence(four_head_arm_torso_chassis_phases)  memory=true
    ├── Parallel(phase_1_home)         头回正 + 双臂 / 躯干归零
    ├── Parallel(phase_2_turn_left)    头左转 + JiBot 相对左转 + 双臂 / 躯干左向姿态
    ├── Parallel(phase_3_turn_right)   头右转 + JiBot 相对右转 + 双臂 / 躯干右向姿态
    └── Parallel(phase_4_return_home)  头回正 + JiBot 相对回正 + 双臂 / 躯干回零
```

### 外层：控制权守护

- `NodeEnableVelControl(enable=false)` 初始化时尝试交权给 JiBot 导航，正常切换后保持 `RUNNING`；硬件无该方法时也会跳过并 `RUNNING`，并非交权成功的证明。
- 外层 `success_on_one` 在四阶段序列成功时终止守护；`restore_on_stop=true` 只表示尝试恢复。未读到原权限且未配置 `restore_to` 时不恢复；恢复失败只告警，不改变已成功的根状态。
- 切换接口返回失败时守护节点及根树会失败。但 `Parallel` 会先 tick 各分支，失败的同拍仍可能执行动作分支；它不是防止其他分支发令的安全门。
- 必须人工核对运行前后的权限与在途任务；节点实际返回 `FAILURE` 后不进入后续阶段，不等于所有硬件异常都能可靠上报，也不保证已提交运动立即停止。

### 阶段内：子节点顺序

有底盘动作的阶段按以下顺序排列：

```text
HeadControlSdkMove           下发头部目标后立即返回
BaseMoveRelativeJibotMove    调用 /move_base/base_move 提交底盘任务后立即返回
MultiTimedCommandMove        提交手臂与躯干目标，再按规划时长定时等待
```

`Parallel` 依次 tick 子节点，物理运动可以重叠，但头部不查到位、底盘不查任务完成、TimedCmd 技能只按 `actual_time + settle_time` 等待，不能据此证明同步到位。如果把底盘节点放在 `MultiTimedCommandMove` 后面，只会延后底盘任务提交，并不会增加到达检查。

---

## 三、JiBot 相对运动

`BaseMoveRelativeJibotMove` 底层调用 `hardware.base_move_relative_jibot()`：`x`、`y` 是相对当前本体坐标系的位移（米），`theta` 是相对当前朝向的旋转量（**弧度**，不受 `angle_unit` 影响）。

| 阶段 | 相对 `theta`（rad） | 约合角度 | 理想累计朝向变化 |
|---|---:|---:|---:|
| 1：初始归零 | 不下发底盘任务 | 0° | 0° |
| 2：向左 | `+0.349066` | +20° | +20° |
| 3：向右 | `-0.698132` | -40° | -20° |
| 4：回正 | `+0.349066` | +20° | 0° |

三次增量之和为零只是命令层的理想结果；本树不验证每段到达，不能保证实际回到起始朝向。本案例只请求原地旋转（`x = y = 0`）。

底盘参数（`board.json`）：

| 黑板键 | 默认值 | 说明 |
|---|---:|---|
| `base_relative_x_m` / `base_relative_y_m` | `0.0` | 相对位移（米） |
| `base_turn_left_rad` / `base_turn_right_rad` / `base_return_center_rad` | `0.349066` / `-0.698132` / `0.349066` | 三个阶段的相对旋转量 |
| `base_angular_velocity_rad_s` | `0.35` | 角速度（rad/s） |
| `base_angle_threshold_rad` | `0.05` | 角度到达阈值（rad） |

### 时间窗口的限制

`BaseMoveRelativeJibotMove` **只提交任务，不查询是否到达**。最大一段按 `0.698132 / 0.35` 粗算约 2 秒，未含加减速和导航/调度开销，不是完成时间上限。同阶段 TimedCmd 的名义 3 秒加默认 0.5 秒额外等待也不是底盘到达条件；即使默认参数也不能排除下一段提交时上一段尚在途，或收尾恢复权限时底盘仍在运动。

需要自动确认每段完成时，应另行编排保存提交结果的真实 `task_id`，再查询到达及超时。当前 `BaseMoveRelativeJibotMove` 不把 `task_id` 写入黑板，因此不能只加一个未绑定真实任务 ID 的 `CheckArrivedJibotMove` 就认为形成了闭环。相关参数见 [节点清单](../03-接口详细文档/11-节点清单.md)，本场景尚未实现该闭环。

### 不要和 TimedCmd 底盘 planner 混淆

| 接口 | 是否使用 planner 编号 | 运动含义 | 角度单位 |
|---|---|---|---|
| `BaseMoveRelativeJibotMove` | 否 | JiBot 相对当前位姿运动 | 弧度 |
| `MultiTimedCommandMove` 的 planner `0` | 是 | TimedCmd 世界坐标系底盘目标 | 随 `angle_unit`，默认度 |
| `MultiTimedCommandMove` 的 planner `1` | 是 | TimedCmd 本体坐标系底盘目标 | 随 `angle_unit`，默认度 |

本案例的底盘动作走 JiBot 接口，没有使用 planner `0` / `1`。

---

## 四、头、手和躯干动作

本案例的 `MultiTimedCommandMove` 一次提交 planner `3`（4D 下肢/躯干关节角）、`8`（7D 左臂）、`9`（7D 右臂），与 case03 的 planner `2` 躯干位姿后接独立双臂节点不同。三组 board 键依次为 `home_commands`、`forward_and_left_commands`、`forward_and_right_commands`，阶段 4 复用 `home_commands`。

`motion_duration_sec=3.0` 是期望规划时长，默认 `is_sync=true` 请求同组 planner 同步规划，并非实测同时到位保证；技能按返回 `actual_time + 0.5` 秒等待，无返回时回退到请求时长。默认 `prepare_arm_control=true` 会先切手臂模式 2，再设 `focus_ee=false` / `focus_z=false`，任一步失败则不下发该节点的 TimedCmd，但已产生的副作用不回滚。结束不会恢复手臂模式/原焦点或物理复位。以下角度按默认 Adapter `angle_unit=deg` 填写，不能与 JiBot 的弧度混用。

| 阶段 | 头部 `[yaw, pitch]`（度） | 左 / 右臂 `[J0..J6]`（度） | 下肢 / 躯干 `[j0..j3]`（度） |
|---|---|---|---|
| 1 | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |
| 2 | `[20, 0]` | `[0, 0, 0, -90, 0, 30, 0]` | `[29.00, -41.11, 32.21, 20]` |
| 3 | `[-20, 0]` | `[0, 0, 0, -90, 0, -30, 0]` | `[29.00, -41.11, 32.21, -20]` |
| 4 | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |

---

## 五、运行

### 5.1 干跑

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_04 \
  --dry-run --tick-once
```

预期输出（关键行）：

```text
[apps][dry-run] 已加载树，根节点: jibot_control_guard_and_motion
[apps][dry-run] tick 后根状态: Status.SUCCESS
```

dry-run 只覆盖加载及无硬件分支；当前根失败时也可能退出 0，应看根状态而不是只看退出码。它不能验证到达、权限、可达性或碰撞。

### 5.2 真机

1. 确认 `config/app_config.yaml` 中 `navigation_chassis_backend` 为 `jibot`，JiBot `/move_base/base_move`、控制权读取/切换、TimedCmd 及模式/焦点服务可用；排除其他在途任务，并现场确认当前控制权限，不能只以服务名出现在列表中判定就绪；
2. 下位机启动真机控制器，宿主机按 [02-执行命令 · 运行前环境准备](02-执行命令.md#运行前环境准备仿真与真机共用) 准备终端，并确认 TF 转发已启动；
3. 运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_04
```

> ⚠️ **真机**：运行前按 [02-执行命令 · 真机运行前安全检查](02-执行命令.md#真机运行前安全检查) 逐项确认。本案例底盘会原地左右旋转，同时头部、双臂和躯干运动；`BaseMoveRelativeJibotMove` 的避障默认关闭，确认机器人四周以及头部、双臂和躯干的完整运动范围内没有人员、线缆或障碍物。

---

## 六、验证方式

| 层次 | 判据 |
|------|------|
| 加载 | 干跑打印 `根节点: jibot_control_guard_and_motion` |
| 单拍 tick | `tick 后根状态: Status.SUCCESS` |
| 程序结果 | 三次 JiBot 提交日志与退出码 0 只说明节点路径成功，不能证明底盘到达或回正 |
| 真机验收 | 另行记录每个任务的实际到达、头/臂/躯干反馈及最终朝向，并核对底盘权限、手臂模式与焦点；恢复告警或在途任务不能因根成功而忽略 |
| 排查 | 只看到 TimedCmd 日志、看不到 `/move_base/base_move` 时，先确认实际加载的场景路径是 `instructional_case_04` |

---

## 七、动手改一改

| 改什么 | 怎么改 | 注意 |
|---|---|---|
| 转得更慢 | `base_angular_velocity_rad_s` 改为 `0.25` | 40° 按匀速粗算约 2.8 秒，不是到达时限；必须另验任务状态 |
| 转得更大 | 三个 `theta` 同比例放大，保持总和为零 | 和为零也不保证回正；重新核对场地、时序与到达证据（见第三节） |
| 加一点平移 | `base_relative_x_m` 改为非零 | 阶段 2–4 各提交一次平移，阶段 1 无底盘任务；相对坐标随朝向变化，务必核对累计路径 |

---

## 八、下一步

- [case05：01→04 顺序联测](../../orchestration/scenarios/instructional_case_05/README.md)：复用现有动作及返回边界，相邻案例间用公共 `WaitSeconds` 等待 2 秒，不额外增加到达检查或模式恢复。
- [10-搬箱场景-grasp_mtbf](10-搬箱场景-grasp_mtbf.md)：导航、识别、抓取、搬运、放置的完整链路
- [`03-adapters-硬件适配层`](../03-接口详细文档/03-adapters-硬件适配层.md)：JiBot 接口与标准导航接口
